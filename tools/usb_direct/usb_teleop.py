#!/usr/bin/env python3
"""Веб-телеоп GIGAROVER через один USB-VESC (CAN-forward на остальные).

Запуск:  python usb_teleop.py [--port COM18] [--max-erpm 4000] [--ramp 3000]
Открыть: http://127.0.0.1:8770

Страница: джойстик + мастер настройки (как в vesc_config_app): «Сканировать
шину» → «Крутить» мотор → кликнуть позицию колеса на схеме → указать, куда оно
едет. Раскладка сохраняется в usb_motors.json рядом со скриптом.

Логика привода: TX 50 Гц, deadman 0.5 c, рамп скорости, подпор min_erpm.

ROS 2: поднимает UDP-API на 127.0.0.1:8460 с протоколом rover-motord, поэтому
штатный vesc_bridge_node (rover-bringup, base_driver.type: vesc) шлёт /cmd_vel
сюда без каких-либо правок. Арбитраж: веб-джойстик перехватывает у ros,
у каждого источника свой deadman. Телеметрии (энкодеры/батарея) в USB-режиме
нет — мост публикует /wheel/encoders c valid=false (одометрия стоит).
"""
import argparse
import glob
import json
import os
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import socket

import serial

COMM_FW_VERSION = 0
COMM_SET_CURRENT = 6
COMM_SET_RPM = 8
COMM_FORWARD_CAN = 34
COMM_PING_CAN = 62

LOCAL = "local"  # ключ VESC, висящего на USB
CFG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "usb_motors.json")
POSITIONS = ("FL", "FR", "RL", "RR")
SIDE = {"FL": -1, "RL": -1, "FR": +1, "RR": +1}

MIN_ERPM = 900
IDENT_ERPM = 1100      # обороты при опознании мотора
DEADMAN_S = 0.5
LOOP_HZ = 50
RAMP_ERPM_S = 3000     # максимальный темп изменения скорости, eRPM/с

# UDP-API, совместимый по протоколу с rover-motord (vesc_bridge_node ходит сюда):
#   {"v":1,"src":"ros","cmd":"drive","vx":м/с,"wz":рад/с} | stop | estop | sub | ping
# Арбитраж источников: web (джойстик) перехватывает у ros; deadman у каждого свой.
UDP_HOST = "127.0.0.1"
UDP_PORT = 8460
SRC_PRIORITY = {"web": 2, "ros": 1}
STATE_HZ = 20
SUB_TTL_S = 3.0

# геометрия GIGAROVER (как в gigarover_v1.yaml): vx/wz -> eRPM на борт
WHEEL_RADIUS_M = 0.0825
TRACK_WIDTH_M = 0.40
GEAR_RATIO = 3.0
POLE_PAIRS = 7
ERPM_PER_MPS = 60.0 * POLE_PAIRS * GEAR_RATIO / (2.0 * 3.141592653589793 * WHEEL_RADIUS_M)


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def frame(payload: bytes) -> bytes:
    return bytes([2, len(payload)]) + payload + struct.pack(">H", crc16(payload)) + b"\x03"


def read_packet(ser: serial.Serial, timeout: float = 1.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        b = ser.read(1)
        if not b or b[0] != 2:
            continue
        ln = ser.read(1)
        if not ln:
            continue
        n = ln[0]
        rest = ser.read(n + 3)
        if len(rest) == n + 3 and rest[-1] == 3:
            payload = rest[:n]
            if crc16(payload) == struct.unpack(">H", rest[n:n + 2])[0]:
                return payload
    return None


def resolve_port(port: str) -> str:
    if port != "auto":
        return port
    if sys.platform == "win32":
        return "COM18"
    for pat in ("/dev/serial/by-id/*ChibiOS*", "/dev/serial/by-id/*VESC*", "/dev/ttyACM*"):
        found = sorted(glob.glob(pat))
        if found:
            return found[0]
    return ""


def open_port_wait(port: str) -> serial.Serial:
    """Ждём появления VESC на USB (удобно под systemd: воткнул провод — поехали)."""
    warned = False
    while True:
        p = resolve_port(port)
        if p:
            try:
                ser = serial.Serial(p, 115200, timeout=0.1)
                print(f"открыл {p}")
                return ser
            except serial.SerialException as e:
                msg = str(e)
        else:
            msg = "устройство не найдено"
        if not warned:
            print(f"жду USB-VESC ({port}): {msg}")
            warned = True
        time.sleep(2)


class Drive:
    def __init__(self, port: str, max_erpm: int):
        self.port = port
        self.ser = open_port_wait(port)
        self.ser_lock = threading.Lock()   # TX-контур vs скан шины
        self.max_erpm = max_erpm
        self.lock = threading.Lock()
        # источник -> {'l': eRPM левого борта, 'r': eRPM правого, 't': время команды}
        self.sources = {}
        self.active_src = None
        self.estop = False
        self.tx_count = 0
        self.identify_key = None           # какой мотор крутим на опознании
        self.identify_until = 0.0
        self.known_ids = []                # CAN ID, видимые через ping (без локального)
        self.load_cfg()
        self.out = {}                      # сглаженная скорость по ключу мотора
        self.subs = {}                     # addr -> время окончания подписки
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind((UDP_HOST, UDP_PORT))
        threading.Thread(target=self._loop, daemon=True).start()
        threading.Thread(target=self._udp_loop, daemon=True).start()
        threading.Thread(target=self._state_loop, daemon=True).start()

    # ---------- конфиг ----------
    def load_cfg(self):
        self.config = {}
        try:
            with open(CFG_PATH, encoding="utf-8") as f:
                d = json.load(f)
            self.config = d.get("config", {})
            self.max_erpm = d.get("max_erpm", self.max_erpm)
        except (OSError, ValueError):
            pass

    def save_cfg(self):
        with open(CFG_PATH, "w", encoding="utf-8") as f:
            json.dump({"config": self.config, "max_erpm": self.max_erpm}, f,
                      ensure_ascii=False, indent=2)

    # ---------- команды ----------
    def _set_lr(self, src: str, left: float, right: float):
        m = self.max_erpm
        with self.lock:
            self.sources[src] = {"l": max(-m, min(m, left)),
                                 "r": max(-m, min(m, right)),
                                 "t": time.time()}
            self.estop = False

    def command_web(self, lin: float, ang: float):
        lin = max(-1.0, min(1.0, lin))
        ang = max(-1.0, min(1.0, ang))
        self._set_lr("web", (lin - ang) * self.max_erpm, (lin + ang) * self.max_erpm)

    def command_vel(self, src: str, vx: float, wz: float):
        if not (vx == vx and wz == wz and abs(vx) < 1e6 and abs(wz) < 1e6):
            return
        half = TRACK_WIDTH_M / 2.0
        self._set_lr(src, (vx - wz * half) * ERPM_PER_MPS, (vx + wz * half) * ERPM_PER_MPS)

    def stop(self, src=None, estop=False):
        with self.lock:
            if src is None:
                self.sources.clear()
            else:
                self.sources.pop(src, None)
            self.identify_key = None
            if estop:
                self.estop = True

    def identify(self, key: str):
        with self.lock:
            self.sources.clear()
            self.estop = False
            self.identify_key = key
            self.identify_until = time.time() + 1.0

    def heartbeat(self):
        with self.lock:
            if self.identify_key is not None:
                self.identify_until = time.time() + 1.0

    def assign(self, key: str, position: str, forward_sign: int):
        # позиция уникальна — снимаем её с других моторов
        for k in list(self.config):
            if self.config[k]["position"] == position and k != key:
                del self.config[k]
        self.config[key] = {"position": position, "forward_sign": 1 if forward_sign >= 0 else -1}
        self.save_cfg()

    def unassign(self, key: str):
        self.config.pop(key, None)
        self.save_cfg()

    def scan(self):
        with self.ser_lock:
            self.ser.reset_input_buffer()
            self.ser.write(frame(bytes([COMM_PING_CAN])))
            p = read_packet(self.ser, timeout=12.0)
        if p and p[0] == COMM_PING_CAN:
            self.known_ids = sorted(p[1:])
            return self.known_ids
        return None

    # ---------- UDP-API (протокол rover-motord, ходит vesc_bridge_node) ----------
    def _state_snapshot(self) -> dict:
        with self.lock:
            src = self.active_src if self.sources else None
            cmd = self.sources.get(src) if src else None
        return {
            "type": "state", "v": 1,
            "link": {"state": "ok"},        # USB-порт открыт, телеметрии по CAN нет
            "drive": {"src": src, "deadman": src is None,
                      "command": {"left_erpm": cmd["l"] if cmd else 0.0,
                                  "right_erpm": cmd["r"] if cmd else 0.0}},
            "enc": {"valid": False, "counts": [0] * 4, "mps": [0.0] * 4,
                    "seq": 0, "age_ms": None},
            "battery": {}, "wheels": [],
        }

    def _udp_loop(self):
        while True:
            try:
                data, addr = self.udp.recvfrom(65535)
            except ConnectionResetError:
                continue  # Windows: ICMP unreachable от прошлого sendto
            except OSError:
                time.sleep(0.1)
                continue
            try:
                msg = json.loads(data.decode("utf-8"))
            except ValueError:
                continue
            if not isinstance(msg, dict):
                continue
            src = (str(msg.get("src", "")).strip() or "udp")[:16]
            cmd = msg.get("cmd")
            reply = None
            if cmd == "drive":
                self.command_vel(src, float(msg.get("vx", 0.0)), float(msg.get("wz", 0.0)))
            elif cmd == "stop":
                self.stop(src)
            elif cmd == "estop":
                self.stop(estop=True)
            elif cmd == "sub":
                self.subs[addr] = time.time() + SUB_TTL_S
            elif cmd in ("get_state", "ping"):
                reply = self._state_snapshot() if cmd == "get_state" else {"type": "pong"}
            if reply is not None:
                try:
                    self.udp.sendto(json.dumps(reply, separators=(",", ":")).encode(), addr)
                except OSError:
                    pass

    def _state_loop(self):
        while True:
            time.sleep(1.0 / STATE_HZ)
            now = time.time()
            for a in [a for a, t in self.subs.items() if t < now]:
                self.subs.pop(a, None)
            if not self.subs:
                continue
            data = json.dumps(self._state_snapshot(), separators=(",", ":")).encode()
            for a in list(self.subs):
                try:
                    self.udp.sendto(data, a)
                except OSError:
                    pass

    # ---------- отправка ----------
    def _send_rpm(self, key, erpm: int):
        p = bytes([COMM_SET_RPM]) + struct.pack(">i", int(erpm))
        if key != LOCAL:
            p = bytes([COMM_FORWARD_CAN, int(key)]) + p
        self.ser.write(frame(p))

    def _send_release(self, key):
        p = bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0)
        if key != LOCAL:
            p = bytes([COMM_FORWARD_CAN, int(key)]) + p
        self.ser.write(frame(p))

    def motor_keys(self):
        return [LOCAL] + [str(i) for i in self.known_ids]

    def _loop(self):
        period = 1.0 / LOOP_HZ
        while True:
            t0 = time.time()
            step = RAMP_ERPM_S / LOOP_HZ
            with self.lock:
                ident = self.identify_key if (not self.estop and time.time() < self.identify_until) else None
                if self.identify_key is not None and ident is None:
                    self.identify_key = None
                for s in [s for s, c in self.sources.items() if t0 - c["t"] >= DEADMAN_S]:
                    del self.sources[s]
                src = max(self.sources, key=lambda s: SRC_PRIORITY.get(s, 0), default=None)
                if not self.estop and ident is None and src != self.active_src:
                    print(f"источник управления: {src or 'нет (deadman)'}")
                    self.active_src = src
                active = ident is None and (not self.estop) and src is not None
                left = self.sources[src]["l"] if active else 0.0
                right = self.sources[src]["r"] if active else 0.0
            if not self.ser_lock.acquire(blocking=False):
                time.sleep(period)
                continue
            try:
                for key in self.motor_keys():
                    if ident is not None:
                        target = IDENT_ERPM if key == ident else 0.0
                        sign = 1
                    else:
                        cfg = self.config.get(key)
                        if active and cfg:
                            target = left if SIDE[cfg["position"]] < 0 else right
                            sign = cfg["forward_sign"]
                        else:
                            target, sign = 0.0, 1
                    cur = self.out.get(key, 0.0)
                    if target > cur + step:
                        cur += step
                    elif target < cur - step:
                        cur -= step
                    else:
                        cur = target
                    self.out[key] = cur
                    v = cur
                    if 0 < abs(v) < MIN_ERPM:
                        v = MIN_ERPM if v > 0 else -MIN_ERPM
                    if abs(cur) > 1 or (ident is not None and key == ident) or active:
                        self._send_rpm(key, v * sign)
                    else:
                        self._send_release(key)
                self.tx_count += 1
                if self.ser.in_waiting:
                    self.ser.read(self.ser.in_waiting)
            except serial.SerialException as e:
                print(f"serial: {e} — переоткрываю порт")
                try:
                    self.ser.close()
                except serial.SerialException:
                    pass
                self.ser = open_port_wait(self.port)
            finally:
                self.ser_lock.release()
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)


PAGE = """<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>GIGAROVER USB teleop</title><style>
:root{--bg:#0e1116;--panel:#161b22;--panel2:#1c232d;--line:#2a333f;--txt:#e6edf3;
--dim:#9aa7b4;--accent:#3b82f6;--accent2:#22c55e;--danger:#ef4444}
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,sans-serif;background:var(--bg);color:var(--txt);
touch-action:manipulation;-webkit-user-select:none;user-select:none}
main{display:grid;grid-template-columns:1fr 1fr;gap:16px;padding:16px;max-width:1000px;margin:0 auto}
@media(max-width:820px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:16px}
.card h2{font-size:13px;margin:0 0 12px;text-transform:uppercase;color:var(--dim)}
button{font:inherit;color:var(--txt);background:var(--panel2);border:1px solid var(--line);
padding:9px 14px;border-radius:10px;cursor:pointer}
button.primary{background:var(--accent);border-color:var(--accent);color:#fff}
button.good{background:var(--accent2);border-color:var(--accent2);color:#04210f}
button.stop{background:var(--danger);border-color:var(--danger);color:#fff;font-weight:700}
button.ghost{background:transparent}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
.muted{color:var(--dim);font-size:13px}
.motorlist{display:flex;flex-direction:column;gap:8px;margin-top:10px}
.motor{display:flex;align-items:center;gap:10px;padding:10px 12px;border:1px solid var(--line);
border-radius:10px;background:var(--panel2)}
.motor .id{font-weight:700;min-width:72px}
.badge{font-size:11px;padding:2px 7px;border-radius:6px;background:#20303f;color:#8ec5ff}
.badge.fwd{background:#123020;color:#7ee2a8}
.badge.rev{background:#3a2416;color:#f0b483}
.spacer{flex:1}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:12px 0}
.cell{padding:16px 10px;border:1.5px solid var(--line);border-radius:12px;text-align:center;
cursor:pointer;background:var(--panel2)}
.cell:hover{border-color:var(--accent)}
.cell.sel{border-color:var(--accent2);background:#12251a}
.cell.taken{opacity:.55}
.cell b{display:block;font-size:15px}
.cell small{color:var(--dim)}
.frontlabel{grid-column:1/3;text-align:center;color:var(--dim);font-size:11px;
letter-spacing:.15em;text-transform:uppercase}
.hint{background:#111a24;border:1px dashed #2c3d4e;border-radius:10px;padding:10px 12px;
font-size:13px;color:#9fd0ff;margin:10px 0}
.dirbtns{display:flex;gap:10px;margin-top:8px}
.dirbtns button{flex:1}
.padwrap{display:flex;flex-direction:column;align-items:center;gap:14px}
#pad{width:min(70vw,320px);height:min(70vw,320px);border-radius:50%;position:relative;
touch-action:none;border:1px solid var(--line);
background:radial-gradient(circle at center,#1b2530,#12181f)}
#knob{width:26%;height:26%;border-radius:50%;background:#4caf50;position:absolute;
left:37%;top:37%;box-shadow:0 0 20px #4caf5080}
#info{font-size:13px;color:var(--dim);min-height:1.2em}
</style></head><body>
<main>
<section class="card">
<h2>Настройка</h2>
<div class="row">
  <button class="primary" id="scanBtn">🔍 Сканировать шину</button>
  <span class="muted" id="scanInfo"></span>
</div>
<div class="motorlist" id="motorList"></div>
<div id="wizard" style="display:none">
  <div class="hint" id="wizHint"></div>
  <div class="grid2">
    <div class="frontlabel">↑ Перёд робота ↑</div>
    <div class="cell" data-pos="FL"><b>◱</b><small>перёд-лево</small></div>
    <div class="cell" data-pos="FR"><b>◲</b><small>перёд-право</small></div>
    <div class="cell" data-pos="RL"><b>◳</b><small>зад-лево</small></div>
    <div class="cell" data-pos="RR"><b>◰</b><small>зад-право</small></div>
    <div class="frontlabel">↓ Зад робота ↓</div>
  </div>
  <div id="dirStep" style="display:none">
    <div class="muted">В какую сторону сейчас едет это колесо?</div>
    <div class="dirbtns">
      <button class="good" data-dir="1">➡️ Вперёд</button>
      <button data-dir="-1">⬅️ Назад</button>
    </div>
  </div>
  <div class="row" style="margin-top:12px"><button class="ghost" id="wizStop">Остановить</button></div>
</div>
</section>
<section class="card">
<h2>Управление</h2>
<div class="padwrap">
  <div id="pad"><div id="knob"></div></div>
  <button class="stop" id="stop">⛔ СТОП</button>
  <div id="info">—</div>
</div>
</section>
</main>
<script>
const api=(p,b)=>fetch(p,{method:'POST',headers:{'Content-Type':'application/json'},
body:JSON.stringify(b||{})}).then(r=>r.json());
let ST={config:{},known:[],max_erpm:0};
let identifying=null, hbTimer=null;

async function refresh(){
  ST=await fetch('/api/state').then(r=>r.json());
  renderMotors();
}
setInterval(refresh,1500);

document.getElementById('scanBtn').onclick=async e=>{
  const b=e.target;b.disabled=true;b.textContent='Сканирую… (~10 c)';
  const r=await api('/api/scan');
  b.disabled=false;b.textContent='🔍 Сканировать шину';
  document.getElementById('scanInfo').textContent=
    r.ids?('на CAN найдено: '+r.ids.join(', ')):'скан не удался';
  refresh();
};

function renderMotors(){
  const list=document.getElementById('motorList');list.innerHTML='';
  (ST.known||[]).forEach(key=>{
    const c=ST.config[key];
    const el=document.createElement('div');el.className='motor';
    let posHtml='<span class="muted">не назначен</span>';
    if(c){
      const dir=c.forward_sign>=0?'<span class="badge fwd">вперёд =+</span>'
                                 :'<span class="badge rev">вперёд =−</span>';
      posHtml=`<span class="badge">${c.position}</span> ${dir}`;
    }
    el.innerHTML=`<span class="id">${key==='local'?'USB':'CAN '+key}</span> ${posHtml}
      <div class="spacer"></div>`;
    const bI=document.createElement('button');
    bI.textContent=identifying&&identifying.key===key?'● крутится':'▶ Крутить';
    bI.className=identifying&&identifying.key===key?'good':'';
    bI.onclick=()=>startIdentify(key);
    el.appendChild(bI);
    if(c){
      const bU=document.createElement('button');bU.className='ghost';bU.textContent='✕';
      bU.title='убрать назначение';
      bU.onclick=async()=>{await api('/api/unassign',{key});refresh();};
      el.appendChild(bU);
    }
    list.appendChild(el);
  });
}

function startIdentify(key){
  identifying={key,position:null};
  api('/api/identify',{key});
  startHeartbeat();
  document.getElementById('wizard').style.display='block';
  document.getElementById('dirStep').style.display='none';
  document.getElementById('wizHint').textContent=
    `Мотор ${key==='local'?'USB':'CAN '+key} медленно крутится. Нажми позицию крутящегося колеса.`;
  updateCells();renderMotors();
}
function updateCells(){
  document.querySelectorAll('.cell').forEach(c=>{
    const pos=c.dataset.pos;
    c.classList.toggle('sel',identifying&&identifying.position===pos);
    const taken=Object.entries(ST.config).some(([k,v])=>
      v.position===pos&&(!identifying||identifying.key!==k));
    c.classList.toggle('taken',taken);
  });
}
document.querySelectorAll('.cell').forEach(cell=>{
  cell.onclick=()=>{
    if(!identifying)return;
    identifying.position=cell.dataset.pos;
    updateCells();
    document.getElementById('dirStep').style.display='block';
    document.getElementById('wizHint').textContent=
      `Позиция ${cell.dataset.pos} выбрана. Куда едет колесо при этом вращении?`;
  };
});
document.querySelectorAll('#dirStep [data-dir]').forEach(b=>{
  b.onclick=async()=>{
    if(!identifying||!identifying.position)return;
    await api('/api/assign',{key:identifying.key,position:identifying.position,
      forward_sign:parseInt(b.dataset.dir)});
    await stopIdentify();
  };
});
document.getElementById('wizStop').onclick=stopIdentify;
async function stopIdentify(){
  await api('/api/stop');
  identifying=null;stopHeartbeat();
  document.getElementById('wizard').style.display='none';
  refresh();
}
function startHeartbeat(){stopHeartbeat();hbTimer=setInterval(()=>api('/api/heartbeat'),200);}
function stopHeartbeat(){if(hbTimer){clearInterval(hbTimer);hbTimer=null;}}

// ---------- джойстик ----------
const pad=document.getElementById('pad'),knob=document.getElementById('knob'),
info=document.getElementById('info');
let lin=0,ang=0,active=false;
function setKnob(nx,ny){const r=pad.clientWidth/2,k=knob.clientWidth/2;
knob.style.left=(r+nx*r*0.72-k)+'px';knob.style.top=(r+ny*r*0.72-k)+'px';}
function fromEvent(e){const rect=pad.getBoundingClientRect();
const t=e.touches?e.touches[0]:e;
let nx=((t.clientX-rect.left)/rect.width)*2-1,ny=((t.clientY-rect.top)/rect.height)*2-1;
const m=Math.hypot(nx,ny);if(m>1){nx/=m;ny/=m}
lin=-ny;ang=-nx;setKnob(nx,ny);}
function release(){active=false;lin=0;ang=0;setKnob(0,0);send();}
pad.addEventListener('pointerdown',e=>{active=true;pad.setPointerCapture(e.pointerId);fromEvent(e);});
pad.addEventListener('pointermove',e=>{if(active)fromEvent(e);});
pad.addEventListener('pointerup',release);
pad.addEventListener('pointercancel',release);
document.getElementById('stop').onclick=async()=>{release();await api('/api/stop');};
async function send(){try{
const j=await api('/api/drive',{linear:lin,angular:ang});
info.textContent=`lin ${lin.toFixed(2)}  ang ${ang.toFixed(2)}  tx ${j.tx}`;
}catch(e){info.textContent='нет связи с сервером';}}
setInterval(()=>{if(active)send();},100);
setKnob(0,0);refresh();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    drive: Drive = None

    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        # мастер настройки моторов встроен и в веб-морду ROS-шлюза (:8765) —
        # она ходит сюда кросс-доменно
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/state":
            d = self.drive
            self._json({"config": d.config, "known": d.motor_keys(), "max_erpm": d.max_erpm})
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            d = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self._json({"error": "bad json"}, 400)
        drv = self.drive
        if self.path == "/api/drive":
            drv.command_web(float(d.get("linear", 0)), float(d.get("angular", 0)))
            self._json({"ok": True, "tx": drv.tx_count})
        elif self.path in ("/api/stop", "/api/drive/stop"):
            drv.stop(estop=True)
            self._json({"ok": True})
        elif self.path == "/api/scan":
            ids = drv.scan()
            self._json({"ids": ids} if ids is not None else {"ids": None, "error": "no reply"})
        elif self.path == "/api/identify":
            drv.identify(str(d.get("key")))
            self._json({"ok": True})
        elif self.path == "/api/heartbeat":
            drv.heartbeat()
            self._json({"ok": True})
        elif self.path == "/api/assign":
            drv.assign(str(d.get("key")), str(d.get("position")), int(d.get("forward_sign", 1)))
            self._json({"ok": True})
        elif self.path == "/api/unassign":
            drv.unassign(str(d.get("key")))
            self._json({"ok": True})
        else:
            self._json({"error": "not found"}, 404)


def main():
    global RAMP_ERPM_S
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="auto", help="COMx | /dev/ttyACMx | auto")
    ap.add_argument("--bind", default="127.0.0.1", help="адрес HTTP-сервера (0.0.0.0 — для доступа извне)")
    ap.add_argument("--http-port", type=int, default=8770)
    ap.add_argument("--max-erpm", type=int, default=4000)
    ap.add_argument("--ramp", type=int, default=RAMP_ERPM_S, help="темп изменения скорости, eRPM/с")
    args = ap.parse_args()
    RAMP_ERPM_S = args.ramp
    drv = Drive(args.port, args.max_erpm)
    Handler.drive = drv
    print("Сканирую CAN-шину при старте…")
    ids = drv.scan()
    print(f"CAN ID: {ids}" if ids is not None else "скан не удался (можно повторить с веба)")
    srv = ThreadingHTTPServer((args.bind, args.http_port), Handler)
    print(f"Телеоп: http://{args.bind}:{args.http_port}  (VESC на {args.port}, max {args.max_erpm} eRPM)")
    srv.serve_forever()


if __name__ == "__main__":
    main()
