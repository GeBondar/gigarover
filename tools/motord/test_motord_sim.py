#!/usr/bin/env python3
"""Интеграционный тест rover-motord в режиме --sim (без железа и без ROS).

Запускает motord.py --sim отдельным процессом и через реальные UDP/HTTP API
проверяет весь контракт демона, на который опирается ROS-мост:

  1. поток state подписчику, рост enc.seq, link ok, enc.valid;
  2. команда drive от src=ros: арбитраж, слю-разгон, eRPM, рост тахометра;
  3. клампы скорости (vx=99 -> max_linear_speed);
  4. перехват управления web (HTTP) у ros и возврат к ros по таймауту web;
  5. estop (/api/stop): мгновенный коаст, stop_hold, автосброс блокировки;
  6. deadman при молчании всех источников;
  7. форма HTTP /api/status (совместимость с веб-мордой) и UDP get_state.

Гоняется на любой машине: python3 test_motord_sim.py
Код возврата 0 = все проверки прошли.
"""
from __future__ import annotations

import json
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
UDP_PORT = 18460
HTTP_PORT = 18767
UDP_ADDR = ('127.0.0.1', UDP_PORT)
HTTP_BASE = f'http://127.0.0.1:{HTTP_PORT}'


class TestFailure(AssertionError):
    pass


def log(message: str) -> None:
    print(f'[test] {message}', flush=True)


class StateListener(threading.Thread):
    """Держит подписку на state-поток motord и хранит последний снимок."""

    def __init__(self) -> None:
        super().__init__(name='state_listener', daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.3)
        self._lock = threading.Lock()
        self._latest: dict | None = None
        self._count = 0
        self._stop = False
        self._last_sub = 0.0

    def run(self) -> None:
        while not self._stop:
            now = time.monotonic()
            if now - self._last_sub >= 1.0:
                self._last_sub = now
                self.send({'v': 1, 'cmd': 'sub'})
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except ConnectionResetError:
                continue              # Windows: ICMP от sub до старта motord
            except OSError:
                break
            try:
                msg = json.loads(data.decode('utf-8'))
            except ValueError:
                continue
            if msg.get('type') == 'state':
                with self._lock:
                    self._latest = msg
                    self._count += 1

    def send(self, payload: dict) -> None:
        try:
            self.sock.sendto(json.dumps(payload).encode('utf-8'), UDP_ADDR)
        except OSError:
            pass

    @property
    def latest(self) -> dict | None:
        with self._lock:
            return self._latest

    @property
    def count(self) -> int:
        with self._lock:
            return self._count

    def stop(self) -> None:
        self._stop = True
        try:
            self.sock.close()
        except OSError:
            pass


class RosCommander(threading.Thread):
    """Имитация ROS-моста: шлёт drive src=ros на 20 Гц, пока enabled."""

    def __init__(self) -> None:
        super().__init__(name='ros_commander', daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vx = 0.0
        self.wz = 0.0
        self.enabled = False
        self._stop = False

    def run(self) -> None:
        while not self._stop:
            if self.enabled:
                payload = {'v': 1, 'src': 'ros', 'cmd': 'drive',
                           'vx': self.vx, 'wz': self.wz}
                try:
                    self.sock.sendto(json.dumps(payload).encode(), UDP_ADDR)
                except OSError:
                    pass
            time.sleep(0.05)

    def stop(self) -> None:
        self._stop = True
        try:
            self.sock.close()
        except OSError:
            pass


def http_json(method: str, path: str, body: dict | None = None) -> dict:
    data = json.dumps(body).encode('utf-8') if body is not None else None
    request = urllib.request.Request(
        HTTP_BASE + path, data=data, method=method,
        headers={'Content-Type': 'application/json'},
    )
    with urllib.request.urlopen(request, timeout=3.0) as response:
        return json.loads(response.read().decode('utf-8'))


def wait_for(listener: StateListener, predicate, timeout: float,
             what: str) -> dict:
    deadline = time.monotonic() + timeout
    last_error = ''
    while time.monotonic() < deadline:
        state = listener.latest
        if state is not None:
            try:
                if predicate(state):
                    return state
            except (KeyError, TypeError, IndexError) as exc:
                last_error = f' (ошибка предиката: {exc!r})'
        time.sleep(0.02)
    state = listener.latest
    raise TestFailure(
        f'таймаут {timeout} с: {what}{last_error}; последний state: '
        + json.dumps(state, ensure_ascii=False)[:2000]
    )


def check(condition: bool, what: str, context: dict | None = None) -> None:
    if condition:
        log(f'OK: {what}')
        return
    extra = ''
    if context is not None:
        extra = '; контекст: ' + json.dumps(context, ensure_ascii=False)[:2000]
    raise TestFailure(f'ПРОВАЛ: {what}{extra}')


def main() -> int:
    proc = subprocess.Popen(
        [sys.executable, str(HERE / 'motord.py'), '--sim',
         '--udp-port', str(UDP_PORT), '--http-port', str(HTTP_PORT),
         '--http-host', '127.0.0.1', '--stats', '0',
         '--motors-config', str(HERE / 'no_such_motors.yaml')],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, encoding='utf-8', errors='replace',
    )
    output: list[str] = []

    def drain() -> None:
        assert proc.stdout is not None
        for line in proc.stdout:
            output.append(line.rstrip())

    threading.Thread(target=drain, name='drain', daemon=True).start()

    listener = StateListener()
    commander = RosCommander()
    try:
        listener.start()
        commander.start()

        # 1. Поток состояния и готовность симулированной шины.
        state = wait_for(listener, lambda s: s.get('type') == 'state', 3.0,
                         'первый state-датаграм')
        check(state['can']['sim'] is True, 'sim-шина активна', state['can'])
        state = wait_for(
            listener,
            lambda s: s['link']['state'] == 'ok' and s['enc']['valid'],
            3.0, 'link ok + свежая телеметрия всех колёс')
        check(len(state['wheels']) == 4, '4 колеса в state')
        seq_start = state['enc']['seq']
        time.sleep(0.5)
        state = listener.latest or state
        gained = state['enc']['seq'] - seq_start
        check(gained >= 10, f'enc.seq растёт ({gained} сэмплов за 0.5 с)',
              state['enc'])
        rate = listener.count
        time.sleep(1.0)
        rate = listener.count - rate
        check(rate >= 30, f'поток state >=30 Гц (получено {rate}/с)')

        # 2. Команды от ros: арбитраж, разгон, телеметрия.
        commander.vx = 0.4
        commander.enabled = True
        state = wait_for(listener, lambda s: s['drive']['src'] == 'ros', 1.5,
                         'источник ros активен')
        check(state['drive']['deadman'] is False, 'deadman снят')
        state = wait_for(
            listener,
            lambda s: abs(s['drive']['wheels_output_mps'][0] - 0.4) < 0.02,
            1.5, 'слю-лимитер дошёл до 0.4 м/с')
        state = wait_for(
            listener,
            lambda s: all(w['erpm'] > 500 for w in s['wheels']),
            2.0, 'все колёса крутятся (eRPM > 500 с учётом знаков)')
        counts_before = state['enc']['counts']
        state = wait_for(
            listener,
            lambda s: all(c > b + 10 for c, b in
                          zip(s['enc']['counts'], counts_before)),
            2.0, 'тахометры всех колёс растут (знаки согласованы)')

        # 3. Кламп скорости (командер молчит, чтобы не перетирать команду).
        commander.enabled = False
        time.sleep(0.1)
        listener.send({'v': 1, 'src': 'ros', 'cmd': 'drive', 'vx': 99.0, 'wz': 0.0})
        state = wait_for(
            listener,
            lambda s: abs(s['drive']['command']['linear_x'] - 1.5) < 0.001,
            1.0, 'vx=99 клампится ровно до max_linear_speed (1.5)')
        commander.enabled = True

        # 4. Перехват web у ros и возврат.
        reply = http_json('POST', '/api/drive/command', {'linear_x': -0.3})
        check(reply.get('ok') is True, 'HTTP drive принят')
        state = wait_for(
            listener,
            lambda s: (s['drive']['src'] == 'web'
                       and abs(s['drive']['command']['linear_x'] + 0.3) < 1e-6),
            1.0, 'web перехватил управление у ros')
        # web замолкает (одна команда), ros продолжает спамить 20 Гц.
        state = wait_for(listener, lambda s: s['drive']['src'] == 'ros', 1.5,
                         'после молчания web управление вернулось к ros')

        # 5. estop: мгновенный коаст + блокировка, затем автосброс.
        reply = http_json('POST', '/api/stop', {})
        check(reply.get('stopped') is True, 'HTTP estop принят')
        state = wait_for(
            listener,
            lambda s: (s['drive']['stop_hold'] is True
                       and all(v == 0.0 for v in s['drive']['wheels_output_mps'])),
            1.0, 'estop: stop_hold и мгновенный коаст')
        state = wait_for(
            listener,
            lambda s: (s['drive']['stop_hold'] is False
                       and s['drive']['src'] == 'ros'
                       and s['drive']['wheels_output_mps'][0] > 0.05),
            2.5, 'после stop_hold управление ros возобновилось')

        # 6. deadman при молчании всех источников.
        commander.enabled = False
        state = wait_for(
            listener,
            lambda s: (s['drive']['deadman'] is True
                       and s['drive']['src'] is None
                       and all(v == 0.0 for v in s['drive']['wheels_output_mps'])),
            1.5, 'deadman: тишина -> коаст')

        # 7. HTTP-снимок и UDP get_state.
        snap = http_json('GET', '/api/status')
        check(snap.get('ok') is True and snap.get('app') == 'rover-motord',
              '/api/status отвечает', {'app': snap.get('app')})
        check('front_left' in snap.get('wheels', {})
              and 'measured_mps' in snap['wheels']['front_left'],
              '/api/status: форма wheels совместима с веб-мордой')
        check(snap['link']['state'] == 'ok', '/api/status: link ok',
              snap.get('link'))
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.settimeout(2.0)
        probe.sendto(json.dumps({'v': 1, 'cmd': 'get_state'}).encode(), UDP_ADDR)
        data, _ = probe.recvfrom(65535)
        probe.close()
        one_shot = json.loads(data.decode('utf-8'))
        check(one_shot.get('type') == 'state', 'UDP get_state без подписки')

        health = http_json('GET', '/api/health')
        check(health.get('link_state') == 'ok', '/api/health: link_state ok')

        check(proc.poll() is None, 'демон жив после всех сценариев')
        log('ВСЕ ПРОВЕРКИ ПРОШЛИ')
        return 0
    except TestFailure as exc:
        log(str(exc))
        log('--- вывод motord (хвост) ---')
        for line in output[-40:]:
            log('  ' + line)
        return 1
    finally:
        commander.stop()
        listener.stop()
        if proc.poll() is None:
            proc.kill()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            pass


if __name__ == '__main__':
    sys.exit(main())
