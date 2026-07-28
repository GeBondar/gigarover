# Деплой rover-motord на ровер — раннбук

Дата подготовки: 2026-07-28. Все шаги, которые можно было сделать без
железа, сделаны: код написан, интеграционный тест демона на симуляторе
(`tools/motord/test_motord_sim.py`) зелёный, синтаксис всех скриптов
проверен. Осталось перенести на ровер, собрать и проверить на месте.

## Что меняется

Ходовой теперь владеет постоянный демон **rover-motord** (без ROS,
`tools/motord/motord.py`): RX в отдельном блокирующем потоке (лечит
провал частоты команд 50→10 Гц на тихой шине), арбитраж web>ros, deadman
на источник, метрики линка, watchdog systemd, аварийный телеоп на
**:8767**. ROS-узел VESC заменён тонким мостом `vesc_bridge_node`
(cmd_vel → UDP, state → энкодеры/батарея/диагностика; QoS keep_last(1)).
Прежний драйвер сохранён как режим отката `vesc_direct`.

Затронутые файлы:

```
tools/motord/                                  (новый: демон, web, тест, README)
src/rover_vesc_driver/rover_vesc_driver/vesc_bridge_node.py   (новый)
src/rover_vesc_driver/setup.py                 (+entry point)
src/rover_vesc_driver/README.md
src/rover_bringup/launch/robot.launch.py       (vesc→мост, vesc_direct→легаси)
src/rover_bringup/config/components/base.yaml  (+motord_host/port/timeout)
deploy/systemd/rover-motord.service            (новый)
deploy/systemd/{rover-bringup,rover-setup-web}.service, install.sh, uninstall.sh
tools/can_teleop/{rover-can-teleop.service,install.sh,README.md}  (Conflicts+пометка)
install.sh, README.md, docs/deploy_motord.md
```

## Шаг 1. Перенос файлов (с рабочего ПК, Windows)

Из каталога `Desktop\_\gigarover` (PowerShell; tar в Windows 10+ есть).
Архив через файл, а не через конвейер — пайпы PowerShell 5.1 портят
бинарные потоки:

```bash
cd C:\Users\George\Desktop\_\gigarover
tar -C rover_ws -czf motord_deploy.tgz install.sh README.md docs deploy tools/motord tools/can_teleop src/rover_vesc_driver src/rover_bringup/launch/robot.launch.py src/rover_bringup/config/components/base.yaml
scp motord_deploy.tgz gigarover:/tmp/
ssh gigarover "tar -C ~/rover_ws -xzf /tmp/motord_deploy.tgz && rm /tmp/motord_deploy.tgz"
```

Переносится только изменённое: правленные на ровере конфиги
(`gigarover_v1.yaml`, nav2-параметры, `~/rover_config/motors.yaml`)
не трогаются.

## Шаг 2. Сборка и установка (на ровере)

```bash
ssh gigarover
```

```bash
sudo systemctl stop rover-bringup
cd ~/rover_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select rover_vesc_driver rover_bringup
bash ~/rover_ws/deploy/systemd/install.sh
bash ~/rover_ws/tools/can_teleop/install.sh
```

`deploy/systemd/install.sh` ставит и включает rover-motord, обновляет
юниты bringup/setup-web (Wants/Conflicts), делает daemon-reload.
Порядок важен: bringup останавливается ДО первого старта motord — старая
сборка bringup ещё держит адаптер прямым драйвером.

## Шаг 3. Первый запуск демона (bringup пока выключен)

```bash
sudo systemctl start rover-motord
journalctl -u rover-motord -n 30 --no-pager
```

Ожидаемо в журнале: конфиг робота подхвачен, `Конфиг моторов:
~/rover_config/motors.yaml`, `CAN поднят: seeedstudio:/dev/rover_can @
500000`, `CAN-линк: down -> ok` (если VESC запитаны). Затем:

```bash
curl -s localhost:8767/api/health
# ожидаем: "can_connected": true, "link_state": "ok"
curl -s localhost:8767/api/status | python3 -m json.tool | head -60
systemctl show rover-motord -p CPUSchedulingPolicy,CPUAffinity,WatchdogUSec
```

Если старт падает именно на CPUScheduling (ядро без RT-планировщика —
маловероятно) — закомментировать обе строки `CPUScheduling*` в
`/etc/systemd/system/rover-motord.service`, `sudo systemctl daemon-reload
&& sudo systemctl restart rover-motord`.

**Тест телеопа без ROS** (колёса на подставке!): с телефона в сети
GIGAROVER открыть `http://10.42.0.1:8767`, подвигать джойстик, проверить
телеметрию и кнопку STOP. Это и есть аварийный канал управления.

## Шаг 4. Запуск ROS-стека и сквозная проверка

```bash
sudo systemctl start rover-bringup
source /opt/ros/jazzy/setup.bash && source ~/rover_ws/install/setup.bash
ros2 topic hz /wheel/encoders     # ~частота Status 5 (50 Гц после настройки VESC)
ros2 topic echo /wheel/encoders --once     # valid: true, счётчики живые
ros2 topic echo /diagnostics --once        # vesc_fl..rr OK + vesc_can_link ok
```

Чеклист поведения:

1. Телеоп с веб-морды ROS (`:8765`) едет; в `journalctl -u rover-motord`
   видно `команды: источник ros`.
2. Открыть `:8767` с телефона, дать команду — в журнале `источник web
   (перехват у ros)`; отпустить джойстик — через 0.5 с управление
   возвращается ros.
3. Deadman: `sudo systemctl stop rover-bringup` во время движения от ROS —
   колёса коастятся ≤0.5 с; `start` обратно — управление восстанавливается.
4. Watchdog: `sudo kill -STOP $(pidof -x motord.py)` (симуляция зависания) —
   через ~15 с systemd перезапустит демона (`journalctl -u rover-motord`),
   моторы на время рестарта коастятся VESC-таймаутом. Не забыть, что STOP
   заморозит и коаст-кадры — делать на подставке.
5. Метрики за поездку: строка `метрики:` в журнале каждые 30 с
   (gap_max, gaps150, decode_err, unknown, reopens) — это теперь основной
   инструмент оценки качества линка вместо тест-скриптов.

## Шаг 5. VESC Tool (до сих пор не сделано — важно)

На каждом из четырёх контроллеров:

- **включить CAN Status 5** (тахометр+V_in — без него нет одометрии) и
  выставить Status 1, 4, 5 на **50 Гц**, Status 2 — 1–5 Гц или выключить;
- App Settings → General → **Timeout** (CAN command timeout): 0.5–1.0 с —
  последний рубеж failsafe при полном зависании борта;
- прогнать Motor Wizard (FOC detection) под PX60 — настоящая причина
  «дёрганий» на малых оборотах, min_erpm=900 лишь обходит её.

## Откат

```bash
# 1) на ровере: конфиг робота
sed -i 's/^  type: vesc$/  type: vesc_direct/' ~/rover_ws/src/rover_bringup/config/gigarover_v1.yaml
# 2) выключить демона и перезапустить стек со старым драйвером
sudo systemctl disable --now rover-motord
sudo systemctl restart rover-bringup
```

Прямой драйвер (`vesc_driver_node`) остался в пакете нетронутым.
Обратно: `type: vesc`, `sudo systemctl enable --now rover-motord`,
`sudo systemctl restart rover-bringup`. Последний резерв — старый
`rover-can-teleop` (`sudo systemctl start rover-can-teleop`, телеоп
на :8765 при остановленном всём остальном).

## Известные ограничения

- `vesc_direct` и rover-motord одновременно = два писателя на шине;
  Conflicts= это не ловит (bringup формально не конфликтует с демоном) —
  порядок отката выше обязателен.
- Демон слушает UDP только на 127.0.0.1 — управление извне только через
  веб-морды. HTTP :8767 без аутентификации, как и :8765 (защита — WPA2
  точки GIGAROVER).
- Физика прежняя: EMI-провалы при вращении моторов лечатся витой парой,
  двумя терминаторами 120 Ом и общим GND; питание платы — отдельная
  задача (brownout'ы). Демон это не чинит — он это переживает и меряет.
