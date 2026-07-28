# rover-motord — демон ходовой GIGAROVER (владелец CAN)

Постоянный процесс без ROS, единственный владелец USB-CAN адаптера. Вся
моторная логика (скид-стир микширование, слю-лимиты, deadman, подпор
`min_erpm`, предел `max_erpm`, коаст при любом выходе) живёт здесь и
работает одинаково при живом и мёртвом ROS-стеке. ROS-узел
`vesc_bridge_node` из `rover_vesc_driver` — тонкий клиент этого демона.

Зачем так: ROS 2 поверх этой CAN-шины показал себя ненадёжно, а прежняя
схема «дренаж RX в цикле управления» роняла частоту команд с 50 до ~10 Гц
на тихой шине (seeedstudio-бэкенд python-can игнорирует timeout у `recv()`
и блокируется по таймауту серийника). Здесь приём вынесен в отдельный
блокирующий поток, и команды идут строго по расписанию.

## Архитектура

```
телефон ── HTTP :8767 ──┐            ┌── RX-поток (блокирующий recv)
                        ├─ MotorCore ┤
ROS-мост ── UDP :8460 ──┘   50 Гц    └── TX: SET_RPM/SET_CURRENT
   ↑ state 50 Гц │ арбитраж web>ros, deadman на источник,
   └─────────────┘ enc_seq, link ok/degraded/down, метрики
```

- **Арбитраж**: источник `web` (телефон, HTTP) перехватывает управление у
  `ros`, пока шлёт команды; замолчал на `command_timeout_sec` (0.5 с) —
  управление возвращается к `ros`. Тишина ото всех — deadman, коаст.
- **estop** (`POST /api/stop` или UDP `estop`): мгновенный коаст + блок
  движения `stop_hold_sec` (0.75 с) для всех источников.
- **Линк**: `ok` / `degraded` (телеметрия колёс протухла) / `down` (нет ни
  одного кадра дольше 1 с). Метрики: частоты статусов по колёсам, max gap,
  количество gap>150 мс, ошибки декодирования, кадры с чужих CAN ID,
  ошибки TX, переоткрытия шины, оверраны такта.
- **Watchdog**: systemd `Type=notify` + `WatchdogSec=15`; зависший демон
  перезапускается сам (`Restart=always`).

## Файлы

| Файл | Что это |
|---|---|
| `motord.py` | Весь демон: ядро CAN + UDP API + HTTP-телеоп |
| `web/index.html` | Аварийная веб-морда (телефон), самодостаточная |
| `test_motord_sim.py` | Интеграционный тест поверх `--sim` (без железа) |
| `../../deploy/systemd/rover-motord.service` | systemd-юнит |

## Конфигурация (в порядке применения)

1. Встроенные дефолты (совпадают с `gigarover_v1.yaml`).
2. Конфиг робота `src/rover_bringup/config/gigarover_v1.yaml` — геометрия,
   энкодеры (`reduction_ratio`, `encoder_lines`), пределы `base_driver`.
   Ищется в workspace автоматически; перекрывается `$ROVER_ROBOT_CONFIG`
   или `--robot-config`.
3. `~/rover_config/motors.yaml` (экспорт rover-setup-web) — CAN ID колёс,
   инверсии, канал, битрейт. Правила слияния те же, что в launch ROS-стека
   (волатильные `/dev/ttyUSB*` заменяются на `/dev/rover_can`).
   `$ROVER_CONFIG_DIR` учитывается; переопределение — `--motors-config`.
4. Флаги CLI (`--channel`, порты, `--stats N`, `--sim`). Для systemd
   дополнительные флаги задаются переменной `MOTORD_OPTS` в
   `/etc/default/rover-bringup`.

## UDP API (127.0.0.1:8460, JSON-датаграммы)

| Датаграмма | Действие |
|---|---|
| `{"v":1,"src":"ros","cmd":"drive","vx":0.5,"wz":0.1}` | команда (м/с, рад/с); клампы и микширование внутри |
| `{"v":1,"src":"ros","cmd":"stop"}` | мягкий стоп источника (цель 0) |
| `{"v":1,"cmd":"estop"}` | аварийный стоп всем + stop_hold |
| `{"v":1,"cmd":"sub"}` | подписка на поток state (TTL 3 с, любой трафик продлевает) |
| `{"v":1,"cmd":"get_state"}` | разовый state в ответ |
| `{"v":1,"cmd":"ping"}` | проверка живости, ответ `{"v":1,"type":"pong"}` |

Подписчикам на каждом такте (50 Гц) уходит датаграмма `type:"state"`:
`can` (connected/channel/reopens), `link` (state+метрики), `drive`
(src/deadman/цели/выход), `enc` (seq/valid/age_ms/counts/mps — гейтинг:
seq растёт только когда ВСЕ 4 колеса дали новый Status 5), `wheels`
(телеметрия по колёсам), `battery`. UDP слушает только localhost — снаружи
управления нет.

## HTTP API (0.0.0.0:8767)

Совместимо по формам с веб-мордой ROS-стека: `GET /` (телеоп),
`/api/health`, `/api/status`, `/api/drive`; `POST /api/drive/command
{linear_x,angular_z}`, `/api/drive/stop`, `/api/stop`, `/api/heartbeat`
(совместимость со старым UI, no-op). Команды с HTTP идут
от источника `web` и имеют приоритет над ROS — телефон всегда может
перехватить ровер: `http://10.42.0.1:8767` в сети GIGAROVER.

## Службы и владение адаптером

| Служба | Роль | CAN |
|---|---|---|
| `rover-motord` | демон ходовой, автозапуск | владелец |
| `rover-bringup` | ROS-стек, автозапуск (`Wants/After` motord) | не трогает |
| `rover-setup-web` | первичная настройка (Conflicts= motord+bringup) | владеет на время настройки |
| `rover-can-teleop` | замороженный резерв (Conflicts= со всеми) | владеет, если запущен |

Откат на прямой драйвер: в `gigarover_v1.yaml` `base_driver.type:
vesc_direct`, затем `sudo systemctl disable --now rover-motord` и
`sudo systemctl restart rover-bringup`. Не запускайте `vesc_direct` при
живом motord — два писателя на шине.

## Диагностика

```bash
journalctl -u rover-motord -f          # журнал + сводка метрик (--stats 30)
curl -s localhost:8767/api/health      # живость, link_state, drive_src
curl -s localhost:8767/api/status | python3 -m json.tool   # всё состояние
```

Метрики также видны в ROS: `/diagnostics`, статус `vesc_can_link`.

## Тест без железа

```bash
python3 test_motord_sim.py             # интеграционный прогон на симуляторе
python3 motord.py --sim                # демон на симуляторе (проверка стека
                                       # и веб-морды без ходовой)
```
