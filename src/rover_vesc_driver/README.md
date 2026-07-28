# rover_vesc_driver

Пакет ходовой части GIGAROVER. Два исполняемых узла:

- **`vesc_bridge_node` — штатный режим** (`base_driver.type: vesc`).
  Тонкий мост к демону [`rover-motord`](../../tools/motord/README.md),
  который единолично владеет USB-CAN адаптером и ведёт контур 50 Гц.
  Узел не трогает шину вообще.
- **`vesc_driver_node` — легаси/откат** (`base_driver.type: vesc_direct`).
  Прежний драйвер с прямым доступом к CAN (python-can seeedstudio).
  Перед использованием остановите rover-motord: `sudo systemctl
  disable --now rover-motord` — два писателя на шине недопустимы.

## Мост (vesc_bridge_node)

- `/cmd_vel` подписывается с QoS `keep_last(1) + best_effort`: демону
  всегда уходит только самая свежая команда, очередь устаревших cmd_vel
  после стопора DDS не реплеится. Каждая команда — UDP-датаграмма
  `{"src":"ros","cmd":"drive"}` на `motord_host:motord_port`.
- Deadman — на стороне демона (0.5 с): рестарт ROS-стека безопасен,
  моторы коастятся сами; узел при завершении шину не дёргает.
- Телеметрия приходит потоком state (подписка обновляется раз в секунду):
  - `/wheel/encoders` — семантика прежнего драйвера сохранена полностью:
    публикация только когда все четыре колеса дали новый Status 5
    (демон ведёт счётчик `enc.seq`), штамп = `now - age` телеметрии,
    при устаревших данных — keep-alive `valid=false` на частоте
    `publish_rate_hz`. Одометрия downstream работает без изменений.
  - `/battery/state` — 2 Гц; `present=false`, если демон недоступен.
  - `/diagnostics` — 1 Гц: прежние статусы `vesc_fl..vesc_rr` плюс
    `vesc_can_link` с метриками линка (state ok/degraded/down, rx Гц,
    max gap, ошибки декодирования, чужие кадры, переоткрытия шины,
    активный источник команд). Демон недоступен — ERROR.

### Параметры моста

| Параметр | Дефолт | Смысл |
|---|---|---|
| `motord_host` / `motord_port` | `127.0.0.1` / `8460` | UDP API демона |
| `cmd_vel_topic` … `diagnostics_topic` | как раньше | топики |
| `publish_rate_hz` | 50.0 | темп проверки enc.seq и keep-alive |
| `state_timeout_sec` | 1.0 | молчание демона дольше — считаем его мёртвым |

Параметры собирает launch из `components/base.yaml` (секция
`vesc_driver`) — там же остаются can_*/wheel_* ключи для легаси-режима
(мост их игнорирует, демон читает свои конфиги сам).

## Легаси-драйвер (vesc_driver_node)

Прежний узел сохранён без изменений как режим отката: cmd_vel → SET_RPM
четырём VESC через python-can seeedstudio, телеметрия Status 1/2/4/5,
deadman, слю-лимиты, min_erpm. Подробности — в docstring узла. Известное
ограничение (одна из причин перехода на демона): приём в цикле управления
блокируется до 100 мс на тихой шине, роняя частоту команд.

## Запуск

Штатно всё поднимает `rover_bringup` (`base_driver.type: vesc` в
`gigarover_v1.yaml`) при работающей службе `rover-motord`. Вручную:

```bash
ros2 run rover_vesc_driver vesc_bridge_node   # нужен запущенный rover-motord
```

Проверка цепочки: `journalctl -u rover-motord -f`,
`curl -s localhost:8767/api/health`, `ros2 topic hz /wheel/encoders`,
`ros2 topic echo /diagnostics` (статус `vesc_can_link`).
