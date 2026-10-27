# AIDA · внешняя камера: робот на карте

Неподвижная RGB-камера над рабочей зоной определяет положение мобильного робота
на полу (x, y, курс) по доске AprilGrid на его корпусе. Всё работает в Docker
и управляется через локальный веб-интерфейс.

| Шаг | Страница | Что делает | Подробно |
|---|---|---|---|
| 1 | `/` | intrinsics камеры по доске AprilGrid 11 × 8 | [docs/INTRINSICS.md](docs/INTRINSICS.md) |
| 2 | `/floor.html` | привязка камеры к полу: доска 3 × 3 свободно перекладывается по полу | [docs/FLOOR.md](docs/FLOOR.md) |
| 3 | `/robot.html`, `/map.html` | высота и центр доски на роботе, живая карта, API позы | [docs/ROBOT.md](docs/ROBOT.md) |

Определение семейства маркеров по фото доски — [docs/BOARD_ID.md](docs/BOARD_ID.md).

## Быстрый старт

```sh
./run.sh            # сборка образа и запуск, http://localhost:8080
docker compose logs -f
docker compose stop
```

По умолчанию контейнеру передаётся Jabra PanaCast 20 по постоянному пути
`/dev/v4l/by-id/...`. Другая камера: `USB_CAMERA_DEVICE=/dev/v4l/by-id/<камера>-video-index0 ./run.sh`.

Поза робота для других программ: `GET http://localhost:8080/api/robot/pose`
или поток `GET /api/robot/events` (SSE). Метры, радианы, система координат пола.

## Калибровки в репозитории

Текущие результаты лежат в `data/` и подхватываются приложением при запуске.
Исходные кадры (PNG/JPG) в репозиторий не входят.

| Файл | Содержимое |
|---|---|
| `data/sessions/ea37…/intrinsics.json`, `.yaml` | матрица камеры и дисторсия, 3840 × 2160 |
| `data/current.json` | сессия intrinsics: наблюдения углов, результат |
| `data/floor/sessions/e7c8…/extrinsics.json`, `.yaml` | камера → пол: `T_floor_from_camera`, гомографии, высота камеры |
| `data/floor/current.json` | сессия пола: положения доски, результат |
| `data/robot/current.json` | доска на роботе: высота, смещение центра, курс |

Установка, для которой они получены: Jabra PanaCast 20, 4K, камера на высоте ≈ 1,81 м,
RMS intrinsics 1,52 px, RMS привязки пола 0,53 px по 7 положениям.
При перемещении камеры шаги 2–3 нужно повторить; при смене фокуса/зума — все три.

## Структура

```
app.py               веб-сервер FastAPI: камера, анализ, API, режимы шагов
calibration.py       доска, детекция AprilGrid, расчёт intrinsics
tag_edges.py         измерение границ маркеров, проверка кода, поиск дальних маркеров
floor_geometry.py    привязка камеры к полу (шаг 2)
floor_session.py     хранение и состояние шага 2
robot_tracking.py    поза доски/робота, калибровка центра и курса (шаг 3)
web/                 интерфейс; map.js — анимированная карта
tests/               unittest + браузерные проверки (Docker-образ tests/Dockerfile)
identify_board.py    определение семейства маркеров по фото (Dockerfile)
```

## Проверки

Образ тестов собирается поверх образа приложения, поэтому сначала `./run.sh`.

```sh
docker build --builder default -f tests/Dockerfile -t ext-cam-floor-qa .
docker run --rm -v "$PWD:/work" ext-cam-floor-qa
```

Тесты, которым нужно фото доски `image copy.png`, без него пропускаются.
