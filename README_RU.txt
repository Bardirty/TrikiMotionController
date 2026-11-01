TRIKI MOTION CONTROLLER — MODULAR EDITION

Установка (Windows):
  py -m pip install -r requirements.txt
  py TrikiMotionController.py

Структура проекта:
  TrikiMotionController.py       Точка входа
  triki_controller/app.py        Tkinter UI и координация компонентов
  triki_controller/bluetooth.py  Поиск BLE-устройств
  triki_controller/protocol.py   Разбор пакетов Triki и модель IMU-сэмпла
  triki_controller/orientation.py Фильтр ориентации и калибровка
  triki_controller/math3d.py     Векторная и quaternion-математика
  triki_controller/profiles.py   Профили, миграция и проверка очереди
  triki_controller/config_store.py Чтение/запись JSON-конфигурации
  triki_controller/constants.py  UUID, команды и общие константы

ПОИСК И ПОДКЛЮЧЕНИЕ BLUETOOTH
- Поиск и подключение вынесены в triki_controller/bluetooth.py.
- Используется тот же порядок вызовов Bleak, что и в приложенном рабочем
  монолитном файле: BleakScanner.discover(timeout=6.0), фильтр по имени,
  BleakClient(address), start_notify(TX_UUID), затем WAKE_COMMAND.
- В список попадают только устройства, чьё объявляемое имя содержит "triki"
  (без учёта регистра), как в исходном варианте. Безымянные BLE-устройства
  и устройства с другим именем намеренно отбрасываются.
- При подключении читается заряд (если характеристика доступна); при отключении
  отправляется SLEEP_COMMAND и отключается подписка уведомлений.

ПРОФИЛИ
- До 9 профилей (ID 1–9), индивидуальные оси и кнопки.
- Очереди 123, 1213 или 1,2,3; двойное нажатие переключает очередь.
- Старый JSON переносится в профили 1 и 2.
- Конфигурация сохраняется в triki_motion_config.json рядом с launcher.

Проверки:
- Исходные модули компилируются через py_compile.
- Проверены функции разбора последовательности и миграция старого конфига.
- Реальное BLE-обнаружение/соединение и ввод в игре требуют Triki и не
  могут быть подтверждены в этой среде.
