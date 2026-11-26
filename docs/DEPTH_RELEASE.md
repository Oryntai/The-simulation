# Т8.3 · Опись снимка и порядок приёмки

Снимок рабочего дерева на 26.11.2026. `git ls-files --stage` вернул код 128: Git-индекс недоступен. Опись охватывает файлы на диске, включая скрытые. Точный состав отслеживаемых файлов и diff к `d3a25a8` сверяются в отдельной копии для публикации. [Т0](DEPTH_BASELINE.md), [миграция](DEPTH_MIGRATION.md), [результаты А–Ж](DEPTH_REPORT.md).

## Состав поставки

| Путь | Содержимое и решение |
|---|---|
| `Cargo.toml`, `Cargo.lock`, `.gitignore`, `.cargo/config.toml` | Манифест 27 крейтов, зафиксированные зависимости, алиасы `sim` и `verify`. В копии для публикации убрать временный профиль и локальный лимит `jobs` из строк 6–10, сохранив алиасы. |
| `crates/` | Rust-код, тесты, манифесты и фикстуры, включая ELF/HEX, модельные flash/eFuse, сохранённые трассы QEMU и два файла SIMLOG для совместимости. |
| `catalog/`, `devices/` | Каталоги, описания сцен и сборок, схемы, платы, задачи, контрольные пары и отдельный стенд поля с собственным манифестом и lock-файлом. |
| `assets/{firmware,models,third_party}/` | Готовые прошивки, модели, MuJoCo и лицензии. Включать исходные данные с учётом ограничений в таблице ниже. |
| `firmware/`, `projects/`, `golden/` | Исходники отдельной сборки прошивок, образ стенда `projects/elbow-bench.elf`, эталонные данные. |
| `docs/`, `README.md`, `ARCHITECTURE.md`, `HUMAN_MODEL.md` | Документация, эталоны происхождения и приёмки, иллюстрации. |
| Исключить | `target/`, `firmware/target/`, временные журналы MuJoCo и QEMU, выгрузки прогонов, точки возобновления, дампы среды и локальные сборочные каталоги. Сохранённые трассы и журналы, на которые ссылаются тесты, входят как входные данные. |
| Исключить ngspice | `assets/third_party/ngspice/{bin,lib,share}/`: локальный исполняемый файл, семь модулей `.cm` и локальный `spinit`. Сохранить `COPYING` и запись происхождения. Созданную при приёмке ссылку на установленный ngspice также исключить. |

`find` и `stat`: до записи Т8.3 проверено 2 183 обычных файла (129 819 395 байт) и 6 разрешимых относительных ссылок внутри дерева. При упаковке сохранять ссылки, включая цепочки внутри `mujoco.framework` и ссылку электрического каталога `devices/07-balance-cart/electrical-parts.toml`.

| Файл больше 5 000 000 байт | Байт |
|---|---:|
| `assets/third_party/mujoco/mujoco.framework/Versions/A/libmujoco.4.0.1.dylib` | 12 132 432 |
| `assets/third_party/myo_sim/myo_sim/models/meshes/osl_knee_v2.stl` | 6 172 534 |
| `assets/third_party/myo_sim/myo_sim/models/meshes/osl_ankle_v2.stl` | 5 835 934 |

Локальный `ngspice/bin/ngspice` занимает 4 988 184 байта, вся папка ngspice — 5 782 585 байт обычных файлов. После приёмки сохранять отдельный `sim-next_*.tgz` в `~/projects/future-might-projects/sim-work/backups/`, держать последние шесть. Публикация из выделенной Git-копии в `hardware-depth` без force-push. Архив и публикация требуют выделенного владельцем пути по Т8.3. [§2 ТЗ](HARDWARE_DEPTH_SPEC.md#2-рабочая-копия-обязательное-условие), [план](DEPTH_PLAN.md#т8--сдача-и-подготовка-к-слиянию--6--4-задачи).

## Сторонние модели и лицензии

| Под `assets/third_party/` | Версия и лицензия по локальным файлам | Условие поставки |
|---|---|---|
| `mujoco/` | 4.0.1, Apache-2.0, Mach-O arm64 + x86_64 по `file` и `otool -L` | Framework со ссылками, `LICENSE` и `THIRD_PARTY_NOTICES`. |
| `ms_human_700/` | MS-Human-700, `2d686957aefd5739cf4d2859a6acfd94c8f84400`, Apache-2.0 | MJCF, сетки и `LICENSE` по описи происхождения. |
| `myo_sim/` | MyoSuite, `5cd36ee4d823bced6cbd7c02424458ed0fefb2b2`, Apache-2.0. У данных руки MoBL в описи отмечено некоммерческое ограничение. | Сохранить `LICENSE`. До коммерческой поставки подтвердить права на данные руки и состав передаваемых файлов. |
| `ngspice/` | 47 по `--version`, основная часть Modified BSD, исключения в `COPYING` | Внешний процесс. Фактически есть модули XSPICE, включая `table.cm`, хотя `PROVENANCE.md` говорит, что их исключили. Локальные бинарники целиком убрать из поставки. |

Коммиты моделей взяты из [PROVENANCE.md](../assets/third_party/PROVENANCE.md). Сверка лицензий выполнена по локальным `LICENSE`, `THIRD_PARTY_NOTICES`, `COPYING` и описаниям моделей.

## Секреты, пути, почта и хосты

`rg` и побайтовый обход Python проверили скрытые, текстовые и бинарные файлы вне двух `target/`. Проверялись имена файлов секретов, сигнатуры приватных ключей и токенов, присваивания учётных данных, URL с паролем, домашние и временные пути, почта и сетевые имена. Совпадений по ключам, токенам и присваиваниям с секретами не найдено. Для бинарников ниже приведены смещения от начала файла. В таблице пути сторонних файлов даны от `assets/third_party/`.

| Файл | Строка или смещение | Находка | Риск |
|---|---|---|---|
| `ngspice/bin/ngspice` | 4 072 548, 4 072 652 | Вшитые домашние пути к `bin` и `share`. | Да, идентифицируют владельца и дерево сборки. Исключить. |
| `ngspice/lib/ngspice/digital.cm` | 89 212 | Вшитый домашний путь к каталогу модулей. | Да. Исключить. |
| `ngspice/lib/ngspice/table.cm` | 43 886, 44 345, 44 844 | Временные пути сборки с именем учётной записи и идентификатором сеанса. | Да. Исключить. |
| `mujoco/mujoco.framework/Versions/A/Headers/SDL2/SDL_filesystem.h` | 97, 101, 105 | Примеры SDL с вымышленной учётной записью `bob` для Windows, Linux и macOS. | Нет, документация поставщика. |
| `…/Headers/SDL2/SDL_atomic.h` | 39 | `/home/` в пути публичной статьи. | Нет. |
| `mujoco/THIRD_PARTY_NOTICES` | 47, 177, 202, 909, 2554, 2567, 2591 | Публичные контакты авторов и библиотек. | Нет, атрибуция. |
| `…/Headers/SDL2/`, 86 заголовков | 3, дополнительно `SDL_bits.h:89`, `SDL_test_fuzzer.h:47`, `SDL_test_harness.h:33` | Контакты в копирайтах SDL. | Нет, атрибуция. |
| `…/Versions/A/libmujoco.4.0.1.dylib` | 5 558 740, 5 560 793, 11 182 504, 11 184 557 | Публичный адрес поддержки Qhull в строках двух архитектур. | Нет, диагностика библиотеки. |
| `ms_human_700/Body_Manipulation/Body_Waterbottle.xml` | 5 | Контакты авторов модели. | Нет, атрибуция. |
| `myo_sim/pyproject.toml` | 9–10 | Почта разработчиков пакета. | Нет, атрибуция. |
| `myo_sim/myo_sim/models/{arm,head}/assets/` | 5 в `myoarm_r_{assets,chain}.xml`, `myohead_{rigid_chain,simple_assets}.xml` | Почта в поле автора. | Нет, атрибуция. |
| `myo_sim/myo_sim/models/leg/assets/` | 5 в `myolegs_{assets,chain,muscle,tendon}.xml`, 6 в `myolegs26_{assets,chain,muscle,tendon}.xml` | То же. | Нет, атрибуция. |
| `myo_sim/myo_sim/models/legacy/{elbow,finger,torso}/` | 5 в 12 XML, 6 в `myoelbow_1dof6muscles_1dofSoftexo_{Ideal,sim2}.xml` | То же. | Нет, атрибуция. |
| `myo_sim/myo_sim/models/legacy/osl/` | 6–7 в `myolegs_osl.xml`, 5–6 в `assets/myolegs_osl_{assets,chain}.xml` | То же. | Нет, атрибуция. |
| `myo_sim/myo_sim/models/{scene,torso/assets}/` | 5 в `myosuite_scene{,_noPedestal}.xml`, 6 в `myosuite_quad.xml` и `myotorso_chain.xml`, 7 в `myotorso_assets.xml` | То же. | Нет, атрибуция. |
| `myo_sim/docs/images/models/myohands.png` | 113 566 | Email-подобная последовательность внутри сжатых байтов изображения, текстовый контакт в метаданных отсутствует. | Срабатывание шаблона. |
| `ngspice/COPYING` | 565 | Контакт в копирайте. | Нет, атрибуция. |
| `Cargo.lock`, документация и источники в коде/данных | Поля `source`, ссылки и комментарии | 167 уникальных хостов в URL, в том числе реестры, документация, поставщики, статьи, службы сертификатов и `www.example.com`. Совпадения по внутренним именам и адресам разобраны по контексту. | Хосты владельца и сетевые учётные данные при поиске не обнаружены. |

Личные абсолютные пути обнаружены в исключаемых бинарниках. В текстовых исходниках, документации и фикстурах обнаружены только нейтральные примеры. В IFD0 всех 30 JPEG из `docs/screenshots/` отсутствуют теги GPS и авторства. Просмотр миниатюр подтвердил, что это снимки симулятора. OCR завершился ошибкой Vision, мелкий текст изображений полностью не проверен. Git-история и скрытые данные произвольного формата остаются вне подтверждения.

## Инструменты и признаки неполной проверки

Пути Rust в таблице даны от `crates/`. Для отсутствующих инструментов поведение сверено по точкам поиска, запуска и проверкам результата.

| Инструмент и область | Как подключается | Без инструмента |
|---|---|---|
| Rust/Cargo, линкер macOS, Xcode CLT с libclang, MuJoCo | `sim-mujoco/build.rs` канонизирует путь framework, линкует его и генерирует FFI через bindgen. Локально Rust/Cargo 1.100.0, Darwin arm64, libclang из CLT. Разрешённый Bevy 0.20.1 декларирует Rust 1.97.1 по `cargo metadata`. | Ошибка подготовки или сборки, проверки с MuJoCo не выполняются. |
| ngspice 47 · обычная сверка и `cargo verify` | `sim-circuit/src/ngspice.rs:21` ищет файл по `NGSPICE`, затем по пути в дереве, запускает `-n -b`. | `reference_checks()` исключает 11 записей `ngspice.*`. `reference_checks::ngspice_cross_check` печатает пропуск, Cargo считает тест успешным. Приёмка требует 11 записей и сравнения общего числа с Т0. |
| ngspice · обязательные внешние сравнения | `sim-circuit` — `cosim`, `emc_power`, `emc_cable`, `emc_coupling`.<br>`sim-process` — `capacitor_stress`, `esd_event`.<br>`sim-cli` — модульные тесты `electrical/input.rs` и цели `electrical_corpus`, `electrical_depth`, `depth_t6`.<br>`sim-scenarios` — `circuit_pcb`. | Обязательные тесты падают по отсутствию файла, ошибке запуска или отсутствию сравнения. `emc_cable`, `emc_coupling`, `electrical_depth` и лаборатория `sim-scenarios/src/circuit_lab.rs:17` требуют фиксированного пути. Лаборатория запускает `-b`. Адаптер ESD на ошибке сверки возвращает `NotAnalyzed`, схемный адаптер `compare_ngspice` возвращает ошибку. |
| openEMS 0.37 + CSXCAD · ЭМС | `sim-circuit/src/emc/openems.rs:500` принимает путь к процессу. Реальные тесты `sim-circuit/tests/openems.rs:313` и `devices/electrical/depth_t6/loop/reference.rs:722` берут `OPENEMS` или `~/opt/openEMS/bin/openEMS`. Вход XML формирует Rust. | Адаптер возвращает `NotAnalyzed` при недоступном файле или неполной сверке. Реальные тесты `#[ignore]` по умолчанию, при принудительном запуске без решателя падают. Аналитика и тесты адаптера с подставным процессом доступны. |
| ESP-IDF v6.1.1, RISC-V GCC, Python/esptool, CMake/Ninja, arduino-cli 1.6.0 с ядром ESP32 4.0.0, QEMU Espressif 9.2.2 RISC-V | В `sim-boards/firmware-tests/esp_sdk/capture.sh` и `esp_peripherals/reproduce.sh` ищется `export.sh` через `SIM_ESP_IDF_PATH` / `ESP_PERIPHERALS_IDF_PATH` или `~/esp/esp-idf`. Python проверяет версии IDF/QEMU и ядро Arduino для Blink. `esp_image` отдельно вызывает `esptool` и генератор таблицы IDF из `IDF_PATH`. | Обычный `esp_sdk_trace` проверяет сохранённые трассы. Пересборка помечена `#[ignore]`. Отсутствующий SDK даёт `NotAnalyzed` и код 2 из shell, недостающие программы дают ошибку. Принудительный Cargo-тест проверяет код процесса и побайтовое совпадение. |
| ROM и входы QEMU C3 · BoardSim | `sim-boards/tests/{esp_boot,esp_sdk_boot,esp_sdk_acceptance,esp_sdk_io,esp_irq,esp_sleep,esp_flash}.rs` и тест в `src/riscv.rs`. Часть опытов читает файлы на диске из `SIM_ESP_BOOT_*`, `SIM_ESP_ACCEPTANCE_ROM`, `ESP_SLEEP_QEMU_ROM`, `SIM_T25_FLASH` и сохранённые фикстуры. | Внешние опыты помечены `#[ignore]`. При форсировании отсутствующие входы дают ошибку. Проверки BoardSim с сохранёнными входами не требуют живого процесса QEMU. |
| QEMU Espressif Xtensa · S3 | `sim-runtime/src/control/external_board.rs:222` запускает QEMU из профиля, `SIM_EXTERNAL_BOARD_QEMU` задаёт путь в `sim-runtime/tests/external_board.rs` и `sim-cli/tests/mcu_catalog.rs`. Flash-опыт требует также `SIM_EXTERNAL_BOARD_FLASH` и `SIM_EXTERNAL_BOARD_EFUSE`. | Неудачный старт даёт `Stop::Unavailable`, повторяемость `NotAnalyzed`. Живые тесты `#[ignore]`, при форсировании без входов падают. Тест с отсутствующим QEMU проверяет открытый исход. |
| KiCad и Freerouting | `sim-cli/src/electrical/pcb.rs:443` читает `.kicad_pcb` через Rust-парсер `sim-circuit/src/pcb/kicad.rs`, версии `20240108` и `20241229`. KiCad нужен для редактирования или пересчёта устаревшей заливки. | `pcb_*`, `depth_t4`, `audit_t4` используют файлы. Некорректный или неподдерживаемый вход даёт ошибку. Автотрассировка и вызов Freerouting отсутствуют. |
| Пересборка ELF · Rust-цели и Python 3 | `firmware/build.sh` и `firmware/.cargo/config.toml` — `riscv32imc-unknown-none-elf` и `rust-lld`.<br>`sim-boards/firmware-tests/build.sh` — также `thumbv6m-none-eabi`, `thumbv7m-none-eabi`, генератор AVR на Python.<br>`firmware/bench-enable/build.sh` — `thumbv6m-none-eabi`. | Хостовые тесты читают готовые образы. Отсутствующая цель или Python останавливают соответствующую пересборку. Shell и системные утилиты нужны для тестов дочерних процессов. |

## Чистая сборка и проверка заказчиком

В новом каталоге копии сохранить содержимое framework и ссылки. Предварительно нужен заполненный локальный кэш Cargo для lock-файлов и целевой архитектуры. На arm64 маке обе команды `cargo fetch --offline` ниже вернули код 0 с имеющимся кэшем. Чистая сборка с пустым кэшем требует отдельно подготовить зависимости. Оставить штатный `sim-next/target` и сверить лимиты диска по §2.

```sh
cargo fetch --offline --locked --target aarch64-apple-darwin
cargo fetch --offline --locked --target aarch64-apple-darwin --manifest-path devices/electrical/depth_t6/loop/Cargo.toml
# NGSPICE должен указывать на отдельно установленный исполняемый файл.
export NGSPICE="${NGSPICE:-$(command -v ngspice)}"
test -x "$NGSPICE"
mkdir -p assets/third_party/ngspice/bin
ln -s "$NGSPICE" assets/third_party/ngspice/bin/ngspice
# По очереди, после завершения предыдущей команды.
cargo test --offline -p sim-verify --test platform_boundary
cargo test --offline --workspace
cargo verify --offline
```

Если ngspice вне `PATH`, заранее задать его абсолютным путём в `NGSPICE`. При ошибке шага остановиться и проверить вход. Ссылка нужна в чистой копии для тестов с фиксированным путём. Успех `cargo verify` сверить с числом и именами проверок. У Т0 зафиксировано 1 192 прошедших теста, 2 пропуска и 430/430 сверок. Новые проверки и пропуски сверить отдельно по блоку. [Т0](DEPTH_BASELINE.md), [§6 ТЗ](HARDWARE_DEPTH_SPEC.md#6-как-сдавать).

| Отдельный запуск владельца после штатной регрессии | Условие |
|---|---|
| `cargo test --offline -p sim-circuit --test openems -- --ignored --nocapture` | Указать `OPENEMS` и обеспечить доступность библиотек решателя. |
| `cargo test --offline --locked --manifest-path devices/electrical/depth_t6/loop/Cargo.toml --target-dir target -- --include-ignored --nocapture --test-threads=1` | Сравнение трёх канонических структур и уплотнённой частотной сетки. |
| `cargo test --offline -p sim-boards --test esp_sdk_trace -- --ignored --nocapture --test-threads=1`<br>`bash crates/sim-boards/firmware-tests/esp_peripherals/reproduce.sh` | Подготовить SDK, Python, QEMU RISC-V и Arduino. Запуск `reproduce.sh` без `--write` сверяет сохранённые артефакты. [ESP SDK](../crates/sim-boards/firmware-tests/esp_sdk/SOURCES.md), [периферия](../crates/sim-boards/firmware-tests/esp_peripherals/README.md). |
| `cargo test --offline -p sim-boards --test esp_sdk_acceptance -- --include-ignored --nocapture --test-threads=1`<br>`cargo test --offline -p sim-runtime --test external_board qemu_uart0_rom -- --ignored --nocapture --test-threads=1` | Для первого дать ROM C3 в `SIM_ESP_ACCEPTANCE_ROM`. Для второго дать QEMU Xtensa в `SIM_EXTERNAL_BOARD_QEMU`. Остальные игнорируемые опыты запускать адресно с входами из таблицы инструментов. |

## Проверки Т8.3 и открытые пункты

- `cargo test --offline -p sim-verify --test platform_boundary` — `test result: ok. 4 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.19s`.
- `cargo metadata --offline --locked --format-version 1 --no-deps`: 27 членов workspace, 1 цель `verify-all`, путь сборки `sim-next/target`. `cargo --list` подтвердил алиас. `shasum -a 256 -c SHA256SUMS` подтвердил 9/9 файлов в двух наборах `esp_peripherals` и `esp_sdk_acceptance`.
- Приёмка workspace, `verify-all`, поле и штатные прогоны прошивок в Т8.3 не выполнялись. Холодная сборка из опубликованного дерева и сравнение с Т0 относятся к сдаче владельцем. Диагностический запуск, модельная сверка и пропуск теста сами по себе не закрывают обязательное физическое доказательство.
- Для переносимой поставки нужно согласовать общее обнаружение ngspice в `sim-circuit`, `sim-cli`, `sim-scenarios`, явный счётчик пропусков в `verify-all` и поправку описи XSPICE в `assets/third_party/PROVENANCE.md`. До этого для тестов с фиксированным путём предусмотрена локальная ссылка. [Бэклог](DEPTH_BACKLOG.md#сквозные-находки-и-регрессия).
