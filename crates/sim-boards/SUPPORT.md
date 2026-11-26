# sim-boards

`board_spec` возвращает электрические пределы микроконтроллера, `supply_range` — пределы выбранного входа платы. `board_pins` и `pin_spec` используют номера GP на Pico, GPIO на ESP32-C3, PA0–PA15 как 0–15, PB0–PB15 как 16–31, PC13–PC15 как 45–47 и BOOT0 как 64 у Blue Pill. У Arduino это D0–D13 и A0–A7 как 14–21. A6/A7 относятся к Nano. Для ESP32-C3 GPIO5 соответствует физическому ADC2_CH0. В каталоге его номер 5, канал ADC2 в эмуляторе не реализован.

`check_wiring` принимает описания отдельных цепей. Одно значение `net` связывает активные двухтактные выходы одной цепи. Для LED задаются минимальное прямое падение и максимальное напряжение источника, ток вычисляется по закону Ома. Для других нагрузок задаётся пиковый ток в миллиамперах. `wiring_outcome` возвращает `Fail` при наличии хотя бы одного нарушения, затем учитывает `NotAnalyzed`. `Pass` относится к представленным и проверенным ограничениям. Типовой ток ESP32-C3 при максимальной силе драйвера в каталоге указан отдельно от гарантированного. Для Pico при неизвестной настройке DRIVE используется нижняя паспортная ступень. Для Uno/Nano проверяются отдельные группы выводов по даташиту.

Динамика не включает тепловой режим, ЭМС, пусковые и обратные токи за драйвером, электрический шум, импеданс источника АЦП, порядок включения и взаимодействие нескольких источников питания. Для этих свойств нужны данные схемы и отдельная проверка. Поддержка UART, I²C, SPI или PWM в каталоге относится к распиновке чипа.

## Исполнение прошивки

`load` принимает ELF32 little-endian с исполняемыми сегментами `PT_LOAD`, а для AVR также Intel HEX. Старт ARM идёт через таблицу векторов, AVR — с адреса 0, RISC-V — через точку входа ELF. У Pico ищутся таблицы в начале XIP и после стандартного отступа boot2. В этом режиме образы запускаются в фиксированных профилях тактирования. Загрузка начинается со входа образа или таблицы векторов. У ESP32-C3 для старта из ROM применяется отдельный вход raw flash ниже. `load` применяет входы хоста и заменяет образ. При ошибке загрузки предыдущий образ недоступен. `reset` повторно выполняет старт CPU и возвращает периферию в исходное состояние профиля, сохраняя подключения. Прежние аппаратные профили ELF/HEX сохраняют SRAM. VMCU-C3 восстанавливает ОЗУ по правилам `Mcu::reset`.

`set_adc_volts` ожидает вольты. `run_for` получает секунды. Для конечного нулевого интервала такты не начисляются. Дробные такты переносятся в следующий вызов. После ошибки исполнения `pin_output` и `pwm_duty` возвращают `None`. В штатном режиме `None` означает отсутствие активного драйвера или ШИМ на выводе. `uart_take` потребляет переданные к моменту чтения байты. Их можно читать для диагностики после ошибки.

### Инструкции

- **RP2040.** 16-битные Thumb ADD/ADC/SUB/SBC/RSB, CMP/CMN/TST, MOV/MVN, AND/ORR/EOR/BIC, LSL/LSR/ASR/ROR, MUL, SXTB/SXTH/UXTB/UXTH, REV/REV16/REVSH, ADR, LDR/STR с байтовыми, полусловными и знаковыми загрузками, LDM/STM, PUSH/POP. Переходы B/Bcc/BX/BL/BLX-регистр, CPS, SVC из Thread, BKPT, NOP/YIELD/WFI, DSB/DMB/ISB, часть MRS/MSR. Деление на Cortex-M0+ выполняет скомпилированная программная процедура. WFE/SEV, MPU, PSP, выключение ядра и второе ядро не моделируются.
- **STM32F103C8.** Перечисленная основа Thumb, IT, CBZ/CBNZ, B.W и Bcc.W, MOVW/MOVT, ADR.W, ADDW/SUBW, выборка простых 32-битных операций с модифицированной константой и сдвинутым регистром, LDR/STR.W, LDRD/STRD, MUL/MLA, SDIV/UDIV. Охвачены MSP, PRIMASK, вложенные прерывания с разными приоритетами и EXC_RETURN на MSP. Остальные кодировки, включая LDREX/STREX, TBB/TBH, широкие LDM/STM, битовые поля, насыщение, PSP, BASEPRI/FAULTMASK, MPU и обработку аппаратных BusFault/UsageFault, дают ошибку или недоступны.
- **ATmega328P.** LDI, MOV/MOVW, ADD/ADC, SUB/SUBI/SBC/SBCI, ADIW/SBIW, MUL/MULS, AND/ANDI/OR/ORI/EOR, COM/NEG/INC/DEC, ASR/LSR/ROR/SWAP, CP/CPI/CPC/CPSE, BSET/BCLR/BST/BLD, SBIC/SBIS/SBRC/SBRS, SBI/CBI, IN/OUT. Поддержаны LDS/STS, LD/ST через X/Y/Z, LDD/STD, LPM, PUSH/POP, JMP/RJMP/IJMP, CALL/RCALL/ICALL, BRBS/BRBC, RET/RETI, WDR, NOP и BREAK. Деление программное. SPM, SLEEP и дробные умножения не реализованы.
- **ESP32-C3.** Декодер RV32I/M/C из `sim-mcu`. Реализованы целочисленные и сжатые операции, переходы, загрузка и запись, MUL/MULH/MULHSU/MULHU/DIV/DIVU/REM/REMU, FENCE/FENCE.I, ECALL, EBREAK, WFI, MRET и операции CSR. Поддержаны `mstatus`, `misa`, `mtvec`, `mscratch`, `mepc`, `mcause`, чтение `mtval`, `mhartid`, счётчиков `cycle`/`instret` и их старших половин. Исполняется M-mode. Для ELF набор CPU ограничен указанными режимами. В raw flash перечисленные в `esp_rom.rs` входы ROM переходят в предоставленный дамп. Остальные входы дают явный отказ.

### Карта периферии

Регистры перечислены по именам производителя. Поддерживаются указанные режимы. Другие адреса, размеры доступа, кодировки и неподдержанные настройки дают диагностируемый `BoardFault`.

| Плата | Блоки и реализованные регистры | Границы |
|---|---|---|
| Pico | SIO `0xd0000000` CPUID, GPIO_IN, OUT/OE и SET/CLR/XOR. IO_BANK0 `0x40014000` GPIO_CTRL, PADS_BANK0 `0x4001c000` PAD_GPIO. RESETS `0x4000c000` RESET/DONE. TIMER `0x40054000` TIME*, ALARM0–3, ARMED, PAUSE, INTR/INTE/INTF/INTS. PWM `0x40050000` CSR/DIV/CTR/CC/TOP восьми срезов, EN и IRQ. ADC `0x4004c000` CS/RESULT. UART0/1 `0x40034000`/`0x40038000` DR/FR, IBRD/FBRD, LCR_H/CR. PSM `0x40010000` WDSEL, WATCHDOG `0x40058000` CTRL/LOAD/REASON/SCRATCH/TICK. | Однократный ADC0–3. PWM со свободным счётом вверх. UART TX 8N1. Стандартный профиль WDSEL и выборочные APB-псевдонимы. Перезапуск блоков RESETS, CLOCKS/PLL, PIO, QSPI-контроллер, DMA, USB, SPI/I²C недоступны. |
| Blue Pill | RCC `0x40021000` CR/CFGR/APB1ENR/APB2ENR/CSR. GPIOA/B/C `0x40010800`/`0x40010c00`/`0x40011000` CRL/CRH/IDR/ODR/BSRR/BRR. TIM1 `0x40012c00` и TIM2/3/4 `0x40000000`/`0x40000400`/`0x40000800` CR1/DIER/SR/EGR/CCMR1–2/CCER/CNT/PSC/ARR/CCR1–4, базовые TIM1 RCR/BDTR. ADC1 `0x40012400` SR/CR1/CR2/SMPR1–2/SQR1–3/DR. USART1/2/3 `0x40013800`/`0x40004400`/`0x40004800` SR/DR/BRR/CR1–3. IWDG `0x40003000` KR/PR/RLR/SR. | HSI 8 МГц, штатные назначения выводов, таймеры со счётом вверх, PWM1/2. ADC1 CH0–9 в одиночном режиме. TX 8N1. PLL, AFIO remap/EXTI, ADC2, DMA, захват, комплементарный PWM, dead time, WWDG, USB/CAN, SPI/I²C недоступны. |
| Cortex-M | SCS `0xe000e000`: SysTick CTRL/LOAD/VAL, NVIC ISER/ICER/ISPR/ICPR/IPR, IABR на STM; SCB CPUID/ICSR/VTOR/AIRCR/SHPR. | SysTick целочисленный от CPU или HCLK/8 в профиле STM. Фиксированная группировка приоритетов. CALIB, отладочные блоки и большинство SCB не моделируются. |
| Uno/Nano | Байтовые адреса данных: PINx/DDRx/PORTx `0x23–0x2b`, TIFR0–2 `0x35–0x37`, TIMSK0–2 `0x6e–0x70`, TCCR/TCNT/OCR таймеров 0/1/2 `0x44–0x48`/`0x80–0x8b`/`0xb0–0xb4` (Timer1 также ICR1), MCUSR `0x54`, WDTCSR `0x60`, ADCL/ADCH/ADCSRA/ADCSRB/ADMUX `0x78–0x7c`, USART0 `0xc0–0xc6` (UCSR0A/B/C, UBRR0, UDR0). Ядро обрабатывает SREG/SPL/SPH. | Normal/CTC и выборочные Fast/Phase Correct PWM. IRQ сравнения и переполнения. АЦП с AVCC или номинальной внутренней опорой. TX 8N1. Асинхронный Timer2, внешнее тактирование, захват ICP, GPIO IRQ, EEPROM, SPI/I²C не реализованы. |
| ESP32-C3 | GPIO `0x60004000` OUT/OUT_W1TS/W1TC, ENABLE/ENABLE_W1TS/W1TC, IN, PINn и FUNCn_OUT_SEL_CFG. IO_MUX `0x60009000` GPIO0–21. TIMG0/1 `0x6001f000`/`0x60020000` T0CONFIG/LO/HI/UPDATE/ALARM/LOAD, INT_ENA/RAW/ST/CLR, WDTCONFIG0–2/FEED/WPROTECT. Матрица IRQ `0x600c2000` TIMER GROUP MAP, CPU_INT_ENABLE/TYPE/EIP_STATUS/PRI/THRESH. LEDC `0x60019000` CONF, CONF0/1/HPOINT/DUTY/DUTY_R каналов 0–5 и CONF/VALUE таймеров 0–3. ADC `0x60040000` ONETIME_SAMPLE, ADC1_DATA_STATUS, INT_RAW/CLR. UART0/1 `0x60000000`/`0x60010000` FIFO/INT_RAW/INT_ST/INT_ENA/INT_CLR/CLKDIV/STATUS/CONF0/CONF1/FSM_STATUS/CLK_CONF. I²C0 `0x60013000`, SPI2 `0x60024000`. | APB/CPU 80 МГц. IRQ таймеров по уровню. MWDT со сбросом системы на первой стадии. LEDC с APB и постоянной скважностью. ADC1 CH0–4 и программный запуск. UART 8N1. Режимы шин приведены ниже. SYSTIMER, RTC WDT, DMA, ADC2, Wi-Fi/BLE, USB и изменение системных часов не реализованы. |

## Каталог и программный стык

`BoardCatalog::builtin` читает `catalog/boards/catalog.toml`. `BoardCatalog::parse` принимает строгую схему `[[board]]` с обязательными строками `id`, `profile`, `name` и `source` без escape-последовательностей. Ключ позиции состоит из строчных ASCII-букв, цифр и дефисов. Несколько позиций могут указывать на один профиль. Неизвестные поля, профили и повтор ключей отклоняются. `BoardCatalog::create` вызывает общую фабрику `new_board(BoardId)`. В прежнем конструкторе физических плат виртуальная позиция скрыта, поскольку у неё отсутствуют паспортные электрические пределы.

1. Загрузить образ и при необходимости записать байты через `write_config` и вызвать `reset` до старта.
2. Применять `InputEvent` в текущем такте `cycles()`. Планировщик преобразует время по `capabilities().cpu_hz`, держит будущие доставки у себя и проверяет `InputOutcome`.
3. Продвигать `run_cycles` до ближайшей доставки. При `bus_request_pending` забрать `take_events`, обработать запрос по номеру и продолжить на остаток интервала. При ошибке `take_events` возвращает успевшие произойти события. Ответ I²C нужен строго до `address_ack_cycle`, ответ SPI строго до `complete_cycle`.
4. Снимать цифровой уровень и ШИМ через `sample(OutputPort)`. Для ШИМ возвращаются скважность, период и длительность высокого уровня в секундах. `None` означает недоступный выход.
5. `clone_state` копирует память, CPU, дробные фазы, настройки, запросы и ответы в полёте и накопленные выходы. Последующие входы применяются к каждой копии отдельно.

`take_events` и `uart_take` потребляют одну очередь переданных UART-байтов. Прежний `run_for` проходит интервал целиком, шины при этом не получают ответы от хоста. Обесточивание отключает выходы и отменяет незавершённые обмены. Такты обесточенной платы можно продвигать целочисленным методом. Возврат питания загружает состояние холодного старта. При `Execution { enabled: false }` ядро стоит, аппаратные таймеры и сторож продолжаются. Чтение АЦП без заданного напряжения даёт ошибку. Если схема задаёт уровень при обрыве, его подают в вольтах обычным входом.

## Матрица регистровых стендов ELF/HEX

| Профиль | CPU · АЦП · PWM | UART | I²C | SPI | Пакеты |
|---|---|---|---|---|---|
| VMCU-C3 | 160 МГц, ADC0–7, 12 бит, 3.3 В, PWM0–3 с отдельным номером канала | UART0, RX FIFO 256, TX буфер 1 МиБ, мгновенная передача по ABI, IRQ уровня RX | — | — | RADIO0, до 251 байта, по 8 пакетов RX/TX, входной RSSI |
| ESP32-C3 | 80 МГц, ADC1 CH0–4, LEDC с APB | UART0/1, 8N1, XTAL 40 МГц или PLL80, целый делитель источника и UART 12.4. RX FIFO 128, до 128 кадров на линии, IRQ порога и переполнения. 128 байтов TX в очереди и активный кадр | Ведущий I²C0, 7-битный адрес, MSB, XTAL 40 МГц с целым делителем, FIFO 32 на направление | Ведущий SPI2, 1–64 байта, полный дуплекс, один активный низкий CS из 0–5, режимы 0–3, отдельный порядок битов RX/TX, XTAL/PLL80 | — |
| RP2040 | 125 МГц, ADC0–3, PWM со счётом вверх | UART0/1, прежний TX 8N1 | — | — | — |
| STM32F103C8 | 8 МГц, ADC1 CH0–9, PWM1/2 со счётом вверх | USART1/2/3 как порты 0/1/2, прежний TX 8N1 | — | — | — |
| ATmega328P | 16 МГц, ADC0–7, прежние Fast/Phase Correct PWM | USART0, прежний TX 8N1 | — | — | — |

- **VMCU-C3.** Карта памяти, конфигурация до 4096 байт, холодный старт и сторож описаны в [карте VMCU-C3](../sim-mcu/MEMORY_MAP.md). Данные АЦП, GPIO и конфигурация сохраняются при сбросе. UART и RADIO получают отметки записи MMIO. Начатая инструкция может закончиться за границей интервала хоста. Её остаток учитывается в следующем интервале. Тип канала BLE или Wi-Fi, задержки и потери задаются моделью связи снаружи.
- **UART ESP32-C3.** Входной 8N1-кадр занимает 10 битовых интервалов. Окончание округляется вверх до такта CPU. `RXFIFO_FULL` срабатывает при количестве байтов выше порога, `RXFIFO_OVF` при переполнении FIFO. Избыток кадров в очереди хоста возвращается в `InputOutcome`. Flow control, тайм-аут RX и ошибки кадра не поддерживаются.
- **I²C ESP32-C3.** До восьми команд `COMD0..7`. START, WRITE с проверкой ACK, необязательный повторный START к тому же адресу, READ с ACK до последнего байта, READ последнего байта с NACK и STOP. Включены `CTR`, `SR`, настройка периода и START/STOP, FIFO reset/data и IRQ завершения/NACK. Первый NACK завершает транзакцию после адресного кадра и STOP. Задержка считается по девяти тактам SCL на байт и длительностям START/STOP. Ведомый режим, 10-битный адрес, NACK посреди данных, END без STOP, clock stretching, арбитраж и фильтрация фронтов не моделируются.
- **SPI2 ESP32-C3.** `CMD/CTRL/CLOCK/USER/USER1/MS_DLEN/MISC/CLK_GATE`, буфер `W0..W15` и IRQ `TRANS_DONE`. `CMD.USR` удерживается до завершения. Для разделённого SCLK поддержаны чётный делитель и равные high/low. Время включает интервал между фронтами SCLK и задержки CS по TRM §27.6.1. Отсутствие подтверждённого ответа даёт `NoResponse` и ошибку. Flash SPI, ведомый, half-duplex, command/address/dummy, DMA, dual/quad и удержание CS между обменами не поддерживаются.
- Входы UART RX, I²C и SPI подключаются на уровне логических портов. Электрические уровни, GPIO-матрица входов шин, фронты, подтяжки и работа полных драйверов ESP-IDF требуют отдельного стенда.

Карта регистров и время сверены с первоисточниками. [ESP32-C3 TRM, главы 26–28](https://documentation.espressif.com/esp32-c3_technical_reference_manual_en.pdf) · [UART registers](https://raw.githubusercontent.com/espressif/esp-idf/v5.4.2/components/soc/esp32c3/register/soc/uart_reg.h) · [I²C HAL](https://raw.githubusercontent.com/espressif/esp-idf/v5.4.2/components/hal/esp32c3/include/hal/i2c_ll.h) · [SPI HAL](https://raw.githubusercontent.com/espressif/esp-idf/v5.4.2/components/hal/esp32c3/include/hal/spi_ll.h)

## Проверка и воспроизведение

`cargo test --offline -p sim-boards` загружает готовые образы из `testdata/`. Исходники ARM/RISC-V находятся в `firmware-tests/src/main.rs`. AVR-программа вручную закодирована в `firmware-tests/avr_image.py` по DS40002198C. Скрипт выпускает ELF и Intel HEX из одной последовательности машинных слов. Присутствие AVR-тулчейна не требуется.

`sh crates/sim-boards/firmware-tests/build.sh` пересобирает образы на установленных целях и удаляет временный каталог Cargo. Прошивки стенда используют явные адреса MMIO. Для цифрового входа стенд задаёт высокий уровень, отдельный отрицательный тест задаёт низкий. Сверки проверяют время между фронтами GPIO, долю высоких отсчётов реального ШИМ-вывода, байты UART с кодом ADC, интервал до повторного запуска сторожем и доставку прерываний. На STM дополнительно проверяется вытеснение обработчика TIM2 исключением SysTick. Эталонные периоды и тайм-ауты рассчитываются из настроек прошивок и частот документации.

Циклы ядра ARM и RISC-V оценочные. Времена ожидания шины, джиттер, режимы сна и переходные процессы при включении не сверены с чипами. Тайм-ауты AVR WDT и STM IWDG используют типовые частоты внутренних генераторов. АЦП имеет идеальную передаточную функцию. Для ESP32-C3 дополнительно оценены raw-код и время преобразования, индивидуальная калибровка отсутствует. Для входа без внешнего сигнала и подтяжки цифровая модель использует низкий уровень. Стенд запускался в эмуляторе. Проверка на физических платах не проводилась.

`sh crates/sim-boards/firmware-tests/build_ports.sh` воспроизводит три ELF регистровых стендов. Тест `ports` проверяет времена линии ручным расчётом, результаты прямого `Mcu`, отрицательные ответы, несогласованные входы и копирование транзакции. Все профили публикуют `blocks_hardware_proof`.

## Приёмка Т2.23 · штатный raw flash ESP32-C3

`load_esp_flash_with_profile` принимает объединённый flash, смещение таблицы разделов, ROM, слова eFuse, уровни strap, фазу SYSTIMER и ответы внешней flash. Старт идёт из ROM. `run_until_quarter_ns` ведёт абсолютную шкалу при смене частоты. `take_events` отдаёт кадры, запросы, события шины и сброса. `esp_boot_position` показывает PC первой непройденной инструкции даже после остановки. `esp_reset_value_uses` и `esp_virtual_rng_use` показывают фактическое применение профиля и виртуального RNG. Старые ELF и Intel HEX остаются на входе `load`.

### Фактические опыты

| Программа и контур | BoardSim и независимая сверка | Открыто |
|---|---|---|
| ESP-IDF v6.1.1, `hello_world` | PC `0x42006bda` на шаге 23 126 662 от единицы. UART0 передал `Hello world!\r\n`, строка зафиксирована к опросу на 233 мс. QEMU 9.2.2 исполнил тот же PC (`-d in_asm`) и выдал те же байты. Повреждённая таблица отсекается в BoardSim и в загрузчике QEMU. Для eFuse `SPI_BOOT_CRYPT_CNT=0b001` BoardSim отклоняет шифрованную загрузку, `espefuse --virt` подтверждает значение Enable. Без профиля первое чтение `UART_STATUS_REG` даёт `UnsupportedRegister` (`0x6000001c`, PC `0x4004c0d4`, инструкция №6196). С профилем одних линий UART следующая остановка на `ANA_CONFIG_REG` по `0x6000e044` при заданном readback SPI flash. | Абсолютное время холодного старта на плате `NotAnalyzed`. TRM §6.2.4.1 задаёт XTAL/2, QEMU стартует от PLL. |
| Arduino ESP32 4.0.0, Blink на GPIO3 | Четыре логических уровня 1/0/1/0. Между фронтами 1000/1005/1000 тиков FreeRTOS при 1000 Гц. Добавка 5 тиков соответствует `yieldIfNecessary` в `cores/esp32/main.cpp`. QEMU подтвердил записи W1TS/W1TC и интервалы SYSTIMER 999 925/1 005 000/1 000 000 мкс. Допуск от `vTaskDelay` и сравнения временных шкал — 1 мс на фазу каждого прогона и 1 мкс на квант метки. Измерение вывода добавляет шаг опроса 1 мс и интервал между двумя отметками вокруг `digitalWrite`. | Драйвер вывода, электрические фронты и уровень на кремнии `NotAnalyzed`. GPIO в QEMU наблюдался на MMIO. |
| 2.21, GPIO3 → GPIO10 | Вход повторил 0/1/0. Высокий уровень 20,000 мс, `vTaskDelay(2)` при 100 Гц. Допуск — один тик и 5 мкс опроса. QEMU подтвердил последовательность записей, его GPIO10 читался как 0. | Электрическая петля и пороги входа `NotAnalyzed`. |
| 2.21, UART1 | BoardSim и QEMU передали и приняли `55 a6 00 ff`. ESP-IDF обработал приём через IRQ, в BoardSim неполный порог ведёт к IRQ тайм-аута RX. TRM гл. 26 и `uart_ll_set_baudrate`: 8N1, XTAL 40 МГц, делитель 5555/16, 86 796,875 нс на кадр. Интервалы 86 793,750 / 86 800,000 / 86 793,750 нс, допуск 1 такт CPU (6,25 нс). QEMU подтвердил байты и делитель. Парный тест `uart_raw_flash_уровневый_irq_входит_в_вектор_только_с_маской` различает поднятый статус и вход в trap при маске CPU. | Такты входа IRQ именно 2.21, проводные фронты и ошибки уровня `NotAnalyzed`. |
| 2.21, LEDC на GPIO5 | Штатные `ledc_set_duty` и `ledc_update_duty` меняют заполнение с 256/1024 на 768/1024. При XTAL 40 МГц и делителе 2500/256 период составляет 250 мкс, высокий уровень 62,5 и 187,5 мкс. Формулы сверены по 143 и 159 фронтам с допуском 5 мкс плюс 1 такт CPU. Гость прочитал 4000 Гц и код 768, затем выход остался в нуле. | QEMU подтвердил записи `DUTY=0x1000 → 0x3000`, в нём чтение применённого кода дало 0. Фронты в QEMU и на физическом выводе `NotAnalyzed`. |
| 2.21, I²C0 | По `i2c_ll_master_cal_bus_clk` и TRM гл. 28 длительность SCL 10 мкс, START 5 мкс, девять тактов на кадр. Адресный ACK/NACK в 95 мкс. Положительная проба завершилась в 105 мкс, комбинированная запись `10` / чтение `5a a5` с repeated START — в 475 мкс. Гость различил `ESP_OK` и `ESP_ERR_NOT_FOUND`. Снимок без ответа дал только `NoResponse` через 26,2144 мс после ACK-слота (2²⁰ / 40 МГц), поздний ответ отвергнут. Сравнение на целых тактах CPU без процентного допуска. | QEMU C3 не моделирует I²C. Там видны только команды FIFO `a0/a2` и тайм-аут. Аналитика подтверждает пакетную модель, времена ACK/RSTART/STOP на реальных выводах `NotAnalyzed`. |
| 2.21, deep sleep | Гость запрашивает таймер 2 000 000 мкс. Без профиля CRC BoardSim явно останавливается на записи START в `0x600c0048`, PC `0x4201764e`. QEMU в трёх опытах взвёл будильник `0x42659`, дважды прочитал 0 после CRC_START и остался на ветке `0x42017662` до `RTC_CNTL_SLEEP_EN`. Стенд регистров проверил поля и права по TRM 16.11–16.12. С явным профилем QEMU он воспроизвёл оба нулевых чтения и отказал при запросе результата или попытке deep sleep до `CRC_FINISH`. | Аппаратные алгоритм, порядок данных и длительность расчёта CRC отсутствуют в TRM. Штатный вход, оба сквозных пробуждения и проверка wake stub остаются `NotAnalyzed`. |
| Регистровый RTC, `esp_sleep` | Таймер проверен на тике 272 000 (2 с при 136 кГц), GPIO на тике 272 013 после подачи уровня на тике 272 010. Синтетический ROM подтверждает два цифровых пробуждения, удержание RTC FAST, отсутствие пробуждения до события и при коротком импульсе. | Разброс RC_SLOW, задержка аналоговых доменов и штатный wake stub `NotAnalyzed`. |
| Прежние ELF/Intel HEX | `testdata/esp.elf` останавливается по EBREAK `0x403801d2` за 75 тактов. Оба AVR-образа дают UART `02 00` и duty 0,25. Тесты `boards` и `ports` сверяют остальные старые образы. | Аппаратное время ядра, электрические уровни и погрешность платы `NotAnalyzed`. |

В Т2.23 UART ведётся по байтам после целого кадра, а GPIO и LEDC — по дискретным уровням. АЦП и его калибровка, DMA/SPI2, RF и потребление не участвовали в опытах штатных образов. Для этих контуров сквозной результат `NotAnalyzed`. `blocks_hardware_proof` сохраняется у всех опытов.

### Профиль QEMU

Таблица перечисляет 22 записи базового профиля и 9 записей дополнения I/O. У каждого поля `kind = "qemu"` и ссылка на конкретный исходник QEMU. Наличие поля в профиле означает доступность подстановки. Фактическое использование возвращает `esp_reset_value_uses()`. Для переноса на кремний нужны данные той же ревизии и трассы платы. [Базовый профиль](firmware-tests/esp_sdk/reference/esp32c3-qemu-reset.toml), [дополнение I/O](tests/data/esp_sdk_acceptance/io-reset.toml).

| Адрес | Поля и значения |
|---|---|
| `0x6000e044`, `0x6000e048` | `ANA_CONFIG_REG` и `ANA_CONFIG2_REG` = `0x00ffffff`. |
| `0x6000e040` | `BBPLL_STOP_FORCE_HIGH/LOW` = `0x0c`. |
| `0x6000e000` | `I2C_MST0_COMMAND` = `0x00ffffff`. Байты `BBPLL.5`, `BBPLL.9`, `SARADC.7`, `DIG_REG.4/.5/.6/.7/.13`, `ULP_BOD.0/.3/.5` = `0xff`. |
| `0x6000001c` (UART0, базовый профиль), `0x6001001c` (UART1, дополнение I/O) | `UART_STATUS_LINES` = 0 по трассе ROM UART0 и общему обработчику обоих портов. Профиль задаёт уровни TXD, RTSN, DTRN, RXD, CTSN и DSRN, счётчики FIFO вычисляет модель. [QEMU 9.2.2, `hw/char/esp32_uart.c`, строки 68–87](https://github.com/espressif/qemu/blob/esp-develop-9.2.2-20260417/hw/char/esp32_uart.c#L68-L87) |
| `0x60002058` | `SFDP.0x10` и `RDSR2` = 0. |
| `0x6001d054`, `0x6001ccd4`, `0x60006090`, `0x600050f0` | `BBPD_CTRL`, `NRXPD_CTRL`, `FE_GEN_CTRL`, `FE2_TX_INTERP_CTRL` = 0. |
| `0x3fc80000`, `0x50000000` | `SRAM1_COLD_FILL` и `RTC_FAST_COLD_FILL` = 0 по снимкам QEMU. |
| `0x60004008`, `0x6000400c`, `0x60004024`, `0x60004028`, `0x6000404c` | Чтение `GPIO_OUT_W1TS/W1TC`, `GPIO_ENABLE_W1TS/W1TC`, `GPIO_STATUS_W1TC` = 0. Эффекты WT при записи следуют TRM. |
| `0x60023000` | `SYSTIMER_WFI_STALL` = 0. Это наблюдаемая связь счётчика с затвором CPU при WFI. TRM §10.4 и рег. 16.8 не определяют эту связь. В QEMU счётчик работает от виртуального времени. |

Отдельный [профиль ожидания RTC CRC](firmware-tests/esp_sdk/reference/esp32c3-qemu-rtc-crc.toml)
задаёт только наблюдение после `CRC_START`. У записи `kind = "qemu"` и источник в трассе. Он
подключается явно через `with_reset_profile`, а чтения регистрируются в `esp_reset_value_uses()`.

### Сверка MMIO при холодном старте без профиля

Временный журнал шины снят с ROM rev. 3 и свежего ESP-IDF v6.1.1 до смены поведения,
включая чтения `UART_INT_ST` при опросе IRQ. В таблице перечислены адреса между
прежними границами тестов и прежним фактическим отказом. Смещения прибавляются к указанной базе.
Заголовки — из `soc/esp32c3/register/soc/` установленного ESP-IDF v6.1.1.

| Адреса MMIO | Источник наблюдаемого значения и эффект операции |
|---|---|
| `0x60008000` + `38, 84, 8c, ac, b8, bc, d4` (hex) | `rtc_cntl_reg.h` и TRM рег. 9.12. `RESET_STATE` состоит из кода PowerOn (TRM §6.1.1), флага события сброса и битов 13:12 из таблицы. `PWC`, `DIG_ISO`, `SWD_CONF`, `STORE4/5` и `DIG_PAD_HOLD` используют сбросы 0, `0xaa805080`, `0x04b00000`, 0 и 0 соответственно, затем хранят записи разрешённых полей. |
| `0x60008800` + `30, 38, 3c, 48, 4c` | `efuse_reg.h` задаёт карту. Значения `RD_REPEAT_DATA0/2/3`, `RD_MAC_SPI_SYS_1/2` приходят из переданного readback QEMU. |
| `0x600c0000` + `00, 04, 10, 18, 40, 58`; `0x600ce044`, `0x600ce048` | `system_reg.h`: `CPU_PERI_CLK_EN/RST_EN` 0/`0xc0`, `PERIP_CLK_EN0/RST_EN0` `0xf9c1e06f`/0, `CACHE_CONTROL` 5, `SYSCLK_CONF` 1. `assist_debug_reg.h`: `RCD_PDEBUGPC` сброшен в 0, `RCD_EN` включает запись PC/SP. |
| `0x60009000` + `34, 38, 3c, 40, 44, 48, 54, 58`; `0x60004020`, `0x60004038`, `0x60004154`, `0x60004158` | `io_mux_reg.h` задаёт SPIHD/WP/CS0/CLK/D/Q и U0RXD/TXD. Сбросы функц. полей по TRM табл. 5.12-1: `0xb00`, для U0TXD `0x900`; далее хранится запись ROM. `gpio_reg.h`: GPIO_ENABLE и FUNC0/1_IN_SEL_CFG сброшены в 0. GPIO_STRAP берётся из входных strap, 0x0d или 0x08 по постановке. |
| `0x60000000` + `00, 08, 0c, 10, 14, 1c, 20, 24, 78`; `0x60010010` | `uart_reg.h`: FIFO и IRQ по состоянию модели, INT_ENA и CONF0 сброшены в 0 и `0x1000001c`, INT_CLR очищает запрошенные флаги. CLKDIV, CONF1 и CLK_CONF на этом пути записывает ROM до чтения. В STATUS FIFO вычисляются, уровни шести линий раньше были молчаливыми нулями. Теперь у `0x6000001c` это явный профиль. |
| `0x60043000`, `0x60043004` | `usb_serial_jtag_reg.h` и TRM §30.3.2: EP1 принимает байт, EP1_CONF сброшен в 2 и вычисляется по занятости IN FIFO без USB-хоста. WR_DONE закрывает пакет. |
| `0x60002000` + `00, 08, 0c, 10, 14, 18, 58, a4`; `0x60003000` + `08, 10, 14, 18, 1c, 20, 24, 28, 34, 3c` | `spi_mem_reg.h`: CTRL/CTRL1/CTRL2, CLOCK, USER/1/2, MOSI/MISO_DLEN, MISC, CACHE_FCTRL, W0 и SUS_STATUS. Сбросы совпадают с заголовком, записи меняют состояние. CMD читает idle между атомарными командами. RES завершался, на RDID (`CMD` `0x10000000`) требовались параметры микросхемы flash. |
| `0x600c4000` + `00, 04, 28, 2c, 30, 34, 40, 7c, a8, c4, cc`; `0x600c5000..=0x600c51fc`; `0x600c101c` | `extmem_reg.h` и TRM §3.3.3: CTRL/CTRL1 сброшены в 0/3, SYNC_ADDR/SIZE, WRAP_AROUND и MMU_OWNER в 0, PRELOAD/AUTOLOAD_DONE в 1. SYNC_DONE и FREEZE_DONE в модели следуют атомарной операции; ILG_INT_CLR очищает флаги. MMU-слова на этом участке записывает ROM. INTERNAL_SRAM_USAGE_1 сброшен в 0xf по TRM рег. 14.6 и `sensitive_reg.h`. |

Строгие `esp_boot`, `esp_irq` и `esp_sdk_io` раньше доходили до записи RDID
на инструкции №103037 (`0x4004db3c`, QEMU №402341). Теперь они останавливаются на
первом чтении `UART_STATUS_REG` с `UnsupportedRegister` (`0x6000001c`, PC `0x4004c0d4`,
инструкция №6196, в QEMU тот же PC на №391182).
Разница индексов включает разные ветви выбора часов и ожидания USB.
Модульный тест CPU без raw flash проходит `RTC_CNTL_RESET_STATE` на №5524
и останавливается на документированном `EXTMEM_ICACHE_CTRL1` на №5630
(`0x4005113c`, совпадает с QEMU), поскольку у этого стенда отсутствует модель кэша.
Его опыт с raw flash подтверждает остановку на `UART_STATUS`.

### ESP32-S3 в отдельном процессе

В `sim-runtime --test external_board` QEMU Espressif 9.2.2 дважды дал побайтово равный UART0 ROM, включая SLIP SYNC 46 байт → 14 байт. Ввод в этом опыте диагностический. Привязка к виртуальному такту и полный снимок остаются `NotAnalyzed`.

| Область | Статус |
|---|---|
| SDMMC, LEDC, Wi-Fi, Bluetooth, USB, RMT, SPI, I²C, I²S, ULP, GPIO Matrix, IO MUX, direct boot, Secure Boot | Каждый блок `NotAnalyzed`. См. [таблицу возможностей Espressif](https://github.com/espressif/esp-toolchain-docs/blob/main/qemu/README.md#supported-features). |
| ADC, калибровка ADC, PCNT, MCPWM, датчик температуры, touch, камера, LCD, GPIO, RTC, RTC WDT, дерево часов, modem/light/deep sleep, timer/GPIO wake, brownout, питание, ESP-NOW, USB Serial/JTAG | Каждый блок или режим `NotAnalyzed`. Покрытие конкретного режима не подтверждено. |
| Остальные документированные блоки QEMU, кроме проверенного UART0 ROM | Здесь `NotAnalyzed`. Адаптер Т2.20 подключает UART0, отдельные опыты этих блоков в Т2.23 не выполнены. |

### Повтор приёмки

Нужны готовый ESP32-C3 ROM rev. 3 из QEMU Espressif 9.2.2 (`share/qemu/esp32c3-rom.bin`, SHA-256 `0de1e65020e803bea0d7443dca149d61895e01fca3bb9c82d073234eebd73f99`) и [проверяемые фикстуры](tests/data/esp_sdk_acceptance/SHA256SUMS). Без внешнего ROM штатные опыты `NotAnalyzed`. Снимок eFuse снят с профиля SDK v6.1.1 через QEMU Monitor `pmemsave 0x6000882c 336 "readback.bin"` до `cont`. Для 2 и 4 МиБ соответственно использованы JEDEC w25x16 и gd25q32 из `hw/riscv/esp32c3.c` и `hw/block/m25p80.c`.

Из корня репозитория

```sh
(cd crates/sim-boards/tests/data/esp_sdk_acceptance && shasum -a 256 -c SHA256SUMS)
(cd crates/sim-boards/tests/data/esp_peripherals && shasum -a 256 -c SHA256SUMS)
SIM_ESP_ACCEPTANCE_ROM=/путь/к/esp32c3-rom.bin \
  cargo test --offline -p sim-boards --test esp_sdk_acceptance -- --include-ignored --nocapture --test-threads=1
```

Для Blink команда ниже использует `arduino-cli 1.6.0` и установленное ядро `esp32:esp32 4.0.0`. Параметр `-ffile-prefix-map` заменяет абсолютные пути в строках исходников. `SOURCE_DATE_EPOCH=0` фиксирует дату и время служебного отчёта GCC в начале эпохи Unix. `--strip-all` убирает из ELF отладочные секции и символы до записи отпечатка ELF в образ. Две сборки в разных каталогах дали один `Blink.ino.merged.bin` на 4 МиБ, его SHA-256 записана в `SHA256SUMS`. `esptool --chip esp32c3 image-info` подтвердил XOR и SHA-256 загрузчика и приложения.

```sh
(
  work=$(mktemp -d /tmp/esp-t223-blink.XXXXXX)
  trap 'rm -rf "$work"' EXIT
  cp -R crates/sim-boards/firmware-tests/esp_sdk/Blink "$work/"
  cd "$work"
  export SOURCE_DATE_EPOCH=0 ARDUINO_BUILD_CACHE_PATH="$work/cache" ARDUINO_UPDATER_ENABLE_NOTIFICATION=false
  flags="-ffile-prefix-map=/$HOME=. -ffile-prefix-map=$HOME=. -ffile-prefix-map=$(pwd -P)=."
  arduino-cli compile -j 4 --fqbn 'esp32:esp32:esp32c3:CDCOnBoot=default,FlashMode=dio' \
    --build-path "$(pwd -P)/build" \
    --build-property "compiler.c.extra_flags=$flags" \
    --build-property "compiler.cpp.extra_flags=$flags" \
    --build-property 'compiler.c.elf.extra_flags=-Wl,--strip-all' Blink
  shasum -a 256 build/Blink.ino.merged.bin
)
```

Для `hello_world` использована сборка из [команд ESP-IDF и QEMU](firmware-tests/esp_sdk/SOURCES.md). Сохранённый образ заканчивается на конце приложения, QEMU использует его с дополнением `0xff` до 2 МиБ. Для 2.21 команда [reproduce.sh](firmware-tests/esp_peripherals/reproduce.sh) пересобирает и сравнивает пять исходных артефактов. Опорные параметры модели и адреса относятся к [TRM ESP32-C3 v1.4](https://documentation.espressif.com/esp32-c3_technical_reference_manual_en.pdf).

## Источники

- Raspberry Pi RP2040 §§1.4, 2, 4, 5.5.3 и паспорт питания Pico. [RP2040 Datasheet](https://pip-assets.raspberrypi.com/categories/814-rp2040/documents/RP-008371-DS-1-rp2040-datasheet.pdf), [Pico Datasheet](https://pip-assets.raspberrypi.com/categories/610-raspberry-pi-pico/documents/RP-008307-DS-3-pico-datasheet.pdf), [Pico SDK watchdog](https://raw.githubusercontent.com/raspberrypi/pico-sdk/master/src/rp2_common/hardware_watchdog/watchdog.c).
- ST DS5319, RM0008 и PM0056. [STM32F103x8/xB](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf), [RM0008](https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf), [PM0056](https://www.st.com/resource/en/programming_manual/pm0056-stm32f10xxx20xxx21xxxl1xxxx-cortex-m3-programming-manual-stmicroelectronics.pdf).
- Microchip 7810D, DS40002198C и распиновки Arduino. [ATmega328P](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf), [AVR Instruction Set Manual](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/ReferenceManuals/AVR-InstructionSet-Manual-DS40002198.pdf), [Uno](https://content.arduino.cc/assets/A000066-full-pinout.pdf), [Nano](https://docs.arduino.cc/resources/pinouts/A000005-full-pinout.pdf).
- Espressif ESP32-C3, главы по регистрам и официальные заголовки. [Datasheet](https://documentation.espressif.com/esp32-c3_datasheet_en.html), [TRM](https://documentation.espressif.com/esp32-c3_technical_reference_manual_en.pdf), [ESP-IDF v5.2](https://github.com/espressif/esp-idf/tree/v5.2/components/soc/esp32c3/include/soc).
- Архитектурные спецификации. [Arm Architecture Reference Manuals](https://developer.arm.com/community/arm-community-blogs/b/architectures-and-processors-blog/posts/cortex-m-resources), [RISC-V ISA](https://docs.riscv.org/reference/isa/unpriv/unpriv-index.html), [RISC-V Machine-Level ISA](https://docs.riscv.org/reference/isa/priv/machine.html).
