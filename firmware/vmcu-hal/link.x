/* Скрипт компоновки VMCU-C3. Карта памяти — crates/sim-mcu/MEMORY_MAP.md. */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x42000000, LENGTH = 4M
  RAM   (rwx) : ORIGIN = 0x3FC80000, LENGTH = 384K
}

ENTRY(_start)

SECTIONS
{
  .text : ALIGN(4)
  {
    KEEP(*(.text.init))
    KEEP(*(.text.trap))
    *(.text .text.*)
  } > FLASH

  .rodata : ALIGN(4)
  {
    *(.srodata .srodata.*)
    *(.rodata .rodata.*)
    . = ALIGN(4);
  } > FLASH

  .data : ALIGN(4)
  {
    _sdata = .;
    *(.sdata .sdata.*)
    *(.data .data.*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH
  _sidata = LOADADDR(.data);

  .bss (NOLOAD) : ALIGN(4)
  {
    _sbss = .;
    *(.sbss .sbss.*)
    *(.bss .bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /* Не обнуляется стартовым кодом: данные, которые должны пережить сброс
     сторожем (ОЗУ при таком сбросе сохраняется). После включения питания —
     мусор, поэтому их защищают магией и контрольным словом. */
  .noinit (NOLOAD) : ALIGN(4)
  {
    *(.noinit .noinit.*)
    . = ALIGN(4);
  } > RAM

  /* Стек — от конца ОЗУ вниз. */
  _stack_top = ORIGIN(RAM) + LENGTH(RAM);

  /DISCARD/ :
  {
    *(.eh_frame .eh_frame_hdr)
  }
}
