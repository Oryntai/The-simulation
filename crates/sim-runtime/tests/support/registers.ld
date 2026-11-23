ENTRY(_start)
MEMORY {
  IRAM (rwx) : ORIGIN = 0x40380000, LENGTH = 128K
}
PHDRS { iram PT_LOAD FLAGS(5); }
SECTIONS {
  .text : {
    KEEP(*(.text.start))
    KEEP(*(.text.vectors))
    *(.text*) *(.rodata*)
  } > IRAM :iram
  /DISCARD/ : { *(.eh_frame*) }
}
