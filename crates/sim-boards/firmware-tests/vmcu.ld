ENTRY(_start)
MEMORY {
  FLASH (rx) : ORIGIN = 0x42000000, LENGTH = 2M
}
PHDRS { flash PT_LOAD FLAGS(5); }
SECTIONS {
  .text : { KEEP(*(.text.start)) *(.text*) } > FLASH :flash
  /DISCARD/ : { *(.eh_frame*) }
}
