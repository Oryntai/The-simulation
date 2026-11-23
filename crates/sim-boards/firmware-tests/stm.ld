ENTRY(start)
MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
  RAM (rw) : ORIGIN = 0x20000000, LENGTH = 20K
}
PHDRS { flash PT_LOAD FLAGS(5); }
SECTIONS {
  .vectors ORIGIN(FLASH) : { KEEP(*(.vectors)) } > FLASH :flash
  .text : { *(.text*) *(.rodata*) } > FLASH :flash
  /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) *(.eh_frame*) }
}
