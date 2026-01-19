/*
 * Linker script for STM32WL55 Bootloader
 * Bootloader occupies first 32KB of flash (0x08000000 - 0x08007FFF)
 */

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM1) + LENGTH(RAM1);

/* Bootloader uses minimal heap/stack */
_Min_Heap_Size = 0x200;     /* 512 bytes heap */
_Min_Stack_Size = 0x800;    /* 2KB stack */

/* Memory regions for bootloader */
MEMORY
{
  ROM         (rx)  : ORIGIN = 0x08000000, LENGTH = 32K       /* Bootloader: 32KB */
  FLASH_USER  (rw)  : ORIGIN = 0x0803B800, LENGTH = 2K        /* BL state storage */
  RAM1        (xrw) : ORIGIN = 0x20000000, LENGTH = 32K       /* SRAM1 */
  RAM2        (xrw) : ORIGIN = 0x20008000, LENGTH = 32K       /* SRAM2 (backup) */
}

/* Sections */
SECTIONS
{
  /* Vector table at start of flash */
  .isr_vector :
  {
    . = ALIGN(8);
    KEEP(*(.isr_vector))
    . = ALIGN(8);
  } >ROM

  /* Program code */
  .text :
  {
    . = ALIGN(8);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(8);
    _etext = .;
  } >ROM

  /* Constant data */
  .rodata :
  {
    . = ALIGN(8);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(8);
  } >ROM

  .ARM.extab :
  {
    . = ALIGN(8);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(8);
  } >ROM

  .ARM :
  {
    . = ALIGN(8);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(8);
  } >ROM

  .preinit_array :
  {
    . = ALIGN(8);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(8);
  } >ROM

  .init_array :
  {
    . = ALIGN(8);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(8);
  } >ROM

  .fini_array :
  {
    . = ALIGN(8);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(8);
  } >ROM

  /* Used by startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data */
  .data :
  {
    . = ALIGN(8);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(8);
    _edata = .;
  } >RAM1 AT> ROM

  /* Uninitialized data */
  . = ALIGN(8);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(8);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM1

  /* User heap and stack */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM1

  /* Bootloader state in flash user area */
  .bl_state :
  {
    . = ALIGN(8);
    KEEP(*(.bl_state))
    KEEP(*(.bl_state*))
    . = ALIGN(8);
  } >FLASH_USER

  /* Remove debug info from libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
