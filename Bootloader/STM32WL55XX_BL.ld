/*
 * Linker script for STM32WL55 Bootloader
 * Bootloader placed AFTER application (0x08033000 - 0x0803AFFF)
 *
 * Memory Layout:
 * 0x08000000 - 0x08032FFF : Application Code (204KB)
 * 0x08033000 - 0x0803AFFF : Bootloader (32KB) <-- THIS
 * 0x0803B000 - 0x0803FFFF : Flash User Data (20KB)
 */

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM1) + LENGTH(RAM1);

/* Bootloader uses minimal heap/stack */
_Min_Heap_Size = 0x200;     /* 512 bytes heap */
_Min_Stack_Size = 0x800;    /* 2KB stack */

/* Application address for jump */
_app_addr = 0x08000000;

/* Application end address (for flash operations) */
_app_end_addr = 0x08032FFF;

/* Memory regions for bootloader */
MEMORY
{
  /* Bootloader ROM after application */
  ROM         (rx)  : ORIGIN = 0x08033000, LENGTH = 32K       /* Bootloader: 32KB */
  /* Flash user for BL state - small portion at start of flash_user */
  FLASH_USER  (rw)  : ORIGIN = 0x0803B000, LENGTH = 2K        /* BL state storage */
  RAM1        (xrw) : ORIGIN = 0x20000000, LENGTH = 32K       /* SRAM1 */
  RAM2        (xrw) : ORIGIN = 0x20008000, LENGTH = 32K - 8   /* SRAM2 (backup) */
  RAM_NOINIT  (xrw) : ORIGIN = 0x2000FFF8, LENGTH = 8         /* DFU flag (shared with app) */
}

/* Sections */
SECTIONS
{
  /* Vector table at start of bootloader region */
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

  /* Non-initialized data section - DFU flag shared with app, not zeroed */
  .noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.noinit)
    *(.noinit*)
    . = ALIGN(4);
  } >RAM_NOINIT

  /* Remove debug info from libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
