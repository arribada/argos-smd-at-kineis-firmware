/*
 * Linker script for STM32WL55 Application (with bootloader after app)
 * Application starts at 0x08000000 (for Kineis library compatibility)
 *
 * Memory Layout:
 * 0x08000000 - 0x08032FFF : Application Code (204KB)
 * 0x08033000 - 0x0803AFFF : Bootloader (32KB)
 * 0x0803B000 - 0x0803FFFF : Flash User Data (20KB) - PRESERVED
 */

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM1) + LENGTH(RAM1);

_Min_Heap_Size = 0x600;      /* required amount of heap */
_Min_Stack_Size = 0x2000;    /* required amount of stack */

/* Bootloader address for jump */
_bootloader_addr = 0x08033000;

/* Memory regions for application */
MEMORY
{
  /* Application code at start of flash (Kineis compatible) */
  ROM         (rx)  : ORIGIN = 0x08000000, LENGTH = 204K      /* 204KB for app */
  /* Bootloader region (not used by app, just reserved) */
  /* BL_ROM   (rx)  : ORIGIN = 0x08033000, LENGTH = 32K */
  /* Flash user data - PRESERVED at original location */
  FLASH_USER  (rw)  : ORIGIN = 0x0803B000, LENGTH = 20K       /* 20KB user data */
  RAM1        (xrw) : ORIGIN = 0x20000000, LENGTH = 32K       /* Non-backup SRAM1 */
  RAM2        (xrw) : ORIGIN = 0x20008000, LENGTH = 32K       /* Backup SRAM2 */
  RTC_BKPR    (xrw) : ORIGIN = 0x4000B100, LENGTH = 128       /* RTC backup registers */
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

  /* RAM2 data (Kineis context - retention) */
  _sidata2 = LOADADDR(.data2);

  .data2 :
  {
    . = ALIGN(4);
    _sdata2 = .;
    *(.knsCtxtData)
    *(.retentionRamData)
    . = ALIGN(4);
    _edata2 = .;
  } >RAM2 AT> ROM

  /* RAM1 initialized data */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(8);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(8);
    _edata = .;
  } >RAM1 AT> ROM

  /* RAM2 BSS (Kineis context) */
  . = ALIGN(4);
  .bss2 :
  {
    _sbss2 = .;
    __bss2_start__ = _sbss2;
    *(.knsCtxtBss)
    *(.retentionRamBss)
    . = ALIGN(4);
    _ebss2 = .;
    __bss2_end__ = _ebss2;
  } >RAM2

  /* RAM1 BSS */
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

  /* RAM1 region */
  . = ALIGN(8);
  RAM1_region :
  {
    _sRAM1_region = .;
    *(.RAM1_region)
    . = ALIGN(8);
    _eRAM1_region = .;
  } >RAM1

  /* RAM2 region */
  . = ALIGN(8);
  RAM2_region :
  {
    _sRAM2_region = .;
    *(.RAM2_region)
    . = ALIGN(8);
    _eRAM2_region = .;
  } >RAM2

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

  /* RTC backup registers (LPM context) */
  .rtc_bkpr :
  {
    . = ALIGN(4);
    _srtc_bkpr = .;
    *(.lpmSection)
    *(.msgCntSectionData)
    *(.rtc_bkpr)
    *(.rtc_bkpr*)
    . = ALIGN(4);
    _ertc_bkpr = .;
  } >RTC_BKPR

  /* Flash user data - at fixed address */
  .flash_user_data :
  {
    . = ALIGN(8);
    KEEP(*(.flash_user_data))
    KEEP(*(.flash_user_data*))
    . = ALIGN(8);
    _sflash_user_data = .;
  } >FLASH_USER

  /* Discard library debug info */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
