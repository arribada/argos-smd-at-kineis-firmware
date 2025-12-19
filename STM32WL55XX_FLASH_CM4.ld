/*
** LinkerScript
** Note: For specific memory allocation, linker and startup files must be customized.
**       Refer to STM32CubeIDE user guide (UM2609), chapter "Modify the linker script".
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM1) + LENGTH(RAM1); /* end of "SRAM1" Ram type memory */

_Min_Heap_Size = 0x600;      /* required amount of heap  */
_Min_Stack_Size = 0x2000; /* required amount of stack */

/* Memories definition */
MEMORY
{
  ROM         (rx)  : ORIGIN = 0x08000000, LENGTH = 236K       /* 256K - 20K */
  FLASH_USER  (rw)  : ORIGIN = 0x0803B000, LENGTH = 20K        /* 0x8000 bytes */
  RAM1   (xrw)     : ORIGIN = 0x20000000, LENGTH = 32K    /* Non-backup SRAM1 */
  RAM2   (xrw)     : ORIGIN = 0x20008000, LENGTH = 32K    /* Backup SRAM2 */
  RTC_BKPR (xrw)   : ORIGIN = 0x4000B100, LENGTH = 128     /* TAMP_BKPR register used to backup over LPM */

}

/* Sections */
SECTIONS
{
  /* The startup code into "ROM" Rom type memory */
  .isr_vector :
  {
    . = ALIGN(8);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(8);
  } >ROM

  /* The program code and other data into "ROM" Rom type memory */
  .text :
  {
    . = ALIGN(8);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(8);
    _etext = .;        /* define a global symbols at end of code */
  } >ROM

  /* Constant data into "ROM" Rom type memory */
  .rodata :
  {
    . = ALIGN(8);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(8);
  } >ROM

  .ARM.extab   : { 
    . = ALIGN(8);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(8);
  } >ROM
  
  .ARM : {
    . = ALIGN(8);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(8);
  } >ROM

  .preinit_array     :
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


  /* ------------------------------------------------------------------------------ 
   * ---- Initialized data sections goes into RAM1, load LMA copy after code   ---- 
   * ---- so far, put all into RAM1 section                                    ---- 
   * ---- RAM2 contains kineis stack lpm retention data                        ---- 
   * ------------------------------------------------------------------------------ 
   */

  _sidata2 = LOADADDR(.data2); /* used by the startup to initialize data */

  .data2 : 
  {
    . = ALIGN(4);
    _sdata2 = .;        /* create a global symbol at data start */

    *(.knsCtxtData)
    *(.retentionRamData)

    . = ALIGN(4);
    
    _edata2 = .;        /* define a global symbol at data end */
  } >RAM2 AT> ROM

  _sidata = LOADADDR(.data); /* Used by the startup to initialize data */

  /* Initialized data sections into "SRAM1" Ram type memory */
  .data : 
  {
    . = ALIGN(8);
    _sdata = .;        /* create a global symbol at data start */

    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(8);
    _edata = .;        /* define a global symbol at data end */
  } >RAM1 AT> ROM

  /* ------------------------------------------------------------------------------
   * ---- Uninitialized data sections goes into RAM                            ----
   * ---- so far, put all into RAM section                                     ----
   * ---- RAM3 is empty so far, RAM2 contains kineis stack lpm retention data  ----
   * ------------------------------------------------------------------------------
   */

  . = ALIGN(4);
  .bss2 :
  {
    /* This is used by the startup in order to initialize the .bss2 secion */
    _sbss2 = .;         /* define a global symbol at bss2 start */
    __bss2_start__ = _sbss2;

    *(.knsCtxtBss)
    *(.retentionRamBss)

    . = ALIGN(4);
    _ebss2 = .;         /* define a global symbol at bss2 end */
    __bss2_end__ = _ebss2;
  } >RAM2

  . = ALIGN(8);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;

    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(8);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM1

  /* Data section into "SRAM1" Ram type memory: Non-backup SRAM1 dedicated to CM4 */
  . = ALIGN(8);
  RAM1_region :
  {
    _sRAM1_region = .;         /* define a global symbol at section start */
    *(.RAM1_region)

    . = ALIGN(8);
    _eRAM1_region = .;         /* define a global symbol at section end */
  } >RAM1

  /* Data section into "SRAM2" Ram type memory: Backup SRAM2 dedicated to CM4 */
  . = ALIGN(8);
  RAM2_region :
  {
    _sRAM2_region = .;         /* define a global symbol at section start */
    *(.RAM2_region)

    . = ALIGN(8);
    _eRAM2_region = .;         /* define a global symbol at section end */
  } >RAM2

  /* User_heap_stack section, used to check that there is enough "SRAM1" Ram  type memory left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM1

  /* ------------------------------------------------------------------------------ */

  /* RTC backup domain register section (32 32-bit-long registers), retention during shutdown and standby modes */
  .rtc_bkpr :
  {
    . = ALIGN(4);
    _srtc_bkpr = .;        /* create a global symbol at rtc_bkpr start */
    *(.lpmSection)         /* LPM context section, used to store data what ever the low power mode is */
    *(.msgCntSectionData)
    *(.rtc_bkpr)           /* .rtc_bkpr sections */
    *(.rtc_bkpr*)          /* .rtc_bkpr* sections */

    . = ALIGN(4);
    _ertc_bkpr = .;        /* define a global symbol at rtc_bkpr end */
  } >RTC_BKPR

  /* ------------------------------------------------------------------------------ */

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
    /* Protection: End of usable ROM before FLASH_USER */
  
  /* Flash user-defined data section (placed at the end of flash) */
	.flash_user_data :
	{
		. = ALIGN(8);
		KEEP(*(.flash_user_data))
		KEEP(*(.flash_user_data*))
		. = ALIGN(8);
		_sflash_user_data = .;
	} >FLASH_USER
  
}
