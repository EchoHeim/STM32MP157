OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(bl2_entrypoint)
MEMORY {
    RAM (rwx): ORIGIN = (((0x2FFC0000) + ((0x00040000) - ((1) << (12))) - (0x00012000)) - (0x00017000)), LENGTH = ((((0x2FFC0000) + ((0x00040000) - ((1) << (12))) - (0x00012000)) - (0x00017000)) + (0x00017000)) - (((0x2FFC0000) + ((0x00040000) - ((1) << (12))) - (0x00012000)) - (0x00017000))
}
SECTIONS
{
    . = (((0x2FFC0000) + ((0x00040000) - ((1) << (12))) - (0x00012000)) - (0x00017000));
    ASSERT(. == ALIGN(((1) << (12))),
           "BL2_BASE address is not aligned on a page boundary.")
    ro . : {
        __RO_START__ = .;
 __TEXT_RESIDENT_START__ = .;
 *bl2_el3_entrypoint.o(.text*)
 *(.text.asm.*)
 __TEXT_RESIDENT_END__ = .;
        *(.text*)
        *(.rodata*)
        . = ALIGN(8);
        __CPU_OPS_START__ = .;
        KEEP(*(cpu_ops))
        __CPU_OPS_END__ = .;
        . = ALIGN(8);
        __PARSER_LIB_DESCS_START__ = .;
        KEEP(*(.img_parser_lib_descs))
        __PARSER_LIB_DESCS_END__ = .;
        *(.vectors)
        __RO_END_UNALIGNED__ = .;
        . = ALIGN(((1) << (12)));
        __RO_END__ = .;
    } >RAM
    ASSERT(__CPU_OPS_END__ > __CPU_OPS_START__,
          "cpu_ops not defined for this platform.")
    __RW_START__ = . ;
    .data . : {
        __DATA_RAM_START__ = .;
        *(.data*)
        __DATA_RAM_END__ = .;
    } >RAM AT>RAM
    stacks (NOLOAD) : {
        __STACKS_START__ = .;
        *(tzfw_normal_stacks)
        __STACKS_END__ = .;
    } >RAM
    .bss : ALIGN(16) {
        __BSS_START__ = .;
        *(SORT_BY_ALIGNMENT(.bss*))
        *(COMMON)
        __BSS_END__ = .;
    } >RAM
    xlat_table (NOLOAD) : {
        *(xlat_table)
    } >RAM
    __RW_END__ = .;
    __BL2_END__ = .;
    __BSS_SIZE__ = SIZEOF(.bss);
    ASSERT(. <= ((((0x2FFC0000) + ((0x00040000) - ((1) << (12))) - (0x00012000)) - (0x00017000)) + (0x00017000)), "BL2 image has exceeded its limit.")
}
