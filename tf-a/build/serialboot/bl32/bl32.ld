OUTPUT_FORMAT(elf32-littlearm)
OUTPUT_ARCH(arm)
ENTRY(sp_min_vector_table)
MEMORY {
    RAM (rwx): ORIGIN = ((0x2FFC0000) + ((0x00040000) - ((1) << (12))) - (0x00012000)), LENGTH = (((0x2FFC0000) + ((0x00040000) - ((1) << (12))) - (0x00012000)) + (0x00012000)) - ((0x2FFC0000) + ((0x00040000) - ((1) << (12))) - (0x00012000))
}
SECTIONS
{
    . = ((0x2FFC0000) + ((0x00040000) - ((1) << (12))) - (0x00012000));
   ASSERT(. == ALIGN(((1) << (12))),
          "BL32_BASE address is not aligned on a page boundary.")
    ro . : {
        __RO_START__ = .;
        *entrypoint.o(.text*)
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        __RT_SVC_DESCS_START__ = .;
        KEEP(*(rt_svc_descs))
        __RT_SVC_DESCS_END__ = .;
        . = ALIGN(4);
        __CPU_OPS_START__ = .;
        KEEP(*(cpu_ops))
        __CPU_OPS_END__ = .;
        . = ALIGN(8);
__pubsub_psci_cpu_on_finish_start = .; KEEP(*(__pubsub_psci_cpu_on_finish)); __pubsub_psci_cpu_on_finish_end = .;
__pubsub_psci_suspend_pwrdown_start_start = .; KEEP(*(__pubsub_psci_suspend_pwrdown_start)); __pubsub_psci_suspend_pwrdown_start_end = .;
__pubsub_psci_suspend_pwrdown_finish_start = .; KEEP(*(__pubsub_psci_suspend_pwrdown_finish)); __pubsub_psci_suspend_pwrdown_finish_end = .;
        *(.vectors)
        __RO_END_UNALIGNED__ = .;
        . = ALIGN(((1) << (12)));
        __RO_END__ = .;
    } >RAM
    ASSERT(__CPU_OPS_END__ > __CPU_OPS_START__,
           "cpu_ops not defined for this platform.")
    __RW_START__ = . ;
    .data . : {
        __DATA_START__ = .;
        *(.data*)
        __DATA_END__ = .;
    } >RAM
    stacks (NOLOAD) : {
        __STACKS_START__ = .;
        *(tzfw_normal_stacks)
        __STACKS_END__ = .;
    } >RAM
    .bss (NOLOAD) : ALIGN(8) {
        __BSS_START__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(((1) << 6));
        __BAKERY_LOCK_START__ = .;
        __PERCPU_BAKERY_LOCK_START__ = .;
        *(bakery_lock)
        . = ALIGN(((1) << 6));
        __PERCPU_BAKERY_LOCK_END__ = .;
        __PERCPU_BAKERY_LOCK_SIZE__ = ABSOLUTE(__PERCPU_BAKERY_LOCK_END__ - __PERCPU_BAKERY_LOCK_START__);
        . = . + (__PERCPU_BAKERY_LOCK_SIZE__ * (((0) + (2)) - 1));
        __BAKERY_LOCK_END__ = .;
        __BSS_END__ = .;
    } >RAM
    xlat_table (NOLOAD) : {
        *(xlat_table)
    } >RAM
     __BSS_SIZE__ = SIZEOF(.bss);
    __RW_END__ = .;
   __BL32_END__ = .;
}
