/****************************************************************************
 * boards/t3gemstone/o1/nuttx-config/scripts/script.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* PX4 links the final loadable ELF with THIS script (see
 * platforms/nuttx/CMakeLists.txt: --script=.../scripts/script.ld), not the
 * NuttX board's sdram.ld. It must therefore reproduce the exact memory map
 * that Linux remoteproc already loads (p_paddr) for the NuttX-only image.
 *
 * CRITICAL: CONFIG_BOOT_RUNFROMISRAM=y means arm_head.S does NOT copy .data
 * from LMA to VMA. So .data must have LMA == VMA (no AT()). Adding a phantom
 * LMA here reintroduces the boot data-abort we already root-caused and fixed
 * in sdram.ld. Keep .data as a plain "> ddr" section.
 */

MEMORY
{
  atcm         (rwx): ORIGIN = 0x00000000, LENGTH = 32K      /* Instruction TCM for vectors/boot code */
  btcm         (rwx): ORIGIN = 0x41010000, LENGTH = 32K      /* Data TCM, ideal for stack */
  ddr_rsctable (rwx): ORIGIN = 0xA2100000, LENGTH = 1K       /* Resource Table for RemoteProc/IPC */
  ddr_rsv      (rwx): ORIGIN = 0xA2200000, LENGTH = 256K     /* VRING + RPMsg (VDEV0_VRING_BASE) */
  ddr          (rwx): ORIGIN = 0xA2240000, LENGTH = 0xDC0000 /* Main DDR RAM (~13.75 MB) */
}

EXTERN(_vector_start)
ENTRY(_vector_start)

SECTIONS
{
    .vectors :
    {
        _stext = ABSOLUTE(.);
        KEEP(*(.vectors))
        KEEP(*arm_head*.o*(.text))
        KEEP(*(.text.arm_boot))
        KEEP(*(.text.am67_mpu_init))
        KEEP(*(.text.mpu_control))
        KEEP(*(.text.cp15_rdsctlr))
        KEEP(*(.text.am67_mpu_reset))
        KEEP(*(.text.mpu_set_region_zero))
        KEEP(*(.text.am67_mpu_disable_br))
        KEEP(*(.text.mpu_configure_region))
        KEEP(*(.text.mpu_allocregion))
        KEEP(*(.text.mpu_modify_region))
        KEEP(*(.text.mpu_set_rgnr))
        KEEP(*(.text.mpu_set_drbar))
        KEEP(*(.text.mpu_log2regionceil))
        KEEP(*(.text.mpu_subregion))
        KEEP(*(.text.mpu_subregion_ms))
        KEEP(*(.text.mpu_subregion_ls))
        KEEP(*(.text.mpu_set_drsr))
        KEEP(*(.text.mpu_set_dracr))
        KEEP(*(.text.cp15_wrsctlr))
        KEEP(*(.text.nx_start))
        KEEP(*(.tick_timer))
        KEEP(*(.text.nxsched_process_timer))
        KEEP(*(.text.clock_timer))
        KEEP(*(.text.wd_timer))
        KEEP(*(.text.clock_systime_ticks))
        /* DShot FIQ bit engine and its state (am67_dshot.c) */
        KEEP(*(.am67_fastcode))
        KEEP(*(.am67_fastdata))

        /* Hot code (PC profile, docs/adimlar/16 A2). A DDR instruction
         * cache miss costs ~137 cycles, TCM fetches none. Calls between
         * here and DDR go through linker veneers (a few cycles).
         */

        /* Exception entry, dispatch, scheduler, semaphores */
        *arm_vectors*.o*(.text)
        *arm_saveusercontext*.o*(.text)
        *(.text.arm_decodeirq)
        *(.text.arm_doirq)
        *(.text.irq_dispatch)
        *(.text.arm_syscall)
        *(.text.up_switch_context)
        *(.text.nxsched_add_readytorun)
        *(.text.nxsched_remove_readytorun)
        *(.text.nxsched_merge_pending)
        *(.text.nxsched_switch_context)
        *(.text.nxsched_add_prioritized)
        *(.text.nxsched_get_tcb)
        *(.text.nxsem_post)
        *(.text.nxsem_post_slow)
        *(.text.nxsem_wait)
        *(.text.nxsem_wait_slow)
        *(.text.nxsem_trywait)
        *(.text.nxsem_tickwait)
        *(.text.nxsig_clockwait)
        *(.text.sem_wait)
        *(.text.sem_post)
        *(.text.enter_critical_section)
        *(.text.leave_critical_section)
        *(.text.clock_systime_timespec)

        /* libc copies (optimized assembly) */
        *arch_memcpy*.o*(.text)
        *arch_memset*.o*(.text)
        *arch_strcmp*.o*(.text)

        /* HRT and the IMU SPI driver */
        *(.text.hrt_absolute_time)
        *(.text.hrt_call_internal)
        *(.text.hrt_call_enter)
        *(.text.hrt_call_invoke)
        *(.text.hrt_call_reschedule)
        *(.text.hrt_call_after)
        *(.text.hrt_tim_isr)
        *(.text.hrt_latency_update)
        *(.text.am67_mcspi_*)
        *(.text.perf_begin)
        *(.text.perf_end)
        *(.text.perf_count)
        *(.text.perf_set_elapsed)

        /* uORB publish/copy and work queues */
        *(.text._ZN4uORB10DeviceNode5write*)
        *(.text._ZN4uORB10DeviceNode7publish*)
        *(.text._ZN4uORB10DeviceNode15poll_notify_one*)
        *(.text._ZN4uORB7Manager13orb_data_copy*)
        *(.text._ZN4uORB7Manager17get_device_master*)
        *(.text._ZN4uORB7Manager27orb_add_internal_subscriber*)
        *(.text._ZN4uORB12DeviceMaster19getDeviceNodeLocked*)
        *(.text._ZN4uORB12Subscription*)
        *(.text._ZN4uORB24SubscriptionIntervalBase*)
        *(.text._ZN4uORB20PublicationMultiBase7publish*)
        *(.text._Z12get_orb_meta*)
        *(.text._ZN4cdev4CDev11poll_notify*)
        *(.text._ZN4cdev4CDev15poll_notify_one*)
        *(.text._ZN3px49WorkQueue3Run*)
        *(.text._ZN3px49WorkQueue3Add*)
        *(.text._ZN3px48WorkItem11ScheduleNow*)
        *(.text.param_get)

        /* IMU sample path at 9 kHz and the rate loop filters */
        *(.text._Z9rotate_3i*)
        *(.text._ZN12PX4Gyroscope10updateFIFO*)
        *(.text._ZN16PX4Accelerometer10updateFIFO*)
        *(.text._ZN8ICM2094811ProcessGyro*)
        *(.text._ZN8ICM2094812ProcessAccel*)
        *(.text._ZN7sensors22VehicleAngularVelocity3Run*)
        *(.text._ZN7sensors22VehicleAngularVelocity21FilterAngularVelocity*)
    } > atcm

    .resource_table :
    {
        KEEP(*(.resource_table))
        . = ALIGN(1024);
    } > ddr_rsctable

    .ddr_rsv (NOLOAD) :
    {
        _vring_base = ABSOLUTE(.);
        . = . + 256K;
    } > ddr_rsv

    .text :
    {
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.glue_7)
        *(.glue_7t)
        *(.gcc_except_table)
    } > ddr

    .rodata :
    {
        *(.rodata .rodata.*)
        *(.gnu.linkonce.r.*)
        *(.got)
    } > ddr

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > ddr

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > ddr

    _etext = ABSOLUTE(.); /* End of code/read-only sections */

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > ddr

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > ddr

    .noinit (NOLOAD) :
    {
        _snoinit = ABSOLUTE(.);
        *(.noinit*)
        _enoinit = ABSOLUTE(.);
    } > ddr

    .bss (NOLOAD) :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > ddr

    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        _sheap = ABSOLUTE(.);
        _end = _sheap;
        . = . + 64K;
        _eheap = ABSOLUTE(.);
    } > ddr

    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        _sstack = ABSOLUTE(.);
        . = . + 8K;
        _estack = ABSOLUTE(.);
    } > btcm

    /* Stabs debugging sections. */

    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}
