*****************************************************************************
** ChibiOS/RT port for the T3 Gemstone O1 (TI AM67A/J722S) Cortex-R5F.    **
*****************************************************************************

** TARGET **

The demo runs on the MCU domain Cortex-R5F (MCU_R5FSS0 core 0) of the T3
Gemstone O1 board, loaded through the Linux k3-r5 remoteproc driver running
on the A53 cores.

The MCU core is used rather than the Main domain R5F so that the Main core
stays available for NuttX (whose upstream t3-gem-o1 port targets it) and for
the TI vision/AI stack. Note that the two cores are NOT interchangeable: the
MCU core has a single usable TCM at address 0, a different VIM base, a
different tick timer and its own DDR carveout, so an image built for one core
will not boot on the other.

** THE DEMO **

The demo spawns a counter thread and, when the FPU is enabled, an FPU
context-validation thread, while the main thread logs the counters once per
second into the RemoteProc trace buffer.

Memory layout (device addresses, must match the host device tree carveouts):

  0x00000000  TCM    32K  exception vectors, pre-MPU boot code, mode stacks
  0xA1100000  DDR     4K  remoteproc resource table
  0xA1110000  DDR    16K  remoteproc trace buffer
  0xA1240000  DDR   ~14M  code, data, bss, heap

The TCM at address 0 is the BTCM: the device tree sets ti,loczrama = <0> for
this core, which swaps the two TCMs relative to the Main core, and then
ti,atcm-enable = <0> leaves the ATCM (at 0x41010000) switched off. Cortex-R5
has no VBAR, so the vectors must live at address 0 regardless.

The OS tick comes from MCU_TIMER0 (0x04800000, 25 MHz, VIM IRQ 28), and
interrupts are dispatched through the TI VIM controller (0x07FF0000, the MCU
core's VIC_CFG in its own address view).

** BUILD INSTRUCTIONS **

The demo needs an arm-none-eabi GCC toolchain on the PATH:

  make

The firmware image is build/chibios-gemstone-o1-r5f.elf.

** DEPLOY INSTRUCTIONS (on the board's Linux) **

Copy the ELF to the board, then:

  # confirm which instance is the MCU R5F, expected: 79000000.r5f
  head /sys/class/remoteproc/remoteproc*/name

  RP=/sys/class/remoteproc/remoteproc2          # N from the step above
  cp chibios-gemstone-o1-r5f.elf /lib/firmware/
  echo stop > $RP/state 2>/dev/null || true
  echo chibios-gemstone-o1-r5f.elf > $RP/firmware
  echo start > $RP/state

  # watch the ChibiOS log
  cat /sys/kernel/debug/remoteproc/remoteproc2/trace0

The stock firmware in this slot (j722s-mcu-r5f0_0-fw) is TI's RPMsg echo
demo from MCU+ SDK 9.0; nothing on the system depends on it, but keep a
backup before replacing it. Always verify the copy actually landed:

  sudo grep -ao 'ChibiOS[a-zA-Z0-9/. ]*' /lib/firmware/j722s-mcu-r5f0_0-fw

Expected output:

  ChibiOS/RT on T3 Gemstone O1 R5F
  port: ARMv7-R, core: ARM Cortex-R5
  kernel started, tick at 1000 Hz
  alive: main=1 thread=10 fpu=100 fpu_errors=0
  ...

The alive line repeats once per second, thread counts 10x main, fpu 100x,
fpu_errors must stay 0.

** TROUBLESHOOTING **

- "start" fails: check dmesg, the reserved-memory carveouts for the MCU R5F
  in the device tree must cover 0xA1100000-0xA11FFFFF and 0xA1240000 onward
  (mcu_r5fss0_core0_memory_region@a1100000).
- trace0 missing or empty: the resource table was not accepted, check that
  the .resource_table section is present in the ELF (readelf -S).
- No "alive" lines but the header prints: tick interrupt not firing, check
  that MCU_TIMER0 is not claimed/gated by the host (its clock must be left
  running by the bootloader, IRQ 28 at the MCU R5F VIM).
- "alive" lines appear but at the wrong rate: the device tree does not pin
  MCU_TIMER0's clock parent, so AM67_TIMER0_CLK_HZ in board.h may not match
  the mux default. The error will be a clean integer ratio, correct it there.
- Nothing at all, not even the header: read the boot witness blocks written
  by board.c. The core's TCM is visible from Linux at physical 0x79020000
  (BTCM), so the vector witness is at 0x79027FF0 and the fault block at
  0x79027FE0; mmap it via /dev/mem, aligned 4-byte reads only.

** NOTES **

- The D-cache is intentionally left disabled during bring-up (board.c),
  enable it together with cache maintenance once shared-memory paths are
  audited. The I-cache and branch prediction are enabled.
- The MPU is configured with the same region layout as the NuttX am67 port
  plus a non-cacheable window for the remoteproc shared data.
