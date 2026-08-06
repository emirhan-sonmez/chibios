/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    main.c
 * @brief   ChibiOS/RT bring-up demo for the T3 Gemstone O1 Cortex-R5F.
 * @details Two threads increment counters, progress is periodically logged
 *          to the RemoteProc trace buffer and to the SD1 serial port (UART1,
 *          40-pin header pins 8/10, 115200 8N1). Characters received on SD1
 *          are echoed back. On the Linux host:
 *
 *          cat /sys/kernel/debug/remoteproc/remoteproc2/trace0
 */

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "trace.h"
#include "am67_epwm.h"
#include "am67_ecap.h"

static volatile uint32_t thread_counter;
static volatile uint32_t main_counter;
#if CORTEX_USE_FPU == TRUE
static volatile uint32_t fpu_thread_counter;
static volatile uint32_t fpu_error_counter;
#endif

/* Several threads share the console, the mutex keeps their messages from
   interleaving mid-line.*/
static MUTEX_DECL(sd1_mtx);

static void sd1_puts(const char *s) {

  chMtxLock(&sd1_mtx);
  chnWrite(&SD1, (const uint8_t *)s, strlen(s));
  chMtxUnlock(&sd1_mtx);
}

/* Prints a hundredths-of-a-unit fixed point value as "-12.34". Avoids
   pulling floating point and printf into the demo for two numbers.*/
static void sd1_putcenti(int32_t centi) {
  char out[16];
  uint32_t whole, frac, u;
  unsigned i = 0U, j;

  if (centi < 0) {
    out[i++] = '-';
    u = (uint32_t)(-centi);
  }
  else {
    u = (uint32_t)centi;
  }
  whole = u / 100U;
  frac  = u % 100U;

  j = i;
  if (whole == 0U) {
    out[i++] = '0';
  }
  else {
    char rev[10];
    unsigned n = 0U;

    while (whole > 0U) {
      rev[n++] = (char)('0' + (whole % 10U));
      whole /= 10U;
    }
    while (n > 0U) {
      out[i++] = rev[--n];
    }
  }
  (void)j;
  out[i++] = '.';
  out[i++] = (char)('0' + (frac / 10U));
  out[i++] = (char)('0' + (frac % 10U));

  chMtxLock(&sd1_mtx);
  chnWrite(&SD1, (const uint8_t *)out, i);
  chMtxUnlock(&sd1_mtx);
}

/* Prints a byte as two hexadecimal digits.*/
static void sd1_puthex(uint8_t value) {
  static const char digits[] = "0123456789abcdef";
  char out[2];

  out[0] = digits[(value >> 4) & 0x0FU];
  out[1] = digits[value & 0x0FU];

  chMtxLock(&sd1_mtx);
  chnWrite(&SD1, (const uint8_t *)out, sizeof out);
  chMtxUnlock(&sd1_mtx);
}

/*===========================================================================*/
/* FlySky i-BUS RC receiver test (FS-iA10B).                                 */
/*===========================================================================*/

/*
 * Set to 0 to restore the interactive console demo. While this is 1 the RX
 * side of SD1 belongs to the i-BUS decoder and EchoThread is NOT started:
 * two readers on one input queue steal each other's bytes.
 *
 * Wiring: receiver i-BUS *servo* output -> Gemstone header pin 10 (UART1 RX),
 * and a shared ground between the receiver supply and the board. Nothing is
 * connected to pin 8 for this test. i-BUS is plain non-inverted UART, so no
 * inverter circuit is needed (unlike SBUS).
 *
 * MEASURE THE SIGNAL PIN FIRST. FlySky receivers run off 5 V and some drive
 * 5 V logic. Pin 10 is a 3.3 V pad; feeding it 5 V can damage it permanently.
 * Meter the idle level against ground before connecting.
 */
#define GEMSTONE_IBUS_TEST      1

#if GEMSTONE_IBUS_TEST

#define IBUS_FRAME_LEN          32U
#define IBUS_HDR0               0x20U
#define IBUS_HDR1               0x40U
/* The frame always carries 14 channel slots; an FS-iA10B drives 10. */
#define IBUS_NUM_CHANNELS       14U

/*===========================================================================*/
/* i-BUS -> PWM passthrough (BENCH ONLY).                                    */
/*===========================================================================*/

/*
 * Manual passthrough for bench validation of the RC -> actuator chain.
 * This is NOT a flight controller: there is no mixer, no attitude
 * stabilisation, no ArduPilot arming state machine, and no vehicle
 * failsafe logic. Do not fly this.
 *
 * SAFETY, all mandatory:
 *   - PROPELLERS OFF.
 *   - ESC power from a separate BEC/battery, never from a board header.
 *   - ESC supply ground bonded to board ground.
 *   - Airframe restrained.
 *
 * Interlocks implemented below:
 *   - outputs are held at PT_IDLE_US until explicitly armed;
 *   - arming requires the arm switch high AND throttle already at minimum,
 *     so flipping the switch with throttle up cannot spin a motor;
 *   - loss of valid i-BUS frames for PT_FAILSAFE_MS disarms and idles.
 *
 * KNOWN GAP, read this: the FS-iA10B keeps streaming frames with the last
 * values held when the transmitter is switched off, so the frame-timeout
 * failsafe below does NOT catch transmitter loss -- it only catches the
 * receiver being unplugged or dying. Transmitter-loss protection must be
 * configured in the transmitter itself (RX Setup -> Failsafe, throttle to
 * minimum) and verified before any motor is connected.
 *
 * There is also no watchdog: if the R5F faults, the PWM peripherals keep
 * emitting the last commanded pulse indefinitely.
 */

#define PT_IDLE_US              1000U
#define PT_MIN_US               1000U
#define PT_MAX_US               2000U
#define PT_ARM_HIGH_US          1700U   /* arm switch considered ON above    */
#define PT_ARM_LOW_US           1300U   /* ...and OFF below (hysteresis)     */
#define PT_THR_MIN_GATE_US      1050U   /* throttle must be under this to arm*/
/*
 * Throttle at or below this means the transmitter link is gone, not that the
 * pilot is idling: it sits BELOW the receiver's normal 1000us minimum, so a
 * resting stick cannot reach it. Same idea as ArduPilot's FS_THR_VALUE.
 *
 * This only works if transmitter-side failsafe is configured to drive the
 * throttle channel below 1000us on signal loss. Verified on hardware that
 * this FS-iA10B otherwise HOLDS the last value and keeps streaming frames at
 * full rate with the transmitter off -- so neither the frame-timeout below
 * nor any frame-content check can detect the loss on its own.
 */
#define PT_FS_THR_US            950U
#define PT_FAILSAFE_MS          200U    /* no valid frame for this -> idle   */
#define PT_FRAME_HZ             50U

/* i-BUS channel indices (0-based: index 0 == i-BUS channel 1). Confirmed on
   hardware by moving each stick: ch1 roll, ch2 pitch, ch3 throttle (does not
   self-centre), ch4 yaw, ch5..8 switches, ch9/10 knobs. */
#define IB_ROLL                 0U
#define IB_PITCH                1U
#define IB_THR                  2U
#define IB_YAW                  3U
#define IB_ARM                  4U      /* i-BUS channel 5 */

#define PT_NUM_PERIPH           5U
#define PT_NUM_OUT              6U

/*
 * Quad X mixer, bench verification only.
 *
 * THIS IS NOT A FLIGHT CONTROLLER. There is no IMU and no gyro feedback, so
 * nothing corrects attitude: the aircraft would flip the instant it left the
 * ground. The mixer exists so each ESC can be checked for response, correct
 * motor numbering and correct direction with the propellers removed.
 *
 * Motor layout is ArduPilot's Quad X numbering, viewed from ABOVE with the
 * nose pointing away from you:
 *
 *        M3 (CW)        M1 (CCW)          front
 *            \           /
 *             \         /
 *              [ nose ]
 *             /         \
 *            /           \
 *        M2 (CCW)       M4 (CW)           rear
 *
 * Control authority is deliberately limited to PT_MIX_GAIN_PCT of full stick
 * so a stick input cannot swamp the throttle command on the bench.
 *
 * The *_SIGN defines exist because which physical direction a stick's rising
 * value corresponds to depends on transmitter setup. Verify on the bench and
 * flip a sign to -1 if an axis drives the wrong pair of motors.
 */
#define PT_MIX_GAIN_PCT         30      /* % of full stick deflection      */
#define PT_ROLL_SIGN            1
#define PT_PITCH_SIGN           1
#define PT_YAW_SIGN             1
#define PT_NUM_MOTORS           4U

static const struct {
  int16_t roll_f;               /* mixing factors, scaled x1000 */
  int16_t pitch_f;
  int16_t yaw_f;
  uint8_t out;                  /* index into pt_out[] */
} pt_motor[PT_NUM_MOTORS] = {
  { -707,  707,  1000, 0U },    /* M1 front-right, CCW -> out0, pin 29 */
  {  707, -707,  1000, 1U },    /* M2 back-left,   CCW -> out1, pin 31 */
  {  707,  707, -1000, 2U },    /* M3 front-left,  CW  -> out2, pin 33 */
  { -707, -707, -1000, 3U },    /* M4 back-right,  CW  -> out3, pin 32 */
};

static const struct {
  uint32_t base;
  bool     is_ecap;
} pt_periph[PT_NUM_PERIPH] = {
  { AM67_EPWM0_BASE, false },
  { AM67_EPWM1_BASE, false },
  { AM67_ECAP0_BASE, true  },
  { AM67_ECAP1_BASE, true  },
  { AM67_ECAP2_BASE, true  },
};

/* Output -> peripheral/pin map, identical to the verified six-channel
   ChibiOS_K3::RCOutput mapping. */
static const struct {
  uint8_t periph;
  bool    out_b;
  uint8_t pin;
} pt_out[PT_NUM_OUT] = {
  { 0U, false, 29U },   /* EHRPWM0_A */
  { 1U, false, 31U },   /* EHRPWM1_A */
  { 1U, true,  33U },   /* EHRPWM1_B */
  { 2U, false, 32U },   /* ECAP0     */
  { 3U, false, 36U },   /* ECAP1     */
  { 4U, false, 12U },   /* ECAP2     */
};

static bool pt_periph_ok[PT_NUM_PERIPH];
static bool pt_out_ok[PT_NUM_OUT];

static bool pt_wait_timebase(uint32_t base, bool is_ecap, uint32_t max_tries) {
  uint32_t tries;

  for (tries = 0U; tries < max_tries; tries++) {
    uint32_t a = is_ecap ? ecap_read_tsctr(base) : ehrpwm_read_tbctr(base);
    chThdSleepMilliseconds(5);
    if (a != (is_ecap ? ecap_read_tsctr(base) : ehrpwm_read_tbctr(base))) {
      return true;
    }
    if ((tries + 1U) < max_tries) {
      chThdSleepMilliseconds(300);
    }
  }
  return false;
}

static void pt_set(uint8_t out, uint16_t us) {
  uint8_t p;

  if ((out >= PT_NUM_OUT) || !pt_out_ok[out]) {
    return;
  }
  if (us < PT_MIN_US) { us = PT_MIN_US; }
  if (us > PT_MAX_US) { us = PT_MAX_US; }

  p = pt_out[out].periph;
  if (pt_periph[p].is_ecap) {
    ecap_set_pulse_us(pt_periph[p].base, us);
  }
  else {
    ehrpwm_out_set_pulse_us(pt_periph[p].base, pt_out[out].out_b, us);
  }
}

static void pt_all_idle(void) {
  uint8_t i;

  for (i = 0U; i < PT_NUM_OUT; i++) {
    pt_set(i, PT_IDLE_US);
  }
}

/*
 * Bring the outputs up. The PWM time bases are gated by Linux (a pwm channel
 * must be enabled from user space), so each peripheral is polled until its
 * counter actually advances and is skipped if it never starts, rather than
 * writing registers into a dead clock domain.
 */
static bool pt_init(uint32_t tries) {
  uint8_t i;
  bool all_ok = true;

  for (i = 0U; i < PT_NUM_PERIPH; i++) {
    if (pt_periph_ok[i]) {
      continue;                 /* already running, leave it alone */
    }
    if (!pt_wait_timebase(pt_periph[i].base, pt_periph[i].is_ecap, tries)) {
      if (tries > 1U) {
        trace_printf("pt: periph %u clock not running -- enable the Linux PWM "
                     "channels, this will keep retrying\n", (uint32_t)i);
      }
      all_ok = false;
      continue;
    }
    if (pt_periph[i].is_ecap) {
      ecap_start(pt_periph[i].base, PT_FRAME_HZ);
    }
    else {
      ehrpwm_start(pt_periph[i].base, PT_FRAME_HZ);
    }
    pt_periph_ok[i] = true;
  }

  /* Enable each output at 0% first, then park it at idle: enabling an
     output whose compare register is uninitialised can emit an arbitrary
     pulse width. */
  for (i = 0U; i < PT_NUM_OUT; i++) {
    uint8_t p = pt_out[i].periph;

    if (pt_out_ok[i] || !pt_periph_ok[p]) {
      continue;
    }
    if (!pt_periph[p].is_ecap) {
      ehrpwm_out_enable(pt_periph[p].base, pt_out[i].out_b);
    }
    pt_out_ok[i] = true;
    pt_set(i, PT_IDLE_US);
    trace_printf("pt: out%u ready (pin %u) at %u us\n",
                 (uint32_t)i, (uint32_t)pt_out[i].pin, (uint32_t)PT_IDLE_US);
  }
  pt_all_idle();
  return all_ok;
}

/*
 * 4 KiB, not 1 KiB. All CH_DBG_* checks are FALSE in this demo's chconf.h, so
 * a stack overflow is NOT trapped -- it silently corrupts whatever is adjacent
 * and the whole system dies without a message (observed: trace stops dead
 * right after "kernel started", every thread gone, remoteproc still reporting
 * "running"). This thread carries the i-BUS decoder, the quad mixer and a
 * trace_printf with 11 varargs, and the ARMv7-R port saves FPU context on top,
 * so the nominal size was far too close to the edge. DDR is 14 MB; there is no
 * reason to be tight here.
 */
static THD_WORKING_AREA(waIBusThread, 4096);
static THD_FUNCTION(IBusThread, arg) {
  static uint8_t frame[IBUS_FRAME_LEN];
  static uint8_t sample[8];
  /* static, not stack: single instance, and it keeps the per-frame working
     set off a stack that has already bitten us once. */
  static uint16_t ch[IBUS_NUM_CHANNELS];
  static uint16_t motor_us[PT_NUM_MOTORS];
  uint32_t bytes_seen = 0U, frames_ok = 0U, frames_bad = 0U, hdr_seen = 0U;
  uint32_t period_bytes = 0U, period_zeros = 0U, period_ff = 0U;
  uint32_t sample_n = 0U;
  systime_t last_report;
  systime_t last_good_frame;
  bool armed = false;
  bool arm_gate_ok = false;   /* an unconsumed OFF->ON switch edge is available */
  bool pt_outputs_ready;
  bool last_armed = false;
  int32_t last_thr = -1000, last_roll = -1000, last_pitch = -1000,
          last_yaw = -1000;
  uint8_t init_i;

  (void)arg;
  chRegSetThreadName("ibus");

  for (init_i = 0U; init_i < IBUS_NUM_CHANNELS; init_i++) {
    ch[init_i] = 0U;
  }
  for (init_i = 0U; init_i < PT_NUM_MOTORS; init_i++) {
    motor_us[init_i] = PT_IDLE_US;
  }

  /* Explicit config rather than relying on the default, so the baud this
     test runs at is stated in one obvious place. i-BUS is 115200 8N1. */
  {
    SerialConfig cfg = { 115200 };
    sdStart(&SD1, &cfg);
  }

  last_report = chVTGetSystemTimeX();
  last_good_frame = last_report;

  trace_printf("ibus: listening on SD1 RX (header pin 10) @115200 8N1\n");
  trace_printf("ibus: expect 32-byte frames, header 20 40, every ~7.7ms\n");

  pt_outputs_ready = pt_init(16U);
  trace_printf("pt: DISARMED. To arm: throttle DOWN, arm switch OFF then ON. "
               "PROPELLERS OFF.\n");

  while (true) {
    uint8_t b;

    if (chnReadTimeout(&SD1, &b, 1U, TIME_MS2I(50)) != (size_t)1) {
      goto report;
    }
    bytes_seen++;
    period_bytes++;
    if (b == 0x00U) {
      period_zeros++;
    }
    if (b == 0xFFU) {
      period_ff++;
    }
    /* Rolling sample of what is actually on the wire right now, so the
       report below shows live bytes rather than only the first few seen
       at boot. */
    if (sample_n < sizeof sample) {
      sample[sample_n++] = b;
    }

    /* Frame sync on the 2-byte header. */
    if (b != IBUS_HDR0) {
      goto report;
    }
    hdr_seen++;
    frame[0] = b;
    if (chnReadTimeout(&SD1, &frame[1], 1U, TIME_MS2I(50)) != (size_t)1) {
      goto report;
    }
    if (frame[1] != IBUS_HDR1) {
      goto report;
    }
    if (chnReadTimeout(&SD1, &frame[2], IBUS_FRAME_LEN - 2U,
                       TIME_MS2I(50)) != (size_t)(IBUS_FRAME_LEN - 2U)) {
      frames_bad++;
      goto report;
    }

    /* Checksum is 0xFFFF minus the sum of the first 30 bytes, stored LE. */
    {
      uint32_t sum = 0U, i;
      uint16_t want, got;

      for (i = 0U; i < IBUS_FRAME_LEN - 2U; i++) {
        sum += frame[i];
      }
      want = (uint16_t)(0xFFFFU - (sum & 0xFFFFU));
      got  = (uint16_t)frame[30] | (uint16_t)((uint16_t)frame[31] << 8);
      if (want != got) {
        frames_bad++;
        goto report;
      }
    }
    frames_ok++;
    last_good_frame = chVTGetSystemTimeX();

    /* Decode all channels once per valid frame. */
    {
      uint32_t i;
      for (i = 0U; i < IBUS_NUM_CHANNELS; i++) {
        ch[i] = (uint16_t)frame[2U + i * 2U] |
                (uint16_t)((uint16_t)frame[3U + i * 2U] << 8);
      }
    }

    /* RC failsafe by throttle threshold. Requires transmitter-side failsafe
       to be configured to push throttle below 1000us on signal loss --
       without that this can never fire, because the receiver holds the last
       value and the link looks healthy. */
    if (armed && (ch[IB_THR] <= PT_FS_THR_US)) {
      armed = false;
      arm_gate_ok = false;
      pt_all_idle();
      trace_printf("pt: RC FAILSAFE (thr=%u <= %u) -> DISARMED, outputs idle\n",
                   (uint32_t)ch[IB_THR], (uint32_t)PT_FS_THR_US);
    }

    /* Arm state machine, edge triggered: arming happens only on a fresh
       OFF->ON transition of the switch, and only if the throttle is at
       minimum at that instant. Holding the switch on with the throttle up
       therefore cannot arm; the switch must be cycled off and back on with
       the throttle down. The edge is consumed either way, so a refused
       attempt does not silently arm later when the throttle happens to
       drop. */
    if (ch[IB_ARM] < PT_ARM_LOW_US) {
      if (armed) {
        armed = false;
        pt_all_idle();
        trace_printf("pt: DISARMED (arm switch off)\n");
      }
      arm_gate_ok = true;             /* switch is off: an edge is available */
    }
    else if ((ch[IB_ARM] > PT_ARM_HIGH_US) && !armed && arm_gate_ok) {
      arm_gate_ok = false;            /* consume the edge either way */
      if (ch[IB_THR] < PT_THR_MIN_GATE_US) {
        armed = true;
        trace_printf("pt: ARMED. outputs now follow the sticks.\n");
      }
      else {
        trace_printf("pt: ARM REFUSED, throttle %u not at minimum (<%u). "
                     "Lower throttle, switch OFF, then ON again.\n",
                     (uint32_t)ch[IB_THR], (uint32_t)PT_THR_MIN_GATE_US);
      }
    }

    if (armed) {
      uint8_t m;

      if (ch[IB_THR] < PT_THR_MIN_GATE_US) {
        /* Throttle at idle: hold every motor at PT_IDLE_US and apply NO
           mixing. Without this, a full roll or yaw input would raise a
           motor above idle with the throttle closed -- i.e. sticks alone
           could spin a propeller. */
        for (m = 0U; m < PT_NUM_MOTORS; m++) {
          motor_us[m] = PT_IDLE_US;
          pt_set(pt_motor[m].out, PT_IDLE_US);
        }
      }
      else {
        const int32_t thr_off = (int32_t)ch[IB_THR] - (int32_t)PT_MIN_US;
        const int32_t r = ((int32_t)ch[IB_ROLL]  - 1500) * PT_ROLL_SIGN;
        const int32_t p = ((int32_t)ch[IB_PITCH] - 1500) * PT_PITCH_SIGN;
        const int32_t y = ((int32_t)ch[IB_YAW]   - 1500) * PT_YAW_SIGN;

        for (m = 0U; m < PT_NUM_MOTORS; m++) {
          int32_t mix = ((r * pt_motor[m].roll_f) +
                         (p * pt_motor[m].pitch_f) +
                         (y * pt_motor[m].yaw_f)) / 1000;
          int32_t us;

          mix = (mix * PT_MIX_GAIN_PCT) / 100;
          us  = (int32_t)PT_MIN_US + thr_off + mix;

          if (us < (int32_t)PT_MIN_US) { us = (int32_t)PT_MIN_US; }
          if (us > (int32_t)PT_MAX_US) { us = (int32_t)PT_MAX_US; }

          motor_us[m] = (uint16_t)us;
          pt_set(pt_motor[m].out, (uint16_t)us);
        }
      }
      /* out4 (pin 36) and out5 (pin 12) stay at idle -- the pusher motor
         is out of scope for this test. */
    }
    else {
      uint8_t m;
      for (m = 0U; m < PT_NUM_MOTORS; m++) {
        motor_us[m] = PT_IDLE_US;
      }
    }

    /* Report ~1 Hz, not per frame: 130 frames/s would bury the trace
       buffer in seconds. */
report:
    /* Frame-timeout failsafe. Catches the receiver being unplugged or
       dying. Does NOT catch transmitter-off: this receiver keeps streaming
       held values, which is why transmitter-side failsafe must be
       configured and verified separately. */
    if (armed &&
        (chVTTimeElapsedSinceX(last_good_frame) >= TIME_MS2I(PT_FAILSAFE_MS))) {
      armed = false;
      arm_gate_ok = false;
      pt_all_idle();
      trace_printf("pt: FAILSAFE, no valid frame for %ums -> DISARMED, idle\n",
                   (uint32_t)PT_FAILSAFE_MS);
    }

    /*
     * Log on CHANGE, with a slow heartbeat -- not at a fixed 1 Hz. The
     * RemoteProc trace buffer is 16 KiB and does not wrap: trace.c stops
     * accepting once full, so a chatty steady state silently throws away
     * everything that happens later (which is exactly how the first
     * transmitter-off failsafe test got lost). Printing only when the arm
     * state or the throttle actually moves, plus one line every 5 s so a
     * frozen value is still visible, stretches the buffer from ~1 minute
     * to well over ten.
     */
    {
      const systime_t since = chVTTimeElapsedSinceX(last_report);
      /* Watch ALL four stick channels, not just throttle: roll and pitch
         live on the other stick, so a throttle-only trigger left them
         showing stale values between the 5 s heartbeats and made a working
         receiver look dead. */
      static const int32_t move_us = 20;
      const int32_t d_thr   = (int32_t)ch[IB_THR]   - last_thr;
      const int32_t d_roll  = (int32_t)ch[IB_ROLL]  - last_roll;
      const int32_t d_pitch = (int32_t)ch[IB_PITCH] - last_pitch;
      const int32_t d_yaw   = (int32_t)ch[IB_YAW]   - last_yaw;
      const bool moved = (d_thr   >  move_us) || (d_thr   < -move_us) ||
                         (d_roll  >  move_us) || (d_roll  < -move_us) ||
                         (d_pitch >  move_us) || (d_pitch < -move_us) ||
                         (d_yaw   >  move_us) || (d_yaw   < -move_us);
      const bool changed = (armed != last_armed) || moved;

      if (since < TIME_MS2I(500)) {
        continue;                       /* rate limit while sticks move */
      }
      if (!changed && (since < TIME_MS2I(5000))) {
        continue;                       /* idle: heartbeat only */
      }
    }
    last_report = chVTGetSystemTimeX();
    last_armed  = armed;
    last_thr    = (int32_t)ch[IB_THR];
    last_roll   = (int32_t)ch[IB_ROLL];
    last_pitch  = (int32_t)ch[IB_PITCH];
    last_yaw    = (int32_t)ch[IB_YAW];

    /* If the Linux PWM channels were not enabled when we started, keep
       retrying cheaply so enabling them later recovers the outputs without
       needing another reboot cycle. */
    if (!pt_outputs_ready) {
      pt_outputs_ready = pt_init(1U);
      if (pt_outputs_ready) {
        trace_printf("pt: all outputs now live\n");
      }
    }

    if (frames_ok > 0U) {
      /* One compact line. cmp is read straight back from the EPWM0
         hardware, not echoed, so it proves the pin 29 waveform really
         moved: at the 3.125 MHz TBCLK, 1000us -> 3125, 1500us -> 4687,
         2000us -> 6250, with tbprd 62500 throughout. */
      trace_printf("pt: %s thr=%u | m1=%u m2=%u m3=%u m4=%u | r=%u p=%u y=%u | cmp=%u ok=%u bad=%u\n",
                   armed ? "ARMED " : (arm_gate_ok ? "disarm/rdy" : "disarm/cyc"),
                   (uint32_t)ch[IB_THR],
                   (uint32_t)motor_us[0], (uint32_t)motor_us[1],
                   (uint32_t)motor_us[2], (uint32_t)motor_us[3],
                   (uint32_t)ch[IB_ROLL], (uint32_t)ch[IB_PITCH],
                   (uint32_t)ch[IB_YAW],
                   (uint32_t)ehrpwm_read_cmp(AM67_EPWM0_BASE, false),
                   frames_ok, frames_bad);
    }
    else {
      /* No valid frame yet. Report what is actually on the wire so the
         failure mode is identifiable:
           rate 0            -> line dead: wrong pin, no ground, rx off
           rate ~30-100, mostly 00 -> PWM servo channel, not i-BUS (a servo
                                      output idles LOW, which the UART sees
                                      as a continuous framing error)
           rate ~4000, hdr20=0     -> real serial but wrong baud/format
           hdr20 climbing, ok=0    -> framing right, checksum/length wrong */
      trace_printf("ibus: NO FRAMES. rate=%uB/s total=%u zeros=%u ff=%u hdr20=%u bad=%u\n",
                   period_bytes, bytes_seen, period_zeros, period_ff,
                   hdr_seen, frames_bad);
      trace_printf("ibus: live bytes %x %x %x %x %x %x %x %x\n",
                   (uint32_t)sample[0], (uint32_t)sample[1],
                   (uint32_t)sample[2], (uint32_t)sample[3],
                   (uint32_t)sample[4], (uint32_t)sample[5],
                   (uint32_t)sample[6], (uint32_t)sample[7]);
    }
    period_bytes = 0U;
    period_zeros = 0U;
    period_ff    = 0U;
    sample_n     = 0U;
  }
}
#endif /* GEMSTONE_IBUS_TEST */

/*
 * RTOS example thread.
 */
static THD_WORKING_AREA(waThread1, 256);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;

  chRegSetThreadName("counter");

  while (true) {
    chThdSleepMilliseconds(100);
    thread_counter++;
  }
}

/*
 * The SPI and I2C tests run on demand: pressing 's' or 'i' on the serial
 * console signals the matching semaphore, so a test can be started after
 * the Linux side has been prepared (its driver for the shared peripheral
 * unbound).
 */
static BSEMAPHORE_DECL(spi_trigger_bsem, true);
static BSEMAPHORE_DECL(i2c_trigger_bsem, true);

/* Address the 'r' command talks to, the one the bus scan reports.*/
#define I2C_TEST_ADDR           0x70U

/* Compile time of this binary, see the build stamp in main().*/
#define BUILD_DATE              __DATE__
#define BUILD_TIME              __TIME__

/* Which operation the console asked the I2C thread to run.*/
#define I2C_CMD_SCAN            0U
#define I2C_CMD_READ            1U
static volatile uint32_t i2c_command = I2C_CMD_SCAN;

/*
 * UART RX echo thread: any character received on SD1 is sent back. The
 * thread sleeps inside chnGetTimeout() until the UART interrupt pushes a
 * received byte into the input queue, no polling involved.
 */
static THD_WORKING_AREA(waEchoThread, 256);
static THD_FUNCTION(EchoThread, arg) {

  (void)arg;

  chRegSetThreadName("uart-echo");

  while (true) {
    msg_t c = chnGetTimeout(&SD1, TIME_INFINITE);
    if (c >= MSG_OK) {
      if ((char)c == 's') {
        chBSemSignal(&spi_trigger_bsem);
      }
      if ((char)c == 'i') {
        i2c_command = I2C_CMD_SCAN;
        chBSemSignal(&i2c_trigger_bsem);
      }
      if ((char)c == 'r') {
        i2c_command = I2C_CMD_READ;
        chBSemSignal(&i2c_trigger_bsem);
      }
      chnPutTimeout(&SD1, (uint8_t)c, TIME_INFINITE);
    }
  }
}

/*
 * On-demand SPI smoke test (SPID1 = MCU_MCSPI0): an interrupt-driven
 * loopback exchange. With MOSI (D0) jumpered to MISO (D1) every
 * transmitted byte is received back; without the jumper the exchange
 * still completes, which alone validates the RX0_FULL interrupt path
 * through the VIM.
 *
 * The controller is shared with Linux (4b00000.spi): unless the device
 * tree reserves it for the R5F, unbind the Linux driver first or the
 * transfer stalls (its handler clears IRQENABLE on interrupts it
 * considers spurious):
 *
 *   echo 4b00000.spi | sudo tee /sys/bus/platform/drivers/omap2_mcspi/unbind
 */
static THD_WORKING_AREA(waSpiTestThread, 1024);
static THD_FUNCTION(SpiTestThread, arg) {
  static const SPIConfig spicfg = {
    .end_cb   = NULL,
    .speed    = 1000000U,   /* 1 MHz SCLK.*/
    .mode     = 0U          /* CPOL=0 CPHA=0.*/
  };
  static const uint8_t spi_tx[8] =
    {0xA5U, 0x5AU, 0xDEU, 0xADU, 0xBEU, 0xEFU, 0x12U, 0x34U};
  uint8_t spi_rx[8];

  (void)arg;

  chRegSetThreadName("spi-test");

  while (true) {
    sd1_puts("press 's' to run the SPI test\r\n");
    (void) chBSemWait(&spi_trigger_bsem);

    /* The echo of the trigger key sits on the current line.*/
    sd1_puts("\r\n");

    spiStart(&SPID1, &spicfg);
    if (!SPID1.ready) {
      trace_printf("spi: module did not leave reset (clock gated?)\n");
      sd1_puts("SPI module not ready (clock gated?)\r\n");
      continue;
    }

    spiSelect(&SPID1);
    spiExchange(&SPID1, sizeof spi_tx, spi_tx, spi_rx);
    spiUnselect(&SPID1);

    if (memcmp(spi_tx, spi_rx, sizeof spi_tx) == 0) {
      trace_printf("spi: loopback OK\n");
      sd1_puts("SPI loopback OK\r\n");
    }
    else {
      trace_printf("spi: loopback mismatch, rx: %x %x %x %x %x %x %x %x\n",
                   spi_rx[0], spi_rx[1], spi_rx[2], spi_rx[3],
                   spi_rx[4], spi_rx[5], spi_rx[6], spi_rx[7]);
      sd1_puts("SPI loopback MISMATCH (jumper D0-D1 missing?)\r\n");
    }
  }
}

static const char *i2c_init_error_text(uint32_t reason) {

  switch (reason) {
  case I2C_INIT_RESET_TIMEOUT:
    return "module never left reset (clock gated?)";
  case I2C_INIT_TIMING_LOST:
    return "SCL timing wiped by a late reset";
  case I2C_INIT_BAD_FREQUENCY:
    return "requested SCL rate unreachable";
  case I2C_INIT_BUS_STUCK:
    return "bus still held after a STOP (slave stretching?)";
  default:
    return "unknown";
  }
}

/*
 * On-demand I2C bus scan (I2CD1 = MCU_I2C0, header pins 3/5): a 1-byte
 * read is attempted at every 7-bit address, a device that ACKs its
 * address is reported. With nothing attached every address NACKs, which
 * still exercises the full START/address/NACK/STOP interrupt path.
 *
 * If 4900000.i2c is bound to Linux's omap_i2c the two masters fight over
 * the controller, so unbind it first. It is normally NOT bound on this
 * board, and "No such device" from the unbind means there is nothing to
 * do rather than something to fix:
 *
 *   echo 4900000.i2c | sudo tee /sys/bus/platform/drivers/omap_i2c/unbind
 */
/*
 * Reports why a transfer timed out. The controller snapshot separates a
 * bus held by a slave (BB, bit 12 of raw) from a module with no clock
 * configured (scll 0) from an interrupt that was raised but never
 * dispatched (irqen and vim both sane).
 */
static void i2c_report_timeout(void) {

  trace_printf("i2c: timeout con=%x raw=%x scll=%u irqen=%x vim=%x\n",
               I2CD1.dbg.con, I2CD1.dbg.raw, I2CD1.dbg.scll,
               I2CD1.dbg.irqen, I2CD1.dbg.vim);
  sd1_puts("I2C timeout: con=0x");
  sd1_puthex((uint8_t)(I2CD1.dbg.con >> 8));
  sd1_puthex((uint8_t)I2CD1.dbg.con);
  sd1_puts(" raw=0x");
  sd1_puthex((uint8_t)(I2CD1.dbg.raw >> 8));
  sd1_puthex((uint8_t)I2CD1.dbg.raw);
  sd1_puts(" irqen=0x");
  sd1_puthex((uint8_t)(I2CD1.dbg.irqen >> 8));
  sd1_puthex((uint8_t)I2CD1.dbg.irqen);
  sd1_puts(" vim=0x");
  sd1_puthex((uint8_t)I2CD1.dbg.vim);
  sd1_puts("\r\n");
}

/*
 * Sensirion SHTC3 temperature and humidity sensor, the part that answers
 * at 0x70. It takes 16-bit commands rather than a register index, and
 * every reply is 16 bits of data followed by a CRC-8.
 *
 * Talking to it exercises the two-segment transfer path, which the bus
 * scan never touches: the scan is address-only, so it never moves a data
 * byte, never takes the transmit-to-receive phase change on a repeated
 * START and never fills the RX FIFO.
 */
#define SHTC3_CMD_SLEEP         0xB098U
#define SHTC3_CMD_WAKEUP        0x3517U
#define SHTC3_CMD_READ_ID       0xEFC8U
/* Normal mode, temperature first, clock stretching disabled: the result
   is collected after a fixed delay instead of relying on the slave
   holding SCL, which keeps the driver out of clock-stretch handling.*/
#define SHTC3_CMD_MEASURE       0x7866U
#define SHTC3_MEAS_DELAY_MS     20U     /* Datasheet max is 12.1 ms.      */
/* The part number is encoded in bits 11 and 5:0 of the ID word.*/
#define SHTC3_ID_MASK           0x083FU
#define SHTC3_ID_VALUE          0x0807U

static uint8_t shtc3_crc(const uint8_t *data, size_t len) {
  uint8_t crc = 0xFFU;
  size_t i;
  unsigned bit;

  for (i = 0U; i < len; i++) {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++) {
      if ((crc & 0x80U) != 0U) {
        crc = (uint8_t)((crc << 1) ^ 0x31U);
      }
      else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

/* Sends a 16-bit command, optionally reading a reply after a repeated
   START. rxlen 0 sends the command on its own.*/
static msg_t shtc3_command(uint16_t cmd, uint8_t *rxbuf, size_t rxlen) {
  uint8_t tx[2];
  msg_t msg;

  tx[0] = (uint8_t)(cmd >> 8);
  tx[1] = (uint8_t)cmd;

  i2cAcquireBus(&I2CD1);
  msg = i2cMasterTransmitTimeout(&I2CD1, (i2caddr_t)I2C_TEST_ADDR,
                                 tx, sizeof tx, rxbuf, rxlen,
                                 TIME_MS2I(50));
  i2cReleaseBus(&I2CD1);
  return msg;
}

/* Reports a failed step and says which one it was.*/
static void shtc3_report_error(const char *step, msg_t msg) {

  if (msg == MSG_TIMEOUT) {
    sd1_puts("  timeout during ");
    sd1_puts(step);
    sd1_puts("\r\n");
    i2c_report_timeout();
  }
  else {
    sd1_puts("  NACK during ");
    sd1_puts(step);
    sd1_puts("\r\n");
    trace_printf("i2c: shtc3 NACK during %s\n", step);
  }
}

static void i2c_do_read(void) {
  uint8_t rx[6];
  uint16_t id, raw_t, raw_rh;
  int32_t temp_centi, rh_centi;
  msg_t msg;

  /* The sensor powers up asleep and is put back to sleep at the end of
     this function, so every run starts by waking it.*/
  msg = shtc3_command(SHTC3_CMD_WAKEUP, NULL, 0U);
  if (msg != MSG_OK) {
    shtc3_report_error("wakeup", msg);
    return;
  }
  chThdSleepMilliseconds(1);

  msg = shtc3_command(SHTC3_CMD_READ_ID, rx, 3U);
  if (msg != MSG_OK) {
    shtc3_report_error("read id", msg);
    return;
  }
  id = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);

  sd1_puts("  id=0x");
  sd1_puthex(rx[0]);
  sd1_puthex(rx[1]);
  if (shtc3_crc(rx, 2U) != rx[2]) {
    sd1_puts(" CRC BAD (bus noise or wrong part)\r\n");
    trace_printf("i2c: shtc3 id crc bad, id %x\n", id);
    return;
  }
  if ((id & SHTC3_ID_MASK) != SHTC3_ID_VALUE) {
    sd1_puts(" not an SHTC3\r\n");
    trace_printf("i2c: unexpected id %x\n", id);
    return;
  }
  sd1_puts(" SHTC3 confirmed\r\n");

  msg = shtc3_command(SHTC3_CMD_MEASURE, NULL, 0U);
  if (msg != MSG_OK) {
    shtc3_report_error("measure", msg);
    return;
  }
  chThdSleepMilliseconds(SHTC3_MEAS_DELAY_MS);

  /* The conversion result is read back as a plain 6-byte read.*/
  i2cAcquireBus(&I2CD1);
  msg = i2cMasterReceiveTimeout(&I2CD1, (i2caddr_t)I2C_TEST_ADDR,
                                rx, sizeof rx, TIME_MS2I(50));
  i2cReleaseBus(&I2CD1);
  if (msg != MSG_OK) {
    shtc3_report_error("result read", msg);
    return;
  }

  if ((shtc3_crc(&rx[0], 2U) != rx[2]) || (shtc3_crc(&rx[3], 2U) != rx[5])) {
    sd1_puts("  measurement CRC BAD\r\n");
    trace_printf("i2c: shtc3 measurement crc bad\n");
    return;
  }

  raw_t  = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
  raw_rh = (uint16_t)(((uint16_t)rx[3] << 8) | rx[4]);

  /* Datasheet transfer functions, in hundredths to stay in integers:
     T[C] = -45 + 175 * raw / 65536, RH[%] = 100 * raw / 65536.*/
  temp_centi = -4500 + (int32_t)((17500U * (uint32_t)raw_t) / 65536U);
  rh_centi   = (int32_t)((10000U * (uint32_t)raw_rh) / 65536U);

  sd1_puts("  T=");
  sd1_putcenti(temp_centi);
  sd1_puts(" C  RH=");
  sd1_putcenti(rh_centi);
  sd1_puts(" %\r\n");
  trace_printf("i2c: shtc3 t=%d centiC rh=%d centi%%\n",
               (int)temp_centi, (int)rh_centi);

  (void)shtc3_command(SHTC3_CMD_SLEEP, NULL, 0U);
}

static THD_WORKING_AREA(waI2cTestThread, 1024);
static THD_FUNCTION(I2cTestThread, arg) {
  static const I2CConfig i2ccfg = {
    .frequency = 100000U    /* Standard mode, 100 kHz.*/
  };

  (void)arg;

  chRegSetThreadName("i2c-test");

  while (true) {
    unsigned addr, found;

    sd1_puts("press 'i' to scan the I2C bus, 'r' to read from 0x");
    sd1_puthex((uint8_t)I2C_TEST_ADDR);
    sd1_puts("\r\n");
    (void) chBSemWait(&i2c_trigger_bsem);

    /* The echo of the trigger key sits on the current line.*/
    sd1_puts("\r\n");

    i2cStart(&I2CD1, &i2ccfg);
    if (!I2CD1.ready) {
      trace_printf("i2c: init failed, reason %u\n", I2CD1.init_error);
      sd1_puts("I2C init failed: ");
      sd1_puts(i2c_init_error_text(I2CD1.init_error));
      sd1_puts("\r\n");
      continue;
    }

    if (i2c_command == I2C_CMD_READ) {
      sd1_puts("I2C reading...\r\n");
      i2c_do_read();
      continue;
    }

    sd1_puts("I2C scanning...\r\n");
    trace_printf("i2c: scanning\n");
    found = 0U;
    for (addr = 0x08U; addr <= 0x77U; addr++) {
      uint8_t dummy;
      msg_t msg;

      i2cAcquireBus(&I2CD1);
      msg = i2cMasterReceiveTimeout(&I2CD1, (i2caddr_t)addr, &dummy, 1U,
                                    TIME_MS2I(50));
      i2cReleaseBus(&I2CD1);

      if (msg == MSG_OK) {
        trace_printf("i2c: device at %x\n", addr);
        sd1_puts("  device at 0x");
        sd1_puthex((uint8_t)addr);
        sd1_puts("\r\n");
        found++;
      }
      else if (msg == MSG_TIMEOUT) {
        /* A timeout leaves the driver in I2C_LOCKED, only i2cStart()
           recovers it, so the scan cannot usefully continue.*/
        trace_printf("i2c: timeout at address %x\n", addr);
        i2c_report_timeout();
        break;
      }
      /* MSG_RESET is the normal NACK of an empty address.*/
    }
    trace_printf("i2c: scan done, %u device(s)\n", found);
    if (found == 0U) {
      sd1_puts("I2C scan done, no devices found\r\n");
    }
    else {
      sd1_puts("I2C scan done\r\n");
    }
  }
}

#if CORTEX_USE_FPU == TRUE
/*
 * FPU context switching test thread, the d8 marker register must survive
 * every reschedule.
 */
static THD_WORKING_AREA(waThread2, 512);
static THD_FUNCTION(Thread2, arg) {
  register double marker asm ("d8");
  double expected;

  (void)arg;

  chRegSetThreadName("fpu");

  marker = 1000.0;
  expected = 1000.0;

  while (true) {
    __asm volatile ("" : "+w" (marker));
    chThdSleepMilliseconds(10);
    __asm volatile ("" : "+w" (marker));

    if (marker != expected) {
      fpu_error_counter++;
      marker = expected;
    }

    marker += 1.0;
    expected += 1.0;
    fpu_thread_counter++;
  }
}
#endif

/*
 * EPWM0_A bring-up (M4). Drives EHRPWM0_A on the 40-pin header (pin 29,
 * GPIO5 pad muxed by the Linux DT overlay) at a 50 Hz servo/ESC frame,
 * stepping the pulse width through 1000/1500/2000 us every 3 s so the
 * waveform can be checked on a scope. SCOPE ONLY -- do NOT connect an ESC
 * or motor until AM67_EPWM0_CLK_HZ has been calibrated against the measured
 * period, because the 100 MHz clock is still provisional.
 */
static void pwm_dump_regs(const char *when) {
  trace_printf("pwm: [%s] TBCTL=%x TBCTR=%u AQCTLA=%x AQCSFRC=%x TBPRD=%u CMPA=%u\n",
               when,
               (uint32_t)epwm0a_read_tbctl(),
               (uint32_t)epwm0a_read_tbctr(),
               (uint32_t)epwm0a_read_aqctla(),
               (uint32_t)epwm0a_read_aqcsfrc(),
               (uint32_t)epwm0a_read_tbprd(),
               (uint32_t)epwm0a_read_cmpa());
}

static THD_WORKING_AREA(waPwmThread, 256);
static THD_FUNCTION(PwmThread, arg) {
  static const uint32_t pulses_us[] = { 1000U, 1500U, 2000U };
  unsigned i = 0U;
  uint16_t a, b;

  (void)arg;
  chRegSetThreadName("pwm");

  /*
   * Do NOT touch EPWM0 at boot. The epwm_tbclk gate is owned by Linux and is
   * only enabled when a PWM channel is enabled from user space; until then the
   * time-base counter is frozen. Wait until it is actually running by sampling
   * TBCTR twice and detecting that it advances, THEN take over the registers.
   *
   * Prerequisite: the EPWM0 module clock (fck) must be on for these reads to be
   * safe -- keep it active from Linux with
   *   echo on > /sys/bus/platform/devices/23000000.pwm/power/control
   * and enable a channel (which turns the tbclk gate on and retains it).
   */
  chThdSleepMilliseconds(1000);   /* let Linux finish probing EPWM0 first */
  trace_printf("pwm: waiting for EPWM0 time base (enable a Linux pwm channel)\n");

  for (;;) {
    a = epwm0a_read_tbctr();
    chThdSleepMilliseconds(5);    /* 5 ms << 20 ms frame: counter moves a lot */
    b = epwm0a_read_tbctr();
    if (a != b) {
      break;                      /* TBCTR advancing -> tbclk gate is running */
    }
    trace_printf("pwm: TBCTR frozen (a=%u b=%u), tbclk not enabled yet\n",
                 (uint32_t)a, (uint32_t)b);
    chThdSleepMilliseconds(500);
  }
  trace_printf("pwm: TBCTR advancing (a=%u b=%u) -> R5F taking over EPWM0\n",
               (uint32_t)a, (uint32_t)b);

  /* Register state as Linux left it (before we program anything). */
  pwm_dump_regs("before start");

  /* Take over the time base: our prescale/period/action-qualifier, 0% duty
     first (rest low), then cycle the pulse widths. The tbclk gate stays on
     because Linux keeps its channel enabled. */
  epwm0a_start(50U);              /* 50 Hz -> 20000 us frame. */

  /* Register state after our programming. Expect TBCTL=e80, AQCTLA=12,
     AQCSFRC=0 (force released), TBPRD=62500. */
  pwm_dump_regs("after start");

  while (true) {
    epwm0a_set_pulse_us(pulses_us[i]);
    pwm_dump_regs("after set_pulse");
    trace_printf("pwm: EPWM0_A frame=20000us pulse=%u us\n", pulses_us[i]);
    chThdSleepMilliseconds(3000);
    i = (i + 1U) % 3U;
  }
}

/*
 * Application entry point.
 */
int main(void) {

  trace_init();
  trace_printf("ChibiOS/RT on %s\n", BOARD_NAME);
  trace_printf("port: %s, core: %s\n",
               PORT_ARCHITECTURE_NAME, PORT_CORE_VARIANT_NAME);
  /* Build stamp: remoteproc cannot stop this core without a mailbox
     handshake we do not implement, so "cp + start" silently keeps the
     OLD image running. Printing when this binary was compiled makes a
     stale firmware obvious instead of costing a debugging round.*/
  trace_printf("build: %s %s\n", BUILD_DATE, BUILD_TIME);

  /*
   * HAL initialization: platform (VIM), drivers (SD1 object), board hook
   * and the ST tick timer, in that order.
   */
  halInit();

  /*
   * System initialization, the main() function becomes a thread and the
   * RTOS is active.
   */
  chSysInit();

  /*
   * Activates SD1 (UART1) with the default configuration (115200 8N1).
   */
  trace_printf("uart: starting SD1\n");
  sdStart(&SD1, NULL);
#if GEMSTONE_IBUS_TEST
  /* RX-only in this mode: the i-BUS decoder reads the receiver and reports
     via trace0, nothing needs the TX side. Deliberately NOT writing to the
     console here -- sd1_puts() is a blocking chnWrite, and the THR-empty
     interrupt on this UART is not firing (confirmed: am67_uart1_thre_count
     stays 0), so a blocking TX write can wedge main() before any thread is
     ever created. */
  trace_printf("uart: SD1 started, RX-only (no console TX in i-BUS mode)\n");
#else
  sd1_puts("SD1 started from the ChibiOS HAL\r\n");
  trace_printf("uart: first message sent\n");
#endif

  /*
   * SPI test runs in its own thread so that a stuck transfer suspends
   * only that thread, the alive messages keep flowing either way.
   */
#if !GEMSTONE_IBUS_TEST
  /* Both of these report through sd1_puts() (blocking UART TX). Irrelevant
     to an RC receiver test and a hang risk while THRE is not firing. */
  (void) chThdCreateStatic(waSpiTestThread,
                           sizeof(waSpiTestThread),
                           NORMALPRIO,
                           SpiTestThread,
                           NULL);

  (void) chThdCreateStatic(waI2cTestThread,
                           sizeof(waI2cTestThread),
                           NORMALPRIO,
                           I2cTestThread,
                           NULL);
#endif

  trace_printf("kernel started, tick at %u Hz\n",
               (uint32_t)CH_CFG_ST_FREQUENCY);

  (void) chThdCreateStatic(waThread1,
                           sizeof(waThread1),
                           NORMALPRIO,
                           Thread1,
                           NULL);

#if GEMSTONE_IBUS_TEST
  /* i-BUS decoder owns SD1 RX; EchoThread would steal its bytes. */
  (void) chThdCreateStatic(waIBusThread,
                           sizeof(waIBusThread),
                           NORMALPRIO,
                           IBusThread,
                           NULL);
#else
  (void) chThdCreateStatic(waEchoThread,
                           sizeof(waEchoThread),
                           NORMALPRIO,
                           EchoThread,
                           NULL);
#endif

#if !GEMSTONE_IBUS_TEST
  /* Not started during the i-BUS test: it is irrelevant to an RC receiver
     check, it prints "TBCTR frozen" twice a second forever when the Linux
     PWM clocks are not enabled (burying the i-BUS output in trace0), and
     its EPWM register reads are only safe once Linux has turned the module
     clock on. */
  (void) chThdCreateStatic(waPwmThread,
                           sizeof(waPwmThread),
                           NORMALPRIO,
                           PwmThread,
                           NULL);
#endif

#if CORTEX_USE_FPU == TRUE
  (void) chThdCreateStatic(waThread2,
                           sizeof(waThread2),
                           NORMALPRIO,
                           Thread2,
                           NULL);
#endif

  while (true) {
    /* Beacon slowed right down during the i-BUS test so it does not compete
       with the decoder for the 16 KiB trace buffer. */
#if GEMSTONE_IBUS_TEST
    chThdSleepMilliseconds(10000);
#else
    chThdSleepMilliseconds(1000);
#endif
    main_counter++;

    /* Health beacon goes to trace0 only: the interactive serial console
       is for typed commands and their echo/results, a message every
       second there would bury both under a constant flood.*/
#if CORTEX_USE_FPU == TRUE
    trace_printf("alive: main=%u thread=%u fpu=%u fpu_errors=%u\n",
                 main_counter, thread_counter,
                 fpu_thread_counter, fpu_error_counter);
#else
    trace_printf("alive: main=%u thread=%u\n", main_counter, thread_counter);
#endif
  }
}
