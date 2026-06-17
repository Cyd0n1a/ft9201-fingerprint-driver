/*
 * FocalTech FT9201 Fingerprint Reader — libfprint User-Space Driver
 * USB ID: 2808:93a9  (FT9361 sensor variant)
 *
 * Copyright (C) 2024 Contributors
 * Ported from the Linux kernel module by Mak Krnic <mak@banianitc.com>
 * Kernel source: https://github.com/banianitc/ft9201-fingerprint-driver
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * ============================================================================
 * PORTING NOTES — KERNEL MODULE TO LIBFPRINT
 * ============================================================================
 *
 * All blocking usb_control_msg_send/recv() calls become async
 * fpi_usb_transfer_submit() calls that advance the FpiSsm in their callback.
 *
 * All msleep() / wait_event_interruptible_timeout() calls become either
 * fpi_ssm_next_state_delayed() (for fixed waits inside a sequential flow)
 * or g_timeout_add() (for retry-loops that must jump back to a prior state).
 *
 * The kernel module's two main blocking sequences map to two FpiSsm machines:
 *   ft9201_initialize()  → init_ssm     (driven from ft9201_open)
 *   ft9201_read() loop   → capture_ssm  (driven from ft9201_activate)
 *
 * ============================================================================
 * EXTRACTED USB PROTOCOL — COMPLETE REFERENCE
 * ============================================================================
 *
 * All vendor control transfers use:
 *   bmRequestType IN  = 0xC0  (USB_DIR_IN  | USB_TYPE_VENDOR | USB_RECIP_DEVICE)
 *   bmRequestType OUT = 0x40  (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE)
 *
 * ── IN REQUESTS (Device → Host) ─────────────────────────────────────────────
 *
 *  bRequest  wValue  wIndex        wLength  Purpose / Response
 *  ────────  ──────  ────────────  ───────  ─────────────────────────────────
 *  0x1a      0x0000  0x0000        4        GET_SUI_VERSION
 *                                           Response[0..1]: SUI version (u16 LE)
 *                                           Response[2..3]: discarded
 *
 *  0x3a      0x0000  <reg_index>   4        READ_REGISTERS
 *                                           Response[0]: register value
 *                                           Response[1]: only used for REG_MCU_STATUS
 *                                           Response[2..3]: unused
 *
 *  0x43      0x0000  0x0000        1        GET_SENSOR_INT_PORT_STATES
 *                                           Response[0]: interrupt port state byte
 *                                           (Note: not used in the primary init/
 *                                            capture flow; reserved for alternate
 *                                            polling strategies)
 *
 * ── OUT REQUESTS (Host → Device) ────────────────────────────────────────────
 *
 *  bRequest  wValue         wIndex        wLength  Purpose
 *  ────────  ─────────────  ────────────  ───────  ─────────────────────────
 *  0x22      0x0070         0x0070        0        SENSOR_MODE_EXIT
 *                                                  Called ×2, 20ms apart.
 *                                                  Wakes the MCU from sensor
 *                                                  scan mode into command mode.
 *
 *  0x34      0x00ff         0x0000        0        START_CAPTURE step 1
 *  0x34      0x0003         0x0000        0        START_CAPTURE step 2
 *                                                  (issued 20ms after step 1)
 *
 *  0x35      <img_size+2>   0x3400        0        CONFIGURE_BULK_XFER
 *                                                  wValue = width*height + 2
 *                                                  Tells device how many bytes
 *                                                  to stage for the bulk IN.
 *
 *  0x3b      <value & 0xFF> <reg_index>   0        WRITE_REGISTER
 *                                                  Single-byte register write.
 *
 * ── AFE REGISTER MAP (wIndex for READ_REGISTERS / WRITE_REGISTER) ───────────
 *
 *  Index  R/W  Field               Notes
 *  ─────  ───  ──────────────────  ─────────────────────────────────────────
 *  0x14   R    sensor_width        Physical width in pixels (e.g., 0x40 = 64)
 *  0x15   R    sensor_height       Physical height in pixels (e.g., 0x50 = 80)
 *  0x16   R    chip_id_high        AFE chip ID high byte
 *  0x17   R    chip_id_low         AFE chip ID low byte
 *  0x1a   R    fw_version          Embedded firmware version
 *  0x1d   R    finger_present      0x01 or 0xa0 = finger on sensor
 *  0x1e   W    pwr_ctrl_1e         Write 0x01 during auto-power init
 *  0x1f   W    pwr_ctrl_1f         Write 0x01 during auto-power init
 *  0x20   R    mcu_status          [0]=0xa5 && [1]=0x5a → MCU ready
 *  0x22   W    chip_cfg_a          Write 0x00 (variant 2/3 only)
 *  0x23   W    chip_cfg_b          Write 0x0e (variant 2/3 only)
 *  0x30   R    capture_ready       0xbb = sensor armed for capture
 *  0x3c   R    agc_version         Auto Gain Control version
 *
 * ── KNOWN AFE CHIP IDs ───────────────────────────────────────────────────────
 *
 *  Chip ID  Variant  Name
 *  ───────  ───────  ──────────────────
 *  0x9338     1      FT9338W
 *  0x9536     6      FT9536W
 *  0x95a8     3      FT9361  ← this device (USB 2808:93a9)
 *
 * ── BULK TRANSFER ────────────────────────────────────────────────────────────
 *
 *  Direction : IN (Device → Host)
 *  Endpoint  : First bulk-IN endpoint reported by the device
 *              (kernel uses usb_find_common_endpoints; typically 0x81)
 *  Length    : sensor_width * sensor_height + 2 bytes
 *  Format    : [0x00][0x00][pixel_0]...[pixel_(W*H-1)]
 *               ↑ 2-byte opaque header, always discarded
 *               Remaining bytes: raw 8-bit grayscale, row-major order
 *
 * ============================================================================
 * INITIALIZATION SEQUENCE (ordered)
 * ============================================================================
 *
 *  1.  READ  REG_MCU_STATUS (0x20)           — is MCU already running?
 *  2.  If NOT ready: sensor mode exit sequence:
 *        OUT 0x22 (wV=0x70, wI=0x70)         — first reset command
 *        msleep(20)
 *        OUT 0x22 (wV=0x70, wI=0x70)         — second reset command
 *        msleep(10)
 *        READ REG_MCU_STATUS (0x20)           — verify (result not checked)
 *  3.  READ  REG_CHIP_ID_HIGH (0x16)
 *  4.  READ  REG_CHIP_ID_LOW  (0x17)         — assemble afe_chip_id
 *  5.  READ  REG_SENSOR_WIDTH (0x14)
 *  6.  READ  REG_SENSOR_HEIGHT(0x15)
 *  7.  READ  REG_FW_VERSION   (0x1a)
 *  8.  READ  REG_AGC_VERSION  (0x3c)
 *  9.  READ  REG_SENSOR_WIDTH (0x14)         — second read (mirrors kernel exactly)
 * 10.  READ  REG_SENSOR_HEIGHT(0x15)
 * 11.  If chip_variant == 2 or 3:
 *        WRITE REG_CHIP_CFG_A (0x22) = 0x00
 *        WRITE REG_CHIP_CFG_B (0x23) = 0x0e
 * 12.  Auto power init:
 *        Poll REG_MCU_STATUS (0x20), up to 5×, 10ms apart, until 0xa5,0x5a
 *        WRITE REG_PWR_CTRL_1F (0x1f) = 0x01
 *        WRITE REG_PWR_CTRL_1E (0x1e) = 0x01
 *        msleep(10)
 *        Poll REG_MCU_STATUS (0x20), up to 5×, 1ms apart, until 0xa5,0x5a
 *        READ  REG_FINGER_PRESENT (0x1d)     — value not checked, required
 *        READ  REG_MCU_STATUS (0x20)         — final confirmation
 * 13.  → fpi_image_device_open_complete(NULL)
 *
 * ============================================================================
 * CAPTURE SEQUENCE (ordered, loops until deactivated)
 * ============================================================================
 *
 *  1.  Poll REG_MCU_STATUS (0x20) until 0xa5,0x5a      (100ms between polls)
 *  2.  READ REG_CAPTURE_READY (0x30); retry if != 0xbb (50ms between retries)
 *  3.  READ REG_FINGER_PRESENT (0x1d)
 *      If neither 0x01 nor 0xa0: go back to step 1
 *  4.  fpi_image_device_report_finger_status(TRUE)
 *  5.  OUT 0x34 (wV=0x00ff)                            — capture start step 1
 *      msleep(20)
 *  6.  OUT 0x34 (wV=0x0003)                            — capture start step 2
 *  7.  OUT 0x35 (wV=width*height+2, wI=0x3400)         — configure bulk
 *  8.  Bulk IN: width*height+2 bytes
 *      Skip 2-byte header; copy remaining into FpImage->data
 *  9.  fpi_image_device_image_captured(image)
 * 10.  fpi_image_device_report_finger_status(FALSE)
 * 11.  → loop back to step 1
 */

#define FP_COMPONENT "ft9201"

#include <string.h>

#include "drivers_api.h"


/* ============================================================================
 * USB Protocol Constants
 * ============================================================================ */

/* bRequest codes — IN (Device → Host) */
#define FT9201_REQ_GET_SUI_VERSION         0x1a
#define FT9201_REQ_READ_REGISTERS          0x3a
#define FT9201_REQ_GET_INT_PORT_STATES     0x43

/* bRequest codes — OUT (Host → Device) */
#define FT9201_REQ_SENSOR_MODE_EXIT        0x22
#define FT9201_REQ_START_CAPTURE           0x34
#define FT9201_REQ_CONFIGURE_BULK_XFER     0x35
#define FT9201_REQ_WRITE_REGISTER          0x3b

/* AFE register indices (used as wIndex with READ/WRITE_REGISTER) */
#define FT9201_REG_SENSOR_WIDTH            0x14
#define FT9201_REG_SENSOR_HEIGHT           0x15
#define FT9201_REG_CHIP_ID_HIGH            0x16
#define FT9201_REG_CHIP_ID_LOW             0x17
#define FT9201_REG_FW_VERSION              0x1a
#define FT9201_REG_FINGER_PRESENT          0x1d
#define FT9201_REG_PWR_CTRL_1E             0x1e
#define FT9201_REG_PWR_CTRL_1F             0x1f
#define FT9201_REG_MCU_STATUS              0x20
#define FT9201_REG_CHIP_CFG_A              0x22
#define FT9201_REG_CHIP_CFG_B              0x23
#define FT9201_REG_CAPTURE_READY           0x30
#define FT9201_REG_AGC_VERSION             0x3c

/* Magic response values */
#define FT9201_MCU_READY_BYTE0             0xa5  /* reg_buf[0] when MCU ready */
#define FT9201_MCU_READY_BYTE1             0x5a  /* reg_buf[1] when MCU ready */
#define FT9201_CAPTURE_ARMED_MAGIC         0xbb  /* REG_CAPTURE_READY when armed */
#define FT9201_FINGER_VAL_A                0x01  /* REG_FINGER_PRESENT: finger on */
#define FT9201_FINGER_VAL_B                0xa0  /* REG_FINGER_PRESENT: finger on (alt) */

/* Known AFE chip IDs */
#define FT9201_AFE_ID_FT9338W              0x9338  /* chip_variant = 1 */
#define FT9201_AFE_ID_FT9536W              0x9536  /* chip_variant = 6 */
#define FT9201_AFE_ID_FT9361               0x95a8  /* chip_variant = 3, this device */

/* wValue / wIndex for the SENSOR_MODE_EXIT command (both fields = 0x70) */
#define FT9201_SENSOR_EXIT_PARAM           0x70

/* Bulk IN endpoint address — FALLBACK ONLY.
 *
 * ft9201_open() discovers the real bulk-IN endpoint at runtime from the
 * claimed interface's descriptors (see discover_bulk_in_endpoint() below),
 * mirroring what the kernel driver does with usb_find_common_endpoints().
 * This constant is only used if that discovery fails for some reason, so
 * the driver still has *something* to try rather than refusing to open.
 * Verify the discovered value against `lsusb -v -d 2808:93a9` on first run;
 * a debug-level log line reports whichever address was actually selected. */
#define FT9201_EP_BULK_IN_FALLBACK          0x81

/* Image geometry for the FT9361 sensor (USB ID 2808:93a9).
 * Actual dimensions are also read from REG_SENSOR_WIDTH/HEIGHT at runtime. */
#define FT9201_IMAGE_WIDTH                 64
#define FT9201_IMAGE_HEIGHT                80

/* The bulk IN payload includes a 2-byte opaque header before pixel data */
#define FT9201_BULK_HEADER_SIZE            2

/* Timing constants (milliseconds) */
#define FT9201_CTRL_TIMEOUT_MS             1000
#define FT9201_BULK_TIMEOUT_MS             1000
#define FT9201_SENSOR_EXIT_STEP_DELAY_MS   20    /* Between exit commands */
#define FT9201_SENSOR_EXIT_POST_DELAY_MS   10    /* After second exit command */
#define FT9201_POWER_SETTLE_DELAY_MS       10    /* After writing power regs */
#define FT9201_CAPTURE_CMD_GAP_MS          20    /* Between start_capture 1 and 2 */
#define FT9201_MCU_POLL_DELAY_MS           10    /* Between MCU ready polls (phase 1) */
#define FT9201_MCU_POLL2_DELAY_MS          1     /* Between MCU ready polls (phase 2) */
#define FT9201_CAPTURE_MCU_POLL_DELAY_MS   100   /* Between MCU polls during capture */
#define FT9201_CAPTURE_ARMED_RETRY_MS      50    /* Between capture-ready polls */

/* Maximum poll retry attempts (mirrors kernel AUTO_POWER_MAX_RETRY_COUNT) */
#define FT9201_MCU_POLL_MAX_RETRIES        5


/* ============================================================================
 * Device Private Data
 * ============================================================================ */

/**
 * FpiDeviceFt9201:
 *
 * Per-instance state. Hardware fields are populated during init_ssm and
 * remain valid for the lifetime of the open device.
 */
struct _FpiDeviceFt9201
{
  FpImageDevice parent;

  /* Hardware state — populated by init_ssm */
  guint16  afe_chip_id;      /**< Chip ID: (REG_CHIP_ID_HIGH << 8) | REG_CHIP_ID_LOW */
  guint8   chip_variant;     /**< 1=FT9338W  2=FT9348W  3=FT9361  6=FT9536W */
  guint8   sensor_width;     /**< Pixels per row  (REG_SENSOR_WIDTH)  */
  guint8   sensor_height;    /**< Rows per frame  (REG_SENSOR_HEIGHT) */
  guint8   fw_version;       /**< Embedded firmware version (REG_FW_VERSION) */
  guint8   agc_version;      /**< AGC version (REG_AGC_VERSION) */

  /* Scratch buffer for 4-byte register reads (REQ_READ_REGISTERS response) */
  guint8   reg_buf[4];

  /* Retry counter reused by both MCU poll loops in init_ssm */
  gint     poll_retry_count;

  /* Handle to the active state machine so ft9201_deactivate can cancel it */
  FpiSsm  *task_ssm;

  /* Bulk-IN endpoint address, resolved once at open() time by
   * discover_bulk_in_endpoint(). Falls back to FT9201_EP_BULK_IN_FALLBACK
   * if discovery doesn't turn up exactly one candidate. */
  guint8   bulk_in_ep;

  /* The USB transfer currently in flight for capture_ssm, or NULL if none.
   * ft9201_deactivate() cancels this directly so a stuck poll doesn't run
   * past deactivation. Only used by capture_ssm — init_ssm transfers run
   * during open(), which deactivate() never interrupts. */
  FpiUsbTransfer *active_transfer;

  /* GLib source id of a pending g_timeout_add() retry/delay in capture_ssm,
   * or 0 if none is pending. Needed because a scheduled retry is not a USB
   * transfer and is therefore invisible to the device's GCancellable —
   * ft9201_deactivate() must remove it explicitly via g_source_remove(). */
  guint    retry_source_id;
};

G_DECLARE_FINAL_TYPE (FpiDeviceFt9201,
                      fpi_device_ft9201,
                      FPI, DEVICE_FT9201,
                      FpImageDevice);

G_DEFINE_TYPE (FpiDeviceFt9201, fpi_device_ft9201, FP_TYPE_IMAGE_DEVICE);


/* ============================================================================
 * Transfer Builder Helpers
 *
 * All vendor control transfers share the same bmRequestType base:
 *   IN : G_USB_DEVICE_DIRECTION_DEVICE_TO_HOST  | VENDOR | DEVICE
 *   OUT: G_USB_DEVICE_DIRECTION_HOST_TO_DEVICE  | VENDOR | DEVICE
 * ============================================================================ */

/**
 * ctrl_in_transfer:
 * @device:   the #FpDevice instance
 * @bRequest: vendor bRequest code
 * @wValue:   wValue field (typically 0x0000 for reads)
 * @wIndex:   wIndex field (register address for READ_REGISTERS)
 * @length:   number of response bytes expected
 *
 * Allocates an IN control transfer. Caller sets ->ssm and calls
 * fpi_usb_transfer_submit(). Ownership passes to the caller.
 */
static FpiUsbTransfer *
ctrl_in_transfer (FpDevice *device,
                  guint8    bRequest,
                  guint16   wValue,
                  guint16   wIndex,
                  guint16   length)
{
  FpiUsbTransfer *t = fpi_usb_transfer_new (device);

  fpi_usb_transfer_fill_control (t,
                                 G_USB_DEVICE_DIRECTION_DEVICE_TO_HOST,
                                 G_USB_DEVICE_REQUEST_TYPE_VENDOR,
                                 G_USB_DEVICE_RECIPIENT_DEVICE,
                                 bRequest, wValue, wIndex, length);
  return t;
}

/**
 * ctrl_out_transfer:
 * @device:   the #FpDevice instance
 * @bRequest: vendor bRequest code
 * @wValue:   wValue field (often the register value to write)
 * @wIndex:   wIndex field (often the register address)
 *
 * Allocates a zero-length OUT control transfer. All kernel OUT requests
 * carry no data body — the command is encoded entirely in wValue/wIndex.
 */
static FpiUsbTransfer *
ctrl_out_transfer (FpDevice *device,
                   guint8    bRequest,
                   guint16   wValue,
                   guint16   wIndex)
{
  FpiUsbTransfer *t = fpi_usb_transfer_new (device);

  fpi_usb_transfer_fill_control (t,
                                 G_USB_DEVICE_DIRECTION_HOST_TO_DEVICE,
                                 G_USB_DEVICE_REQUEST_TYPE_VENDOR,
                                 G_USB_DEVICE_RECIPIENT_DEVICE,
                                 bRequest, wValue, wIndex, 0);
  return t;
}

/**
 * reg_read_transfer:
 * @device:    the #FpDevice instance
 * @reg_index: AFE register address (becomes wIndex)
 *
 * Convenience wrapper for the very common READ_REGISTERS pattern.
 * Always requests 4 bytes; only byte[0] is meaningful for most registers.
 * For REG_MCU_STATUS (0x20), bytes[0] and [1] are both significant.
 */
static inline FpiUsbTransfer *
reg_read_transfer (FpDevice *device, guint8 reg_index)
{
  return ctrl_in_transfer (device,
                           FT9201_REQ_READ_REGISTERS,
                           0x0000, reg_index, 4);
}

/**
 * reg_write_transfer:
 * @device:    the #FpDevice instance
 * @reg_index: AFE register address (wIndex)
 * @value:     byte value to write (placed in wValue & 0xFF)
 *
 * Convenience wrapper for WRITE_REGISTER. The kernel driver masks
 * `value & 0xFF` explicitly; we keep that here for clarity.
 */
static inline FpiUsbTransfer *
reg_write_transfer (FpDevice *device, guint8 reg_index, guint8 value)
{
  return ctrl_out_transfer (device,
                            FT9201_REQ_WRITE_REGISTER,
                            (guint16) (value & 0xFF),
                            (guint16) reg_index);
}


/* ============================================================================
 * Status Parsing Helpers
 * ============================================================================ */

/**
 * mcu_is_ready:
 * @buf: 4-byte buffer from a REG_MCU_STATUS read
 *
 * Returns %TRUE if the MCU signals it is running and ready.
 * The kernel driver checks [0]==0xa5 && [1]==0x5a.
 */
static inline gboolean
mcu_is_ready (const guint8 *buf)
{
  return (buf[0] == FT9201_MCU_READY_BYTE0 &&
          buf[1] == FT9201_MCU_READY_BYTE1);
}

/**
 * finger_is_present:
 * @val: byte[0] from a REG_FINGER_PRESENT read
 *
 * Returns %TRUE if either known "finger on sensor" sentinel is present.
 */
static inline gboolean
finger_is_present (guint8 val)
{
  return (val == FT9201_FINGER_VAL_A || val == FT9201_FINGER_VAL_B);
}


/* ============================================================================
 * Generic Transfer Callbacks
 * ============================================================================ */

/**
 * reg_read_to_buf_cb:
 *
 * Generic callback for any READ_REGISTERS transfer. Copies the 4-byte
 * device response into self->reg_buf and advances the SSM by one state.
 * The next state handler reads reg_buf before issuing its own transfer,
 * so reg_buf is never stale when accessed.
 */
static void
reg_read_to_buf_cb (FpiUsbTransfer *transfer,
                    FpDevice       *device,
                    gpointer        user_data,
                    GError         *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  /* No-op for init_ssm callers (never set); clears the in-flight marker
   * for capture_ssm callers (CAPTURE_CHECK_FINGER) before it completes. */
  self->active_transfer = NULL;

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  memcpy (self->reg_buf, transfer->buffer, 4);
  fpi_ssm_next_state (transfer->ssm);
}

/**
 * ctrl_out_cb:
 *
 * Generic callback for any zero-length OUT control transfer.
 * Simply advances the SSM on success.
 */
static void
ctrl_out_cb (FpiUsbTransfer *transfer,
             FpDevice       *device,
             gpointer        user_data,
             GError         *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  self->active_transfer = NULL;

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  fpi_ssm_next_state (transfer->ssm);
}


/* ============================================================================
 * STATE MACHINE 1: INITIALIZATION (init_ssm)
 *
 * Translates ft9201_initialize() → ft9201_init_auto_power() into an
 * async FpiSsm. Each state issues one USB transfer and delegates
 * to its callback. "Decision" states (those that branch or retry) use
 * named custom callbacks rather than the generic ones above.
 * ============================================================================ */

/**
 * InitState:
 *
 * Ordered set of states for the initialisation state machine.
 * States labelled "→ next" just call fpi_ssm_next_state(); states labelled
 * with a jump target call fpi_ssm_jump_to_state() to skip or retry blocks.
 */
typedef enum
{
  /* ── Phase 1: Initial MCU check ─────────────────────────────────────────── */
  INIT_CHECK_MCU,              /**< READ REG_MCU_STATUS; branch on result      */

  /* ── Phase 2: Sensor mode exit (entered only when Phase 1 finds MCU asleep) */
  INIT_SENSOR_EXIT_CMD_1,      /**< OUT 0x22 (first of two)                    */
  INIT_SENSOR_EXIT_DELAY_1,    /**< 20ms settle                                */
  INIT_SENSOR_EXIT_CMD_2,      /**< OUT 0x22 (second of two)                   */
  INIT_SENSOR_EXIT_DELAY_2,    /**< 10ms settle                                */
  INIT_SENSOR_EXIT_VERIFY,     /**< READ REG_MCU_STATUS (result not checked)   */

  /* ── Phase 3: Hardware identification ───────────────────────────────────── */
  INIT_READ_CHIP_ID_HIGH,      /**< READ REG_CHIP_ID_HIGH (0x16)               */
  INIT_READ_CHIP_ID_LOW,       /**< READ REG_CHIP_ID_LOW  (0x17); build chip ID */
  INIT_READ_SENSOR_WIDTH,      /**< READ REG_SENSOR_WIDTH  (0x14)              */
  INIT_READ_SENSOR_HEIGHT,     /**< READ REG_SENSOR_HEIGHT (0x15)              */

  /* ── Phase 4: Version metadata ──────────────────────────────────────────── */
  INIT_READ_FW_VERSION,        /**< READ REG_FW_VERSION (0x1a)                 */
  INIT_READ_AGC_VERSION,       /**< READ REG_AGC_VERSION (0x3c)                */

  /* ── Phase 5: Second dimension read (kernel does this; mirrors it exactly) ─ */
  INIT_READ_SENSOR_WIDTH2,     /**< READ REG_SENSOR_WIDTH  (0x14) again        */
  INIT_READ_SENSOR_HEIGHT2,    /**< READ REG_SENSOR_HEIGHT (0x15) again        */

  /* ── Phase 6: Chip-variant config writes (FT9361 / FT9348W only) ─────────  */
  INIT_WRITE_CFG_A,            /**< WRITE REG_CHIP_CFG_A (0x22) = 0x00        */
  INIT_WRITE_CFG_B,            /**< WRITE REG_CHIP_CFG_B (0x23) = 0x0e        */

  /* ── Phase 7: Auto-power — first MCU poll (up to 5×, 10ms apart) ──────── */
  INIT_POWER_POLL1,            /**< READ MCU_STATUS; retry via g_timeout_add   */
  INIT_POWER_WRITE_1F,         /**< WRITE REG_PWR_CTRL_1F (0x1f) = 0x01       */
  INIT_POWER_WRITE_1E,         /**< WRITE REG_PWR_CTRL_1E (0x1e) = 0x01       */
  INIT_POWER_SETTLE_DELAY,     /**< 10ms settle after power writes             */

  /* ── Phase 8: Auto-power — second MCU poll (up to 5×, 1ms apart) ──────── */
  INIT_POWER_POLL2,            /**< READ MCU_STATUS; retry via g_timeout_add   */
  INIT_POWER_READ_1D,          /**< READ REG_FINGER_PRESENT (0x1d); req'd      */
  INIT_POWER_READ_MCU_FINAL,   /**< READ REG_MCU_STATUS — final confirmation   */

  INIT_NUM_STATES
} InitState;


/* Forward declaration */
static void init_ssm_state (FpiSsm *ssm, FpDevice *device);


/* ── Poll retry callbacks for Phase 7 and Phase 8 ───────────────────────── */

/**
 * retry_power_poll1:
 * GSourceFunc — fired by g_timeout_add after the 10ms delay between
 * consecutive Phase-7 MCU poll attempts. Jumps the SSM back to INIT_POWER_POLL1.
 */
static gboolean
retry_power_poll1 (gpointer user_data)
{
  fpi_ssm_jump_to_state ((FpiSsm *) user_data, INIT_POWER_POLL1);
  return G_SOURCE_REMOVE;
}

/**
 * power_poll1_cb:
 * Callback for the Phase-7 MCU status read.
 *
 *  • MCU ready   → proceed to INIT_POWER_WRITE_1F
 *  • Not ready, retries remain → 10ms then jump back to INIT_POWER_POLL1
 *  • Exhausted   → log warning and proceed anyway (mirrors kernel behaviour)
 */
static void
power_poll1_cb (FpiUsbTransfer *transfer,
                FpDevice       *device,
                gpointer        user_data,
                GError         *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  memcpy (self->reg_buf, transfer->buffer, 4);

  if (mcu_is_ready (self->reg_buf))
    {
      fp_dbg ("init: MCU ready after %d poll(s)", self->poll_retry_count + 1);
      self->poll_retry_count = 0;
      fpi_ssm_next_state (transfer->ssm);   /* → INIT_POWER_WRITE_1F */
    }
  else if (self->poll_retry_count >= FT9201_MCU_POLL_MAX_RETRIES)
    {
      fp_warn ("init: MCU not ready after %d attempts — continuing",
               FT9201_MCU_POLL_MAX_RETRIES);
      self->poll_retry_count = 0;
      fpi_ssm_next_state (transfer->ssm);   /* → INIT_POWER_WRITE_1F */
    }
  else
    {
      self->poll_retry_count++;
      g_timeout_add (FT9201_MCU_POLL_DELAY_MS, retry_power_poll1, transfer->ssm);
    }
}

/**
 * retry_power_poll2:
 * GSourceFunc — 1ms delay between Phase-8 poll attempts.
 */
static gboolean
retry_power_poll2 (gpointer user_data)
{
  fpi_ssm_jump_to_state ((FpiSsm *) user_data, INIT_POWER_POLL2);
  return G_SOURCE_REMOVE;
}

/**
 * power_poll2_cb:
 * Callback for the Phase-8 MCU status read (post-power-write).
 * Identical logic to power_poll1_cb but with a 1ms retry gap.
 */
static void
power_poll2_cb (FpiUsbTransfer *transfer,
                FpDevice       *device,
                gpointer        user_data,
                GError         *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  memcpy (self->reg_buf, transfer->buffer, 4);

  if (mcu_is_ready (self->reg_buf))
    {
      fp_dbg ("init: MCU ready after power writes (%d poll(s))",
              self->poll_retry_count + 1);
      self->poll_retry_count = 0;
      fpi_ssm_next_state (transfer->ssm);   /* → INIT_POWER_READ_1D */
    }
  else if (self->poll_retry_count >= FT9201_MCU_POLL_MAX_RETRIES)
    {
      fp_warn ("init: MCU not ready after power writes — continuing");
      self->poll_retry_count = 0;
      fpi_ssm_next_state (transfer->ssm);   /* → INIT_POWER_READ_1D */
    }
  else
    {
      self->poll_retry_count++;
      g_timeout_add (FT9201_MCU_POLL2_DELAY_MS, retry_power_poll2, transfer->ssm);
    }
}

/**
 * check_mcu_cb:
 * Specialised callback for INIT_CHECK_MCU. Unlike reg_read_to_buf_cb,
 * this branches the SSM: if the MCU is already running it skips the entire
 * Phase-2 sensor-exit block and jumps directly to INIT_READ_CHIP_ID_HIGH.
 */
static void
check_mcu_cb (FpiUsbTransfer *transfer,
              FpDevice       *device,
              gpointer        user_data,
              GError         *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  memcpy (self->reg_buf, transfer->buffer, 4);

  if (mcu_is_ready (self->reg_buf))
    {
      fp_dbg ("init: MCU already running — skipping sensor mode exit");
      fpi_ssm_jump_to_state (transfer->ssm, INIT_READ_CHIP_ID_HIGH);
    }
  else
    {
      fp_dbg ("init: MCU not running — entering sensor mode exit");
      fpi_ssm_next_state (transfer->ssm);  /* → INIT_SENSOR_EXIT_CMD_1 */
    }
}

/**
 * init_ssm_state:
 * @ssm:    the active #FpiSsm
 * @device: our #FpDevice
 *
 * State handler for init_ssm. Each case issues one USB transfer (or a delay)
 * that advances the SSM when complete. Data accumulated in self->reg_buf and
 * hardware fields is consumed at the start of the *next* state.
 */
static void
init_ssm_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);
  FpiUsbTransfer  *xfer;

  switch ((InitState) fpi_ssm_get_cur_state (ssm))
    {

    /* ── Phase 1 ──────────────────────────────────────────────────────────── */

    case INIT_CHECK_MCU:
      fp_dbg ("init: phase 1 — checking MCU status");
      xfer = reg_read_transfer (device, FT9201_REG_MCU_STATUS);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, check_mcu_cb, NULL);
      break;

    /* ── Phase 2 ──────────────────────────────────────────────────────────── */

    case INIT_SENSOR_EXIT_CMD_1:
      fp_dbg ("init: sensor mode exit (1/2)");
      xfer = ctrl_out_transfer (device,
                                FT9201_REQ_SENSOR_MODE_EXIT,
                                FT9201_SENSOR_EXIT_PARAM,
                                FT9201_SENSOR_EXIT_PARAM);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, ctrl_out_cb, NULL);
      break;

    case INIT_SENSOR_EXIT_DELAY_1:
      fpi_ssm_next_state_delayed (ssm, FT9201_SENSOR_EXIT_STEP_DELAY_MS);
      break;

    case INIT_SENSOR_EXIT_CMD_2:
      fp_dbg ("init: sensor mode exit (2/2)");
      xfer = ctrl_out_transfer (device,
                                FT9201_REQ_SENSOR_MODE_EXIT,
                                FT9201_SENSOR_EXIT_PARAM,
                                FT9201_SENSOR_EXIT_PARAM);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, ctrl_out_cb, NULL);
      break;

    case INIT_SENSOR_EXIT_DELAY_2:
      fpi_ssm_next_state_delayed (ssm, FT9201_SENSOR_EXIT_POST_DELAY_MS);
      break;

    case INIT_SENSOR_EXIT_VERIFY:
      xfer = reg_read_transfer (device, FT9201_REG_MCU_STATUS);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, reg_read_to_buf_cb, NULL);
      break;

    /* ── Phase 3 ──────────────────────────────────────────────────────────── */

    case INIT_READ_CHIP_ID_HIGH:
      fp_dbg ("init: phase 3 — reading AFE chip ID");
      xfer = reg_read_transfer (device, FT9201_REG_CHIP_ID_HIGH);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, reg_read_to_buf_cb, NULL);
      break;

    case INIT_READ_CHIP_ID_LOW:
      /*
       * reg_buf[0] holds the high byte from INIT_READ_CHIP_ID_HIGH.
       * Stash it in afe_chip_id now; the low byte will be ORed in at
       * INIT_READ_SENSOR_WIDTH (the next state after this read completes).
       */
      self->afe_chip_id = (guint16) (self->reg_buf[0] << 8);
      xfer = reg_read_transfer (device, FT9201_REG_CHIP_ID_LOW);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, reg_read_to_buf_cb, NULL);
      break;

    case INIT_READ_SENSOR_WIDTH:
      {
        /*
         * Combine the chip ID and derive the chip variant.
         * Variant 3 (FT9361) is the expected target for USB 2808:93a9.
         * The kernel module also sets variant=3 when sensor dims are
         * NOT 0x60×0x60 for chip_id 0x95a8; we derive it from the ID
         * directly as that is equivalent for this device.
         */
        guint8 id_low = self->reg_buf[0];
        self->afe_chip_id |= id_low;

        switch (self->afe_chip_id)
          {
          case FT9201_AFE_ID_FT9338W: self->chip_variant = 1; break;
          case FT9201_AFE_ID_FT9536W: self->chip_variant = 6; break;
          case FT9201_AFE_ID_FT9361:  self->chip_variant = 3; break;
          default:
            fp_warn ("init: unknown AFE chip ID 0x%04x", self->afe_chip_id);
            break;
          }

        fp_dbg ("init: AFE chip ID=0x%04x variant=%d",
                self->afe_chip_id, self->chip_variant);

        xfer = reg_read_transfer (device, FT9201_REG_SENSOR_WIDTH);
        xfer->ssm = ssm;
        fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                                 NULL, reg_read_to_buf_cb, NULL);
      }
      break;

    case INIT_READ_SENSOR_HEIGHT:
      self->sensor_width = self->reg_buf[0];
      fp_dbg ("init: sensor_width=%u", self->sensor_width);

      xfer = reg_read_transfer (device, FT9201_REG_SENSOR_HEIGHT);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, reg_read_to_buf_cb, NULL);
      break;

    /* ── Phase 4 ──────────────────────────────────────────────────────────── */

    case INIT_READ_FW_VERSION:
      self->sensor_height = self->reg_buf[0];
      fp_dbg ("init: sensor_height=%u", self->sensor_height);

      xfer = reg_read_transfer (device, FT9201_REG_FW_VERSION);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, reg_read_to_buf_cb, NULL);
      break;

    case INIT_READ_AGC_VERSION:
      self->fw_version = self->reg_buf[0];
      fp_dbg ("init: fw_version=%u", self->fw_version);

      xfer = reg_read_transfer (device, FT9201_REG_AGC_VERSION);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, reg_read_to_buf_cb, NULL);
      break;

    /* ── Phase 5 ──────────────────────────────────────────────────────────── */

    case INIT_READ_SENSOR_WIDTH2:
      /*
       * The kernel module reads dimensions a second time after the version
       * reads. We mirror this exactly so the USB transaction sequence matches
       * what the Windows driver produces (important for umockdev replays).
       */
      self->agc_version = self->reg_buf[0];
      fp_dbg ("init: agc_version=%u — second dimension read", self->agc_version);

      xfer = reg_read_transfer (device, FT9201_REG_SENSOR_WIDTH);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, reg_read_to_buf_cb, NULL);
      break;

    case INIT_READ_SENSOR_HEIGHT2:
      self->sensor_width = self->reg_buf[0];  /* refresh */

      xfer = reg_read_transfer (device, FT9201_REG_SENSOR_HEIGHT);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, reg_read_to_buf_cb, NULL);
      break;

    /* ── Phase 6 ──────────────────────────────────────────────────────────── */

    case INIT_WRITE_CFG_A:
      self->sensor_height = self->reg_buf[0];  /* refresh */
      fp_dbg ("init: sensor %ux%u", self->sensor_width, self->sensor_height);

      if (self->chip_variant == 2 || self->chip_variant == 3)
        {
          fp_dbg ("init: writing chip config (variant %d)", self->chip_variant);
          xfer = reg_write_transfer (device, FT9201_REG_CHIP_CFG_A, 0x00);
          xfer->ssm = ssm;
          fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                                   NULL, ctrl_out_cb, NULL);
        }
      else
        {
          /* Skip Phase 6 entirely — jump directly into Phase 7 */
          fpi_ssm_jump_to_state (ssm, INIT_POWER_POLL1);
        }
      break;

    case INIT_WRITE_CFG_B:
      xfer = reg_write_transfer (device, FT9201_REG_CHIP_CFG_B, 0x0e);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, ctrl_out_cb, NULL);
      break;

    /* ── Phase 7 ──────────────────────────────────────────────────────────── */

    case INIT_POWER_POLL1:
      fp_dbg ("init: phase 7 — MCU poll (attempt %d/%d)",
              self->poll_retry_count + 1, FT9201_MCU_POLL_MAX_RETRIES);

      xfer = reg_read_transfer (device, FT9201_REG_MCU_STATUS);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, power_poll1_cb, NULL);
      break;

    case INIT_POWER_WRITE_1F:
      fp_dbg ("init: writing REG_PWR_CTRL_1F = 0x01");
      xfer = reg_write_transfer (device, FT9201_REG_PWR_CTRL_1F, 0x01);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, ctrl_out_cb, NULL);
      break;

    case INIT_POWER_WRITE_1E:
      fp_dbg ("init: writing REG_PWR_CTRL_1E = 0x01");
      xfer = reg_write_transfer (device, FT9201_REG_PWR_CTRL_1E, 0x01);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, ctrl_out_cb, NULL);
      break;

    case INIT_POWER_SETTLE_DELAY:
      fp_dbg ("init: 10ms power settle");
      fpi_ssm_next_state_delayed (ssm, FT9201_POWER_SETTLE_DELAY_MS);
      break;

    /* ── Phase 8 ──────────────────────────────────────────────────────────── */

    case INIT_POWER_POLL2:
      fp_dbg ("init: phase 8 — post-power MCU poll (attempt %d/%d)",
              self->poll_retry_count + 1, FT9201_MCU_POLL_MAX_RETRIES);

      xfer = reg_read_transfer (device, FT9201_REG_MCU_STATUS);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, power_poll2_cb, NULL);
      break;

    case INIT_POWER_READ_1D:
      /*
       * The kernel reads REG_FINGER_PRESENT here but does not check the value —
       * it is required by the device's internal state machine to advance past
       * the power-up sequence. We must issue this read.
       */
      fp_dbg ("init: required REG_FINGER_PRESENT read (0x1d)");
      xfer = reg_read_transfer (device, FT9201_REG_FINGER_PRESENT);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, reg_read_to_buf_cb, NULL);
      break;

    case INIT_POWER_READ_MCU_FINAL:
      fp_dbg ("init: final MCU status confirmation");
      xfer = reg_read_transfer (device, FT9201_REG_MCU_STATUS);
      xfer->ssm = ssm;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               NULL, reg_read_to_buf_cb, NULL);
      break;

    default:
      g_assert_not_reached ();
    }
}

/**
 * init_ssm_cb:
 * @ssm:    the completed #FpiSsm
 * @device: our #FpDevice
 * @error:  set if the SSM failed, %NULL on success
 *
 * Completion callback for init_ssm. Signals libfprint that the device
 * has been successfully opened and is ready to accept capture requests,
 * or forwards the failure error if initialisation did not complete.
 */
static void
init_ssm_cb (FpiSsm   *ssm,
             FpDevice *device,
             GError   *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  self->task_ssm = NULL;

  if (error)
    {
      fp_err ("FT9201 init failed: %s", error->message);
      fpi_image_device_open_complete (FPI_IMAGE_DEVICE (device), error);
      return;
    }

  fp_dbg ("FT9201 init complete — chip_id=0x%04x variant=%d size=%ux%u "
          "fw=%u agc=%u",
          self->afe_chip_id, self->chip_variant,
          self->sensor_width, self->sensor_height,
          self->fw_version, self->agc_version);

  fpi_image_device_open_complete (FPI_IMAGE_DEVICE (device), NULL);
}


/* ============================================================================
 * STATE MACHINE 2: CAPTURE (capture_ssm)
 *
 * Translates the ft9201_read() polling loop into an async FpiSsm.
 * Polling transfers are submitted with fpi_device_get_cancellable() so
 * that ft9201_deactivate() terminates them cleanly via G_IO_ERROR_CANCELLED.
 * ============================================================================ */

/**
 * CaptureState:
 *
 * States for the capture state machine.
 */
typedef enum
{
  CAPTURE_POLL_MCU,             /**< READ REG_MCU_STATUS; retry on not-ready  */
  CAPTURE_CHECK_ARMED,          /**< READ REG_CAPTURE_READY; check for 0xbb   */
  CAPTURE_CHECK_FINGER,         /**< READ REG_FINGER_PRESENT                  */
  CAPTURE_START_CMD_1,          /**< OUT 0x34 wValue=0xff (decide + fire)      */
  CAPTURE_START_DELAY,          /**< 20ms gap between start commands           */
  CAPTURE_START_CMD_2,          /**< OUT 0x34 wValue=0x03                     */
  CAPTURE_CONFIGURE_BULK,       /**< OUT 0x35 — set bulk byte count            */
  CAPTURE_READ_IMAGE,           /**< Bulk IN: sensor_width*sensor_height+2     */
  CAPTURE_NUM_STATES
} CaptureState;


/* ── Capture retry helpers ────────────────────────────────────────────────── */

/**
 * retry_capture_mcu_poll:
 * GSourceFunc — fires after FT9201_CAPTURE_MCU_POLL_DELAY_MS to retry
 * the MCU status check. Uses a slow (100ms) interval; during capture the
 * driver can afford to wait politely for the MCU to be ready.
 *
 * Takes @device (not the FpiSsm directly) so it can clear
 * self->retry_source_id before jumping — this is what lets
 * ft9201_deactivate() tell "a retry is pending" apart from "this source
 * already fired," which matters because g_source_remove() on an id that
 * already fired is a programming error in GLib.
 */
static gboolean
retry_capture_mcu_poll (gpointer user_data)
{
  FpDevice        *device = FP_DEVICE (user_data);
  FpiDeviceFt9201 *self   = FPI_DEVICE_FT9201 (device);

  self->retry_source_id = 0;

  /* task_ssm can be NULL if deactivate() raced this callback and already
   * tore the SSM down via a different path; nothing to do in that case. */
  if (self->task_ssm != NULL)
    fpi_ssm_jump_to_state (self->task_ssm, CAPTURE_POLL_MCU);

  return G_SOURCE_REMOVE;
}

/**
 * retry_capture_armed_poll:
 * GSourceFunc — fires after FT9201_CAPTURE_ARMED_RETRY_MS to retry
 * the REG_CAPTURE_READY check when the sensor is not yet armed.
 * See retry_capture_mcu_poll for why @device is passed instead of @ssm.
 */
static gboolean
retry_capture_armed_poll (gpointer user_data)
{
  FpDevice        *device = FP_DEVICE (user_data);
  FpiDeviceFt9201 *self   = FPI_DEVICE_FT9201 (device);

  self->retry_source_id = 0;

  if (self->task_ssm != NULL)
    fpi_ssm_jump_to_state (self->task_ssm, CAPTURE_POLL_MCU);

  return G_SOURCE_REMOVE;
}

/**
 * fire_capture_start_delay:
 * GSourceFunc — fires after the 20ms gap between the two start-capture
 * commands and advances the SSM to CAPTURE_START_CMD_2.
 *
 * Deliberately implemented as a tracked g_timeout_add() rather than
 * fpi_ssm_next_state_delayed(): the latter's internal timer is not
 * something ft9201_deactivate() can reach or cancel, whereas this one
 * is tracked in self->retry_source_id exactly like the poll retries
 * above, so a single code path in deactivate() covers every wait state
 * in capture_ssm.
 */
static gboolean
fire_capture_start_delay (gpointer user_data)
{
  FpDevice        *device = FP_DEVICE (user_data);
  FpiDeviceFt9201 *self   = FPI_DEVICE_FT9201 (device);

  self->retry_source_id = 0;

  if (self->task_ssm != NULL)
    fpi_ssm_next_state (self->task_ssm);

  return G_SOURCE_REMOVE;
}


/* ── Capture transfer callbacks ───────────────────────────────────────────── */

/**
 * capture_poll_mcu_cb:
 * Callback for CAPTURE_POLL_MCU.
 * MCU ready → advance to CAPTURE_CHECK_ARMED.
 * Not ready → schedule retry after 100ms.
 * Cancelled → propagate (deactivate path).
 */
static void
capture_poll_mcu_cb (FpiUsbTransfer *transfer,
                     FpDevice       *device,
                     gpointer        user_data,
                     GError         *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  self->active_transfer = NULL;

  if (error)
    {
      if (!g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
        fp_warn ("capture: MCU poll error: %s — retrying", error->message);
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  memcpy (self->reg_buf, transfer->buffer, 4);

  if (mcu_is_ready (self->reg_buf))
    {
      fpi_ssm_next_state (transfer->ssm);   /* → CAPTURE_CHECK_ARMED */
    }
  else
    {
      self->retry_source_id =
        g_timeout_add (FT9201_CAPTURE_MCU_POLL_DELAY_MS,
                       retry_capture_mcu_poll, device);
    }
}

/**
 * capture_check_armed_cb:
 * Callback for CAPTURE_CHECK_ARMED (READ REG_CAPTURE_READY).
 * 0xbb → armed, advance to CAPTURE_CHECK_FINGER.
 * Other → sensor not yet armed, restart from MCU poll after 50ms.
 */
static void
capture_check_armed_cb (FpiUsbTransfer *transfer,
                        FpDevice       *device,
                        gpointer        user_data,
                        GError         *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  self->active_transfer = NULL;

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  memcpy (self->reg_buf, transfer->buffer, 4);

  if (self->reg_buf[0] == FT9201_CAPTURE_ARMED_MAGIC)
    {
      fpi_ssm_next_state (transfer->ssm);   /* → CAPTURE_CHECK_FINGER */
    }
  else
    {
      fp_dbg ("capture: REG_CAPTURE_READY=0x%02x (not armed) — retrying",
              self->reg_buf[0]);
      self->retry_source_id =
        g_timeout_add (FT9201_CAPTURE_ARMED_RETRY_MS,
                       retry_capture_armed_poll, device);
    }
}

/**
 * capture_bulk_cb:
 * Callback for CAPTURE_READ_IMAGE (bulk IN).
 *
 * Validates the transfer length, skips the 2-byte header, copies the
 * raw pixel data into a newly allocated #FpImage, and delivers it to
 * the libfprint daemon via fpi_image_device_image_captured().
 */
static void
capture_bulk_cb (FpiUsbTransfer *transfer,
                 FpDevice       *device,
                 gpointer        user_data,
                 GError         *error)
{
  FpiDeviceFt9201 *self   = FPI_DEVICE_FT9201 (device);
  guint            img_sz = (guint) self->sensor_width * self->sensor_height;
  FpImage         *image;

  self->active_transfer = NULL;

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  if (transfer->actual_length != (gssize) (img_sz + FT9201_BULK_HEADER_SIZE))
    {
      fpi_ssm_mark_failed (
        transfer->ssm,
        fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_INVALID,
                                  "bulk read: got %zd bytes, expected %u",
                                  transfer->actual_length,
                                  img_sz + FT9201_BULK_HEADER_SIZE));
      return;
    }

  image = fp_image_new (self->sensor_width, self->sensor_height);

  /*
   * Skip the 2-byte opaque header.  The kernel module does the same:
   *   memcpy(dev->read_img_data, img_with_header + 2, img_size);
   */
  memcpy (image->data,
          transfer->buffer + FT9201_BULK_HEADER_SIZE,
          img_sz);

  fpi_image_device_report_finger_status (FPI_IMAGE_DEVICE (device), FALSE);
  fpi_image_device_image_captured (FPI_IMAGE_DEVICE (device), image);

  /* SSM complete — libfprint will restart capture if more images are needed */
  fpi_ssm_mark_completed (transfer->ssm);
}


/**
 * capture_ssm_state:
 * @ssm:    the active #FpiSsm
 * @device: our #FpDevice
 *
 * State handler for capture_ssm.
 */
static void
capture_ssm_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceFt9201 *self   = FPI_DEVICE_FT9201 (device);
  FpiUsbTransfer  *xfer;
  guint            img_sz;
  GCancellable    *cancel = fpi_device_get_cancellable (device);

  switch ((CaptureState) fpi_ssm_get_cur_state (ssm))
    {

    case CAPTURE_POLL_MCU:
      /*
       * Poll REG_MCU_STATUS until the MCU signals ready (0xa5, 0x5a).
       * Equivalent to the kernel's data_ready check on REG_MCU_STATUS
       * inside the ft9201_read() loop.
       * Use the device cancellable so deactivate() terminates this loop.
       */
      xfer = reg_read_transfer (device, FT9201_REG_MCU_STATUS);
      xfer->ssm = ssm;
      self->active_transfer = xfer;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               cancel, capture_poll_mcu_cb, NULL);
      break;

    case CAPTURE_CHECK_ARMED:
      /*
       * READ REG_CAPTURE_READY (0x30).
       * Equivalent to ft9201_get_afe_state(dev, 0x30, &translate_data)
       * followed by the check translate_data == 0xbb.
       */
      xfer = reg_read_transfer (device, FT9201_REG_CAPTURE_READY);
      xfer->ssm = ssm;
      self->active_transfer = xfer;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               cancel, capture_check_armed_cb, NULL);
      break;

    case CAPTURE_CHECK_FINGER:
      /*
       * READ REG_FINGER_PRESENT (0x1d).
       * The result is inspected in CAPTURE_START_CMD_1 (next state).
       */
      xfer = reg_read_transfer (device, FT9201_REG_FINGER_PRESENT);
      xfer->ssm = ssm;
      self->active_transfer = xfer;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               cancel, reg_read_to_buf_cb, NULL);
      break;

    case CAPTURE_START_CMD_1:
      /*
       * reg_buf[0] holds the REG_FINGER_PRESENT value from the previous state.
       * If no finger is detected (not 0x01 or 0xa0) restart the poll loop.
       * Otherwise report finger-on and fire the first start-capture command.
       */
      if (!finger_is_present (self->reg_buf[0]))
        {
          fp_dbg ("capture: finger not present (0x%02x) — restarting poll",
                  self->reg_buf[0]);
          fpi_ssm_jump_to_state (ssm, CAPTURE_POLL_MCU);
          break;
        }

      fp_dbg ("capture: finger detected (0x%02x) — starting capture",
              self->reg_buf[0]);
      fpi_image_device_report_finger_status (FPI_IMAGE_DEVICE (device), TRUE);

      /*
       * OUT 0x34 wValue=0xff — equivalent to ft9201_probably_start_capture(0xff).
       * Now cancellable: once committed to the capture sequence, a
       * deactivate() request should still be able to interrupt it cleanly
       * rather than running to completion regardless.
       */
      xfer = ctrl_out_transfer (device, FT9201_REQ_START_CAPTURE, 0x00ff, 0x0000);
      xfer->ssm = ssm;
      self->active_transfer = xfer;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               cancel, ctrl_out_cb, NULL);
      break;

    case CAPTURE_START_DELAY:
      /*
       * The kernel sleeps 20ms between the two start-capture commands.
       * Implemented as a tracked g_timeout_add() rather than
       * fpi_ssm_next_state_delayed() — see fire_capture_start_delay()
       * for why: deactivate() needs a GSource id it can remove, and
       * next_state_delayed() doesn't expose one.
       */
      self->retry_source_id =
        g_timeout_add (FT9201_CAPTURE_CMD_GAP_MS, fire_capture_start_delay, device);
      break;

    case CAPTURE_START_CMD_2:
      /* OUT 0x34 wValue=0x03 — equivalent to ft9201_probably_start_capture(0x03) */
      xfer = ctrl_out_transfer (device, FT9201_REQ_START_CAPTURE, 0x0003, 0x0000);
      xfer->ssm = ssm;
      self->active_transfer = xfer;
      fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                               cancel, ctrl_out_cb, NULL);
      break;

    case CAPTURE_CONFIGURE_BULK:
      {
        img_sz = (guint) self->sensor_width * self->sensor_height;

        /*
         * OUT 0x35 wValue=(img_size+2) wIndex=0x3400
         * Equivalent to ft9201_probably_configure_bulk_transfer(dev, 0x3400,
         *               sensor_width * sensor_height + 2)
         * This tells the device firmware how many bytes to stage in its
         * internal buffer before asserting the bulk-in endpoint.
         */
        xfer = ctrl_out_transfer (device,
                                  FT9201_REQ_CONFIGURE_BULK_XFER,
                                  (guint16) (img_sz + FT9201_BULK_HEADER_SIZE),
                                  0x3400);
        xfer->ssm = ssm;
        self->active_transfer = xfer;
        fpi_usb_transfer_submit (xfer, FT9201_CTRL_TIMEOUT_MS,
                                 cancel, ctrl_out_cb, NULL);
      }
      break;

    case CAPTURE_READ_IMAGE:
      {
        img_sz = (guint) self->sensor_width * self->sensor_height;

        fp_dbg ("capture: bulk IN — requesting %u bytes from endpoint 0x%02x",
                img_sz + FT9201_BULK_HEADER_SIZE, self->bulk_in_ep);

        /*
         * Equivalent to usb_bulk_msg(dev->udev,
         *   usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
         *   img_with_header, transfer_size, &read_length, timeout)
         *
         * self->bulk_in_ep is resolved once in ft9201_open() by
         * discover_bulk_in_endpoint() — see that function for why this
         * is no longer a bare compile-time constant.
         */
        xfer = fpi_usb_transfer_new (device);
        fpi_usb_transfer_fill_bulk (xfer,
                                    self->bulk_in_ep,
                                    img_sz + FT9201_BULK_HEADER_SIZE);
        xfer->ssm = ssm;
        self->active_transfer = xfer;
        fpi_usb_transfer_submit (xfer, FT9201_BULK_TIMEOUT_MS,
                                 cancel, capture_bulk_cb, NULL);
      }
      break;

    default:
      g_assert_not_reached ();
    }
}

/**
 * capture_ssm_cb:
 * @ssm:    the completed #FpiSsm
 * @device: our #FpDevice
 * @error:  set if the SSM failed, %NULL on normal completion
 *
 * Completion callback for capture_ssm. A G_IO_ERROR_CANCELLED indicates
 * a clean deactivation and is silently swallowed. All other errors are
 * forwarded to the libfprint session error handler.
 */
static void
capture_ssm_cb (FpiSsm   *ssm,
                FpDevice *device,
                GError   *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  self->task_ssm = NULL;

  if (error)
    {
      if (!g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
        fpi_image_device_session_error (FPI_IMAGE_DEVICE (device), error);
      else
        g_error_free (error);
    }
}


/* ============================================================================
 * Device Lifecycle Overrides
 * ============================================================================ */

/**
 * discover_bulk_in_endpoint:
 * @self:    our device, for storing the result in self->bulk_in_ep
 * @usb_dev: the claimed #GUsbDevice
 *
 * Finds interface 0's bulk-IN endpoint dynamically instead of trusting a
 * hardcoded constant, mirroring what the kernel driver does with
 * usb_find_common_endpoints().
 *
 * Caveat worth knowing before trusting this blindly: libgusb's public
 * #GUsbEndpoint API (gusb-endpoint.h, confirmed against the upstream
 * source) exposes address, direction, number, max-packet-size, and
 * polling interval — but does NOT expose bmAttributes / transfer type.
 * So "IN endpoint" can be determined exactly via
 * g_usb_endpoint_get_direction(), but "bulk vs. interrupt" cannot be
 * read directly off the wrapper.
 *
 * In practice this device only exposes one IN endpoint that matters to
 * this driver (the kernel module never touches an interrupt endpoint
 * either — see the top-of-file protocol notes), so the common case is
 * trivial: if interface 0 has exactly one IN endpoint, that's it. If
 * there happen to be more than one, we fall back to picking the one
 * with the largest max packet size (bulk endpoints are conventionally
 * larger than interrupt endpoints on full/high-speed USB) and log a
 * warning so this is easy to catch and override during testing.
 *
 * Returns %TRUE and sets self->bulk_in_ep on success; %FALSE if
 * interface 0 or its endpoint list could not be read at all, leaving
 * the caller to fall back to FT9201_EP_BULK_IN_FALLBACK.
 */
static gboolean
discover_bulk_in_endpoint (FpiDeviceFt9201 *self, GUsbDevice *usb_dev)
{
  g_autoptr(GError)      error      = NULL;
  g_autoptr(GPtrArray)   interfaces = NULL;
  g_autoptr(GPtrArray)   endpoints  = NULL;
  GUsbInterface         *iface0     = NULL;
  guint8                 best_addr  = 0;
  guint16                best_size  = 0;
  guint                  in_count   = 0;
  guint                  i;

  /* g_usb_device_get_interfaces() is (transfer container): we own the
   * GPtrArray itself (must unref it) but not the GUsbInterface elements
   * inside it — same for g_usb_interface_get_endpoints() below. */
  interfaces = g_usb_device_get_interfaces (usb_dev, &error);
  if (interfaces == NULL)
    {
      fp_warn ("ft9201: could not enumerate USB interfaces: %s — "
               "falling back to endpoint 0x%02x",
               error->message, FT9201_EP_BULK_IN_FALLBACK);
      return FALSE;
    }

  for (i = 0; i < interfaces->len; i++)
    {
      GUsbInterface *candidate = g_ptr_array_index (interfaces, i);

      if (g_usb_interface_get_number (candidate) == 0)
        {
          iface0 = candidate;
          break;
        }
    }

  if (iface0 == NULL)
    {
      fp_warn ("ft9201: interface 0 not found in descriptor — "
               "falling back to endpoint 0x%02x",
               FT9201_EP_BULK_IN_FALLBACK);
      return FALSE;
    }

  endpoints = g_usb_interface_get_endpoints (iface0);
  if (endpoints == NULL || endpoints->len == 0)
    {
      fp_warn ("ft9201: interface 0 has no endpoints — "
               "falling back to endpoint 0x%02x",
               FT9201_EP_BULK_IN_FALLBACK);
      return FALSE;
    }

  for (i = 0; i < endpoints->len; i++)
    {
      GUsbEndpoint *ep      = g_ptr_array_index (endpoints, i);
      guint8        address = g_usb_endpoint_get_address (ep);
      guint16       size    = g_usb_endpoint_get_maximum_packet_size (ep);

      if (g_usb_endpoint_get_direction (ep) != G_USB_DEVICE_DIRECTION_DEVICE_TO_HOST)
        continue;   /* not an IN endpoint */

      in_count++;
      fp_dbg ("ft9201: candidate IN endpoint 0x%02x, max packet size %u",
              address, size);

      if (size >= best_size)
        {
          best_addr = address;
          best_size = size;
        }
    }

  if (in_count == 0)
    {
      fp_warn ("ft9201: no IN endpoints on interface 0 — "
               "falling back to endpoint 0x%02x",
               FT9201_EP_BULK_IN_FALLBACK);
      return FALSE;
    }

  if (in_count > 1)
    {
      fp_warn ("ft9201: %u IN endpoints found on interface 0 — picked 0x%02x "
               "(largest max-packet-size); verify with `lsusb -v -d 2808:93a9` "
               "if captures fail", in_count, best_addr);
    }
  else
    {
      fp_dbg ("ft9201: discovered bulk-IN endpoint 0x%02x", best_addr);
    }

  self->bulk_in_ep = best_addr;
  return TRUE;
}

/**
 * ft9201_open:
 * @dev: our #FpImageDevice
 *
 * Claims USB interface 0, resolves the bulk-IN endpoint, and launches
 * init_ssm. The SSM calls fpi_image_device_open_complete() when the
 * hardware is ready (or on error).
 */
static void
ft9201_open (FpImageDevice *dev)
{
  FpiDeviceFt9201 *self    = FPI_DEVICE_FT9201 (dev);
  GUsbDevice      *usb_dev = fpi_device_get_usb_device (FP_DEVICE (dev));
  GError          *error   = NULL;

  if (!g_usb_device_claim_interface (usb_dev, 0, 0, &error))
    {
      fpi_image_device_open_complete (dev, error);
      return;
    }

  /* Initialise runtime state */
  self->poll_retry_count = 0;
  self->sensor_width     = FT9201_IMAGE_WIDTH;   /* sensible fallback */
  self->sensor_height    = FT9201_IMAGE_HEIGHT;
  self->active_transfer  = NULL;
  self->retry_source_id  = 0;

  if (!discover_bulk_in_endpoint (self, usb_dev))
    self->bulk_in_ep = FT9201_EP_BULK_IN_FALLBACK;

  fp_dbg ("ft9201: open — launching init_ssm");

  self->task_ssm = fpi_ssm_new (FP_DEVICE (dev), init_ssm_state, INIT_NUM_STATES);
  fpi_ssm_start (self->task_ssm, init_ssm_cb);
}

/**
 * ft9201_close:
 * @dev: our #FpImageDevice
 *
 * Releases USB interface 0. Any errors are forwarded to libfprint.
 */
static void
ft9201_close (FpImageDevice *dev)
{
  GError *error = NULL;

  fp_dbg ("ft9201: close");

  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (dev)),
                                  0, 0, &error);
  fpi_image_device_close_complete (dev, error);
}

/**
 * ft9201_activate:
 * @dev: our #FpImageDevice
 *
 * Starts capture_ssm to begin the finger-present polling loop.
 * Calls fpi_image_device_activate_complete() immediately; the SSM
 * runs in the background until a finger is detected.
 */
static void
ft9201_activate (FpImageDevice *dev)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (dev);

  fp_dbg ("ft9201: activate — launching capture_ssm");

  self->poll_retry_count = 0;
  self->active_transfer  = NULL;
  self->retry_source_id  = 0;

  self->task_ssm = fpi_ssm_new (FP_DEVICE (dev), capture_ssm_state, CAPTURE_NUM_STATES);
  fpi_ssm_start (self->task_ssm, capture_ssm_cb);

  fpi_image_device_activate_complete (dev, NULL);
}

/**
 * ft9201_deactivate:
 * @dev: our #FpImageDevice
 *
 * Actually stops capture_ssm, which the previous version of this function
 * did not do — it called deactivate_complete() without touching
 * self->task_ssm at all. The device's GCancellable only reaches submitted
 * USB transfers; it does not reach a pending g_timeout_add() retry (used
 * by the MCU/armed poll loops and the inter-command delay), so a retry
 * scheduled right before deactivation would previously survive it, fire
 * later, and could end up calling fpi_image_device_image_captured() on a
 * device libfprint already considers deactivated.
 *
 * Two cases, mutually exclusive by construction (every capture_ssm state
 * leaves exactly one of these set while task_ssm is non-NULL):
 *
 *   1. A retry/delay timer is pending (self->retry_source_id != 0) — for
 *      example mid-wait in CAPTURE_START_DELAY, or between MCU polls.
 *      Remove it directly; nothing else is in flight to generate a
 *      completion, so the SSM is failed explicitly with CANCELLED.
 *
 *   2. A USB transfer is in flight (self->active_transfer != NULL) —
 *      cancel it. Its callback receives G_IO_ERROR_CANCELLED and calls
 *      fpi_ssm_mark_failed() itself, which drives capture_ssm_cb() and
 *      tears down task_ssm from there.
 *
 * The final `else if (task_ssm != NULL)` branch is a defensive fallback
 * for a state that doesn't set either field — not reachable today, but
 * cheap insurance against a future state being added without the
 * invariant holding.
 */
static void
ft9201_deactivate (FpImageDevice *dev)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (dev);

  fp_dbg ("ft9201: deactivate");

  if (self->retry_source_id != 0)
    {
      g_source_remove (self->retry_source_id);
      self->retry_source_id = 0;

      if (self->task_ssm != NULL)
        fpi_ssm_mark_failed (self->task_ssm,
                             g_error_new (G_IO_ERROR, G_IO_ERROR_CANCELLED,
                                          "FT9201 deactivated"));
    }
  else if (self->active_transfer != NULL)
    {
      fpi_usb_transfer_cancel (self->active_transfer);
    }
  else if (self->task_ssm != NULL)
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           g_error_new (G_IO_ERROR, G_IO_ERROR_CANCELLED,
                                        "FT9201 deactivated"));
    }

  fpi_image_device_deactivate_complete (dev, NULL);
}


/* ============================================================================
 * Class and Type Registration
 * ============================================================================ */

/** USB device ID table */
static const FpIdEntry id_table[] = {
  { .vid = 0x2808, .pid = 0x93a9 },   /* FocalTech FT9201 (FT9361 sensor) */
  { .vid = 0,      .pid = 0         },
};

static void
fpi_device_ft9201_init (FpiDeviceFt9201 *self)
{
  /* GObject zero-initialises all fields; nothing extra needed here */
}

/**
 * fpi_device_ft9201_class_init:
 *
 * Registers device metadata and binds lifecycle method overrides.
 * img_width / img_height are set to the FT9361 defaults; the driver
 * also reads the actual dimensions at runtime from REG_SENSOR_WIDTH/HEIGHT
 * and stores them in self->sensor_width / self->sensor_height so the
 * bulk transfer and FpImage allocation use the real values.
 */
static void
fpi_device_ft9201_class_init (FpiDeviceFt9201Class *klass)
{
  FpDeviceClass      *dev_class = FP_DEVICE_CLASS (klass);
  FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS (klass);

  /* Device identity */
  dev_class->id        = "ft9201";
  dev_class->full_name = "FocalTech FT9201 Fingerprint Reader";
  dev_class->type      = FP_DEVICE_TYPE_USB;
  dev_class->id_table  = id_table;
  dev_class->features  = FP_DEVICE_FEATURE_CAPTURE   |
                         FP_DEVICE_FEATURE_IDENTIFY   |
                         FP_DEVICE_FEATURE_VERIFY     |
                         FP_DEVICE_FEATURE_ENROLL;

  /* Sensor geometry (FT9361 defaults) */
  img_class->img_width   = FT9201_IMAGE_WIDTH;
  img_class->img_height  = FT9201_IMAGE_HEIGHT;

  /*
   * bz3_threshold: Bozorth3 match score below which a verify/identify
   * result is rejected. 24 is a common starting point for area sensors;
   * tune empirically with real fingerprint captures and umockdev replays.
   */
  img_class->bz3_threshold = 24;

  /*
   * scan_type: PRESS because the FT9361 is a fixed-area sensor that captures
   * a complete image in a single bulk transfer. The user places their finger
   * on the pad (not swipes it).
   */
  img_class->scan_type = FPI_SCAN_TYPE_PRESS;

  /* Lifecycle overrides */
  img_class->open       = ft9201_open;
  img_class->close      = ft9201_close;
  img_class->activate   = ft9201_activate;
  img_class->deactivate = ft9201_deactivate;
}
