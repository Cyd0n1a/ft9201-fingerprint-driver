/*
 * FocalTech FT9201 Fingerprint Reader libfprint Driver
 * USB ID: 2808:93a9
 */

#define FP_COMPONENT "ft9201"

#include <fprint.h>
#include "fpi-log.h"
#include "fpi-image-device.h"
#include "fpi-usb-transfer.h"

/* * 1. Object Definition 
 * Inherit from FpImageDevice since the FT9201 is a Match-on-Host scanner 
 * that returns a 64x80 image rather than a pass/fail boolean.
 */
struct _FpiDeviceFt9201
{
  FpImageDevice parent;

  /* State machines for asynchronous operations */
  FpiSsm *init_ssm;
  FpiSsm *capture_ssm;

  /* Polling transfer for interrupts */
  FpiUsbTransfer *poll_transfer;
};

G_DECLARE_FINAL_TYPE (FpiDeviceFt9201, fpi_device_ft9201, FPI, DEVICE_FT9201, FpImageDevice);
G_DEFINE_TYPE (FpiDeviceFt9201, fpi_device_ft9201, FP_TYPE_IMAGE_DEVICE);

/* Define your USB endpoints based on lsusb -v output */
#define FT9201_EP_IN_INT  0x81 /* Change to actual Interrupt IN endpoint */
#define FT9201_EP_IN_BULK 0x82 /* Change to actual Bulk IN endpoint */
#define FT9201_EP_OUT     0x02 /* Change to actual Bulk OUT endpoint */

#define FT9201_IMAGE_WIDTH  64
#define FT9201_IMAGE_HEIGHT 80
#define FT9201_IMAGE_SIZE   (FT9201_IMAGE_WIDTH * FT9201_IMAGE_HEIGHT)


/* * ============================================================================
 * STATE MACHINE: INITIALIZATION
 * ============================================================================
 */
enum init_states {
  INIT_SEND_WAKEUP,
  INIT_READ_VERSION,
  INIT_SEND_CONFIG,
  INIT_NUM_STATES
};

static void
init_ssm_cb (FpiUsbTransfer *transfer, FpDevice *device,
             gpointer user_data, GError *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  if (error) {
    fpi_ssm_mark_failed (self->init_ssm, error);
    return;
  }

  /* Advance the state machine when the async USB transfer completes */
  fpi_ssm_next_state (self->init_ssm);
}

static void
init_ssm_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);
  FpiUsbTransfer *transfer;

  switch (fpi_ssm_get_cur_state (ssm)) {
  case INIT_SEND_WAKEUP:
    fp_dbg ("Sending wakeup payload...");
    transfer = fpi_usb_transfer_new (device);
    
    /* TODO: AMANDA - Replace this with the control transfer parameters 
       found in the kernel module's initialization sequence. */
    fpi_usb_transfer_fill_control (transfer,
                                   G_USB_DEVICE_DIRECTION_HOST_TO_DEVICE,
                                   G_USB_DEVICE_REQUEST_TYPE_VENDOR,
                                   G_USB_DEVICE_RECIPIENT_DEVICE,
                                   0x00, /* Request */
                                   0x00, /* Value */
                                   0x00, /* Index */
                                   4);   /* Length */
    
    /* transfer->buffer[0] = 0xXX; ... */
    
    fpi_usb_transfer_submit (transfer, 1000, NULL, init_ssm_cb, NULL);
    break;

  case INIT_READ_VERSION:
    fp_dbg ("Reading firmware version...");
    /* TODO: Implement the read request */
    fpi_ssm_next_state (ssm);
    break;

  case INIT_SEND_CONFIG:
    fp_dbg ("Sending sensor gain/contrast configuration...");
    /* TODO: Implement final config payload */
    fpi_ssm_next_state (ssm);
    break;
  }
}

static void
init_ssm_done (FpiSsm *ssm, FpDevice *device, GError *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  self->init_ssm = NULL;

  if (error) {
    fpi_image_device_open_complete (FPI_IMAGE_DEVICE (device), error);
    return;
  }

  fp_dbg ("FT9201 initialization complete.");
  fpi_image_device_open_complete (FPI_IMAGE_DEVICE (device), NULL);
}

/* * ============================================================================
 * STATE MACHINE: CAPTURE (POLL -> READ IMAGE)
 * ============================================================================
 */
enum capture_states {
  CAPTURE_POLL_INTERRUPT,
  CAPTURE_READ_BULK_IMAGE,
  CAPTURE_NUM_STATES
};

static void
capture_bulk_cb (FpiUsbTransfer *transfer, FpDevice *device,
                 gpointer user_data, GError *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);
  FpImage *image;

  if (error) {
    fpi_ssm_mark_failed (self->capture_ssm, error);
    return;
  }

  /* Assemble the raw image buffer into an FpImage object */
  image = fp_image_new (FT9201_IMAGE_WIDTH, FT9201_IMAGE_HEIGHT);
  image->flags = FPI_IMAGE_COLORS_INVERTED; /* Adjust if ridges are white instead of black */
  
  /* Copy the raw payload from the bulk transfer into the image buffer */
  memcpy (image->data, transfer->buffer, FT9201_IMAGE_SIZE);

  /* Send the image up to the libfprint daemon for bozorth3 extraction */
  fpi_image_device_image_captured (FPI_IMAGE_DEVICE (device), image);

  fpi_image_device_report_finger_status (FPI_IMAGE_DEVICE (device), FALSE);
  fpi_ssm_next_state (self->capture_ssm);
}

static void
capture_interrupt_cb (FpiUsbTransfer *transfer, FpDevice *device,
                      gpointer user_data, GError *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  if (g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
    return; /* Deactivated by the system */

  if (error) {
    fpi_ssm_mark_failed (self->capture_ssm, error);
    return;
  }

  /* TODO: AMANDA - Inspect transfer->buffer to see if the interrupt packet 
     actually signifies a finger press. Often devices send keep-alives. 
     If it's just a keep-alive, resubmit the interrupt transfer here. */

  fp_dbg ("Finger detected on sensor. Proceeding to bulk read.");
  fpi_image_device_report_finger_status (FPI_IMAGE_DEVICE (device), TRUE);
  fpi_ssm_next_state (self->capture_ssm);
}

static void
capture_ssm_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);
  FpiUsbTransfer *transfer;

  switch (fpi_ssm_get_cur_state (ssm)) {
  case CAPTURE_POLL_INTERRUPT:
    fp_dbg ("Waiting for finger interrupt...");
    self->poll_transfer = fpi_usb_transfer_new (device);
    fpi_usb_transfer_fill_interrupt (self->poll_transfer,
                                     FT9201_EP_IN_INT,
                                     16); /* Expected interrupt packet size */
    
    fpi_usb_transfer_submit (self->poll_transfer, 0 /* Infinite timeout */, 
                             NULL, capture_interrupt_cb, NULL);
    break;

  case CAPTURE_READ_BULK_IMAGE:
    fp_dbg ("Reading image frame via bulk endpoint...");
    transfer = fpi_usb_transfer_new (device);
    fpi_usb_transfer_fill_bulk (transfer,
                                FT9201_EP_IN_BULK,
                                FT9201_IMAGE_SIZE);
    
    fpi_usb_transfer_submit (transfer, 2000, NULL, capture_bulk_cb, NULL);
    break;
  }
}

static void
capture_ssm_done (FpiSsm *ssm, FpDevice *device, GError *error)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (device);

  self->capture_ssm = NULL;

  if (error) {
    if (!g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
      fpi_image_device_session_error (FPI_IMAGE_DEVICE (device), error);
  } else {
    /* If capture succeeds but we need more enrollments, 
       we typically restart the capture state machine here. */
  }
}

/* * ============================================================================
 * DEVICE LIFECYCLE OVERRIDES
 * ============================================================================
 */
static void
ft9201_open (FpImageDevice *dev)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (dev);
  GError *error = NULL;

  if (!g_usb_device_claim_interface (fpi_device_get_usb_device (FP_DEVICE (dev)), 0, 0, &error)) {
    fpi_image_device_open_complete (dev, error);
    return;
  }

  fp_dbg ("Starting initialization state machine.");
  self->init_ssm = fpi_ssm_new (FP_DEVICE (dev), init_ssm_state, INIT_NUM_STATES);
  fpi_ssm_start (self->init_ssm, init_ssm_done);
}

static void
ft9201_close (FpImageDevice *dev)
{
  GError *error = NULL;

  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (dev)), 0, 0, &error);
  fpi_image_device_close_complete (dev, error);
}

static void
ft9201_activate (FpImageDevice *dev)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (dev);

  fp_dbg ("Device activated. Waiting for finger...");
  self->capture_ssm = fpi_ssm_new (FP_DEVICE (dev), capture_ssm_state, CAPTURE_NUM_STATES);
  fpi_ssm_start (self->capture_ssm, capture_ssm_done);
}

static void
ft9201_deactivate (FpImageDevice *dev)
{
  FpiDeviceFt9201 *self = FPI_DEVICE_FT9201 (dev);

  fp_dbg ("Deactivating device. Cancelling active transfers.");
  if (self->capture_ssm)
    fpi_ssm_cancel (self->capture_ssm);
  
  if (self->poll_transfer) {
    /* Cancelling the transfer will safely terminate the state machine callback */
    fpi_usb_transfer_cancel (self->poll_transfer);
    self->poll_transfer = NULL;
  }

  fpi_image_device_deactivate_complete (dev, NULL);
}

/* * ============================================================================
 * CLASS AND TYPE REGISTRATION
 * ============================================================================
 */
static const FpIdEntry id_table[] = {
  { .vid = 0x2808,  .pid = 0x93a9, },
  { .vid = 0x2808,  .pid = 0x9338, }, /* The other known FocalTech PID */
  { .vid = 0,  .pid = 0,  .driver_data = 0 },
};

static void
fpi_device_ft9201_init (FpiDeviceFt9201 *self)
{
  /* Instance initialization (variables setup) */
}

static void
fpi_device_ft9201_class_init (FpiDeviceFt9201Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);
  FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS (klass);

  dev_class->id = "ft9201";
  dev_class->full_name = "FocalTech FT9201";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;

  /* Feature flags: This device can capture an image and detect a finger */
  dev_class->features = FP_DEVICE_FEATURE_CAPTURE | FP_DEVICE_FEATURE_IDENTIFY;

  img_class->img_width = FT9201_IMAGE_WIDTH;
  img_class->img_height = FT9201_IMAGE_HEIGHT;
  img_class->bz3_threshold = 24; /* Adjust based on Bozorth3 matching success */

  /* Bind Lifecycle Overrides */
  img_class->open = ft9201_open;
  img_class->close = ft9201_close;
  img_class->activate = ft9201_activate;
  img_class->deactivate = ft9201_deactivate;
}