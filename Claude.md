# MISSION DIRECTIVE: FT9201 libfprint User-Space Driver Port

## 1. Context and Primary Goal
You are tasked with porting an existing Linux kernel module (`ft9201.c`) for the FocalTech FT9201 fingerprint reader (USB ID: `2808:93a9`) into a pure user-space biometric driver for the `libfprint` project. 

The existing kernel module successfully interacts with the hardware but uses a fundamentally incompatible architecture (kernel-space character device mapping). Your goal is to reverse-engineer the hexadecimal USB control payloads and bulk transfer timings from the existing kernel module and wrap them securely inside `libfprint`'s GLib/GObject asynchronous architecture using `libgusb`.

## 2. Strict Architectural Objectives (Non-Negotiable)
To ensure the resulting driver is eligible for upstream merging into the official `freedesktop.org` repository, you must strictly adhere to the following architectural constraints. Any deviation from these principles will result in a rejected implementation.

* **Strict User-Space Execution:** You are explicitly forbidden from interacting with `/dev/` nodes, writing `.ko` files, or using kernel `ioctl` commands. All hardware communication must occur via `FpiUsbTransfer` and `libgusb` API calls.
* **Class Inheritance Enforcement:** The FT9201 is a Match-on-Host scanner. The driver must inherit from `FpImageDevice`, not the base `FpDevice`. Do not implement cryptographic matching or Bozorth3 algorithms; your sole responsibility is to extract a clean 64x80 grayscale image buffer and pass it to the parent class.
* **Asynchronous Non-Blocking Flow:** The driver must never block the main thread. You are strictly forbidden from using `sleep()`, `usleep()`, or synchronous while-loops to wait for hardware readiness. All multi-step USB operations must be implemented using `FpiSsm` (Sequential State Machines).
* **Lifecycle Method Implementation:** You must implement and correctly bind the standard `FpImageDeviceClass` overrides: `open`, `close`, `activate`, and `deactivate`.
* **Zero-Leak Memory Management:** All allocated `FpiUsbTransfer` objects and GLib variables must be explicitly freed or cancelled during the `deactivate` or `close` phases to prevent memory leaks during prolonged system uptime.

## 3. Translation Guide: Kernel to libfprint
When reviewing the reference kernel module, map its logical components to the `libfprint` API using the following translation matrix:

### A. Initialization & Device Waking
* **Kernel Module:** Uses blocking `usb_control_msg()` calls in an initialization function.
* **libfprint Target:** Implement an `FpiSsm` (e.g., `init_ssm`). Use `fpi_usb_transfer_new()` and `fpi_usb_transfer_fill_control()` to send the setup packets. Advance the state machine only inside the asynchronous callback when the transfer successfully completes.

### B. Hardware Interrupt Polling
* **Kernel Module:** Waits on wait-queues or uses custom URB interrupt callbacks.
* **libfprint Target:** During the `activate` phase, submit a continuous interrupt transfer using `fpi_usb_transfer_fill_interrupt()`. When the callback fires indicating a finger is present, call `fpi_image_device_report_finger_status(device, TRUE)` and immediately trigger the bulk read state machine.

### C. Image Extraction
* **Kernel Module:** Dumps raw byte streams into a `/dev/` file for external Python/ImageMagick scripts to assemble.
* **libfprint Target:** In your bulk transfer callback, allocate a new image using `fp_image_new(64, 80)`. Copy the `transfer->buffer` payload precisely into the `image->data` array. Submit the completed image to the daemon using `fpi_image_device_image_captured()`.

## 4. State Machine Architecture Plan
You will implement two primary Sequential State Machines:

1. **`init_ssm`:** Triggered during `dev_class->open`.
   * *State 0:* Send hardware wakeup packet.
   * *State 1:* Request firmware version/status.
   * *State 2:* Send sensor gain and contrast configuration payloads.
   * *Conclusion:* Call `fpi_image_device_open_complete()`.

2. **`capture_ssm`:** Triggered during `img_class->activate`.
   * *State 0:* Submit interrupt transfer and wait (handled asynchronously).
   * *State 1 (Triggered by Interrupt CB):* Submit bulk transfer to read the 64x80 image array.
   * *Conclusion:* Assemble `FpImage`, submit to daemon, and reset state machine for the next read if required.

## 5. Development and Testing Workflow
* **Sandbox Environment:** You will be executing within a secure, headless virtual machine. 
* **Compilation:** Use the `meson` build system. Run `meson setup builddir` followed by `ninja -C builddir` to compile the C code.
* **Testing:** Do not attempt to interact with physical hardware. We will use `umockdev`. Assume all test traffic will be provided via `.pcapng` files. Your state machine must perfectly mirror the expected USB transfer sequence, or the `umockdev` virtual bus will throw a desynchronization fault.

## 6. Execution Command
Begin by reviewing the provided `ft9201.c` kernel module source code. Extract the `usb_control_msg` hex arrays, define the `FpiDeviceFt9201` class structure, and write the complete, compilable `libfprint` driver source file adhering strictly to the architecture defined above. Output the complete C code heavily commented with GLib documentation standards.