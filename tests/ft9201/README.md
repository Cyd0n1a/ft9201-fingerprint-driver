# FT9201 umockdev test fixture

This directory wires ft9201 into libfprint's existing `umockdev` driver-test
infrastructure (see `tests/README.md` and `tests/umockdev-test.py` in the
libfprint tree). It is **intentionally incomplete**: the three files the
test runner actually replays —

```
device          umockdev-record dump of the USB device's sysfs/descriptors
capture.pcapng   full USB traffic capture of one init+capture cycle (tshark/usbmon)
capture.png      reference image — future captures are compared to this byte-for-byte
```

— do not exist here yet, and can't be generated without the physical FT9201
reader connected to a machine with root and `tshark`. Nothing in this
session has that, so rather than hand-fabricate a multi-hundred-KB binary
USB capture and call it a test fixture, this directory ships the wiring and
the exact recording command instead.

**Why not fake it:** a synthetic capture would make `meson test` report a
pass without having validated anything about real device timing, real
register values, or real bulk-endpoint behavior — which is precisely the
class of bug this fixture exists to catch. A fixture that always passes
regardless of whether the driver actually works is worse than no fixture,
because it looks like coverage.

**Until those three files exist**, the registered test isn't a no-op
failure — `umockdev-test.py` only calls into `capture()`/`custom()` if it
finds a `capture.*` file to glob, so `meson test ft9201` will report a
*pass* while not actually exercising the driver at all. Don't mistake a
green run for a working driver until the real fixture is in place.

## Generating the real fixture (one-shot, needs hardware)

1. Make sure ft9201 is staged and wired into the build (see the project's
   top-level `Makefile`: `make register`, then fix up the three meson.build
   entries it points you to if it says they're missing).
2. `make build` — the recording tool only exists once Meson has templated
   it into the build directory, and it loads the freshly built libfprint
   via `LD_LIBRARY_PATH`/`GI_TYPELIB_PATH`, so a stale or missing build
   means it's testing the wrong code.
3. Plug in the FT9201 reader (USB ID `2808:93a9`) and make sure nothing
   else has it open — `fprintd` in particular, if it's running.
4. `make record-test` (wraps the command below) — or directly:
   ```sh
   sudo build/tests/create-driver-test.py ft9201
   ```
   This needs root (to read `/dev/bus/usb/...` and run `tshark`) and will
   prompt you to touch the sensor partway through, while `capture.py` runs
   inside the umockdev sandbox.

   The capture gets committed to a test fixture, so consider using the
   side of a finger or a knuckle rather than an actual enrolled
   fingerprint — several existing libfprint test fixtures do the same for
   this reason (see `tests/README.md`'s note on this).
5. Confirm it replays cleanly: `meson test -C build ft9201 -v`, or
   `make test` from the wrapper Makefile.

## Registering the test (manual — meson.build isn't auto-edited)

Same philosophy as the `register` Makefile target: editing someone else's
build files with `sed` risks a diff that doesn't match what a human
reviewer would write, and risks drifting from whatever exact version of
these files your checkout has. Three places, verified against an actual
libfprint checkout while building this:

**`meson.build`** (top level) — add `'ft9201'` to the `default_drivers`
list:
```meson
    'fpcmoc',
    'ft9201',

    # SPI
    'elanspi',
```

**`libfprint/meson.build`** — add the driver's source file to the
driver-name → sources dict:
```meson
    'ft9201' :
        [ 'drivers/ft9201.c' ],
```

**`tests/meson.build`** — add `'ft9201'` to the `drivers_tests` list:
```meson
    'egismoc',
    'fpcmoc',
    'ft9201',
]
```

No changes are needed to `tests/capture.py` — it's already generic across
every image device. It asserts `CAPTURE`, `IDENTIFY`, and `VERIFY` are
present and that `DUPLICATES_CHECK`/`STORAGE`/`STORAGE_LIST`/
`STORAGE_DELETE`/`STORAGE_CLEAR` are absent, which already matches
ft9201's class registration (`CAPTURE | IDENTIFY | VERIFY | ENROLL`, no
storage features) — checked directly against `ft9201.c`'s
`fpi_device_ft9201_class_init()`.
