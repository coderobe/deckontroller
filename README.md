# deckontroller: Steam Deck as USB controller

Presents the Steam Deck's **own internal controller** to a second computer
over USB-C as a genuine Steam Deck Controller with full Steam Input support.  

**Supports both trackpads, gyro, all four rear grip buttons, rumble, and anything normal steam input does.**

__Not a generic HID gamepad emulation.__

**Warning:** This passes a bunch of USB traffic through directly between your host and the Steam Deck controller.
I think at least for the regular Steam Controller and other Valve accessories, Steam may perform firmware updates and stuff like that.
I have no idea if or how this applies to the deck's controller, so use this software at your own risk.  
tl;dr: while this software itself does not interface with the Deck's controller in any way that could cause problems, software on your computer might. i use this software myself and will update this README in the event anything spooky happens.  


## Runtime requirements

- USB **Dual-Role Device (DRD)** mode enabled in the BIOS: shut down, hold
  `Volume Up` + `Power`, go to `Setup Utility` -> `Advanced` -> `USB
  Configuration` -> `USB Dual Role Device` -> change `XHCI` to `DRD` -> `Exit
  Saving Changes`.
- I recommend running this from Desktop mode, with the Steam client **fully closed** so that only one Steam client (the one on your computer) speaks to the controller at any given time. You can set up a terminal shortcut on your desktop for this, or launch deckontroller via SSH.

## Compile-time requirements

The makefile uses `cargo-zigbuild` to statically cross compile deckontroller for the Steam Deck (`x86_64-unknown-linux-musl`). This means compiling also works from macOS ^^  

tl;dr: If you want to compile this from source, this means you either need to have `rust`, `zig`, and `cargo-zigbuild` installed, or build it normally from a compatible system.

## Build

```
make
```

Produces `target/x86_64-unknown-linux-musl/release/deckontroller`. This binary can be copied to the steam deck.  
No additional runtime dependencies are required.

## Run

```
sudo ./deckontroller run
```

Connect the Steam Deck to another computer with a usb cable and you should be good to go.
Ctrl-C the program to return the controls to the Steam Deck, or `sudo ./deckontroller teardown`

On the laptop side: open Steam, go to `Settings` -> `Controller` -> make
sure "Steam Deck Controller" support is enabled (Valve's own controllers
are recognized without needing the "Generic gamepad configuration
support" toggle GadgetDeck-style software requires)

Steam should just show it as a normal Deck controller now since as far as it can tell, that's exactly what it is.  
If it does not show up under `Settings` -> `Controller`, make sure deckontroller has started without errors and the deck is connected to the computer. If it still does not show up, restart Steam on your computer.


### If it doesn't enumerate

1. `dmesg -w` on the Deck while running: the kernel logs why FunctionFS rejected a descriptor set, if it did.
2. On the laptop, `lsusb -v` (Linux) or Device Manager (Windows) to see whether *anything* enumerated, and what VID/PID/class it reported.

   ### If the UDC is busy

   The Deck exposes only one USB Device Controller, so it can host only one
   USB gadget at a time. If startup says that the UDC is already bound, another
   gadget owns it. Stop the service or application that owns the reported
   configfs gadget before running `deckontroller`. `deckontroller` intentionally
   does not unbind another gadget on its own, because doing so would disconnect
   that device unexpectedly.  

   If the UDC has no owner but is unavailable for peripheral mode, the kernel
   could not start the gadget driver. Confirm that DRD mode is enabled in the
   BIOS, then connect the Deck to a USB host with a data-capable cable before
   starting deckontroller.  

   If it still does not work, please open an issue report with as much information (deckontroller output, deck dmesg, host lsusb or device manager info, host system info, etc) as possible, and describe the issue you are having.

## Info for macOS users
The awesome project CrossPuck (https://github.com/scryner/crosspuck) can forward the deckontroller to Steam running in Wine or CrossOver if you patch the USB VID/PID it is looking for to be 28de:1205 Valve Software Steam Controller instead of the Steam Controller Puck.

## Acknowledgements

This software (deckontroller) was originally written in 2026 by Mara Broda (@coderobe).  

It would have been significantly more work to understand the Deck and make this software, if it weren't for:  

https://github.com/Frederic98/GadgetDeck - Another neat USB Gadget software for the Steam Deck, including partial gamepad emulation.  
https://github.com/hifihedgehog/HIDMaestro - Virtual game controllers for Windows that show up as real hardware.  
SDL & linux (hid-steam) - Runtime and driver-side code helped understanding the controller traffic while debugging.  

Their information and writeups on the Steam Deck controller were instrumental to getting deckontroller working and recognised by Steam.
