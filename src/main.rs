//! deckontroller: presents the Steam Deck's own internal controller to a second computer over USB-C as a genuine Steam Deck Controller,
//! with full Steam Input support (trackpads, gyro, all four rear buttons).
//!
//! How: reads the real controller's exact HID report descriptor and
//! VID/PID, build a USB gadget that presents an identical interface,
//! then transparently relay raw bytes both directions.
//!
//! doesn't parse or modify Valve's report format.
//!
//! Requires: USB Dual-Role Device (DRD) enabled in the BIOS,
//!           and running this as root (configfs + hidraw + FunctionFS all need it)

mod ffs;
mod gadget;
mod hid;
mod relay;

use std::env;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};

static VERBOSE: AtomicBool = AtomicBool::new(false);

pub fn verbose_enabled() -> bool {
    VERBOSE.load(Ordering::Relaxed)
}

fn usage(prog: &str) -> String {
    format!(
        "deckontroller by coderobe\nusage: {prog} run [--pid 0xNNNN] [--verbose]\n       {prog} teardown [--verbose]\n\n\
         run        Discover the internal controller, stand up the USB gadget,\n\
                     and relay until killed (Ctrl-C or systemd stop).\n\
         teardown   Tear down a gadget left behind by a previous run.\n\
         --pid      Pin the internal controller's USB product ID instead of\n\
                     auto-detecting by vendor usage page (0x28de:0x1205 is the\n\
                     Steam Deck's own controller; override if yours differs).\n\
         --verbose   Show configfs operations and detailed USB/HID relay traces."
    )
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    let prog = args
        .get(0)
        .cloned()
        .unwrap_or_else(|| "deckontroller".into());
    VERBOSE.store(args.iter().any(|arg| arg == "--verbose"), Ordering::Relaxed);

    if unsafe { libc::geteuid() } != 0 {
        eprintln!("this must run as root (configfs, hidraw and FunctionFS all require it)");
        return ExitCode::FAILURE;
    }

    match args.get(1).map(String::as_str) {
        Some("run") => {
            let want_pid = args
                .iter()
                .position(|a| a == "--pid")
                .and_then(|i| args.get(i + 1))
                .and_then(|s| u16::from_str_radix(s.trim_start_matches("0x"), 16).ok());

            match run(want_pid) {
                Ok(()) => ExitCode::SUCCESS,
                Err(e) => {
                    eprintln!("fatal: {e}");
                    ExitCode::FAILURE
                }
            }
        }

        Some("teardown") => match gadget::Gadget::teardown_existing() {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("teardown failed: {e}");
                ExitCode::FAILURE
            }
        },

        _ => {
            eprintln!("{}", usage(&prog));
            ExitCode::FAILURE
        }
    }
}

fn run(want_pid: Option<u16>) -> std::io::Result<()> {
    eprintln!("[deckontroller] looking for the internal controller...");

    let controller = hid::discover(want_pid)?;
    let auxiliary = hid::discover_auxiliary_interfaces(&controller)?;

    eprintln!(
        "[deckontroller] found {} (vendor 0x{:04x} product 0x{:04x}), report descriptor {} bytes",
        controller.path.display(),
        controller.info.vendor as u16,
        controller.info.product as u16,
        controller.report_descriptor.len()
    );
    eprintln!(
        "[deckontroller] found sibling mouse {} and keyboard {}",
        auxiliary.mouse.path.display(),
        auxiliary.keyboard.path.display()
    );

    gadget::ensure_libcomposite_loaded()?;

    let product_id = controller.info.product as u16;
    let vendor_id = controller.info.vendor as u16;

    let g = gadget::Gadget::create(vendor_id, product_id, "Steam Controller")?;

    let spec = ffs::HidInterfaceSpec {
        report_descriptor: controller.report_descriptor.clone(),
        interface_string_index: 1,
    };

    let descriptors = ffs::build_descriptors_blob(&spec);
    let strings = ffs::build_strings_blob("Steam Controller");

    // IMPORTANT PART I SPENT WAY TOO LONG DEBUGGING:
    // FunctionFS requires the EP0 file descriptor to remain open while the function is active.
    // So do NOT drop this handle before binding the gadget to the UDC or it will mysteriously disappear and fail.
    //
    // Keep this handle alive until after relay::run() returns.
    let mut ep0 = ffs::Ep0::open(&g.ffs_mount)?;

    ep0.write_descriptors(&descriptors, &strings)?;

    eprintln!("[deckontroller] FunctionFS descriptors registered");

    if let Err(ready_error) = g.wait_ready() {
        drop(ep0);
        return match g.teardown() {
            Ok(()) => Err(ready_error),
            Err(cleanup_error) => Err(std::io::Error::new(
                ready_error.kind(),
                format!("{ready_error}; cleanup also failed: {cleanup_error}"),
            )),
        };
    }

    eprintln!("[deckontroller] FunctionFS ready");

    if let Err(bind_error) = g.bind_udc() {
        eprintln!("[deckontroller] failed to bind gadget to UDC: {bind_error}");
        // The descriptor-owning FunctionFS fd prevents unmounting, but is no longer needed after a failed bind
        drop(ep0);
        return match g.teardown() {
            Ok(()) => Err(bind_error),
            Err(cleanup_error) => Err(std::io::Error::new(
                bind_error.kind(),
                format!("{bind_error}; cleanup also failed: {cleanup_error}"),
            )),
        };
    }

    eprintln!("[deckontroller] gadget bound, waiting for host to enumerate us...");

    let (mouse_hidg, keyboard_hidg) = match g.hid_gadget_devices() {
        Ok(devices) => devices,
        Err(device_error) => {
            let unbind_result = g.unbind_udc();
            drop(ep0);
            let teardown_result = g.teardown();
            return match (unbind_result, teardown_result) {
                (Ok(()), Ok(())) => Err(device_error),
                (Err(unbind_error), Ok(())) => Err(std::io::Error::new(
                    device_error.kind(),
                    format!("{device_error}; unbinding also failed: {unbind_error}"),
                )),
                (Ok(()), Err(teardown_error)) => Err(std::io::Error::new(
                    device_error.kind(),
                    format!("{device_error}; cleanup also failed: {teardown_error}"),
                )),
                (Err(unbind_error), Err(teardown_error)) => Err(std::io::Error::new(
                    device_error.kind(),
                    format!(
                        "{device_error}; unbinding also failed: {unbind_error}; cleanup also failed: {teardown_error}"
                    ),
                )),
            };
        }
    };

    // Keep the sole EP0 handle alive for the entire lifetime of the gadget.
    let result = relay::run(
        controller,
        auxiliary,
        &g.ffs_mount,
        &mouse_hidg,
        &keyboard_hidg,
        &mut ep0,
    );

    // Unbind while the descriptor-owning EP0 handle is still open, then close it before unmounting FunctionFS.
    // Keeping it through the unmount makes `umount` fail with EBUSY
    let unbind_result = g.unbind_udc();
    drop(ep0);
    let teardown_result = g.teardown();

    match (result, unbind_result, teardown_result) {
        (Err(run_error), Ok(()), Ok(())) => Err(run_error),

        (Ok(()), Err(unbind_error), Ok(())) => Err(unbind_error),

        (Ok(()), Ok(()), Err(teardown_error)) => Err(teardown_error),

        (Err(run_error), Err(unbind_error), Ok(())) => Err(std::io::Error::new(
            run_error.kind(),
            format!("{run_error}; unbinding also failed: {unbind_error}"),
        )),

        (Err(run_error), Ok(()), Err(teardown_error)) => Err(std::io::Error::new(
            run_error.kind(),
            format!("{run_error}; cleanup also failed: {teardown_error}"),
        )),

        (Ok(()), Err(unbind_error), Err(teardown_error)) => Err(std::io::Error::new(
            unbind_error.kind(),
            format!("{unbind_error}; cleanup also failed: {teardown_error}"),
        )),

        (Err(run_error), Err(unbind_error), Err(teardown_error)) => Err(std::io::Error::new(
            run_error.kind(),
            format!(
                "{run_error}; unbinding also failed: {unbind_error}; cleanup also failed: {teardown_error}"
            ),
        )),

        (Ok(()), Ok(()), Ok(())) => Ok(()),
    }
}
