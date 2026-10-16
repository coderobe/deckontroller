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
mod usbdesc;

use std::env;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};

static VERBOSE: AtomicBool = AtomicBool::new(false);

pub fn verbose_enabled() -> bool {
    VERBOSE.load(Ordering::Relaxed)
}

fn usage(prog: &str) -> String {
    format!(
        "deckontroller by coderobe\nusage: {prog} run [--pid 0xNNNN] [--usage-page 0xFFNN] [--verbose]\n       {prog} teardown [--verbose]\n\n\
         run        Discover the internal controller, stand up the USB gadget,\n\
                     and relay until killed (Ctrl-C or systemd stop).\n\
         teardown   Tear down a gadget left behind by a previous run.\n\
         --pid      Pin the internal controller's USB product ID instead of\n\
                     auto-detecting by vendor usage page (0x28de:0x1205 is the\n\
                     Steam Deck's own controller; override if yours differs).\n\
         --usage-page  Advertise this HID usage page for the vendor (controller)\n\
                     interface to the host instead of the real one (the Deck\n\
                     reports 0xFFFF). Must be vendor-defined, 0xFF00-0xFFFF.\n\
                     Only the usage page label is changed; descriptor length\n\
                     and report layout stay identical. 0xFF00 is the original\n\
                     Steam Controller's page.\n\
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

            let want_usage_page = match args.iter().position(|a| a == "--usage-page") {
                None => None,
                Some(i) => {
                    let parsed = args.get(i + 1).and_then(|s| {
                        let t = s.trim_start_matches("0x").trim_start_matches("0X");
                        u16::from_str_radix(t, 16).ok()
                    });
                    match parsed {
                        Some(v) if (0xff00..=0xffff).contains(&v) => Some(v),
                        _ => {
                            eprintln!("--usage-page needs a vendor-defined hex value between 0xFF00 and 0xFFFF (e.g. --usage-page 0xFF00)");
                            return ExitCode::FAILURE;
                        }
                    }
                }
            };

            match run(want_pid, want_usage_page) {
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

fn run(want_pid: Option<u16>, want_usage_page: Option<u16>) -> std::io::Result<()> {
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

    // Present the same USB identity as the physical device: bcdDevice, manufacturer, product and serial strings
    let identity = hid::UsbIdentity::read(&controller)?;
    eprintln!(
        "[deckontroller] forwarding USB identity: {:04x}:{:04x} bcdDevice={} manufacturer={:?} product={:?} serial={:?} configuration={:?}",
        identity.vendor_id,
        identity.product_id,
        identity
            .bcd_device
            .map_or_else(|| "unknown".to_owned(), |v| format!("0x{v:04x}")),
        identity.manufacturer,
        identity.product,
        identity.serial,
        identity.configuration,
    );

    // ...and the same descriptor-level details: device class, bcdUSB, power, interface subclass/protocol, HID descriptor, endpoint
    let profile = usbdesc::PhysicalProfile::read(&controller, &auxiliary);
    verbose_log_profile(&profile);

    let g = gadget::Gadget::create(&identity, &profile)?;

    // Mouse, keyboard and vendor interfaces, in the physical device's interface order
    let layout = build_layout(&controller, &auxiliary, &profile, want_usage_page);
    for (index, spec) in layout.iter().enumerate() {
        eprintln!(
            "[deckontroller] interface {index}: {:?} subclass={} protocol={} bcdHID=0x{:04x} country={} endpoint=0x{:02x} maxpacket={} interval={} string={:?}",
            spec.role,
            spec.subclass,
            spec.protocol,
            spec.bcd_hid,
            spec.country_code,
            spec.endpoint_address,
            spec.max_packet,
            spec.interval,
            spec.string,
        );
    }

    let descriptors = ffs::build_descriptors_blob(&layout);
    let strings = ffs::build_strings_blob(&layout);

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

    // Keep the sole EP0 handle alive for the entire lifetime of the gadget.
    let result = relay::run(
        controller,
        auxiliary,
        &g.ffs_mount,
        &layout,
        &mut ep0,
        || g.unbind_udc(),
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

fn verbose_log_profile(profile: &usbdesc::PhysicalProfile) {
    if verbose_enabled() {
        eprintln!("[deckontroller] mirroring physical USB descriptors: {profile:#?}");
    }
}

/// Describe the gadget's interfaces by copying every value from the physical device.
/// They are ordered by the physical interface number so host-visible interface numbers match the real device
/// (the real Deck uses mouse, keyboard, vendor); if the numbers can't be read that is also the default order.
fn build_layout(
    controller: &hid::HidRaw,
    auxiliary: &hid::SteamDeckAuxiliaryInterfaces,
    profile: &usbdesc::PhysicalProfile,
    usage_page: Option<u16>,
) -> Vec<ffs::InterfaceSpec> {
    let mut physical = vec![
        (ffs::Role::Mouse, &auxiliary.mouse.report_descriptor, profile.mouse.as_ref()),
        (ffs::Role::Keyboard, &auxiliary.keyboard.report_descriptor, profile.keyboard.as_ref()),
        (ffs::Role::Vendor, &controller.report_descriptor, profile.vendor.as_ref()),
    ];
    if physical.iter().all(|(_, _, iface)| iface.is_some()) {
        physical.sort_by_key(|(_, _, iface)| iface.map_or(u8::MAX, |i| i.number));
    }

    physical
        .into_iter()
        .enumerate()
        .map(|(position, (role, report_descriptor, iface))| {
            let mut spec = ffs::InterfaceSpec::with_defaults(role, report_descriptor.clone(), position);
            // Optionally advertise a different vendor usage page for the controller interface. Only the copy sent to the host
            // is modified: the physical device's descriptor (used for discovery and relaying) is left untouched
            if role == ffs::Role::Vendor {
                if let Some(page) = usage_page {
                    let changed = hid::rewrite_vendor_usage_page(&mut spec.report_descriptor, page);
                    if changed == 0 {
                        eprintln!("[deckontroller] warning: --usage-page given but the vendor report descriptor has no vendor usage page to rewrite; descriptor left unchanged");
                    } else {
                        eprintln!("[deckontroller] rewrote vendor usage page to 0x{page:04x} in {changed} descriptor item(s) advertised to the host");
                    }
                }
            }
            let Some(iface) = iface else { return spec };
            spec.subclass = iface.subclass;
            spec.protocol = iface.protocol;
            spec.string = iface.interface_string.clone();
            if let Some(hid) = &iface.hid {
                spec.bcd_hid = hid.bcd_hid;
                spec.country_code = hid.country_code;
            }
            // Only interrupt IN endpoints are relayed
            const INTERRUPT: u8 = 0x03;
            if iface.endpoints.len() > 1 {
                eprintln!(
                    "[deckontroller] warning: physical {role:?} interface has {} endpoints; only its first interrupt IN endpoint is mirrored",
                    iface.endpoints.len()
                );
            }
            if let Some(ep) = iface
                .endpoints
                .iter()
                .find(|e| e.address & 0x80 != 0 && e.attributes & 0x03 == INTERRUPT)
            {
                // The endpoint address is deliberately not copied: see InterfaceSpec::endpoint_address
                // Packet size and interval are only meaningful to copy if the real link is full-speed like the gadget
                if profile.full_speed {
                    spec.max_packet = ep.max_packet_size.min(64);
                    spec.interval = ep.interval;
                }
            }
            spec
        })
        .collect()
}
