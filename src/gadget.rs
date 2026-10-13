//! Creates the USB gadget in configfs, mounts the FunctionFS instance for our function,
//! and binds/unbinds it to the Deck's dual-role USBcontroller
//! NOTE: USB controller must be switched to DRD mode in the BIOS for this to work

use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

use crate::hid::UsbIdentity;
use crate::usbdesc::PhysicalProfile;

const CONFIGFS_ROOT: &str = "/sys/kernel/config/usb_gadget";
const GADGET_NAME: &str = "steamdeck_passthrough";
const FUNCTION_NAME: &str = "steamdeck"; // -> functions/ffs.steamdeck, mount source "steamdeck"

// Functions that older versions added next to the FunctionFS one (stock f_hid for mouse and keyboard, plus a CDC ACM serial port).
// The gadget no longer creates them; teardown still removes them so a gadget left behind by an older build can be cleaned up.
const LEGACY_MOUSE_FUNCTION_NAME: &str = "mouse";
const LEGACY_KEYBOARD_FUNCTION_NAME: &str = "keyboard";
const LEGACY_ACM_FUNCTION_NAME: &str = "GS0";

// Only used when the physical device doesn't report the field
const FALLBACK_BCD_DEVICE: u16 = 0x0100;
const FALLBACK_MANUFACTURER: &str = "Valve Software";
const FALLBACK_PRODUCT: &str = "Steam Controller";

pub struct Gadget {
    pub root: PathBuf,
    pub ffs_mount: PathBuf,
}

fn write_file(path: impl AsRef<Path>, contents: &str) -> io::Result<()> {
    log(&format!("writing {}", path.as_ref().display()));
    let mut f = fs::OpenOptions::new()
        .write(true)
        .truncate(true)
        .open(&path)
        .map_err(|e| {
            io::Error::new(
                e.kind(),
                format!("writing {}: {e}", path.as_ref().display()),
            )
        })?;
    f.write_all(contents.as_bytes())
}

fn link_function(function: &Path, config: &Path, name: &str) -> io::Result<()> {
    let link = config.join(name);
    if link.exists() {
        return Ok(());
    }
    log(&format!(
        "linking {} to {}",
        link.display(),
        function.display()
    ));
    std::os::unix::fs::symlink(function, &link).map_err(|e| {
        io::Error::new(
            e.kind(),
            format!("linking {} to {}: {e}", link.display(), function.display()),
        )
    })
}

impl Gadget {
    /// Idempotent: safe to call even if a previous run left the gadget half-configured.
    /// Returns once configfs + the ffs mount are ready;
    /// NOTE: the caller still has to write descriptors to ep0 *before* calling `bind_udc`
    pub fn create(identity: &UsbIdentity, profile: &PhysicalProfile) -> io::Result<Self> {
        let root = PathBuf::from(CONFIGFS_ROOT).join(GADGET_NAME);
        let ffs_mount = PathBuf::from("/dev/deckontroller-ffs");

        // A previous crashed run may have left this partially set up.
        // Tear down first so we start from a known state.
        log("removing any previous gadget state");
        Self::teardown_inner(&root, &ffs_mount)?;

        log("creating USB gadget configuration");
        create_dir_all(&root)?;
        write_file(root.join("idVendor"), &format!("0x{:04x}\n", identity.vendor_id))?;
        write_file(root.join("idProduct"), &format!("0x{:04x}\n", identity.product_id))?;
        write_file(
            root.join("bcdDevice"),
            &format!("0x{:04x}\n", identity.bcd_device.unwrap_or(FALLBACK_BCD_DEVICE)),
        )?;
        write_file(
            root.join("bcdUSB"),
            &format!(
                "0x{:04x}\n",
                profile.device.as_ref().map_or(0x0200, |d| d.bcd_usb)
            ),
        )?;
        if let Some(device) = &profile.device {
            write_file(root.join("bDeviceClass"), &format!("0x{:02x}\n", device.class))?;
            write_file(
                root.join("bDeviceSubClass"),
                &format!("0x{:02x}\n", device.subclass),
            )?;
            write_file(
                root.join("bDeviceProtocol"),
                &format!("0x{:02x}\n", device.protocol),
            )?;
            // The UDC may refuse sizes it can't do; it only accepts the values a real device would use
            if let Err(e) = write_file(
                root.join("bMaxPacketSize0"),
                &format!("0x{:02x}\n", device.max_packet_size0),
            ) {
                log(&format!("could not set bMaxPacketSize0, keeping default: {e}"));
            }
        }
        write_file(root.join("max_speed"), "full-speed\n")?;

        // Forward the physical device's string descriptors; a string the real device doesn't have is left unset
        let strings = root.join("strings/0x409");
        create_dir_all(&strings)?;
        write_file(
            strings.join("manufacturer"),
            &format!(
                "{}\n",
                identity.manufacturer.as_deref().unwrap_or(FALLBACK_MANUFACTURER)
            ),
        )?;
        write_file(
            strings.join("product"),
            &format!("{}\n", identity.product.as_deref().unwrap_or(FALLBACK_PRODUCT)),
        )?;
        if let Some(serial) = &identity.serial {
            write_file(strings.join("serialnumber"), &format!("{serial}\n"))?;
        }

        let config = root.join("configs/c.1");
        create_dir_all(&config)?;
        // configfs expresses MaxPower directly in milliamps
        let (max_power_ma, attributes) = profile
            .config
            .as_ref()
            .map_or((500, 0x80), |c| (c.max_power_ma, c.attributes));
        write_file(config.join("MaxPower"), &format!("{max_power_ma}\n"))?;
        write_file(config.join("bmAttributes"), &format!("0x{attributes:02x}\n"))?;
        if let Some(configuration) = &identity.configuration {
            let config_strings = config.join("strings/0x409");
            create_dir_all(&config_strings)?;
            write_file(config_strings.join("configuration"), &format!("{configuration}\n"))?;
        }

        // One FunctionFS function carries every interface (mouse, keyboard, vendor HID), so their descriptors can be copied
        // from the physical device exactly. Interface order and endpoint allocation follow the order in its descriptors blob.
        let function = root.join(format!("functions/ffs.{FUNCTION_NAME}"));
        create_dir_all(&function)?;
        link_function(&function, &config, &format!("ffs.{FUNCTION_NAME}"))?;

        create_dir_all(&ffs_mount)?;
        log("mounting FunctionFS");
        run(&[
            "mount",
            "-t",
            "functionfs",
            FUNCTION_NAME,
            ffs_mount.to_str().unwrap(),
        ])?;

        Ok(Gadget { root, ffs_mount })
    }

    /// Bind the gadget to a UDC (USB Device Controller), which makes it visible on the wire.
    /// NOTE: Must be called only after ep0's descriptors have already been written;
    ///       the kernel starts answering enumeration requests as soon as this completes.
    pub fn bind_udc(&self) -> io::Result<()> {
        let udc = first_udc()?;
        match write_file(self.root.join("UDC"), &format!("{udc}\n")) {
            Ok(()) => Ok(()),
            Err(e) if e.kind() == io::ErrorKind::ResourceBusy => {
                match udc_owner(&udc)? {
                    Some(owner) => Err(io::Error::new(
                        io::ErrorKind::ResourceBusy,
                        format!(
                            "UDC {udc} is already bound by {}; stop or unbind that gadget before starting deckontroller",
                            owner.display()
                        ),
                    )),
                    None => Err(io::Error::new(
                        io::ErrorKind::ResourceBusy,
                        format!(
                            "UDC {udc} rejected the gadget despite having no configfs owner; \
                             it is unavailable for peripheral mode. Verify DRD mode in the BIOS, \
                             connect a data-capable cable to a USB host, and retry"
                        ),
                    )),
                }

            }
            Err(e) => Err(e),
        }
    }

    /// Verify that FunctionFS accepted ep0's descriptor and string blocks before making the gadget visible to the USB host.
    pub fn wait_ready(&self) -> io::Result<()> {
        let ready = self
            .root
            .join(format!("functions/ffs.{FUNCTION_NAME}/ready"));
        let value = fs::read_to_string(&ready)?;
        if value.trim() == "1" {
            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::Other,
                format!(
                    "FunctionFS is not ready at {} (expected 1, found {:?})",
                    ready.display(),
                    value.trim()
                ),
            ))
        }
    }

    /// Idempotent: unbinding a gadget that is already unbound (ENODEV) is not an error
    pub fn unbind_udc(&self) -> io::Result<()> {
        match write_file(self.root.join("UDC"), "\n") {
            Err(e) if e.raw_os_error() == Some(libc::ENODEV) => Ok(()),
            other => other,
        }
    }

    pub fn teardown(&self) -> io::Result<()> {
        Self::teardown_inner(&self.root, &self.ffs_mount)
    }

    /// Remove a gadget left behind by an earlier process without first attempting to create or mount a new one
    pub fn teardown_existing() -> io::Result<()> {
        let root = PathBuf::from(CONFIGFS_ROOT).join(GADGET_NAME);
        let ffs_mount = PathBuf::from("/dev/deckontroller-ffs");
        Self::teardown_inner(&root, &ffs_mount)
    }

    fn teardown_inner(root: &Path, ffs_mount: &Path) -> io::Result<()> {
        let udc = root.join("UDC");
        if udc.exists() {
            log(&format!("unbinding {}", udc.display()));
            match write_file(&udc, "\n") {
                Ok(()) => {}
                // The UDC selected by a crashed prior process can disappear before cleanup runs
                // e.g. there may not be a remaining binding to clear
                Err(e) if e.raw_os_error() == Some(libc::ENODEV) => {}
                Err(e) => {
                    return Err(io::Error::new(
                        e.kind(),
                        format!("unbinding previous gadget: {e}"),
                    ));
                }
            }
        }
        if is_mounted(ffs_mount)? {
            log(&format!("unmounting {}", ffs_mount.display()));
            run(&["umount", ffs_mount.to_str().unwrap_or_default()]).map_err(|e| {
                io::Error::new(
                    e.kind(),
                    format!("unmounting previous FunctionFS instance: {e}"),
                )
            })?;
        }

        let config = root.join("configs/c.1");
        remove_file_if_exists(&config.join(format!("acm.{LEGACY_ACM_FUNCTION_NAME}")))?;
        remove_file_if_exists(&config.join(format!("ffs.{FUNCTION_NAME}")))?;
        remove_file_if_exists(&config.join(format!("hid.{LEGACY_KEYBOARD_FUNCTION_NAME}")))?;
        remove_file_if_exists(&config.join(format!("hid.{LEGACY_MOUSE_FUNCTION_NAME}")))?;
        remove_dir_if_exists(&config.join("strings/0x409"))?;
        remove_dir_if_exists(&config)?;
        remove_dir_if_exists(&root.join(format!("functions/acm.{LEGACY_ACM_FUNCTION_NAME}")))?;
        remove_dir_if_exists(&root.join(format!("functions/ffs.{FUNCTION_NAME}")))?;
        remove_dir_if_exists(&root.join(format!("functions/hid.{LEGACY_KEYBOARD_FUNCTION_NAME}")))?;
        remove_dir_if_exists(&root.join(format!("functions/hid.{LEGACY_MOUSE_FUNCTION_NAME}")))?;
        remove_dir_if_exists(&root.join("strings/0x409"))?;
        remove_dir_if_exists(root)?;
        remove_dir_if_exists(ffs_mount)?;
        Ok(())
    }
}

fn is_mounted(path: &Path) -> io::Result<bool> {
    log(&format!("checking whether {} is mounted", path.display()));
    let path = path.to_string_lossy();
    Ok(fs::read_to_string("/proc/self/mountinfo")?
        .lines()
        .any(|line| line.split_whitespace().nth(4) == Some(path.as_ref())))
}

fn create_dir_all(path: &Path) -> io::Result<()> {
    log(&format!("creating {}", path.display()));
    fs::create_dir_all(path)
        .map_err(|e| io::Error::new(e.kind(), format!("creating {}: {e}", path.display())))
}

fn remove_file_if_exists(path: &Path) -> io::Result<()> {
    log(&format!("removing {}", path.display()));
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(io::Error::new(
            e.kind(),
            format!("removing {}: {e}", path.display()),
        )),
    }
}

fn remove_dir_if_exists(path: &Path) -> io::Result<()> {
    log(&format!("removing {}", path.display()));
    match fs::remove_dir(path) {
        Ok(()) => Ok(()),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(io::Error::new(
            e.kind(),
            format!("removing {}: {e}", path.display()),
        )),
    }
}

fn first_udc() -> io::Result<String> {
    for entry in fs::read_dir("/sys/class/udc")? {
        let entry = entry?;
        return Ok(entry.file_name().to_string_lossy().into_owned());
    }
    Err(io::Error::new(
        io::ErrorKind::NotFound,
        "no UDC found under /sys/class/udc — is USB Dual-Role Device (DRD) mode \
         enabled in the BIOS? (Volume Up + Power -> Setup Utility -> Advanced -> \
         USB Configuration -> USB Dual Role Device -> DRD)",
    ))
}

fn udc_owner(udc: &str) -> io::Result<Option<PathBuf>> {
    for entry in fs::read_dir(CONFIGFS_ROOT)? {
        let path = entry?.path();
        let binding = match fs::read_to_string(path.join("UDC")) {
            Ok(binding) => binding,
            Err(e) if e.kind() == io::ErrorKind::NotFound => continue,
            Err(e) => return Err(e),
        };
        if binding.trim() == udc {
            return Ok(Some(path));
        }
    }
    Ok(None)
}

fn run(argv: &[&str]) -> io::Result<()> {
    log(&format!("running {}", argv.join(" ")));
    let status = std::process::Command::new(argv[0])
        .args(&argv[1..])
        .status()?;
    if !status.success() {
        return Err(io::Error::new(
            io::ErrorKind::Other,
            format!("command failed: {}", argv.join(" ")),
        ));
    }
    Ok(())
}

fn log(message: &str) {
    if crate::verbose_enabled() {
        eprintln!("[deckontroller] {message}");
    }
}

pub fn ensure_libcomposite_loaded() -> io::Result<()> {
    // Ignore failure: it may already be built-in or already loaded
    let _ = run(&["modprobe", "libcomposite"]);
    Ok(())
}
