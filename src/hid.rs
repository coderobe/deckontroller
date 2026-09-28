//! Everything needed to find and talk to the Steam Deck's own internal
//! controller via its `hidraw` device node.
//!
//! We need three things from the kernel:
//!   1. Which `/dev/hidrawN` is the vendor-defined "Steam Controller" interface
//!      (as opposed to the virtual mouse/keyboard interfaces the same
//!      physical device also exposes).
//!   2. Its exact HID report descriptor bytes, verbatim, so our gadget can
//!      present an identical one to the host.
//!   3. A way to shuttle GET_FEATURE / SET_FEATURE requests through to the
//!      real hardware and back.

use std::fs;
use std::fs::File;
use std::io::{self, Write};
use std::os::unix::io::AsRawFd;
use std::path::{Path, PathBuf};

pub const VALVE_VENDOR_ID: u16 = 0x28de;

pub const STEAM_DECK_MOUSE_REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01, 0xa1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x02,
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x02, 0x81, 0x02, 0x75, 0x06, 0x95, 0x01, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x02, 0x81, 0x06,
    0x95, 0x01, 0x09, 0x38, 0x81, 0x06, 0x05, 0x0c, 0x0a, 0x38, 0x02, 0x95, 0x01, 0x81, 0x06, 0xc0,
    0xc0,
];

pub const STEAM_DECK_KEYBOARD_REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x81, 0x01, 0x19, 0x00, 0x29, 0x65, 0x15, 0x00, 0x25, 0x65,
    0x75, 0x08, 0x95, 0x06, 0x81, 0x00, 0xc0,
];

pub const STEAM_DECK_MOUSE_REPORT_LENGTH: usize = 5;
pub const STEAM_DECK_KEYBOARD_REPORT_LENGTH: usize = 8;

/// Linux ioctl direction bits (asm-generic/ioctl.h), used to hand-build the hidraw ioctl request codes so we don't need extra ioctl-codegen crates
const IOC_WRITE: u32 = 1;
const IOC_READ: u32 = 2;

const fn ioc(dir: u32, ty: u8, nr: u8, size: usize) -> libc::c_int {
    ((dir << 30) | ((size as u32 & 0x3fff) << 16) | ((ty as u32) << 8) | nr as u32) as libc::c_int
}

const HID_MAX_DESCRIPTOR_SIZE: usize = 4096;
const HID_MAX_BUFFER_SIZE: usize = 4096;

/// `EVIOCGRAB`: make an evdev client the exclusive recipient of events from an input device
const EVIOCGRAB: libc::c_int = ioc(IOC_WRITE, b'E', 0x90, std::mem::size_of::<i32>());

#[repr(C)]
struct HidrawReportDescriptor {
    size: u32,
    value: [u8; HID_MAX_DESCRIPTOR_SIZE],
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct HidrawDevInfo {
    pub bustype: u32,
    pub vendor: i16,
    pub product: i16,
}

fn hidiocgrdescsize() -> libc::c_int {
    ioc(IOC_READ, b'H', 0x01, std::mem::size_of::<i32>())
}
fn hidiocgrdesc() -> libc::c_int {
    ioc(
        IOC_READ,
        b'H',
        0x02,
        std::mem::size_of::<HidrawReportDescriptor>(),
    )
}
fn hidiocgrawinfo() -> libc::c_int {
    ioc(IOC_READ, b'H', 0x03, std::mem::size_of::<HidrawDevInfo>())
}
fn hidiocsfeature(len: usize) -> libc::c_int {
    ioc(IOC_READ | IOC_WRITE, b'H', 0x06, len)
}
fn hidiocgfeature(len: usize) -> libc::c_int {
    ioc(IOC_READ | IOC_WRITE, b'H', 0x07, len)
}

unsafe fn ioctl_raw(fd: i32, req: libc::c_int, arg: *mut libc::c_void) -> io::Result<i32> {
    let ret = libc::ioctl(fd, req, arg);
    if ret < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(ret)
    }
}

pub struct HidRaw {
    pub path: PathBuf,
    pub file: File,
    pub info: HidrawDevInfo,
    pub report_descriptor: Vec<u8>,
    pub numbered_reports: bool,
}

pub struct SteamDeckAuxiliaryInterfaces {
    pub mouse: HidRaw,
    pub keyboard: HidRaw,
}

/// Exclusive handles for every evdev node created by the selected HID controller interface.
/// Dropping the handles automatically releases the grabs, including on errors and process exit
pub struct EvdevGrabs {
    _files: Vec<File>,
}

impl EvdevGrabs {
    /// Stop the local Deck session from receiving this controller's input.
    ///
    /// We intentionally fail if there are no associated event nodes or if an existing consumer has an exclusive grab:
    /// avoids silently forwarding while still controlling the Deck
    ///
    /// NOTE: Looks like sometimes the Steam client will still receive input events even with an exclusive grab.
    ///       I'm not sure how to prevent that, probably best to have the Steam client closed entirely instead.
    pub fn acquire(controller: &HidRaw) -> io::Result<Self> {
        let mut nodes = controller.event_nodes()?;
        if nodes.is_empty() {
            // The vendor HID interface exposed as hidraw2 has no `input/` children of its own?
            // driver publishes the controller's evdev device separately, but preserves Valve's VID/PID in input/id
            // Fallback to that stable identity
            nodes = event_nodes_by_id(
                controller.info.vendor as u16,
                controller.info.product as u16,
            )?;
        }
        if nodes.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!(
                    "no evdev input nodes found for {}; cannot suppress local input",
                    controller.path.display()
                ),
            ));
        }

        let mut files = Vec::with_capacity(nodes.len());
        for node in nodes {
            let file = fs::OpenOptions::new().read(true).open(&node)?;
            let mut grab: libc::c_int = 1;
            let result =
                unsafe { libc::ioctl(file.as_raw_fd(), EVIOCGRAB, &mut grab as *mut libc::c_int) };
            if result < 0 {
                return Err(io::Error::new(
                    io::Error::last_os_error().kind(),
                    format!("could not exclusively grab {}", node.display()),
                ));
            }
            files.push(file);
        }
        Ok(Self { _files: files })
    }
}

impl HidRaw {
    fn open(path: PathBuf) -> io::Result<Self> {
        let file = fs::OpenOptions::new().read(true).write(true).open(&path)?;
        let fd = file.as_raw_fd();

        let mut info = HidrawDevInfo {
            bustype: 0,
            vendor: 0,
            product: 0,
        };
        unsafe {
            ioctl_raw(
                fd,
                hidiocgrawinfo(),
                &mut info as *mut _ as *mut libc::c_void,
            )?;
        }

        let mut size: i32 = 0;
        unsafe {
            ioctl_raw(
                fd,
                hidiocgrdescsize(),
                &mut size as *mut _ as *mut libc::c_void,
            )?;
        }
        if !(0..=HID_MAX_DESCRIPTOR_SIZE as i32).contains(&size) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("kernel returned invalid HID report descriptor size: {size}"),
            ));
        }
        let mut rdesc = HidrawReportDescriptor {
            size: size as u32,
            value: [0u8; HID_MAX_DESCRIPTOR_SIZE],
        };
        unsafe {
            ioctl_raw(
                fd,
                hidiocgrdesc(),
                &mut rdesc as *mut _ as *mut libc::c_void,
            )?;
        }
        let descriptor_size = usize::try_from(rdesc.size).map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid HID report descriptor size",
            )
        })?;
        if descriptor_size > HID_MAX_DESCRIPTOR_SIZE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("kernel returned oversized HID report descriptor: {descriptor_size} bytes"),
            ));
        }
        let report_descriptor = rdesc.value[..descriptor_size].to_vec();
        let numbered_reports = Self::descriptor_uses_report_ids(&report_descriptor);

        Ok(HidRaw {
            path,
            file,
            info,
            report_descriptor,
            numbered_reports,
        })
    }

    /// Forward a raw feature-report SET request straight to the physical
    /// controller. For an unnumbered physical interface, hidraw requires a
    /// synthetic zero report-number byte before the USB payload.
    pub fn set_feature(&self, data: &[u8]) -> io::Result<()> {
        if data.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "cannot set an empty HID feature report",
            ));
        }
        let mut buf = self.hidraw_write_buffer(data)?;
        unsafe {
            ioctl_raw(
                self.file.as_raw_fd(),
                hidiocsfeature(buf.len()),
                buf.as_mut_ptr() as *mut libc::c_void,
            )?;
        }
        Ok(())
    }

    /// Ask the physical controller for a feature report and hand back the USB report bytes it returns.
    pub fn get_feature(&self, report_id: u8, len: usize) -> io::Result<Vec<u8>> {
        if !self.numbered_reports && report_id != 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "cannot request a nonzero report ID from an unnumbered HID interface",
            ));
        }
        let hidraw_len = self.feature_get_hidraw_len(len)?;
        let mut buf = vec![0u8; hidraw_len];
        buf[0] = report_id;
        let n = unsafe {
            ioctl_raw(
                self.file.as_raw_fd(),
                hidiocgfeature(buf.len()),
                buf.as_mut_ptr() as *mut libc::c_void,
            )?
        };
        let returned = usize::try_from(n).map_err(|_| {
            io::Error::new(io::ErrorKind::InvalidData, "invalid feature report length")
        })?;
        if returned != hidraw_len {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("hidraw returned {returned} bytes for a {hidraw_len}-byte feature report"),
            ));
        }
        let usb_response = if self.numbered_reports {
            buf
        } else {
            // HIDIOCGFEATURE includes a report-number byte in its buffer, so we take that shit out
            buf[1..].to_vec()
        };
        debug_assert_eq!(usb_response.len(), len);
        Ok(usb_response)
    }

    /// Forward a raw USB output report to the physical controller.
    /// NOTE: For an unnumbered physical interface, hidraw requires a synthetic zero report-number byte before the USB payload
    pub fn write_output(&self, data: &[u8]) -> io::Result<()> {
        if data.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "cannot write an empty HID output report",
            ));
        }
        let buf = self.hidraw_write_buffer(data)?;
        let mut file = &self.file;
        file.write_all(&buf)
    }

    pub fn feature_set_hidraw_len(&self, usb_len: usize) -> io::Result<usize> {
        self.hidraw_write_len(usb_len)
    }

    pub fn feature_get_hidraw_len(&self, usb_len: usize) -> io::Result<usize> {
        if usb_len == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "cannot get a zero-length HID feature report",
            ));
        }
        let hidraw_len = if self.numbered_reports {
            usb_len
        } else {
            usb_len.checked_add(1).ok_or_else(|| {
                io::Error::new(io::ErrorKind::InvalidInput, "HID report length overflow")
            })?
        };
        if hidraw_len > HID_MAX_BUFFER_SIZE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "hidraw feature report is too large: {hidraw_len} bytes (maximum {HID_MAX_BUFFER_SIZE})"
                ),
            ));
        }
        Ok(hidraw_len)
    }

    fn hidraw_write_buffer(&self, data: &[u8]) -> io::Result<Vec<u8>> {
        let hidraw_len = self.hidraw_write_len(data.len())?;
        let mut buf = Vec::with_capacity(hidraw_len);
        if !self.numbered_reports {
            buf.push(0);
        }
        buf.extend_from_slice(data);
        Ok(buf)
    }

    fn hidraw_write_len(&self, usb_len: usize) -> io::Result<usize> {
        if usb_len == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "cannot write an empty HID report",
            ));
        }
        let hidraw_len = if self.numbered_reports {
            usb_len
        } else {
            usb_len.checked_add(1).ok_or_else(|| {
                io::Error::new(io::ErrorKind::InvalidInput, "HID report length overflow")
            })?
        };
        if hidraw_len > HID_MAX_BUFFER_SIZE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "hidraw report is too large: {hidraw_len} bytes (maximum {HID_MAX_BUFFER_SIZE})"
                ),
            ));
        }
        Ok(hidraw_len)
    }

    /// Locate event nodes below this hidraw interface's sysfs device.
    /// The kernel publishes them as `.../input/inputN/eventN`.
    /// this avoids accidentally grabbing an unrelated Valve controller
    fn event_nodes(&self) -> io::Result<Vec<PathBuf>> {
        let name = self.path.file_name().ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "hidraw path has no file name")
        })?;
        let sysfs_device = Path::new("/sys/class/hidraw").join(name).join("device");
        let mut names = Vec::new();
        find_event_node_names(&sysfs_device, &mut names)?;
        names.sort();
        names.dedup();
        Ok(names
            .into_iter()
            .map(|name| Path::new("/dev/input").join(name))
            .collect())
    }

    /// Return whether a HID descriptor contains a Report ID global item.
    /// This walks item prefixes so a 0x85 byte inside an item's payload is ignored
    fn descriptor_uses_report_ids(desc: &[u8]) -> bool {
        let mut index = 0;
        while index < desc.len() {
            let prefix = desc[index];
            index += 1;

            if prefix == 0xfe {
                if index + 2 > desc.len() {
                    break;
                }
                let size = desc[index] as usize;
                index += 2; // long-item size and tag
                if let Some(next) = index.checked_add(size) {
                    if next <= desc.len() {
                        index = next;
                        continue;
                    }
                }
                break;
            }

            let size = match prefix & 0x03 {
                0 => 0,
                1 => 1,
                2 => 2,
                3 => 4,
                _ => unreachable!(),
            };
            let item_type = (prefix >> 2) & 0x03;
            let tag = (prefix >> 4) & 0x0f;
            if index + size > desc.len() {
                break;
            }
            if item_type == 1 && tag == 8 {
                return true;
            }
            index += size;
        }
        false
    }
}

fn find_event_node_names(dir: &Path, names: &mut Vec<String>) -> io::Result<()> {
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if is_event_node_name(&name) {
            names.push(name.into_owned());
        } else if entry.file_type()?.is_dir() {
            find_event_node_names(&entry.path(), names)?;
        }
    }
    Ok(())
}

fn is_event_node_name(name: &str) -> bool {
    name.strip_prefix("event").is_some_and(|suffix| {
        !suffix.is_empty() && suffix.bytes().all(|byte| byte.is_ascii_digit())
    })
}

/// Find input devices with the same hardware identity as the selected raw interface.
/// `input/id/vendor` and `input/id/product` are hex like `28de` and `1205`.
fn event_nodes_by_id(vendor: u16, product: u16) -> io::Result<Vec<PathBuf>> {
    let mut nodes = Vec::new();
    for entry in fs::read_dir("/sys/class/input")? {
        let entry = entry?;
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if !is_event_node_name(&name) {
            continue;
        }
        let device = entry.path().join("device");
        let input_id = device.join("id");
        let matches = read_hex_u16(&input_id.join("vendor")) == Some(vendor)
            && read_hex_u16(&input_id.join("product")) == Some(product);
        if matches {
            nodes.push(Path::new("/dev/input").join(name.as_ref()));
        }
    }
    nodes.sort();
    nodes.dedup();
    Ok(nodes)
}

fn read_hex_u16(path: &Path) -> Option<u16> {
    u16::from_str_radix(fs::read_to_string(path).ok()?.trim(), 16).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ioctl_codes_preserve_the_high_direction_bit() {
        assert_eq!(hidiocgrawinfo() as u32, 0x8008_4803);
    }

    #[test]
    fn identifies_only_evdev_node_names() {
        assert!(is_event_node_name("event0"));
        assert!(is_event_node_name("event123"));
        assert!(!is_event_node_name("event"));
        assert!(!is_event_node_name("event-1"));
        assert!(!is_event_node_name("input0"));
    }
}

fn usb_device_parent(hidraw: &HidRaw) -> io::Result<PathBuf> {
    let name = hidraw.path.file_name().ok_or_else(|| {
        io::Error::new(io::ErrorKind::InvalidInput, "hidraw path has no file name")
    })?;
    let mut current = fs::canonicalize(Path::new("/sys/class/hidraw").join(name).join("device"))?;

    loop {
        if current.join("idVendor").is_file() && current.join("idProduct").is_file() {
            return Ok(current);
        }
        if !current.pop() {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!(
                    "could not locate the USB-device parent for {}",
                    hidraw.path.display()
                ),
            ));
        }
    }
}

/// Discover the physical mouse and keyboard HID interfaces attached to the same USB device as the selected vendor controller.
pub fn discover_auxiliary_interfaces(
    controller: &HidRaw,
) -> io::Result<SteamDeckAuxiliaryInterfaces> {
    let controller_parent = usb_device_parent(controller)?;
    let mut mouse = None;
    let mut keyboard = None;

    for entry in fs::read_dir("/sys/class/hidraw")? {
        let entry = entry?;
        let name = entry.file_name();
        let name = name.to_string_lossy();
        let path = PathBuf::from("/dev").join(name.as_ref());
        if path == controller.path || !path.exists() {
            continue;
        }

        let hidraw = match HidRaw::open(path) {
            Ok(hidraw) => hidraw,
            Err(_) => continue,
        };
        if hidraw.info.vendor != controller.info.vendor
            || hidraw.info.product != controller.info.product
        {
            continue;
        }
        let Ok(candidate_parent) = usb_device_parent(&hidraw) else {
            continue;
        };
        if candidate_parent != controller_parent {
            continue;
        }

        if hidraw.report_descriptor == STEAM_DECK_MOUSE_REPORT_DESCRIPTOR {
            if mouse.replace(hidraw).is_some() {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "found more than one matching Steam Deck mouse HID interface",
                ));
            }
        } else if hidraw.report_descriptor == STEAM_DECK_KEYBOARD_REPORT_DESCRIPTOR {
            if keyboard.replace(hidraw).is_some() {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "found more than one matching Steam Deck keyboard HID interface",
                ));
            }
        }
    }

    match (mouse, keyboard) {
        (Some(mouse), Some(keyboard)) => Ok(SteamDeckAuxiliaryInterfaces { mouse, keyboard }),
        (None, Some(_)) => Err(io::Error::new(
            io::ErrorKind::NotFound,
            "could not find the physical Steam Deck mouse HID interface beside the vendor controller",
        )),
        (Some(_), None) => Err(io::Error::new(
            io::ErrorKind::NotFound,
            "could not find the physical Steam Deck keyboard HID interface beside the vendor controller",
        )),
        (None, None) => Err(io::Error::new(
            io::ErrorKind::NotFound,
            "could not find physical Steam Deck mouse and keyboard HID interfaces beside the vendor controller",
        )),
    }
}

/// Scan a raw HID report descriptor for its global "Usage Page" items and report whether any of them is a vendor-defined page (0xFF00-0xFFFF)
/// to tell the real gamepad/trackpad interface apart from the same physical device's virtual keyboard/mouse interfaces
fn declares_vendor_usage_page(desc: &[u8]) -> bool {
    let mut i = 0;
    while i < desc.len() {
        let prefix = desc[i];
        let tag = prefix & 0xfc;
        let size_code = prefix & 0x03;
        let size = match size_code {
            0 => 0,
            1 => 1,
            2 => 2,
            3 => 4,
            _ => unreachable!(),
        };
        if i + 1 + size > desc.len() {
            break;
        }
        // Usage Page (Global item, tag byte 0x04 with type bits = Global(1))
        if tag == 0x04 {
            let value: u32 = match size {
                0 => 0,
                1 => desc[i + 1] as u32,
                2 => u16::from_le_bytes([desc[i + 1], desc[i + 2]]) as u32,
                4 => u32::from_le_bytes([desc[i + 1], desc[i + 2], desc[i + 3], desc[i + 4]]),
                _ => 0,
            };
            if (0xff00..=0xffff).contains(&value) {
                return true;
            }
        }
        i += 1 + size;
    }
    false
}

/// Find the Deck's internal vendor-HID controller interface.
///
/// `want_product` lets the caller pin a specific PID;
/// pass `None` to accept the first Valve-vendor hidraw node whose descriptor declares a vendor usage page
pub fn discover(want_product: Option<u16>) -> io::Result<HidRaw> {
    let mut candidates = Vec::new();
    for entry in fs::read_dir("/sys/class/hidraw")? {
        let entry = entry?;
        let name = entry.file_name();
        let name = name.to_string_lossy();
        let dev_path = PathBuf::from("/dev").join(name.as_ref());
        if !dev_path.exists() {
            continue;
        }
        let hidraw = match HidRaw::open(dev_path.clone()) {
            Ok(h) => h,
            Err(_) => continue, // permission or transient race; skip
        };
        if hidraw.info.vendor as u16 != VALVE_VENDOR_ID {
            continue;
        }
        if let Some(pid) = want_product {
            if hidraw.info.product as u16 != pid {
                continue;
            }
        }
        if declares_vendor_usage_page(&hidraw.report_descriptor) {
            candidates.push(hidraw);
        }
    }

    candidates.into_iter().next().ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::NotFound,
            "no Valve vendor-HID hidraw interface found (is this running on a Steam Deck, \
             with Steam's own use of the controller not blocking hidraw access?)",
        )
    })
}
