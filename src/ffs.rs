//! Minimal FunctionFS bindings
//!
//! We hand-build the raw descriptor blobs ourselves instead of depending on a gadget-framework,
//! because i couldn't get the host to enumerate us as a legitimate HID device at all otherwise.
//!
//! Kernel ABI reference: Documentation/usb/functionfs.rst and include/uapi/linux/usb/functionfs.h
//! NOTE: (the "legacy" V1 layout used below has been stable and supported since FunctionFS was introduced)

use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicBool, Ordering};

const FUNCTIONFS_DESCRIPTORS_MAGIC: u32 = 1;
const FUNCTIONFS_STRINGS_MAGIC: u32 = 2;

const USB_DT_INTERFACE: u8 = 0x04;
const USB_DT_ENDPOINT: u8 = 0x05;
pub const USB_DT_HID: u8 = 0x21;

const USB_CLASS_HID: u8 = 0x03;
const USB_ENDPOINT_XFER_INT: u8 = 0x03;

/// Which physical HID interface an exposed interface mirrors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    Mouse,
    Keyboard,
    /// The vendor "Steam Controller" interface: feature reports are forwarded to the real controller
    Vendor,
}

/// One HID interface of the gadget: interface descriptor, HID class descriptor and one interrupt IN endpoint.
/// All values are copied from the physical interface; `with_defaults` is only the fallback.
#[derive(Debug, Clone)]
pub struct InterfaceSpec {
    pub role: Role,
    pub report_descriptor: Vec<u8>,
    /// iInterface string, None if the physical interface has none
    pub string: Option<String>,
    pub subclass: u8,
    pub protocol: u8,
    pub bcd_hid: u16,
    pub country_code: u8,
    /// Label of the endpoint in the descriptors: must stay sequential (0x81, 0x82, ...) in interface order, because FunctionFS
    /// numbers its endpoint files that way. The address the host actually sees is chosen by the UDC when the function binds.
    pub endpoint_address: u8,
    pub max_packet: u16,
    pub interval: u8,
}

impl InterfaceSpec {
    /// Built-in values, used for anything the physical descriptors didn't tell us.
    /// `position` is the interface's index within the gadget (0-based), used to pick a default endpoint address.
    pub fn with_defaults(role: Role, report_descriptor: Vec<u8>, position: usize) -> Self {
        let (subclass, protocol) = match role {
            Role::Mouse => (1, 2),
            Role::Keyboard => (1, 1),
            Role::Vendor => (0, 0),
        };
        InterfaceSpec {
            role,
            report_descriptor,
            string: None,
            subclass,
            protocol,
            bcd_hid: 0x0111,
            country_code: 0,
            endpoint_address: 0x81 + position as u8,
            max_packet: 64,
            interval: 1,
        }
    }
}

fn interface_descriptor(
    number: u8,
    num_endpoints: u8,
    iface_string: u8,
    subclass: u8,
    protocol: u8,
) -> Vec<u8> {
    vec![
        9,                // bLength
        USB_DT_INTERFACE, // bDescriptorType
        number,           // bInterfaceNumber: the function-local index; FunctionFS maps it to the real composite number
        0,                // bAlternateSetting
        num_endpoints,    // bNumEndpoints
        USB_CLASS_HID,    // bInterfaceClass
        subclass,         // bInterfaceSubClass
        protocol,         // bInterfaceProtocol
        iface_string,     // iInterface
    ]
}

pub fn hid_descriptor(report_len: u16, bcd_hid: u16, country_code: u8) -> Vec<u8> {
    let bcd = bcd_hid.to_le_bytes();
    let mut v = vec![
        9,            // bLength
        USB_DT_HID,   // bDescriptorType
        bcd[0], bcd[1], // bcdHID
        country_code, // bCountryCode
        0x01,         // bNumDescriptors
        0x22,         // bDescriptorType (Report)
    ];
    v.extend_from_slice(&report_len.to_le_bytes());
    v
}

fn endpoint_descriptor(address: u8, max_packet: u16, interval: u8) -> Vec<u8> {
    let mut v = vec![
        7,                     // bLength
        USB_DT_ENDPOINT,       // bDescriptorType
        address,               // bEndpointAddress
        USB_ENDPOINT_XFER_INT, // bmAttributes: Interrupt
    ];
    v.extend_from_slice(&max_packet.to_le_bytes());
    v.push(interval);
    v
}

/// 1-based FunctionFS string index of each interface (0 = no string), in interface order
pub fn string_indices(interfaces: &[InterfaceSpec]) -> Vec<u8> {
    let mut next = 0u8;
    interfaces
        .iter()
        .map(|i| {
            if i.string.is_some() {
                next += 1;
                next
            } else {
                0
            }
        })
        .collect()
}

/// Assemble the full-speed-only blob to write to ep0 as the first write().
/// Interfaces appear in the order given, and so do their endpoints: FunctionFS exposes them as ep1, ep2, ...
/// The physical composite exposes full-speed endpoints, and configfs caps this gadget at full speed before the UDC is bound.
pub fn build_descriptors_blob(interfaces: &[InterfaceSpec]) -> Vec<u8> {
    let indices = string_indices(interfaces);
    let mut fs = Vec::new();
    for (number, (spec, string_index)) in interfaces.iter().zip(indices).enumerate() {
        fs.extend(interface_descriptor(
            number as u8,
            1,
            string_index,
            spec.subclass,
            spec.protocol,
        ));
        fs.extend(hid_descriptor(
            spec.report_descriptor.len() as u16,
            spec.bcd_hid,
            spec.country_code,
        ));
        fs.extend(endpoint_descriptor(
            spec.endpoint_address,
            spec.max_packet,
            spec.interval,
        ));
    }

    // V1 header: magic(4) + length(4) + fs_count(4) + hs_count(4).
    // The descriptor counts describe entries (interface + HID + endpoint per interface) rather than byte lengths.
    let fs_count = 3 * interfaces.len() as u32;
    const HS_COUNT: u32 = 0;
    let header_len = 16u32;
    let total_len = header_len + fs.len() as u32;

    let mut blob = Vec::with_capacity(total_len as usize);
    blob.extend_from_slice(&FUNCTIONFS_DESCRIPTORS_MAGIC.to_le_bytes());
    blob.extend_from_slice(&total_len.to_le_bytes());
    blob.extend_from_slice(&fs_count.to_le_bytes());
    blob.extend_from_slice(&HS_COUNT.to_le_bytes());
    blob.extend_from_slice(&fs);
    blob
}

/// Second write() to ep0: the strings block, one string per interface that has an iInterface, in US English.
/// With no strings at all, a block with zero strings and zero languages is valid.
pub fn build_strings_blob(interfaces: &[InterfaceSpec]) -> Vec<u8> {
    let strings: Vec<&str> = interfaces.iter().filter_map(|i| i.string.as_deref()).collect();

    let mut body = Vec::new();
    if !strings.is_empty() {
        let lang_id: u16 = 0x0409; // en-US
        body.extend_from_slice(&lang_id.to_le_bytes());
        for string in &strings {
            body.extend_from_slice(string.as_bytes());
            body.push(0); // NUL terminator
        }
    }
    let lang_count = u32::from(!strings.is_empty());

    let header_len = 16u32;
    let total_len = header_len + body.len() as u32;

    let mut blob = Vec::with_capacity(total_len as usize);
    blob.extend_from_slice(&FUNCTIONFS_STRINGS_MAGIC.to_le_bytes());
    blob.extend_from_slice(&total_len.to_le_bytes());
    blob.extend_from_slice(&(strings.len() as u32).to_le_bytes()); // str_count
    blob.extend_from_slice(&lang_count.to_le_bytes());
    blob.extend_from_slice(&body);
    blob
}

// --- ep0 control-transfer event loop -------------------------------------

#[repr(C)]
#[derive(Clone, Copy)]
struct UsbCtrlRequest {
    b_request_type: u8,
    b_request: u8,
    w_value: u16,
    w_index: u16,
    w_length: u16,
}

const FUNCTIONFS_BIND: u8 = 0;
const FUNCTIONFS_UNBIND: u8 = 1;
const FUNCTIONFS_ENABLE: u8 = 2;
const FUNCTIONFS_DISABLE: u8 = 3;
const FUNCTIONFS_SETUP: u8 = 4;
const FUNCTIONFS_SUSPEND: u8 = 5;
const FUNCTIONFS_RESUME: u8 = 6;

pub enum Ep0Event {
    Bind,
    Unbind,
    Enable,
    Disable,
    Suspend,
    Resume,
    Setup(SetupRequest),
}

#[derive(Debug, Clone, Copy)]
pub struct SetupRequest {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

impl SetupRequest {
    pub fn is_device_to_host(&self) -> bool {
        self.request_type & 0x80 != 0
    }
    pub fn is_class(&self) -> bool {
        (self.request_type & 0x60) == 0x20
    }
}

// HID class requests (USB HID spec table 7.1-ish)
pub const HID_GET_REPORT: u8 = 0x01;
pub const HID_GET_IDLE: u8 = 0x02;
pub const HID_GET_PROTOCOL: u8 = 0x03;
pub const HID_SET_REPORT: u8 = 0x09;
pub const HID_SET_IDLE: u8 = 0x0a;
pub const HID_SET_PROTOCOL: u8 = 0x0b;
pub const USB_GET_DESCRIPTOR: u8 = 0x06;
pub const USB_DT_REPORT: u8 = 0x22;

pub struct Ep0 {
    file: File,
}

impl Ep0 {
    pub fn open(mount: &std::path::Path) -> io::Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(mount.join("ep0"))?;
        Ok(Ep0 { file })
    }

    pub fn write_descriptors(&mut self, descriptors: &[u8], strings: &[u8]) -> io::Result<()> {
        self.file.write_all(descriptors)?;
        self.file.write_all(strings)?;
        Ok(())
    }

    pub fn next_event(&mut self) -> io::Result<Ep0Event> {
        // struct usb_functionfs_event: 8-byte union + 1 byte type + 3 pad = 12 bytes
        let mut buf = [0u8; 12];
        self.file.read_exact(&mut buf)?;
        let event_type = buf[8];
        match event_type {
            FUNCTIONFS_BIND => Ok(Ep0Event::Bind),
            FUNCTIONFS_UNBIND => Ok(Ep0Event::Unbind),
            FUNCTIONFS_ENABLE => Ok(Ep0Event::Enable),
            FUNCTIONFS_DISABLE => Ok(Ep0Event::Disable),
            FUNCTIONFS_SUSPEND => Ok(Ep0Event::Suspend),
            FUNCTIONFS_RESUME => Ok(Ep0Event::Resume),
            FUNCTIONFS_SETUP => {
                let req = SetupRequest {
                    request_type: buf[0],
                    request: buf[1],
                    value: u16::from_le_bytes([buf[2], buf[3]]),
                    index: u16::from_le_bytes([buf[4], buf[5]]),
                    length: u16::from_le_bytes([buf[6], buf[7]]),
                };
                Ok(Ep0Event::Setup(req))
            }
            other => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("unknown FunctionFS event type {other}"),
            )),
        }
    }

    /// Answer a device-to-host control request (e.g. GET_REPORT) with data.
    pub fn respond_in(&mut self, data: &[u8]) -> io::Result<()> {
        if data.is_empty() {
            let result = self.zero_length_read_or_write(true);
            if result.is_err() {
                log_result("ep0 zero-length response write failed", &result);
            }
            result
        } else {
            // A FunctionFS EP0 write is the control transfer's IN data stage.
            // NOTE: Keep it as one write so a short write cannot start a second, unrelated EP0 operation.
            match self.file.write(data) {
                Ok(written) if written == data.len() => Ok(()),
                Ok(written) => {
                    let error = io::Error::new(
                        io::ErrorKind::WriteZero,
                        format!(
                            "short FunctionFS EP0 response: wrote {written} of {} bytes",
                            data.len()
                        ),
                    );
                    log(&format!(
                        "ep0 response write failed: short write {written}/{} bytes errno={:?}: {error}",
                        data.len(),
                        error.raw_os_error()
                    ));
                    Err(error)
                }
                Err(error) => {
                    log(&format!(
                        "ep0 response write failed errno={:?}: {error}",
                        error.raw_os_error()
                    ));
                    Err(error)
                }
            }
        }
    }

    /// Complete a host-to-device control transfer with its required empty status stage.
    pub fn acknowledge(&mut self) -> io::Result<()> {
        self.zero_length_read_or_write(false)
    }

    /// Consume the data stage of a host-to-device control request (e.g. SET_REPORT) and return the payload.
    pub fn read_out(&mut self, length: usize) -> io::Result<Vec<u8>> {
        if length == 0 {
            self.zero_length_read_or_write(false)?;
            return Ok(Vec::new());
        }
        let mut buf = vec![0u8; length];
        self.file.read_exact(&mut buf)?;
        Ok(buf)
    }

    /// Stall the control endpoint to signal "not handled" for requests we don't proxy
    /// NOTE: some standard requests are handled by the kernel automatically; this is only for stuff neither cares about.
    pub fn stall(&mut self, device_to_host: bool) -> io::Result<()> {
        // FunctionFS stalls by attempting the EP0 operation opposite the setup packet's data direction: read for IN requests, write for OUT.
        match self.zero_length_read_or_write(!device_to_host) {
            Ok(()) => Ok(()),
            Err(e) if e.raw_os_error() == Some(libc::EL2HLT) => Ok(()),
            Err(e) => Err(e),
        }
    }

    fn zero_length_read_or_write(&mut self, write: bool) -> io::Result<()> {
        let mut dummy = 0u8;
        let result = unsafe {
            if write {
                libc::write(
                    self.file.as_raw_fd(),
                    &dummy as *const u8 as *const libc::c_void,
                    0,
                )
            } else {
                libc::read(
                    self.file.as_raw_fd(),
                    &mut dummy as *mut u8 as *mut libc::c_void,
                    0,
                )
            }
        };

        match result {
            0 => Ok(()),
            n if n < 0 => Err(io::Error::last_os_error()),
            n => Err(io::Error::new(
                io::ErrorKind::Other,
                format!("unexpected {n}-byte zero-length FunctionFS EP0 operation"),
            )),
        }
    }
}

fn log_result(context: &str, result: &io::Result<()>) {
    match result {
        Ok(()) => log(&format!("{context} result=ok")),
        Err(error) => log(&format!(
            "{context} result=error errno={:?}: {error}",
            error.raw_os_error()
        )),
    }
}

fn log(message: &str) {
    eprintln!("[deckontroller] {message}");
}

pub struct EpIn {
    file: File,
}
impl EpIn {
    /// `number` is the 1-based FunctionFS endpoint file: endpoints are numbered in descriptor order
    pub fn open(mount: &std::path::Path, number: usize) -> io::Result<Self> {
        Ok(EpIn {
            file: OpenOptions::new()
                .write(true)
                .open(mount.join(format!("ep{number}")))?,
        })
    }

    #[cfg(test)]
    pub fn from_file(file: File) -> Self {
        EpIn { file }
    }

    /// Send one report. The write blocks until the host has polled it, which is the flow control for the whole relay.
    ///
    /// While the endpoint is disabled (host reconfiguring, cable pulled) FunctionFS makes the write wait until it is enabled again.
    /// Such a wait ends with EINTR when the thread is signalled; `running` going false then turns that into an error
    /// instead of retrying, so a stopping forwarder can never get stuck here.
    pub fn send(&mut self, report: &[u8], running: &AtomicBool) -> io::Result<()> {
        loop {
            let written = unsafe {
                libc::write(
                    self.file.as_raw_fd(),
                    report.as_ptr() as *const libc::c_void,
                    report.len(),
                )
            };
            if written >= 0 {
                return if written as usize == report.len() {
                    Ok(())
                } else {
                    Err(io::Error::new(
                        io::ErrorKind::WriteZero,
                        format!("short endpoint write: {written} of {} bytes", report.len()),
                    ))
                };
            }
            let error = io::Error::last_os_error();
            if error.kind() == io::ErrorKind::Interrupted && running.load(Ordering::SeqCst) {
                continue;
            }
            return Err(error);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(role: Role, position: usize) -> InterfaceSpec {
        InterfaceSpec::with_defaults(role, vec![0u8; 10], position)
    }

    #[test]
    fn descriptor_blob_carries_mirrored_values() {
        let mut s = spec(Role::Vendor, 2);
        s.subclass = 1;
        s.protocol = 2;
        s.bcd_hid = 0x0110;
        s.country_code = 0x21;
        s.endpoint_address = 0x83;
        s.max_packet = 32;
        s.interval = 4;
        s.string = Some("Vendor".into());

        let blob = build_descriptors_blob(&[s]);
        let fs = &blob[16..]; // skip the V1 header
        assert_eq!(fs[8], 1); // iInterface -> first string
        assert_eq!(&fs[5..8], &[3, 1, 2]); // class HID, subclass, protocol
        assert_eq!(&fs[11..14], &[0x10, 0x01, 0x21]); // bcdHID 1.10, country code
        assert_eq!(&fs[18..25], &[7, 5, 0x83, 3, 32, 0, 4]); // endpoint
    }

    #[test]
    fn interfaces_are_numbered_by_position() {
        let blob = build_descriptors_blob(&[
            spec(Role::Mouse, 0),
            spec(Role::Keyboard, 1),
            spec(Role::Vendor, 2),
        ]);
        let fs = &blob[16..];
        assert_eq!([fs[2], fs[25 + 2], fs[50 + 2]], [0, 1, 2]); // bInterfaceNumber
    }

    #[test]
    fn several_interfaces_keep_order_and_count() {
        let blob = build_descriptors_blob(&[
            spec(Role::Mouse, 0),
            spec(Role::Keyboard, 1),
            spec(Role::Vendor, 2),
        ]);
        assert_eq!(u32::from_le_bytes(blob[8..12].try_into().unwrap()), 9); // fs_count
        let fs = &blob[16..];
        // endpoint addresses appear in interface order: 25 bytes per interface
        assert_eq!([fs[20], fs[25 + 20], fs[50 + 20]], [0x81, 0x82, 0x83]);
        // mouse is boot mouse, keyboard boot keyboard by default
        assert_eq!(&fs[6..8], &[1, 2]);
        assert_eq!(&fs[25 + 6..25 + 8], &[1, 1]);
    }

    #[test]
    fn strings_only_for_interfaces_that_have_one() {
        let mut a = spec(Role::Mouse, 0);
        let b = spec(Role::Keyboard, 1);
        let mut c = spec(Role::Vendor, 2);
        a.string = Some("A".into());
        c.string = Some("CC".into());
        let all = [a, b, c];
        assert_eq!(string_indices(&all), vec![1, 0, 2]);
        let blob = build_strings_blob(&all);
        assert_eq!(u32::from_le_bytes(blob[8..12].try_into().unwrap()), 2); // str_count
        assert_eq!(u32::from_le_bytes(blob[12..16].try_into().unwrap()), 1); // lang_count
        assert_eq!(&blob[16..], &[0x09, 0x04, b'A', 0, b'C', b'C', 0]);
    }

    #[test]
    fn no_strings_gives_empty_block() {
        let blob = build_strings_blob(&[spec(Role::Vendor, 0)]);
        assert_eq!(blob.len(), 16);
        assert_eq!(u32::from_le_bytes(blob[8..12].try_into().unwrap()), 0);
        assert_eq!(u32::from_le_bytes(blob[12..16].try_into().unwrap()), 0);
    }
}
