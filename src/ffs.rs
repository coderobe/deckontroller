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

const FUNCTIONFS_DESCRIPTORS_MAGIC: u32 = 1;
const FUNCTIONFS_STRINGS_MAGIC: u32 = 2;

const USB_DT_INTERFACE: u8 = 0x04;
const USB_DT_ENDPOINT: u8 = 0x05;
const USB_DT_HID: u8 = 0x21;

const USB_CLASS_HID: u8 = 0x03;
const USB_ENDPOINT_XFER_INT: u8 = 0x03;

/// One instance of the physical vendor HID interface:
/// one interrupt IN endpoint and a HID class descriptor pointing at the report descriptor bytes we copied from the real hardware
pub struct HidInterfaceSpec {
    pub report_descriptor: Vec<u8>,
    pub interface_string_index: u8, // 1-based index into the strings block, 0 = none
}

fn interface_descriptor(num_endpoints: u8, iface_string: u8) -> Vec<u8> {
    vec![
        9,                // bLength
        USB_DT_INTERFACE, // bDescriptorType
        0,                // bInterfaceNumber (FunctionFS renumbers as needed)
        0,                // bAlternateSetting
        num_endpoints,    // bNumEndpoints
        USB_CLASS_HID,    // bInterfaceClass
        0x00,             // bInterfaceSubClass (no boot protocol)
        0x00,             // bInterfaceProtocol
        iface_string,     // iInterface
    ]
}

fn hid_descriptor(report_len: u16) -> Vec<u8> {
    let mut v = vec![
        9,          // bLength
        USB_DT_HID, // bDescriptorType
        0x11, 0x01, // bcdHID = 1.11
        0x00, // bCountryCode
        0x01, // bNumDescriptors
        0x22, // bDescriptorType (Report)
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

/// Build one speed's worth of descriptors (interface + HID + interrupt IN).
/// `interval` is in the units that speed expects (ms for full-speed, 125us-log2 units for high-speed), see the call sites below.
fn speed_descriptors(spec: &HidInterfaceSpec, max_packet: u16, interval: u8) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend(interface_descriptor(1, spec.interface_string_index));
    out.extend(hid_descriptor(spec.report_descriptor.len() as u16));
    out.extend(endpoint_descriptor(0x83, max_packet, interval)); // FunctionFS ep1, USB EP3 IN
    out
}

/// Assemble the full-speed-only blob to write to ep0 as the first write().
/// The physical composite exposes full-speed endpoints, and configfs caps this gadget at full speed before the UDC is bound.
pub fn build_descriptors_blob(spec: &HidInterfaceSpec) -> Vec<u8> {
    let fs = speed_descriptors(spec, 64, 1); // full-speed: 1ms poll

    // V1 header: magic(4) + length(4) + fs_count(4) + hs_count(4).
    // The descriptor counts describe entries rather than byte lengths.
    const FS_COUNT: u32 = 3;
    const HS_COUNT: u32 = 0;
    let header_len = 16u32;
    let total_len = header_len + fs.len() as u32;

    let mut blob = Vec::with_capacity(total_len as usize);
    blob.extend_from_slice(&FUNCTIONFS_DESCRIPTORS_MAGIC.to_le_bytes());
    blob.extend_from_slice(&total_len.to_le_bytes());
    blob.extend_from_slice(&FS_COUNT.to_le_bytes());
    blob.extend_from_slice(&HS_COUNT.to_le_bytes());
    blob.extend_from_slice(&fs);
    blob
}

/// Second write() to ep0: the strings block.
/// Register exactly one string (the interface name) in exactly one language (US English).
pub fn build_strings_blob(interface_name: &str) -> Vec<u8> {
    let mut str_bytes = interface_name.as_bytes().to_vec();
    str_bytes.push(0); // NUL terminator

    let lang_id: u16 = 0x0409; // en-US
    let mut body = Vec::new();
    body.extend_from_slice(&lang_id.to_le_bytes());
    body.extend_from_slice(&str_bytes);

    let header_len = 16u32;
    let total_len = header_len + body.len() as u32;

    let mut blob = Vec::with_capacity(total_len as usize);
    blob.extend_from_slice(&FUNCTIONFS_STRINGS_MAGIC.to_le_bytes());
    blob.extend_from_slice(&total_len.to_le_bytes());
    blob.extend_from_slice(&1u32.to_le_bytes()); // str_count
    blob.extend_from_slice(&1u32.to_le_bytes()); // lang_count
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
    pub fn open(mount: &std::path::Path) -> io::Result<Self> {
        // FunctionFS numbers ep files by descriptor order.
        // NOTE: This is the first and only endpoint descriptor, even though the USB address is 0x83
        Ok(EpIn {
            file: OpenOptions::new().write(true).open(mount.join("ep1"))?,
        })
    }
    pub fn send(&mut self, report: &[u8]) -> io::Result<()> {
        self.file.write_all(report)
    }
}
