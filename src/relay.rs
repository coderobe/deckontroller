//! device relay logic:
//! every byte that comes off the real controller goes to the host unchanged;
//! every control transfer the host sends goes to the real controller unchanged.
//! straight pass-through lets Steam on the host handle everything rather than exposing a limited xinput hid device ourselves

use std::io::{Read, Write};
use std::os::unix::io::{AsRawFd, FromRawFd};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, OnceLock};
use std::thread;
use std::time::Instant;

use crate::ffs::{
    Ep0, Ep0Event, EpIn, SetupRequest, HID_GET_IDLE, HID_GET_PROTOCOL, HID_GET_REPORT,
    HID_SET_IDLE, HID_SET_PROTOCOL, HID_SET_REPORT, USB_DT_REPORT, USB_GET_DESCRIPTOR,
};
use crate::hid::{
    HidRaw, SteamDeckAuxiliaryInterfaces, STEAM_DECK_KEYBOARD_REPORT_LENGTH,
    STEAM_DECK_MOUSE_REPORT_LENGTH,
};

const MAX_REPORT: usize = 4096;
const HID_REPORT_TYPE_INPUT: u8 = 1;
const HID_REPORT_TYPE_OUTPUT: u8 = 2;
const HID_REPORT_TYPE_FEATURE: u8 = 3;

static TRACE_START: OnceLock<Instant> = OnceLock::new();

// enum names ripped from the linux hid-steam driver and SDL, this is just for human readable verbose logging
const VALVE_SETTING_NAMES: &[&str] = &[
    "MOUSE_SENSITIVITY",
    "MOUSE_ACCELERATION",
    "TRACKBALL_ROTATION_ANGLE",
    "HAPTIC_INTENSITY_UNUSED",
    "LEFT_GAMEPAD_STICK_ENABLED",
    "RIGHT_GAMEPAD_STICK_ENABLED",
    "USB_DEBUG_MODE",
    "LEFT_TRACKPAD_MODE",
    "RIGHT_TRACKPAD_MODE",
    "LIZARD_MODE",
    "DPAD_DEADZONE",
    "MINIMUM_MOMENTUM_VELOCITY",
    "MOMENTUM_DECAY_AMMOUNT",
    "TRACKPAD_RELATIVE_MODE_TICKS_PER_PIXEL",
    "HAPTIC_INCREMENT",
    "DPAD_ANGLE_SIN",
    "DPAD_ANGLE_COS",
    "MOMENTUM_VERTICAL_DIVISOR",
    "MOMENTUM_MAXIMUM_VELOCITY",
    "TRACKPAD_Z_ON",
    "TRACKPAD_Z_OFF",
    "SENSITIVY_SCALE_AMMOUNT",
    "LEFT_TRACKPAD_SECONDARY_MODE",
    "RIGHT_TRACKPAD_SECONDARY_MODE",
    "SMOOTH_ABSOLUTE_MOUSE",
    "STEAM_BUTTON_POWEROFF_TIME",
    "UNUSED_1",
    "TRACKPAD_OUTER_RADIUS",
    "TRACKPAD_Z_ON_LEFT",
    "TRACKPAD_Z_OFF_LEFT",
    "TRACKPAD_OUTER_SPIN_VEL",
    "TRACKPAD_OUTER_SPIN_RADIUS",
    "TRACKPAD_OUTER_SPIN_HORIZONTAL_ONLY",
    "TRACKPAD_RELATIVE_MODE_DEADZONE",
    "TRACKPAD_RELATIVE_MODE_MAX_VELOCITY",
    "TRACKPAD_RELATIVE_MODE_INVERT_Y",
    "TRACKPAD_DOUBLE_TAP_BEEP_ENABLED",
    "TRACKPAD_DOUBLE_TAP_BEEP_PERIOD",
    "TRACKPAD_DOUBLE_TAP_BEEP_COUNT",
    "TRACKPAD_OUTER_RADIUS_RELEASE_ON_TRANSITION",
    "RADIAL_MODE_ANGLE",
    "HAPTIC_INTENSITY_MOUSE_MODE",
    "LEFT_DPAD_REQUIRES_CLICK",
    "RIGHT_DPAD_REQUIRES_CLICK",
    "LED_BASELINE_BRIGHTNESS",
    "LED_USER_BRIGHTNESS",
    "ENABLE_RAW_JOYSTICK",
    "ENABLE_FAST_SCAN",
    "IMU_MODE",
    "WIRELESS_PACKET_VERSION",
    "SLEEP_INACTIVITY_TIMEOUT",
    "TRACKPAD_NOISE_THRESHOLD",
    "LEFT_TRACKPAD_CLICK_PRESSURE",
    "RIGHT_TRACKPAD_CLICK_PRESSURE",
    "LEFT_BUMPER_CLICK_PRESSURE",
    "RIGHT_BUMPER_CLICK_PRESSURE",
    "LEFT_GRIP_CLICK_PRESSURE",
    "RIGHT_GRIP_CLICK_PRESSURE",
    "LEFT_GRIP2_CLICK_PRESSURE",
    "RIGHT_GRIP2_CLICK_PRESSURE",
    "PRESSURE_MODE",
    "CONTROLLER_TEST_MODE",
    "TRIGGER_MODE",
    "TRACKPAD_Z_THRESHOLD",
    "FRAME_RATE",
    "TRACKPAD_FILT_CTRL",
    "TRACKPAD_CLIP",
    "DEBUG_OUTPUT_SELECT",
    "TRIGGER_THRESHOLD_PERCENT",
    "TRACKPAD_FREQUENCY_HOPPING",
    "HAPTICS_ENABLED",
    "STEAM_WATCHDOG_ENABLE",
    "TIMP_TOUCH_THRESHOLD_ON",
    "TIMP_TOUCH_THRESHOLD_OFF",
    "FREQ_HOPPING",
    "TEST_CONTROL",
    "HAPTIC_MASTER_GAIN_DB",
    "THUMB_TOUCH_THRESH",
    "DEVICE_POWER_STATUS",
    "HAPTIC_INTENSITY",
    "STABILIZER_ENABLED",
    "TIMP_MODE_MTE",
];

const VALVE_ATTRIBUTE_NAMES: &[&str] = &[
    "UNIQUE_ID",
    "PRODUCT_ID",
    "PRODUCT_REVISION_OR_CAPABILITIES",
    "FIRMWARE_VERSION",
    "FIRMWARE_BUILD_TIME",
    "RADIO_FIRMWARE_BUILD_TIME",
    "RADIO_DEVICE_ID0",
    "RADIO_DEVICE_ID1",
    "DONGLE_FIRMWARE_BUILD_TIME",
    "HW_ID_OR_BOARD_REVISION",
    "BOOTLOADER_BUILD_TIME",
    "CONNECTION_INTERVAL_IN_US",
    "SECONDARY_FIRMWARE_BUILD_TIME",
    "SECONDARY_BOOTLOADER_BUILD_TIME",
    "SECONDARY_HW_ID_OR_BOARD_REVISION",
    "STREAMING",
    "TRACKPAD_ID",
    "SECONDARY_TRACKPAD_ID",
];

fn valve_feature_command_name(command: u8) -> &'static str {
    match command {
        0x80 => "SET_DIGITAL_MAPPINGS",
        0x81 => "CLEAR_DIGITAL_MAPPINGS",
        0x82 => "GET_DIGITAL_MAPPINGS",
        0x83 => "GET_ATTRIBUTES_VALUES",
        0x84 => "GET_ATTRIBUTE_LABEL",
        0x85 => "SET_DEFAULT_DIGITAL_MAPPINGS",
        0x86 => "FACTORY_RESET",
        0x87 => "SET_SETTINGS_VALUES",
        0x88 => "CLEAR_SETTINGS_VALUES",
        0x89 => "GET_SETTINGS_VALUES",
        0x8a => "GET_SETTING_LABEL",
        0x8b => "GET_SETTINGS_MAXS",
        0x8c => "GET_SETTINGS_DEFAULTS",
        0x8d => "SET_CONTROLLER_MODE",
        0x8e => "LOAD_DEFAULT_SETTINGS",
        0x8f => "TRIGGER_HAPTIC_PULSE",
        0x9f => "TURN_OFF_CONTROLLER",
        0xa1 => "GET_DEVICE_INFO",
        0xa7 => "CALIBRATE_TRACKPADS",
        0xa8 => "RESERVED_0",
        0xa9 => "SET_SERIAL_NUMBER",
        0xaa => "GET_TRACKPAD_CALIBRATION",
        0xab => "GET_TRACKPAD_FACTORY_CALIBRATION",
        0xac => "GET_TRACKPAD_RAW_DATA",
        0xad => "ENABLE_PAIRING",
        0xae => "GET_STRING_ATTRIBUTE",
        0xaf => "RADIO_ERASE_RECORDS",
        0xb0 => "RADIO_WRITE_RECORD",
        0xb1 => "SET_DONGLE_SETTING",
        0xb2 => "DONGLE_DISCONNECT_DEVICE",
        0xb3 => "DONGLE_COMMIT_DEVICE",
        0xb4 => "DONGLE_GET_WIRELESS_STATE",
        0xb5 => "CALIBRATE_GYRO",
        0xb6 => "PLAY_AUDIO",
        0xb7 => "AUDIO_UPDATE_START",
        0xb8 => "AUDIO_UPDATE_DATA",
        0xb9 => "AUDIO_UPDATE_COMPLETE",
        0xba => "GET_CHIPID",
        0xbf => "CALIBRATE_JOYSTICK",
        0xc0 => "CALIBRATE_ANALOG_TRIGGERS",
        0xc1 => "SET_AUDIO_MAPPING",
        0xc2 => "CHECK_GYRO_FW_LOAD",
        0xc3 => "CALIBRATE_ANALOG",
        0xc4 => "DONGLE_GET_CONNECTED_SLOTS",
        0xce => "RESET_IMU",
        0xea => "TRIGGER_HAPTIC_CMD",
        0xeb => "TRIGGER_RUMBLE_CMD",
        _ => "UNKNOWN",
    }
}

fn valve_setting_name(setting: u8) -> &'static str {
    VALVE_SETTING_NAMES
        .get(setting as usize)
        .copied()
        .unwrap_or("UNKNOWN_SETTING")
}

fn valve_trackpad_mode_name(value: u16) -> Option<&'static str> {
    match value {
        0 => Some("ABSOLUTE_MOUSE"),
        1 => Some("RELATIVE_MOUSE"),
        2 => Some("DPAD_FOUR_WAY_DISCRETE"),
        3 => Some("DPAD_FOUR_WAY_OVERLAP"),
        4 => Some("DPAD_EIGHT_WAY"),
        5 => Some("RADIAL_MODE"),
        6 => Some("ABSOLUTE_DPAD"),
        7 => Some("NONE"),
        8 => Some("GESTURE_KEYBOARD"),
        _ => None,
    }
}

fn valve_imu_mode_name(value: u16) -> Option<String> {
    if value == 0 {
        return Some("OFF".to_owned());
    }
    if value & !0x1f != 0 {
        return None;
    }
    let mut flags = Vec::new();
    for (bit, name) in [
        (0x01, "STEERING"),
        (0x02, "TILT"),
        (0x04, "SEND_ORIENTATION"),
        (0x08, "SEND_RAW_ACCEL"),
        (0x10, "SEND_RAW_GYRO"),
    ] {
        if value & bit != 0 {
            flags.push(name);
        }
    }
    Some(flags.join("|"))
}

fn valve_setting_value_name(setting: u8, value: u16) -> Option<String> {
    match setting {
        7 | 8 | 22 | 23 => valve_trackpad_mode_name(value).map(str::to_owned),
        9 | 71 => match value {
            0 => Some("DISABLED".to_owned()),
            1 => Some("ENABLED".to_owned()),
            _ => None,
        },
        48 => valve_imu_mode_name(value),
        _ => None,
    }
}

fn valve_attribute_name(attribute: u8) -> &'static str {
    VALVE_ATTRIBUTE_NAMES
        .get(attribute as usize)
        .copied()
        .unwrap_or("UNKNOWN_ATTRIBUTE")
}

fn describe_attribute_response(data: &[u8]) -> String {
    if data.len() < 2 {
        return "malformed attribute response".to_owned();
    }
    let declared_bytes = data[1] as usize;
    let available = data.len().saturating_sub(2);
    let records_len = declared_bytes.min(available);
    let mut attributes = Vec::new();
    for record in data[2..2 + records_len].chunks_exact(5) {
        let attribute = record[0];
        let value = u32::from_le_bytes([record[1], record[2], record[3], record[4]]);
        attributes.push(format!(
            "{}[0x{attribute:02x}]=0x{value:08x}({value})",
            valve_attribute_name(attribute)
        ));
    }
    let suffix = if declared_bytes > available {
        format!(
            "; malformed payload: declares {declared_bytes} attribute bytes, only {available} available"
        )
    } else if declared_bytes % 5 != 0 {
        format!("; malformed attribute byte count {declared_bytes}")
    } else {
        String::new()
    };
    format!("attributes: {}{suffix}", attributes.join(", "))
}

fn describe_string_attribute_response(data: &[u8]) -> String {
    if data.len() < 3 {
        return "malformed string-attribute response".to_owned();
    }
    let declared_len = data[1] as usize;
    let available = data.len().saturating_sub(3);
    let string_len = declared_len.min(available);
    let value = String::from_utf8_lossy(&data[3..3 + string_len])
        .trim_end_matches('\0')
        .to_owned();
    let attribute = match data[2] {
        0 => "BOARD_SERIAL",
        1 => "UNIT_SERIAL",
        _ => "UNKNOWN_STRING_ATTRIBUTE",
    };
    format!("string attribute {attribute}[{}]={value:?}", data[2])
}

fn describe_feature_payload(data: &[u8], is_response: bool) -> String {
    let Some(&command) = data.first() else {
        return "empty payload".to_owned();
    };
    match command {
        0x81 => "clears controller digital mappings".to_owned(),
        0x83 if is_response => describe_attribute_response(data),
        0x83 => "requests controller attribute values".to_owned(),
        0xae if is_response => describe_string_attribute_response(data),
        0xae if data.len() >= 3 => {
            let attribute = match data[2] {
                0 => "BOARD_SERIAL",
                1 => "UNIT_SERIAL",
                _ => "UNKNOWN_STRING_ATTRIBUTE",
            };
            format!("requests string attribute {attribute}[{}]", data[2])
        }
        0x87 if data.len() >= 2 => {
            let declared_bytes = data[1] as usize;
            let available = data.len().saturating_sub(2);
            let settings_len = declared_bytes.min(available);
            let mut settings = Vec::new();
            for chunk in data[2..2 + settings_len].chunks_exact(3) {
                let setting = chunk[0];
                let value = u16::from_le_bytes([chunk[1], chunk[2]]);
                let value_name = valve_setting_value_name(setting, value)
                    .map(|name| format!("{name}({value})"))
                    .unwrap_or_else(|| value.to_string());
                settings.push(format!(
                    "{}[0x{setting:02x}]={value_name}",
                    valve_setting_name(setting)
                ));
            }
            let suffix = if declared_bytes > available {
                format!(
                    "; malformed payload: declares {declared_bytes} setting bytes, only {available} available"
                )
            } else if declared_bytes % 3 != 0 {
                format!("; malformed setting byte count {declared_bytes}")
            } else {
                String::new()
            };
            format!("settings: {}{suffix}", settings.join(", "))
        }
        _ => "opaque Valve command payload".to_owned(),
    }
}

fn hid_request_name(request: u8) -> &'static str {
    match request {
        HID_GET_REPORT => "GET_REPORT",
        HID_GET_IDLE => "GET_IDLE",
        HID_GET_PROTOCOL => "GET_PROTOCOL",
        HID_SET_REPORT => "SET_REPORT",
        HID_SET_IDLE => "SET_IDLE",
        HID_SET_PROTOCOL => "SET_PROTOCOL",
        _ => "UNKNOWN_HID_REQUEST",
    }
}

fn hid_report_type_name(report_type: u8) -> &'static str {
    match report_type {
        HID_REPORT_TYPE_INPUT => "INPUT",
        HID_REPORT_TYPE_OUTPUT => "OUTPUT",
        HID_REPORT_TYPE_FEATURE => "FEATURE",
        _ => "UNKNOWN_REPORT_TYPE",
    }
}

fn valve_input_report_name(report_type: u8) -> &'static str {
    match report_type {
        1 => "CONTROLLER_STATE",
        2 => "CONTROLLER_DEBUG",
        3 => "CONTROLLER_WIRELESS",
        4 => "CONTROLLER_STATUS",
        5 => "CONTROLLER_DEBUG2",
        6 => "CONTROLLER_SECONDARY_STATE",
        7 => "CONTROLLER_BLE_STATE",
        9 => "CONTROLLER_DECK_STATE",
        _ => "UNKNOWN_INPUT_REPORT",
    }
}

fn describe_input_report(data: &[u8]) -> String {
    if data.len() < 4 {
        return "no Valve report header".to_owned();
    }
    format!(
        "Valve header version={}.{} type=0x{:02x}({}) declared_len={}",
        data[0],
        data[1],
        data[2],
        valve_input_report_name(data[2]),
        data[3]
    )
}

fn describe_setup_request(req: SetupRequest) -> String {
    let direction = if req.is_device_to_host() {
        "device-to-host"
    } else {
        "host-to-device"
    };
    let request_kind = match req.request_type & 0x60 {
        0x00 => "standard",
        0x20 => "HID class",
        0x40 => "vendor",
        _ => "reserved",
    };
    let recipient = match req.request_type & 0x1f {
        0 => "device",
        1 => "interface",
        2 => "endpoint",
        3 => "other",
        _ => "reserved",
    };
    if req.is_class() {
        let report_type = (req.value >> 8) as u8;
        let report_id = req.value as u8;
        format!(
            "{request_kind} {direction} {} {} report_type={}({}) report_id={report_id} interface={} length={}",
            hid_request_name(req.request),
            recipient,
            report_type,
            hid_report_type_name(report_type),
            req.index,
            req.length
        )
    } else {
        format!(
            "{request_kind} {direction} request=0x{:02x} recipient={recipient} value=0x{:04x} index={} length={}",
            req.request, req.value, req.index, req.length
        )
    }
}

struct FeatureTrace {
    next_sequence: u64,
    active_sequence: Option<u64>,
    last_set: Option<Vec<u8>>,
}

impl FeatureTrace {
    fn sequence(&mut self) -> u64 {
        if let Some(sequence) = self.active_sequence {
            return sequence;
        }
        self.next_sequence += 1;
        self.active_sequence = Some(self.next_sequence);
        self.next_sequence
    }

    fn finish_sequence(&mut self) {
        self.active_sequence = None;
    }
}

pub fn run(
    controller: HidRaw,
    auxiliary: SteamDeckAuxiliaryInterfaces,
    mount: &std::path::Path,
    mouse_hidg: &std::path::Path,
    keyboard_hidg: &std::path::Path,
    ep0: &mut Ep0,
) -> std::io::Result<()> {
    let running = Arc::new(AtomicBool::new(false));
    let event_counter = Arc::new(AtomicU64::new(0));
    let mut input_forwarders = Vec::new();
    // This must stay alive for precisely the period in which we are sending reports to the USB host.
    // hidraw itself is non-exclusive; these evdev grabs try to prevent the same physical buttons reaching KDE/Steam locally.
    let mut _local_input_grab = None;
    let mut feature_trace = FeatureTrace {
        next_sequence: 0,
        active_sequence: None,
        last_set: None,
    };
    let controller = Arc::new(controller);
    let auxiliary = Arc::new(auxiliary);

    loop {
        match ep0.next_event()? {
            Ep0Event::Bind => trace(&event_counter, "FunctionFS Bind"),
            Ep0Event::Unbind => trace(&event_counter, "FunctionFS Unbind"),
            Ep0Event::Suspend => trace(&event_counter, "FunctionFS Suspend"),
            Ep0Event::Resume => trace(&event_counter, "FunctionFS Resume"),

            Ep0Event::Enable => {
                log("host configured gadget; starting controller forwarding");
                trace(
                    &event_counter,
                    "FunctionFS Enable: host configured us, starting report forwarding",
                );
                _local_input_grab = Some(crate::hid::EvdevGrabs::acquire(&controller)?);
                trace(&event_counter, "local controller input exclusively grabbed");
                running.store(true, Ordering::SeqCst);
                let input_running = running.clone();
                let input_controller = controller.clone();
                let input_mount = mount.to_path_buf();
                let input_events = event_counter.clone();
                input_forwarders.push(thread::spawn(move || {
                    if let Err(e) = forward_reports(
                        &input_mount,
                        &input_controller,
                        &input_running,
                        &input_events,
                    ) {
                        log(&format!("report forwarder stopped: {e}"));
                    }
                }));

                let mouse_running = running.clone();
                let mouse_controller = auxiliary.clone();
                let mouse_hidg = mouse_hidg.to_path_buf();
                let mouse_events = event_counter.clone();
                input_forwarders.push(thread::spawn(move || {
                    if let Err(e) = forward_auxiliary_reports(
                        "mouse",
                        &mouse_controller.mouse,
                        &mouse_hidg,
                        STEAM_DECK_MOUSE_REPORT_LENGTH,
                        &mouse_running,
                        &mouse_events,
                    ) {
                        log(&format!("mouse report forwarder stopped: {e}"));
                    }
                }));

                let keyboard_running = running.clone();
                let keyboard_controller = auxiliary.clone();
                let keyboard_hidg = keyboard_hidg.to_path_buf();
                let keyboard_events = event_counter.clone();
                input_forwarders.push(thread::spawn(move || {
                    if let Err(e) = forward_auxiliary_reports(
                        "keyboard",
                        &keyboard_controller.keyboard,
                        &keyboard_hidg,
                        STEAM_DECK_KEYBOARD_REPORT_LENGTH,
                        &keyboard_running,
                        &keyboard_events,
                    ) {
                        log(&format!("keyboard report forwarder stopped: {e}"));
                    }
                }));
            }

            Ep0Event::Disable => {
                log("host deconfigured gadget; stopping controller forwarding");
                trace(
                    &event_counter,
                    "FunctionFS Disable: host dropped configuration, stopping forwarding",
                );
                running.store(false, Ordering::SeqCst);
                for h in input_forwarders.drain(..) {
                    let _ = h.join();
                }
                _local_input_grab = None;
                trace(&event_counter, "local controller input released");
            }

            Ep0Event::Setup(req) => handle_setup(
                &mut *ep0,
                &controller,
                &controller.report_descriptor,
                &mut feature_trace,
                &event_counter,
                req,
            )?,
        }
    }
}

/// Forward an exact physical mouse or keyboard report to the matching configfs HID gadget character device.
/// The descriptors were length matched before the relay starts
fn forward_auxiliary_reports(
    name: &str,
    source: &HidRaw,
    hidg_device: &std::path::Path,
    report_length: usize,
    running: &AtomicBool,
    event_counter: &AtomicU64,
) -> std::io::Result<()> {
    let raw_fd = unsafe { libc::dup(source.file.as_raw_fd()) };
    if raw_fd < 0 {
        return Err(std::io::Error::last_os_error());
    }
    let mut hidraw_read = unsafe { std::fs::File::from_raw_fd(raw_fd) };
    let mut hidg_write = std::fs::OpenOptions::new().write(true).open(hidg_device)?;
    let mut buf = vec![0u8; report_length];
    let mut sequence = 0u64;

    while running.load(Ordering::SeqCst) {
        if !wait_readable(hidraw_read.as_raw_fd(), running)? {
            break;
        }
        let n = hidraw_read.read(&mut buf)?;
        if n == 0 {
            continue;
        }
        if n != report_length {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!(
                    "physical {name} hidraw report is {n} bytes; expected {report_length} bytes"
                ),
            ));
        }
        sequence += 1;
        let log_report = should_log_interrupt(sequence);
        if log_report {
            trace(
                event_counter,
                &format!(
                    "{name} HID sequence={sequence}: hidraw read len={n}; HID gadget write started bytes={}",
                    bounded_hex(&buf)
                ),
            );
        }
        match hidg_write.write_all(&buf) {
            Ok(()) if log_report => trace(
                event_counter,
                &format!("{name} HID sequence={sequence}: HID gadget write completed result=ok"),
            ),
            Ok(()) => {}
            Err(error) => {
                trace(
                    event_counter,
                    &format!(
                        "{name} HID sequence={sequence}: HID gadget write failed errno={:?}: {error}",
                        error.raw_os_error()
                    ),
                );
                return Err(error);
            }
        }
    }
    Ok(())
}

/// Continuously read whatever the real controller sends and push it byte-for-byte out the IN endpoint to the host
fn forward_reports(
    mount: &std::path::Path,
    controller: &HidRaw,
    running: &AtomicBool,
    event_counter: &AtomicU64,
) -> std::io::Result<()> {
    let mut ep_in = EpIn::open(mount)?;
    // duplicate the hidraw fd for blocking reads on this thread
    let raw_fd = unsafe { libc::dup(controller.file.as_raw_fd()) };
    if raw_fd < 0 {
        return Err(std::io::Error::last_os_error());
    }
    let mut hidraw_read = unsafe { std::fs::File::from_raw_fd(raw_fd) };

    let mut buf = [0u8; MAX_REPORT];
    let mut input_sequence = 0u64;
    while running.load(Ordering::SeqCst) {
        if !wait_readable(hidraw_read.as_raw_fd(), running)? {
            break;
        }
        let n = hidraw_read.read(&mut buf)?;
        if n == 0 {
            continue;
        }
        input_sequence += 1;
        let log_input = should_log_interrupt(input_sequence);
        if log_input {
            trace(
                event_counter,
                &format!(
                    "interrupt IN sequence={input_sequence}: hidraw read succeeded len={n} ep1_len={n} unchanged=true {} bytes={}",
                    describe_input_report(&buf[..n]),
                    bounded_hex(&buf[..n])
                ),
            );
            trace(
                event_counter,
                &format!("interrupt IN sequence={input_sequence}: EP1 write started len={n}"),
            );
        }
        match ep_in.send(&buf[..n]) {
            Ok(()) if log_input => trace(
                event_counter,
                &format!("interrupt IN sequence={input_sequence}: EP1 write completed result=ok"),
            ),
            Ok(()) => {}
            Err(e) => {
                trace(
                    event_counter,
                    &format!(
                        "interrupt IN sequence={input_sequence}: EP1 write failed errno={:?}: {e}",
                        e.raw_os_error()
                    ),
                );
                break;
            }
        }
    }
    Ok(())
}

fn wait_readable(fd: i32, running: &AtomicBool) -> std::io::Result<bool> {
    while running.load(Ordering::SeqCst) {
        let mut pollfd = libc::pollfd {
            fd,
            events: libc::POLLIN,
            revents: 0,
        };
        let result = unsafe { libc::poll(&mut pollfd, 1, 100) };
        if result > 0 {
            if pollfd.revents & libc::POLLIN != 0 {
                return Ok(true);
            }
            return Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("poll returned unexpected events: 0x{:x}", pollfd.revents),
            ));
        }
        if result == 0 {
            continue;
        }
        let error = std::io::Error::last_os_error();
        if error.raw_os_error() == Some(libc::EINTR) {
            continue;
        }
        return Err(error);
    }
    Ok(false)
}

/// Handle a single control-transfer SETUP packet from ep0.
///
/// Most standard USB requests are handled by the kernel, the HID report descriptor is an exception:
/// FunctionFS delivers the GET_DESCRIPTOR request to this handler, so it must be answered from the copied physical descriptor.
/// NOTE: HID Feature GET_REPORT / SET_REPORT are forwarded to the real controller; unsupported requests are intentionally stalled
fn handle_setup(
    ep0: &mut Ep0,
    controller: &HidRaw,
    report_descriptor: &[u8],
    feature_trace: &mut FeatureTrace,
    event_counter: &AtomicU64,
    req: SetupRequest,
) -> std::io::Result<()> {
    trace(
        event_counter,
        &format!(
            "setup: {}; raw=bmRequestType=0x{:02x} bRequest=0x{:02x} wValue=0x{:04x} wIndex=0x{:04x} wLength={}",
            describe_setup_request(req),
            req.request_type,
            req.request,
            req.value,
            req.index,
            req.length
        ),
    );

    // HID report descriptors are requested through the *standard* USB GET_DESCRIPTOR request (descriptor type 0x22) rather than a HID class request.
    // FunctionFS delivers this setup packet to us; stalling it makes the host enumerate the USB device but decline to create an IOHID device
    if req.request == USB_GET_DESCRIPTOR
        && req.is_device_to_host()
        && (req.value >> 8) as u8 == USB_DT_REPORT
    {
        let length = usize::from(req.length).min(report_descriptor.len());
        let descriptor = &report_descriptor[..length];
        verbose_log(&format!(
            "report descriptor response: requested_wLength={} descriptor_len={} sending_len={} bytes={}",
            req.length,
            report_descriptor.len(),
            descriptor.len(),
            compact_hex(descriptor)
        ));
        match ep0.respond_in(descriptor) {
            Ok(()) => {
                verbose_log("report descriptor response: result=ok");
                return Ok(());
            }
            Err(error) => {
                log(&format!(
                    "report descriptor response: result=error errno={:?}: {error}",
                    error.raw_os_error()
                ));
                return Err(error);
            }
        }
    }

    if !req.is_class() {
        ep0.stall(req.is_device_to_host())?;
        return Ok(());
    }

    let report_id = (req.value & 0x00ff) as u8;
    let report_type = (req.value >> 8) as u8;

    match req.request {
        HID_GET_REPORT
            if req.is_device_to_host()
                && report_type == HID_REPORT_TYPE_FEATURE
                && req.length != 0 =>
        {
            let hidraw_len = match controller.feature_get_hidraw_len(req.length as usize) {
                Ok(len) => len,
                Err(e) => {
                    log(&format!(
                        "feature GET: report_type={report_type} report_id={report_id} requested_usb_len={} numbered={} result=error errno={:?}: {e}",
                        req.length,
                        controller.numbered_reports,
                        e.raw_os_error()
                    ));
                    ep0.stall(req.is_device_to_host())?;
                    return Ok(());
                }
            };
            match controller.get_feature(report_id, req.length as usize) {
                Ok(data) => {
                    let sequence = feature_trace.sequence();
                    let command = data.first().copied().unwrap_or_default();
                    let matches_last_feature_set = feature_trace
                        .last_set
                        .as_deref()
                        .map(|last_set| last_set == data);
                    let response = if matches_last_feature_set == Some(true) {
                        "same bytes as preceding SET".to_owned()
                    } else {
                        format!(
                            "{}; bytes={}",
                            describe_feature_payload(&data, true),
                            bounded_hex(&data)
                        )
                    };
                    trace(event_counter, &format!(
                        "feature sequence {sequence}: GET command=0x{command:02x}({}) report_type={}({}) report_id={report_id} usb_len={} hidraw_buffer_len={hidraw_len} response={response} matches_last_feature_set={matches_last_feature_set:?}",
                        valve_feature_command_name(command),
                        report_type,
                        hid_report_type_name(report_type),
                        data.len(),
                    ));
                    feature_trace.finish_sequence();
                    match ep0.respond_in(&data) {
                        Ok(()) => trace(
                            event_counter,
                            &format!(
                                "feature sequence {sequence}: EP0 response sent len={} result=ok",
                                data.len()
                            ),
                        ),
                        Err(error) => {
                            trace(
                                event_counter,
                                &format!(
                                    "feature sequence {sequence}: EP0 response write failed errno={:?}: {error}",
                                    error.raw_os_error()
                                ),
                            );
                            return Err(error);
                        }
                    }
                }
                Err(e) => {
                    log(&format!(
                        "feature GET: report_type={report_type} report_id={report_id} requested_usb_len={} hidraw_ioctl_buffer_len={hidraw_len} numbered={} result=error errno={:?}: {e}",
                        req.length,
                        controller.numbered_reports,
                        e.raw_os_error()
                    ));
                    ep0.stall(req.is_device_to_host())?;
                }
            }
        }
        HID_GET_REPORT
            if req.is_device_to_host()
                && (report_type == HID_REPORT_TYPE_INPUT
                    || report_type == HID_REPORT_TYPE_OUTPUT) =>
        {
            ep0.stall(req.is_device_to_host())?;
        }
        HID_SET_REPORT if !req.is_device_to_host() && report_type == HID_REPORT_TYPE_FEATURE => {
            let data = ep0.read_out(req.length as usize)?;
            let sequence = feature_trace.sequence();
            let command = data.first().copied().unwrap_or_default();
            trace(event_counter, &format!(
                "feature sequence {sequence}: SET command=0x{command:02x}({}) report_type={}({}) report_id={report_id} usb_len={} action={} bytes={}",
                valve_feature_command_name(command),
                report_type,
                hid_report_type_name(report_type),
                data.len(),
                describe_feature_payload(&data, false),
                bounded_hex(&data)
            ));
            feature_trace.last_set = Some(data.clone());
            let hidraw_len = match controller.feature_set_hidraw_len(data.len()) {
                Ok(len) => len,
                Err(e) => {
                    trace(event_counter, &format!(
                        "feature SET: report_type={report_type} report_id={report_id} usb_len={} numbered={} result=error errno={:?}: {e}",
                        data.len(),
                        controller.numbered_reports,
                        e.raw_os_error()
                    ));
                    return Err(e);
                }
            };
            if let Err(e) = controller.set_feature(&data) {
                trace(event_counter, &format!(
                    "feature sequence {sequence}: SET physical forward failed usb_len={} hidraw_buffer_len={hidraw_len} numbered={} errno={:?}: {e}",
                    data.len(),
                    controller.numbered_reports,
                    e.raw_os_error()
                ));
                return Err(e);
            }
            trace(event_counter, &format!(
                "feature sequence {sequence}: SET forwarded to physical controller usb_len={} hidraw_buffer_len={hidraw_len} numbered={} result=ok",
                data.len(),
                controller.numbered_reports
            ));
        }

        // These requests are part of normal HID bring-up.
        // Steam Deck controller vendor protocol has no boot-protocol distinction and does not use idle rates,
        // so this reports zero/default values and acknowledges the setters rather than stalling the host
        HID_GET_IDLE | HID_GET_PROTOCOL if req.is_device_to_host() && req.length != 0 => {
            ep0.respond_in(&[0])?
        }
        HID_SET_IDLE | HID_SET_PROTOCOL if !req.is_device_to_host() && req.length == 0 => {
            ep0.acknowledge()?
        }
        _ => {
            ep0.stall(req.is_device_to_host())?;
        }
    }
    Ok(())
}

fn log(msg: &str) {
    eprintln!("[deckontroller] {msg}");
}

fn verbose_log(msg: &str) {
    if crate::verbose_enabled() {
        log(msg);
    }
}

fn trace(event_counter: &AtomicU64, msg: &str) {
    if !crate::verbose_enabled() {
        return;
    }
    let event = event_counter.fetch_add(1, Ordering::SeqCst) + 1;
    let elapsed_ms = TRACE_START.get_or_init(Instant::now).elapsed().as_millis();
    log(&format!("event={event} t=+{elapsed_ms}ms {msg}"));
}

fn compact_hex(bytes: &[u8]) -> String {
    let mut output = String::with_capacity(bytes.len().saturating_mul(2));
    for byte in bytes {
        use std::fmt::Write as _;
        let _ = write!(output, "{byte:02x}");
    }
    output
}

fn bounded_hex(bytes: &[u8]) -> String {
    const REPORT_LOG_BYTES: usize = 128;

    if bytes.len() <= REPORT_LOG_BYTES {
        return compact_hex(bytes);
    }
    format!("{}...", compact_hex(&bytes[..REPORT_LOG_BYTES]))
}

fn should_log_interrupt(sequence: u64) -> bool {
    sequence <= 4 || sequence.is_power_of_two()
}
