//! USB HID
//!
//! ## Interfaces
//!
//! HID class devices use Control and Interrupt pipes for communication
//!
//! 1. Control for USB control and data classes, reports, data from host
//! 2. Interrupt for asynchronous and low latency data to the device
//!
//! ## Settings
//!
//! ### UsbInterfaceDescriptor related settings:
//! The class type for a HID class device is defined by the Interface descriptor.
//! The subclass field is used to identify Boot Devices.
//!
//! * `interface_subclass` - 1 if boot interface, 0 else (most of the time)
//! * `interface_protocol` - 0 if no boot interface, 1 if keyboard boot interface, 2 if mouse bi

const usb = @import("../usb.zig");

// +++++++++++++++++++++++++++++++++++++++++++++++++
// Common Data Types
// +++++++++++++++++++++++++++++++++++++++++++++++++

/// USB HID descriptor
pub const HidDescriptor = packed struct {
    length: u8 = 9,
    descriptor_type: usb.UsbDescType = usb.UsbDescType.Hid,
    /// Numeric expression identifying the HID Class Specification release
    bcd_hid: u16,
    /// Numeric expression identifying country code of the localized hardware
    country_code: u8,
    /// Numeric expression specifying the number of class descriptors
    num_descriptors: u8,
    /// Type of HID class report
    report_type: usb.UsbDescType = usb.UsbDescType.Report,
    /// The total size of the Report descriptor
    report_length: u16,

    pub fn serialize(self: *const @This()) [9]u8 {
        var out: [9]u8 = undefined;
        out[0] = 9; // length
        out[1] = @enumToInt(self.descriptor_type);
        out[2] = @intCast(u8, self.bcd_hid & 0xff);
        out[3] = @intCast(u8, (self.bcd_hid >> 8) & 0xff);
        out[4] = self.country_code;
        out[5] = self.num_descriptors;
        out[6] = @enumToInt(self.report_type);
        out[7] = @intCast(u8, self.report_length & 0xff);
        out[8] = @intCast(u8, (self.report_length >> 8) & 0xff);
        return out;
    }
};

/// HID interface Subclass (for UsbInterfaceDescriptor)
pub const Subclass = enum(u8) {
    /// No Subclass
    None = 0,
    /// Boot Interface Subclass
    Boot = 1,
};

/// HID interface protocol
pub const Protocol = enum(u8) {
    /// No protocol
    None = 0,
    /// Keyboard (only if boot interface)
    Keyboard = 1,
    /// Mouse (only if boot interface)
    Mouse = 2,
};

/// HID request report type
pub const ReportType = enum(u8) {
    Invalid = 0,
    Input = 1,
    Output = 2,
    Feature = 3,
};

/// HID class specific control request
pub const Request = enum(u8) {
    GetReport = 0x01,
    GetIdle = 0x02,
    GetProtocol = 0x03,
    SetReport = 0x09,
    SetIdle = 0x0a,
    SetProtocol = 0x0b,
};

/// HID country codes
pub const CountryCode = enum(u8) {
    NotSupported = 0,
    Arabic,
    Belgian,
    CanadianBilingual,
    CanadianFrench,
    CzechRepublic,
    Danish,
    Finnish,
    French,
    German,
    Greek,
    Hebrew,
    Hungary,
    International,
    Italian,
    JapanKatakana,
    Korean,
    LatinAmerica,
    NetherlandsDutch,
    Norwegian,
    PersianFarsi,
    Poland,
    Portuguese,
    Russia,
    Slovakia,
    Spanish,
    Swedish,
    SwissFrench,
    SwissGerman,
    Switzerland,
    Taiwan,
    TurkishQ,
    Uk,
    Us,
    Yugoslavia,
    TurkishF,
};

// +++++++++++++++++++++++++++++++++++++++++++++++++
// Report Descriptor Data Types
// +++++++++++++++++++++++++++++++++++++++++++++++++

pub const ReportItemTypes = enum(u2) {
    Main = 0,
    Global = 1,
    Local = 2,
};

pub const ReportItemMainGroup = enum(u4) {
    Input = 8,
    Output = 9,
    Collection = 10,
    Feature = 11,
    CollectionEnd = 12,
};

pub const CollectionItem = enum(u4) {
    Physical = 0,
    Application,
    Logical,
    Report,
    NamedArray,
    UsageSwitch,
    UsageModifier,
};

pub const GlobalItem = enum(u4) {
    UsagePage = 0,
    LogicalMin = 1,
    LogicalMax = 2,
    PhysicalMin = 3,
    PhysicalMax = 4,
    UnitExponent = 5,
    Unit = 6,
    ReportSize = 7,
    ReportId = 8,
    ReportCount = 9,
    Push = 10,
    Pop = 11,
};

const HID_DATA: u8 = 0 << 0;
const HID_CONSTANT: u8 = 1 << 0;

const HID_ARRAY = 0 << 1;
const HID_VARIABLE = 1 << 1;

const HID_ABSOLUTE = 0 << 2;
const HID_RELATIVE = 1 << 2;

const HID_WRAP_NO = 0 << 3;
const HID_WRAP = 1 << 3;

const HID_LINEAR = 0 << 4;
const HID_NONLINEAR = 1 << 4;

const HID_PREFERRED_STATE = 0 << 5;
const HID_PREFERRED_NO = 1 << 5;

const HID_NO_NULL_POSITION = 0 << 6;
const HID_NULL_STATE = 1 << 6;

const HID_NON_VOLATILE = 0 << 7;
const HID_VOLATILE = 1 << 7;

const HID_BITFIELD = 0 << 8;
const HID_BUFFERED_BYTES = 1 << 8;

pub fn hid_report_item_1(
    out: []u8,
    size: u2,
    @"type": u2,
    tag: u4,
    data: u8,
) void {
    out[0] = (@intCast(u8, tag) << 4) | (@intCast(u8, @"type") << 4) | size;
    out[1] = data;
}

pub fn hid_report_item_2(
    out: []u8,
    size: u2,
    @"type": u2,
    tag: u4,
    data: u16,
) void {
    out[0] = (@intCast(u8, tag) << 4) | (@intCast(u8, @"type") << 4) | size;
    out[1] = @intCast(u8, data & 0xff);
    out[2] = @intCast(u8, (data >> 8) & 0xff);
}

pub fn hid_input(out: []u8, data: u8) void {
    hid_report_item_1(
        out,
        1,
        @enumToInt(ReportItemTypes.Main),
        @enumToInt(ReportItemMainGroup.Input),
        data,
    );
}

test "create hid report item" {}
