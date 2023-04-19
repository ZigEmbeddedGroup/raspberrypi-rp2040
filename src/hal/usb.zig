//! USB device implementation
//!
//! Inspired by cbiffle's Rust [implementation](https://github.com/cbiffle/rp2040-usb-device-in-one-file/blob/main/src/main.rs)

const std = @import("std");

const microzig = @import("microzig");
const peripherals = microzig.chip.peripherals;

/// Human Interface Device (HID)
pub const usb = microzig.core.usb;
pub const hid = usb.hid;

const rom = @import("rom.zig");
const resets = @import("resets.zig");

pub const EP0_OUT_IDX = 0;
pub const EP0_IN_IDX = 1;

// +++++++++++++++++++++++++++++++++++++++++++++++++
// User Interface
// +++++++++++++++++++++++++++++++++++++++++++++++++

pub const UsbDeviceConfiguration = usb.UsbDeviceConfiguration;
pub const UsbDeviceDescriptor = usb.UsbDeviceDescriptor;
pub const UsbDescType = usb.UsbDescType;
pub const UsbInterfaceDescriptor = usb.UsbInterfaceDescriptor;
pub const UsbConfigurationDescriptor = usb.UsbConfigurationDescriptor;
pub const UsbEndpointDescriptor = usb.UsbEndpointDescriptor;
pub const UsbEndpointConfiguration = usb.UsbEndpointConfiguration;
pub const UsbDir = usb.UsbDir;
pub const UsbTransferType = usb.UsbTransferType;

pub var EP0_OUT_CFG: usb.UsbEndpointConfiguration = .{
    .descriptor = &usb.UsbEndpointDescriptor{
        .length = @intCast(u8, @sizeOf(usb.UsbEndpointDescriptor)),
        .descriptor_type = usb.UsbDescType.Endpoint,
        .endpoint_address = usb.EP0_OUT_ADDR,
        .attributes = @enumToInt(usb.UsbTransferType.Control),
        .max_packet_size = 64,
        .interval = 0,
    },
    .endpoint_control_index = null,
    .buffer_control_index = 1,
    .data_buffer_index = 0,
    .next_pid_1 = false,
};

pub var EP0_IN_CFG: usb.UsbEndpointConfiguration = .{
    .descriptor = &usb.UsbEndpointDescriptor{
        .length = @intCast(u8, @sizeOf(usb.UsbEndpointDescriptor)),
        .descriptor_type = usb.UsbDescType.Endpoint,
        .endpoint_address = usb.EP0_IN_ADDR,
        .attributes = @enumToInt(usb.UsbTransferType.Control),
        .max_packet_size = 64,
        .interval = 0,
    },
    .endpoint_control_index = null,
    .buffer_control_index = 0,
    .data_buffer_index = 0,
    .next_pid_1 = false,
};

// +++++++++++++++++++++++++++++++++++++++++++++++++
// Reference to endpoint buffers
// +++++++++++++++++++++++++++++++++++++++++++++++++

/// USB data buffers
pub const buffers = struct {
    // Address 0x100-0xfff (3840 bytes) can be used for data buffers.
    const USBDPRAM_BASE = 0x50100100;
    // Data buffers are 64 bytes long as this is the max normal packet size
    const BUFFER_SIZE = 64;
    /// EP0 buffer 0 (shared between in and out)
    const USB_EP0_BUFFER0 = USBDPRAM_BASE;
    /// Optional EP0 buffer 1
    const USB_EP0_BUFFER1 = USBDPRAM_BASE + BUFFER_SIZE;
    /// Data buffers
    const USB_BUFFERS = USBDPRAM_BASE + (2 * BUFFER_SIZE);

    /// Mapping to the different data buffers in DPSRAM
    pub var B: usb.Buffers = .{
        .ep0_buffer0 = @intToPtr([*]u8, USB_EP0_BUFFER0),
        .ep0_buffer1 = @intToPtr([*]u8, USB_EP0_BUFFER1),
        // We will initialize this comptime in a loop
        .rest = .{
            @intToPtr([*]u8, USB_BUFFERS + (0 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (1 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (2 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (3 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (4 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (5 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (6 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (7 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (8 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (9 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (10 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (11 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (12 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (13 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (14 * BUFFER_SIZE)),
            @intToPtr([*]u8, USB_BUFFERS + (15 * BUFFER_SIZE)),
        },
    };
};

var usb_config: ?*usb.UsbDeviceConfiguration = null;

// +++++++++++++++++++++++++++++++++++++++++++++++++
// Code
// +++++++++++++++++++++++++++++++++++++++++++++++++

/// Initialize the USB clock to 48 MHz
///
/// This requres that the system clock has been set up before hand
/// using the 12 MHz crystal.
pub fn usb_clk_init() void {
    // Bring PLL_USB up to 48MHz. PLL_USB is clocked from refclk, which we've
    // already moved over to the 12MHz XOSC. We just need to make it x4 that
    // clock.
    //
    // PLL_USB out of reset
    resets.reset(&.{.pll_usb});
    // Configure it:
    //
    // RFDIV = 1
    // FBDIV = 100 => FOUTVC0 = 1200 MHz
    peripherals.PLL_USB.CS.modify(.{ .REFDIV = 1 });
    peripherals.PLL_USB.FBDIV_INT.modify(.{ .FBDIV_INT = 100 });
    peripherals.PLL_USB.PWR.modify(.{ .PD = 0, .VCOPD = 0 });
    // Wait for lock
    while (peripherals.PLL_USB.CS.read().LOCK == 0) {}
    // Set up post dividers to enable output
    //
    // POSTDIV1 = POSTDIV2 = 5
    // PLL_USB FOUT = 1200 MHz / 25 = 48 MHz
    peripherals.PLL_USB.PRIM.modify(.{ .POSTDIV1 = 5, .POSTDIV2 = 5 });
    peripherals.PLL_USB.PWR.modify(.{ .POSTDIVPD = 0 });
    // Switch usbclk to be derived from PLLUSB
    peripherals.CLOCKS.CLK_USB_CTRL.modify(.{ .AUXSRC = .{ .value = .clksrc_pll_usb } });

    // We now have the stable 48MHz reference clock required for USB:
}

pub fn usb_init_device(device_config: *usb.UsbDeviceConfiguration) void {
    // Bring USB out of reset
    resets.reset(&.{.usbctrl});

    // Clear the control portion of DPRAM. This may not be necessary -- the
    // datasheet is ambiguous -- but the C examples do it, and so do we.
    peripherals.USBCTRL_DPRAM.SETUP_PACKET_LOW.write_raw(0);
    peripherals.USBCTRL_DPRAM.SETUP_PACKET_HIGH.write_raw(0);

    peripherals.USBCTRL_DPRAM.EP1_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP1_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP2_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP2_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP3_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP3_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP4_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP4_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP5_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP5_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP6_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP6_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP7_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP7_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP8_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP8_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP9_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP9_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP10_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP10_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP11_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP11_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP12_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP12_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP13_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP13_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP14_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP14_OUT_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP15_IN_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP15_OUT_CONTROL.write_raw(0);

    peripherals.USBCTRL_DPRAM.EP0_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP0_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP1_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP1_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP2_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP2_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP3_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP3_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP4_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP4_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP5_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP5_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP6_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP6_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP7_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP7_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP8_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP8_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP9_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP9_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP10_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP10_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP11_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP11_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP12_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP12_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP13_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP13_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP14_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP14_OUT_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP15_IN_BUFFER_CONTROL.write_raw(0);
    peripherals.USBCTRL_DPRAM.EP15_OUT_BUFFER_CONTROL.write_raw(0);

    // Mux the controller to the onboard USB PHY. I was surprised that there are
    // alternatives to this, but, there are.
    peripherals.USBCTRL_REGS.USB_MUXING.modify(.{
        .TO_PHY = 1,
        // This bit is also set in the SDK example, without any discussion. It's
        // undocumented (being named does not count as being documented).
        .SOFTCON = 1,
    });

    // Force VBUS detect. Not all RP2040 boards wire up VBUS detect, which would
    // let us detect being plugged into a host (the Pi Pico, to its credit,
    // does). For maximum compatibility, we'll set the hardware to always
    // pretend VBUS has been detected.
    peripherals.USBCTRL_REGS.USB_PWR.modify(.{
        .VBUS_DETECT = 1,
        .VBUS_DETECT_OVERRIDE_EN = 1,
    });

    // Enable controller in device mode.
    peripherals.USBCTRL_REGS.MAIN_CTRL.modify(.{
        .CONTROLLER_EN = 1,
        .HOST_NDEVICE = 0,
    });

    // Request to have an interrupt (which really just means setting a bit in
    // the `buff_status` register) every time a buffer moves through EP0.
    peripherals.USBCTRL_REGS.SIE_CTRL.modify(.{
        .EP0_INT_1BUF = 1,
    });

    // Enable interrupts (bits set in the `ints` register) for other conditions
    // we use:
    peripherals.USBCTRL_REGS.INTE.modify(.{
        // A buffer is done
        .BUFF_STATUS = 1,
        // The host has reset us
        .BUS_RESET = 1,
        // We've gotten a setup request on EP0
        .SETUP_REQ = 1,
    });

    // setup endpoints
    for (device_config.endpoints) |ep| {
        // EP0 doesn't have an endpoint control index; only process the other
        // endpoints here.
        if (ep.endpoint_control_index) |epci| {
            // We need to compute the offset from the base of USB SRAM to the
            // buffer we're choosing, because that's how the peripheral do.
            const buf_base = @ptrToInt(buffers.B.get(ep.data_buffer_index));
            const dpram_base = @ptrToInt(peripherals.USBCTRL_DPRAM);
            // The offset _should_ fit in a u16, but if we've gotten something
            // wrong in the past few lines, a common symptom will be integer
            // overflow producing a Very Large Number,
            const dpram_offset = @intCast(u16, buf_base - dpram_base);

            // Configure the endpoint!
            modify_endpoint_control(epci, .{
                .ENABLE = 1,
                // Please set the corresponding bit in buff_status when a
                // buffer is done, thx.
                .INTERRUPT_PER_BUFF = 1,
                // Select bulk vs control (or interrupt as soon as implemented).
                .ENDPOINT_TYPE = .{ .raw = @intCast(u2, ep.descriptor.attributes) },
                // And, designate our buffer by its offset.
                .BUFFER_ADDRESS = dpram_offset,
            });
        }
    }

    // Present full-speed device by enabling pullup on DP. This is the point
    // where the host will notice our presence.
    peripherals.USBCTRL_REGS.SIE_CTRL.modify(.{ .PULLUP_EN = 1 });

    // use this config until a new configuration is provided
    usb_config = device_config;
}

/// Usb task function meant to be executed in regular intervals after
/// initializing the device.
pub fn usb_task(debug: bool) void {
    // We'll keep some state in Plain Old Static Local Variables:
    const S = struct {
        // When the host gives us a new address, we can't just slap it into
        // registers right away, because we have to do an acknowledgement step using
        // our _old_ address.
        var new_address: ?u8 = null;
        // Flag recording whether the host has configured us with a
        // `SetConfiguration` message.
        var configured = false;
        // Flag recording whether we've set up buffer transfers after being
        // configured.
        var started = false;
        // Some scratch space that we'll use for things like preparing string
        // descriptors for transmission.
        var tmp: [64]u8 = .{0} ** 64;
    };

    var device_config: *usb.UsbDeviceConfiguration = if (usb_config) |cfg| cfg else return;

    // Check which interrupt flags are set.
    const ints = get_interrupts();

    // Setup request received?
    if (ints.SetupReq) {
        if (debug) std.log.info("setup req", .{});

        // Get the setup request setup packet
        const setup = get_setup_packet();

        // Reset PID to 1 for EP0 IN. Every DATA packet we send in response
        // to an IN on EP0 needs to use PID DATA1, and this line will ensure
        // that.
        device_config.endpoints[EP0_IN_IDX].next_pid_1 = true;

        // Attempt to parse the request type and request into one of our
        // known enum values, and then inspect them. (These will return None
        // if we get an unexpected numeric value.)
        const reqty = usb.UsbDir.of_endpoint_addr(setup.request_type);
        const req = usb.UsbSetupRequest.from_u8(setup.request);

        if (reqty == usb.UsbDir.Out and req != null and req.? == usb.UsbSetupRequest.SetAddress) {
            // The new address is in the bottom 8 bits of the setup
            // packet value field. Store it for use later.
            S.new_address = @intCast(u8, setup.value & 0xff);
            // The address will actually get set later, we have
            // to use address 0 to send a status response.
            usb_start_tx(
                device_config.endpoints[EP0_IN_IDX], //EP0_IN_CFG,
                &.{}, // <- see, empty buffer
            );
            if (debug) std.log.info("    SetAddress: {}", .{S.new_address.?});
        } else if (reqty == usb.UsbDir.Out and req != null and req.? == usb.UsbSetupRequest.SetConfiguration) {
            // We only have one configuration, and it doesn't really
            // mean anything to us -- more of a formality. All we do in
            // response to this is:
            S.configured = true;
            usb_start_tx(
                device_config.endpoints[EP0_IN_IDX], //EP0_IN_CFG,
                &.{}, // <- see, empty buffer
            );
            if (debug) std.log.info("    SetConfiguration", .{});
        } else if (reqty == usb.UsbDir.Out) {
            // This is sort of a hack, but: if we get any other kind of
            // OUT, just acknowledge it with the same zero-length status
            // phase that we use for control transfers that we _do_
            // understand. This keeps the host from spinning forever
            // while we NAK.
            //
            // This behavior copied shamelessly from the C example.
            usb_start_tx(
                device_config.endpoints[EP0_IN_IDX], // EP0_IN_CFG,
                &.{}, // <- see, empty buffer
            );
            if (debug) std.log.info("    Just OUT", .{});
        } else if (reqty == usb.UsbDir.In and req != null and req.? == usb.UsbSetupRequest.GetDescriptor) {
            // Identify the requested descriptor type, which is in the
            // _top_ 8 bits of value.
            const descriptor_type = usb.UsbDescType.from_u16(setup.value >> 8);
            if (debug) std.log.info("    GetDescriptor: {}", .{setup.value >> 8});
            if (descriptor_type) |dt| {
                switch (dt) {
                    .Device => {
                        if (debug) std.log.info("        Device", .{});
                        // TODO: this sure looks like a duplicate, but it's
                        // a duplicate that was present in the C
                        // implementation.
                        device_config.endpoints[EP0_IN_IDX].next_pid_1 = true;

                        const dc = device_config.device_descriptor.serialize();
                        std.mem.copy(u8, S.tmp[0..dc.len], &dc);

                        // Configure EP0 IN to send the device descriptor
                        // when it's next asked.
                        usb_start_tx(
                            device_config.endpoints[EP0_IN_IDX],
                            S.tmp[0..dc.len],
                        );
                    },
                    .Config => {
                        if (debug) std.log.info("        Config", .{});
                        // Config descriptor requests are slightly unusual.
                        // We can respond with just our config descriptor,
                        // but we can _also_ append our interface and
                        // endpoint descriptors to the end, saving some
                        // round trips. We'll choose to do this if the
                        // number of bytes the host will accept (in the
                        // `length` field) is large enough.
                        var used: usize = 0;

                        const cd = device_config.config_descriptor.serialize();
                        std.mem.copy(u8, S.tmp[used .. used + cd.len], &cd);
                        used += cd.len;

                        if (setup.length > used) {
                            // Do the rest!
                            //
                            // This is slightly incorrect because the host
                            // might have asked for a number of bytes in
                            // between the size of a config descriptor, and
                            // the amount we're going to send back. However,
                            // in practice, the host always asks for either
                            // (1) the exact size of a config descriptor, or
                            // (2) 64 bytes, and this all fits in 64 bytes.
                            const id = device_config.interface_descriptor.serialize();
                            std.mem.copy(u8, S.tmp[used .. used + id.len], &id);
                            used += id.len;

                            // Seems like the host does not bother asking for the
                            // hid descriptor so we'll just send it with the
                            // other descriptors.
                            if (device_config.hid) |hid_conf| {
                                const hd = hid_conf.hid_descriptor.serialize();
                                std.mem.copy(u8, S.tmp[used .. used + hd.len], &hd);
                                used += hd.len;
                            }

                            // TODO: depending on the number of endpoints
                            // this might not fit in 64 bytes -> split message
                            // into multiple packets
                            for (device_config.endpoints[2..]) |ep| {
                                const ed = ep.descriptor.serialize();
                                std.mem.copy(u8, S.tmp[used .. used + ed.len], &ed);
                                used += ed.len;
                            }
                        }

                        // Set up EP0 IN to send the stuff we just composed.
                        usb_start_tx(
                            device_config.endpoints[EP0_IN_IDX],
                            S.tmp[0..used],
                        );
                    },
                    .String => {
                        if (debug) std.log.info("        String", .{});
                        // String descriptor index is in bottom 8 bits of
                        // `value`.
                        const i = @intCast(usize, setup.value & 0xff);
                        const bytes = StringBlk: {
                            if (i == 0) {
                                // Special index 0 requests the language
                                // descriptor.
                                break :StringBlk device_config.lang_descriptor;
                            } else {
                                // Otherwise, set up one of our strings.
                                const s = device_config.descriptor_strings[i - 1];
                                const len = 2 + s.len;

                                S.tmp[0] = @intCast(u8, len);
                                S.tmp[1] = 0x03;
                                std.mem.copy(u8, S.tmp[2..len], s);

                                break :StringBlk S.tmp[0..len];
                            }
                        };
                        // Set up EP0 IN to send whichever thing we just
                        // decided on.
                        usb_start_tx(
                            device_config.endpoints[EP0_IN_IDX],
                            bytes,
                        );
                    },
                    .Interface => {
                        if (debug) std.log.info("        Interface", .{});
                        // We don't expect the host to send this because we
                        // delivered our interface descriptor with the
                        // config descriptor.
                        //
                        // Should probably implement it, though, because
                        // otherwise the host will be unhappy. TODO.
                        //
                        // Note that the C example gets away with ignoring
                        // this.
                    },
                    .Endpoint => {
                        if (debug) std.log.info("        Endpoint", .{});
                        // Same deal as interface descriptors above.
                    },
                    .DeviceQualifier => {
                        if (debug) std.log.info("        DeviceQualifier", .{});
                        // We will just copy parts of the DeviceDescriptor because
                        // the DeviceQualifierDescriptor can be seen as a subset.
                        const dqd = usb.DeviceQualifierDescriptor{
                            .bcd_usb = device_config.device_descriptor.bcd_usb,
                            .device_class = device_config.device_descriptor.device_class,
                            .device_subclass = device_config.device_descriptor.device_subclass,
                            .device_protocol = device_config.device_descriptor.device_protocol,
                            .max_packet_size0 = device_config.device_descriptor.max_packet_size0,
                            .num_configurations = device_config.device_descriptor.num_configurations,
                        };

                        const data = dqd.serialize();
                        std.mem.copy(u8, S.tmp[0..data.len], &data);

                        usb_start_tx(
                            device_config.endpoints[EP0_IN_IDX],
                            S.tmp[0..data.len],
                        );
                    },
                }
            } else {
                // Maybe the unknown request type is a hid request

                if (device_config.hid) |hid_conf| {
                    const _hid_desc_type = hid.HidDescType.from_u16(setup.value >> 8);

                    if (_hid_desc_type) |hid_desc_type| {
                        switch (hid_desc_type) {
                            .Hid => {
                                if (debug) std.log.info("        HID", .{});

                                const hd = hid_conf.hid_descriptor.serialize();
                                std.mem.copy(u8, S.tmp[0..hd.len], &hd);

                                usb_start_tx(
                                    device_config.endpoints[EP0_IN_IDX],
                                    S.tmp[0..hd.len],
                                );
                            },
                            .Report => {
                                if (debug) std.log.info("        Report", .{});

                                // The report descriptor is already a (static)
                                // u8 array, i.e., we can pass it directly
                                usb_start_tx(
                                    device_config.endpoints[EP0_IN_IDX],
                                    hid_conf.report_descriptor,
                                );
                            },
                            .Physical => {
                                if (debug) std.log.info("        Physical", .{});
                                // Ignore for now
                            },
                        }
                    } else {
                        // It's not a valid HID request. This can totally happen
                        // we'll just ignore it for now...
                    }
                }
            }
        } else if (reqty == usb.UsbDir.In) {
            if (debug) std.log.info("    Just IN", .{});
            // Other IN request. Ignore.
        } else {
            if (debug) std.log.info("    This is unexpected", .{});
            // Unexpected request type or request bits. This can totally
            // happen (yay, hardware!) but is rare in practice. Ignore
            // it.
        }
    } // <-- END of setup request handling

    // Events on one or more buffers? (In practice, always one.)
    if (ints.BuffStatus) {
        if (debug) std.log.info("buff status", .{});
        var iter = get_EPBIter(device_config);

        while (iter.next(&iter)) |epb| {
            if (debug) std.log.info("    data: {any}", .{epb.buffer});

            // Perform any required action on the data. For OUT, the `data`
            // will be whatever was sent by the host. For IN, it's a copy of
            // whatever we sent.
            switch (epb.endpoint.descriptor.endpoint_address) {
                usb.EP0_IN_ADDR => {
                    if (debug) std.log.info("    EP0_IN_ADDR", .{});
                    // We use this opportunity to finish the delayed
                    // SetAddress request, if there is one:
                    if (S.new_address) |addr| {
                        // Change our address:
                        peripherals.USBCTRL_REGS.ADDR_ENDP.modify(.{ .ADDRESS = @intCast(u7, addr) });
                    } else {
                        // Otherwise, we've just finished sending
                        // something to the host. We expect an ensuing
                        // status phase where the host sends us (via EP0
                        // OUT) a zero-byte DATA packet, so, set that
                        // up:
                        usb_start_rx(
                            device_config.endpoints[EP0_OUT_IDX], // EP0_OUT_CFG,
                            0,
                        );
                    }
                },
                else => {
                    if (debug) std.log.info("    ELSE, ep_addr: {}", .{
                        epb.endpoint.descriptor.endpoint_address & 0x7f,
                    });
                    // Handle user provided endpoints.

                    // Invoke the callback (if the user provides one).
                    if (epb.endpoint.callback) |callback| callback(device_config, epb.buffer);
                },
            }
        }
    } // <-- END of buf status handling

    // Has the host signaled a bus reset?
    if (ints.BusReset) {
        if (debug) std.log.info("bus reset", .{});

        // Reset the device
        bus_reset();

        // Reset our state.
        S.new_address = null;
        S.configured = false;
        S.started = false;
    }

    // If we have been configured but haven't reached this point yet, set up
    // our custom EP OUT's to receive whatever data the host wants to send.
    if (S.configured and !S.started) {
        // We can skip the first two endpoints because those are EP0_OUT and EP0_IN
        for (device_config.endpoints[2..]) |ep| {
            if (usb.UsbDir.of_endpoint_addr(ep.descriptor.endpoint_address) == .Out) {
                // Hey host! we expect data!
                usb_start_rx(
                    ep,
                    64,
                );
            }
        }
        S.started = true;
    }
}

// +++++++++++++++++++++++++++++++++++++++++++++++++
// Utility functions
// +++++++++++++++++++++++++++++++++++++++++++++++++

/// Check if the corresponding buffer is available
pub fn buffer_available(
    ep: *usb.UsbEndpointConfiguration,
) bool {
    const rbc = read_raw_buffer_control(ep.buffer_control_index);
    // Bit 11 of the EPn_X_BUFFER_CONTROL register represents the AVAILABLE_0 flag
    return ((rbc & 0x400) == 0);
}

/// Configures a given endpoint to send data (device-to-host, IN) when the host
/// next asks for it.
///
/// The contents of `buffer` will be _copied_ into USB SRAM, so you can
/// reuse `buffer` immediately after this returns. No need to wait for the
/// packet to be sent.
pub fn usb_start_tx(
    ep: *usb.UsbEndpointConfiguration,
    buffer: []const u8,
) void {
    // It is technically possible to support longer buffers but this demo
    // doesn't bother.
    // TODO: assert!(buffer.len() <= 64);
    // You should only be calling this on IN endpoints.
    // TODO: assert!(UsbDir::of_endpoint_addr(ep.descriptor.endpoint_address) == UsbDir::In);

    // Copy the given data into the corresponding ep buffer
    const epbuffer = buffers.B.get(ep.data_buffer_index);
    _ = rom.memcpy(epbuffer[0..buffer.len], buffer);

    // Configure the IN:
    const np: u1 = if (ep.next_pid_1) 1 else 0;

    // The AVAILABLE bit in the buffer control register should be set
    // separately to the rest of the data in the buffer control register,
    // so that the rest of the data in the buffer control register is
    // accurate when the AVAILABLE bit is set.

    // Write the buffer information to the buffer control register
    modify_buffer_control(ep.buffer_control_index, .{
        .PID_0 = np, // DATA0/1, depending
        .FULL_0 = 1, // We have put data in
        .LENGTH_0 = @intCast(u10, buffer.len), // There are this many bytes
    });

    // Nop for some clock cycles
    // use volatile so the compiler doesn't optimize the nops away
    asm volatile (
        \\ nop
        \\ nop
        \\ nop
    );

    // Set available bit
    modify_buffer_control(ep.buffer_control_index, .{
        .AVAILABLE_0 = 1, // The data is for the computer to use now
    });

    ep.next_pid_1 = !ep.next_pid_1;
}

pub fn usb_start_rx(
    ep: *usb.UsbEndpointConfiguration,
    len: usize,
) void {
    // It is technically possible to support longer buffers but this demo
    // doesn't bother.
    // TODO: assert!(len <= 64);
    // You should only be calling this on OUT endpoints.
    // TODO: assert!(UsbDir::of_endpoint_addr(ep.descriptor.endpoint_address) == UsbDir::Out);

    // Check which DATA0/1 PID this endpoint is expecting next.
    const np: u1 = if (ep.next_pid_1) 1 else 0;
    // Configure the OUT:
    modify_buffer_control(ep.buffer_control_index, .{
        .PID_0 = np, // DATA0/1 depending
        .FULL_0 = 0, // Buffer is NOT full, we want the computer to fill it
        .AVAILABLE_0 = 1, // It is, however, available to be filled
        .LENGTH_0 = @intCast(u10, len), // Up tho this many bytes
    });

    // Flip the DATA0/1 PID for the next receive
    ep.next_pid_1 = !ep.next_pid_1;
}

pub fn modify_buffer_control(
    i: usize,
    fields: anytype,
) void {
    // haven't found a better way to handle this
    switch (i) {
        0 => peripherals.USBCTRL_DPRAM.EP0_IN_BUFFER_CONTROL.modify(fields),
        1 => peripherals.USBCTRL_DPRAM.EP0_OUT_BUFFER_CONTROL.modify(fields),
        2 => peripherals.USBCTRL_DPRAM.EP1_IN_BUFFER_CONTROL.modify(fields),
        3 => peripherals.USBCTRL_DPRAM.EP1_OUT_BUFFER_CONTROL.modify(fields),
        4 => peripherals.USBCTRL_DPRAM.EP2_IN_BUFFER_CONTROL.modify(fields),
        5 => peripherals.USBCTRL_DPRAM.EP2_OUT_BUFFER_CONTROL.modify(fields),
        else => unreachable, // TODO: actually reachable but we don't care for now
    }
}

pub fn read_raw_buffer_control(
    i: usize,
) u32 {
    // haven't found a better way to handle this
    return switch (i) {
        0 => peripherals.USBCTRL_DPRAM.EP0_IN_BUFFER_CONTROL.raw,
        1 => peripherals.USBCTRL_DPRAM.EP0_OUT_BUFFER_CONTROL.raw,
        2 => peripherals.USBCTRL_DPRAM.EP1_IN_BUFFER_CONTROL.raw,
        3 => peripherals.USBCTRL_DPRAM.EP1_OUT_BUFFER_CONTROL.raw,
        4 => peripherals.USBCTRL_DPRAM.EP2_IN_BUFFER_CONTROL.raw,
        5 => peripherals.USBCTRL_DPRAM.EP2_OUT_BUFFER_CONTROL.raw,
        else => unreachable, // TODO: actually reachable but we don't care for now
    };
}

pub fn modify_endpoint_control(
    epci: usize,
    fields: anytype,
) void {
    // haven't found a better way to handle this
    switch (epci) {
        1 => peripherals.USBCTRL_DPRAM.EP1_IN_CONTROL.modify(fields),
        2 => peripherals.USBCTRL_DPRAM.EP1_OUT_CONTROL.modify(fields),
        3 => peripherals.USBCTRL_DPRAM.EP2_IN_CONTROL.modify(fields),
        4 => peripherals.USBCTRL_DPRAM.EP2_OUT_CONTROL.modify(fields),
        5 => peripherals.USBCTRL_DPRAM.EP3_IN_CONTROL.modify(fields),
        6 => peripherals.USBCTRL_DPRAM.EP3_OUT_CONTROL.modify(fields),
        else => unreachable, // TODO: actually reachable but we don't care for now
    }
}

// -----------------------------------------------------------

/// Check which interrupt flags are set
pub fn get_interrupts() usb.InterruptStatus {
    const ints = peripherals.USBCTRL_REGS.INTS.read();

    return .{
        .BuffStatus = if (ints.BUFF_STATUS == 1) true else false,
        .BusReset = if (ints.BUS_RESET == 1) true else false,
        .DevConnDis = if (ints.DEV_CONN_DIS == 1) true else false,
        .DevSuspend = if (ints.DEV_SUSPEND == 1) true else false,
        .DevResumeFromHost = if (ints.DEV_RESUME_FROM_HOST == 1) true else false,
        .SetupReq = if (ints.SETUP_REQ == 1) true else false,
    };
}

/// Returns a received USB setup packet
///
/// Side effect: The setup request status flag will be cleared
///
/// One can assume that this function is only called if the
/// setup request falg is set.
pub fn get_setup_packet() usb.UsbSetupPacket {
    // Clear the status flag (write-one-to-clear)
    peripherals.USBCTRL_REGS.SIE_STATUS.modify(.{ .SETUP_REC = 1 });

    // This assumes that the setup packet is arriving on EP0, our
    // control endpoint. Which it should be. We don't have any other
    // Control endpoints.

    // Copy the setup packet out of its dedicated buffer at the base of
    // USB SRAM. The PAC models this buffer as two 32-bit registers,
    // which is, like, not _wrong_ but slightly awkward since it means
    // we can't just treat it as bytes. Instead, copy it out to a byte
    // array.
    var setup_packet: [8]u8 = .{0} ** 8;
    const spl: u32 = peripherals.USBCTRL_DPRAM.SETUP_PACKET_LOW.raw;
    const sph: u32 = peripherals.USBCTRL_DPRAM.SETUP_PACKET_HIGH.raw;
    _ = rom.memcpy(setup_packet[0..4], std.mem.asBytes(&spl));
    _ = rom.memcpy(setup_packet[4..8], std.mem.asBytes(&sph));
    // Reinterpret as setup packet
    return std.mem.bytesToValue(usb.UsbSetupPacket, &setup_packet);
}

/// Called on a bus reset interrupt
pub fn bus_reset() void {
    // Acknowledge by writing the write-one-to-clear status bit.
    peripherals.USBCTRL_REGS.SIE_STATUS.modify(.{ .BUS_RESET = 1 });
    peripherals.USBCTRL_REGS.ADDR_ENDP.modify(.{ .ADDRESS = 0 });
}

pub const EPBError = error{
    /// The system has received a buffer event for an unknown endpoint (this is super unlikely)
    UnknownEndpoint,
    /// The buffer is not available (this is super unlikely)
    NotAvailable,
};

pub const EPB = struct {
    endpoint: *usb.UsbEndpointConfiguration,
    buffer: []u8,
};

/// Iterator over all input buffers that hold data
pub const EPBIter = struct {
    bufbits: u32,
    last_bit: ?u32 = null,
    device_config: *const usb.UsbDeviceConfiguration,
    /// Get the next available input buffer
    next: *const fn (self: *@This()) ?EPB,
};

pub fn get_EPBIter(dc: *const usb.UsbDeviceConfiguration) EPBIter {
    return .{
        .bufbits = peripherals.USBCTRL_REGS.BUFF_STATUS.raw,
        .device_config = dc,
        .next = next,
    };
}

pub fn next(self: *EPBIter) ?EPB {
    if (self.last_bit) |lb| {
        // Acknowledge the last handled buffer
        peripherals.USBCTRL_REGS.BUFF_STATUS.write_raw(lb);
        self.last_bit = null;
    }
    // All input buffers handled
    if (self.bufbits == 0) return null;

    // Who's still outstanding? Find their bit index by counting how
    // many LSBs are zero.
    var lowbit_index: u5 = 0;
    while ((self.bufbits >> lowbit_index) & 0x01 == 0) : (lowbit_index += 1) {}
    // Remove their bit from our set.
    const lowbit = @intCast(u32, 1) << lowbit_index;
    self.last_bit = lowbit;
    self.bufbits ^= lowbit;

    // Here we exploit knowledge of the ordering of buffer control
    // registers in the peripheral. Each endpoint has a pair of
    // registers, so we can determine the endpoint number by:
    const epnum = @intCast(u8, lowbit_index >> 1);
    // Of the pair, the IN endpoint comes first, followed by OUT, so
    // we can get the direction by:
    const dir = if (lowbit_index & 1 == 0) usb.UsbDir.In else usb.UsbDir.Out;

    const ep_addr = dir.endpoint(epnum);
    // Process the buffer-done event.

    // Process the buffer-done event.
    //
    // Scan the device table to figure out which endpoint struct
    // corresponds to this address. We could use a smarter
    // method here, but in practice, the number of endpoints is
    // small so a linear scan doesn't kill us.
    var endpoint: ?*usb.UsbEndpointConfiguration = null;
    for (self.device_config.endpoints) |ep| {
        if (ep.descriptor.endpoint_address == ep_addr) {
            endpoint = ep;
            break;
        }
    }
    // Buffer event for unknown EP?!
    // TODO: if (endpoint == null) return EPBError.UnknownEndpoint;
    // Read the buffer control register to check status.
    const bc = read_raw_buffer_control(endpoint.?.buffer_control_index);

    // We should only get here if we've been notified that
    // the buffer is ours again. This is indicated by the hw
    // _clearing_ the AVAILABLE bit.
    //
    // This ensures that we can return a shared reference to
    // the databuffer contents without races.
    // TODO: if ((bc & (1 << 10)) == 1) return EPBError.NotAvailable;

    // Cool. Checks out.

    // Get a pointer to the buffer in USB SRAM. This is the
    // buffer _contents_. See the safety comments below.
    const epbuffer = buffers.B.get(endpoint.?.data_buffer_index);

    // Get the actual length of the data, which may be less
    // than the buffer size.
    const len = @intCast(usize, bc & 0x3ff);

    // Copy the data from SRAM
    return EPB{
        .endpoint = endpoint.?,
        .buffer = epbuffer[0..len],
    };
}

test "usb tests" {
    _ = hid;
}
