//! USB device implementation
//!
//! Inspired by cbiffle's Rust [implementation](https://github.com/cbiffle/rp2040-usb-device-in-one-file/blob/main/src/main.rs)

/// Human Interface Device (HID)
pub const hid = @import("usb/hid.zig");

const std = @import("std");

const microzig = @import("microzig");
const peripherals = microzig.chip.peripherals;

const rom = @import("rom.zig");
const resets = @import("resets.zig");

pub const LED_PIN: u8 = 25;
pub const SETUP_PIN: u8 = 0;
pub const BUFF_PIN: u8 = 1;
pub const RESET_PIN: u8 = 2;
pub const EP_PIN: [3]u8 = .{ 3, 4, 5 };

pub const EP0_OUT_IDX = 0;
pub const EP0_IN_IDX = 1;

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
    pub var B: Buffers = .{
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

var usb_config: ?*UsbDeviceConfiguration = null;

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

pub fn usb_init_device(device_config: *UsbDeviceConfiguration) void {
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

    //////////////////////////////////////////////////////////////////////////
    // Back to USB setup.

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

pub fn usb_task() void {
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

    var device_config: *UsbDeviceConfiguration = if (usb_config) |cfg| cfg else return;

    // Check which interrupt flags are set.
    const ints = peripherals.USBCTRL_REGS.INTS.read();

    // Setup request received?
    if (ints.SETUP_REQ == 1) {
        std.log.info("setup req", .{});
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
        const setup = std.mem.bytesToValue(UsbSetupPacket, &setup_packet);

        // Reset PID to 1 for EP0 IN. Every DATA packet we send in response
        // to an IN on EP0 needs to use PID DATA1, and this line will ensure
        // that.
        device_config.endpoints[EP0_IN_IDX].next_pid_1 = true;

        // Attempt to parse the request type and request into one of our
        // known enum values, and then inspect them. (These will return None
        // if we get an unexpected numeric value.)
        const reqty = UsbDir.of_endpoint_addr(setup.request_type);
        const req = UsbSetupRequest.from_u8(setup.request);

        if (reqty == UsbDir.Out and req != null and req.? == UsbSetupRequest.SetAddress) {
            // The new address is in the bottom 8 bits of the setup
            // packet value field. Store it for use later.
            S.new_address = @intCast(u8, setup.value & 0xff);
            // The address will actually get set later, we have
            // to use address 0 to send a status response.
            usb_start_tx(
                &buffers.B,
                device_config.endpoints[EP0_IN_IDX], //EP0_IN_CFG,
                &.{}, // <- see, empty buffer
            );
            std.log.info("    SetAddress: {}", .{S.new_address.?});
        } else if (reqty == UsbDir.Out and req != null and req.? == UsbSetupRequest.SetConfiguration) {
            // We only have one configuration, and it doesn't really
            // mean anything to us -- more of a formality. All we do in
            // response to this is:
            S.configured = true;
            usb_start_tx(
                &buffers.B,
                device_config.endpoints[EP0_IN_IDX], //EP0_IN_CFG,
                &.{}, // <- see, empty buffer
            );
            std.log.info("    SetConfiguration", .{});
        } else if (reqty == UsbDir.Out) {
            // This is sort of a hack, but: if we get any other kind of
            // OUT, just acknowledge it with the same zero-length status
            // phase that we use for control transfers that we _do_
            // understand. This keeps the host from spinning forever
            // while we NAK.
            //
            // This behavior copied shamelessly from the C example.
            usb_start_tx(
                &buffers.B,
                device_config.endpoints[EP0_IN_IDX], // EP0_IN_CFG,
                &.{}, // <- see, empty buffer
            );
            std.log.info("    Just OUT", .{});
        } else if (reqty == UsbDir.In and req != null and req.? == UsbSetupRequest.GetDescriptor) {
            // Identify the requested descriptor type, which is in the
            // _top_ 8 bits of value.
            const descriptor_type = UsbDescType.from_u16(setup.value >> 8);
            std.log.info("    GetDescriptor: {}", .{setup.value >> 8});
            if (descriptor_type) |dt| {
                switch (dt) {
                    .Device => {
                        std.log.info("        Device", .{});
                        // TODO: this sure looks like a duplicate, but it's
                        // a duplicate that was present in the C
                        // implementation.
                        device_config.endpoints[EP0_IN_IDX].next_pid_1 = true;

                        const dc = device_config.device_descriptor.serialize();
                        _ = rom.memcpy(S.tmp[0..dc.len], &dc);

                        // Configure EP0 IN to send the device descriptor
                        // when it's next asked.
                        usb_start_tx(
                            &buffers.B,
                            device_config.endpoints[EP0_IN_IDX],
                            S.tmp[0..dc.len],
                        );
                    },
                    .Config => {
                        std.log.info("        Config", .{});
                        // Config descriptor requests are slightly unusual.
                        // We can respond with just our config descriptor,
                        // but we can _also_ append our interface and
                        // endpoint descriptors to the end, saving some
                        // round trips. We'll choose to do this if the
                        // number of bytes the host will accept (in the
                        // `length` field) is large enough.
                        var used: usize = 0;

                        const cd = device_config.config_descriptor.serialize();
                        _ = rom.memcpy(S.tmp[used .. used + cd.len], &cd);
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
                            _ = rom.memcpy(S.tmp[used .. used + id.len], &id);
                            used += id.len;

                            // Seems like the host does not bother asking for the
                            // hid descriptor so we'll just send it with the
                            // other descriptors.
                            if (device_config.hid_descriptor) |hid_desc| {
                                const hd = hid_desc.serialize();
                                _ = rom.memcpy(S.tmp[used .. used + hd.len], &hd);
                                used += hd.len;
                            }

                            for (device_config.endpoints[2..]) |ep| {
                                const ed = ep.descriptor.serialize();
                                _ = rom.memcpy(S.tmp[used .. used + ed.len], &ed);
                                used += ed.len;
                            }
                        }

                        // Set up EP0 IN to send the stuff we just composed.
                        usb_start_tx(
                            &buffers.B,
                            device_config.endpoints[EP0_IN_IDX],
                            S.tmp[0..used],
                        );
                    },
                    .String => {
                        std.log.info("        String", .{});
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
                                _ = rom.memcpy(S.tmp[2..len], s);

                                break :StringBlk S.tmp[0..len];
                            }
                        };
                        // Set up EP0 IN to send whichever thing we just
                        // decided on.
                        usb_start_tx(
                            &buffers.B,
                            device_config.endpoints[EP0_IN_IDX],
                            bytes,
                        );
                    },
                    .Interface => {
                        std.log.info("        Interface", .{});
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
                        std.log.info("        Endpoint", .{});
                        // Same deal as interface descriptors above.
                    },
                    .Hid => {
                        std.log.info("        HID", .{});
                        // TODO: return HID descriptor
                    },
                    .Report => {
                        std.log.info("        Report", .{});

                        if (device_config.report_descriptor) |report| {
                            usb_start_tx(
                                &buffers.B,
                                device_config.endpoints[EP0_IN_IDX],
                                report,
                            );
                        }
                    },
                    .Physical => {
                        std.log.info("        Physical", .{});
                        // Ignore for now
                    },
                }
            } else {
                // Unrecognized descriptor type. We should probably
                // indicate an error. Instead we'll just ignore it,
                // because this doesn't happen in practice.
            }
        } else if (reqty == UsbDir.In) {
            std.log.info("    Just IN", .{});
            // Other IN request. Ignore.
        } else {
            std.log.info("    This is unexpected", .{});
            // Unexpected request type or request bits. This can totally
            // happen (yay, hardware!) but is rare in practice. Ignore
            // it.
        }
    } // <-- END of setup request handling

    // Events on one or more buffers? (In practice, always one.)
    if (ints.BUFF_STATUS == 1) {
        std.log.info("buff status", .{});
        const orig_bufbits = peripherals.USBCTRL_REGS.BUFF_STATUS.raw;

        // Let's try being super tricky and iterating through set bits with
        // Cleverness(tm).
        // Become mutable so that I can clear bit as I handle 'em. Keep the
        // original around so I can use it to clear the register in a bit.
        var bufbits = orig_bufbits;

        while (bufbits != 0) {
            // Who's still outstanding? Find their bit index by counting how
            // many LSBs are zero.
            const lowbit_index = rom.ctz32(bufbits);
            std.log.info("    idx: {}", .{lowbit_index});
            // Remove their bit from our set.
            const lowbit = @intCast(u32, 1) << @intCast(u5, lowbit_index);
            bufbits ^= lowbit;

            // Here we exploit knowledge of the ordering of buffer control
            // registers in the peripheral. Each endpoint has a pair of
            // registers, so we can determine the endpoint number by:
            const epnum = @intCast(u8, lowbit_index >> 1);
            // Of the pair, the IN endpoint comes first, followed by OUT, so
            // we can get the direction by:
            const dir = if (lowbit_index & 1 == 0) UsbDir.In else UsbDir.Out;

            const ep_addr = dir.endpoint(epnum);
            // Process the buffer-done event.
            const data = DataBrk: {
                // Scan the device table to figure out which endpoint struct
                // corresponds to this address. We could use a smarter
                // method here, but in practice, the number of endpoints is
                // small so a linear scan doesn't kill us.
                var endpoint: ?*UsbEndpointConfiguration = null;
                for (device_config.endpoints) |ep| {
                    if (ep.descriptor.endpoint_address == ep_addr) {
                        endpoint = ep;
                        break;
                    }
                }
                // Buffer event for unknown EP?!
                if (endpoint == null) continue;
                // Read the buffer control register to check status.
                const bc = read_raw_buffer_control(endpoint.?.buffer_control_index);

                // We should only get here if we've been notified that
                // the buffer is ours again. This is indicated by the hw
                // _clearing_ the AVAILABLE bit.
                //
                // This ensures that we can return a shared reference to
                // the databuffer contents without races.
                //assert!(!bc.available_0().bit());
                if ((bc & (1 << 10)) == 1) continue; // just ignore, instead of assert

                // Cool. Checks out.

                // Get a pointer to the buffer in USB SRAM. This is the
                // buffer _contents_. See the safety comments below.
                const epbuffer = buffers.B.get(endpoint.?.data_buffer_index);

                // Get the actual length of the data, which may be less
                // than the buffer size.
                const len = @intCast(usize, bc & 0x3ff);

                // Make a byte slice pointing into USB SRAM.
                //
                // Safety: because we always use the same data buffer
                // with the same endpoint / buffer control register, and
                // because we have ensured that the available bit in the
                // buffer control register is not set, we can be
                // confident that we're not racing the hardware.
                //
                // You can _definitely_ abuse this code in a way that
                // produces multiple slices pointing to the same buffer,
                // but since these are shared references, that's
                // perfectly legal.
                break :DataBrk epbuffer[0..len];
            };
            std.log.info("    data: {any}", .{data});

            // Perform any required action on the data. For OUT, the `data`
            // will be whatever was sent by the host. For IN, it's a copy of
            // whatever we sent.
            switch (ep_addr) {
                EP0_IN_ADDR => {
                    std.log.info("    EP0_IN_ADDR", .{});
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
                    std.log.info("    ELSE, ep_addr: {}", .{ep_addr & 0x7f});
                    // Handle user provided endpoints.

                    // Find the corresponding endpoint. In practice this
                    // list shouldnt be very long so a linear search should be ok
                    var endpoint: ?*UsbEndpointConfiguration = null;
                    for (device_config.endpoints[2..]) |ep| {
                        if (ep.descriptor.endpoint_address == ep_addr) {
                            endpoint = ep;
                            break;
                        }
                    }

                    if (endpoint) |ep| {
                        // Invoke the callback (if the user provides one).
                        if (ep.callback) |callback| callback(device_config, data);
                    }
                },
            }
        }
        // Acknowledge all buffers, since we handled them all above.
        peripherals.USBCTRL_REGS.BUFF_STATUS.write_raw(orig_bufbits);
    } // <-- END of buf status handling

    // Has the host signaled a bus reset?
    if (ints.BUS_RESET == 1) {
        std.log.info("bus reset", .{});
        // Acknowledge by writing the write-one-to-clear status bit.
        peripherals.USBCTRL_REGS.SIE_STATUS.modify(.{ .BUS_RESET = 1 });

        // Reset our state.
        S.new_address = null;
        S.configured = false;
        S.started = false;
        peripherals.USBCTRL_REGS.ADDR_ENDP.modify(.{ .ADDRESS = 0 });
    }

    // If we have been configured but haven't reached this point yet, set up
    // our custom EP OUT's to receive whatever data the host wants to send.
    if (S.configured and !S.started) {
        // We can skip the first two endpoints because those are EP0_OUT and EP0_IN
        for (device_config.endpoints[2..]) |ep| {
            if (UsbDir.of_endpoint_addr(ep.descriptor.endpoint_address) == .Out) {
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
// Data Types
// +++++++++++++++++++++++++++++++++++++++++++++++++

/// Types of USB descriptor
pub const UsbDescType = enum(u8) {
    Device = 0x01,
    Config = 0x02,
    String = 0x03,
    Interface = 0x04,
    Endpoint = 0x05,
    /// HID descriptor
    Hid = 0x21,
    /// Report descriptor
    Report = 0x22,
    /// Physical descriptor
    Physical = 0x23,

    pub fn from_u16(v: u16) ?@This() {
        return switch (v) {
            1 => @This().Device,
            2 => @This().Config,
            3 => @This().String,
            4 => @This().Interface,
            5 => @This().Endpoint,
            0x21 => @This().Hid,
            0x22 => @This().Report,
            0x23 => @This().Physical,
            else => null,
        };
    }
};

/// Types of transfer that can be indicated by the `attributes` field on
/// `UsbEndpointDescriptor`.
pub const UsbTransferType = enum(u2) {
    Control = 0,
    Isochronous = 1,
    Bulk = 2,
    Interrupt = 3,
};

/// The types of USB SETUP requests that we understand.
pub const UsbSetupRequest = enum(u8) {
    /// Asks the device to send a certain descriptor back to the host. Always
    /// used on an IN request.
    GetDescriptor = 0x06,
    /// Notifies the device that it's being moved to a different address on the
    /// bus. Always an OUT.
    SetAddress = 0x05,
    /// Configures a device by choosing one of the options listed in its
    /// descriptors. Always an OUT.
    SetConfiguration = 0x09,

    pub fn from_u8(request: u8) ?@This() {
        return switch (request) {
            0x06 => UsbSetupRequest.GetDescriptor,
            0x05 => UsbSetupRequest.SetAddress,
            0x09 => UsbSetupRequest.SetConfiguration,
            else => null,
        };
    }
};

/// USB deals in two different transfer directions, called OUT (host-to-device)
/// and IN (device-to-host). In the vast majority of cases, OUT is represented
/// by a 0 byte, and IN by an `0x80` byte.
pub const UsbDir = enum(u8) {
    Out = 0,
    In = 0x80,

    pub inline fn endpoint(self: @This(), num: u8) u8 {
        return num | @enumToInt(self);
    }

    pub inline fn of_endpoint_addr(addr: u8) @This() {
        return if (addr & @enumToInt(@This().In) != 0) @This().In else @This().Out;
    }
};

/// Describes an endpoint within an interface
pub const UsbEndpointDescriptor = packed struct {
    /// Length of this struct, must be 7.
    length: u8,
    /// Type of this descriptor, must be `Endpoint`.
    descriptor_type: UsbDescType,
    /// Address of this endpoint, where the bottom 4 bits give the endpoint
    /// number (0..15) and the top bit distinguishes IN (1) from OUT (0).
    endpoint_address: u8,
    /// Endpoint attributes; the most relevant part is the bottom 2 bits, which
    /// control the transfer type using the values from `UsbTransferType`.
    attributes: u8,
    /// Maximum packet size this endpoint can accept/produce.
    max_packet_size: u16,
    /// Interval for polling interrupt/isochronous endpoints (which we don't
    /// currently support) in milliseconds.
    interval: u8,

    pub fn serialize(self: *const @This()) [7]u8 {
        var out: [7]u8 = undefined;
        out[0] = 7; // length
        out[1] = @enumToInt(self.descriptor_type);
        out[2] = self.endpoint_address;
        out[3] = self.attributes;
        out[4] = @intCast(u8, self.max_packet_size & 0xff);
        out[5] = @intCast(u8, (self.max_packet_size >> 8) & 0xff);
        out[6] = self.interval;
        return out;
    }
};

/// Description of an interface within a configuration.
pub const UsbInterfaceDescriptor = packed struct {
    /// Length of this structure, must be 9.
    length: u8,
    /// Type of this descriptor, must be `Interface`.
    descriptor_type: UsbDescType,
    /// ID of this interface.
    interface_number: u8,
    /// Allows a single `interface_number` to have several alternate interface
    /// settings, where each alternate increments this field. Normally there's
    /// only one, and `alternate_setting` is zero.
    alternate_setting: u8,
    /// Number of endpoint descriptors in this interface.
    num_endpoints: u8,
    /// Interface class code, distinguishing the type of interface.
    interface_class: u8,
    /// Interface subclass code, refining the class of interface.
    interface_subclass: u8,
    /// Protocol within the interface class/subclass.
    interface_protocol: u8,
    /// Index of interface name within string descriptor table.
    interface_s: u8,

    pub fn serialize(self: *const @This()) [9]u8 {
        var out: [9]u8 = undefined;
        out[0] = 9; // length
        out[1] = @enumToInt(self.descriptor_type);
        out[2] = self.interface_number;
        out[3] = self.alternate_setting;
        out[4] = self.num_endpoints;
        out[5] = self.interface_class;
        out[6] = self.interface_subclass;
        out[7] = self.interface_protocol;
        out[8] = self.interface_s;
        return out;
    }
};

/// Description of a single available device configuration.
pub const UsbConfigurationDescriptor = packed struct {
    /// Length of this structure, must be 9.
    length: u8,
    /// Type of this descriptor, must be `Config`.
    descriptor_type: UsbDescType,
    /// Total length of all descriptors in this configuration, concatenated.
    /// This will include this descriptor, plus at least one interface
    /// descriptor, plus each interface descriptor's endpoint descriptors.
    total_length: u16,
    /// Number of interface descriptors in this configuration.
    num_interfaces: u8,
    /// Number to use when requesting this configuration via a
    /// `SetConfiguration` request.
    configuration_value: u8,
    /// Index of this configuration's name in the string descriptor table.
    configuration_s: u8,
    /// Bit set of device attributes:
    ///
    /// - Bit 7 should be set (indicates that device can be bus powered in USB
    /// 1.0).
    /// - Bit 6 indicates that the device can be self-powered.
    /// - Bit 5 indicates that the device can signal remote wakeup of the host
    /// (like a keyboard).
    /// - The rest are reserved and should be zero.
    attributes: u8,
    /// Maximum device power consumption in units of 2mA.
    max_power: u8,

    pub fn serialize(self: *const @This()) [9]u8 {
        var out: [9]u8 = undefined;
        out[0] = 9; // length
        out[1] = @enumToInt(self.descriptor_type);
        out[2] = @intCast(u8, self.total_length & 0xff);
        out[3] = @intCast(u8, (self.total_length >> 8) & 0xff);
        out[4] = self.num_interfaces;
        out[5] = self.configuration_value;
        out[6] = self.configuration_s;
        out[7] = self.attributes;
        out[8] = self.max_power;
        return out;
    }
};

/// Describes a device. This is the most broad description in USB and is
/// typically the first thing the host asks for.
pub const UsbDeviceDescriptor = packed struct {
    /// Length of this structure, must be 18.
    length: u8,
    /// Type of this descriptor, must be `Device`.
    descriptor_type: UsbDescType,
    /// Version of the device descriptor / USB protocol, in binary-coded
    /// decimal. This is typically `0x01_10` for USB 1.1.
    bcd_usb: u16,
    /// Class of device, giving a broad functional area.
    device_class: u8,
    /// Subclass of device, refining the class.
    device_subclass: u8,
    /// Protocol within the subclass.
    device_protocol: u8,
    /// Maximum unit of data this device can move.
    max_packet_size0: u8,
    /// ID of product vendor.
    vendor: u16,
    /// ID of product.
    product: u16,
    /// Device version number, as BCD again.
    bcd_device: u16,
    /// Index of manufacturer name in string descriptor table.
    manufacturer_s: u8,
    /// Index of product name in string descriptor table.
    product_s: u8,
    /// Index of serial number in string descriptor table.
    serial_s: u8,
    /// Number of configurations supported by this device.
    num_configurations: u8,

    pub fn serialize(self: *const @This()) [18]u8 {
        var out: [18]u8 = undefined;
        out[0] = 18; // length
        out[1] = @enumToInt(self.descriptor_type);
        out[2] = @intCast(u8, self.bcd_usb & 0xff);
        out[3] = @intCast(u8, (self.bcd_usb >> 8) & 0xff);
        out[4] = self.device_class;
        out[5] = self.device_subclass;
        out[6] = self.device_protocol;
        out[7] = self.max_packet_size0;
        out[8] = @intCast(u8, self.vendor & 0xff);
        out[9] = @intCast(u8, (self.vendor >> 8) & 0xff);
        out[10] = @intCast(u8, self.product & 0xff);
        out[11] = @intCast(u8, (self.product >> 8) & 0xff);
        out[12] = @intCast(u8, self.bcd_device & 0xff);
        out[13] = @intCast(u8, (self.bcd_device >> 8) & 0xff);
        out[14] = self.manufacturer_s;
        out[15] = self.product_s;
        out[16] = self.serial_s;
        out[17] = self.num_configurations;
        return out;
    }
};

/// Layout of an 8-byte USB SETUP packet.
pub const UsbSetupPacket = packed struct {
    /// Request type; in practice, this is always either OUT (host-to-device) or
    /// IN (device-to-host), whose values are given in the `UsbDir` enum.
    request_type: u8,
    /// Request. Standard setup requests are in the `UsbSetupRequest` enum.
    /// Devices can extend this with additional types as long as they don't
    /// conflict.
    request: u8,
    /// A simple argument of up to 16 bits, specific to the request.
    value: u16,
    /// Not used in the requests we support.
    index: u16,
    /// If data will be transferred after this request (in the direction given
    /// by `request_type`), this gives the number of bytes (OUT) or maximum
    /// number of bytes (IN).
    length: u16,
};

// +++++++++++++++++++++++++++++++++++++++++++++++++
// Driver support stuctures
// +++++++++++++++++++++++++++++++++++++++++++++++++

pub const UsbEndpointConfiguration = struct {
    descriptor: *const UsbEndpointDescriptor,
    /// Index of this endpoint's control register in the `ep_control` array.
    ///
    /// TODO: this can be derived from the endpoint address, perhaps it should
    /// be.
    endpoint_control_index: ?usize,
    /// Index of this endpoint's buffer control register in the
    /// `ep_buffer_control` array.
    ///
    /// TODO this, too, can be derived.
    buffer_control_index: usize,

    /// Index of this endpoint's data buffer in the array of data buffers
    /// allocated from DPRAM. This can be arbitrary, and endpoints can even
    /// share buffers if you're careful.
    data_buffer_index: usize,

    /// Keeps track of which DATA PID (DATA0/DATA1) is expected on this endpoint
    /// next. If `true`, we're expecting `DATA1`, otherwise `DATA0`.
    next_pid_1: bool,

    /// Optional callback for custom OUT endpoints. This function will be called
    /// if the device receives data on the corresponding endpoint.
    callback: ?*const fn (dc: *UsbDeviceConfiguration, data: []const u8) void = null,
};

pub const UsbDeviceConfiguration = struct {
    device_descriptor: *const UsbDeviceDescriptor,
    interface_descriptor: *const UsbInterfaceDescriptor,
    config_descriptor: *const UsbConfigurationDescriptor,
    lang_descriptor: []const u8,
    descriptor_strings: []const []const u8,
    // TODO: group hid and report descriptors together...
    hid_descriptor: ?*const hid.HidDescriptor = null,
    report_descriptor: ?[]const u8 = null,

    endpoints: [4]*UsbEndpointConfiguration,
};

/// Buffer pointers, once they're prepared and initialized.
pub const Buffers = struct {
    /// Fixed EP0 Buffer0, defined by the hardware
    ep0_buffer0: [*]u8,
    /// Fixed EP0 Buffer1, defined by the hardware and NOT USED in this driver
    ep0_buffer1: [*]u8,
    /// /// Remaining buffer pool
    rest: [16][*]u8,

    /// Gets a buffer corresponding to a `data_buffer_index` in a
    /// `UsbEndpointConfiguration`.
    pub fn get(self: *@This(), i: usize) [*]u8 {
        return switch (i) {
            0 => self.ep0_buffer0,
            1 => self.ep0_buffer1,
            else => self.rest[i - 2],
        };
    }
};

// Handy constants for the endpoints we use here
pub const EP0_IN_ADDR: u8 = UsbDir.In.endpoint(0);
pub const EP0_OUT_ADDR: u8 = UsbDir.Out.endpoint(0);
const EP1_OUT_ADDR: u8 = UsbDir.Out.endpoint(1);
const EP1_IN_ADDR: u8 = UsbDir.In.endpoint(1);
const EP2_IN_ADDR: u8 = UsbDir.In.endpoint(2);

pub var EP0_OUT_CFG: UsbEndpointConfiguration = .{
    .descriptor = &UsbEndpointDescriptor{
        .length = @intCast(u8, @sizeOf(UsbEndpointDescriptor)),
        .descriptor_type = UsbDescType.Endpoint,
        .endpoint_address = EP0_OUT_ADDR,
        .attributes = @enumToInt(UsbTransferType.Control),
        .max_packet_size = 64,
        .interval = 0,
    },
    .endpoint_control_index = null,
    .buffer_control_index = 1,
    .data_buffer_index = 0,
    .next_pid_1 = false,
};

pub var EP0_IN_CFG: UsbEndpointConfiguration = .{
    .descriptor = &UsbEndpointDescriptor{
        .length = @intCast(u8, @sizeOf(UsbEndpointDescriptor)),
        .descriptor_type = UsbDescType.Endpoint,
        .endpoint_address = EP0_IN_ADDR,
        .attributes = @enumToInt(UsbTransferType.Control),
        .max_packet_size = 64,
        .interval = 0,
    },
    .endpoint_control_index = null,
    .buffer_control_index = 0,
    .data_buffer_index = 0,
    .next_pid_1 = false,
};

// +++++++++++++++++++++++++++++++++++++++++++++++++
// Utility functions
// +++++++++++++++++++++++++++++++++++++++++++++++++

/// Configures a given endpoint to send data (device-to-host, IN) when the host
/// next asks for it.
///
/// The contents of `buffer` will be _copied_ into USB SRAM, so you can
/// reuse `buffer` immediately after this returns. No need to wait for the
/// packet to be sent.
pub fn usb_start_tx(
    ep_buffers: *Buffers,
    ep: *UsbEndpointConfiguration,
    buffer: []const u8,
) void {
    // It is technically possible to support longer buffers but this demo
    // doesn't bother.
    // TODO: assert!(buffer.len() <= 64);
    // You should only be calling this on IN endpoints.
    // TODO: assert!(UsbDir::of_endpoint_addr(ep.descriptor.endpoint_address) == UsbDir::In);

    // Copy the given data into the corresponding ep buffer
    const epbuffer = ep_buffers.get(ep.data_buffer_index);
    _ = rom.memcpy(epbuffer[0..buffer.len], buffer);

    // Configure the IN:
    const np: u1 = if (ep.next_pid_1) 1 else 0;
    modify_buffer_control(ep.buffer_control_index, .{
        .PID_0 = np, // DATA0/1, depending
        .FULL_0 = 1, // We have put data in
        .AVAILABLE_0 = 1, // The data is for the computer to use now
        .LENGTH_0 = @intCast(u10, buffer.len), // There are this many bytes
    });

    ep.next_pid_1 = !ep.next_pid_1;
}

pub fn usb_start_rx(
    ep: *UsbEndpointConfiguration,
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

test "usb tests" {
    _ = hid;
}
