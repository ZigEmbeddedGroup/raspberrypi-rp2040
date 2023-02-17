const std = @import("std");
const EnumField = std.builtin.Type.EnumField;

const microzig = @import("microzig");
const RESETS = microzig.chip.peripherals.RESETS;
const Mask = @TypeOf(RESETS.RESET).underlying_type;

pub const Module = enum {
    adc,
    busctrl,
    dma,
    i2c0,
    i2c1,
    io_bank0,
    io_qspi,
    jtag,
    pads_bank0,
    pads_qspi,
    pio0,
    pio1,
    pll_sys,
    pll_usb,
    pwm,
    rtc,
    spi0,
    spi1,
    syscfg,
    sysinfo,
    tbman,
    timer,
    uart0,
    uart1,
    usbctrl,
};

pub inline fn reset(comptime modules: []const Module) void {
    comptime var mask = std.mem.zeroes(Mask);

    inline for (modules) |module|
        @field(mask, @tagName(module)) = 1;

    const raw_mask = @bitCast(u32, mask);

    RESETS.RESET.raw = raw_mask;
    RESETS.RESET.raw = 0;

    while ((RESETS.RESET_DONE.raw & raw_mask) != raw_mask) {}
}
