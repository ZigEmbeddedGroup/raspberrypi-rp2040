const microzig = @import("microzig");
const regs = microzig.chip.registers;

pub const pins = @import("hal/pins.zig");
pub const gpio = @import("hal/gpio.zig");
pub const clocks = @import("hal/clocks.zig");
pub const multicore = @import("hal/multicore.zig");
pub const time = @import("hal/time.zig");
pub const uart = @import("hal/uart.zig");
pub const pwm = @import("hal/pwm.zig");

pub const clock_config = clocks.GlobalConfiguration.init(.{
    .ref = .{ .source = .src_xosc },
    .sys = .{
        .source = .pll_sys,
        .freq = 125_000_000,
    },
    .peri = .{ .source = .clk_sys },
    .usb = .{ .source = .pll_usb },
    .adc = .{ .source = .pll_usb },
    .rtc = .{ .source = .pll_usb },
});

pub fn init() void {
    clock_config.apply();
}

pub fn getCpuId() u32 {
    return regs.SIO.CPUID.*;
}
