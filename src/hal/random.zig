//! Random number generator (RNG) using the ROSC
//!
//! _WARNING_: This does not meet the requirements of randomness
//! for security systems because it can be compromised, but it
//! may be useful in less critical applications.

const microzig = @import("microzig");
const peripherals = microzig.chip.peripherals;

/// Fill the buffer with up to buffer.len random bytes
///
/// rand uses the RANDOMBIT register of the ROSC as its source, i. e.,
/// the system clocks _MUST_ run from the XOSC and/or PLLs.
///
/// _WARNING_: This function does not meet the requirements of randomness
/// for security systems because it can be compromised, but it may be useful
/// in less critical applications.
pub fn rand(buffer: []u8) void {
    const rosc_state = peripherals.ROSC.CTRL.read().ENABLE.value;
    // We'll just always enable the ROSC and then set it to the saved state at the end
    peripherals.ROSC.CTRL.modify(.{ .ENABLE = .{ .value = .ENABLE } });
    defer peripherals.ROSC.CTRL.modify(.{ .ENABLE = .{ .value = rosc_state } });

    var i: usize = 0;
    while (i < buffer.len) : (i += 1) {
        // We poll RANDOMBIT eight times per cycle to build a random byte
        var r: u8 = @intCast(u8, peripherals.ROSC.RANDOMBIT.read().RANDOMBIT);
        r = (r << 1) | @intCast(u8, peripherals.ROSC.RANDOMBIT.read().RANDOMBIT);
        r = (r << 1) | @intCast(u8, peripherals.ROSC.RANDOMBIT.read().RANDOMBIT);
        r = (r << 1) | @intCast(u8, peripherals.ROSC.RANDOMBIT.read().RANDOMBIT);
        r = (r << 1) | @intCast(u8, peripherals.ROSC.RANDOMBIT.read().RANDOMBIT);
        r = (r << 1) | @intCast(u8, peripherals.ROSC.RANDOMBIT.read().RANDOMBIT);
        r = (r << 1) | @intCast(u8, peripherals.ROSC.RANDOMBIT.read().RANDOMBIT);
        r = (r << 1) | @intCast(u8, peripherals.ROSC.RANDOMBIT.read().RANDOMBIT);
        buffer[i] = r;
    }
}
