//! Random number generator (RNG) using the ROSC
//!
//! _WARNING_: This does not meet the requirements of randomness
//! for security systems because it can be compromised, but it
//! may be useful in less critical applications.

const std = @import("std");
const assert = std.debug.assert;
const Random = std.rand.Random;

const microzig = @import("microzig");
const peripherals = microzig.chip.peripherals;

/// RNG backed by the ROSC
pub const RoscRng = struct {
    /// Initialize the RNG by enabling the ROSC
    ///
    /// Fails if the system clocks don't run from the XOSC and/or PLLs.
    pub fn init() @This() {
        // Ensure that the system clocks run from the XOSC and/or PLLs
        const ref_src = peripherals.CLOCKS.CLK_REF_CTRL.read().SRC.value;
        const sys_clk_src = peripherals.CLOCKS.CLK_SYS_CTRL.read().SRC.value;
        const aux_src = peripherals.CLOCKS.CLK_SYS_CTRL.read().AUXSRC.value;
        assert((ref_src != .rosc_clksrc_ph and sys_clk_src == .clk_ref) or
            (sys_clk_src == .clksrc_clk_sys_aux and aux_src != .rosc_clksrc));

        // Enable the ROSC so it generates random bits for us
        peripherals.ROSC.CTRL.modify(.{ .ENABLE = .{ .value = .ENABLE } });

        return @This(){};
    }

    /// Returns a `std.rand.Random` structure backed by the current RNG
    pub fn random(self: *@This()) Random {
        return Random.init(@alignCast(1, self), fill);
    }

    /// Fill the buffer with up to buffer.len random bytes
    ///
    /// rand uses the RANDOMBIT register of the ROSC as its source, i. e.,
    /// the system clocks _MUST_ run from the XOSC and/or PLLs.
    ///
    /// _WARNING_: This function does not meet the requirements of randomness
    /// for security systems because it can be compromised, but it may be useful
    /// in less critical applications.
    pub fn fill(self: *@This(), buffer: []u8) void {
        _ = self;
        var i: usize = 0;
        while (i < buffer.len) : (i += 1) {
            // We poll RANDOMBIT eight times per cycle to build a random byte
            var r: u8 = @intCast(u8, peripherals.ROSC.RANDOMBIT.read().RANDOMBIT);
            var j: usize = 0;
            while (j < 7) : (j += 1) {
                r = (r << 1) | @intCast(u8, peripherals.ROSC.RANDOMBIT.read().RANDOMBIT);
            }
            buffer[i] = r;
        }
    }
};
