//! Random number generator (RNG) using the Ascon CSPRNG

const std = @import("std");
const assert = std.debug.assert;
const Random = std.rand.Random;

const microzig = @import("microzig");
const peripherals = microzig.chip.peripherals;

/// Wrapper around the Ascon CSPRNG
///
/// The rng collects its entropy from the ROSC.
///
/// ## Usage
///
/// ```zig
/// var ascon = Ascon.init();
/// var rng = ascon.random();
/// ```
pub const Ascon = struct {
    state: std.rand.Ascon,

    const secret_seed_length = std.rand.Ascon.secret_seed_length;

    pub fn init() @This() {
        // The init function includes just a assertion. One
        // must make sure the processor is in the right state
        // to create random numbers using the ROSC.
        rand_init();

        // We seed the RNG with random bits from the ROSC...
        var b: [secret_seed_length]u8 = undefined;
        var i: usize = 0;
        blk: while (true) {
            var n = rand_rosc();
            for (0..4) |_| {
                if (i >= secret_seed_length) {
                    break :blk;
                }

                b[i] = @intCast(n & 0xff);
                n >>= 8;
                i += 1;
            }
        }

        return @This(){ .state = std.rand.Ascon.init(b) };
    }

    /// Returns a `std.rand.Random` structure backed by the current RNG
    pub fn random(self: *@This()) Random {
        return Random.init(self, fill);
    }

    /// Fills the buffer with random bytes
    pub fn fill(self: *@This(), buf: []u8) void {
        // fill the buffer with random bytes
        self.state.fill(buf);

        // then mix new entropy from the rosc into the state
        const n = rand_rosc();
        self.state.addEntropy(&std.mem.toBytes(n));
    }
};

pub fn rand_init() void {
    // Ensure that the system clocks run from the XOSC and/or PLLs
    const ref_src = peripherals.CLOCKS.CLK_REF_CTRL.read().SRC.value;
    const sys_clk_src = peripherals.CLOCKS.CLK_SYS_CTRL.read().SRC.value;
    const aux_src = peripherals.CLOCKS.CLK_SYS_CTRL.read().AUXSRC.value;
    assert((ref_src != .rosc_clksrc_ph and sys_clk_src == .clk_ref) or
        (sys_clk_src == .clksrc_clk_sys_aux and aux_src != .rosc_clksrc));
}

/// Create a random `u32` value.
///
/// The function uses the RANDOMBIT register of the ROSC as its source, i. e.,
/// the system clocks _MUST_ run from the XOSC and/or PLLs.
pub fn rand_rosc() u32 {
    const rosc_state = peripherals.ROSC.CTRL.read().ENABLE.value;
    // Enable the ROSC so it generates random bits for us
    peripherals.ROSC.CTRL.modify(.{ .ENABLE = .{ .value = .ENABLE } });
    defer peripherals.ROSC.CTRL.modify(.{ .ENABLE = .{ .value = rosc_state } });

    var r: u32 = 0;
    for (0..32) |_| {
        var r1: u32 = 0;
        var r2: u32 = 0;
        while (true) {
            r1 = @as(u32, @intCast(peripherals.ROSC.RANDOMBIT.read().RANDOMBIT));
            // insert delay
            asm volatile (
                \\nop
                \\nop
                \\nop
                \\nop
                \\nop
                \\nop
                \\nop
                \\nop
            );
            r2 = @as(u32, @intCast(peripherals.ROSC.RANDOMBIT.read().RANDOMBIT));
            if (r1 != r2) break;
        }
        r = (r << 1) | r1; // r1 is just one bit so we can append it to the end
    }
    return r;
}
