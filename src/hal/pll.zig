const std = @import("std");
const microzig = @import("microzig");
const assert = std.debug.assert;

const regs = microzig.chip.registers;
const xosc_freq = microzig.board.xosc_freq;

pub const Configuration = struct {
    refdiv: u6,
    vco_freq: u32,
    postdiv1: u3,
    postdiv2: u3,
};

pub const sys = PLL{
    .which = .sys,
    .cs = regs.PLL_SYS.CS,
    .pwr = regs.PLL_SYS.PWR,
    .fbdiv_int = regs.PLL_SYS.FBDIV_INT,
    .prim = regs.PLL_SYS.PRIM,
};

pub const usb = PLL{
    .which = .usb,
    .cs = @ptrCast(CsReg, regs.PLL_USB.CS),
    .pwr = @ptrCast(PwrReg, regs.PLL_USB.PWR),
    .fbdiv_int = @ptrCast(FbdivIntReg, regs.PLL_USB.FBDIV_INT),
    .prim = @ptrCast(PrimReg, regs.PLL_USB.PRIM),
};

const CsReg = @TypeOf(regs.PLL_SYS.CS);
const PwrReg = @TypeOf(regs.PLL_SYS.PWR);
const FbdivIntReg = @TypeOf(regs.PLL_SYS.FBDIV_INT);
const PrimReg = @TypeOf(regs.PLL_SYS.PRIM);

pub const PLL = struct {
    which: enum {
        sys,
        usb,
    },
    cs: CsReg,
    pwr: PwrReg,
    fbdiv_int: FbdivIntReg,
    prim: PrimReg,

    pub fn isLocked(pll: PLL) bool {
        return pll.cs.read().LOCK == 1;
    }

    pub fn reset(pll: PLL) void {
        switch (pll.which) {
            .sys => {
                regs.RESETS.RESET.modify(.{ .pll_sys = 1 });
                regs.RESETS.RESET.modify(.{ .pll_sys = 0 });
                while (regs.RESETS.RESET_DONE.read().pll_sys != 1) {}
            },
            .usb => {
                regs.RESETS.RESET.modify(.{ .pll_usb = 1 });
                regs.RESETS.RESET.modify(.{ .pll_usb = 0 });
                while (regs.RESETS.RESET_DONE.read().pll_usb != 1) {}
            },
        }
    }

    pub fn apply(pll: PLL, comptime config: Configuration) void {
        const ref_freq = xosc_freq / @as(u32, config.refdiv);
        const fbdiv = @intCast(u12, config.vco_freq / ref_freq);

        assert(fbdiv >= 16 and fbdiv <= 320);
        assert(config.postdiv1 >= 1 and config.postdiv1 <= 7);
        assert(config.postdiv2 >= 1 and config.postdiv2 <= 7);
        assert(config.postdiv2 <= config.postdiv1);
        assert(ref_freq <= config.vco_freq / 16);

        // 1. program reference clock divider
        // 2. program feedback divider
        // 3. turn on the main power and vco
        // 4. wait for vco to lock
        // 5. set up post dividers and turn them on

        // do not bother a PLL which is already configured
        if (pll.isLocked() and
            config.refdiv == pll.cs.read().REFDIV and
            fbdiv == pll.fbdiv_int.read() and
            config.postdiv1 == pll.prim.read().POSTDIV1 and
            config.postdiv2 == pll.prim.read().POSTDIV2)
        {
            return;
        }

        pll.reset();

        // load vco related dividers
        pll.cs.modify(.{ .REFDIV = config.refdiv });
        pll.fbdiv_int.modify(fbdiv);

        // turn on PLL
        pll.pwr.modify(.{ .PD = 0, .VCOPD = 0 });

        // wait for PLL to lock
        while (!pll.isLocked()) {}

        pll.prim.modify(.{
            .POSTDIV1 = config.postdiv1,
            .POSTDIV2 = config.postdiv2,
        });

        pll.pwr.modify(.{ .POSTDIVPD = 0 });
    }
};
