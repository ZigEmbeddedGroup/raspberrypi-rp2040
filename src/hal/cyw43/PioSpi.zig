const std = @import("std");
const rp2040 = @import("microzig").hal;
const gpio = rp2040.gpio;
const Pio = rp2040.pio.Pio;
const StateMachine = rp2040.pio.StateMachine;

pub const PioSpi = @This();

cs: gpio.Pin,
pio: Pio,
sm: StateMachine,

const logger = std.log.scoped(.pio_spi);

const program = blk: {
    @setEvalBranchQuota(5000);
    break :blk rp2040.pio.assemble(
        \\ .program prog
        \\ .side_set 1
        \\ .wrap_target
        \\ write_begin:
        \\ out pins, 1             side 0
        \\ jmp x-- write_begin     side 1
        \\ set pindirs, 0          side 0
        \\ nop                     side 0
        \\ read_begin:
        \\ in pins, 1              side 1
        \\ jmp y-- read_begin      side 0
        \\ wait 1 pin 0            side 0
        \\ irq 0                   side 0
        \\ .wrap
    , .{}).get_program_by_name("prog");
};

pub fn init(pio: Pio, sm: StateMachine, cs: gpio.Pin, dio: gpio.Pin, clk: gpio.Pin) !PioSpi {
    pio.gpio_init(dio);
    pio.set_input_sync_bypass(dio, true);
    dio.set_schmitt(true);
    dio.set_pull(null);
    dio.set_drive_strength(.@"12mA");
    dio.set_slew_rate(.fast);

    pio.gpio_init(clk);
    clk.set_drive_strength(.@"12mA");
    clk.set_slew_rate(.fast);

    const offset = try pio.add_program(program);
    std.debug.assert(offset == 0);

    pio.sm_init(sm, offset, .{
        .clkdiv = .{ .int = 5 },
        .shift = .{
            .autopush = true,
            .autopull = true,
            .in_shiftdir = .left,
            .out_shiftdir = .left,
        },
        .pin_mappings = .{
            .out = .{
                .base = @intFromEnum(dio),
                .count = 1,
            },
            .set = .{
                .base = @intFromEnum(dio),
                .count = 1,
            },
            .side_set = .{
                .base = @intFromEnum(clk),
                .count = 1,
            },
            .in_base = @intFromEnum(dio),
        },
        .exec = .{
            .wrap = program.wrap.?,
            .wrap_target = program.wrap_target.?,
            .side_set_optional = program.side_set.?.optional,
            .side_pindir = program.side_set.?.pindir,
        },
    });

    pio.sm_set_pindirs(sm, &.{ clk, dio }, .out);
    pio.sm_set_pins(sm, &.{ clk, dio }, 0);

    pio.sm_set_enabled(sm, true);

    return .{
        .cs = cs,
        .pio = pio,
        .sm = sm,
    };
}

pub fn read(self: *PioSpi, out: []const u32, in: []u32) !void {
    self.cs.put(0);
    defer self.cs.put(1);

    const write_bits = out.len - 1;
    const read_bits = in.len * 32 + 32 - 1;

    logger.debug("write={}({}) read={}({})", .{
        std.fmt.fmtSliceHexLower(std.mem.sliceAsBytes(out)),
        write_bits,
        std.fmt.fmtSliceHexLower(std.mem.sliceAsBytes(in)),
        read_bits,
    });

    self.pio.sm_set_enabled(self.sm, false);

    self.pio.sm_set_pindir(self.sm, .out);
    self.pio.sm_set_x(self.sm, write_bits);
    self.pio.sm_set_y(self.sm, read_bits);
    self.pio.sm_jmp(self.sm, program.wrap_target.?);

    self.pio.sm_set_enabled(self.sm, true);

    for (out) |v| {
        self.pio.sm_blocking_write(self.sm, v);
    }

    for (in) |*v| {
        v.* = self.pio.sm_blocking_read(self.sm);
    }

    const status = self.pio.sm_blocking_read(self.sm);

    logger.debug("status = {}", .{status});
}

pub fn write(self: *PioSpi, out: []const u32) !void {
    self.cs.put(0);

    const write_bits = out.len * 32 - 1;
    const read_bits = 32 - 1;

    logger.debug("write={}({})", .{
        std.fmt.fmtSliceHexLower(std.mem.sliceAsBytes(out)),
        write_bits,
    });

    self.pio.sm_set_enabled(self.sm, false);

    self.pio.sm_set_pindir(self.sm, .out);
    self.pio.sm_set_x(self.sm, write_bits);
    self.pio.sm_set_y(self.sm, read_bits);
    self.pio.sm_jmp(self.sm, program.wrap_target.?);

    self.pio.sm_set_enabled(self.sm, true);

    for (out) |v| {
        self.pio.sm_blocking_write(self.sm, v);
    }
    const status = self.pio.sm_blocking_read(self.sm);

    logger.debug("status = {}", .{status});
    self.cs.put(1);
}
