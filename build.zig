const std = @import("std");
const Builder = std.build.Builder;
const Pkg = std.build.Pkg;
const comptimePrint = std.fmt.comptimePrint;
const LazyPath = std.build.LazyPath;

const microzig = @import("microzig");

pub const chips = @import("src/chips.zig");
pub const boards = @import("src/boards.zig");

const linkerscript_path = root() ++ "rp2040.ld";

pub const BuildOptions = struct {
    optimize: std.builtin.OptimizeMode,
};

pub const PicoExecutableOptions = struct {
    name: []const u8,
    source_file: LazyPath,
    optimize: std.builtin.OptimizeMode = .Debug,
};

pub fn addPiPicoExecutable(
    builder: *Builder,
    opts: PicoExecutableOptions,
) *microzig.EmbeddedExecutable {
    return microzig.addEmbeddedExecutable(builder, .{
        .name = opts.name,
        .source_file = opts.source_file,
        .backing = .{ .board = boards.raspberry_pi_pico },
        .optimize = opts.optimize,
        .linkerscript_source_file = .{ .path = linkerscript_path },
    });
}

const examples: []const []const u8 = &.{
    "adc",
    "blinky",
    "blinky_core1",
    "gpio_clk",
    "i2c_bus_scan",
    "pwm",
    "spi_master",
    "uart",
    "squarewave",
    "flash_program",
    "usb_device",
    "usb_hid",
    "ws2812",
    "random",
};

pub fn build(b: *Builder) !void {
    const optimize = b.standardOptimizeOption(.{});

    const args_dep = b.dependency("args", .{});

    const pio_tests = b.addTest(.{
        .root_source_file = .{
            .path = "src/hal.zig",
        },
        .optimize = optimize,
    });
    pio_tests.addIncludePath(.{ .path = "src/hal/pio/assembler" });

    const test_step = b.step("test", "run unit tests");
    test_step.dependOn(&b.addRunArtifact(pio_tests).step);

    const flash_tool = b.addExecutable(.{
        .name = "rp2040-flash",
        .optimize = .Debug,
        .target = .{},
        .root_source_file = .{ .path = "tools/rp2040-flash.zig" },
    });
    flash_tool.addModule("args", args_dep.module("args"));
    b.installArtifact(flash_tool);

    const ci_step = b.step("ci", "Build all example programs and run tests");
    ci_step.dependOn(test_step);

    // examples
    for (examples) |example| {
        const example_exe = addPiPicoExecutable(b, .{
            .name = b.fmt("example.{s}", .{example}),
            .source_file = .{ .path = b.fmt("examples/{s}.zig", .{example}) },
            .optimize = optimize,
        });
        example_exe.installArtifact(b);
        ci_step.dependOn(&example_exe.inner.step);
    }
}

fn root() []const u8 {
    return comptime (std.fs.path.dirname(@src().file) orelse ".") ++ "/";
}
