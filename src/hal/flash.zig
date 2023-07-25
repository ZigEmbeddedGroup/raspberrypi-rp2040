const std = @import("std");

const rom = @import("rom.zig");

pub const Command = enum(u8) {
    block_erase = 0xd8,
    ruid_cmd = 0x4b,
};

pub const PAGE_SIZE = 256;
pub const SECTOR_SIZE = 4096;
pub const BLOCK_SIZE = 65536;

/// Bus reads to a 16MB memory window start at this address
pub const XIP_BASE = 0x10000000;

pub const boot2 = struct {
    /// Size of the second stage bootloader in words
    const BOOT2_SIZE_WORDS = 64;

    // Buffer for the second stage bootloader
    // Calling into boot2 works because the code will
    // use the value inside lr as return address if
    // available.
    var copyout: [BOOT2_SIZE_WORDS]u32 = undefined;
    var copyout_valid: bool = false;

    /// Copy the 2nd stage bootloader into memory
    pub export fn flash_init() linksection(".time_critical") void {
        if (copyout_valid) return;
        const bootloader = @as([*]u32, @ptrFromInt(XIP_BASE));
        var i: usize = 0;
        while (i < BOOT2_SIZE_WORDS) : (i += 1) {
            copyout[i] = bootloader[i];
        }

        // No flash access after this point
        compiler_memory_barrier();

        copyout_valid = true;
    }

    pub export fn flash_enable_xip() linksection(".time_critical") void {
        // Because the copyout symbol is not of type `%function`
        // we usually get a linker warning. To prevent the warning
        // we trick the linker by using a register with the blx
        // instruction. It's important to note that we set the
        // LSB of the address to 1, to prevent the  processor from
        // switching back into arm mode, which would lead to a exception.
        std.mem.doNotOptimizeAway(asm volatile (
            \\adds r0, #1
            \\blx r0
            :
            : [copyout] "{r0}" (@ptrToInt(&copyout)),
            : "r0", "lr"
        ));
    }
};

pub fn erase(offset: u32, count: u32) void {
    range_erase(offset, count);
}

/// Erase count bytes starting at offset (offset from start of flash)
///
/// The offset must be aligned to a 4096-byte sector, and count must
/// be a multiple of 4096 bytes!
pub export fn range_erase(offset: u32, count: u32) linksection(".time_critical") void {
    std.debug.assert(offset & (SECTOR_SIZE - 1) == 0); // Is correctly aligned?
    std.debug.assert(count & (SECTOR_SIZE - 1) == 0); // Is a multiple of the sector size?

    boot2.flash_init();

    // No flash access after this point
    compiler_memory_barrier();

    rom.connect_internal_flash()();
    rom.flash_exit_xip()();
    rom.flash_range_erase()(offset, count, BLOCK_SIZE, @intFromEnum(Command.block_erase));
    rom.flash_flush_cache()();

    boot2.flash_enable_xip();
}

pub fn program(offset: u32, data: []const u8) void {
    range_program(offset, data.ptr, data.len);
}

/// Program data to flash starting at offset (offset from the start of flash)
///
/// The offset must be aligned to a 256-byte boundary, and the length of data
/// must be a multiple of 256!
pub export fn range_program(offset: u32, data: [*]const u8, data_len: usize) linksection(".time_critical") void {
    std.debug.assert(offset & (PAGE_SIZE - 1) == 0); // Is correctly aligned?
    std.debug.assert(data_len & (PAGE_SIZE - 1) == 0); // Is a multiple of the page size?

    boot2.flash_init();

    // No flash access after this point
    compiler_memory_barrier();

    rom.connect_internal_flash()();
    rom.flash_exit_xip()();
    rom.flash_range_program()(offset, data, data_len);
    rom.flash_flush_cache()();

    boot2.flash_enable_xip();
}

/// The compiler will not move the load from some_other_memory_location
/// above the memory barrier (which it otherwise might - even above the
/// memory store!). This is achieved using the memory clobber which declares
/// that the assembly writes to arbitrary undeclared memory locations.
pub inline fn compiler_memory_barrier() void {
    asm volatile ("" ::: "memory");
}
