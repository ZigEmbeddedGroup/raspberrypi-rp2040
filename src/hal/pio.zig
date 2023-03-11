//! A PIO instance can load a single `Bytecode`, it has to be loaded into memory
const std = @import("std");

const microzig = @import("../../../deps/microzig/build.zig");
const PIO = microzig.chip.types.PIO0;
const PIO0 = microzig.chip.peripherals.PIO0;
const PIO1 = microzig.chip.peripherals.PIO1;

pub const Join = enum {
    none,
    rx,
    tx,
};

pub const Status = enum {
    rx_lessthan,
    tx_lessthan,
};

pub const Configuration = struct {
    pin: u32,
};

pub const Pio = enum {
    pio0,
    pio1,

    fn get_regs(self: Pio) *PIO {
        return switch (self) {
            .pio0 => PIO0,
            .pio1 => PIO1,
        };
    }

    pub fn load(self: Pio, bytecode: Bytecode) !void {
        _ = self;
        _ = bytecode;
    }

    // state machine configuration helpers:
    //
    // - set_out_pins
    // - set_set_pins
    // - set_in_pins
    // - set_sideset_pins
    // - set_sideset
    // - set_clkdiv_int_frac
    // - calculate_clkdiv_from_float
    // - set_clkdiv
    // - set_wrap
    // - set_jmp_pin
    // - set_in_shift
    // - set_out_shift
    // - set_fifo_join
    // - set_out_special
    // - set_mov_status
    //
    // PIO:
    //
    // - can_add_program
    // - add_program_at_offset
    // - add_program
    // - remove_program
    // - clear_instruction_memory
    // -
    //
};

// The assembler

pub const Define = struct {
    name: []const u8,
    value: Value,

    pub const Value = union(enum) {
        integer: u32,
        boolean: bool,
        // TODO: more
    };
};

pub const Program = struct {
    name: []const u8,
    defines: []const Define,
    origin: ?u5,
    side_set: ?u8,
    wrap_target: u8,
    wrap: u8,
    instructions: std.BoundedArray(u16, 32),
};

//CompilationUnit?
pub const Bytecode = struct {
    defines: []const Define,
    programs: []const Program,
};

pub const Parser = struct {
    data: []const u8,
    index: usize,

    pub const Token = union(enum) {
        side_set: u8,
        wrap_target: void,
        wrap: void,
        label: []const u8,
        program: []const u8,
        instruction: Instruction,

        pub const Instruction = struct {
            side_set: ?u8,
            delay: ?u8,
            payload: Payload,
            pub const Payload = union(enum) {
                jmp: Jmp,
                wait: Wait,
                in: In,
                out: Out,
                push: PushPull,
                pull: PushPull,
                mov: Mov,
                irq: Irq,
                set: Set,
            };

            pub const Jmp = struct {
                condition: enum(u3) {
                    Empty = 0b000,
                    NotX = 0b001, // !X
                    XDecrement = 0b010, // X--
                    NotY = 0b011, // !Y
                    YDecrement = 0b100, // Y--
                    XNotEqY = 0b101, //X!=Y
                    Pin = 0b110, // PIN
                    NotOSRE = 0b111, // !OSRE
                },
                label: []const u8,
            };

            pub const Wait = struct {
                polarity: enum(u1) {
                    low = 0,
                    high = 1,
                },
                source: enum(u2) {
                    Gpio = 0b00,
                    Pin = 0b01,
                    Irq = 0b10,
                    // reseved
                    _,
                },
                number: u5,
            };

            pub const In = struct {
                source: enum(u3) {
                    Pins = 0b00,
                    X = 0b001,
                    Y = 0b010,
                    Null = 0b011,
                    // reserved
                    // reserved
                    Isr = 0b110,
                    Osr = 0b111,
                    _,
                },
                bit_count: u5,
            };

            pub const Out = struct {
                destination: enum(u3) {
                    Pins = 0b000,
                    X = 0b001,
                    Y = 0b010,
                    Null = 0b011,
                    PinDirs = 0b100,
                    Pc = 0b101,
                    Isr = 0b110,
                    Exec = 0b111,
                },
                bit_count: u5,
            };

            pub const PushPull = struct {
                if_full: bool,
                block: bool,
            };

            pub const Mov = struct {
                destination: enum(u3) {
                    Pins = 0b000,
                    X = 0b001,
                    Y = 0b010,
                    // reserved
                    Exec = 0b100,
                    Pc = 0b101,
                    Isr = 0b110,
                    Osr = 0b111,
                    _,
                },
                operation: enum(u2) {
                    None = 0b00,
                    Invert = 0b01,
                    BitReverse = 0b10,
                    // reserved
                    _,
                },
                source: enum(u3) {
                    Pins = 0b00,
                    X = 0b001,
                    Y = 0b010,
                    Null = 0b011,
                    // reserved
                    Status = 0b101,
                    Isr = 0b110,
                    Osr = 0b111,
                    _,
                },
            };
            pub const Irq = struct {
                clear: bool,
                wait: bool,
                index: u5,
            };

            pub const Set = struct {
                destination: enum(u3) {
                    Pins = 0b000,
                    X = 0b001,
                    Y = 0b010,
                    // reserved
                    PinDirs = 0b100,
                    // reserved
                    // reserved
                    // reserved
                    _,
                },
                data: u5,
            };
        };

        pub const Mnemonic = enum {
            jmp,
            wait,
            in,
            out,
            push,
            pull,
            mov,
            irq,
            set,
            nop,
        };
    };

    pub fn init(asm_: []const u8) Parser {
        return Parser{ .data = asm_, .index = 0 };
    }

    pub fn parse_macro(self: *Parser) !?[]const u8 {
        const start = self.index;
        const end = blk: while (true) {
            if (self.index >= self.data.len)
                break self.index;
            switch (self.get_character()) {
                '\n' => break :blk self.index,
                else => {},
            }
            try self.consume(1);
        };
        return self.data[start..end];
    }

    pub fn next(self: *Parser) !?Token {
        if (self.index >= self.data.len) return null;

        _ = try self.skip_whitespaces();
        _ = try self.skip_newlines();
        _ = try self.skip_whitespaces();
        std.log.err("done skipping", .{});

        if (self.get_character() == '.') {
            try self.consume(1);
            const macro = (try self.parse_macro()).?;
            var tokenizer = std.mem.tokenize(u8, macro, " ");
            const macro_name = tokenizer.next().?;

            return if (std.mem.eql(u8, macro_name, "program")) blk: {
                const name = tokenizer.next().?;
                std.log.err("program found: {s}", .{name});
                break :blk Token{ .program = name };
            } else if (std.mem.eql(u8, macro_name, "side_set")) blk: {
                var number = try std.fmt.parseInt(u8, tokenizer.next().?, 10);
                break :blk Token{ .side_set = number };
            } else if (std.mem.eql(u8, macro_name, "wrap_target"))
                Token{ .wrap_target = {} }
            else if (std.mem.eql(u8, macro_name, "wrap"))
                Token{ .wrap = {} }
            else if (std.mem.eql(u8, macro_name, "lang_opt"))
                // TODO: parse this
                try self.next()
            else
                @panic("unsupported macro");
            // TODO: .word
        }

        const identifier = (try self.parse_identifier()).?; // TODO: or mnemonic
        if (self.get_character() == ':') {
            try self.consume(1);
            return Token{ .label = identifier };
        }

        if (std.meta.stringToEnum(Token.Mnemonic, identifier)) |mnemonic| {
            try self.skip_whitespaces();

            var instruction: Token.Instruction = undefined;
            switch (mnemonic) {
                // jmp ( <cond> ) <target>
                .jmp => {
                    const negation: bool = if (self.get_character() == '!') blk: {
                        try self.consume(1);
                        break :blk true;
                    } else false;

                    const register = try self.parse_register();
                    try self.skip_whitespaces();

                    const label = (try self.parse_identifier()).?;
                    try self.skip_whitespaces();

                    _ = negation;
                    _ = register;
                    // TODO: conditionals jmps

                    instruction.payload = .{
                        .jmp = .{
                            .condition = .Empty,
                            .label = label,
                        },
                    };
                },
                // wait <polarity> gpio <gpio_num>
                // wait <polarity> pin <pin_num>
                // wait <polarity> irq <irq_num> ( rel )
                .wait => {
                    const polarity = (try self.parse_number(u1)).?;
                    try self.skip_whitespaces();
                    const type_ = (try self.parse_identifier()).?;
                    try self.skip_whitespaces();
                    const number = (try self.parse_number(u5)).?;
                    try self.skip_whitespaces();

                    instruction.payload = .{
                        .wait = .{
                            .polarity = if (polarity == 1) .high else .low,
                            .source = blk: {
                                if (std.mem.eql(u8, type_, "gpio"))
                                    break :blk .Gpio;
                                if (std.mem.eql(u8, type_, "pin"))
                                    break :blk .Pin;
                                if (std.mem.eql(u8, type_, "irq"))
                                    break :blk .Irq;
                            },
                            .number = number,
                        },
                    };
                },
                // in <source>, <bit_count>
                .in => {
                    const source = try self.parse_register();
                    try self.skip_whitespaces();

                    std.debug.assert(self.get_character() == ',');
                    try self.consume(1);
                    try self.skip_whitespaces();

                    const bit_count = (try self.parse_number(u5)).?;
                    try self.skip_whitespaces();

                    _ = source;

                    instruction.payload = .{
                        .in = .{
                            .source = .Pins, // TODO
                            .bit_count = bit_count,
                        },
                    };
                },
                //out <destination>, <bit_count>
                .out => {
                    const destination = try self.parse_register();
                    try self.skip_whitespaces();

                    std.debug.assert(self.get_character() == ',');
                    try self.consume(1);
                    try self.skip_whitespaces();

                    const number = (try self.parse_number(u5)).?;
                    try self.skip_whitespaces();

                    _ = destination;

                    instruction.payload = .{
                        .out = .{
                            .destination = .Pins, // TODO:
                            .bit_count = number,
                        },
                    };
                },
                // push/pull ( iffull )
                // push/pull ( iffull ) block
                // push/pull ( iffull ) noblock
                .push, .pull => |p| {
                    var block: bool = true;
                    var maybe_block: ?[]const u8 = null;
                    const maybe_if_full = try self.parse_identifier();
                    const if_full: bool = blk: {
                        if (maybe_if_full != null and std.mem.eql(u8, maybe_if_full.?, "ifful")) {
                            try self.skip_whitespaces();
                            maybe_block = try self.parse_identifier();
                            break :blk true;
                        } else {
                            maybe_block = maybe_if_full;
                            break :blk false;
                        }
                    };

                    if (maybe_block) |b| {
                        if (std.mem.eql(u8, b, "block")) {
                            block = true;
                        } else if (std.mem.eql(u8, b, "noblock")) {
                            block = false;
                        } else @panic("invalid token");
                    }

                    const payload = .{
                        .if_full = if_full,
                        .block = block,
                    };
                    instruction.payload = switch (p) {
                        .push => .{ .push = payload },
                        .pull => .{ .pull = payload },
                        else => unreachable,
                    };
                },
                // nop
                .nop => {
                    instruction.payload = .{
                        .mov = .{
                            .source = .Y,
                            .operation = .None,
                            .destination = .Y,
                        },
                    };
                },
                else => {},
            }

            const side_set = try self.parse_side_set();
            instruction.side_set = side_set;
            try self.skip_whitespaces();

            const delay = try self.parse_delay();
            instruction.delay = delay;
            try self.skip_whitespaces();

            try self.skip_comments();
            return Token{ .instruction = instruction };
        }

        return null;
    }

    pub fn parse_identifier(self: *Parser) !?[]const u8 {
        const start = self.index;
        const end = blk: while (true) {
            switch (self.get_character()) {
                // TODO: better handling of indentifier characters
                'a'...'z', 'A'...'Z', '0'...'9', '_' => try self.consume(1),
                else => break :blk self.index,
            }
        };
        self.index = start;
        const identifier = self.data[start..end];

        try self.consume(identifier.len);
        return identifier;
    }

    pub fn parse_register(self: *Parser) !?[]const u8 {
        // TODO
        return self.parse_identifier();
    }

    pub fn parse_number(self: *Parser, comptime T: type) !?T {
        const start = self.index;
        const end = blk: while (true) {
            switch (self.get_character()) {
                // TODO: handle hex and octal
                '0'...'9' => try self.consume(1),
                else => break :blk self.index,
            }
        };
        self.index = start;
        const number = self.data[start..end];

        try self.consume(number.len);
        return try std.fmt.parseInt(T, number, 10);
    }

    pub fn parse_side_set(self: *Parser) !?u8 {
        if (self.data.len > self.index + 4)
            return null;

        const maybe_side = self.data[self.index .. self.index + 4];
        if (std.mem.eql(u8, maybe_side, "side")) {
            try self.consume(maybe_side.len);
            try self.skip_whitespaces();

            return try self.parse_number(u8);
        }
        return null;
    }

    pub fn parse_delay(self: *Parser) !?u8 {
        if (self.get_character() == '[') {
            try self.consume(1);

            const delay_number = try self.parse_number(u8);

            std.debug.assert(self.get_character() == ']');
            try self.consume(1);
            return delay_number;
        }
        return null;
    }

    pub fn skip_comments(self: *Parser) !void {
        if (self.get_character() == ';') {
            try self.consume(1);
            while (self.get_character() != '\n') {
                try self.consume(1);
            }
        }
    }

    pub fn skip_whitespaces(self: *Parser) !void {
        loop: while (true) {
            switch (self.get_character()) {
                ' ', '\t' => try self.consume(1),
                else => break :loop,
            }
        }
    }

    pub fn skip_newlines(self: *Parser) !void {
        loop: while (true) {
            switch (self.get_character()) {
                '\r', '\n' => try self.consume(1),
                else => break :loop,
            }
        }
    }

    pub fn consume(self: *Parser, count: usize) !void {
        if (self.index > self.data.len)
            return error.OutOfStream;

        self.index += count;
    }

    pub fn get_character(self: Parser) u8 {
        return self.data[self.index];
    }

    pub fn peek(self: Parser, index: usize) !u8 {
        if (index + self.index > self.data.len)
            return error.EndOfStream;
        return self.data[index + self.index];
    }
};

pub fn assemble(comptime code: []const u8) !Bytecode {
    var parser = Parser.init(code);
    var output = std.mem.zeroes(Program);
    var instruction_index: u8 = 0;
    var labels = std.mem.zeroes([32]?[]const u8);
    var side_set: ?u8 = null;
    var program_name: ?[]const u8 = null;

    var tokens = try std.BoundedArray(Parser.Token, 256).init(0);
    while (try parser.next()) |token| {
        std.log.err("token: {}", .{token});
        try tokens.append(token);
    }

    // TODO: somehow do this in one pass
    for (tokens.constSlice()) |token| {
        switch (token) {
            .program => |name| program_name = name,

            .side_set => |v| {
                std.debug.assert(side_set == null);
                side_set = v;
            },
            .label => |v| labels[instruction_index] = v,
            .instruction => {
                instruction_index += 1;
            },
            else => {},
        }
    }

    instruction_index = 0;
    for (tokens.constSlice()) |token| {
        switch (token) {
            .program => |name| program_name = name,
            .wrap_target => output.wrap_target = instruction_index,
            .wrap => output.wrap = instruction_index,
            .instruction => |i| {
                switch (i.payload) {
                    .jmp => |jmp| try output.instructions.append(try encode_jmp(&labels, jmp, side_set)),
                    else => {},
                }
                instruction_index += 1;
            },
            else => {},
        }
    }
    output.side_set = side_set;
    output.name = program_name.?;
    return Bytecode{
        .programs = &.{output},
    };
}

fn encode_jmp(labels: *[32]?[]const u8, instruction: Parser.Token.Instruction.Jmp, program_side_set: ?u8) !u16 {
    _ = program_side_set;
    const address = blk: for (labels, 0..) |label, i| {
        if (label) |l|
            if (std.mem.eql(u8, l, instruction.label)) break :blk i;
    } else @panic("label not found");
    _ = address;

    return 0;
}

test "tokenize" {
    var parser = Parser.init(@embedFile("pio/test/squarewave.pio"));

    var tokens = try std.BoundedArray(Parser.Token, 256).init(0);
    while (try parser.next()) |token| {
        std.log.err("token: {}", .{token});
        try tokens.append(token);
    }

    try std.testing.expect(tokens.slice()[0] == .program);
}

test "pio" {
    //std.testing.refAllDecls(@import("pio/test.zig"));
}
