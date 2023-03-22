const std = @import("std");
const assert = std.debug.assert;

pub const Define = struct {
    name: []const u8,
    value: u32,
    public: bool,
};

pub const Program = struct {
    name: []const u8,
    defines: []const Define,
    origin: ?u5,
    side_set: ?u8,
    wrap_target: u8,
    wrap: u8,
    instructions: std.BoundedArray(u16, 32),

    pub fn get_mask(program: Program) u32 {
        return (1 << program.instructions.len) - 1;
    }
};

pub const Output = struct {
    defines: []const Define,
    programs: []const Program,
};

pub const AssembleOptions = struct {
    tokenize: TokenizeOptions = .{},
    encode: EncodeOptions = .{},
};

pub fn assemble(comptime source: []const u8, comptime options: AssembleOptions) !Output {
    const tokens = try tokenize(source, options.tokenize);
    return encode(tokens, options.encode);
}

//==============================================================================
// Tokenize
//==============================================================================

const Token = union(enum) {
    directive: Directive,
    label: Label,
    instruction: Instruction,

    const Directive = union(enum) {
        program: []const u8,
        define: Token.Define,
        origin: Value,
        side_set: SideSet,
        wrap_target: void,
        wrap: void,
        lang_opt: LangOpt,
        word: Value,
    };

    const Label = struct {
        name: []const u8,
        public: bool = false,
    };

    const Instruction = struct {
        payload: Payload,
        side_set: ?u5 = null,
        delay: ?u5 = null,

        const Payload = union(enum) {
            nop: void,
            jmp: Jmp,
            wait: Wait,
            in: In,
            out: Out,
            push: Push,
            pull: Pull,
            mov: Mov,
            irq: Irq,
            set: Set,
        };

        const Jmp = struct {
            condition: Condition,
            target: Value,

            const Condition = enum(u3) {
                always = 0b000,
                x_is_zero = 0b001, // !X
                x_dec = 0b010, // X--
                y_is_zero = 0b011, // !Y
                y_dec = 0b100, // Y--
                x_is_not_y = 0b101, //X!=Y
                pin = 0b110, // PIN
                osre_not_empty = 0b111, // !OSRE
            };
        };

        const Wait = struct {
            polarity: u1,
            source: Source,
            num: u5,
            rel: bool,

            const Source = enum(u2) {
                gpio = 0b00,
                pin = 0b01,
                irq = 0b10,
            };
        };

        const In = struct {
            source: Source,
            bit_count: u5,

            const Source = enum(u3) {
                pins = 0b00,
                x = 0b001,
                y = 0b010,
                null = 0b011,
                isr = 0b110,
                osr = 0b111,
            };
        };

        const Out = struct {
            destination: Destination,
            bit_count: u5,

            const Destination = enum(u3) {
                pins = 0b000,
                x = 0b001,
                y = 0b010,
                null = 0b011,
                pindirs = 0b100,
                pc = 0b101,
                isr = 0b110,
                exec = 0b111,
            };
        };

        const Push = struct {
            block: bool,
            iffull: bool,
        };

        const Pull = struct {
            block: bool,
            ifempty: bool,
        };

        const Mov = struct {
            destination: Destination,
            operation: Operation,
            source: Source,

            const Destination = enum(u3) {
                pins = 0b000,
                x = 0b001,
                y = 0b010,
                exec = 0b100,
                pc = 0b101,
                isr = 0b110,
                osr = 0b111,
            };

            const Operation = enum(u2) {
                none = 0b00,
                invert = 0b01,
                bit_reverse = 0b10,
            };

            const Source = enum(u3) {
                pins = 0b00,
                x = 0b001,
                y = 0b010,
                null = 0b011,
                status = 0b101,
                isr = 0b110,
                osr = 0b111,
            };
        };

        const Irq = struct {
            clear: bool,
            wait: bool,
            num: u5,
            rel: bool,
        };

        const Set = struct {
            destination: Destination,
            value: Value,

            const Destination = enum(u3) {
                pins = 0b000,
                x = 0b001,
                y = 0b010,
                pindirs = 0b100,
            };
        };
    };

    const Value = union(enum) {
        // integer, hex, binary
        integer: u32,
        // either a symbol or label
        string: []const u8,
        expression: []const u8,

        pub fn format(
            value: Value,
            comptime fmt: []const u8,
            options: std.fmt.FormatOptions,
            writer: anytype,
        ) !void {
            _ = fmt;
            _ = options;
            switch (value) {
                .string => |str| try writer.print("\"{s}\"", .{str}),
                .expression => |expr| try writer.print("{s}", .{expr}),
                .integer => |int| try writer.print("{}", .{int}),
            }
        }
    };

    const Define = struct {
        name: []const u8,
        value: Value,
        public: bool = false,
    };

    const SideSet = struct {
        count: Value,
        opt: bool = false,
        pindirs: bool = false,
    };

    const LangOpt = struct {
        lang: []const u8,
        name: []const u8,
        option: []const u8,
    };
};

pub const TokenizeOptions = struct {
    capacity: u32 = 256,
};

// the characters we're interested in are:
// ';' -> line comment
// '/' -> '/' -> line comment
// '/' -> '*' -> block comment
// '%' -> <whitespace> -> <identifier> -> <whitespace> -> '{' -> code block
// '.' -> directive
const Tokenizer = struct {
    source: []const u8,
    index: u32,

    pub fn format(
        self: Tokenizer,
        comptime fmt: []const u8,
        options: std.fmt.FormatOptions,
        writer: anytype,
    ) !void {
        _ = fmt;
        _ = options;

        try writer.print(
            \\parser:
            \\  index: {}
            \\
            \\
        , .{self.index});

        var printed_cursor = false;
        var line_it = std.mem.tokenize(u8, self.source, "\n\r");
        while (line_it.next()) |line| {
            try writer.print("{s}\n", .{line});
            if (!printed_cursor and line_it.index > self.index) {
                try writer.writeByteNTimes(' ', line.len - (line_it.index - self.index));
                try writer.writeAll("\x1b[30;42;1m^\x1b[0m\n");
                printed_cursor = true;
            }
        }
    }

    fn init(source: []const u8) Tokenizer {
        return Tokenizer{
            .source = source,
            .index = 0,
        };
    }

    fn consume(self: *Tokenizer, count: u32) void {
        assert(self.index < self.source.len);
        self.index += count;
    }

    fn peek(self: Tokenizer) ?u8 {
        return if (self.index < self.source.len)
            self.source[self.index]
        else
            null;
    }

    fn get(self: *Tokenizer) ?u8 {
        return if (self.index < self.source.len) blk: {
            defer self.index += 1;
            break :blk self.source[self.index];
        } else null;
    }

    fn skip_line(self: *Tokenizer) void {
        while (self.get()) |c|
            if (c == '\n')
                return;
    }

    fn skip_until_end_of_comment_block(self: *Tokenizer) void {
        while (self.get()) |c| {
            if (c == '*') {
                if (self.peek()) |p| {
                    self.consume(1);
                    if (p == '/') {
                        return;
                    }
                }
            }
        }
    }

    fn skip_until_end_of_code_block(self: *Tokenizer) void {
        // TODO: assert we have the code identifier and open curly bracket
        while (self.get()) |c| {
            if (c == '%') {
                if (self.peek()) |p| {
                    self.consume(1);
                    if (p == '}') {
                        return;
                    }
                }
            }
        }
    }

    fn read_until_whitespace_or_end(self: *Tokenizer) ![]const u8 {
        const start = self.index;
        var end: ?u32 = null;
        while (self.peek()) |p| {
            switch (p) {
                ' ', '\n', '\r', '\t', ',' => {
                    end = self.index;
                    break;
                },
                else => self.consume(1),
            }
        } else end = self.index;

        return self.source[start .. end orelse return error.EndOfStream];
    }

    fn skip_whitespace(self: *Tokenizer) void {
        while (self.peek()) |p| {
            switch (p) {
                ' ', '\t', '\r', '\n', ',' => self.consume(1),
                else => return,
            }
        }
    }

    /// returns array of args
    fn get_args(self: *Tokenizer, comptime num: u32) TokenizeError![num]?[]const u8 {
        var args: [num]?[]const u8 = undefined;
        for (&args) |*arg|
            arg.* = try self.get_arg();

        return args;
    }

    const PeekResult = struct {
        str: []const u8,
        start: u32,
    };

    fn peek_arg(self: *Tokenizer) TokenizeError!?PeekResult {
        var tmp_index = self.index;
        return self.peek_arg_impl(&tmp_index);
    }

    fn consume_peek(self: *Tokenizer, result: PeekResult) void {
        const n = result.start + @intCast(u32, result.str.len);
        std.log.debug("index: {}, next: {}", .{ self.index, n });
        assert(self.index <= result.start);
        self.index = result.start + @intCast(u32, result.str.len);
    }

    /// gets next arg without consuming the stream
    fn peek_arg_impl(self: *Tokenizer, index: *u32) TokenizeError!?PeekResult {
        // skip whitespace
        while (index.* < self.source.len) {
            switch (self.source[index.*]) {
                ' ', '\t', ',' => index.* += 1,
                else => break,
            }
        }

        const start = index.*;
        const end = while (index.* < self.source.len) {
            switch (self.source[index.*]) {
                ' ', '\t', '\r', '\n', ',' => break index.*,
                else => index.* += 1,
            }
        } else index.*;

        if (start != end)
            std.log.debug("peeked arg: '{s}'", .{self.source[start..end]});

        return if (start != end)
            PeekResult{
                .str = self.source[start..end],
                .start = start,
            }
        else
            null;
    }

    fn get_arg(self: *Tokenizer) TokenizeError!?[]const u8 {
        return if (try self.peek_arg_impl(&self.index)) |result|
            result.str
        else
            null;
    }

    fn get_identifier(self: *Tokenizer) TokenizeError![]const u8 {
        self.skip_whitespace();
        return try self.read_until_whitespace_or_end();
    }

    const TokenizeError = error{
        TODO,
        EndOfStream,
        NoValue,
        NotAnExpression,
        Overflow,
        InvalidCharacter,
        InvalidSource,
        InvalidCondition,
        MissingArg,
        InvalidDestination,
        InvalidOperation,
        TooBig,
    };

    fn get_program(self: *Tokenizer) TokenizeError!Token {
        const name = try self.get_identifier();
        return Token{
            .directive = .{
                .program = name,
            },
        };
    }

    fn assert_is_lower(str: []const u8) void {
        for (str) |c|
            assert(std.ascii.isLower(c));
    }

    fn eql_lower(comptime lhs: []const u8, rhs: []const u8) bool {
        assert_is_lower(lhs);
        if (lhs.len != rhs.len)
            return false;

        var buf: [lhs.len]u8 = undefined;
        for (&buf, rhs) |*b, r|
            b.* = std.ascii.toLower(r);

        return std.mem.eql(u8, &buf, lhs);
    }

    fn get_define(self: *Tokenizer) TokenizeError!Token {
        const maybe_public = try self.get_identifier();
        var is_public = eql_lower("public", maybe_public);

        const name = if (is_public)
            try self.get_identifier()
        else
            maybe_public;

        const value = try self.get_value();

        return Token{
            .directive = .{
                .define = .{
                    .name = name,
                    .value = value,
                    .public = is_public,
                },
            },
        };
    }

    fn get_expression(self: *Tokenizer) TokenizeError!Token.Value {
        const start = self.index;
        var count: u32 = 1;

        if (self.get()) |c|
            if (c != '(')
                return error.NotAnExpression;

        while (self.get()) |c| {
            switch (c) {
                '(' => count += 1,
                ')' => {
                    count -= 1;
                },
                else => {},
            }

            if (count == 0) {
                return Token.Value{
                    .expression = self.source[start..self.index],
                };
            }
        } else {
            return error.NotAnExpression;
        }
    }

    fn value_from_string(str: []const u8) TokenizeError!Token.Value {
        return Token.Value{
            .integer = std.fmt.parseInt(u32, str, 0) catch |err| {
                std.log.debug("failed to parse: '{s}': {}", .{ str, err });
                return Token.Value{
                    .string = str,
                };
            },
        };
    }

    fn get_value(self: *Tokenizer) TokenizeError!Token.Value {
        self.skip_whitespace();

        if (self.peek()) |p|
            if (p == '(')
                return try self.get_expression()
            else {
                const identifier = try self.get_identifier();
                return value_from_string(identifier);
            }
        else
            return error.NoValue;
    }

    fn get_origin(self: *Tokenizer) TokenizeError!Token {
        return Token{
            .directive = .{
                .origin = try self.get_value(),
            },
        };
    }

    fn get_side_set(self: *Tokenizer) TokenizeError!Token {
        const args = try self.get_args(3);
        const count = try value_from_string(args[0] orelse return error.MissingArg);
        var opt = false;
        var pindirs = false;

        if (args[1]) |arg| {
            if (std.mem.eql(u8, "opt", arg))
                opt = true
            else if (std.mem.eql(u8, "pindirs", arg))
                pindirs = true;
        }

        if (args[2]) |arg| {
            if (std.mem.eql(u8, "pindirs", arg))
                pindirs = true;
        }

        return Token{
            .directive = .{
                .side_set = .{
                    .count = count,
                    // TODO
                    .opt = opt,
                    // TODO
                    .pindirs = pindirs,
                },
            },
        };
    }

    fn get_wrap_target(_: *Tokenizer) TokenizeError!Token {
        return Token{ .directive = .{ .wrap_target = {} } };
    }

    fn get_wrap(_: *Tokenizer) TokenizeError!Token {
        return Token{ .directive = .{ .wrap = {} } };
    }

    fn get_lang_opt(self: *Tokenizer) TokenizeError!Token {
        return Token{
            .directive = .{
                .lang_opt = .{
                    .lang = try self.get_identifier(),
                    .name = try self.get_identifier(),
                    .option = try self.get_identifier(),
                },
            },
        };
    }

    fn get_word(self: *Tokenizer) TokenizeError!Token {
        return Token{
            .directive = .{
                .word = try self.get_value(),
            },
        };
    }

    const directives = std.ComptimeStringMap(*const fn (*Tokenizer) TokenizeError!Token, .{
        .{ "program", get_program },
        .{ "define", get_define },
        .{ "origin", get_origin },
        .{ "side_set", get_side_set },
        .{ "wrap_target", get_wrap_target },
        .{ "wrap", get_wrap },
        .{ "lang_opt", get_lang_opt },
        .{ "word", get_word },
    });

    fn get_directive(self: *Tokenizer) !Token {
        const identifier = try self.read_until_whitespace_or_end();
        std.log.debug("identifier: {s},", .{identifier});
        return if (directives.get(identifier)) |handler|
            try handler(self)
        else
            error.InvalidDirective;
    }

    fn get_nop(_: *Tokenizer) TokenizeError!Token.Instruction.Payload {
        return Token.Instruction.Payload{
            .nop = {},
        };
    }

    fn target_from_string(str: []const u8) TokenizeError!Token.Instruction.Jmp.Target {
        const value = value_from_string(str);
        return Token.Instruction.Payload{
            .jmp = .{
                .condition = .always,
                .target = switch (value) {
                    .string => |label| Token.Instruction.Jmp.Target{
                        .label = label,
                    },
                    else => Token.Instruction.Jmp.Target{
                        .value = value,
                    },
                },
            },
        };
    }

    fn get_jmp(self: *Tokenizer) TokenizeError!Token.Instruction.Payload {
        const Condition = Token.Instruction.Jmp.Condition;
        const conditions = std.ComptimeStringMap(Condition, .{
            .{ "!x", .x_is_zero },
            .{ "x--", .x_dec },
            .{ "!y", .y_is_zero },
            .{ "y--", .y_dec },
            .{ "x!=y", .x_is_not_y },
            .{ "pin", .pin },
            .{ "!osre", .osre_not_empty },
        });

        const maybe_cond = (try self.get_arg()) orelse return error.MissingArg;
        var buf: [8]u8 = undefined;
        for (maybe_cond, 0..) |c, i|
            buf[i] = std.ascii.toLower(c);

        const maybe_cond_lower = buf[0..maybe_cond.len];
        const cond: Condition = conditions.get(maybe_cond_lower) orelse .always;
        const target_str = if (cond == .always)
            maybe_cond
        else
            (try self.get_arg()) orelse return error.MissingArg;

        return Token.Instruction.Payload{
            .jmp = .{
                .condition = cond,
                .target = try value_from_string(target_str),
            },
        };
    }

    fn get_wait(self: *Tokenizer) TokenizeError!Token.Instruction.Payload {
        const polarity = try std.fmt.parseInt(u1, (try self.get_arg()) orelse return error.MissingArg, 0);
        const source_str = (try self.get_arg()) orelse return error.MissingArg;
        const pin = try std.fmt.parseInt(u5, (try self.get_arg()) orelse return error.MissingArg, 0);

        var buf: [8]u8 = undefined;
        for (source_str, 0..) |c, i|
            buf[i] = std.ascii.toLower(c);

        const source_lower = buf[0..source_str.len];
        const source: Token.Instruction.Wait.Source =
            if (std.mem.eql(u8, "gpio", source_lower))
            .gpio
        else if (std.mem.eql(u8, "pin", source_lower))
            .pin
        else if (std.mem.eql(u8, "irq", source_lower))
            .irq
        else
            return error.InvalidSource;

        const rel: bool = if (source == .irq)
            if (try self.peek_arg()) |rel_result| blk: {
                const is_rel = std.mem.eql(u8, "rel", rel_result.str);
                if (is_rel)
                    self.consume_peek(rel_result);

                break :blk is_rel;
            } else false
        else
            false;

        return Token.Instruction.Payload{
            .wait = .{
                .polarity = polarity,
                .source = source,
                .num = pin,
                .rel = rel,
            },
        };
    }

    /// get the lowercase of a string, returns an error if it's too big
    fn lowercase_bounded(comptime max_size: usize, str: []const u8) TokenizeError!std.BoundedArray(u8, max_size) {
        if (str.len > max_size)
            return error.TooBig;

        var ret = try std.BoundedArray(u8, max_size).init(0);
        for (str) |c|
            try ret.append(std.ascii.toLower(c));

        return ret;
    }

    // TODO: I need to take a break. There is no rush to finish this. The thing
    // I need to keep in mind with `get_args()` is that I must only consume the
    // args that are used. side set and delay may be on the same line

    fn get_in(self: *Tokenizer) TokenizeError!Token.Instruction.Payload {
        const source_str = (try self.get_arg()) orelse return error.MissingArg;
        const bit_count_str = (try self.get_arg()) orelse return error.MissingArg;

        const source_lower = try lowercase_bounded(8, source_str);
        const bit_count = try std.fmt.parseInt(u5, bit_count_str, 0);
        return Token.Instruction.Payload{
            .in = .{
                .source = std.meta.stringToEnum(Token.Instruction.In.Source, source_lower.slice()) orelse return error.InvalidSource,
                .bit_count = bit_count,
            },
        };
    }

    fn get_out(self: *Tokenizer) TokenizeError!Token.Instruction.Payload {
        const dest_src = (try self.get_arg()) orelse return error.MissingArg;
        const bit_count_str = (try self.get_arg()) orelse return error.MissingArg;

        const dest_lower = try lowercase_bounded(8, dest_src);
        const bit_count = try std.fmt.parseInt(u5, bit_count_str, 0);
        return Token.Instruction.Payload{
            .out = .{
                .destination = std.meta.stringToEnum(Token.Instruction.Out.Destination, dest_lower.slice()) orelse return error.InvalidDestination,
                .bit_count = bit_count,
            },
        };
    }

    fn block_from_peek(self: *Tokenizer, result: PeekResult) TokenizeError!bool {
        const block_lower = try lowercase_bounded(8, result.str);
        const is_block = std.mem.eql(u8, "block", block_lower.slice());
        const is_noblock = std.mem.eql(u8, "noblock", block_lower.slice());

        if (is_block or is_noblock)
            self.consume_peek(result);

        return if (is_block)
            true
        else if (is_noblock)
            false
        else
            true;
    }

    fn get_push(self: *Tokenizer) TokenizeError!Token.Instruction.Payload {
        return if (try self.peek_arg()) |first_result| ret: {
            const lower = try lowercase_bounded(8, first_result.str);
            const iffull = std.mem.eql(u8, "iffull", lower.slice());

            const block: bool = if (iffull) blk: {
                self.consume_peek(first_result);
                break :blk if (try self.peek_arg()) |block_result|
                    try self.block_from_peek(block_result)
                else
                    true;
            } else try self.block_from_peek(first_result);

            break :ret Token.Instruction.Payload{
                .push = .{
                    .iffull = iffull,
                    .block = block,
                },
            };
        } else Token.Instruction.Payload{
            .push = .{
                .iffull = false,
                .block = true,
            },
        };
    }

    fn get_pull(self: *Tokenizer) TokenizeError!Token.Instruction.Payload {
        return if (try self.peek_arg()) |first_result| ret: {
            const lower = try lowercase_bounded(8, first_result.str);
            const ifempty = std.mem.eql(u8, "ifempty", lower.slice());

            const block: bool = if (ifempty) blk: {
                self.consume_peek(first_result);
                break :blk if (try self.peek_arg()) |block_result|
                    try self.block_from_peek(block_result)
                else
                    true;
            } else try self.block_from_peek(first_result);

            break :ret Token.Instruction.Payload{
                .pull = .{
                    .ifempty = ifempty,
                    .block = block,
                },
            };
        } else Token.Instruction.Payload{
            .pull = .{
                .ifempty = false,
                .block = true,
            },
        };
    }

    fn get_mov(self: *Tokenizer) TokenizeError!Token.Instruction.Payload {
        const dest_str = (try self.get_arg()) orelse return error.MissingArg;
        const destination = std.meta.stringToEnum(Token.Instruction.Mov.Destination, dest_str) orelse return error.InvalidDestination;

        const second = try self.get_arg() orelse return error.MissingArg;
        std.log.debug("second: {s}", .{second});
        const op_prefixed: ?[]const u8 = if (std.mem.startsWith(u8, second, "!"))
            "!"
        else if (std.mem.startsWith(u8, second, "~"))
            "~"
        else if (std.mem.startsWith(u8, second, "::"))
            "::"
        else
            null;

        const source_str = if (op_prefixed) |op_str|
            if (second.len == op_str.len)
                (try self.get_arg()) orelse return error.MissingArg
            else
                second[op_str.len..]
        else
            second;
        std.log.debug("source str: {s}, op prefixed: {?s}", .{ source_str, op_prefixed });
        const source = std.meta.stringToEnum(Token.Instruction.Mov.Source, source_str) orelse return error.InvalidSource;

        const operation: Token.Instruction.Mov.Operation = if (op_prefixed) |op_str|
            if (std.mem.eql(u8, "!", op_str))
                .invert
            else if (std.mem.eql(u8, "~", op_str))
                .invert
            else if (std.mem.eql(u8, "::", op_str))
                .bit_reverse
            else
                return error.InvalidOperation
        else
            .none;

        return Token.Instruction.Payload{
            .mov = .{
                .destination = destination,
                .source = source,
                .operation = operation,
            },
        };
    }

    fn get_irq(self: *Tokenizer) TokenizeError!Token.Instruction.Payload {
        const first = (try self.get_arg()) orelse return error.MissingArg;

        var has_mode = false;
        const num = std.fmt.parseInt(u5, first, 0) catch blk: {
            has_mode = true;
            const num_str = (try self.get_arg()) orelse return error.MissingArg;
            break :blk try std.fmt.parseInt(u5, num_str, 0);
        };

        var clear = false;
        var wait = false;
        if (has_mode) {
            const first_lower = try lowercase_bounded(8, first);
            if (std.mem.eql(u8, "set", first_lower.slice())) {
                // do nothing
            } else if (std.mem.eql(u8, "nowait", first_lower.slice())) {
                // do nothing
            } else if (std.mem.eql(u8, "wait", first_lower.slice())) {
                wait = true;
            } else if (std.mem.eql(u8, "clear", first_lower.slice())) {
                clear = true;
            } else return error.InvalidSource;
        }

        const rel: bool = if (try self.peek_arg()) |result| blk: {
            const rel_lower = try lowercase_bounded(8, result.str);
            const is_rel = std.mem.eql(u8, "rel", rel_lower.slice());
            if (is_rel)
                self.consume_peek(result);

            break :blk is_rel;
        } else false;

        return Token.Instruction.Payload{
            .irq = .{
                .clear = clear,
                .wait = wait,
                .num = num,
                .rel = rel,
            },
        };
    }

    fn get_set(self: *Tokenizer) TokenizeError!Token.Instruction.Payload {
        const dest_str = (try self.get_arg()) orelse return error.MissingArg;
        const value = try value_from_string((try self.get_arg()) orelse return error.MissingArg);

        std.log.debug("dest: {s}", .{dest_str});
        return Token.Instruction.Payload{
            .set = .{
                .destination = std.meta.stringToEnum(Token.Instruction.Set.Destination, dest_str) orelse return error.InvalidDestination,
                .value = value,
            },
        };
    }

    const instructions = std.ComptimeStringMap(*const fn (*Tokenizer) TokenizeError!Token.Instruction.Payload, .{
        .{ "nop", get_nop },
        .{ "jmp", get_jmp },
        .{ "wait", get_wait },
        .{ "in", get_in },
        .{ "out", get_out },
        .{ "push", get_push },
        .{ "pull", get_pull },
        .{ "mov", get_mov },
        .{ "irq", get_irq },
        .{ "set", get_set },
    });

    fn get_instruction(self: *Tokenizer, name: []const u8) !Token {
        std.log.debug("getting instruction", .{});
        var buf: [8]u8 = undefined;
        if (name.len > buf.len)
            return error.InvalidInstruction;

        for (name, 0..) |c, i|
            buf[i] = c;

        const name_lower = buf[0..name.len];
        std.log.debug("instruction: {s}", .{name_lower});
        const payload = if (instructions.get(name_lower)) |handler|
            try handler(self)
        else {
            std.log.debug("invalid instruction: {s}", .{name_lower});
            return error.InvalidInstruction;
        };

        var side_set: ?u5 = null;
        var delay: ?u5 = null;

        std.log.debug("side set: {}", .{self});

        // side set
        if (try self.peek_arg()) |result| if (eql_lower("side", result.str)) {
            self.consume_peek(result);

            const side_set_str = (try self.get_arg()) orelse return error.MissingArg;
            side_set = try std.fmt.parseInt(u5, side_set_str, 0);
        };

        // delay
        if (try self.get_arg()) |arg| {
            if (!std.mem.startsWith(u8, arg, "[") or !std.mem.endsWith(u8, arg, "]"))
                return error.InvalidDelay;

            delay = try std.fmt.parseInt(u5, arg[1 .. arg.len - 1], 0);
        }

        self.skip_line();
        return Token{
            .instruction = .{
                .payload = payload,
                .side_set = side_set,
                .delay = delay,
            },
        };
    }

    fn next(self: *Tokenizer) !?Token {
        while (self.peek()) |p| {
            switch (p) {
                ' ', '\t', '\n', '\r', ',' => self.consume(1),
                ';' => self.skip_line(),
                '/' => {
                    self.consume(1);
                    if (self.peek()) |p2| {
                        self.consume(1);
                        switch (p2) {
                            '/' => self.skip_line(),
                            '*' => self.skip_until_end_of_comment_block(),
                            else => unreachable,
                        }
                    } else return null;
                },
                '%' => {
                    self.consume(1);
                    self.skip_until_end_of_code_block();
                },
                '.' => {
                    self.consume(1);
                    return try self.get_directive();
                },
                'a'...'z', 'A'...'Z', '0'...'9', '_' => {
                    const first = try self.get_identifier();

                    // definitely a label
                    return if (eql_lower("public", first))
                        Token{
                            .label = .{
                                .public = true,
                                .name = blk: {
                                    const tmp = try self.get_identifier();
                                    break :blk tmp[0 .. tmp.len - 1];
                                },
                            },
                        }
                    else if (std.mem.endsWith(u8, first, ":"))
                        Token{
                            .label = .{
                                .name = first[0 .. first.len - 1],
                            },
                        }
                    else
                        try self.get_instruction(first);
                },
                else => return error.Unhandled,
            }
        }

        return null;
    }
};

fn tokenize(
    source: []const u8,
    comptime options: TokenizeOptions,
) !std.BoundedArray(Token, options.capacity) {
    var tokens = try std.BoundedArray(Token, options.capacity).init(0);
    var tokenizer = Tokenizer.init(source);
    while (try tokenizer.next()) |token|
        try tokens.append(token);

    std.log.debug("tokens:", .{});
    for (tokens.slice()) |token|
        std.log.debug("token: {}", .{token});

    return tokens;
}

//==============================================================================
// Encode
//==============================================================================

pub const EncodeOptions = struct {};

fn encode(tokens: []const Token, comptime options: EncodeOptions) !Output {
    _ = tokens;
    _ = options;

    return Output{
        .defines = &.{},
        .programs = &.{},
    };
}

//==============================================================================
// Tokenization Tests
//==============================================================================

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const expectEqualStrings = std.testing.expectEqualStrings;

const TokenTag = @typeInfo(Token).Union.tag_type.?;
const DirectiveTag = @typeInfo(Token.Directive).Union.tag_type.?;
const PayloadTag = @typeInfo(Token.Instruction.Payload).Union.tag_type.?;

fn expect_program(expected: []const u8, actual: Token) !void {
    try expectEqual(TokenTag.directive, actual);
    try expectEqual(DirectiveTag.program, actual.directive);
    try expectEqualStrings(expected, actual.directive.program);
}

fn expect_value(expected: Token.Value, actual: Token.Value) !void {
    switch (expected) {
        .integer => |int| try expectEqual(int, actual.integer),
        .string => |str| try expectEqualStrings(str, actual.string),
        .expression => |expr| try expectEqualStrings(expr, actual.expression),
    }
}

fn expect_define(expected: Token.Define, actual: Token) !void {
    try expectEqual(TokenTag.directive, actual);
    try expectEqual(DirectiveTag.define, actual.directive);

    const define = actual.directive.define;
    try expectEqualStrings(expected.name, define.name);
    try expect_value(expected.value, define.value);
}

fn expect_origin(expected: Token.Value, actual: Token) !void {
    try expectEqual(TokenTag.directive, actual);
    try expectEqual(DirectiveTag.origin, actual.directive);
    try expect_value(expected, actual.directive.origin);
}

fn expect_side_set(expected: Token.SideSet, actual: Token) !void {
    try expectEqual(TokenTag.directive, actual);
    try expectEqual(DirectiveTag.side_set, actual.directive);

    const side_set = actual.directive.side_set;
    try expect_value(expected.count, side_set.count);
    try expectEqual(expected.opt, side_set.opt);
    try expectEqual(expected.pindirs, side_set.pindirs);
}

fn expect_wrap_target(actual: Token) !void {
    try expectEqual(TokenTag.directive, actual);
    try expectEqual(DirectiveTag.wrap_target, actual.directive);
}

fn expect_wrap(actual: Token) !void {
    try expectEqual(TokenTag.directive, actual);
    try expectEqual(DirectiveTag.wrap, actual.directive);
}

fn expect_lang_opt(expected: Token.LangOpt, actual: Token) !void {
    try expectEqual(TokenTag.directive, actual);
    try expectEqual(DirectiveTag.lang_opt, actual.directive);

    const lang_opt = actual.directive.lang_opt;
    try expectEqualStrings(expected.lang, lang_opt.lang);
    try expectEqualStrings(expected.name, lang_opt.name);
    try expectEqualStrings(expected.option, lang_opt.option);
}

fn expect_word(expected: Token.Value, actual: Token) !void {
    try expectEqual(TokenTag.directive, actual);
    try expectEqual(DirectiveTag.word, actual.directive);
    try expect_value(expected, actual.directive.word);
}

fn expect_label(expected: Token.Label, actual: Token) !void {
    try expectEqual(TokenTag.label, actual);

    const label = actual.label;
    try expectEqual(expected.public, label.public);
    try expectEqualStrings(expected.name, label.name);
}

const ExpectedNopInstr = struct {
    delay: ?u5 = null,
    side_set: ?u5 = null,
};

fn expect_instr_nop(expected: ExpectedNopInstr, actual: Token) !void {
    try expectEqual(TokenTag.instruction, actual);
    try expectEqual(PayloadTag.nop, actual.instruction.payload);

    const instr = actual.instruction;
    try expectEqual(expected.delay, instr.delay);
    try expectEqual(expected.side_set, instr.side_set);
}

const ExpectedSetInstr = struct {
    dest: Token.Instruction.Set.Destination,
    value: Token.Value,
    delay: ?u5 = null,
    side_set: ?u5 = null,
};

fn expect_instr_set(expected: ExpectedSetInstr, actual: Token) !void {
    try expectEqual(TokenTag.instruction, actual);
    try expectEqual(PayloadTag.set, actual.instruction.payload);

    const instr = actual.instruction;
    try expectEqual(expected.side_set, instr.side_set);
    try expectEqual(expected.delay, instr.delay);

    const set = instr.payload.set;
    try expectEqual(expected.dest, set.destination);
    try expect_value(expected.value, set.value);
}

const ExpectedJmpInstr = struct {
    cond: Token.Instruction.Jmp.Condition = .always,
    target: Token.Value,
    delay: ?u5 = null,
    side_set: ?u5 = null,
};

fn expect_instr_jmp(expected: ExpectedJmpInstr, actual: Token) !void {
    try expectEqual(TokenTag.instruction, actual);
    try expectEqual(PayloadTag.jmp, actual.instruction.payload);

    const instr = actual.instruction;
    try expectEqual(expected.side_set, instr.side_set);
    try expectEqual(expected.delay, instr.delay);

    const jmp = instr.payload.jmp;
    try expectEqual(expected.cond, jmp.condition);
    try expect_value(expected.target, jmp.target);
}

const ExpectedWaitInstr = struct {
    polarity: u1,
    source: Token.Instruction.Wait.Source,
    num: u5,
    // only valid for irq source
    rel: bool = false,
    delay: ?u5 = null,
    side_set: ?u5 = null,
};

fn expect_instr_wait(expected: ExpectedWaitInstr, actual: Token) !void {
    try expectEqual(TokenTag.instruction, actual);
    try expectEqual(PayloadTag.wait, actual.instruction.payload);

    const instr = actual.instruction;
    try expectEqual(expected.side_set, instr.side_set);
    try expectEqual(expected.delay, instr.delay);

    const wait = instr.payload.wait;
    try expectEqual(expected.polarity, wait.polarity);
    try expectEqual(expected.source, wait.source);
    try expectEqual(expected.num, wait.num);
}

const ExpectedInInstr = struct {
    source: Token.Instruction.In.Source,
    bit_count: u5,
    delay: ?u5 = null,
    side_set: ?u5 = null,
};

fn expect_instr_in(expected: ExpectedInInstr, actual: Token) !void {
    try expectEqual(TokenTag.instruction, actual);
    try expectEqual(PayloadTag.in, actual.instruction.payload);

    const instr = actual.instruction;
    try expectEqual(expected.side_set, instr.side_set);
    try expectEqual(expected.delay, instr.delay);

    const in = instr.payload.in;
    try expectEqual(expected.source, in.source);
    try expectEqual(expected.bit_count, in.bit_count);
}

const ExpectedOutInstr = struct {
    destination: Token.Instruction.Out.Destination,
    bit_count: u5,
    delay: ?u5 = null,
    side_set: ?u5 = null,
};

fn expect_instr_out(expected: ExpectedOutInstr, actual: Token) !void {
    try expectEqual(TokenTag.instruction, actual);
    try expectEqual(PayloadTag.out, actual.instruction.payload);

    const instr = actual.instruction;
    try expectEqual(expected.side_set, instr.side_set);
    try expectEqual(expected.delay, instr.delay);

    const out = instr.payload.out;
    try expectEqual(expected.destination, out.destination);
    try expectEqual(expected.bit_count, out.bit_count);
}

const ExpectedPushInstr = struct {
    block: bool = true,
    iffull: bool = false,
    delay: ?u5 = null,
    side_set: ?u5 = null,
};

fn expect_instr_push(expected: ExpectedPushInstr, actual: Token) !void {
    try expectEqual(TokenTag.instruction, actual);
    try expectEqual(PayloadTag.push, actual.instruction.payload);

    const instr = actual.instruction;
    try expectEqual(expected.side_set, instr.side_set);
    try expectEqual(expected.delay, instr.delay);

    const push = instr.payload.push;
    try expectEqual(expected.block, push.block);
    try expectEqual(expected.iffull, push.iffull);
}

const ExpectedPullInstr = struct {
    block: bool = true,
    ifempty: bool = false,
    delay: ?u5 = null,
    side_set: ?u5 = null,
};

fn expect_instr_pull(expected: ExpectedPullInstr, actual: Token) !void {
    try expectEqual(TokenTag.instruction, actual);
    try expectEqual(PayloadTag.pull, actual.instruction.payload);

    const instr = actual.instruction;
    try expectEqual(expected.side_set, instr.side_set);
    try expectEqual(expected.delay, instr.delay);

    const pull = instr.payload.pull;
    try expectEqual(expected.block, pull.block);
    try expectEqual(expected.ifempty, pull.ifempty);
}

const ExpectedMovInstr = struct {
    source: Token.Instruction.Mov.Source,
    destination: Token.Instruction.Mov.Destination,
    operation: Token.Instruction.Mov.Operation = .none,
    delay: ?u5 = null,
    side_set: ?u5 = null,
};

fn expect_instr_mov(expected: ExpectedMovInstr, actual: Token) !void {
    try expectEqual(TokenTag.instruction, actual);
    try expectEqual(PayloadTag.mov, actual.instruction.payload);

    const instr = actual.instruction;
    try expectEqual(expected.side_set, instr.side_set);
    try expectEqual(expected.delay, instr.delay);

    const mov = instr.payload.mov;
    try expectEqual(expected.source, mov.source);
    try expectEqual(expected.operation, mov.operation);
    try expectEqual(expected.destination, mov.destination);
}

const ExpectedIrqInstr = struct {
    clear: bool,
    wait: bool,
    num: u5,
    rel: bool = false,
    delay: ?u5 = null,
    side_set: ?u5 = null,
};

fn expect_instr_irq(expected: ExpectedIrqInstr, actual: Token) !void {
    try expectEqual(TokenTag.instruction, actual);
    try expectEqual(PayloadTag.irq, actual.instruction.payload);

    const instr = actual.instruction;
    try expectEqual(expected.side_set, instr.side_set);
    try expectEqual(expected.delay, instr.delay);

    const irq = instr.payload.irq;
    try expectEqual(expected.clear, irq.clear);
    try expectEqual(expected.wait, irq.wait);
    try expectEqual(expected.rel, irq.rel);
}

test "tokenize.empty string" {
    const tokens = try tokenize("", .{});
    try expectEqual(@as(usize, 0), tokens.len);
}

test "tokenize.whitespace" {
    const tokens = try tokenize(" \t\r\n", .{});
    try expectEqual(@as(usize, 0), tokens.len);
}

test "tokenize.comma line comment" {
    const tokens = try tokenize("; this is a line comment", .{});

    try expectEqual(@as(usize, 0), tokens.len);
}

test "tokenize.slash line comment" {
    const tokens = try tokenize("// this is a line comment", .{});

    try expectEqual(@as(usize, 0), tokens.len);
}

test "tokenize.block comment" {
    const tokens = try tokenize(
        \\/* this is
        \\   a block comment */
    , .{});

    try expectEqual(@as(usize, 0), tokens.len);
}

test "tokenize.code block" {
    const tokens = try tokenize(
        \\% c-sdk {
        \\   int foo;
        \\%}
    , .{});

    try expectEqual(@as(usize, 0), tokens.len);
}

test "tokenize.directive.program" {
    const tokens = try tokenize(".program arst", .{});
    try expect_program("arst", tokens.get(0));
}

test "tokenize.directive.define" {
    const tokens = try tokenize(".define symbol_name 1", .{});

    try expect_define(.{
        .name = "symbol_name",
        .value = .{ .integer = 1 },
    }, tokens.get(0));
}

test "tokenize.directive.define.public" {
    const tokens = try tokenize(".define public symbol_name 0x1", .{});

    try expect_define(.{
        .name = "symbol_name",
        .value = .{ .integer = 1 },
        .public = true,
    }, tokens.get(0));
}

test "tokenize.directive.define.with expression" {
    std.log.debug("", .{});
    const tokens = try tokenize(
        \\.define symbol_name 0x1
        \\.define something (symbol_name * 2)
    , .{});

    try expect_define(.{
        .name = "symbol_name",
        .value = .{ .integer = 1 },
    }, tokens.get(0));

    try expect_define(.{
        .name = "something",
        .value = .{ .expression = "(symbol_name * 2)" },
    }, tokens.get(1));
}

test "tokenize.directive.origin" {
    const tokens = try tokenize(".origin 0x10", .{});
    try expect_origin(.{ .integer = 0x10 }, tokens.get(0));
}

test "tokenize.directive.side_set" {
    const tokens = try tokenize(".side_set 1", .{});
    try expect_side_set(.{ .count = .{ .integer = 1 } }, tokens.get(0));
}

test "tokenize.directive.side_set.opt" {
    const tokens = try tokenize(".side_set 1 opt", .{});
    try expect_side_set(.{ .count = .{ .integer = 1 }, .opt = true }, tokens.get(0));
}

test "tokenize.directive.side_set.pindirs" {
    const tokens = try tokenize(".side_set 1 pindirs", .{});
    try expect_side_set(.{ .count = .{ .integer = 1 }, .pindirs = true }, tokens.get(0));
}

test "tokenize.directive.wrap_target" {
    const tokens = try tokenize(".wrap_target", .{});
    try expect_wrap_target(tokens.get(0));
}

test "tokenize.directive.wrap" {
    const tokens = try tokenize(".wrap", .{});
    try expect_wrap(tokens.get(0));
}

test "tokenize.directive.lang_opt" {
    const tokens = try tokenize(".lang_opt c flag foo", .{});
    try expect_lang_opt(.{ .lang = "c", .name = "flag", .option = "foo" }, tokens.get(0));
}

test "tokenize.directive.word" {
    const tokens = try tokenize(".word 0xaaaa", .{});
    try expect_word(.{ .integer = 0xaaaa }, tokens.get(0));
}

test "tokenize.label" {
    const tokens = try tokenize("my_label:", .{});
    try expect_label(.{ .name = "my_label" }, tokens.get(0));
}

test "tokenize.label.public" {
    const tokens = try tokenize("public my_label:", .{});
    try expect_label(.{ .name = "my_label", .public = true }, tokens.get(0));
}

test "tokenize.instr.nop" {
    const tokens = try tokenize("nop", .{});
    try expect_instr_nop(.{}, tokens.get(0));
}

test "tokenize.instr.jmp.label" {
    const tokens = try tokenize("jmp my_label", .{});
    try expect_instr_jmp(.{ .target = .{ .string = "my_label" } }, tokens.get(0));
}

test "tokenize.instr.jmp.value" {
    const tokens = try tokenize("jmp 0x2", .{});
    try expect_instr_jmp(.{ .target = .{ .integer = 2 } }, tokens.get(0));
}

test "tokenize.instr.jmp.conditions" {
    const Condition = Token.Instruction.Jmp.Condition;
    const cases = std.ComptimeStringMap(Condition, .{
        .{ "!x", .x_is_zero },
        .{ "x--", .x_dec },
        .{ "!y", .y_is_zero },
        .{ "y--", .y_dec },
        .{ "x!=y", .x_is_not_y },
        .{ "pin", .pin },
        .{ "!osre", .osre_not_empty },
    });

    inline for (cases.kvs) |case| {
        const op = case.key;
        const cond = case.value;
        const tokens = try tokenize(std.fmt.comptimePrint("jmp {s} my_label", .{op}), .{});

        try expect_instr_jmp(.{ .cond = cond, .target = .{
            .string = "my_label",
        } }, tokens.get(0));
    }
}

test "tokenize.instr.wait" {
    inline for (.{ "gpio", "pin", "irq" }) |source| {
        const tokens = try tokenize(std.fmt.comptimePrint("wait 0 {s} 1", .{source}), .{});
        try expect_instr_wait(.{
            .polarity = 0,
            .source = @field(Token.Instruction.Wait.Source, source),
            .num = 1,
        }, tokens.get(0));
    }
}

test "tokenize.instr.wait.irq.rel" {
    const tokens = try tokenize("wait 1 irq 1 rel", .{});
    try expect_instr_wait(.{
        .polarity = 1,
        .source = .irq,
        .num = 1,
        .rel = true,
    }, tokens.get(0));
}

test "tokenize.instr.in" {
    inline for (.{
        "pins",
        "x",
        "y",
        "null",
        "isr",
        "osr",
    }, 1..) |source, bit_count| {
        const tokens = try tokenize(std.fmt.comptimePrint("in {s}, {}", .{
            source,
            bit_count,
        }), .{});

        try expect_instr_in(.{
            .source = @field(Token.Instruction.In.Source, source),
            .bit_count = @intCast(u5, bit_count),
        }, tokens.get(0));
    }
}

test "tokenize.instr.out" {
    inline for (.{
        "pins",
        "x",
        "y",
        "null",
        "pindirs",
        "pc",
        "isr",
        "exec",
    }, 1..) |destination, bit_count| {
        const tokens = try tokenize(std.fmt.comptimePrint("out {s}, {}", .{
            destination,
            bit_count,
        }), .{});

        try expect_instr_out(.{
            .destination = @field(Token.Instruction.Out.Destination, destination),
            .bit_count = @intCast(u5, bit_count),
        }, tokens.get(0));
    }
}

test "tokenize.instr.push" {
    const tokens = try tokenize("push", .{});
    try expect_instr_push(.{}, tokens.get(0));
}

test "tokenize.instr.push.block" {
    const tokens = try tokenize("push block", .{});
    try expect_instr_push(.{
        .block = true,
    }, tokens.get(0));
}

test "tokenize.instr.push.noblock" {
    const tokens = try tokenize("push noblock", .{});
    try expect_instr_push(.{
        .block = false,
    }, tokens.get(0));
}

test "tokenize.instr.push.iffull" {
    const tokens = try tokenize("push iffull noblock", .{});
    try expect_instr_push(.{
        .block = false,
        .iffull = true,
    }, tokens.get(0));
}

test "tokenize.instr.pull" {
    const tokens = try tokenize("pull", .{});
    try expect_instr_pull(.{}, tokens.get(0));
}

test "tokenize.instr.pull.block" {
    const tokens = try tokenize("pull block", .{});
    try expect_instr_pull(.{
        .block = true,
    }, tokens.get(0));
}

test "tokenize.instr.pull.noblock" {
    const tokens = try tokenize("pull noblock", .{});
    try expect_instr_pull(.{
        .block = false,
    }, tokens.get(0));
}

test "tokenize.instr.pull.ifempty" {
    const tokens = try tokenize("pull ifempty noblock", .{});
    try expect_instr_pull(.{
        .block = false,
        .ifempty = true,
    }, tokens.get(0));
}

test "tokenize.instr.mov" {
    inline for (.{
        "pins",
        "x",
        "y",
        "null",
        "status",
        "isr",
        "osr",
    }) |source| {
        const tokens = try tokenize(std.fmt.comptimePrint("mov x {s}", .{source}), .{});

        try expect_instr_mov(.{
            .source = @field(Token.Instruction.Mov.Source, source),
            .destination = .x,
        }, tokens.get(0));
    }

    inline for (.{
        "pins",
        "x",
        "y",
        "exec",
        "pc",
        "isr",
        "osr",
    }) |dest| {
        const tokens = try tokenize(std.fmt.comptimePrint("mov {s} x", .{dest}), .{});

        try expect_instr_mov(.{
            .source = .x,
            .destination = @field(Token.Instruction.Mov.Destination, dest),
        }, tokens.get(0));
    }

    const Operation = Token.Instruction.Mov.Operation;
    const operations = std.ComptimeStringMap(Operation, .{
        .{ "!", .invert },
        .{ "~", .invert },
        .{ "::", .bit_reverse },
    });

    inline for (.{ "", " " }) |space| {
        inline for (operations.kvs) |kv| {
            const str = kv.key;
            const operation = kv.value;
            const tokens = try tokenize(std.fmt.comptimePrint("mov x {s}{s}y", .{
                str,
                space,
            }), .{});

            try expect_instr_mov(.{
                .destination = .x,
                .operation = operation,
                .source = .y,
            }, tokens.get(0));
        }
    }
}

test "tokenize.instr.irq" {
    const ClearWait = struct {
        clear: bool,
        wait: bool,
    };

    const modes = std.ComptimeStringMap(ClearWait, .{
        .{ "", .{ .clear = false, .wait = false } },
        .{ "set", .{ .clear = false, .wait = false } },
        .{ "nowait", .{ .clear = false, .wait = false } },
        .{ "wait", .{ .clear = false, .wait = true } },
        .{ "clear", .{ .clear = true, .wait = false } },
    });

    inline for (modes.kvs, 0..) |kv, num| {
        const tokens = try tokenize(std.fmt.comptimePrint("irq {s} {}", .{
            kv.key,
            num,
        }), .{});

        try expect_instr_irq(.{
            .clear = kv.value.clear,
            .wait = kv.value.wait,
            .num = num,
        }, tokens.get(0));
    }
}

test "tokenize.instr.irq.rel" {
    const tokens = try tokenize("irq set 2 rel", .{});
    try expect_instr_irq(.{
        .clear = false,
        .wait = false,
        .num = 2,
        .rel = true,
    }, tokens.get(0));
}

test "tokenize.instr.set" {
    inline for (.{
        "pins",
        "x",
        "y",
        "pindirs",
    }) |dest| {
        const tokens = try tokenize(std.fmt.comptimePrint("set {s}, 2", .{dest}), .{});
        try expect_instr_set(.{
            .dest = @field(Token.Instruction.Set.Destination, dest),
            .value = .{ .integer = 2 },
        }, tokens.get(0));
    }
}

const instruction_examples = .{
    "nop",
    "jmp arst",
    "wait 0 gpio 1",
    "in pins, 2",
    "out pc, 1",
    "push",
    "pull",
    "mov x y",
    "irq 1",
    "set pins 2",
};

test "tokenize.instr.label prefixed" {
    inline for (instruction_examples) |instr| {
        const tokens = try tokenize(std.fmt.comptimePrint("my_label: {s}", .{instr}), .{});
        try expectEqual(@as(usize, 2), tokens.len);
        try expect_label(.{ .name = "my_label" }, tokens.get(0));
    }
}

test "tokenize.instr.side_set" {
    inline for (instruction_examples) |instr| {
        const tokens = try tokenize(std.fmt.comptimePrint("{s} side 0", .{instr}), .{});
        const token = tokens.get(0);
        try expectEqual(@as(?u5, 0), token.instruction.side_set);
        try expectEqual(@as(?u5, null), token.instruction.delay);
    }
}

test "tokenize.instr.delay" {
    inline for (instruction_examples) |instr| {
        const tokens = try tokenize(std.fmt.comptimePrint("{s} [1]", .{instr}), .{});
        const token = tokens.get(0);
        try expectEqual(@as(?u5, null), token.instruction.side_set);
        try expectEqual(@as(?u5, 1), token.instruction.delay);
    }
}

test "tokenize.instr.side_set and delay" {
    inline for (instruction_examples) |instr| {
        const tokens = try tokenize(std.fmt.comptimePrint("{s} side 1 [2]", .{instr}), .{});
        const token = tokens.get(0);
        try expectEqual(@as(?u5, 1), token.instruction.side_set);
        try expectEqual(@as(?u5, 2), token.instruction.delay);
    }
}

test "comparison" {
    //std.testing.refAllDecls(@import("assembler/comparison_tests.zig"));
}
