const std = @import("std");
const assert = std.debug.assert;

const assembler = @import("assembler.zig");
const Diagnostics = assembler.Diagnostics;

const tokenizer = @import("tokenizer.zig");
const Token = tokenizer.Token;

pub const Options = struct {
    max_defines: u32 = 16,
    max_programs: u32 = 16,
};

pub fn encode(
    comptime tokens: []const Token,
    diags: *?assembler.Diagnostics,
    comptime options: Options,
) !Encoder(options).Output {
    var encoder = Encoder(options).init(tokens);
    return try encoder.encode_output(diags);
}

pub const SideSet = struct {
    count: u3,
    optional: bool,
    pindirs: bool,
};

pub fn Encoder(comptime options: Options) type {
    return struct {
        output: Self.Output,
        tokens: []const Token,
        index: u32,

        const Self = @This();
        const Output = struct {
            global_defines: BoundedDefines,
            private_defines: BoundedDefines,
            programs: BoundedPrograms,
        };

        const BoundedDefines = std.BoundedArray(assembler.Define, options.max_defines);
        const BoundedPrograms = std.BoundedArray(BoundedProgram, options.max_programs);
        const BoundedInstructions = std.BoundedArray(Instruction, 32);
        const BoundedLabels = std.BoundedArray(Label, 32);
        const Label = struct {
            name: []const u8,
            index: u5,
            public: bool,
        };

        const BoundedProgram = struct {
            name: []const u8,
            defines: BoundedDefines,
            private_defines: BoundedDefines,
            instructions: BoundedInstructions,
            labels: BoundedLabels,
            origin: ?u5,
            side_set: ?SideSet,
            wrap_target: ?u5,
            wrap: ?u5,

            pub fn to_exported_program(bounded: BoundedProgram) assembler.Program {
                return assembler.Program{
                    .name = bounded.name,
                    .defines = bounded.defines.slice(),
                    .instructions = @ptrCast([]const u16, bounded.instructions.slice()),
                    .origin = bounded.origin,
                    .side_set = bounded.side_set,
                    .wrap_target = bounded.wrap_target,
                    .wrap = bounded.wrap,
                };
            }
        };

        fn init(tokens: []const Token) Self {
            return Self{
                .output = Self.Output{
                    .global_defines = BoundedDefines.init(0) catch unreachable,
                    .private_defines = BoundedDefines.init(0) catch unreachable,
                    .programs = BoundedPrograms.init(0) catch unreachable,
                },
                .tokens = tokens,
                .index = 0,
            };
        }

        fn peek_token(self: Self) ?Token {
            return if (self.index < self.tokens.len)
                self.tokens[self.index]
            else
                null;
        }

        fn get_token(self: *Self) ?Token {
            return if (self.peek_token()) |token| blk: {
                self.consume(1);
                break :blk token;
            } else null;
        }

        fn consume(self: *Self, count: u32) void {
            self.index += count;
        }

        fn evaluate_impl(
            self: *Self,
            comptime T: type,
            define_lists: []const *const BoundedDefines,
            value: Token.Value,
        ) !T {
            _ = self;
            _ = define_lists;
            return switch (value) {
                .integer => |int| @intCast(T, int),
                .string => {
                    // look up symbol in defines
                    return error.TODO;
                },
                else => error.TODO,
            };
        }

        fn evaluate(
            self: *Self,
            comptime T: type,
            program: BoundedProgram,
            value: Token.Value,
        ) !T {
            return self.evaluate_impl(T, &.{
                &self.output.global_defines,
                &self.output.private_defines,
                &program.defines,
                &program.private_defines,
            }, value);
        }

        fn evaluate_target(
            self: *Self,
            program: BoundedProgram,
            value: Token.Value,
        ) !u5 {
            return switch (value) {
                .integer, .expression => try self.evaluate(u5, program, value),
                // the symbol looking up a string means checking labels
                .string => |str| for (program.labels.slice()) |label| {
                    if (std.mem.eql(u8, str, label.name))
                        break label.index;
                } else error.LabelNotFound,
            };
        }

        fn evaluate_global(self: *Self, comptime T: type, value: Token.Value) !T {
            return self.evaluate_impl(T, &.{
                &self.output.global_defines,
                &self.output.private_defines,
            }, value);
        }

        fn encode_globals(self: *Self) !void {
            assert(self.index == 0);

            // read defines until program is had
            while (self.peek_token()) |token| switch (token.data) {
                .define => |define| {
                    const result = assembler.Define{
                        .name = define.name,
                        .value = try self.evaluate_global(u32, define.value),
                    };

                    if (define.public)
                        try self.output.global_defines.append(result)
                    else
                        try self.output.private_defines.append(result);

                    self.consume(1);
                },
                .program => break,
                else => return error.InvalidTokenInGlobalSpace,
            };
        }

        fn encode_program_init(self: *Self, program: *BoundedProgram) !void {
            while (self.peek_token()) |token| {
                switch (token.data) {
                    .program, .label, .instruction, .word, .wrap_target => break,
                    .define => |define| {
                        const result = assembler.Define{
                            .name = define.name,
                            .value = try self.evaluate(u32, program.*, define.value),
                        };

                        if (define.public)
                            try program.defines.append(result)
                        else
                            try program.private_defines.append(result);

                        self.consume(1);
                    },
                    .origin => |value| {
                        program.origin = try self.evaluate(u5, program.*, value);
                        self.consume(1);
                    },
                    .side_set => |side_set| {
                        program.side_set = .{
                            .count = try self.evaluate(u3, program.*, side_set.count),
                            .optional = side_set.opt,
                            .pindirs = side_set.pindirs,
                        };
                        self.consume(1);
                    },
                    // ignore
                    .lang_opt => {},
                    .wrap => unreachable,
                }
            }
        }

        fn encode_instruction(
            self: *Self,
            program: *BoundedProgram,
            token: Token.Instruction,
            token_index: u32,
            diags: *?Diagnostics,
        ) !void {
            // guaranteed to be an instruction variant
            const payload: Instruction.Payload = switch (token.payload) {
                .nop => .{
                    .mov = .{
                        .destination = .y,
                        .operation = .none,
                        .source = .y,
                    },
                },
                .jmp => |jmp| .{
                    .jmp = .{
                        .condition = jmp.condition,
                        .address = try self.evaluate_target(program.*, jmp.target),
                    },
                },
                .wait => |wait| .{
                    .wait = .{
                        .polarity = wait.polarity,
                        .source = wait.source,
                        .index = try self.evaluate(u5, program.*, wait.num),
                    },
                },
                .in => |in| .{
                    .in = .{
                        .source = in.source,
                        .bit_count = in.bit_count,
                    },
                },
                .out => |out| .{
                    .out = .{
                        .destination = out.destination,
                        .bit_count = out.bit_count,
                    },
                },
                .push => |push| .{
                    .push = .{
                        .if_full = @boolToInt(push.iffull),
                        .block = @boolToInt(push.block),
                    },
                },
                .pull => |pull| .{
                    .pull = .{
                        .if_empty = @boolToInt(pull.ifempty),
                        .block = @boolToInt(pull.block),
                    },
                },
                .mov => |mov| .{
                    .mov = .{
                        .destination = mov.destination,
                        .operation = mov.operation,
                        .source = mov.source,
                    },
                },
                .irq => |irq| .{
                    .irq = .{
                        // TODO: what to do with rel?
                        .clear = @boolToInt(irq.clear),
                        .wait = @boolToInt(irq.wait),
                        .index = irq.num,
                    },
                },
                .set => |set| .{
                    .set = .{
                        // TODO: rel?
                        .destination = set.destination,
                        .data = try self.evaluate(u5, program.*, set.value),
                    },
                },
            };

            const tag: Instruction.Tag = switch (token.payload) {
                .nop => .mov,
                .jmp => .jmp,
                .wait => .wait,
                .in => .in,
                .out => .out,
                .push => .push_pull,
                .pull => .push_pull,
                .mov => .mov,
                .irq => .irq,
                .set => .set,
            };

            if (program.side_set) |side_set| {
                if (!side_set.optional and token.side_set == null) {
                    diags.* = Diagnostics.init(token_index, "'side' must be specified for this instruction because 'opt' was not specied in the .side_set directive", .{});
                    return error.InvalidSideSet;
                }
            } else {
                if (token.side_set != null) {
                    diags.* = Diagnostics.init(token_index, ".side_set directive must be specified for program to use side_set", .{});
                    return error.InvalidSideSet;
                }
            }

            const delay_side_set = try calc_delay_side_set(
                program.side_set,
                token.side_set,
                token.delay,
                diags,
            );

            try program.instructions.append(Instruction{
                .tag = tag,
                .payload = payload,
                .delay_side_set = delay_side_set,
            });
        }

        fn calc_delay_side_set(
            program_settings: ?SideSet,
            side_set_opt: ?u5,
            delay_opt: ?u5,
            diags: *?Diagnostics,
        ) !u5 {
            // TODO: errors for too big values
            _ = diags;
            // errors for side set and delay sizes
            const delay: u5 = if (delay_opt) |delay| delay else 0;
            return if (program_settings) |settings|
                if (settings.optional)
                    if (side_set_opt) |side_set|
                        0x10 | (side_set << @as(u3, 4) - settings.count) | delay
                    else
                        delay
                else
                    (side_set_opt.? << @as(u3, 5) - settings.count) | delay
            else
                delay;
        }

        fn encode_instruction_body(self: *Self, program: *BoundedProgram, diags: *?Diagnostics) !void {
            // first scan through body for labels
            var instr_index: u5 = 0;
            for (self.tokens[self.index..]) |token| {
                switch (token.data) {
                    .label => |label| try program.labels.append(.{
                        .name = label.name,
                        .public = label.public,
                        .index = instr_index,
                    }),
                    .instruction, .word => instr_index += 1,
                    .wrap_target => {
                        if (program.wrap_target != null) {
                            diags.* = Diagnostics.init(token.index, "wrap_target already set for this program", .{});
                            return error.WrapTargetAlreadySet;
                        }

                        program.wrap_target = instr_index;
                    },
                    .wrap => {
                        if (program.wrap != null) {
                            diags.* = Diagnostics.init(token.index, "wrap already set for this program", .{});
                            return error.WrapAlreadySet;
                        }

                        program.wrap = instr_index - 1;
                    },
                    .program => break,
                    else => unreachable, // invalid
                }
            }

            // encode instructions, labels will be populated
            for (self.tokens[self.index..], self.index..) |token, i| {
                switch (token.data) {
                    .instruction => |instr| try self.encode_instruction(program, instr, token.index, diags),
                    .word => |word| try program.instructions.append(
                        @bitCast(Instruction, try self.evaluate(u16, program.*, word)),
                    ),
                    // already processed
                    .label, .wrap_target, .wrap => {},
                    .program => {
                        self.index = @intCast(u32, i);
                        break;
                    },

                    else => unreachable, // invalid
                }
            } else if (self.tokens.len > 0)
                self.index = @intCast(u32, self.tokens.len);
        }

        fn encode_program(self: *Self, diags: *?Diagnostics) !?BoundedProgram {
            const program_token = self.get_token() orelse return null;
            if (program_token.data != .program)
                return error.ExpectedProgramToken;

            var program = BoundedProgram{
                .name = program_token.data.program,
                .defines = BoundedDefines.init(0) catch unreachable,
                .private_defines = BoundedDefines.init(0) catch unreachable,
                .instructions = BoundedInstructions.init(0) catch unreachable,
                .labels = BoundedLabels.init(0) catch unreachable,
                .side_set = null,
                .origin = null,
                .wrap_target = null,
                .wrap = null,
            };

            try self.encode_program_init(&program);
            try self.encode_instruction_body(&program, diags);

            return program;
        }

        fn encode_output(self: *Self, diags: *?Diagnostics) !Self.Output {
            try self.encode_globals();

            while (try self.encode_program(diags)) |program|
                try self.output.programs.append(program);

            return self.output;
        }
    };
}

pub const Instruction = packed struct(u16) {
    payload: Payload,
    delay_side_set: u5,
    tag: Tag,

    pub const Payload = packed union {
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

    pub const Tag = enum(u3) {
        jmp,
        wait,
        in,
        out,
        push_pull,
        mov,
        irq,
        set,
    };

    pub const Jmp = packed struct(u8) {
        address: u5,
        condition: Token.Instruction.Jmp.Condition,
    };

    pub const Wait = packed struct(u8) {
        index: u5,
        source: Token.Instruction.Wait.Source,
        polarity: u1,
    };

    pub const In = packed struct(u8) {
        bit_count: u5,
        source: Token.Instruction.In.Source,
    };

    pub const Out = packed struct(u8) {
        bit_count: u5,
        destination: Token.Instruction.Out.Destination,
    };

    pub const Push = packed struct(u8) {
        _reserved0: u5 = 0,
        block: u1,
        if_full: u1,
        _reserved1: u1 = 0,
    };

    pub const Pull = packed struct(u8) {
        _reserved0: u5 = 0,
        block: u1,
        if_empty: u1,
        _reserved1: u1 = 1,
    };

    pub const Mov = packed struct(u8) {
        source: Token.Instruction.Mov.Source,
        operation: Token.Instruction.Mov.Operation,
        destination: Token.Instruction.Mov.Destination,
    };

    pub const Irq = packed struct(u8) {
        index: u5,
        wait: u1,
        clear: u1,
        reserved: u1 = 0,
    };

    pub const Set = packed struct(u8) {
        data: u5,
        destination: Token.Instruction.Set.Destination,
    };
};

//==============================================================================
// Encoder Tests
//==============================================================================

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const expectEqualStrings = std.testing.expectEqualStrings;

fn encode_bounded_output_impl(source: []const u8, diags: *?assembler.Diagnostics) !Encoder(.{}).Output {
    const tokens = try tokenizer.tokenize(source, diags, .{});
    var encoder = Encoder(.{}).init(tokens.slice());
    return try encoder.encode_output(diags);
}

fn encode_bounded_output(source: []const u8) !Encoder(.{}).Output {
    var diags: ?assembler.Diagnostics = null;
    return encode_bounded_output_impl(source, &diags) catch |err| if (diags) |d| blk: {
        std.log.err("error at index {}: {s}", .{ d.index, d.message.slice() });
        break :blk err;
    } else err;
}

test "encode.define" {
    const output = try encode_bounded_output(".define foo 5");

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 1), output.private_defines.len);
    try expectEqual(@as(usize, 0), output.programs.len);

    try expectEqualStrings("foo", output.private_defines.get(0).name);
    try expectEqual(@as(u32, 5), output.private_defines.get(0).value);
}

test "encode.define.public" {
    const output = try encode_bounded_output(".define PUBLIC foo 5");

    try expectEqual(@as(usize, 1), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 0), output.programs.len);
}

test "encode.program.empty" {
    const output = try encode_bounded_output(".program arst");

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    try expectEqualStrings("arst", output.programs.get(0).name);
    try expectEqual(@as(usize, 0), output.programs.get(0).instructions.len);
}

test "encode.program.define" {
    const output = try encode_bounded_output(
        \\.program arst
        \\.define bruh 7
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    const program = output.programs.get(0);
    try expectEqualStrings("arst", program.name);
    try expectEqual(@as(usize, 0), program.instructions.len);

    const define = program.private_defines.get(0);
    try expectEqualStrings("bruh", define.name);
    try expectEqual(@as(u32, 7), define.value);
}

test "encode.program.define.public" {
    const output = try encode_bounded_output(
        \\.program arst
        \\.define public bruh 7
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    const program = output.programs.get(0);
    try expectEqualStrings("arst", program.name);
    try expectEqual(@as(usize, 0), program.instructions.len);

    const define = program.defines.get(0);
    try expectEqualStrings("bruh", define.name);
    try expectEqual(@as(u32, 7), define.value);
}

test "encode.program.define.namespaced" {
    const output = try encode_bounded_output(
        \\.program arst
        \\.define public bruh 7
        \\.program what
        \\.define public hi 8
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 2), output.programs.len);

    const program_arst = output.programs.get(0);
    try expectEqualStrings("arst", program_arst.name);
    try expectEqual(@as(usize, 0), program_arst.instructions.len);

    const define_bruh = program_arst.defines.get(0);
    try expectEqualStrings("bruh", define_bruh.name);
    try expectEqual(@as(u32, 7), define_bruh.value);

    const program_what = output.programs.get(1);
    try expectEqualStrings("what", program_what.name);
    try expectEqual(@as(usize, 0), program_what.instructions.len);

    const define_hi = program_what.defines.get(0);
    try expectEqualStrings("hi", define_hi.name);
    try expectEqual(@as(u32, 8), define_hi.value);
}

test "encode.origin" {
    const output = try encode_bounded_output(
        \\.program arst
        \\.origin 0
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    const program = output.programs.get(0);
    try expectEqualStrings("arst", program.name);
    try expectEqual(@as(usize, 0), program.instructions.len);

    try expectEqual(@as(?u5, 0), program.origin);
}

test "encode.wrap_target" {
    const output = try encode_bounded_output(
        \\.program arst
        \\nop
        \\.wrap_target
        \\nop
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    const program = output.programs.get(0);
    try expectEqualStrings("arst", program.name);
    try expectEqual(@as(usize, 2), program.instructions.len);

    try expectEqual(@as(?u5, 1), program.wrap_target);
}

test "encode.wrap" {
    const output = try encode_bounded_output(
        \\.program arst
        \\nop
        \\.wrap
        \\nop
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    const program = output.programs.get(0);
    try expectEqualStrings("arst", program.name);
    try expectEqual(@as(usize, 2), program.instructions.len);

    try expectEqual(@as(?u5, 0), program.wrap);
}

test "encode.side_set" {
    const output = try encode_bounded_output(
        \\.program arst
        \\.side_set 1
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    const program = output.programs.get(0);
    try expectEqual(@as(?u5, 1), program.side_set.?.count);
}

test "encode.side_set.opt" {
    const output = try encode_bounded_output(
        \\.program arst
        \\.side_set 1 opt
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    const program = output.programs.get(0);
    try expectEqual(@as(?u5, 1), program.side_set.?.count);
    try expect(program.side_set.?.optional);
}

test "encode.side_set.pindirs" {
    const output = try encode_bounded_output(
        \\.program arst
        \\.side_set 1 pindirs
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    const program = output.programs.get(0);
    try expectEqual(@as(?u5, 1), program.side_set.?.count);
    try expect(program.side_set.?.pindirs);
}

test "encode.label" {
    const output = try encode_bounded_output(
        \\.program arst
        \\nop
        \\my_label:
        \\nop
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    const program = output.programs.get(0);
    try expectEqual(@as(usize, 1), program.labels.len);

    const label = program.labels.get(0);
    try expectEqualStrings("my_label", label.name);
    try expectEqual(@as(u32, 1), label.index);
    try expectEqual(false, label.public);
}

test "encode.label.public" {
    const output = try encode_bounded_output(
        \\.program arst
        \\nop
        \\nop
        \\public my_label:
        \\nop
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    const program = output.programs.get(0);
    try expectEqual(@as(usize, 1), program.labels.len);

    const label = program.labels.get(0);
    try expectEqualStrings("my_label", label.name);
    try expectEqual(@as(u32, 2), label.index);
    try expectEqual(true, label.public);
}

test "encode.side_set.bits" {
    const output = try encode_bounded_output(
        \\.program arst
        \\.side_set 1 opt
        \\nop side 1
        \\nop [1]
        \\nop side 0 [1]
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    const program = output.programs.get(0);

    const instr0 = program.instructions.get(0);
    try expectEqual(@as(u5, 0x18), instr0.delay_side_set);

    const instr1 = program.instructions.get(1);
    try expectEqual(@as(u5, 0x1), instr1.delay_side_set);

    const instr2 = program.instructions.get(2);
    try expectEqual(@as(u5, 0x11), instr2.delay_side_set);
}

//test "encode.evaluate.target" {
//    return error.TODO;
//}
//test "encode.evaluate.global" {
//    return error.TODO;
//}
//test "encode.evaluate.integer" {
//    return error.TODO;
//}
//test "encode.evaluate.addition" {
//    return error.TODO;
//}
//test "encode.evaluate.subtraction" {
//    return error.TODO;
//}
//test "encode.evaluate.multiplication" {
//    return error.TODO;
//}
//test "encode.evaluate.division" {
//    return error.TODO;
//}
//test "encode.evaluate.inversion" {
//    return error.TODO;
//}
//test "encode.evaluate.bit reversal" {
//    return error.TODO;
//}
//test "encode.evaluate.nested expression" {
//    return error.TODO;
//}
//
test "encode.jmp.label" {
    const output = try encode_bounded_output(
        \\.program arst
        \\nop
        \\my_label:
        \\nop
        \\nop
        \\jmp my_label
    );

    try expectEqual(@as(usize, 0), output.global_defines.len);
    try expectEqual(@as(usize, 0), output.private_defines.len);
    try expectEqual(@as(usize, 1), output.programs.len);

    const program = output.programs.get(0);
    try expectEqual(@as(usize, 1), program.labels.len);

    const label = program.labels.get(0);
    try expectEqualStrings("my_label", label.name);
    try expectEqual(@as(u32, 1), label.index);
    try expectEqual(false, label.public);

    const instr = program.instructions.get(3);
    try expectEqual(Instruction.Tag.jmp, instr.tag);
    try expectEqual(@as(u5, 0), instr.delay_side_set);
    try expectEqual(Token.Instruction.Jmp.Condition.always, instr.payload.jmp.condition);
    try expectEqual(@as(u5, 1), instr.payload.jmp.address);
}

//test "encode.error.duplicated program name" {}
//test "encode.error.duplicated define" {}
//test "encode.error.multiple side_set" {}
//test "encode.error.label with no instruction" {}
//test "encode.error.label with no instruction" {}

// Test Plan
// =========
//
// - .program name validation
// - .origin in program init and in program body
// - .side_set must be in the program init
// - .wrap_target must come before an instruction, defaults to start of a program
// - .wrap_target must only be used once in a program
// - .wrap must be placed after an instruction, defaults to end of a program
// - .wrap must only be used once in a program
// -
//
