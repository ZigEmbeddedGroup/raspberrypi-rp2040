const std = @import("std");
const pio = @import("../pio.zig");
const c = @cImport({
    @cDefine("PICO_NO_HARDWARE", "1");
    @cInclude("stdint.h");
    @cInclude("test/addition.pio.h");
    @cInclude("test/apa102.pio.h");
    @cInclude("test/blink.pio.h");
    @cInclude("test/clocked_input.pio.h");
    @cInclude("test/differential_manchester.pio.h");
    @cInclude("test/hello.pio.h");
    @cInclude("test/hub75.pio.h");
    @cInclude("test/i2c.pio.h");
    @cInclude("test/manchester_encoding.pio.h");
    @cInclude("test/nec_carrier_burst.pio.h");
    @cInclude("test/nec_carrier_control.pio.h");
    @cInclude("test/nec_receive.pio.h");
    @cInclude("test/pio_serialiser.pio.h");
    @cInclude("test/pwm.pio.h");
    @cInclude("test/quadrature_encoder.pio.h");
    @cInclude("test/resistor_dac.pio.h");
    @cInclude("test/spi.pio.h");
    @cInclude("test/squarewave.pio.h");
    @cInclude("test/squarewave_fast.pio.h");
    @cInclude("test/squarewave_wrap.pio.h");
    @cInclude("test/st7789_lcd.pio.h");
    @cInclude("test/uart_rx.pio.h");
    @cInclude("test/uart_tx.pio.h");
    @cInclude("test/ws2812.pio.h");
});

fn pio_comparison(comptime source: []const u8) !void {
    const bytecode = comptime try pio.assemble(source);
    try std.testing.expect(bytecode.programs.len > 0);

    inline for (bytecode.programs) |program| {
        const expected_insns = @field(c, program.name ++ "_program_instructions");
        for (program.instructions.slice(), expected_insns) |actual, expected|
            try std.testing.expectEqual(expected, actual);
    }
}

test "pio.comparison.addition" {
    comptime {
        try pio_comparison(@embedFile("test/addition.pio"));
    }
}

test "pio.comparison.apa102" {
    comptime {
        try pio_comparison(@embedFile("test/apa102.pio"));
    }
}

test "pio.comparison.blink" {
    comptime {
        try pio_comparison(@embedFile("test/blink.pio"));
    }
}

test "pio.comparison.clocked_input" {
    comptime {
        try pio_comparison(@embedFile("test/clocked_input.pio"));
    }
}

test "pio.comparison.differential_manchester" {
    comptime {
        try pio_comparison(@embedFile("test/differential_manchester.pio"));
    }
}

test "pio.comparison.hello" {
    comptime {
        try pio_comparison(@embedFile("test/hello.pio"));
    }
}

test "pio.comparison.hub75" {
    comptime {
        try pio_comparison(@embedFile("test/hub75.pio"));
    }
}

test "pio.comparison.i2c" {
    comptime {
        try pio_comparison(@embedFile("test/i2c.pio"));
    }
}

test "pio.comparison.manchester_encoding" {
    comptime {
        try pio_comparison(@embedFile("test/manchester_encoding.pio"));
    }
}

test "pio.comparison.nec_carrier_burst" {
    comptime {
        try pio_comparison(@embedFile("test/nec_carrier_burst.pio"));
    }
}

test "pio.comparison.nec_carrier_control" {
    comptime {
        try pio_comparison(@embedFile("test/nec_carrier_control.pio"));
    }
}

test "pio.comparison.nec_receive" {
    comptime {
        try pio_comparison(@embedFile("test/nec_receive.pio"));
    }
}

test "pio.comparison.pio_serialiser" {
    comptime {
        try pio_comparison(@embedFile("test/pio_serialiser.pio"));
    }
}

test "pio.comparison.pwm" {
    comptime {
        try pio_comparison(@embedFile("test/pwm.pio"));
    }
}

test "pio.comparison.quadrature_encoder" {
    comptime {
        try pio_comparison(@embedFile("test/quadrature_encoder.pio"));
    }
}

test "pio.comparison.resistor_dac" {
    comptime {
        try pio_comparison(@embedFile("test/resistor_dac.pio"));
    }
}

test "pio.comparison.spi" {
    comptime {
        try pio_comparison(@embedFile("test/spi.pio"));
    }
}

test "pio.comparison.squarewave" {
    comptime {
        try pio_comparison(@embedFile("test/squarewave.pio"));
    }
}

test "pio.comparison.squarewave_fast" {
    comptime {
        try pio_comparison(@embedFile("test/squarewave_fast.pio"));
    }
}

test "pio.comparison.squarewave_wrap" {
    comptime {
        try pio_comparison(@embedFile("test/squarewave_wrap.pio"));
    }
}

test "pio.comparison.st7789_lcd" {
    comptime {
        try pio_comparison(@embedFile("test/st7789_lcd.pio"));
    }
}

test "pio.comparison.uart_rx" {
    comptime {
        try pio_comparison(@embedFile("test/uart_rx.pio"));
    }
}

test "pio.comparison.uart_tx" {
    comptime {
        try pio_comparison(@embedFile("test/uart_tx.pio"));
    }
}

test "pio.comparison.ws2812" {
    comptime {
        try pio_comparison(@embedFile("test/ws2812.pio"));
    }
}
