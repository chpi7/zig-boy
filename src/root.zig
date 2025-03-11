const std = @import("std");
const decoder = @import("instructions_generated.zig");

const Instruction = decoder.Instruction;
const OpType = decoder.OpType;
const Operand = decoder.Operand;

const F = enum { Z, N, H, C };

const Registers = struct {
    AF: u16,
    BC: u16,
    DE: u16,
    HL: u16,
    SP: u16,
    PC: u16,
};

fn reg_get_hi(regs: Registers, comptime x: []const u8) u8 {
    return @intCast(@field(regs, x) >> 8);
}

fn reg_get_lo(regs: Registers, comptime x: []const u8) u8 {
    return @truncate(@field(regs, x));
}

fn reg_set_hi(regs: *Registers, v: u16, comptime x: []const u8) void {
    @field(regs, x) = (@field(regs, x) & 0x00FF) | (v << 8);
}

fn reg_set_lo(regs: *Registers, v: u16, comptime x: []const u8) void {
    @field(regs, x) = (@field(regs, x) & 0xFF00) | v;
}

fn flag_mask(comptime flag: F) u8 {
    return switch (flag) {
        F.Z => 0b1000_0000,
        F.N => 0b0100_0000,
        F.H => 0b0010_0000,
        F.C => 0b0001_1000,
    };
}

fn reg_flag_set(regs: *Registers, comptime v: bool, comptime flag: F) void {
    const mask = flag_mask(flag);
    if (v) {
        regs.AF |= mask;
    } else {
        regs.AF &= ~mask;
    }
}

fn reg_flag_get(regs: Registers, comptime flag: F) bool {
    const mask = flag_mask(flag);
    return (regs.AF & mask) != 0;
}

fn get_operand_value(operand: Operand) u8 {
    var v: u16 = switch (operand.t) {
        decoder.OperandType.reg8, decoder.OperandType.reg16 => get_reg(operand.register),
        else => unreachable, // TODO
    };

    if (operand.relative) {
        // TODO
    }
}

fn execute_instruction(mem: [*]u8) void {
    const pc: u16 = 0;
    const instruction = decoder.decode_instruction(mem[pc]);

    std.log.debug("{}", .{instruction});

    switch (instruction.op) {
        OpType.LD => // TODO
        else => unreachable, // TODO
    }
}

// --------------------------------------------

const testing = std.testing;

test "Register Hi Lo Combined" {
    comptime var r = Registers{ .AF = 0, .BC = 0, .DE = 0, .HL = 0, .SP = 0, .PC = 1 };

    comptime for ([_][]const u8{ "AF", "BC", "DE", "HL", "SP", "PC" }) |elem| {
        @field(r, elem) = 0xABCD;
        try testing.expectEqual(0xAB, reg_get_hi(r, elem));
        try testing.expectEqual(0xCD, reg_get_lo(r, elem));

        reg_set_lo(&r, 127, elem);
        reg_set_hi(&r, 5, elem);
        try testing.expectEqual(127, reg_get_lo(r, elem));
        try testing.expectEqual(5, reg_get_hi(r, elem));
    };
}

test "Register Flags" {
    var r = Registers{ .AF = 0, .BC = 0, .DE = 0, .HL = 0, .SP = 0, .PC = 1 };

    try testing.expectEqual(false, reg_flag_get(r, F.Z));

    reg_flag_set(&r, true, F.Z);
    try testing.expectEqual(true, reg_flag_get(r, F.Z));

    reg_flag_set(&r, false, F.Z);
    try testing.expectEqual(false, reg_flag_get(r, F.Z));
}

test "Instruction Decode" {
    const i = decoder.decode_instruction(0x00);
    try testing.expectEqual(decoder.OpType.NOP, i.op);
    try testing.expectEqual(0, i.num_operands);
    std.log.debug("{}", .{i});
}
