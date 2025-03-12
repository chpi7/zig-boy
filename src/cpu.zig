const std = @import("std");
const sys = @import("sys.zig");
const Bus = sys.Bus;

const decoder = @import("instructions_generated.zig");
const Instruction = decoder.Instruction;
const OpType = decoder.OpType;
const Operand = decoder.Operand;
const OperandType = decoder.OperandType;
const RegName = decoder.RegisterName;

const RegFile = struct {
    const F = enum { Z, N, H, C };

    AF: u16,
    BC: u16,
    DE: u16,
    HL: u16,
    SP: u16,
    PC: u16,

    fn get(self: *RegFile, n: RegName, comptime t: type) t {
        return switch (n) {
            RegName.BC => @as(t, self.BC),
            RegName.DE => @as(t, self.DE),
            RegName.HL => @as(t, self.HL),
            RegName.SP => @as(t, self.SP),
            RegName.A => @as(t, self.get_hi("AF")),
            RegName.B => @as(t, self.get_hi("BC")),
            RegName.C => @as(t, self.get_lo("BC")),
            RegName.D => @as(t, self.get_hi("DE")),
            RegName.E => @as(t, self.get_lo("DE")),
            RegName.H => @as(t, self.get_hi("HL")),
            RegName.L => @as(t, self.get_lo("HL")),
            else => unreachable, // dont allow getting non GP regs
        };
    }

    fn get_hi(self: *RegFile, comptime x: []const u8) u8 {
        return @intCast(@field(self, x) >> 8);
    }

    fn get_lo(self: *RegFile, comptime x: []const u8) u8 {
        return @truncate(@field(self, x));
    }

    fn set_hi(self: *RegFile, v: u16, comptime x: []const u8) void {
        @field(self, x) = (@field(self, x) & 0x00FF) | (v << 8);
    }

    fn set_lo(self: *RegFile, v: u16, comptime x: []const u8) void {
        @field(self, x) = (@field(self, x) & 0xFF00) | v;
    }

    fn flag_set(self: *RegFile, comptime v: bool, comptime flag: F) void {
        const mask = flag_to_mask(flag);
        if (v) {
            self.AF |= mask;
        } else {
            self.AF &= ~mask;
        }
    }

    fn flag_get(self: *RegFile, comptime flag: F) bool {
        const mask = flag_to_mask(flag);
        return (self.AF & mask) != 0;
    }

    fn flag_to_mask(comptime flag: F) u8 {
        return switch (flag) {
            F.Z => 0b1000_0000,
            F.N => 0b0100_0000,
            F.H => 0b0010_0000,
            F.C => 0b0001_1000,
        };
    }
};

pub const Cpu = struct {
    rf: RegFile,
    bus: *Bus,

    fn decode_next(self: *Cpu) Instruction {
        const decode_result = decoder.decode_instruction(self.bus.read(self.rf.PC));
        return if (decode_result.op != OpType.PREFIX) decode_result else decoder.decode_instruction_cb(self.bus.read(self.rf.PC + 1));
    }

    fn decode_imm(o: *Operand, instr_mem: [*]const u8) void {
        if (o.t == OperandType.unused) return;

        o.value = switch (o.t) {
            OperandType.imm16 => @as(u16, instr_mem[1]) | (@as(u16, instr_mem[2]) << 8), // load as little endian
            OperandType.imm8 => instr_mem[1],
            OperandType.vec => 0, // TODO
            else => o.value, // nothing to change
        };
        // relative handling per instruction!
    }

    fn execute_instruction(self: *Cpu) void {
        const instr = self.decode_next();
        for (instr.operands) |operand| {
            decode_imm(&operand, self.bus.read_ptr(self.rf.PC));
        }
        self.rf.PC += instr.bytes; // this accounts for prefix + immediates

        switch (instr.op) {
            OpType.ADC => unreachable,
            OpType.ADD => unreachable,
            OpType.AND => unreachable,
            OpType.CALL => unreachable,
            OpType.CCF => unreachable,
            OpType.CP => unreachable,
            OpType.CPL => unreachable,
            OpType.DAA => unreachable,
            OpType.DEC => unreachable,
            OpType.DI => unreachable,
            OpType.EI => unreachable,
            OpType.HALT => unreachable,
            OpType.ILLEGAL => unreachable,
            OpType.INC => unreachable,
            OpType.JP => unreachable,
            OpType.JR => unreachable,
            OpType.LD => self.do_ld(instr, instr.operands[0], instr.operands[1]),
            OpType.LDH => unreachable,
            OpType.NOP => unreachable,
            OpType.OR => unreachable,
            OpType.POP => unreachable,
            OpType.PREFIX => unreachable,
            OpType.PUSH => unreachable,
            OpType.RET => unreachable,
            OpType.RETI => unreachable,
            OpType.RLA => unreachable,
            OpType.RLCA => unreachable,
            OpType.RRA => unreachable,
            OpType.RRCA => unreachable,
            OpType.RST => unreachable,
            OpType.SBC => unreachable,
            OpType.SCF => unreachable,
            OpType.STOP => unreachable,
            OpType.SUB => unreachable,
            OpType.XOR => unreachable,
        }
    }

    fn do_ld(self: *Cpu, _: Instruction, _: Operand, src: Operand) void {
        var value: u16 = 0;
        if (src.t == OperandType.reg8) {
            value = self.rf.get(src.register, u8);
        } else if (src.t == OperandType.reg16) {
            value = self.rf.get(src.register, u16);
        } else {
            unreachable; // TODO
        }
    }
};

// --------------------------------------------

const testing = std.testing;

test "Register Hi Lo Combined" {
    comptime var r = RegFile{ .AF = 0, .BC = 0, .DE = 0, .HL = 0, .SP = 0, .PC = 1 };

    comptime for ([_][]const u8{ "AF", "BC", "DE", "HL", "SP", "PC" }) |elem| {
        @field(r, elem) = 0xABCD;
        try testing.expectEqual(0xAB, r.get_hi(elem));
        try testing.expectEqual(0xCD, r.get_lo(elem));

        r.set_lo(127, elem);
        r.set_hi(5, elem);
        try testing.expectEqual(127, r.get_lo(elem));
        try testing.expectEqual(5, r.get_hi(elem));
    };
}

test "Register Flags" {
    var r = RegFile{ .AF = 0, .BC = 0, .DE = 0, .HL = 0, .SP = 0, .PC = 1 };

    try testing.expectEqual(false, r.flag_get(RegFile.F.Z));

    r.flag_set(true, RegFile.F.Z);
    try testing.expectEqual(true, r.flag_get(RegFile.F.Z));

    r.flag_set(false, RegFile.F.Z);
    try testing.expectEqual(false, r.flag_get(RegFile.F.Z));
}

test "Instruction Decode" {
    const i = decoder.decode_instruction(0x00);
    try testing.expectEqual(decoder.OpType.NOP, i.op);
    try testing.expectEqual(0, i.num_operands);
    std.log.debug("{}", .{i});
}

test "Operand Load Immediate" {
    const progmem = [_]u8{ 0xab, 0xcd, 0xef };

    var imm16 = Operand{ .t = OperandType.imm16 };
    Cpu.decode_imm(&imm16, &progmem);
    try testing.expectEqual(0xefcd, imm16.value);

    var imm8 = Operand{ .t = OperandType.imm8 };
    Cpu.decode_imm(&imm8, &progmem);
    try testing.expectEqual(0xcd, imm8.value);
}
