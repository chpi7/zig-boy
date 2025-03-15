const std = @import("std");
const sys = @import("sys.zig");
const Bus = sys.Bus;

pub const decoder = @import("instructions_generated.zig");
const Instruction = decoder.Instruction;
const OpType = decoder.OpType;
const Operand = decoder.Operand;
const OperandType = decoder.OperandType;
const RegName = decoder.Register;

const RegFile = struct {
    AF: u16 = 0,
    BC: u16 = 0,
    DE: u16 = 0,
    HL: u16 = 0,
    SP: u16 = 0,
    PC: u16 = 0,

    fn get(self: *RegFile, n: RegName, comptime t: type) t {
        if (t == u8) {
            return switch (n) {
                RegName.A => @as(t, RegFile.get_hi(self.AF)),
                RegName.F => @as(t, RegFile.get_lo(self.AF)),
                RegName.B => @as(t, RegFile.get_hi(self.BC)),
                RegName.C => @as(t, RegFile.get_lo(self.BC)),
                RegName.D => @as(t, RegFile.get_hi(self.DE)),
                RegName.E => @as(t, RegFile.get_lo(self.DE)),
                RegName.H => @as(t, RegFile.get_hi(self.HL)),
                RegName.L => @as(t, RegFile.get_lo(self.HL)),
                else => unreachable,
            };
        } else if (t == u16) {
            return switch (n) {
                RegName.AF => @as(t, self.AF),
                RegName.BC => @as(t, self.BC),
                RegName.DE => @as(t, self.DE),
                RegName.HL => @as(t, self.HL),
                RegName.SP => @as(t, self.SP),
                else => unreachable,
            };
        } else {
            unreachable;
        }
    }

    fn set(self: *RegFile, comptime t: type, n: RegName, value: t) void {
        self.get_ptr(n, t).* = value;
    }

    fn get_ptr(self: *RegFile, n: RegName, comptime t: type) *t {
        if (t == u8) {
            return switch (n) {
                RegName.A => RegFile.hi_ptr(&self.AF),
                RegName.B => RegFile.hi_ptr(&self.BC),
                RegName.D => RegFile.hi_ptr(&self.DE),
                RegName.H => RegFile.hi_ptr(&self.HL),
                RegName.F => RegFile.lo_ptr(&self.AF),
                RegName.C => RegFile.lo_ptr(&self.BC),
                RegName.E => RegFile.lo_ptr(&self.DE),
                RegName.L => RegFile.lo_ptr(&self.HL),
                else => unreachable,
            };
        } else if (t == u16) {
            return switch (n) {
                RegName.AF => &self.AF,
                RegName.BC => &self.BC,
                RegName.DE => &self.DE,
                RegName.HL => &self.HL,
                RegName.SP => &self.SP,
                else => unreachable,
            };
        } else {
            unreachable;
        }
    }

    inline fn hi_ptr(p: *u16) *u8 {
        const u8ptr: [*]u8 = @ptrCast(p);
        return @ptrCast(u8ptr + 1);
    }

    inline fn lo_ptr(p: *u16) *u8 {
        return @ptrCast(p);
    }

    inline fn get_hi(v: u16) u8 {
        return @intCast(v >> 8);
    }

    inline fn get_lo(v: u16) u8 {
        return @truncate(v);
    }

    fn set_hi(self: *RegFile, v: u16, comptime x: []const u8) void {
        @field(self, x) = (@field(self, x) & 0x00FF) | (v << 8);
    }

    fn set_lo(self: *RegFile, v: u16, comptime x: []const u8) void {
        @field(self, x) = (@field(self, x) & 0xFF00) | v;
    }

    fn flag_set(self: *RegFile, v: bool, comptime flag: F) void {
        const mask = flag_to_mask(flag);
        if (v) {
            self.AF |= mask;
        } else {
            self.AF &= ~mask;
        }
    }

    const F = enum { Z, N, H, C };

    fn flag_get(self: *RegFile, comptime flag: F) bool {
        const mask = flag_to_mask(flag);
        return (self.AF & mask) != 0;
    }

    fn flag_to_mask(comptime flag: F) u8 {
        return switch (flag) {
            F.Z => 0b1000,
            F.N => 0b0100,
            F.H => 0b0010,
            F.C => 0b0001,
        };
    }
};

pub const Cpu = struct {
    rf: RegFile = .{},
    bus: *Bus,

    fn decode_next(self: *Cpu) struct { Instruction, bool } {
        const decode_result = decoder.decode_instruction(self.bus.read(self.rf.PC));
        return if (decode_result.op != OpType.PREFIX) .{ decode_result, false } else .{ decoder.decode_instruction_cb(self.bus.read(self.rf.PC + 1)), true };
    }

    fn decode_imm(o: *Operand, instr_mem: [*]const u8) void {
        if (o.t == OperandType.unused) return;

        o.value = switch (o.t) {
            OperandType.imm16 => @as(u16, instr_mem[1]) | (@as(u16, instr_mem[2]) << 8), // load as little endian
            OperandType.imm8 => instr_mem[1],
            OperandType.vec => 0, // TODO
            else => o.value, // nothing to change
        };
    }

    fn execute_instruction(self: *Cpu) void {
        const instr, const is_cb = self.decode_next();
        for (instr.operands) |operand| {
            decode_imm(&operand, self.bus.read_ptr(self.rf.PC));
        }
        self.rf.PC += instr.bytes; // this accounts for prefix + immediates

        if (is_cb) {
            std.debug.print("[cpu] execute: {s}", .{decoder.opcode_to_str_cb(instr.opcode)});
        } else {
            std.debug.print("[cpu] execute: {s}", .{decoder.opcode_to_str(instr.opcode)});
        }

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
            OpType.LD => self.op_ld(instr, u8),
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

    pub fn op_ld(self: *Cpu, i: Instruction) void {
        if (i.operands[0].t == OperandType.reg16 and !i.operands[0].relative) {
            self.op_ld_dst_reg16(i);
        } else if (i.opcode == 0x08) {
            self.op_ld_sp_to_mem(i);
        } else {
            self.op_ld_dest_u8(i);
        }
    }

    fn op_ld_sp_to_mem(self: *Cpu, i: Instruction) void {
        const addr = i.operands[0].value;
        self.bus.write(addr, @truncate(self.rf.SP & 0x00ff));
        self.bus.write(addr + 1, @truncate(self.rf.SP >> 8));
    }

    fn op_ld_dst_reg16(self: *Cpu, i: Instruction) void {
        if (i.operands[1].t == OperandType.imm16) {
            self.rf.set(u16, i.operands[0].register, i.operands[1].value);
        } else if (i.opcode == 0xf8) {
            const offset: i16 = @bitCast(i.operands[2].value);
            if (offset < -128 or offset > 127) unreachable; // sanity check that the offset is i8
            // If, bits above [3, 7] are not set yet, and will be set afterwards, set the flag.
            const set_h = self.rf.SP <= 15 and offset > (15 - self.rf.SP);
            const set_c = self.rf.SP <= 255 and offset > (255 - self.rf.SP);

            std.log.debug("load special sp {}", .{self.rf.SP});

            self.rf.flag_set(set_h, RegFile.F.H);
            self.rf.flag_set(set_c, RegFile.F.C);
            if (offset >= 0) {
                self.rf.HL = self.rf.SP +% @as(u16, @bitCast(offset));
            } else {
                const tmp: u16 = @abs(offset);
                std.log.debug("tmp = {}", .{tmp});
                self.rf.HL = self.rf.SP -% tmp;
            }

            std.log.debug("load special offset {}", .{offset});
            std.log.debug("load special sp {}", .{self.rf.SP});
            std.log.debug("load special hl {}", .{self.rf.HL});
        } else if (i.operands[1].t == OperandType.reg16) {
            self.rf.set(u16, i.operands[0].register, self.rf.get(i.operands[1].register, u16));
        }
    }

    fn op_ld_dest_u8(self: *Cpu, i: Instruction) void {
        const dst = i.operands[0];
        const src = i.operands[1];

        var value: u8 = 0;
        if (src.relative) {
            std.debug.assert(src.t == OperandType.imm16 or src.t == OperandType.reg16);
            const addr = if (src.t == OperandType.imm16) src.value else self.rf.get(src.register, u16);
            value = self.bus.read(addr);
        } else {
            std.debug.assert(src.t == OperandType.imm8 or src.t == OperandType.reg8);
            value = if (src.t == OperandType.reg8) self.rf.get(src.register, u8) else @truncate(src.value);
        }

        if (dst.relative) {
            std.debug.assert(dst.t == OperandType.imm16 or dst.t == OperandType.reg16);
            const addr = if (dst.t == OperandType.imm16) dst.value else self.rf.get(dst.register, u16);
            self.bus.write(addr, value);
        } else {
            std.debug.assert(dst.t == OperandType.reg8);
            self.rf.set(u8, dst.register, value);
        }

        // post dec, inc does not affect flags
        switch (i.opcode) {
            0x22, 0x2a => { // [HL+]
                self.rf.HL +%= 1;
            },
            0x32, 0x3a => { // [HL-]
                self.rf.HL -%= 1;
            },
            else => {},
        }
    }
};

// --------------------------------------------

const testing = std.testing;

test "R/W GP Registers" {
    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    const R = RegName;

    for (
        [_]R{ R.AF, R.BC, R.DE, R.HL },
        [_]R{ R.A, R.B, R.D, R.H },
        [_]R{ R.F, R.C, R.E, R.L },
    ) |ab, a, b| {
        const ptr = cpu.rf.get_ptr(ab, u16);
        ptr.* = 0x1234;
        try testing.expectEqual(0x1234, cpu.rf.get(ab, u16));

        const ptr_hi = cpu.rf.get_ptr(a, u8);
        ptr_hi.* = 0x56;
        try testing.expectEqual(0x56, cpu.rf.get(a, u8));
        try testing.expectEqual(0x34, cpu.rf.get(b, u8));

        const ptr_lo = cpu.rf.get_ptr(b, u8);
        ptr_lo.* = 0x78;
        try testing.expectEqual(0x56, cpu.rf.get(a, u8));
        try testing.expectEqual(0x78, cpu.rf.get(b, u8));
    }
}

test "Instruction Decode" {
    const i = decoder.decode_instruction(0x00);
    try testing.expectEqual(decoder.OpType.NOP, i.op);
    try testing.expectEqual(0, i.num_operands);
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

// --------- loads into r8

test "LD r8 r8" {
    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    cpu.rf.get_ptr(RegName.B, u8).* = 0xab;

    const i = decoder.decode_instruction(0x78);
    cpu.op_ld(i);

    try testing.expectEqual(0xab, cpu.rf.get(RegName.A, u8));
}

test "LD r8 imm8" {
    const opcodes = [_]u8{ 0x0e, 0x1e, 0x2e, 0x3e };
    const value: u8 = 0xab;

    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    for (opcodes) |opcode| {
        var i = decoder.decode_instruction(opcode);
        i.operands[1].value = value;

        try testing.expectEqual(0, cpu.rf.get(i.operands[0].register, u8));
        cpu.op_ld(i);
        try testing.expectEqual(value, cpu.rf.get(i.operands[0].register, u8));
    }
}

test "LD r8 [r16]" {
    const opcodes = [_]u8{ 0x46, 0x56, 0x66, 0x4e, 0x5e, 0x6e };
    const value: u8 = 0xbe;
    const addr: u16 = 0x0000;

    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    for (opcodes) |opcode| {
        const i = decoder.decode_instruction(opcode);

        cpu.rf.set(u16, i.operands[1].register, addr);
        bus.write(addr, value);

        cpu.op_ld(i);
        try testing.expectEqual(value, cpu.rf.get(i.operands[0].register, u8));
    }
}

test "LD r8 [imm16]" {
    const opcodes = [_]u8{0xfa};
    const value: u8 = 0xbe;
    const addr: u16 = 0x0000;

    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    for (opcodes) |opcode| {
        var i = decoder.decode_instruction(opcode);

        i.operands[1].value = addr;
        bus.write(addr, value);

        cpu.op_ld(i);
        try testing.expectEqual(value, cpu.rf.get(i.operands[0].register, u8));
    }
}

// --------- loads into [r16]

test "LD [r16] imm8" {
    const opcodes = [_]u8{0x36};
    const value: u8 = 0xbe;
    const addr: u16 = 0x0000;

    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    for (opcodes) |opcode| {
        var i = decoder.decode_instruction(opcode);

        cpu.rf.set(u16, i.operands[0].register, addr);
        i.operands[1].value = value;

        cpu.op_ld(i);
        try testing.expectEqual(value, bus.read(addr));
    }
}

test "LD [r16], r8" {
    const opcodes = [_]u8{ 0x02, 0x12, 0x70, 0x71, 0x72, 0x73, 0x77 };
    const value: u8 = 0xbe;
    const addr: u16 = 0x0000;

    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    for (opcodes) |opcode| {
        const i = decoder.decode_instruction(opcode);
        cpu.rf.set(u8, i.operands[1].register, value);
        cpu.rf.set(u16, i.operands[0].register, addr);

        try testing.expectEqual(value, cpu.rf.get(i.operands[1].register, u8));
        try testing.expectEqual(addr, cpu.rf.get(i.operands[0].register, u16));

        bus.write(addr, 0x00);
        try testing.expectEqual(0, bus.read(addr));

        cpu.op_ld(i);
        try testing.expectEqual(value, bus.read(addr));
    }
}

test "LD [HL], r8 (HL overlap)" {
    const opcodes = [_]u8{ 0x74, 0x75 };
    const value: u8 = 0x00;
    const addr: u16 = 0x0000;

    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    for (opcodes) |opcode| {
        const i = decoder.decode_instruction(opcode);
        cpu.rf.set(u8, i.operands[1].register, value);
        cpu.rf.set(u16, i.operands[0].register, addr);

        bus.write(addr, 0xfe);
        try testing.expectEqual(0xfe, bus.read(addr));

        cpu.op_ld(i);
        try testing.expectEqual(value, bus.read(addr));
    }
}

// --------- loads into [imm16]

test "LD [imm16] A" {
    const opcode = 0xea;
    const value: u8 = 0xbe;
    const addr: u16 = 0x0000;

    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    var i = decoder.decode_instruction(opcode);
    i.operands[0].value = addr;
    cpu.rf.set(u8, i.operands[1].register, value);

    bus.write(addr, 0x00);
    try testing.expectEqual(0, bus.read(addr));

    cpu.op_ld(i);
    try testing.expectEqual(value, bus.read(addr));
}

test "LD [imm16] SP" {
    const opcode = 0x08;
    const value: u16 = 0xbeef;
    const addr: u16 = 0x0000;

    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    var i = decoder.decode_instruction(opcode);
    i.operands[0].value = addr;
    cpu.rf.set(u16, i.operands[1].register, value);

    bus.write(addr, 0x00);
    bus.write(addr + 1, 0x01);
    try testing.expectEqual(0, bus.read(addr));
    try testing.expectEqual(1, bus.read(addr + 1));

    cpu.op_ld(i);
    try testing.expectEqual(0xef, bus.read(addr));
    try testing.expectEqual(0xbe, bus.read(addr + 1));
}

// --------- loads into r16

test "LD r16 imm16" {
    const opcodes = [_]u8{ 0x01, 0x11, 0x21, 0x31 };
    const value: u16 = 0xbeef;

    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    for (opcodes) |opcode| {
        var i = decoder.decode_instruction(opcode);
        i.operands[1].value = value;

        try testing.expectEqual(0, cpu.rf.get(i.operands[0].register, u16));
        cpu.op_ld(i);
        try testing.expectEqual(value, cpu.rf.get(i.operands[0].register, u16));
    }
}

test "LD SP HL" {
    const opcode = 0xf9;
    const value: u16 = 0xbeef;

    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    const i = decoder.decode_instruction(opcode);
    cpu.rf.HL = value;
    cpu.rf.SP = 0;

    try testing.expectEqual(0x0000, cpu.rf.SP);

    cpu.op_ld(i);
    try testing.expectEqual(0xbeef, cpu.rf.SP);
}

test "LD HL SP+e8" {
    const Config = struct {
        base: u16,
        offset: i8,
        expect: u16,
        expect_h: bool = false,
        expect_c: bool = false,
    };

    const opcode = 0xf8;
    const configs = [_]Config{
        .{ .base = 1, .offset = 2, .expect = 3 },
        .{ .base = 0, .offset = -1, .expect = 0xffff },
        .{ .base = 0xfffe, .offset = -1, .expect = 0xfffd },
        .{ .base = 0xfffe, .offset = 3, .expect = 1 },
        // test flag setting
        .{ .base = 0xff, .offset = 1, .expect = 0x100, .expect_c = true },
        .{ .base = 0xf, .offset = 1, .expect = 0x10, .expect_h = true },
        .{ .base = 0xf1, .offset = 0xf, .expect = 0x100, .expect_c = true },
        .{ .base = 0x0, .offset = 16, .expect = 0x10, .expect_h = true },
    };

    for (configs) |c| {
        var bus = Bus{};
        var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

        var i = decoder.decode_instruction(opcode);
        cpu.rf.HL = 0;
        cpu.rf.SP = c.base;
        i.operands[2].value = @bitCast(@as(i16, c.offset));

        try testing.expectEqual(0, cpu.rf.HL);

        try testing.expectEqual(false, cpu.rf.flag_get(RegFile.F.C));
        try testing.expectEqual(false, cpu.rf.flag_get(RegFile.F.H));

        cpu.op_ld(i);
        try testing.expectEqual(c.expect, cpu.rf.HL);

        try testing.expectEqual(c.expect_c, cpu.rf.flag_get(RegFile.F.C));
        try testing.expectEqual(c.expect_h, cpu.rf.flag_get(RegFile.F.H));
    }
}

// ========================== Other tests

test "Register Flags" {
    var r = RegFile{};

    try testing.expectEqual(false, r.flag_get(RegFile.F.Z));

    r.flag_set(true, RegFile.F.Z);
    try testing.expectEqual(true, r.flag_get(RegFile.F.Z));

    r.flag_set(false, RegFile.F.Z);
    try testing.expectEqual(false, r.flag_get(RegFile.F.Z));
}
