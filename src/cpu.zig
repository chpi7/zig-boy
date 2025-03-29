const std = @import("std");
const sys = @import("sys.zig");
const Bus = sys.Bus;

const alu = @import("alu.zig");
pub const Alu = alu.Alu;
const F = Alu.F;

pub const decoder = @import("instructions_generated.zig");
const Instruction = decoder.Instruction;
const OpType = decoder.OpType;
const Operand = decoder.Operand;
const OperandType = decoder.OperandType;
const RegName = decoder.Register;

pub fn log(comptime format: []const u8, args: anytype) void {
    if (false) {
        std.log.debug(format, args);
    }
}

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
                RegName.F => @as(t, RegFile.get_lo(self.AF)) & 0xF0,
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
                RegName.AF => @as(t, self.AF) & 0xFFF0,
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
        if (t == u16 and n == RegName.AF) {
            self.get_ptr(n, t).* = (value & 0xFFF0);
        } else if (t == u8 and n == RegName.F) {
            self.get_ptr(n, t).* = (value & 0xF0);
        } else {
            self.get_ptr(n, t).* = value;
        }
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

    inline fn set_hi(self: *RegFile, v: u16, comptime x: []const u8) void {
        @field(self, x) = (@field(self, x) & 0x00FF) | (v << 8);
    }

    inline fn set_lo(self: *RegFile, v: u16, comptime x: []const u8) void {
        @field(self, x) = (@field(self, x) & 0xFF00) | v;
    }

    // --------------- Flags ----------------

    inline fn set_flag(self: *RegFile, comptime flag: F, value: u1) void {
        const fmask: u4 = 1 << F.offset(flag);
        const mask: u8 = @as(u8, fmask) << 4;
        if (value == 1) {
            self.AF |= mask;
        } else {
            self.AF &= ~mask;
        }
    }

    inline fn set_flags(self: *RegFile, value: u4) void {
        self.AF = (self.AF & 0xff0f) | @as(u8, value) << 4;
    }

    inline fn get_flag(self: *RegFile, comptime flag: F) u1 {
        return F.get(self.get_flags(), flag);
    }

    inline fn get_flags(self: *RegFile) u4 {
        return @truncate(self.AF >> 4);
    }

    fn test_cc(self: *RegFile, cc: decoder.ConditionCode) bool {
        return switch (cc) {
            decoder.ConditionCode.c => self.get_flag(F.C) == 1,
            decoder.ConditionCode.nc => self.get_flag(F.C) == 0,
            decoder.ConditionCode.z => self.get_flag(F.Z) == 1,
            decoder.ConditionCode.nz => self.get_flag(F.Z) == 0,
        };
    }

    fn dump_debug(self: *RegFile) void {
        // _ = self.*;
        const f = self.get_flags();
        log("Registers:", .{});
        log("\tAF {x:04}\tHL {x:04}\t (F: z:{} n:{} h:{} c:{})", .{ self.AF, self.HL, F.z(f), F.n(f), F.h(f), F.c(f) });
        log("\tBC {x:04}\tSP {x:04}", .{ self.BC, self.SP });
        log("\tDE {x:04}\tPC {x:04}", .{ self.DE, self.PC });
    }
};

pub const Cpu = struct {
    rf: RegFile = .{},
    /// global interrupt enable
    ime: u1 = 0,
    /// Effect of ei is delayed by one instruction.
    ime_enable_queued: u1 = 0,
    bus: *Bus,
    halted: bool = false,
    cc_status: u2 = 0, // 0: no cc, 1: cc + not taken, 2: cc + taken     This way cc_status % 2 can be used as the decoder index.
    instruction_debug_counter: u32 = 0,

    /// Execute one instruction or handle an interrupt.
    /// This ticks the rest of the system the correct number of cycles too.
    pub fn step(self: *Cpu) u8 {
        if (self.ime_enable_queued == 1) {
            self.ime = 1;
            self.ime_enable_queued = 0;
            std.log.debug("IME = 1", .{});
        }

        var cycles_passed: u8 = 0;
        if (!self.halted) {
            cycles_passed += self.execute_instruction();
            for (0..cycles_passed) |_| {
                self.tick_sys_1m();
            }
        }

        if (self.ime == 1) {
            for (0..5) |idx| {
                const check_mask: u8 = @as(u8, 1) << @truncate(idx);
                var request_mask: u8 = @bitCast(self.bus.io.ir_if);
                const requested = (check_mask & request_mask) != 0;
                const enabled = (check_mask & @as(u8, @bitCast(self.bus.ir_ie))) != 0;
                if (requested and enabled) {
                    request_mask &= ~check_mask;
                    self.bus.io.ir_if = @bitCast(request_mask);

                    self.ime = 0;
                    const addr: u16 = 0x40 + @as(u8, @truncate(idx)) * 0x08;
                    log("[cpu] handling int request ${x:02}", .{addr});

                    self.tick_sys_1m();
                    self.tick_sys_1m();
                    self.tick_sys_1m();
                    self.tick_sys_1m();

                    self.call(addr);

                    self.tick_sys_1m();
                    cycles_passed += 5;
                    break;
                }
            }
        }

        return cycles_passed;
    }

    fn decode_next(self: *Cpu) struct { Instruction, bool } {
        const decode_result = decoder.decode_instruction(self.bus.read(self.rf.PC));
        return if (decode_result.op != OpType.PREFIX) .{ decode_result, false } else .{ decoder.decode_instruction_cb(self.bus.read(self.rf.PC + 1)), true };
    }

    fn decode_imm(o: *Operand, instr_mem: [*]const u8) void {
        if (o.t == OperandType.unused) return;

        o.value = switch (o.t) {
            OperandType.imm16 => @as(u16, instr_mem[1]) | (@as(u16, instr_mem[2]) << 8), // load as little endian
            OperandType.imm8 => instr_mem[1],
            else => o.value, // nothing to change
        };
    }

    /// This ticks the rest of the system forward 1 m cycle.
    fn tick_sys_1m(self: *Cpu) void {
        self.bus.tick_1m();
    }

    fn execute_instruction(self: *Cpu) u8 {
        var i, const is_cb = self.decode_next();

        if (is_cb) {
            log("[cpu] ---- {} @ {x:04}  {s}", .{ self.instruction_debug_counter, self.rf.PC, decoder.opcode_to_str_cb(i.opcode) });
        } else {
            log("[cpu] ---- {} @ {x:04}  {s}", .{ self.instruction_debug_counter, self.rf.PC, decoder.opcode_to_str(i.opcode) });
        }

        const next_two_bytes = [_]u8{
            i.opcode,
            if (i.bytes > 1) self.bus.read(self.rf.PC + 1) else 0,
            if (i.bytes > 2) self.bus.read(self.rf.PC + 2) else 0,
        };
        decode_imm(&i.operands[0], &next_two_bytes);
        decode_imm(&i.operands[1], &next_two_bytes);
        decode_imm(&i.operands[2], &next_two_bytes);

        self.rf.PC += i.bytes; // this accounts for prefix + immediates

        self.cc_status = 0;

        const dp = if (is_cb) decoder.opcode_to_dp_cb(i.opcode) else decoder.opcode_to_dp(i.opcode);
        const Dt = decoder.DispatchType;
        switch (dp) {
            Dt.adc_u8_u8 => self.op_alu_op_u8_u8(i, Alu.adc8),
            Dt.add_u16_u16 => self.op_alu_op_u16_u16(i, Alu.add16),
            Dt.add_u16_u8 => self.op_add_sp_e8(i),
            Dt.add_u8_u8 => self.op_alu_op_u8_u8(i, Alu.add8),
            Dt.and_u8_u8 => self.op_alu_op_u8_u8(i, Alu.and8),
            Dt.bit_u3_u8 => self.op_bit(i),
            Dt.call_cc_u16 => self.op_call(i),
            Dt.call_u16 => self.op_call(i),
            Dt.ccf => self.op_ccf(),
            Dt.cp_u8_u8 => self.op_alu_op_u8_u8_fl(i, Alu.cp),
            Dt.cpl => self.op_cpl(),
            Dt.daa_u8 => self.op_alu_op_u8(i, Alu.daa),
            Dt.dec_u16 => self.op_alu_op_u16(i, Alu.dec16),
            Dt.dec_u8 => self.op_alu_op_u8(i, Alu.dec8),
            Dt.di => {
                self.ime = 0;
                std.log.debug("IME = 0", .{});
            },
            Dt.ei => {
                // self.ime = 1;
                self.ime_enable_queued = 1;
                std.log.debug("IME enabled queued", .{});
            },
            Dt.halt => self.op_halt(),
            Dt.illegal => unreachable,
            Dt.inc_u16 => self.op_alu_op_u16(i, Alu.inc16),
            Dt.inc_u8 => self.op_alu_op_u8(i, Alu.inc8),
            Dt.jp_cc_u16 => self.op_jp(i),
            Dt.jp_u16 => self.op_jp(i),
            Dt.jr_cc_u8 => self.op_jr(i),
            Dt.jr_u8 => self.op_jr(i),
            Dt.ld_u16_u16 => self.op_ld(i),
            Dt.ld_u16_u16_u8 => self.op_ld(i),
            Dt.ld_u8_u16 => self.op_ld(i),
            Dt.ld_u8_u8 => self.op_ld(i),
            Dt.ldh_u8_u8 => self.op_ldh(i),
            Dt.nop => {},
            Dt.or_u8_u8 => self.op_alu_op_u8_u8(i, Alu.or8),
            Dt.pop_u16 => self.op_pop(i),
            Dt.prefix => unreachable,
            Dt.push_u16 => self.op_push(i),
            Dt.res_u3_u8 => self.op_res(i),
            Dt.ret => self.op_ret(i),
            Dt.ret_cc => self.op_ret(i),
            Dt.reti => {
                self.ime = 1;
                self.op_ret(i);
                log("RETI done", .{});
            },
            Dt.rl_u8 => self.op_alu_op_u8_pref(i, Alu.rl8),
            Dt.rla_u8 => self.op_alu_op_u8(i, Alu.rl8),
            Dt.rlc_u8 => self.op_alu_op_u8_pref(i, Alu.rlc8),
            Dt.rlca_u8 => self.op_alu_op_u8(i, Alu.rlc8),
            Dt.rr_u8 => self.op_alu_op_u8_pref(i, Alu.rr8),
            Dt.rra_u8 => self.op_alu_op_u8(i, Alu.rr8),
            Dt.rrc_u8 => self.op_alu_op_u8_pref(i, Alu.rrc8),
            Dt.rrca_u8 => self.op_alu_op_u8(i, Alu.rrc8),
            Dt.rst_vec => self.op_call(i),
            Dt.sbc_u8_u8 => self.op_alu_op_u8_u8(i, Alu.sbc8),
            Dt.scf => self.op_scf(),
            Dt.set_u3_u8 => self.op_set(i),
            Dt.sla_u8 => self.op_alu_op_u8(i, Alu.sla8),
            Dt.sra_u8 => self.op_alu_op_u8(i, Alu.sra8),
            Dt.srl_u8 => self.op_alu_op_u8(i, Alu.srl8),
            Dt.stop_u8 => self.op_stop(),
            Dt.sub_u8_u8 => self.op_alu_op_u8_u8(i, Alu.sub8),
            Dt.swap_u8 => self.op_alu_op_u8(i, Alu.swap8),
            Dt.xor_u8_u8 => self.op_alu_op_u8_u8(i, Alu.xor8),
        }

        const o = &i.operands[0];
        if ((o.t == OperandType.reg8 or o.t == OperandType.reg16) and !o.relative) {
            // if the first (maybe dst) operand is a register, dump them
            self.rf.dump_debug();
        }

        const m_cycle_idx: u1 = @truncate(self.cc_status % 2);
        var m_cycles: u8 = 0;
        if (is_cb) {
            m_cycles = decoder.get_m_cycles_cb(i.opcode, m_cycle_idx);
        } else {
            m_cycles = decoder.get_m_cycles(i.opcode, m_cycle_idx);
        }

        log("[cpu] execute done", .{});
        self.instruction_debug_counter +%= 1;

        return m_cycles;
    }

    /// Prepare i8 stored in u16 immediate for arithmetic.
    /// This is because we read all immediates into a u16:
    /// 0x00ff would be -1 = 0xff written into a u16.
    /// For the arithmentic to be less annyoing, sign extend the encoded i8
    /// to i16. So 0xffff in this case.
    fn se_i8_in_u16(x: u16) u16 {
        const a: i8 = @bitCast(@as(u8, @truncate(x)));
        const b: i16 = @intCast(a); // properly sign extend in i16
        return @bitCast(b); // treat as u16 again
    }

    fn msb(x: u16) u8 {
        return @truncate(x >> 8);
    }

    fn lsb(x: u16) u8 {
        return @truncate(x);
    }

    pub fn op_push(self: *Cpu, i: Instruction) void {
        std.debug.assert(i.operands[0].t == OperandType.reg16);
        const reg = i.operands[0].register;

        self.rf.SP -%= 1;
        self.bus.write(self.rf.SP, msb(self.rf.get(reg, u16)));
        self.rf.SP -%= 1;
        self.bus.write(self.rf.SP, lsb(self.rf.get(reg, u16)));
    }

    pub fn op_pop(self: *Cpu, i: Instruction) void {
        std.debug.assert(i.operands[0].t == OperandType.reg16);

        var l: u16 = self.bus.read(self.rf.SP);
        self.rf.SP +%= 1;
        const m: u16 = self.bus.read(self.rf.SP);
        self.rf.SP +%= 1;

        if (i.operands[0].register == RegName.AF) {
            // we need to only pop the flags into F and not anything else in the other 4 bits
            l &= 0xf0;
        }

        self.rf.set(u16, i.operands[0].register, m << 8 | l);
    }

    fn update_cc_status(self: *Cpu, is_cc: bool, taken: bool) void {
        std.debug.assert(self.cc_status == 0); // should always be reset.
        if (is_cc) {
            if (taken) {
                self.cc_status = 2;
            } else {
                self.cc_status = 1;
            }
        }
    }

    fn call(self: *Cpu, target: u16) void {
        self.rf.SP -%= 1;
        self.bus.write(self.rf.SP, msb(self.rf.PC));
        self.rf.SP -%= 1;
        self.bus.write(self.rf.SP, lsb(self.rf.PC));
        self.rf.PC = target;
        self.rf.dump_debug();
    }

    fn op_call(self: *Cpu, i: Instruction) void {
        var target: u16 = 0;
        var is_cc = false;

        if (i.num_operands == 2) {
            std.debug.assert(i.operands[1].t == OperandType.imm16);
            std.debug.assert(i.operands[0].t == OperandType.cc);
            target = i.operands[1].value;
            is_cc = true;
        } else {
            std.debug.assert(i.num_operands == 1);
            std.debug.assert(i.operands[0].t == OperandType.imm16 or
                i.operands[0].t == OperandType.vec);
            switch (i.operands[0].t) {
                OperandType.imm16 => target = i.operands[0].value,
                // reset vec is just the target address
                OperandType.vec => target = i.operands[0].value,
                else => unreachable,
            }
        }

        var taken = false;
        if (!is_cc or self.rf.test_cc(i.operands[0].cc)) {
            self.call(target);
            taken = true;
        }

        self.update_cc_status(is_cc, taken);
    }

    fn op_ret(self: *Cpu, i: Instruction) void {
        var is_cc = false;

        if (i.num_operands == 1) {
            std.debug.assert(i.operands[0].t == OperandType.cc);
            is_cc = true;
        }

        var taken = false;
        if (!is_cc or self.rf.test_cc(i.operands[0].cc)) {
            const l: u16 = self.bus.read(self.rf.SP);
            self.rf.SP +%= 1;
            const m: u16 = self.bus.read(self.rf.SP);
            self.rf.SP +%= 1;

            self.rf.PC = m << 8 | l;
            log("[cpu] pc = {x:04}", .{self.rf.PC});

            taken = true;
        }
        self.update_cc_status(is_cc, taken);
    }

    fn op_stop(_: *Cpu) void {
        // TODO: decide what to do here
        // prob nothing: "no licensed rom makes use of STOP outside of CGB
        // speed switching"
    }

    fn op_halt(self: *Cpu) void {
        const ie: u8 = @bitCast(self.bus.ir_ie);
        const ir: u8 = @bitCast(self.bus.io.ir_if);
        const interrupts_pending = (ir & ie) != 0;

        if (self.ime == 1) {
            // enter low power mode
            // after the next interrupt is serviced, wake up
            // execute the next instruction after halt
            self.halted = true;
        } else if (!interrupts_pending) {
            // like above, except we don't call the handler
            self.halted = true;
        } else {
            // don't halt, continue execution, byte after read twice, pc only
            // incremented once though
            // Probably fine if we just ignore that though...
        }

        // --> this is it here, on step, we check if we are halted.
        // If yes, only wake up if an interrupt becomes pending.
        // In case one is pending now, service it according to the ime flag.
        // (which you can not change while the cpu is halted)
        //
        // If the ime flag is set PC points to the next instruction after the HALT.
        // When the interrupt is serviced, that PC gets pushed, the handler called,
        // on RETI we jump back to the correct instruction.
    }

    fn op_jp(self: *Cpu, i: Instruction) void {
        var is_cc = false;
        var dst: *const Operand = &i.operands[0];

        if (i.num_operands == 2) {
            std.debug.assert(i.operands[0].t == OperandType.cc);
            dst = &i.operands[1];
            is_cc = true;
        }

        var target: u16 = 0;
        if (dst.t == OperandType.imm16) {
            target = dst.value;
        } else if (dst.t == OperandType.reg16) {
            std.debug.assert(dst.register == RegName.HL);
            target = self.rf.HL;
        }

        var taken = false;
        if (!is_cc or self.rf.test_cc(i.operands[0].cc)) {
            self.rf.PC = target;
            log("[cpu] jp {x:04}", .{self.rf.PC});
            taken = true;
        }

        self.update_cc_status(is_cc, taken);
    }

    fn op_jr(self: *Cpu, i: Instruction) void {
        var is_cc = false;
        var offset = i.operands[0].value;

        if (i.num_operands == 2) {
            std.debug.assert(i.operands[0].t == OperandType.cc);
            std.debug.assert(i.operands[1].t == OperandType.imm8);
            offset = i.operands[1].value;
            is_cc = true;
        } else {
            std.debug.assert(i.num_operands == 1);
            std.debug.assert(i.operands[0].t == OperandType.imm8);
        }

        var taken = false;
        if (!is_cc or self.rf.test_cc(i.operands[0].cc)) {
            const offset_as_u16: u16 = se_i8_in_u16(offset);
            self.rf.PC = self.rf.PC +% offset_as_u16;
            taken = true;
        }
        self.update_cc_status(is_cc, taken);
    }

    pub fn op_ldh(self: *Cpu, i: Instruction) void {
        const src = i.operands[1];
        const dst = i.operands[0];

        var value: u8 = 0;

        if (src.t == OperandType.reg8) {
            value = self.rf.get(src.register, u8);
        } else {
            std.debug.assert(src.t == OperandType.imm8 and src.relative);
            value = @truncate(src.value);
        }
        if (src.relative) {
            value = self.bus.read(@as(u16, 0xff00) | value);
        }

        if (!dst.relative) {
            std.debug.assert(dst.t == OperandType.reg8);
            self.rf.set(u8, dst.register, value);
        } else {
            std.debug.assert(dst.t == OperandType.reg8 or dst.t == OperandType.imm8);
            const offset: u8 = if (dst.t == OperandType.imm8) @truncate(dst.value) else self.rf.get(dst.register, u8);
            self.bus.write(@as(u16, 0xff00) | offset, value);
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
            const sp = self.rf.SP;
            const imm = Cpu.se_i8_in_u16(self.fetch_op_u8(i.operands[2]));

            const sp_new, const f = Alu.add16i8(sp, imm, self.rf.get_flags());

            self.rf.set_flags(f);
            self.rf.HL = sp_new;
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

    fn op_ccf(self: *Cpu) void {
        const flags_now = self.rf.get_flags();
        self.rf.set_flags(F.build(~F.c(flags_now), 0, 0, F.z(flags_now)));
    }

    fn op_scf(self: *Cpu) void {
        var flags: F.T = @bitCast(self.rf.get_flags());
        flags.c = 1;
        flags.n = 0;
        flags.h = 0;
        self.rf.set_flags(@bitCast(flags));
    }

    fn op_cpl(self: *Cpu) void {
        // complement accumulator
        self.rf.AF ^= 0xff00; // flip bits in A
        var flags: F.T = @bitCast(self.rf.get_flags());
        flags.n = 1;
        flags.h = 1;
        self.rf.set_flags(@bitCast(flags));
    }

    fn op_bit(self: *Cpu, i: Instruction) void {
        std.debug.assert(i.operands[0].t == OperandType.bit3);

        const mask: u8 = @as(u8, 1) << @as(u3, @truncate(i.operands[0].value));
        const v = self.fetch_op_u8(i.operands[1]);

        var flags: F.T = @bitCast(self.rf.get_flags());
        flags.n = 0;
        flags.h = 1;
        flags.z = if ((v & mask) == 0) 1 else 0;
        self.rf.set_flags(@bitCast(flags));
    }

    fn op_set(self: *Cpu, i: Instruction) void {
        std.debug.assert(i.operands[0].t == OperandType.bit3);

        const mask: u8 = @as(u8, 1) << @as(u3, @truncate(i.operands[0].value));
        var v = self.fetch_op_u8(i.operands[1]);

        v |= mask;

        self.store_op_u8(i.operands[1], v);
    }

    fn op_res(self: *Cpu, i: Instruction) void {
        std.debug.assert(i.operands[0].t == OperandType.bit3);

        const mask: u8 = @as(u8, 1) << @as(u3, @truncate(i.operands[0].value));
        var v = self.fetch_op_u8(i.operands[1]);

        v &= ~mask;

        self.store_op_u8(i.operands[1], v);
    }

    fn fetch_op_u8(self: *Cpu, o: Operand) u8 {
        // source can be one of reg8, imm8, [reg16], [imm16]
        std.debug.assert(o.t == OperandType.reg8 or
            o.t == OperandType.imm8 or
            (o.t == OperandType.reg16 and o.relative) or
            (o.t == OperandType.imm16 and o.relative));

        return switch (o.t) {
            OperandType.reg8 => self.rf.get(o.register, u8),
            OperandType.imm8 => @truncate(o.value),
            OperandType.reg16 => self.bus.read(self.rf.get(o.register, u16)),
            OperandType.imm16 => self.bus.read(o.value),
            else => unreachable,
        };
    }

    fn fetch_op_u16(self: *Cpu, o: Operand) u16 {
        // source can only be one of reg16, imm16
        std.debug.assert(o.t == OperandType.reg16 or o.t == OperandType.imm16);

        return switch (o.t) {
            OperandType.reg16 => self.rf.get(o.register, u16),
            OperandType.imm16 => o.value,
            else => unreachable,
        };
    }

    fn store_op_u8(self: *Cpu, o: Operand, v: u8) void {
        // dst can be one of reg8, [reg16], [imm16]
        std.debug.assert(o.t == OperandType.reg8 or
            (o.t == OperandType.reg16 and o.relative) or
            (o.t == OperandType.imm16 and o.relative));

        switch (o.t) {
            OperandType.reg8 => self.rf.set(u8, o.register, v),
            OperandType.reg16 => self.bus.write(self.rf.get(o.register, u16), v),
            OperandType.imm16 => self.bus.write(o.value, v),
            else => unreachable,
        }
    }

    fn store_op_u16(self: *Cpu, o: Operand, v: u16) void {
        // dst can only be reg16
        std.debug.assert(o.t == OperandType.reg16);

        switch (o.t) {
            OperandType.reg16 => self.rf.set(u16, o.register, v),
            else => unreachable,
        }
    }

    fn op_alu_op_u8_pref(self: *Cpu, i: Instruction, op: Alu.op8_t) void {
        self.op_alu_op_u8_gen(i, op, true);
    }

    fn op_alu_op_u8(self: *Cpu, i: Instruction, op: Alu.op8_t) void {
        self.op_alu_op_u8_gen(i, op, false);
    }

    inline fn op_alu_op_u8_gen(self: *Cpu, i: Instruction, op: Alu.op8_t, comptime prefixed: bool) void {
        std.debug.assert(i.num_operands == 1);

        const x = self.fetch_op_u8(i.operands[0]);

        const res, var f = op(x, self.rf.get_flags());

        if (!prefixed) {
            switch (i.opcode) {
                0x07, 0x17, 0x0f, 0x1f => {
                    // RLCA, RLA, RRCA, RRA always set Z flag = 0
                    // not super nice to fix this here but it's fine for now.
                    f &= ~(@as(u4, 1) << F.offset(F.Z));
                },
                else => {},
            }
        }

        self.rf.set_flags(f);
        self.store_op_u8(i.operands[0], res);
    }

    fn op_alu_op_u16(self: *Cpu, i: Instruction, op: Alu.op16_t) void {
        std.debug.assert(i.num_operands == 1);

        const x = self.fetch_op_u16(i.operands[0]);

        const res, const f = op(x, self.rf.get_flags());

        self.rf.set_flags(f);
        self.store_op_u16(i.operands[0], res);
    }

    fn op_alu_op_u8_u8(self: *Cpu, i: Instruction, op: Alu.op8_8_t) void {
        std.debug.assert(i.num_operands == 2);
        const a = self.fetch_op_u8(i.operands[0]);
        const b = self.fetch_op_u8(i.operands[1]);

        const res, const f = op(a, b, self.rf.get_flags());

        self.rf.set_flags(f);
        self.store_op_u8(i.operands[0], res);
    }

    fn op_alu_op_u8_u8_fl(self: *Cpu, i: Instruction, op: Alu.op8_8_fl_t) void {
        std.debug.assert(i.num_operands == 2);
        const a = self.fetch_op_u8(i.operands[0]);
        const b = self.fetch_op_u8(i.operands[1]);

        const f = op(a, b, self.rf.get_flags());

        self.rf.set_flags(f);
    }

    fn op_alu_op_u16_u16(self: *Cpu, i: Instruction, op: Alu.op16_16_t) void {
        std.debug.assert(i.num_operands == 2);
        const a = self.fetch_op_u16(i.operands[0]);
        const b = self.fetch_op_u16(i.operands[1]);

        const res, const f = op(a, b, self.rf.get_flags());

        self.rf.set_flags(f);
        self.store_op_u16(i.operands[0], res);
    }

    fn op_add_sp_e8(self: *Cpu, i: Instruction) void {
        std.debug.assert(i.num_operands == 2);
        const a = self.rf.SP;
        const b = Cpu.se_i8_in_u16(self.fetch_op_u8(i.operands[1]));

        const res, const f = Alu.add16i8(a, b, self.rf.get_flags());

        self.rf.set_flags(f);
        self.rf.SP = res;
    }
};

// --------------------------------------------

const testing = std.testing;

test "R/W GP Registers" {
    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus, .rf = RegFile{} };

    const R = RegName;

    for (
        [_]R{ R.BC, R.DE, R.HL },
        [_]R{ R.B, R.D, R.H },
        [_]R{ R.C, R.E, R.L },
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
        .{ .base = 0xfffe, .offset = 3, .expect = 1, .expect_h = true },
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

        try testing.expectEqual(0, cpu.rf.get_flag(F.C));
        try testing.expectEqual(0, cpu.rf.get_flag(F.H));

        cpu.op_ld(i);
        try testing.expectEqual(c.expect, cpu.rf.HL);
    }
}

// ========================== Other tests

test "AF register get/set flags" {
    var rf = RegFile{};
    rf.set(u16, RegName.AF, 0xff0f);
    try testing.expectEqual(0, rf.get_flags());

    rf.set_flags(0b1010);
    try testing.expectEqual(0xffa0, rf.AF);
}

test "push/pop" {
    var bus: Bus = Bus{};
    var cpu: Cpu = Cpu{ .bus = &bus };

    try testing.expectEqual(256, Bus.fake_mem_size);
    cpu.rf.SP = 0xff;

    cpu.rf.AF = 0x1230;

    // --- push

    const i: Instruction = Instruction{
        .op = OpType.PUSH,
        .num_operands = 1,
        .operands = .{
            Operand{ .t = OperandType.reg16, .register = RegName.AF },
            Operand{ .t = OperandType.unused },
            Operand{ .t = OperandType.unused },
        },
        .bytes = 1,
    };

    cpu.op_push(i);

    try testing.expectEqual(0x12, cpu.bus.fake_memory[0xfe]);
    try testing.expectEqual(0x30, cpu.bus.fake_memory[0xfd]);

    // --- pop

    cpu.rf.AF = 0;

    const j: Instruction = Instruction{
        .op = OpType.POP,
        .num_operands = 1,
        .operands = .{
            Operand{ .t = OperandType.reg16, .register = RegName.AF },
            Operand{ .t = OperandType.unused },
            Operand{ .t = OperandType.unused },
        },
        .bytes = 1,
    };

    cpu.op_pop(j);
    // try testing.expectEqual(0x1234, cpu.rf.AF);
    try testing.expectEqual(0x1230, cpu.rf.AF);
}

test "twos complement math sanity checks" {
    const a: u16 = 1;

    const b: u8 = @bitCast(@as(i8, -1));
    try testing.expectEqual(0xff, b);

    const c = Cpu.se_i8_in_u16(b);
    try testing.expectEqual(0xffff, c);

    const r = a +% c;
    try testing.expectEqual(0, r);

    const offset: u8 = 0xfe;
    const offset_as_u16: u16 = Cpu.se_i8_in_u16(offset);
    var pc: u16 = 0xcc5f;
    pc = pc +% offset_as_u16;
    try testing.expectEqual(0xcc5d, pc);
}

test "AF register get/set individual" {
    var r = RegFile{};
    r.AF = 0;
    try testing.expectEqual(0, r.get_flag(F.C));
    r.set_flag(F.C, 1);
    try testing.expectEqual(1, r.get_flag(F.C));
    r.set_flag(F.C, 0);
    try testing.expectEqual(0, r.get_flag(F.C));

    r.AF = 0b1000_0000;
    try testing.expectEqual(1, r.get_flag(F.Z));

    r.AF = 0b0010_0000;
    try testing.expectEqual(1, r.get_flag(F.H));

    r.AF = 0;
    r.set_flag(F.Z, 1);
    try testing.expectEqual(0b1000_0000, r.AF);

    r.set_flag(F.C, 1);
    try testing.expectEqual(0b1001_0000, r.AF);
}

test "AF set flags in correct position" {
    var r = RegFile{};
    r.AF = 0;

    r.set_flags(0xf);
    try testing.expectEqual(0x00f0, r.AF);

    try testing.expectEqual(0xf, r.get_flags());
}
