const std = @import("std");
const lib = @import("gb_emulator_lib");

const Cpu = lib.cpu.Cpu;
const Bus = lib.sys.Bus;
const Cartridge = lib.cartridge.Cartridge;
const decoder = lib.cpu.decoder;

fn foo() void {
    const Instruction = decoder.Instruction;
    const OpType = decoder.OpType;
    const Operand = decoder.Operand;
    const OperandType = decoder.OperandType;
    const RegName = decoder.Register;

    var bus: Bus = Bus{};
    var cpu: Cpu = Cpu{ .bus = &bus };

    cpu.rf.SP = 0xff;

    cpu.rf.AF = 0x1234;

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

    std.log.debug("AF = {x}", .{cpu.rf.AF});
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var cartridge = try Cartridge.load("./external/gb-test-roms/cpu_instrs/individual/01-special.gb", allocator);
    // var cartridge = try Cartridge.load("./external/gb-test-roms/cpu_instrs/individual/06-ld r,r.gb", allocator);
    // var cartridge = try Cartridge.load("./external/gb-test-roms/cpu_instrs/individual/09-op r,r.gb", allocator);
    // var cartridge = try Cartridge.load("./external/gb-test-roms/cpu_instrs/cpu_instrs.gb", allocator);
    defer cartridge.deinit();

    var bus = Bus{ .cartridge = &cartridge };
    var cpu = Cpu{ .bus = &bus };

    cpu.rf.PC = 0x0100;
    const max_cycles: usize = 100000000;
    for (0..max_cycles) |_| {
        cpu.execute_instruction();
    }
}
