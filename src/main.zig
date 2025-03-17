const std = @import("std");
const lib = @import("gb_emulator_lib");

const Cpu = lib.cpu.Cpu;
const Bus = lib.sys.Bus;
const Cartridge = lib.cartridge.Cartridge;
const decoder = lib.cpu.decoder;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var cartridge = try Cartridge.load("./external/gb-test-roms/cpu_instrs/cpu_instrs.gb", allocator);
    defer cartridge.deinit();

    var bus = Bus{ .cartridge = &cartridge };
    var cpu = Cpu{ .bus = &bus };

    cpu.rf.PC = 0x0100;
    for (0..10000) |_| {
        cpu.execute_instruction();
    }
}
