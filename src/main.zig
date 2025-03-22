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

    var args = std.process.args();
    _ = args.skip();
    var cartridge = if (args.next()) |first_arg| tmp: {
        break :tmp try Cartridge.load(first_arg, allocator);
    } else tmp: {
        break :tmp try Cartridge.load("./external/gb-test-roms/cpu_instrs/individual/08-misc instrs.gb", allocator);

        // var cartridge = try Cartridge.load("./external/gb-test-roms/cpu_instrs/individual/01-special.gb", allocator);
        // var cartridge = try Cartridge.load("./external/gb-test-roms/cpu_instrs/individual/08-misc instrs.gb", allocator);
        // var cartridge = try Cartridge.load("./external/gb-test-roms/cpu_instrs/individual/06-ld r,r.gb", allocator);
        // var cartridge = try Cartridge.load("./external/gb-test-roms/cpu_instrs/individual/09-op r,r.gb", allocator);
        // var cartridge = try Cartridge.load("./external/gb-test-roms/cpu_instrs/cpu_instrs.gb", allocator);
    };
    defer cartridge.deinit();

    var bus = Bus{ .cartridge = &cartridge };
    var cpu = Cpu{ .bus = &bus };

    cpu.rf.PC = 0x0100;
    const max_cycles: usize = 100000000;
    for (0..max_cycles) |_| {
        cpu.execute_instruction();
    }
}
