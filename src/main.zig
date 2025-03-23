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
    };
    defer cartridge.deinit();

    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus };
    bus.cartridge = &cartridge;
    bus.link();

    cpu.rf.PC = 0x0100;
    while (true) {
        const pc = cpu.rf.PC;
        cpu.step();
        if (cpu.rf.PC == pc and !cpu.halted) {
            std.log.debug("stop, pc not changed and not halted", .{});
            // assume infinite loop at the end
            break;
        }
    }

    const stdout = std.io.getStdOut().writer();
    try stdout.print("Terminated instruction counter = {}!\n", .{cpu.instruction_debug_counter});
}
