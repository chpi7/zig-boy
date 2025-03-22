const std = @import("std");
const lib = @import("gb_emulator_lib");

const Cpu = lib.cpu.Cpu;
const Bus = lib.sys.Bus;
const Cartridge = lib.cartridge.Cartridge;
const decoder = lib.cpu.decoder;

const clock_callback_t = *const fn (*lib.sys.Timer) void;

fn clock(hz: u64, on_tick: clock_callback_t, t_ptr: *lib.sys.Timer, stop_token: *u64) void {
    std.debug.print("sleep ns = {}\n", .{std.time.ns_per_s / hz});
    while (stop_token.* == 0) {
        // (1s / hz) * ns_per_s === ns_per_s / hz
        std.time.sleep(std.time.ns_per_s / hz);
        on_tick(t_ptr);
    }
    std.log.debug("clock stopping", .{});
}

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

    var timer_token: u64 = 0;
    var timer_thread = try std.Thread.spawn(.{}, clock, .{
        @as(u64, 262144),
        lib.sys.Timer.tick_4m,
        &bus.io.timer,
        &timer_token,
    });

    cpu.rf.PC = 0x0100;
    const max_cycles: usize = 100000000;
    for (0..max_cycles) |_| {
        const pc = cpu.rf.PC;
        cpu.tick_m();
        if (cpu.rf.PC == pc) {
            // assume infinite loop at the end
            break;
        }
    }

    timer_token = 1;
    timer_thread.join();

    const stdout = std.io.getStdOut().writer();
    try stdout.print("Terminated instruction counter = {}!\n", .{cpu.instruction_debug_counter});
}
