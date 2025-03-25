const std = @import("std");
const lib = @import("gb_emulator_lib");

const rl = @import("raylib");

const Cpu = lib.cpu.Cpu;
const Bus = lib.sys.Bus;
const Cartridge = lib.cartridge.Cartridge;
const decoder = lib.cpu.decoder;

fn gameboy_fun(cpu: *Cpu, done: *bool) !void {
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

    done.* = true;
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

    var thread_done = false;
    var gb_thread = try std.Thread.spawn(.{}, gameboy_fun, .{ &cpu, &thread_done });

    // Raylib Initialization
    //--------------------------------------------------------------------------------------
    const real_screen_width = 160;
    const real_screen_height = 144;
    const screen_size_factor = 5;

    const screenWidth = real_screen_width * screen_size_factor;
    const screenHeight = real_screen_height * screen_size_factor;

    rl.initWindow(screenWidth, screenHeight, "raylib-zig [core] example - basic window");
    defer rl.closeWindow(); // Close window and OpenGL context

    var fb_img = rl.genImageColor(screenWidth, screenHeight, rl.Color.dark_green);
    fb_img.setFormat(.uncompressed_r8g8b8);
    const fb_tex = try rl.loadTextureFromImage(fb_img);

    rl.setTargetFPS(60); // Set our game to run at 60 frames-per-second

    // Main game loop
    while (!rl.windowShouldClose()) { // Detect window close button or ESC key
        rl.beginDrawing();
        defer rl.endDrawing();

        for (0..real_screen_height) |h| {
            for (0..real_screen_width) |w| {
                const x: i32 = @intCast(w * screen_size_factor);
                const y: i32 = @intCast(h * screen_size_factor);
                const s: i32 = @intCast(screen_size_factor);
                const c: u32 = cpu.bus.io.lcd.get_pixel(w, h) * 64;
                rl.imageDrawRectangle(&fb_img, x, y, s, s, rl.getColor(0x000000ff | c << 16));
            }
        }

        rl.updateTexture(fb_tex, fb_img.data);
        rl.drawTexture(fb_tex, 0, 0, rl.Color.white);
        rl.drawFPS(0, 0);

        // rl.drawText("Congrats! You created your first window!", 190, 200, 20, .light_gray);
        //----------------------------------------------------------------------------------
    }

    gb_thread.join();
}
