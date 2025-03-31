const std = @import("std");
const lib = @import("gb_emulator_lib");

const rl = @import("raylib");

const Cpu = lib.cpu.Cpu;
const Bus = lib.sys.Bus;
const Cartridge = lib.cartridge.Cartridge;
const decoder = lib.cpu.decoder;

inline fn btoi(b: bool) u1 {
    return @intFromBool(b);
}

fn draw_gameboy(
    px: i32,
    py: i32,
    smallRadius: i32,
    bigRadius: i32,
    innerWidth: i32,
    innerHeight: i32,
    render_screen_width: i32,
) void {
    // Inner Shell
    rl.drawRectangle(
        px + smallRadius,
        py + smallRadius,
        innerWidth,
        innerHeight,
        .red,
    );
    rl.drawRectangle(
        px + smallRadius,
        py,
        innerWidth + (bigRadius - smallRadius),
        smallRadius,
        .red,
    );
    rl.drawRectangle(
        px + smallRadius,
        py + smallRadius + innerHeight,
        innerWidth,
        bigRadius,
        .red,
    );
    rl.drawRectangle(
        px,
        py + smallRadius,
        smallRadius,
        innerHeight + (bigRadius - smallRadius),
        .red,
    );
    rl.drawRectangle(
        px + smallRadius + innerWidth,
        py + smallRadius,
        bigRadius,
        innerHeight,
        .red,
    );

    // Corners
    rl.drawCircle(
        px + smallRadius,
        py + smallRadius,
        @floatFromInt(smallRadius),
        .red,
    );
    rl.drawCircle(
        px + smallRadius,
        py + smallRadius + innerHeight + (bigRadius - smallRadius),
        @floatFromInt(smallRadius),
        .red,
    );
    rl.drawCircle(
        px + smallRadius + innerWidth + (bigRadius - smallRadius),
        py + smallRadius,
        @floatFromInt(smallRadius),
        .red,
    );
    rl.drawCircle(
        px + smallRadius + innerWidth,
        py + smallRadius + innerHeight,
        @floatFromInt(bigRadius),
        .red,
    );

    const dpad_width = @as(f32, @floatFromInt(render_screen_width)) * 0.35 * 0.33;
    const dpad_pos_x = @as(f32, @floatFromInt(innerWidth)) * 0.15;
    const dpad_pos_y = @as(f32, @floatFromInt(innerHeight)) * 0.65;
    // origin is top left, outside of both horizontal and vertical
    // horizontal
    rl.drawRectangleRounded(.{ .x = dpad_pos_x, .y = dpad_pos_y + dpad_width, .width = dpad_width * 3, .height = dpad_width }, 0.1, 10, .dark_gray);
    // vertical
    rl.drawRectangleRounded(.{ .x = dpad_pos_x + dpad_width, .y = dpad_pos_y, .width = dpad_width, .height = dpad_width * 3 }, 0.1, 10, .dark_gray);

    // Buttons:
    const btn_radius = @as(f32, @floatFromInt(bigRadius)) * 0.5;
    const btn_pos_xa: i32 = @intFromFloat(@as(f32, @floatFromInt(innerWidth)) * 0.95 + btn_radius);
    const btn_pos_xb: i32 = @intFromFloat(@as(f32, @floatFromInt(btn_pos_xa)) - 3 * btn_radius);
    rl.drawCircle(
        btn_pos_xa,
        @intFromFloat(dpad_pos_y * 1.05),
        btn_radius,
        .dark_gray,
    );
    rl.drawCircle(
        btn_pos_xb,
        @intFromFloat(dpad_pos_y * 1.15),
        btn_radius,
        .dark_gray,
    );
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
        break :tmp try Cartridge.load("./external/tetris_world_reva.gb", allocator);
        // break :tmp try Cartridge.load("./external/gb-test-roms/cpu_instrs/individual/08-misc instrs.gb", allocator);
    };
    defer cartridge.deinit();

    if (args.next()) |second_arg| {
        if (std.mem.eql(u8, second_arg, "dump_header")) {
            return;
        }
    }

    var bus = Bus{};
    var cpu = Cpu{ .bus = &bus };
    bus.cartridge = &cartridge;
    bus.link();
    cpu.rf.AF = 0x0; // A = $11 indicates CGB.
    cpu.rf.PC = 0x0100;

    const real_screen_width = 160;
    const real_screen_height = 144;
    const screen_size_factor = 3;

    const render_screen_width = real_screen_width * screen_size_factor;
    const render_screen_height = real_screen_height * screen_size_factor;

    const screenWidth: usize = @intFromFloat(render_screen_width * 1.5);
    const screenHeight: usize = @intFromFloat(render_screen_height * 2.7);

    rl.initWindow(screenWidth, screenHeight, "zig-boi");
    defer rl.closeWindow(); // Close window and OpenGL context

    var fb_img = rl.genImageColor(render_screen_width, render_screen_height, rl.Color.dark_green);
    fb_img.setFormat(.uncompressed_r8g8b8);
    const fb_tex = try rl.loadTextureFromImage(fb_img);

    rl.setTargetFPS(60);
    const cycles_per_frame = 17476; // = 1048576 / 60;

    while (!rl.windowShouldClose()) {
        // 1) set inputs
        const up = rl.isKeyDown(.w);
        const down = rl.isKeyDown(.s);
        const left = rl.isKeyDown(.a);
        const right = rl.isKeyDown(.d);
        const sel = rl.isKeyDown(.n);
        const start = rl.isKeyDown(.m);
        const a = rl.isKeyDown(.j);
        const b = rl.isKeyDown(.k);

        bus.io.joy.host_set_dpad_state(btoi(up), btoi(down), btoi(left), btoi(right));
        bus.io.joy.host_set_btn_state(btoi(start), btoi(sel), btoi(b), btoi(a));

        // 2) process some instructions
        var cycles: usize = 0;
        while (cycles < cycles_per_frame) {
            cycles += cpu.step();
        }

        // 3) render frame to host
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

        const px = 0;
        const py = 0;
        const smallRadius = 10 * screen_size_factor;
        const bigRadius = 30 * screen_size_factor;
        const innerWidth = screenWidth - (smallRadius + bigRadius);
        const innerHeight = screenHeight - (smallRadius + bigRadius);
        draw_gameboy(px, py, smallRadius, bigRadius, innerWidth, innerHeight, render_screen_width);

        // background behind the screen
        rl.drawRectangleRounded(
            .{
                .x = px + render_screen_width * 0.15,
                .y = py + render_screen_width * 0.15,
                .width = render_screen_width * 1.25,
                .height = render_screen_height * 1.25,
            },
            0.1,
            10,
            .dark_gray,
        );

        rl.drawTexture(
            fb_tex,
            @intFromFloat(px + render_screen_width * 0.25),
            @intFromFloat(py + render_screen_width * 0.25),
            rl.Color.white,
        );
        // rl.drawFPS(0, 0);
    }
}
