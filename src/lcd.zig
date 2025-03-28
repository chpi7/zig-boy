const std = @import("std");
const sys = @import("sys.zig");
const Bus = sys.Bus;

pub fn log(comptime format: []const u8, args: anytype) void {
    if (true) {
        std.log.debug(format, args);
    }
}

pub const Lcd = struct {
    const Lcdc = packed struct { // The LCDC LCD Control register
        bg_win_en: u1 = 0,
        ob_en: u1 = 0,
        ob_sz: u1 = 0,
        bg_tm: u1 = 0,
        bg_wt: u1 = 0,
        wi_en: u1 = 0,
        wi_tm: u1 = 0,
        en: u1 = 0,

        inline fn sprite_height(self: *Lcdc) u8 {
            return @as(u8, 8) << self.ob_sz;
        }

        inline fn bg_tm_base(self: *Lcdc) u16 {
            return if (self.bg_tm == 0) 0x9800 else 0x9c00;
        }

        inline fn win_tm_base(self: *Lcdc) u16 {
            return if (self.wi_tm == 0) 0x9800 else 0x9c00;
        }

        inline fn bg_wn_td_base(self: *Lcdc) u16 {
            return if (self.bg_wt == 0) 0x8800 else 0x8000;
        }
    };
    const Stat = packed struct { // The LCDC Stat register
        ppu_mode: u2 = 0,
        lyc_eq: u1 = 0,
        m0_int_sel: u1 = 0,
        m1_int_sel: u1 = 0,
        m2_int_sel: u1 = 0,
        lyc_int_sel: u1 = 0,
        _padding: u1 = 0,

        fn write(self: *Stat, value: u8) void {
            const write_mask: u8 = 0b0111_1000;
            self.* = @bitCast(@as(u8, @bitCast(self.*)) & ~write_mask | (write_mask & value));
        }
    };

    const OamEntry = packed struct {
        const Flags = packed struct {
            cbg_only: u3 = 0,
            bank: u1 = 0,
            palette: u1 = 0,
            x_flip: u1 = 0,
            y_flip: u1 = 0,
            priority: u1 = 0,
        };

        flags: OamEntry.Flags = .{},
        tile_idx: u8 = 0,
        x: u8 = 0,
        y: u8 = 0,
    };

    lcdc: Lcdc = Lcdc{},
    ly: u8 = 0x00, // current scanline. range 0-153, 144-153 indicates VBLANK
    lyc: u8 = 0, // LY compare
    scy: u8 = 0,
    scx: u8 = 0,
    wx: u8 = 0,
    wy: u8 = 0,
    bg_palette: u8 = 0,
    ob0_palette: u8 = 0,
    ob1_palette: u8 = 0,
    stat: Stat = Stat{},
    bus: ?*Bus = null,

    // PPU stuff:
    ppu_ly: u8 = 0,
    ppu_dot_line: u16 = 0,
    oam_buffer: [10]OamEntry = [_]OamEntry{OamEntry{}} ** 10,
    oam_buffer_count: usize = 0,
    framebuffer: [23040]u8 = [_]u8{0} ** 23040, // row major 160 * 144 screen

    fn address_to_name(address: u16) []const u8 {
        return switch (address) {
            0xff40 => "lcdc",
            0xff41 => "stat",
            0xff42 => "scy",
            0xff43 => "scx",
            0xff44 => "ly",
            0xff45 => "lyc",
            0xff46 => "bus",
            0xff47 => "bgp",
            0xff48 => "ob0p",
            0xff49 => "ob1p",
            0xff4a => "wy",
            0xff4b => "wx",
            else => "UNKNOWN",
        };
    }

    pub fn write(self: *Lcd, address: u16, value: u8) void {
        switch (address) {
            0xff40 => {
                self.lcdc = @bitCast(value);
                log("{}", .{self.lcdc});
            },
            0xff41 => {
                self.stat.write(value);
                log("{}", .{self.stat});
            },
            0xff42 => self.scy = value,
            0xff43 => self.scx = value,
            0xff44 => {}, // ly, read only
            0xff45 => self.lyc = value, // lyc
            0xff46 => self.bus.?.init_oam_dma(value),
            0xff47 => self.bg_palette = value,
            0xff48 => self.ob0_palette = value,
            0xff49 => self.ob1_palette = value,
            0xff4a => self.wy = value,
            0xff4b => self.wx = value,
            else => {}, // dont care
        }
        if (0xff40 <= address and address <= 0xff45) {
            log("[lcd] write {s} := {x:02}", .{ Lcd.address_to_name(address), value });
        }
    }

    pub fn read(self: *Lcd, address: u16) u8 {
        std.debug.assert(0 <= self.ly and self.ly <= 153);
        const result: u8 = switch (address) {
            0xff40 => @bitCast(self.lcdc),
            0xff41 => @as(u8, @bitCast(self.stat)) & 0b0111_1111,
            0xff42 => self.scy,
            0xff43 => self.scx,
            0xff44 => self.ly,
            0xff45 => self.lyc,
            0xff47 => self.bg_palette,
            0xff48 => self.ob0_palette,
            0xff49 => self.ob1_palette,
            0xff4a => self.wy,
            0xff4b => self.wx,
            else => 0, // dont care, return 0 aka no signal on any line
        };
        if (0xff40 <= address and address <= 0xff45) {
            // log("[lcd] read {s} -> {x:02}", .{ Lcd.address_to_name(address), result });
        }
        return result;
    }

    /// This drives the PPU drawing to the screen
    ///
    /// One frame takes 70224 dots, with 1 m cycle == 4 dots.
    pub fn tick_1m(self: *Lcd) void {
        // Return if we are disabled
        if (self.lcdc.en == 0) {
            @memset(&self.framebuffer, 0);
            // log("[lcd] skip tick (disabled)", .{});
            return;
        }
        // log("[lcd] tick begin", .{});

        // Update PPU mode before doing something.
        switch (self.stat.ppu_mode) {
            0 => if (self.ppu_dot_line == 456) {
                const next_mode: u2 = if (self.ppu_ly == 143) 1 else 2;
                self.set_ppu_mode(next_mode);
            }, // HBLANK
            1 => if (self.ppu_ly == 153 and self.ppu_dot_line == 456) {
                self.set_ppu_mode(2);
            }, // VBLANK
            2 => if (self.ppu_dot_line == 80) self.set_ppu_mode(3), // OAM
            3 => {
                // We draw one pixel per dot
                const pixel_x = self.ppu_dot_line - 80;
                if (pixel_x == 160) self.set_ppu_mode(0);
            },
        }

        if (self.ppu_dot_line == 456) {
            self.ppu_dot_line = 0;
            self.ppu_ly = (self.ppu_ly + 1) % 154;
        }

        std.debug.assert(self.ppu_ly <= 153);
        self.set_stat_ly(@truncate(self.ppu_ly));

        switch (self.stat.ppu_mode) {
            0, 1 => {}, // H/V BLANK
            2 => if (self.ppu_dot_line == 0) {
                self.oam_scan();
            },
            3 => {
                const pixel_x = self.ppu_dot_line - 80;
                std.debug.assert(pixel_x < 160);
                for (0..4) |i| {
                    self.draw_pixel(@truncate(pixel_x + i), @truncate(self.ppu_ly));
                }
            },
        }

        self.ppu_dot_line += 4;

        // log("[lcd] tick end", .{});
    }

    fn oam_scan(self: *Lcd) void {
        self.oam_buffer_count = 0;
        var oam_scan_idx: u8 = 0;
        const sprite_height: u8 = self.lcdc.sprite_height();

        while (self.oam_buffer_count < 10 and oam_scan_idx < 40) {
            const candidate = self.read_oam_entry(oam_scan_idx);
            const okay = (candidate.x > 0) and
                (self.ly + 16 >= candidate.y) and
                (self.ly + 16 < candidate.y + sprite_height);

            if (okay) {
                self.oam_buffer[self.oam_buffer_count] = candidate;
                self.oam_buffer_count += 1;
            }

            oam_scan_idx += 1;
        }
        // log("[ppu] oam scan found {} sprites", .{self.oam_buffer_count});
    }

    fn read_oam_entry(self: *Lcd, idx: u8) OamEntry {
        std.debug.assert(idx < 40);
        return OamEntry{
            .y = self.bus.?.oam[4 * idx],
            .x = self.bus.?.oam[4 * idx + 1],
            .tile_idx = self.bus.?.oam[4 * idx + 2],
            .flags = @bitCast(self.bus.?.oam[4 * idx + 3]),
        };
    }

    fn draw_pixel(self: *Lcd, px: u8, ly: u8) void {
        var bg_win_pixel: u8 = 0b11;

        if (self.lcdc.bg_win_en == 1) {
            // Window is not transparent, it completely overlaps the background.
            const in_window = ly >= self.wy and (px + 7) >= self.wx;
            if (self.lcdc.wi_en == 1 and in_window) {
                const win_x = px + 7 - self.wx; // because px+7 >= wx, this will never underflow
                const win_y = ly - self.wy;
                const tile_idx = self.get_win_tile_idx(win_x / 8, win_y / 8);
                bg_win_pixel = self.get_pixel_from_tile(tile_idx, win_x % 8, win_x % 8, false);
            } else {
                const bg_x = (self.scx +% px);
                const bg_y = (self.scy +% ly);
                const tile_idx = self.get_bg_tile_idx(bg_x / 8, bg_y / 8);
                bg_win_pixel = self.get_pixel_from_tile(tile_idx, bg_x % 8, bg_y % 8, false);
            }
        }

        const bg_color: u8 = (self.bg_palette >> (2 * @as(u3, @truncate(bg_win_pixel)))) & 0b11;
        self.write_framebuffer(px, ly, bg_color);

        if (self.lcdc.ob_en == 1 and false) {
            // Draw object
            for (0..self.oam_buffer_count) |idx| {
                const ox = self.oam_buffer[idx].x;
                if (ox <= px and (px - ox) < 8) {
                    const obj_pixel = self.get_obj_pixel(self.oam_buffer[idx], px, ly);

                    const draw_obj = (self.lcdc.bg_win_en == 0) or
                        (self.oam_buffer[idx].flags.priority == 0) or
                        bg_win_pixel != 0b00;

                    if (draw_obj and obj_pixel != 0b00) {
                        const palette = if (self.oam_buffer[idx].flags.palette == 0) self.ob0_palette else self.ob1_palette;
                        const ob_color: u8 = (palette >> (2 * @as(u3, @truncate(obj_pixel)))) & 0b11;
                        self.write_framebuffer(px, ly, ob_color);
                    }

                    break;
                }
            }
        }
    }

    fn get_bg_tile_idx(self: *Lcd, tile_x: u16, tile_y: u16) u8 {
        const address = self.lcdc.bg_tm_base() + (tile_y * 32 + tile_x);
        return self.bus.?.read(address);
    }

    fn get_win_tile_idx(self: *Lcd, tile_x: u16, tile_y: u16) u8 {
        const address = self.lcdc.win_tm_base() + (tile_y * 32 + tile_x);
        return self.bus.?.read(address);
    }

    fn get_obj_pixel(self: *Lcd, obj: OamEntry, px: u8, ly: u8) u8 {
        std.debug.assert(obj.x <= px and (px - obj.x) < 8);
        std.debug.assert(obj.y <= ly and (ly - obj.y) < 16);

        var tile_x = px - obj.x;
        var tile_y = ly - obj.y;

        if (obj.flags.x_flip == 1) tile_x = 8 - tile_x;
        if (obj.flags.y_flip == 1) {
            tile_x = if (self.lcdc.ob_sz == 1) (16 - tile_y) else (8 - tile_y);
        }

        var tile_idx: u8 = 0;
        if (self.lcdc.ob_sz == 1) {
            if (tile_y >= 8) {
                tile_idx = obj.tile_idx | 0x01;
                tile_y = tile_y % 8;
            } else {
                tile_idx = obj.tile_idx & 0xfe;
            }
        } else {
            tile_idx = obj.tile_idx;
        }

        return self.get_pixel_from_tile(tile_idx, tile_x, tile_y, true);
    }

    fn get_pixel_from_tile(self: *Lcd, tile_idx: u8, x: u8, y: u8, comptime is_obj: bool) u8 {
        std.debug.assert(x < 8);
        std.debug.assert(y < 8);

        const addr_mode = if (is_obj) 0x8000 else self.lcdc.bg_wn_td_base();

        const tile_data_start = switch (addr_mode) {
            0x8000 => 0x8000 + @as(u16, tile_idx) * 16,
            0x8800 => if (tile_idx > 127)
                0x8800 + @as(u16, tile_idx - 128) * 16
            else
                0x9000 + @as(u16, tile_idx) * 16,
            else => unreachable,
        };

        const line_start: u16 = tile_data_start + (2 * @as(u16, y)); // 2 bytes per line
        const lsb = self.bus.?.read(line_start);
        const msb = self.bus.?.read(line_start + 1);
        // 8 - x because x is left to right, but bit 7 is left most (so x 0 must be bit 7)
        const bit_l = lsb >> @as(u3, @truncate(7 - x));
        const bit_m = msb >> @as(u3, @truncate(7 - x));
        const color_id = ((bit_m << 1) & 0b10) | (bit_l & 0b01);
        return color_id;
    }

    fn set_ppu_mode(self: *Lcd, new_mode: u2) void {
        self.stat.ppu_mode = new_mode;
        // log("[ppu] mode := {}", .{new_mode});

        if (new_mode == 0 and self.stat.m0_int_sel == 1) {
            self.bus.?.io.ir_if.lcd = 1;
        } else if (new_mode == 1 and self.stat.m1_int_sel == 1) {
            self.bus.?.io.ir_if.lcd = 1;
        } else if (new_mode == 2 and self.stat.m2_int_sel == 1) {
            self.bus.?.io.ir_if.lcd = 1;
        }

        if (new_mode == 1) {
            std.log.debug("VBLANK", .{});
            if (self.bus) |b| {
                std.log.debug("request vblank int", .{});
                b.io.ir_if.vblank = 1;
            }
        }
    }

    fn set_stat_ly(self: *Lcd, ly: u8) void {
        self.ly = ly;
        const lyc_eq = ly == self.lyc;
        self.stat.lyc_eq = @intFromBool(lyc_eq);

        if (self.stat.lyc_int_sel == 1) {
            log("[lcd] request stat intr", .{});
            self.bus.?.io.ir_if.lcd = 1;
        }
    }

    fn write_framebuffer(self: *Lcd, x: u8, y: u8, color: u8) void {
        std.debug.assert(x < 160);
        std.debug.assert(y < 144);
        std.debug.assert(color <= 0b11);

        // log("write fb {} {} = {}", .{ x, y, color });
        self.framebuffer[@as(usize, y) * 160 + x] = color;
    }

    pub fn get_pixel(self: *Lcd, x: usize, y: usize) u8 {
        return self.framebuffer[@as(usize, y) * 160 + x];
    }
};

const testing = std.testing;

test "Lcd Lcdc flag order" {
    var r = Lcd.Lcdc{};
    try testing.expectEqual(0, r.bg_win_en);
    r = @bitCast(@as(u8, 0b0000_0001));
    try testing.expectEqual(1, r.bg_win_en);
}

test "Lcd Stat flag order" {
    var r = Lcd.Stat{};
    try testing.expectEqual(0, r.ppu_mode);
    r = @bitCast(@as(u8, 0b0000_0010));
    try testing.expectEqual(2, r.ppu_mode);
}

test "Lcd Stat Stat ro bits" {
    var lcd = Lcd{};
    lcd.stat = @bitCast(@as(u8, 0xff));
    lcd.write(0xff41, 0);
    // check that the compare reg and ppu mode did not get zeroed
    const stat_raw: u8 = @bitCast(lcd.stat);
    try testing.expectEqual(0b1000_0111, stat_raw);
}
