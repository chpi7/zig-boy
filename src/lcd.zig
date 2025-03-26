const std = @import("std");
const sys = @import("sys.zig");
const Bus = sys.Bus;

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
        _padding: u1 = 1,

        fn write(self: *Stat, value: u8) void {
            const write_mask: u8 = 0b0111_1000;
            self.* = @bitCast(@as(u8, @bitCast(self.*)) & ~write_mask | (write_mask & value));
        }
    };

    const OamEntry = packed struct {
        flags: u8 = 0,
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
    stat: Stat = Stat{},
    bus: ?*Bus = null,

    // PPU stuff:
    ppu_ly: u8 = 0,
    ppu_dot_line: u16 = 0,
    oam_buffer: [10]OamEntry = [_]OamEntry{OamEntry{}} ** 10,
    oam_buffer_count: usize = 0,
    framebuffer: [23040]u8 = [_]u8{0} ** 23040, // row major 160 * 144 screen

    pub fn write(self: *Lcd, address: u16, value: u8) void {
        switch (address) {
            0xff40 => self.lcdc = @bitCast(value), // lcdc
            0xff41 => self.stat.write(value),
            0xff44 => {}, // ly, read only
            0xff45 => self.lyc = value, // lyc
            0xff46 => self.bus.?.init_oam_dma(value),
            else => {}, // dont care
        }
        if (0xff40 <= address and address <= 0xff45) {
            std.log.debug("[lcd] write {x:02}", .{value});
        }
    }

    pub fn read(self: *Lcd, address: u16) u8 {
        std.debug.assert(0 <= self.ly and self.ly <= 153);
        const result: u8 = switch (address) {
            0xff40 => @bitCast(self.lcdc),
            0xff41 => @bitCast(self.stat),
            0xff44 => self.ly,
            0xff45 => self.lyc,
            else => 0, // dont care, return 0 aka no signal on any line
        };
        if (0xff40 <= address and address <= 0xff45) {
            std.log.debug("[lcd] read {x:02}", .{result});
        }
        return result;
    }

    /// This drives the PPU drawing to the screen
    ///
    /// One frame takes 70224 dots, with 1 m cycle == 4 dots.
    pub fn tick_1m(self: *Lcd) void {
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
        // std.log.debug("[ppu] oam scan found {} sprites", .{self.oam_buffer_count});
    }

    fn read_oam_entry(self: *Lcd, idx: u8) OamEntry {
        std.debug.assert(idx < 40);
        return OamEntry{
            .y = self.bus.?.oam[4 * idx],
            .x = self.bus.?.oam[4 * idx + 1],
            .tile_idx = self.bus.?.oam[4 * idx + 2],
            .flags = self.bus.?.oam[4 * idx + 3],
        };
    }

    fn draw_pixel(self: *Lcd, px: u8, ly: u8) void {
        if (self.lcdc.bg_win_en == 1) {
            // Window is not transparent, it completely overlaps the background.
            const in_window = ly >= self.wy and (px + 7) >= self.wx;
            if (self.lcdc.wi_en == 1 and in_window) {
                // Draw window
                const win_x = px + 7 - self.wx; // because px+7 >= wx, this will never underflow
                const win_y = ly - self.wy;
                const tile_idx = self.get_win_tile_idx(win_x / 8, win_y / 8);
                const pixel = self.get_pixel_from_tile(tile_idx, win_x % 8, win_x % 8, false);
                self.write_framebuffer(px, ly, pixel);
            } else {
                // Draw background
                const bg_x = (self.scx +% px);
                const bg_y = (self.scy +% ly);
                const tile_idx = self.get_bg_tile_idx(bg_x / 8, bg_y / 8);
                const pixel = self.get_pixel_from_tile(tile_idx, bg_x % 8, bg_y % 8, false);
                self.write_framebuffer(px, ly, pixel);
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
        const pixel = ((bit_m << 1) & 0b10) | (bit_l & 0b01);
        return pixel;
    }

    fn set_ppu_mode(self: *Lcd, new_mode: u2) void {
        self.stat.ppu_mode = new_mode;
        // std.log.debug("[ppu] mode := {}", .{new_mode});

        if (new_mode == 0 and self.stat.m0_int_sel == 1) {
            self.bus.?.io.ir_if.lcd = 1;
        } else if (new_mode == 1 and self.stat.m1_int_sel == 1) {
            self.bus.?.io.ir_if.lcd = 1;
        } else if (new_mode == 2 and self.stat.m2_int_sel == 1) {
            self.bus.?.io.ir_if.lcd = 1;
        }
    }

    fn set_stat_ly(self: *Lcd, ly: u8) void {
        self.ly = ly;
        const lyc_eq = ly == self.lyc;
        self.stat.lyc_eq = @intFromBool(lyc_eq);

        if (self.stat.lyc_int_sel == 1) {
            self.bus.?.io.ir_if.lcd = 1;
        }
    }

    fn write_framebuffer(self: *Lcd, x: u8, y: u8, pixel: u8) void {
        std.debug.assert(x < 160);
        std.debug.assert(y < 144);
        std.debug.assert(pixel <= 0b11);

        std.log.debug("write fb {} {} = {}", .{ x, y, pixel });
        self.framebuffer[@as(usize, y) * 160 + x] = pixel;
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
