const std = @import("std");

pub const Lcd = struct {
    const Lcdc = packed struct { // The LCDC LCD Control register
        bg_we_pr: u1 = 0,
        ob_en: u1 = 0,
        ob_sz: u1 = 0,
        bg_tm: u1 = 0,
        bg_wt: u1 = 0,
        wi_en: u1 = 0,
        wi_tm: u1 = 0,
        en: u1 = 0,
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
            const write_mask: u8 = 0b1111_1000;
            self.* = @bitCast(@as(u8, @bitCast(self.*)) & ~write_mask | (write_mask & value));
        }
    };

    lcdc: Lcdc = Lcdc{},
    ly: u8 = 0x00, // current scanline. range 0-153, 144-153 indicates VBLANK
    lyc: u8 = 0, // LY compare
    stat: Stat = Stat{},

    pub fn write(self: *Lcd, address: u16, value: u8) void {
        switch (address) {
            0xff40 => self.lcdc = @bitCast(value), // lcdc
            0xff41 => self.stat.write(value),
            0xff44 => {}, // ly, read only
            0xff45 => self.lyc = value, // lyc
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
};

const testing = std.testing;

test "Lcd Lcdc flag order" {
    var r = Lcd.Lcdc{};
    try testing.expectEqual(0, r.bg_we_pr);
    r = @bitCast(@as(u8, 0b0000_0001));
    try testing.expectEqual(1, r.bg_we_pr);
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
    try testing.expectEqual(0b0000_0111, stat_raw);
}
