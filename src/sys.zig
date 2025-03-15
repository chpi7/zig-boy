const std = @import("std");
const cartridge = @import("cartridge.zig");

const MemoryMap = struct {
    pub const serial_data: u16 = 0xff01;
    pub const serial_ctrl: u16 = 0xff02;
    pub const ie: u16 = 0xffff;
    pub const ir: u16 = 0xff0f;
    pub const audio_b: u16 = 0xff10;
    pub const audio_e: u16 = 0xff26;
};

const SerialPort = struct {
    ouput_debug_buffer: [256]u8 = [_]u8{0} ** 256,
    sb: u8 = 0,
    sc: u8 = 0,

    pub fn set_sb(self: *SerialPort, data: u8) void {
        self.sb = data;
    }

    pub fn set_sc(self: *SerialPort, data: u8) void {
        if (data == 0x81) {
            // set to 0x81, but immediately clear transfer in flight bit
            self.sc = 0x81 & 0b0111_1111;
            std.log.debug("[serial] tx requested", .{});
            std.mem.rotate(u8, &self.ouput_debug_buffer, 1);
            self.ouput_debug_buffer[self.ouput_debug_buffer.len - 1] = self.sb;
        }
    }

    fn print_buf(self: *SerialPort) void {
        std.mem.reverse(u8, &self.ouput_debug_buffer);
        std.log.debug("[serial] {s}", .{self.ouput_debug_buffer});
        std.mem.reverse(u8, &self.ouput_debug_buffer);
    }
};

const IEIF = packed struct {
    vblank: u1 = 0,
    lcd: u1 = 0,
    timer: u1 = 0,
    serial: u1 = 0,
    joypad: u1 = 0,
    padding: u3 = 0,
};

/// 🚎 Brumm brumm!
pub const Bus = struct {
    cartridge: ?*cartridge.Cartridge = null,
    fake_memory: [2]u8 = .{ 0, 0 },
    serial: SerialPort = .{},
    ie: IEIF = .{},
    ir: IEIF = .{}, // IF in docs (but is interrupt request)

    pub fn read(self: *Bus, address: u16) u8 {
        var result: u8 = 0;
        if (self.cartridge) |c| {
            result = c.data.items[address];
        } else {
            result = self.fake_memory[address % 2];
        }
        std.log.debug("[bus] read({x:04}) -> {x:02}", .{ address, result });
        return result;
    }

    pub fn write(self: *Bus, address: u16, value: u8) void {
        std.log.debug("[bus] write({x:04}, {x:02})", .{ address, value });
        switch (address) {
            MemoryMap.serial_data => {
                self.serial.set_sb(value);
            },
            MemoryMap.serial_ctrl => {
                self.serial.set_sc(value);
            },
            MemoryMap.ie => {
                self.ie = @bitCast(value);
                std.log.debug("[bus] set IE = {}", .{self.ie});
            },
            MemoryMap.ir => {
                self.ir = @bitCast(value);
                std.log.debug("[bus] set IF = {}", .{self.ir});
            },
            MemoryMap.audio_b...MemoryMap.audio_e => {
                std.log.debug("[bus] audio I/O unsupported", .{});
            },
            else => {},
        }

        if (self.cartridge) |c| {
            c.data.items[address] = value;
        } else {
            self.fake_memory[address % 2] = value;
        }
    }
};

const testing = std.testing;

test "IE Type" {
    var ie = IEIF{};
    try testing.expectEqual(0, ie.lcd);

    ie.lcd = 1;
    try testing.expectEqual(1, ie.lcd);

    ie.lcd = 0;
    try testing.expectEqual(0, ie.lcd);
    try testing.expectEqual(0, ie.joypad);

    ie = @bitCast(@as(u8, 0b0001_0010));
    try testing.expectEqual(1, ie.lcd);
    try testing.expectEqual(1, ie.joypad);
}
