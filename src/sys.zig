const std = @import("std");
const cartridge = @import("cartridge.zig");

/// Interrupt enable / flag register layout
const Ieif = packed struct {
    vblank: u1 = 0,
    lcd: u1 = 0,
    timer: u1 = 0,
    serial: u1 = 0,
    joypad: u1 = 0,
    padding: u3 = 0,
};

const Io = struct {
    pub const R = enum { serial_sb, serial_sc, ir_if, audio, not_impl };

    pub fn ioreg(a: u16) R {
        return switch (a) {
            0xff01 => R.serial_sb,
            0xff02 => R.serial_sc,
            0xff0f => R.ir_if,
            0xff10...0xff26 => R.audio,
            else => R.not_impl,
        };
    }

    serial: SerialPort = .{},
    ir_if: Ieif = .{}, // IF in docs (but is interrupt request)

    pub fn write(self: *Io, address: u16, value: u8) void {
        const r = ioreg(address);
        std.log.debug("[io]  {} := {}", .{ r, value });
        switch (r) {
            R.serial_sb => self.serial.set_sb(value),
            R.serial_sc => self.serial.set_sc(value),
            R.ir_if => self.ir_if = @bitCast(value),
            R.not_impl, R.audio => {},
        }
    }

    pub fn read(self: *Io, address: u16) u8 {
        const r = ioreg(address);
        // std.log.debug("[io] u8 {} <- {}", .{ r, value });
        const result: u8 = switch (r) {
            R.serial_sb => self.serial.sb,
            R.serial_sc => self.serial.sc,
            R.ir_if => @bitCast(self.ir_if),
            R.not_impl, R.audio => 0xff,
        };
        std.log.debug("[io]  read {} (={})", .{ r, result });
        return result;
    }
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
            self.print_buf();
        }
    }

    fn print_buf(self: *SerialPort) void {
        std.mem.reverse(u8, &self.ouput_debug_buffer);
        std.log.debug("[serial] {s}", .{self.ouput_debug_buffer});
        std.mem.reverse(u8, &self.ouput_debug_buffer);
    }
};

const MemoryMap = struct {
    pub const Region = enum { rom_bank_0, rom_bank_n, vram, eram, wram_0, wram_n, echo, oam, unused, io, hram, ie };

    pub fn region(a: u16) Region {
        return switch (a) {
            0x0000...0x3fff => Region.rom_bank_0,
            0x4000...0x7fff => Region.rom_bank_n,
            0x8000...0x9fff => Region.vram,
            0xa000...0xbfff => Region.eram,
            0xc000...0xcfff => Region.wram_0,
            0xd000...0xdfff => Region.wram_n,
            0xe000...0xfdff => Region.echo,
            0xfe00...0xfe9f => Region.oam,
            0xfea0...0xfeff => Region.unused,
            0xff00...0xff7f => Region.io,
            0xff80...0xfffe => Region.hram,
            0xffff...0xffff => Region.ie,
        };
    }
};

/// 🚎 Brumm brumm!
pub const Bus = struct {
    cartridge: ?*cartridge.Cartridge = null,
    fake_memory: [2]u8 = .{ 0, 0 },
    io: Io = .{},
    ir_ie: Ieif = .{},

    pub fn read(self: *Bus, address: u16) u8 {
        const Region = MemoryMap.Region;
        const unmapped_result = 0x00;
        const result: u8 = switch (MemoryMap.region(address)) {
            Region.rom_bank_0 => self.read_cartridge(address),
            Region.rom_bank_n => self.read_cartridge(address),
            Region.vram => unmapped_result,
            Region.eram => unmapped_result,
            Region.wram_0 => unmapped_result,
            Region.wram_n => unmapped_result,
            Region.echo => unmapped_result, // use prohibited
            Region.oam => unmapped_result,
            Region.unused => unmapped_result, // not used
            Region.io => self.io.read(address),
            Region.hram => unmapped_result,
            Region.ie => @bitCast(self.ir_ie),
        };

        std.log.debug("[bus] read({x:04}) -> {x:02}", .{ address, result });
        return result;
    }
    pub fn write(self: *Bus, address: u16, value: u8) void {
        const Region = MemoryMap.Region;

        const region = MemoryMap.region(address);
        std.log.debug("[bus] write({x:04}, {x:02}) ({})", .{ address, value, region });

        switch (region) {
            Region.rom_bank_0 => self.write_cartridge(address, value),
            Region.rom_bank_n => self.write_cartridge(address, value),
            Region.vram => {},
            Region.eram => {},
            Region.wram_0 => {},
            Region.wram_n => {},
            Region.echo => {}, // use prohibited
            Region.oam => {},
            Region.unused => {}, // not used
            Region.io => self.io.write(address, value),
            Region.hram => {},
            Region.ie => {
                self.ir_ie = @bitCast(value);
                std.log.debug("[bus] set IE = {}", .{self.ir_ie});
            },
        }
    }

    fn read_cartridge(self: *Bus, address: u16) u8 {
        if (self.cartridge) |c| {
            return c.data.items[address];
        } else {
            return self.fake_memory[address % 2];
        }
    }

    fn write_cartridge(self: *Bus, address: u16, value: u8) void {
        if (self.cartridge) |c| {
            c.data.items[address] = value;
        } else {
            self.fake_memory[address % 2] = value;
        }
    }
};

const testing = std.testing;

test "IE Type" {
    var ie = Ieif{};
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
