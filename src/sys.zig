const std = @import("std");
const cartridge = @import("cartridge.zig");
const lcd = @import("lcd.zig");

const Lcd = lcd.Lcd;

/// Interrupt enable / flag register layout
const Ieif = packed struct {
    vblank: u1 = 0,
    lcd: u1 = 0,
    timer: u1 = 0,
    serial: u1 = 0,
    joypad: u1 = 0,
    padding: u3 = 0,
};

/// The JOYP/P1 register
const JoypP1 = packed struct {
    // 1 == not pressed
    a_r: u1 = 1,
    b_l: u1 = 1,
    sel_u: u1 = 1,
    sta_d: u1 = 1,
    // 0 == <src> is selected (either dpad or buttons)
    dpad: u1 = 0,
    butt: u1 = 0,
    _padding: u2 = 0,

    fn write(self: *JoypP1, value: u8) void {
        const write_mask: u8 = 0b1111_0000;
        self.* = @bitCast(@as(u8, @bitCast(self.*)) & ~write_mask | (write_mask & value));
    }

    fn read(self: *JoypP1) u8 {
        // TODO actually hook up input here at some point
        return @bitCast(self.*);
    }
};

pub const Timer = struct {
    div: u8 = 0, // incremented at 16384Hz
    tima: u8 = 0, // incremented at the frequency specified by tac
    tma: u8 = 0,
    tac: u8 = 0,

    tick_1m_count: u64 = 0,
    tick_4m_count: u64 = 0,
    tima_modulo: u64 = 64, // clock select 0 equals increment every 256M cycles. 256 / 4 == 64.

    bus: ?*Bus = null,

    fn write(self: *Timer, address: u16, value: u8) void {
        switch (address) {
            0xff04 => self.div = 0, // writing any value resets the reg to 0
            0xff05 => self.tima = value,
            0xff06 => self.tma = value,
            0xff07 => {
                self.tac = (value & 0b0000_0111);
                self.tima_modulo = switch (@as(u2, @truncate(self.tac & 0b11))) {
                    0b00 => 64, // 256 / 4
                    0b01 => 1, // 4 / 4
                    0b10 => 4, // 16 / 4
                    0b11 => 16, // 64 / 4
                };
            },
            else => {},
        }
    }

    fn read(self: *Timer, address: u16) u8 {
        return switch (address) {
            0xff04 => self.div,
            0xff05 => self.tima,
            0xff06 => self.tma,
            0xff07 => self.tac,
            else => 0,
        };
    }

    inline fn tima_enabled(self: *Timer) bool {
        return (self.tac & 0b100) != 0;
    }

    /// This is the external timer input, called every m cycle.
    pub fn tick_1m(self: *Timer) void {
        self.tick_1m_count = (self.tick_1m_count +% 1) % 4;
        if (self.tick_1m_count == 0) self.tick_4m();
    }

    /// This should be called every 4th m cycle.
    /// This is the fastest frequency that TIMA increments at and 16x the
    /// frequency that DIV has to increment at.
    fn tick_4m(self: *Timer) void {
        self.tick_4m_count += 1;

        if (self.tick_4m_count % 16 == 0) {
            self.div +%= 1;
        }

        if (self.tima_enabled() and (self.tick_4m_count % self.tima_modulo) == 0) {
            self.tima +%= 1;
            if (self.tima == 0) {
                self.tima = self.tma;
                std.debug.assert(self.bus != null); // call bus.link()!
                self.bus.?.io.ir_if.timer = 1;
            }
        }
    }
};

const Io = struct {
    pub const R = enum { joy, serial_sb, serial_sc, ir_if, audio, other };

    pub fn ioreg(a: u16) R {
        return switch (a) {
            0xff00 => R.joy,
            0xff01 => R.serial_sb,
            0xff02 => R.serial_sc,
            0xff0f => R.ir_if,
            0xff10...0xff26 => R.audio,
            else => R.other,
        };
    }

    serial: SerialPort = .{},
    /// IF in docs (but is interrupt request)
    ir_if: Ieif = .{},
    joy: JoypP1 = .{},
    lcd: Lcd = .{},
    timer: Timer = .{},

    pub fn write(self: *Io, address: u16, value: u8) void {
        const r = ioreg(address);
        std.log.debug("[io]  {} := {}", .{ r, value });

        // start a new approach of less annoying implementation here:
        // just write through to everything and let the receiver discard
        // the write if the address isn't correct. They check the address
        // anyways to know where they need to write in 99% of cases.
        // So checking here is a bit pointless.
        // Really, it is pointless to route the reads/writes at all if it isn't
        // targeting a register directly already.

        self.lcd.write(address, value);
        self.timer.write(address, value);

        switch (r) {
            R.joy => self.joy.write(value),
            R.serial_sb => self.serial.set_sb(value),
            R.serial_sc => self.serial.set_sc(value),
            R.ir_if => self.ir_if = @bitCast(value),
            else => {},
        }
    }

    pub fn read(self: *Io, address: u16) u8 {
        const r = ioreg(address);
        // std.log.debug("[io] u8 {} <- {}", .{ r, value });
        var result: u8 = switch (r) {
            R.joy => self.joy.read(),
            R.serial_sb => self.serial.sb,
            R.serial_sc => self.serial.sc,
            R.ir_if => @bitCast(self.ir_if),
            else => 0x00,
        };

        // This is basically what a bus would do. Only one connected device
        // should answer anyways.
        result |= self.lcd.read(address);
        result |= self.timer.read(address);

        std.log.debug("[io] read {} (={})", .{ r, result });
        return result;
    }
};

const SerialPort = struct {
    dbg_out_buf_pos: usize = 0,
    dbg_out_buf: [64]u8 = [_]u8{' '} ** 64,

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
            self.dbg_out_buf[self.dbg_out_buf_pos] = SerialPort.replace_printable(self.sb);
            self.dbg_out_buf_pos = (self.dbg_out_buf_pos + 1) % self.dbg_out_buf.len;
            if (self.dbg_out_buf_pos == 0) {
                for (&self.dbg_out_buf) |*b| {
                    b.* = 0;
                }
            }
            self.print_buf();
            // TODO: request interrupt (8 serial clock cycles after sc has been set)
        }
    }

    fn replace_printable(c: u8) u8 {
        return switch (c) {
            0 => '.',
            '\n' => ' ',
            '\t' => ' ',
            else => c,
        };
    }

    fn print_buf(self: *SerialPort) void {
        std.log.debug("[serial] {s}", .{self.dbg_out_buf});
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
    pub const fake_mem_size: usize = 256;

    cartridge: ?*cartridge.Cartridge = null,
    fake_memory: [256]u8 = [_]u8{0} ** 256, // used in unit tests
    io: Io = .{},
    ir_ie: Ieif = .{},
    wram_0: [4096]u8 = [_]u8{0} ** 4096,
    wram_1: [4096]u8 = [_]u8{0} ** 4096, // Non CBG only has one bank here
    hram: [0x80]u8 = [_]u8{0} ** 0x80, // technically only 0x79, from ff80-fffe. Mapper handles that though.

    /// This set the Bus pointer in the various children of the bus.
    ///
    /// Mostly to allow accessing the interrupt request register.
    pub fn link(self: *Bus) void {
        self.io.timer.bus = self;
    }

    pub fn read(self: *Bus, address: u16) u8 {
        const Region = MemoryMap.Region;
        const unmapped_result = 0x00;
        const result: u8 = switch (MemoryMap.region(address)) {
            Region.rom_bank_0 => self.read_cartridge(address),
            Region.rom_bank_n => self.read_cartridge(address),
            Region.vram => unmapped_result,
            Region.eram => self.read_cartridge(address),
            Region.wram_0 => self.wram_0[@as(u12, @truncate(address - 0xc000))],
            Region.wram_n => self.wram_1[@as(u12, @truncate(address - 0xd000))],
            Region.echo => unmapped_result, // use prohibited
            Region.oam => unmapped_result,
            Region.unused => unmapped_result, // not used
            Region.io => self.io.read(address),
            Region.hram => self.hram[@as(u7, @truncate(address - 0xff80))],
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
            Region.eram => self.write_cartridge(address, value),
            Region.wram_0 => {
                self.wram_0[@as(u12, @truncate(address - 0xc000))] = value;
            },
            Region.wram_n => {
                self.wram_1[@as(u12, @truncate(address - 0xd000))] = value;
            },
            Region.echo => {}, // use prohibited
            Region.oam => {},
            Region.unused => {}, // not used
            Region.io => self.io.write(address, value),
            Region.hram => {
                self.hram[@as(u7, @truncate(address - 0xff80))] = value;
            },
            Region.ie => {
                self.ir_ie = @bitCast(value);
                std.log.debug("[bus] set IE = {}", .{self.ir_ie});
            },
        }
    }

    fn read_cartridge(self: *Bus, address: u16) u8 {
        if (self.cartridge) |c| {
            return c.read(address);
        } else {
            return self.fake_memory[address];
        }
    }

    fn write_cartridge(self: *Bus, address: u16, value: u8) void {
        if (self.cartridge) |c| {
            c.write(address, value);
        } else {
            self.fake_memory[address] = value;
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

test "R/W wram 0" {
    var bus = Bus{};
    const base = 0xc000;
    const size = 4096;

    for (0..size) |i| {
        bus.write(@truncate(base + i), @truncate(i));
    }
    for (0..size) |i| {
        try testing.expectEqual(i % 256, bus.read(@truncate(base + i)));
    }
}

test "R/W wram n" {
    var bus = Bus{};
    const base = 0xd000;
    const size = 4096;

    for (0..size) |i| {
        bus.write(@truncate(base + i), @truncate(i));
    }
    for (0..size) |i| {
        try testing.expectEqual(i % 256, bus.read(@truncate(base + i)));
    }
}

test "R/W hram" {
    var bus = Bus{};
    const base = 0xff80;
    const size = 0xfffe - 0xff80 + 1;

    for (0..size) |i| {
        bus.write(@truncate(base + i), @truncate(i));
    }
    for (0..size) |i| {
        try testing.expectEqual(i % 256, bus.read(@truncate(base + i)));
    }
}
