const std = @import("std");

fn get_absolute_path(filename: []const u8, allocator: std.mem.Allocator) ![]u8 {
    const cwd = try std.fs.cwd().realpathAlloc(allocator, ".");
    defer allocator.free(cwd);

    return try std.fs.path.resolve(allocator, &.{ cwd, filename });
}

const CartridgeType = enum(u8) {
    rom_only = 0x00,
    mbc1 = 0x01,
    mbc1_ram = 0x02,
    mbc1_ram_battery = 0x03,
    mbc2 = 0x05,
    mbc2_battery = 0x06,
    rom_ram = 0x08,
    rom_ram_battery = 0x09,
    mmm01 = 0x0b,
    mmm01_ram = 0x0c,
    mmm01_ram_battery = 0x0d,
    mbc3_timer_battery = 0x0f,
    mbc3_timer_ram_battery = 0x10,
    mbc3 = 0x11,
    mbc3_ram = 0x12,
    mbc3_ram_battery = 0x13,
    mbc5 = 0x19,
    mbc5_ram = 0x1a,
    mbc5_ram_battery = 0x1b,
    mbc5_rumble = 0x1c,
    mbc5_rumble_ram = 0x1d,
    mbc5_rumble_ram_battery = 0x1e,
    mbc6 = 0x20,
    mbc7_sensor_rumble_ram_battery = 0x22,
    pocket_camera = 0xfc,
    bandai_tama5 = 0xfd,
    huc3 = 0xfe,
    huc1_ram_battery = 0xff,
};

const HdSz = enum(u16) {
    title = 0xf,
};

const HdOff = enum(u16) {
    entrypoint = 0x100,
    logo = 0x0104,
    title = 0x0134,
    cgb = 0x0143,
    new_lic = 0x0144,
    sgb = 0x0146,
    cart_type = 0x0147,
    rom_size = 0x0148,
    ram_size = 0x0149,
    dst_code = 0x014A,
    old_lic = 0x014B,
    version = 0x014C,
    header_checksum = 0x014D,
    global_checksum = 0x014E,
};

const Mbc1 = struct {
    const R = struct {
        ram_enable: u1 = 0,
        rom_bank: u5 = 1, // 0 is treated as 1, so set it to that
        rom_bank_hi: u2 = 0,
        bank_mode: u1 = 0,
    };

    const Testing = struct {
        addr_ram_enable: u16 = 0x0 << 13,
        addr_bank: u16 = 0x2 << 13,
        addr_bank_hi: u16 = 0x4 << 13,
        addr_mode: u16 = 0x6 << 13,
    };

    regs: R = .{},
    test_consts: Testing = .{},

    pub fn write(self: *Mbc1, addr: u16, v: u8) void {
        const reg_select: u3 = @truncate(addr >> 13);
        switch (reg_select) {
            0x0, 0x1 => if (v & 0x0f == 0xa) {
                self.regs.ram_enable = 1;
            } else {
                self.regs.ram_enable = 0;
            },
            0x2, 0x3 => {
                if (v == 0) {
                    self.regs.rom_bank = 1;
                } else {
                    self.regs.rom_bank = @truncate(v);
                }
            },
            0x4, 0x5 => self.regs.rom_bank_hi = @truncate(v),
            0x6, 0x7 => self.regs.bank_mode = @truncate(v),
        }
        std.log.debug("[mbc] {}", .{self.regs});
    }

    pub fn rom_address(self: *Mbc1, addr: u16) u21 {
        std.debug.assert(addr <= 0x7fff);

        // Bank 0: 0000 - 3fff
        //         | 20 19 | 18 .. 14 | 13 .. 0 |
        // mode 0  |   0   |    0     |   gb    |
        // mode 1  |bank_hi|    0     |   gb    |
        //
        // Bank N: 4000 - 7fff
        //         | 20 19 | 18 .. 14 | 13 .. 0 |
        // mode 0/1|bank_hi|   bank   |   gb    |

        const gb: u21 = @as(u14, @truncate(addr));

        if (addr >= 0x4000) {
            const hi: u21 = @as(u21, @intCast(self.regs.rom_bank_hi)) << 19;
            const mid: u21 = @as(u21, @intCast(self.regs.rom_bank)) << 14;
            return hi | mid | gb;
        } else if (self.regs.bank_mode == 1) {
            const hi: u21 = @as(u21, @intCast(self.regs.rom_bank_hi)) << 19;
            return hi | gb;
        } else {
            return gb;
        }
    }

    pub fn ram_address(self: *Mbc1, addr: u16) u15 {
        std.debug.assert(0xa000 <= addr and addr <= 0xbfff);
        const gb: u15 = @as(u13, @truncate(addr));
        if (self.regs.bank_mode == 0) {
            return gb;
        } else {
            const hi: u15 = @as(u15, @intCast(self.regs.rom_bank_hi)) << 13;
            return hi | gb;
        }
    }
};

pub const Cartridge = struct {
    ram: std.ArrayList(u8),
    data: std.ArrayList(u8),
    mapper: Mbc1 = .{},

    pub fn deinit(self: *Cartridge) void {
        self.ram.deinit();
        self.data.deinit();
    }

    pub fn write(self: *Cartridge, addr: u16, v: u8) void {
        self.mapper.write(addr, v); // update mapper regs

        if (addr <= 0x7fff) { // ROM
            // can't write to rom
        } else if (0xa000 <= addr and addr <= 0xbfff) { // RAM
            const ram_addr: usize = self.mapper.ram_address(addr);
            if (ram_addr < self.ram.items.len) {
                self.ram.items[ram_addr] = v;
            }
        }
    }

    pub fn read(self: *Cartridge, addr: u16) u8 {
        var result: u8 = 0;

        if (addr <= 0x7fff) { // ROM
            const rom_addr: usize = self.mapper.rom_address(addr);
            if (rom_addr < self.data.items.len) {
                result = self.data.items[rom_addr];
            }
        } else if (0xa000 <= addr and addr <= 0xbfff) { // RAM
            const ram_addr: usize = self.mapper.ram_address(addr);
            if (self.mapper.regs.ram_enable == 1) {
                if (ram_addr < self.ram.items.len) {
                    result = self.ram.items[ram_addr];
                }
            } else {
                result = 0xff; // this is a common return value when ram is disabled
            }
        }

        return result;
    }

    /// Return { ROM size, #banks }
    fn decode_rom_size(enc: u8) struct { usize, usize } {
        return .{ 32 * 1024 * (@as(usize, 1) << @intCast(enc)), @as(usize, 2) << @intCast(enc) };
    }

    /// Return total RAM size
    fn decode_ram_size(enc: u8) usize {
        return switch (enc) {
            0, 1 => 0,
            2 => 8 * 1024,
            3 => 32 * 1024,
            4 => 128 * 1024,
            5 => 64 * 1024,
            else => return 0,
        };
    }

    fn compute_header_checksum(self: *Cartridge) u8 {
        var checksum: u8 = 0;
        for (0x134..0x14D) |address| {
            checksum = checksum -% self.data.items[address] -% 1;
        }
        return checksum;
    }

    fn compute_global_checksum(self: *Cartridge) u16 {
        var checksum: u16 = 0;
        for (0..self.data.items.len) |address| {
            // skip the two checksum bytes itself in the sum
            if (address == 0x014e or address == 0x014f) continue;
            checksum +%= self.data.items[address];
        }
        return checksum;
    }

    fn get_header_entry(self: *Cartridge, field: HdOff) u8 {
        return self.data.items[@intFromEnum(field)];
    }

    pub fn verify(self: *Cartridge) bool {
        const hcs = self.compute_header_checksum();
        const hcs_header = self.get_header_entry(HdOff.header_checksum);
        std.log.debug("Header Checksum: Computed = {x}, Header = {x}", .{ hcs, hcs_header });

        if (hcs != hcs_header) {
            std.log.debug("Error: Invalid Header Checksum", .{});
            return false;
        }

        std.log.debug("Title: {s}", .{self.data.items[@intFromEnum(HdOff.title)..(@intFromEnum(HdOff.title) + @intFromEnum(HdSz.title))]});
        std.log.debug("CGB: 0x{x:02}", .{self.data.items[@intFromEnum(HdOff.cgb)]});
        std.log.debug("SGB: 0x{x:02}", .{self.data.items[@intFromEnum(HdOff.sgb)]});
        const ct: CartridgeType = @enumFromInt(self.data.items[@intFromEnum(HdOff.cart_type)]);
        std.log.debug("Cartridge Type: {s} (0x{x:02})", .{ @tagName(ct), @intFromEnum(ct) });
        const rom_size, const num_banks = decode_rom_size(self.data.items[@intFromEnum(HdOff.rom_size)]);
        std.log.debug("ROM Size: size = {}, {} banks", .{ rom_size, num_banks });
        const ram_size = decode_ram_size(self.data.items[@intFromEnum(HdOff.ram_size)]);
        std.log.debug("RAM Size: size = {}, {} banks", .{ ram_size, ram_size / 8 * 1024 });

        return true;
    }

    pub fn load(filename: []const u8, allocator: std.mem.Allocator) !Cartridge {
        const abs_input = get_absolute_path(filename, allocator) catch |err| {
            std.debug.print("Error getting absolute path. ({})", .{err});
            return err;
        };
        defer allocator.free(abs_input);

        const file = std.fs.openFileAbsolute(abs_input, .{}) catch |err| {
            std.debug.print("Error opening input file: {}", .{err});
            return err;
        };
        defer file.close();

        const file_size = try file.getEndPos();
        // TODO: sanity check size is inside a reasonable bound! (<= max ROM)

        var cartridge = Cartridge{
            .data = std.ArrayList(u8).init(allocator),
            .ram = std.ArrayList(u8).init(allocator),
        };
        try cartridge.data.resize(file_size);

        const actually_read = try file.readAll(cartridge.data.items);
        if (actually_read != file_size) {
            std.log.err("Bytes read vs file size mismatch! (expected {}, found {})", .{ file_size, actually_read });
        }

        std.log.debug("ROM {s} loaded (size = 0x{x}, {})", .{ filename, file_size, file_size });

        if (cartridge.verify()) {
            const ram_size = decode_ram_size(cartridge.data.items[@intFromEnum(HdOff.ram_size)]);
            if (ram_size == 0) {} else if (ram_size <= 8192 * 16) {
                // Only allocate RAM if it inside the max theoretical limit
                try cartridge.ram.resize(ram_size);
                std.log.debug("RAM allocated (size = 0x{x})", .{ram_size});
            } else {
                std.log.err("Header reported ram size > 128 KiB, not allocating RAM!", .{});
            }
        }

        return cartridge;
    }
};

// -----------------------------------------------------------------------------
const testing = std.testing;

test "mbc1 addressing rom 0" {
    var m = Mbc1{};

    // Set high bank
    m.write(m.test_consts.addr_bank_hi, 0b01);

    // This should not use the high bank
    const result_mode_0 = m.rom_address(0b0011_0000_0110_0000);

    // Now it should use it
    m.write(m.test_consts.addr_mode, 1);
    const result_mode_1 = m.rom_address(0b0011_0000_0110_0000);

    // Check that high bank was set or not depending on the mode
    const expect_mode_0: u21 = 0b0_0000_0011_0000_0110_0000;
    const expect_mode_1: u21 = 0b0_1000_0011_0000_0110_0000;
    try testing.expectEqual(expect_mode_0, result_mode_0);
    try testing.expectEqual(expect_mode_1, result_mode_1);
}

test "mbc1 addressing rom n" {
    var m = Mbc1{};

    // Set low bank
    m.write(m.test_consts.addr_bank, 0b11011);
    // Set high bank
    m.write(m.test_consts.addr_bank_hi, 0b01);

    // This should use the high bank
    const result_mode_0 = m.rom_address(0b0111_0000_0110_0000);

    // This should also use the high bank
    m.write(m.test_consts.addr_mode, 1);
    const result_mode_1 = m.rom_address(0b0111_0000_0110_0000);

    // Check that high bank is always set
    const expect: u21 = 0b0_1110_1111_0000_0110_0000;
    try testing.expectEqual(expect, result_mode_0);
    try testing.expectEqual(expect, result_mode_1);
}

test "mbc1 addressing ram" {
    var m = Mbc1{};

    // Set high bank
    m.write(m.test_consts.addr_bank_hi, 0b10);

    // This should not use the high bank
    const result_mode_0 = m.ram_address(0b1011_0000_0110_0000);

    // Now it should use it
    m.write(m.test_consts.addr_mode, 1);
    const result_mode_1 = m.ram_address(0b1011_0000_0110_0000);

    // Check that high bank was set or not depending on the mode
    const expect_mode_0: u15 = 0b001_0000_0110_0000;
    const expect_mode_1: u15 = 0b101_0000_0110_0000;
    try testing.expectEqual(expect_mode_0, result_mode_0);
    try testing.expectEqual(expect_mode_1, result_mode_1);
}
