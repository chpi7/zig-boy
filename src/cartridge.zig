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

pub const Cartridge = struct {
    data: std.ArrayList(u8),

    pub fn deinit(self: *Cartridge) void {
        self.data.deinit();
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

    pub fn verify(self: *Cartridge) void {
        const hcs = self.compute_header_checksum();
        const hcs_header = self.get_header_entry(HdOff.header_checksum);
        std.log.debug("Header Checksum: Computed = {x}, Header = {x}", .{ hcs, hcs_header });

        if (hcs != hcs_header) {
            std.log.debug("Error: Invalid Header Checksum", .{});
            return;
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

        var cartridge = Cartridge{ .data = std.ArrayList(u8).init(allocator) };
        try cartridge.data.resize(file_size);

        const actually_read = try file.readAll(cartridge.data.items);
        if (actually_read != file_size) {
            std.log.err("Bytes read vs file size mismatch! (expected {}, found {})", .{ file_size, actually_read });
        }

        std.log.debug("ROM {s} loaded (size = 0x{x}, {})", .{ filename, file_size, file_size });

        cartridge.verify();

        return cartridge;
    }
};
