const std = @import("std");

/// 🚎 Brumm brumm!
pub const Bus = struct {
    fake_memory: [4]u8 = .{ 1, 2, 3, 4 },

    pub fn read(_: *Bus, address: u16) u8 {
        const result = 0xff;
        std.log.debug("Bus::read({}) -> ", .{ address, result });
        return result;
    }

    pub fn read_ptr(self: *Bus, address: u16) [*]const u8 {
        const result = 0xff;
        std.log.debug("Bus::read({}) -> ", .{ address, result });
        return &self.fake_memory;
    }

    pub fn write(_: *Bus, address: u16, value: u8) void {
        std.log.debug("Bus::write({}, {})", .{ address, value });
    }
};
