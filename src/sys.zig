const std = @import("std");

/// 🚎 Brumm brumm!
pub const Bus = struct {
    fake_memory: [4]u8 = .{ 1, 2, 3, 4 },

    pub fn read(_: *Bus, address: u16) u8 {
        const result = 0xff;
        std.log.debug("Bus::read({}) -> ", .{ address, result });
        return result;
    }

    /// Turn a GB address into a host pointer that we can read from or write to.
    pub fn addr_to_ptr(self: *Bus, address: u16) [*]const u8 {
        const result = 0xff;
        std.log.debug("Bus::addr_to_ptr({}) -> {}", .{ address, result });
        return &self.fake_memory;
    }

    pub fn write(_: *Bus, address: u16, value: u8) void {
        std.log.debug("Bus::write({}, {})", .{ address, value });
    }
};
