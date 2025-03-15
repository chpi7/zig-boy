const std = @import("std");

/// 🚎 Brumm brumm!
pub const Bus = struct {
    fake_memory: [2]u8 = .{ 0, 0 },

    pub fn read(self: *Bus, address: u16) u8 {
        const result = self.fake_memory[address % 2];
        std.log.debug("Bus::read({}) -> {}", .{ address, result });
        return result;
    }

    pub fn write(self: *Bus, address: u16, value: u8) void {
        self.fake_memory[address % 2] = value;
        std.log.debug("Bus::write({}, {})", .{ address, value });
    }
};
