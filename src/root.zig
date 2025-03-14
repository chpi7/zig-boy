const std = @import("std");
pub const cpu = @import("cpu.zig");
pub const cartridge = @import("cartridge.zig");

test {
    std.testing.refAllDeclsRecursive(@This());
}
