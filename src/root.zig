const std = @import("std");
pub const cpu = @import("cpu.zig");

test {
    std.testing.refAllDeclsRecursive(@This());
}
