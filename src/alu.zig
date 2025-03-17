const std = @import("std");

const Flags = enum {
    Z,
    N,
    H,
    C,

    const F = Flags;

    pub fn offset(comptime flag: F) u2 {
        return switch (flag) {
            F.Z => 0,
            F.N => 1,
            F.H => 2,
            F.C => 3,
        };
    }

    fn mask(comptime flag: F) u4 {
        return @as(u4, 1) << F.offset(flag);
    }

    pub fn set(current: u4, comptime f: F, value: u1) u4 {
        return (current & ~mask(f)) | (@as(u4, @intCast(value)) << F.offset(f));
    }

    pub fn get(current: u4, comptime flag: F) u1 {
        return @truncate(current >> F.offset(flag));
    }

    pub fn c(current: u4) u1 {
        return @truncate(current >> F.offset(F.C));
    }

    pub fn h(current: u4) u1 {
        return @truncate(current >> F.offset(F.H));
    }

    pub fn z(current: u4) u1 {
        return @truncate(current >> F.offset(F.Z));
    }

    pub fn n(current: u4) u1 {
        return @truncate(current >> F.offset(F.N));
    }

    pub fn build(vc: u1, vh: u1, vn: u1, vz: u1) u4 {
        return (@as(u4, @intCast(vc)) << F.offset(F.C)) |
            (@as(u4, @intCast(vh)) << F.offset(F.H)) |
            (@as(u4, @intCast(vn)) << F.offset(F.N)) |
            (@as(u4, @intCast(vz)) << F.offset(F.Z));
    }
};

inline fn btoi(b: bool) u1 {
    return @intFromBool(b);
}

pub const Alu = struct {
    pub const op8_8_t = *const fn (a: u8, b: u8, f: u4) struct { u8, u4 };
    pub const op8_8_fl_t = *const fn (a: u8, b: u8, f: u4) u4;
    pub const op8_t = *const fn (a: u8, f: u4) struct { u8, u4 };
    pub const op16_16_t = *const fn (a: u16, b: u16, f: u4) struct { u16, u4 };
    pub const op16_t = *const fn (a: u16, f: u4) struct { u16, u4 };

    pub const F = Flags;

    pub fn adc8(a: u8, b: u8, f: u4) struct { u8, u4 } {
        // adc a+b == add a+b+c
        const c = F.c(f);
        const tmp_res, const flags1 = Alu.add8(b, c, f);
        const res, const flags2 = Alu.add8(a, tmp_res, f);
        const flags = F.build(
            F.c(flags1) | F.c(flags2),
            F.h(flags1) | F.h(flags2),
            0,
            btoi(res == 0),
        );

        return .{ res, flags };

        // const a: u16 = a_in;
        // const b: u16 = b_in;
        // const set_h = (((a & 0xf) +% (b & 0xf) +% F.c(f)) & 0x10) == 0x10;
        // const set_c = (((a & 0xff) +% (b & 0xff) +% F.c(f)) & 0x100) == 0x100;
        // const res = a_in +% b_in +% F.c(f);

        // return .{ res, F.build(btoi(set_c), btoi(set_h), 0, btoi(res == 0)) };
    }

    pub fn add8(a: u8, b: u8, _: u4) struct { u8, u4 } {
        const set_h = (((a & 0xf) +% (b & 0xf)) & 0x10) == 0x10;
        const res, const set_c = @addWithOverflow(a, b);

        return .{ res, F.build(set_c, btoi(set_h), 0, btoi(res == 0)) };
    }

    pub fn add16(a: u16, b: u16, f: u4) struct { u16, u4 } {
        const set_h = (((a & 0xfff) +% (b & 0xfff)) & 0x1000) == 0x1000;
        const res, const set_c = @addWithOverflow(a, b);

        return .{ res, F.build(set_c, btoi(set_h), 0, F.z(f)) };
    }

    pub fn add16i8(a: u16, b_in: i8, _: u4) struct { u16, u4 } {
        const b: u16 = @bitCast(@as(i16, b_in));
        const set_h = (((a & 0xf) +% (b & 0xf)) & 0x10) == 0x10;
        const set_c = (((a & 0xff) +% (b & 0xff)) & 0x100) == 0x100;
        const res = a +% b;

        return .{ res, F.build(btoi(set_c), btoi(set_h), 0, 0) };
    }

    pub inline fn bit(a: u3, b: u8, f: u4) u4 {
        const mask = 1 << a;
        return F.build(F.c(f), 1, 0, btoi((b & mask) != 0));
    }

    pub inline fn ccf(f: u4) u4 {
        return (f ^ F.mask(F.C)) & ~(F.mask(F.N) | F.mask(F.H));
    }

    pub fn cp(a: u8, b: u8, _: u4) u4 {
        const set_h = @as(u4, @truncate(b)) > @as(u4, @truncate(a)); // TODO: is this correct?
        return F.build(btoi(b > a), btoi(set_h), 1, btoi(a == b));
    }

    pub fn cpl(a: u8, f: u4) struct { u8, u4 } {
        return .{ ~a, F.build(F.c(f), 1, 1, F.z(f)) };
    }

    pub fn dda(a: u8, f: u4) struct { u8, u4 } {
        var adjust: u8 = 0;
        var res: u8 = 0;
        var carry: u1 = false;

        if (F.n(f)) {
            if (F.h(f) == 1) adjust += 0x6;
            if (F.c(f) == 1) adjust += 0x60;
            res = a -% adjust;
        } else {
            if (F.h(f) == 1 or (a & 0xf > 0x9)) adjust += 0x6;
            if (F.c(f) == 1 or (a > 0x99)) {
                carry = 1;
                adjust += 0x60;
            }
            res = a +% adjust;
        }

        return .{ res, F.build(carry, 0, F.n(f), btoi(res == 0)) };
    }

    pub fn inc8(a: u8, f: u4) struct { u8, u4 } {
        const res, const new_f = Alu.add8(a, 1, f);
        // inc should leave c alone, so restore it.
        return .{ res, (new_f & ~F.mask(F.C)) | F.c(f) };
    }

    pub fn inc16(a: u16, f: u4) struct { u16, u4 } {
        const res, _ = Alu.add16(a, 1, f);
        return .{ res, f };
    }

    pub fn or8(a: u8, b: u8, _: u4) struct { u8, u4 } {
        const res = a | b;
        return .{ res, F.build(0, 0, 0, btoi(res == 0)) };
    }

    pub fn and8(a: u8, b: u8, _: u4) struct { u8, u4 } {
        const res = a & b;
        return .{ res, F.build(0, 1, 0, btoi(res == 0)) };
    }

    pub fn xor8(a: u8, b: u8, _: u4) struct { u8, u4 } {
        const res = a ^ b;
        return .{ res, F.build(0, 0, 0, btoi(res == 0)) };
    }
};

// =========================== TESTS =============================

const testing = std.testing;

test "adc" {
    const flags = 0;
    const res, const new_flags = Alu.adc8(1, 1, flags);
    try testing.expectEqual(2, res);
    try testing.expectEqual(0, new_flags);
}

test "Get/Set Flags" {
    var flags: u4 = 0;
    const F = Alu.F;

    try testing.expectEqual(0, F.get(flags, F.C));
    flags = F.set(flags, F.C, 1);
    try testing.expectEqual(1, F.get(flags, F.C));
    flags = F.set(flags, F.C, 0);
    try testing.expectEqual(0, F.get(flags, F.C));

    try testing.expectEqual(0, F.get(flags, F.H));
    flags = F.set(flags, F.H, 1);
    try testing.expectEqual(1, F.get(flags, F.H));
    flags = F.set(flags, F.H, 0);
    try testing.expectEqual(0, F.get(flags, F.H));

    try testing.expectEqual(0, F.get(flags, F.N));
    flags = F.set(flags, F.N, 1);
    try testing.expectEqual(1, F.get(flags, F.N));
    flags = F.set(flags, F.N, 0);
    try testing.expectEqual(0, F.get(flags, F.N));

    try testing.expectEqual(0, F.get(flags, F.Z));
    flags = F.set(flags, F.Z, 1);
    try testing.expectEqual(1, F.get(flags, F.Z));
    flags = F.set(flags, F.Z, 0);
    try testing.expectEqual(0, F.get(flags, F.Z));
}
