import json
import argparse
import pathlib
from dataclasses import dataclass

OUTPUT_HEADER = """pub const Register = enum { A, F, B, C, D, E, H, L, AF, BC, DE, HL, SP, };
pub const ConditionCode = enum (u2) { nc = 0, z = 1, nz = 2, c = 3 };
pub const OperandType = enum { unused, reg8, reg16, imm8, imm16, bit3, vec, cc, };
pub const Operand = struct { t: OperandType, register: Register = Register.A, value: u16 = 0, relative: bool = false, cc: ConditionCode = ConditionCode.nc, };
pub const Instruction = struct { opcode: u8 = 0, op: OpType, operands: [3]Operand, num_operands: u2 , bytes: u2, };
"""

@dataclass
class Operand:
    name: str
    immediate: bool
    increment: bool = False
    decrement: bool = False
    bytes: int = 0

    def pretty_str(self):
        inc_dec_sym = "+" if self.increment else "-" if self.decrement else ""
        ob = "[" if not self.immediate else ""
        cb = "]" if not self.immediate else ""
        return f"{ob}{self.name}{inc_dec_sym}{cb}"

    def is_register(self):
        return self.is_r8() or self.is_r16()

    def is_u3(self):
        return self.name in (str(x) for x in range(8))

    def is_r8(self):
        # !! C can both be a condition code and a register !!
        return self.name in ("A", "B", "C", "D", "E", "H", "L")

    def is_r16(self):
        return self.name in ("AF", "BC", "DE", "HL", "SP")

    def is_condition_code(self):
        # !! C can both be a condition code and a register !!
        return self.name in ("NC", "Z", "NZ", "C")

    def dispatch_type_old(self, treat_c_as_cc = False):
        if self.is_r8() and (self.name != "C" or not treat_c_as_cc):
            op_type = "reg8"
        elif self.is_r16():
            op_type = "reg16"
        elif self.name.startswith("$"):
            op_type = "vec"
        elif self.is_u3():
            op_type = "bit3"
        elif self.is_condition_code():
            op_type = "cc"
        elif "8" in self.name:
            op_type = "imm8"
        elif "16" in self.name:
            op_type = "imm16"
        else:
            op_type = "invalid"

        if not self.immediate:
            op_type += "r"

        return op_type

    def dispatch_type(self, treat_c_as_cc = False):

        if not self.immediate:
            op_type = "u8" # relative (aka memory load) always load/store 1 byte
        elif self.is_r8() and (self.name != "C" or not treat_c_as_cc):
            op_type = "u8"
        elif self.is_r16():
            op_type = "u16"
        elif self.name.startswith("$"):
            op_type = "vec"
        elif self.is_u3():
            op_type = "u3"
        elif self.is_condition_code():
            op_type = "cc"
        elif "8" in self.name:
            op_type = "u8"
        elif "16" in self.name:
            op_type = "u16"
        else:
            op_type = "invalid"



        return op_type

    def to_zig(self, treat_c_as_cc = False):

        value = None
        op_type = "blubb"
        rn = ""
        cc = ""
        if self.is_r8() and (self.name != "C" or not treat_c_as_cc):
            op_type = "reg8"
            rn = f".register = Register.{self.name}"
        elif self.is_r16():
            op_type = "reg16"
            rn = f".register = Register.{self.name}"
        elif self.name.startswith("$"):
            op_type = "vec"
            value = f"{hex(int(self.name[1:], 16))}"
        elif self.is_u3():
            op_type = "bit3"
            value = int(self.name)
        elif self.is_condition_code():
            op_type = "cc"
            cc = f".cc = ConditionCode.{self.name.lower()}"
        elif "8" in self.name:
            op_type = "imm8"
        elif "16" in self.name:
            op_type = "imm16"
        elif self.name == "unused":
            op_type = "unused"

        if op_type == "blubb":
            print(self)
        assert(op_type != "blubb")

        t = f".t=OperandType.{op_type}"
        rel = f".relative = { 'false' if self.immediate else 'true' }"
        # inc_dec_mode = f".idm = {'+1' if self.increment else '-1' if self.decrement else '0'}"

        args = [t, rel]
        if value is not None:
            args.append(f".value = {value}")

        if rn:
            args.append(rn)

        if cc:
            args.append(cc)

        return f"Operand{{ {', '.join(args)} }}"

@dataclass
class Instruction:
    value: int
    mnemonic: str
    bytes: int
    cycles: list[int]
    operands: list[Operand]

    def fix_mnemonic(self):
        if self.mnemonic.startswith("ILLEGAL"):
            self.mnemonic = "ILLEGAL"
        elif self.mnemonic in ["RLA", "RLCA", "RRA", "RRCA", "DAA"]:
            # These have A as an implicit r8 operand. Add it here so we don't
            # have to handle that at runtime all the time.
            self.operands.append(Operand(name="A", immediate=True))

    def pretty_str(self):
        return f"{self.mnemonic} {', '.join([o.pretty_str() for o in self.operands])}".strip()

    def dispatch_type(self):
        treat_c_as_cc_ops = [0x38, 0xdc, 0xda, 0xd8];
        ops = '_'.join([o.dispatch_type(treat_c_as_cc=self.value in treat_c_as_cc_ops) for o in self.operands])
        return f"{self.mnemonic.lower()}{'_' if ops else ''}{ops}"

    def to_zig(self):
        filler_operand = Operand(name="unused", immediate=True).to_zig()
        treat_c_as_cc_ops = [0x38, 0xdc, 0xda, 0xd8];
        operands = [o.to_zig(treat_c_as_cc=self.value in treat_c_as_cc_ops) for o in self.operands]

        num_operands = len(operands)

        # fill to 3 because we declare a statically sized array of operands
        for _ in range(3 - len(operands)):
            operands.append(filler_operand)
        assert(len(operands) == 3)

        op = self.mnemonic.upper()
        return f"Instruction{{ .opcode = 0x{self.value:02x}, .op = OpType.{op}, .operands = .{{ {', '.join(operands)} }}, .num_operands = {num_operands}, .bytes = {self.bytes} }}"

def generate_opcodes(instr: list[Instruction], instr_cb: list[Instruction]):
    all = instr + instr_cb
    mnemonics = sorted(set(i.mnemonic.upper() for i in all))
    return f"const OpType = enum {{ {', '.join(mnemonics)} }};"

def generate_decoder(instructions: list[Instruction], fname: str):
    result = []
    result += [f"pub fn {fname}(opcode: u8) Instruction {{"]
    result += ["    const result = switch (opcode) {"]

    for i in instructions:
        # print(i.to_zig())
        result += [f"        0x{i.value:02x} => {i.to_zig()},"]

    # result += ["        else => unreachable,"]
    result += ["    };"]
    result += ["    return result;"]
    result += ["}"]

    return '\n'.join(result)

def generate_opcode_to_str(instructions: list[Instruction], fname: str):
    result = []
    result += [f"pub fn {fname}(opcode: u8) []const u8 {{"]
    result += ["    switch (opcode) {"]

    for i in instructions:
        result += [f"        0x{i.value:02x} => return \"{i.pretty_str()}\","]

    result += ["    }"]
    result += ["}"]

    return '\n'.join(result)

def generate_opcode_to_dispatch(instructions: list[Instruction], fname: str):
    result = []
    result += [f"pub fn {fname}(opcode: u8) DispatchType {{"]
    result += ["    const Dt = DispatchType;"]
    result += ["    switch(opcode) {"]
    for i in instructions:
        result += [f"        0x{i.value:02x} => return Dt.{i.dispatch_type()},"]
    result += ["    }"]
    result += ["}"]

    return '\n'.join(result)

def generate_dispatch_types(instr: list[Instruction], instr_cb: list[Instruction]):
    ts = [i.dispatch_type() for i in instr]
    ts += [i.dispatch_type() for i in instr_cb]
    as_str = ', '.join(sorted(set(ts)))
    return f"pub const DispatchType = enum {{ {as_str} }};"

def parse_instructions(definitions: dict):
    instructions: list[Instruction] = []
    for value, op in definitions.items():
        operands = []
        for operand in op["operands"]:
            operands.append(Operand(**operand))
        instructions.append(Instruction(
            value=int(value, 16),
            mnemonic=op["mnemonic"],
            bytes=op["bytes"],
            cycles=op["cycles"],
            operands=operands
        ))

    for i in instructions:
        i.fix_mnemonic()

    return instructions

def generate(definitions: dict, args: argparse.Namespace):
    instructions: list[Instruction] = parse_instructions(definitions["unprefixed"])
    prefixed_instructions: list[Instruction] = parse_instructions(definitions["cbprefixed"])

    op_types = f"pub {generate_opcodes(instructions, prefixed_instructions)}"
    dp_types = generate_dispatch_types(instructions, prefixed_instructions)

    with open(args.output, "wt") as f:
        f.write(op_types + '\n')
        f.write(dp_types + '\n')
        f.write(OUTPUT_HEADER)
        f.write(generate_decoder(instructions, "decode_instruction") + '\n')
        f.write(generate_decoder(prefixed_instructions, "decode_instruction_cb") + '\n')
        f.write(generate_opcode_to_str(instructions, "opcode_to_str") + '\n')
        f.write(generate_opcode_to_str(prefixed_instructions, "opcode_to_str_cb") + '\n')
        f.write(generate_opcode_to_dispatch(instructions, "opcode_to_dp") + '\n')
        f.write(generate_opcode_to_dispatch(prefixed_instructions, "opcode_to_dp_cb") + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=pathlib.Path, default="tools/Opcodes.json")
    parser.add_argument("-o", "--output", type=pathlib.Path, default="src/instructions_generated.zig")
    args = parser.parse_args()

    with open(args.input, "rt") as f:
        input = json.load(f)

    generate(input, args)

if __name__ == "__main__":
    main()
