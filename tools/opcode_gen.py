import json
import argparse
import pathlib
from dataclasses import dataclass

OUTPUT_HEADER = """pub const Register = enum { A, F, B, C, D, E, H, L, AF, BC, DE, HL, SP, };
pub const OperandType = enum { unused, reg8, reg16, imm8, imm16, bit3, vec, };
pub const Operand = struct { t: OperandType, register: Register = Register.A, value: u16 = 0, relative: bool = false, };
pub const Instruction = struct { opcode: u8 = 0, op: OpType, operands: [3]Operand, num_operands: u8 , bytes: u8};
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
        return self.name in ("A", "B", "C", "D", "E", "H", "L")

    def is_r16(self):
        return self.name in ("AF", "BC", "DE", "HL", "SP")

    def is_condition_code(self):
        return self.name in ("NC", "Z", "NZ")

    def to_zig(self):
        assert(not self.is_condition_code())

        value = None
        op_type = "blubb"
        rn = ""
        if self.is_r8():
            op_type = "reg8"
            rn = f".register = Register.{self.name}"
        elif self.is_r16():
            op_type = "reg16"
            rn = f".register = Register.{self.name}"
        elif self.name.startswith("$"):
            op_type = "vec"
        elif self.is_u3():
            op_type = "bit3"
            value = int(self.name)
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

    def pretty_str(self):
        return f"{self.mnemonic} {', '.join([o.pretty_str() for o in self.operands])}".strip()

    def to_zig(self):
        filler_operand = Operand(name="unused", immediate=True).to_zig()
        operands = [o.to_zig() for o in self.operands if not o.is_condition_code()]

        num_operands = len(operands)

        # fill to 3 because we declare a statically sized array of operands
        for _ in range(3 - len(operands)):
            operands.append(filler_operand)
        assert(len(operands) == 3)

        op = self.mnemonic.upper()
        return f"Instruction{{ .opcode = 0x{self.value:02x}, .op = OpType.{op}, .operands = .{{ {', '.join(operands)} }}, .num_operands = {num_operands}, .bytes = {self.bytes} }}"

def generate_opcodes(instructions: list[Instruction]):
    mnemonics = sorted(set(i.mnemonic.upper() for i in instructions))
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

    op_types = f"pub {generate_opcodes(instructions)}"

    with open(args.output, "wt") as f:
        f.write(op_types + '\n')
        f.write(OUTPUT_HEADER)
        f.write(generate_decoder(instructions, "decode_instruction") + '\n')
        f.write(generate_decoder(prefixed_instructions, "decode_instruction_cb") + '\n')
        f.write(generate_opcode_to_str(instructions, "opcode_to_str"))
        f.write(generate_opcode_to_str(prefixed_instructions, "opcode_to_str_cb"))


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
