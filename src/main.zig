const std = @import("std");

const MEMORY_CAPACITY = 1024 * 512;

const Memory = struct {
    data: [MEMORY_CAPACITY]u8,

    pub fn new() Memory {
        return Memory{
            .data = [_]u8{0} ** MEMORY_CAPACITY,
        };
    }

    fn translate_address(address: u64) u64 {
        const begin = 0x80000000;
        std.debug.assert(address >= begin);
        return address - begin;
    }

    pub fn read_byte(self: *Memory, address: u64) u8 {
        return self.data[translate_address(address)];
    }

    pub fn read_bytes(self: *Memory, address: u64, width: u64) u64 {
        var data: u64 = 0;
        var i: usize = 0;
        while (i < width) {
            data |= @shlExact(@intCast(u64, self.read_byte(address + i)), @intCast(u6, (i * 8)));
            i += 1;
        }
        return data;
    }

    pub fn write_byte(self: *Memory, address: u64, value: u8) void {
        self.data[translate_address(address)] = value;
    }

    pub fn write_bytes(self: *Memory, address: u64, width: u64, value: u64) void {
        var i: usize = 0;
        while (i < width) {
            self.data[translate_address(address) + i] = @truncate(u8, value >> @intCast(u6, (i * 8)));
            i += 1;
        }
    }
};

const Opcode = enum(u7) {
    LUI = 0b0110111,
    AUIPC = 0b0010111,
    JAL = 0b1101111,
    JALR = 0b1100111,
    BRANCH = 0b1100011,
    LOAD = 0b0000011,
    OP_IMM = 0b0010011,
    OP_IMM_W = 0b0011011,
    OP = 0b0110011,
    MISC_MEM = 0b0001111,
    SYSTEM = 0b1110011,
};

const BranchFunct3 = enum(u3) {
    BEQ = 0b000,
    BNE = 0b001,
    BLT = 0b100,
    BGE = 0b101,
    BLTU = 0b110,
    BGEU = 0b111,
};

const LoadFunct3 = enum(u3) {
    LB = 0b000,
    LH = 0b001,
    LW = 0b010,
    LBU = 0b100,
    LHU = 0b101,
};

const StoreFunct3 = enum(u3) {
    SB = 0b000,
    SH = 0b001,
    SQ = 0b010,
};

const OpImmFunct3 = enum(u3) {
    ADDI = 0b000,
    SLTI = 0b010, // set on less than
    SLTIU = 0b011, // set on less than unsigned
    XORI = 0b100,
    ORI = 0b110,
    ANDI = 0b111,
    SLLI = 0b001, // shift left
    SRLI_SRAI = 0b101, // shift right
};

const OpImmWFunct3 = enum(u3) {
    ADDIW = 0b000,
    SLLIW = 0b001,
    SRLIW_SRAIW = 0b101,
};

const OpFunct3 = enum(u3) {
    ADD_SUB = 0b000,
    SLL = 0b001, // shift left
    SLT = 0b010, // set on less than
    SLTU = 0b011, // set on less than unsigned
    XOR = 0b100,
    OR = 0b110,
    AND = 0b111,
    SRL_SRA = 0b101, // shift right
};

const SystemFunct3 = enum(u3) {
    ECALL_EBREAK = 0b000,
    CSRRW = 0b001,
    CSRRS = 0b010,
    CSRRC = 0b011,
    CSRRWI = 0b101,
    CSRRSI = 0b110,
    CSRRCI = 0b111,
};

const Cpu = struct {
    clock: u64,
    regs: [32]u64,
    pc: u64,
    mem: Memory,

    pub fn new() Cpu {
        var cpu = Cpu{
            .clock = 0,
            .regs = .{0} ** 32,
            .pc = 0,
            .mem = Memory.new(),
        };
        cpu.regs[2] = MEMORY_CAPACITY;

        return cpu;
    }

    fn write_register(self: *Cpu, idx: usize, value: u32) void {
        if (idx != 0) {
            self.regs[idx] = value;
        }
    }

    pub fn tick(self: *Cpu) bool {
        var inst = self.fetch();

        std.debug.print("------------\n", .{});
        std.debug.print("{x}\n", .{self.pc});
        std.debug.print("{x}, {b}\n", .{ inst, inst });
        std.debug.print("{x}\n", .{self.regs});

        if (inst == 0) {
            return false;
        }

        self.clock += 1;
        self.execute(inst);
        self.pc += 4;

        return true;
    }

    fn fetch(self: *Cpu) u32 {
        return @truncate(u32, self.mem.read_bytes(self.pc, 4));
    }

    fn execute(self: *Cpu, inst: u32) void {
        var opcode = @intToEnum(Opcode, @truncate(u7, inst & 0x7F));
        std.debug.print("{}\n", .{opcode});

        switch (opcode) {
            Opcode.JAL => {
                var rd = @truncate(u5, (inst >> 7) & 0b11111);
                // https://stackoverflow.com/questions/58414772/why-are-risc-v-s-b-and-u-j-instruction-types-encoded-in-this-way
                var imm = ((inst & 0x80000000) >> 11) | (inst & 0xFF000) | ((inst >> 9) & 0x800) | ((inst >> 20) & 0x7FE);
                // sub 4 to account for pc increment on next tick
                self.pc += imm - 4;
            },
            Opcode.BRANCH => {
                var funct3 = @intToEnum(BranchFunct3, @truncate(u3, (inst >> 12) & 0x7));
                std.debug.print("funct3: {b}\n", .{funct3});

                var rs1 = @intCast(usize, (inst >> 15) & 0x1F);
                var rs2 = @intCast(usize, (inst >> 20) & 0x1F);
                var imm = ((inst << 4) & 0x800) |
                    ((inst >> 20) & 0x7E0) |
                    ((inst >> 7) & 0x1E);
                var immx = switch (inst & 0x80000000) {
                    0x80000000 => imm | 0xFFFFF000,
                    else => imm,
                };

                std.debug.print("immx: {x}\n", .{immx});

                switch (funct3) {
                    BranchFunct3.BNE => {
                        if (self.regs[rs1] != self.regs[rs2]) {
                            self.pc = @truncate(u32, self.pc +% immx - 4);
                        }
                    },
                    BranchFunct3.BEQ => {
                        if (self.regs[rs1] == self.regs[rs2]) {
                            self.pc += imm - 4;
                        }
                    },
                    BranchFunct3.BGE => {
                        if (self.regs[rs1] >= self.regs[rs2]) {
                            self.pc += imm - 4;
                        }
                    },
                    else => {
                        unreachable;
                    },
                }
            },
            Opcode.OP => {
                var funct3 = @intToEnum(OpFunct3, @truncate(u3, (inst >> 12) & 0x7));
                std.debug.print("funct3: {b}\n", .{funct3});

                var rd = @intCast(usize, (inst >> 7) & 0x1F);
                var rs1 = @intCast(usize, (inst >> 15) & 0x1F);
                var rs2 = @intCast(usize, (inst >> 20) & 0x1F);

                var funct7 = (inst >> 25) & 0x7F;

                std.debug.print("rs1: {x}\n", .{rs1});
                std.debug.print("rs2: {x}\n", .{rs2});

                switch (funct3) {
                    OpFunct3.ADD_SUB => {
                        switch (funct7) {
                            // add
                            0x00 => {
                                self.write_register(rd, @truncate(u32, self.regs[rs1] +% self.regs[rs2]));
                            },
                            // sub
                            0x20 => {
                                unreachable;
                            },
                            else => {
                                unreachable;
                            },
                        }
                    },
                    else => {
                        unreachable;
                    },
                }
            },
            Opcode.OP_IMM => {
                var funct3 = @intToEnum(OpImmFunct3, @truncate(u3, (inst >> 12) & 0x7));
                std.debug.print("funct3: {b}\n", .{funct3});

                var rd = @intCast(usize, (inst >> 7) & 0x1F);
                var rs1 = @intCast(usize, (inst >> 15) & 0x1F);
                var imm = (inst & 0xFFF00000) >> 20;
                var immx = switch (imm & 0x800) {
                    0x800 => imm | 0xFFFFF000,
                    else => imm,
                };
                var shamt = @truncate(u6, imm & 0x3F);

                std.debug.print("rd: {b}\n", .{rd});
                std.debug.print("rs1: {b}\n", .{rs1});
                std.debug.print("imm: {b}\n", .{imm});
                std.debug.print("immx: {b}\n", .{immx});

                switch (funct3) {
                    OpImmFunct3.ADDI => {
                        self.write_register(rd, @truncate(u32, self.regs[rs1] +% immx));
                    },
                    OpImmFunct3.SLLI => {
                        std.debug.print("shamt: {b}\n", .{shamt});
                        self.write_register(rd, @truncate(u32, self.regs[rs1] << shamt));
                    },
                    OpImmFunct3.ORI => {
                        //
                    },
                    else => {
                        unreachable;
                    },
                }
            },
            Opcode.OP_IMM_W => {
                var funct3 = @intToEnum(OpImmWFunct3, @truncate(u3, (inst >> 12) & 0x7));
                std.debug.print("funct3: {b}\n", .{funct3});

                var rd = @intCast(usize, (inst >> 7) & 0x1F);
                var rs1 = @intCast(usize, (inst >> 15) & 0x1F);
                var imm = inst >> 20;
                var immx = switch (imm & 0x800) {
                    0x800 => imm | 0xFFFFFFFF,
                    else => imm,
                };

                std.debug.print("rd: {x}\n", .{rd});
                std.debug.print("rs1: {x}\n", .{rs1});
                std.debug.print("imm: {x}\n", .{imm});
                std.debug.print("immx: {x}\n", .{immx});

                switch (funct3) {
                    OpImmWFunct3.ADDIW => {
                        self.write_register(rd, @truncate(u32, self.regs[rs1] +% immx));
                    },
                    else => {
                        unreachable;
                    },
                }
            },
            Opcode.AUIPC => {
                var rd = @intCast(usize, (inst >> 7) & 0x1F);
                var imm = (inst & 0xFFFFF000) >> 12;
                self.write_register(rd, @truncate(u32, self.pc + imm - 4));
            },
            Opcode.LUI => {
                var rd = @intCast(usize, (inst >> 7) & 0x1F);
                var imm = (inst & 0xFFFFF000);
                std.debug.print("rd: {x}\n", .{rd});
                std.debug.print("imm: {x}\n", .{imm});
                self.write_register(rd, imm);
            },
            Opcode.MISC_MEM => {
                // fence call
            },
            Opcode.SYSTEM => {
                var funct3 = @intToEnum(SystemFunct3, @truncate(u3, (inst >> 12) & 0x7));
                std.debug.print("funct3: {b}\n", .{funct3});

                switch (funct3) {
                    SystemFunct3.ECALL_EBREAK => {
                        if (self.regs[3] == 1) {
                            std.debug.print("test passes\n", .{});
                            return;
                        }

                        if (self.regs[3] >= 1) {
                            unreachable;
                        }
                    },
                    else => {
                        // ignoring for now
                    },
                }
            },
            else => {
                unreachable;
            },
        }
    }
};

pub fn main() anyerror!void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();

    var cpu = Cpu.new();

    var elf_file = try std.fs.cwd().openFile("./riscv-tests/isa/rv64ui-p-add", .{});
    defer elf_file.close();

    const elf_hdr = try std.elf.Header.read(&elf_file);
    var program_headers = elf_hdr.program_header_iterator(&elf_file);
    while (try program_headers.next()) |phdr| {
        if (phdr.p_flags & 1 == 1) {
            const MAX_PHDR_FILESZ = 4096;
            var buf: [MAX_PHDR_FILESZ]u8 = undefined;
            const to_read = std.math.min(phdr.p_filesz, MAX_PHDR_FILESZ);
            const did_read = try elf_file.preadAll(buf[0..to_read], phdr.p_offset);

            cpu.pc = phdr.p_paddr;
            var i: usize = 0;
            while (i < to_read) {
                cpu.mem.write_byte(cpu.pc + i, buf[i]);
                i += 1;
            }
        }
    }

    while (true) {
        var has_next_instruction = cpu.tick();
        if (!has_next_instruction) {
            break;
        }
    }
}
