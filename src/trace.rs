use crate::cpu::{CPU, AddressingMode};
use crate::opcodes::{OPCODES_MAP, Mnemonic};


impl CPU {
     fn get_absolute_address(&mut self, mode: &AddressingMode, addr: u16) -> u16 {
        match mode {
            AddressingMode::ZeroPage => self.mem_read(addr) as u16,

            AddressingMode::Absolute => self.mem_read_u16(addr),

            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read(addr);
                let addr = pos.wrapping_add(self.register_x) as u16;
                addr
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read(addr);
                let addr = pos.wrapping_add(self.register_y) as u16;
                addr
            }

            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(addr);
                let addr = base.wrapping_add(self.register_x as u16);
                addr
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(addr);
                let addr = base.wrapping_add(self.register_y as u16);
                addr
            }

            AddressingMode::Indirect_X => {
                let base = self.mem_read(addr);

                let ptr: u8 = (base as u8).wrapping_add(self.register_x);
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(addr);

                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                deref
            }

            _ => {
                panic!("mode {:?} is not supported", mode);
            }
        }
    }
}





pub fn trace(cpu: &mut CPU) -> String {
    let ref opscodes = OPCODES_MAP;

    let code = cpu.mem_read(cpu.program_counter);
    let ops = opscodes.get(&code).unwrap();

    let begin = cpu.program_counter;
    let mut hex_dump = vec![];
    hex_dump.push(code);

    let (mem_addr, stored_value) = match ops.mode {
        AddressingMode::Immediate 
            | AddressingMode::Implicit 
            | AddressingMode::Accumulator 
            | AddressingMode::Relative 
            | AddressingMode::Indirect => (0, 0),
        _ => {
            let addr = cpu.get_absolute_address(&ops.mode, begin + 1);
            (addr, cpu.mem_read(addr))
        }
    };


    let tmp = match ops.bytes {
        1 => {
            match ops.mode {
                AddressingMode::Implicit => format!(""),
                AddressingMode::Accumulator => format!("A "),
                _ => panic!("unexpected addressing mode {:?} has ops-len 1. code {:02x}", ops.mode, ops.code),
            }
        }
        2 => {
            let addr = cpu.mem_read(begin + 1);
            hex_dump.push(addr);

            match ops.mode {
                AddressingMode::Immediate => format!("#${:02x} ", addr),
                AddressingMode::ZeroPage => format!("${:02x} = {:02x} ", mem_addr, stored_value),
                AddressingMode::ZeroPage_X => format!(
                    "${:02x},X @ {:02x} = {:02x} ",
                    addr, mem_addr, stored_value
                ),
                AddressingMode::ZeroPage_Y => format!(
                    "${:02x},Y @ {:02x} = {:02x} ",
                    addr, mem_addr, stored_value
                ),
                AddressingMode::Relative => {
                    let addr: usize = (begin as usize + 2).wrapping_add(addr as i8 as usize);
                    format!("${:04x} ", addr)
                }
                AddressingMode::Indirect_X => format!(
                    "(${:02x},X) @ {:02x} = {:04x} = {:02x} ",
                    addr, addr.wrapping_add(cpu.register_x), mem_addr, stored_value
                ),
                AddressingMode::Indirect_Y => {
                    format!(
                        "(${:02x}),Y = {:04x} @ {:04x} = {:02x} ",
                        addr, mem_addr.wrapping_sub(cpu.register_y as u16), mem_addr, stored_value
                    )
                }
                _ => panic!("unexpected addressing mode {:?} has ops-len 2. code {:02x}", ops.mode, ops.code),
            }
        }
        3 => {
            let addr_lo = cpu.mem_read(begin + 1);
            let addr_hi = cpu.mem_read(begin + 2);
            hex_dump.push(addr_lo);
            hex_dump.push(addr_hi);

            let addr = cpu.mem_read_u16(begin + 1);

            match ops.mode {
                AddressingMode::Absolute => {
                    match ops.mnemonic {
                        Mnemonic::JMP | Mnemonic::JSR => format!("${:04x}", addr),
                        _ => format!("${:04x} = {:02x} ", addr, stored_value),
                    }
                }
                AddressingMode::Absolute_X => format!(
                    "${:04x},X @ {:04x} = {:02x} ",
                    addr, mem_addr, stored_value
                ),
                AddressingMode::Absolute_Y => format!(
                    "${:04x},Y @ {:04x} = {:02x} ",
                    addr, mem_addr, stored_value
                ),
                AddressingMode::Indirect => {
                    match ops.mnemonic {
                        Mnemonic::JMP => {
                            let jmp_addr = if addr & 0x00FF == 0x00FF {
                                let lo = cpu.mem_read(addr);
                                let hi = cpu.mem_read(addr & 0xFF00);
                                (hi as u16) << 8 | (lo as u16)
                            } else {
                                cpu.mem_read_u16(addr)
                            };
                            format!("(${:04x}) = {:04x}", addr, jmp_addr)
                        }
                        _ => format!("${:04x}", addr)
                    }
                },
                _ => panic!("unexpected addressing mode {:?} has ops-len 3. code {:02x}", ops.mode, ops.code),
            }
        }
        _ => {
            panic!("unsupported bytes: {}", ops.bytes);
        }
    };

    let hex_str = hex_dump
        .iter()
        .map(|x| format!("{:02x}", x))
        .collect::<Vec<String>>()
        .join(" ");
    let mnemonic_str = format!("{:?}", ops.mnemonic);
    let asm_str = format!("{:04x}  {:8} {: >4} {}", begin, hex_str, mnemonic_str, tmp)
        .trim()
        .to_string();

    format!(
        "{:47} A:{:02x} X:{:02x} Y:{:02x} P:{:02x} SP:{:02x}",
        asm_str,
        cpu.register_a,
        cpu.register_x,
        cpu.register_y,
        cpu.status.to_byte(),
        cpu.stack_pointer,
    ).to_ascii_uppercase()
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::bus::Bus;
    use crate::cartridge::test::test_rom;

    #[test]
    fn test_format_trace() {
        let mut bus = Bus::new(test_rom());
        bus.mem_write(100, 0xa2);
        bus.mem_write(101, 0x01);
        bus.mem_write(102, 0xca);
        bus.mem_write(103, 0x88);
        bus.mem_write(104, 0x00);

        let mut cpu = CPU::new(bus);
        cpu.program_counter = 0x64;
        cpu.register_a = 1;
        cpu.register_x = 2;
        cpu.register_y = 3;
        let mut result: Vec<String> = vec![];
        cpu.run_with_callback(|cpu| {
            result.push(trace(cpu));
        });
        assert_eq!(
            "0064  A2 01     LDX #$01                        A:01 X:02 Y:03 P:24 SP:FD",
            result[0]
        );
        assert_eq!(
            "0066  CA        DEX                             A:01 X:01 Y:03 P:24 SP:FD",
            result[1]
        );
        assert_eq!(
            "0067  88        DEY                             A:01 X:00 Y:03 P:26 SP:FD",
            result[2]
        );
    }

    #[test]
    fn test_format_mem_access() {
        let mut bus = Bus::new(test_rom());
        // ORA ($33), Y
        bus.mem_write(100, 0x11);
        bus.mem_write(101, 0x33);

        //data
        bus.mem_write(0x33, 00);
        bus.mem_write(0x34, 04);

        //target cell
        bus.mem_write(0x400, 0xAA);

        let mut cpu = CPU::new(bus);
        cpu.program_counter = 0x64;
        cpu.register_y = 0;
        let mut result: Vec<String> = vec![];
        cpu.run_with_callback(|cpu| {
            result.push(trace(cpu));
        });
        assert_eq!(
            "0064  11 33     ORA ($33),Y = 0400 @ 0400 = AA  A:00 X:00 Y:00 P:24 SP:FD",
            result[0]
        );
    }
}
