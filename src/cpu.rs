use crate::opcodes::{OPCODES_MAP, Mnemonic, OpCode};
use crate::bus::Bus;


const STACK: u16 = 0x0100;
const STACK_RESET: u8 = 0xfd;




pub struct StatusRegister {
    carry: bool,
    zero: bool,
    interrupt_disable: bool,
    decimal_mode: bool,
    break_command: bool,
    _reserved: bool,
    overflow: bool,
    negative: bool,
}

impl StatusRegister {
    pub fn from_byte(byte: u8) -> Self {
        Self {
            carry: byte & 1 == 1,
            zero: (byte >> 1) & 1 == 1,
            interrupt_disable: (byte >> 2) & 1 == 1,
            decimal_mode: (byte >> 3) & 1 == 1,
            break_command: (byte >> 4) & 1 == 1,
            _reserved: (byte >> 5) & 1 == 1,
            overflow: (byte >> 6) & 1 == 1,
            negative: (byte >> 7) & 1 == 1,
        }
    }

    pub fn to_byte(&self) -> u8 {
        let mut status = 0;
        status |= self.carry as u8;
        status |= (self.zero as u8) << 1;
        status |= (self.interrupt_disable as u8) << 2;
        status |= (self.decimal_mode as u8) << 3;
        status |= (self.break_command as u8) << 4;
        status |= 1 << 5;
        status |= (self.overflow as u8) << 6;
        status |= (self.negative as u8) << 7;
        status
    }
}


pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub stack_pointer: u8,
    pub status: StatusRegister,
    pub program_counter: u16,
    pub bus: Bus,
}

impl CPU {
    pub fn new(bus: Bus) -> Self {
        Self {
            register_a: 0,
            register_x: 0,
            register_y: 0,
            stack_pointer: STACK_RESET,
            status: StatusRegister::from_byte(0b0010_0100),
            program_counter: 0,
            bus,
        }
    }
    // memory access
    pub fn mem_read(&self, addr: u16) -> u8 {
        self.bus.mem_read(addr)
    }

    pub fn mem_read_u16(&mut self, addr: u16) -> u16 {
        let lo = self.mem_read(addr) as u16;
        let hi = self.mem_read(addr.wrapping_add(1)) as u16;
        (hi << 8) | lo
    }

    pub fn mem_write(&mut self, addr: u16, data: u8) {
        self.bus.mem_write(addr, data);
    }

    pub fn mem_write_u16(&mut self, addr: u16, data: u16) {
        let lo = (data & 0xff) as u8;
        let hi = (data >> 8) as u8;
        self.mem_write(addr, lo);
        self.mem_write(addr.wrapping_add(1), hi);
    }



    // stack operations
    fn stack_push(&mut self, data: u8) {
        self.mem_write(STACK + self.stack_pointer as u16, data);
        self.stack_pointer = self.stack_pointer.wrapping_sub(1);
    }

    fn stack_push_u16(&mut self, data: u16) {
        let lo = (data & 0xff) as u8;
        let hi = (data >> 8) as u8;
        self.stack_push(hi);
        self.stack_push(lo);
    }

    fn stack_pop(&mut self) -> u8 {
        self.stack_pointer = self.stack_pointer.wrapping_add(1);
        self.mem_read(STACK + self.stack_pointer as u16)
    }

    fn stack_pop_u16(&mut self) -> u16 {
        let lo = self.stack_pop() as u16;
        let hi = self.stack_pop() as u16;
        (hi << 8) | lo
    }


    // main functions
    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.program_counter = 0x0600;
        self.run();
    }

    pub fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.stack_pointer = STACK_RESET;
        self.status = StatusRegister::from_byte(0b0010_0100);

        self.program_counter = self.mem_read_u16(0xFFFC);
    }

    pub fn load(&mut self, program: Vec<u8>) {
        for i in 0..program.len() {
            self.mem_write(0x0600 + i as u16, program[i]);
        }
        // self.mem_write_u16(0xFFFC, 0x8000);
    }

    pub fn run(&mut self) {
        self.run_with_callback(|_| {});
    }

    pub fn run_with_callback<F>(&mut self, mut callback: F)
    where
        F: FnMut(&mut Self),
    {
        let ref map = OPCODES_MAP;

        while let Some(opcode) = map.get(&self.mem_read(self.program_counter)) {
            callback(self);

            self.program_counter += 1;

            let pc = self.program_counter;

            let OpCode { mnemonic, mode, .. } = opcode;

            match mnemonic {
                Mnemonic::ADC => self.adc(mode),
                Mnemonic::AHX => self.ahx(mode),
                Mnemonic::ALR => self.alr(mode),
                Mnemonic::ANC => self.anc(mode),
                Mnemonic::AND => self.and(mode),
                Mnemonic::ARR => self.arr(mode),
                Mnemonic::ASL => self.asl(mode),
                Mnemonic::AXS => self.axs(mode),
                Mnemonic::BCC => self.bcc(mode),
                Mnemonic::BCS => self.bcs(mode),
                Mnemonic::BEQ => self.beq(mode),
                Mnemonic::BIT => self.bit(mode),
                Mnemonic::BMI => self.bmi(mode),
                Mnemonic::BNE => self.bne(mode),
                Mnemonic::BPL => self.bpl(mode),
                Mnemonic::BRK => self.brk(mode),
                Mnemonic::BVC => self.bvc(mode),
                Mnemonic::BVS => self.bvs(mode),
                Mnemonic::CLC => self.clc(mode),
                Mnemonic::CLD => self.cld(mode),
                Mnemonic::CLI => self.cli(mode),
                Mnemonic::CLV => self.clv(mode),
                Mnemonic::CMP => self.cmp(mode),
                Mnemonic::CPX => self.cpx(mode),
                Mnemonic::CPY => self.cpy(mode),
                Mnemonic::DCP => self.dcp(mode),
                Mnemonic::DEC => self.dec(mode),
                Mnemonic::DEX => self.dex(mode),
                Mnemonic::DEY => self.dey(mode),
                Mnemonic::EOR => self.eor(mode),
                Mnemonic::INC => self.inc(mode),
                Mnemonic::INX => self.inx(mode),
                Mnemonic::INY => self.iny(mode),
                Mnemonic::ISC => self.isc(mode),
                Mnemonic::JMP => self.jmp(mode),
                Mnemonic::JSR => self.jsr(mode),
                Mnemonic::LAS => self.las(mode),
                Mnemonic::LAX => self.lax(mode),
                Mnemonic::LDA => self.lda(mode),
                Mnemonic::LDX => self.ldx(mode),
                Mnemonic::LDY => self.ldy(mode),
                Mnemonic::LSR => self.lsr(mode),
                Mnemonic::NOP => self.nop(mode),
                Mnemonic::ORA => self.ora(mode),
                Mnemonic::PHA => self.pha(mode),
                Mnemonic::PHP => self.php(mode),
                Mnemonic::PLA => self.pla(mode),
                Mnemonic::PLP => self.plp(mode),
                Mnemonic::RLA => self.rla(mode),
                Mnemonic::ROL => self.rol(mode),
                Mnemonic::ROR => self.ror(mode),
                Mnemonic::RRA => self.rra(mode),
                Mnemonic::RTI => self.rti(mode),
                Mnemonic::RTS => self.rts(mode),
                Mnemonic::SAX => self.sax(mode),
                Mnemonic::SBC => self.sbc(mode),
                Mnemonic::SEC => self.sec(mode),
                Mnemonic::SED => self.sed(mode),
                Mnemonic::SEI => self.sei(mode),
                Mnemonic::SHX => self.shx(mode),
                Mnemonic::SHY => self.shy(mode),
                Mnemonic::SLO => self.slo(mode),
                Mnemonic::SRE => self.sre(mode),
                Mnemonic::STA => self.sta(mode),
                Mnemonic::STP => self.stp(mode),
                Mnemonic::STX => self.stx(mode),
                Mnemonic::STY => self.sty(mode),
                Mnemonic::TAS => self.tas(mode),
                Mnemonic::TAX => self.tax(mode),
                Mnemonic::TAY => self.tay(mode),
                Mnemonic::TSX => self.tsx(mode),
                Mnemonic::TXA => self.txa(mode),
                Mnemonic::TXS => self.txs(mode),
                Mnemonic::TYA => self.tya(mode),
                Mnemonic::XAA => self.xaa(mode),
            }
            if self.program_counter == pc {
                self.program_counter += opcode.bytes as u16 - 1;
            }
        }

    }
    



    // Instructions
    fn adc(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        let carry = if self.status.carry { 1 } else { 0 };
        let a_sign = self.register_a >> 7;
        let value_sign = value >> 7;
        let (result, overflow1) = self.register_a.overflowing_add(value);
        let (result, overflow2) = result.overflowing_add(carry);
        let result_sign = result >> 7;
        self.register_a = result;
        self.status.carry = overflow1 || overflow2;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
        self.status.overflow = a_sign == value_sign && a_sign != result_sign;
    }

    fn ahx(&mut self, mode: &AddressingMode) {
        match mode {
            AddressingMode::Indirect_Y => {
                let pos = self.mem_read(self.program_counter);
                // self.program_counter += 1;
                let mem_address = self.mem_read_u16(pos as u16) + self.register_y as u16;
                let data = self.register_a & self.register_x & (mem_address >> 8) as u8;
                self.mem_write(mem_address, data);
            }
            AddressingMode::Absolute_Y => {
                let mem_address = self.mem_read_u16(self.program_counter) + self.register_y as u16;
                // self.program_counter += 2;
                let data = self.register_a & self.register_x & (mem_address >> 8) as u8;
                self.mem_write(mem_address, data);
            }
            _ => panic!("Invalid addressing mode for AHX instruction"),
        }
    }

    fn alr(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        self.register_a &= value;
        self.status.carry = self.register_a & 1 == 1;
        self.register_a >>= 1;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn anc(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        self.register_a &= value;
        self.status.carry = self.register_a >> 7 == 1;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn and(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        self.register_a &= value;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn arr(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        self.register_a &= value;
        self.register_a >>= 1;
        let bit_5 = (self.register_a >> 5) & 1;
        let bit_6 = (self.register_a >> 6) & 1;
        self.status.carry = bit_6 == 1;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
        self.status.overflow = bit_5 ^ bit_6 == 1;
    }

    fn asl(&mut self, mode: &AddressingMode) {
        if let AddressingMode::Accumulator = mode {
            let carry = if self.register_a >> 7 == 1 { 1 } else { 0 };
            self.status.carry = carry == 1;
            self.register_a <<= 1;
            self.status.zero = self.register_a == 0;
            self.status.negative = self.register_a >> 7 == 1;
        } else {
            let addr = self.get_operand_address(mode);
            let value = self.mem_read(addr);
            let carry = if value >> 7 == 1 { 1 } else { 0 };
            self.status.carry = carry == 1;
            let result = value << 1;
            self.mem_write(addr, result);
            self.status.zero = result == 0;
            self.status.negative = result >> 7 == 1;
        }
    }

    fn axs(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        let x_and_a = self.register_x & self.register_a;
        let result = x_and_a.wrapping_sub(value);
        self.status.carry = x_and_a >= value;
        self.register_x = result as u8;
        self.status.zero = self.register_x == 0;
        self.status.negative = self.register_x >> 7 == 1;
    }

    fn bcc(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if !self.status.carry {
            self.program_counter = self.program_counter.wrapping_add(1 + offset as u16);
        }
    }

    fn bcs(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if self.status.carry {
            self.program_counter = self.program_counter.wrapping_add(1 + offset as u16);
        }
    }

    fn beq(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if self.status.zero {
            self.program_counter = self.program_counter.wrapping_add(1 + offset as u16);
        }
    }

    fn bit(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        let result = self.register_a & value;
        self.status.zero = result == 0;
        self.status.overflow = (value >> 6) & 1 == 1;
        self.status.negative = value >> 7 == 1;
    }

    fn bmi(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if self.status.negative {
            self.program_counter = self.program_counter.wrapping_add(1 + offset as u16);
        }
    }

    fn bne(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if !self.status.zero {
            self.program_counter = self.program_counter.wrapping_add(1 + offset as u16);
        }
    }

    fn bpl(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if !self.status.negative {
            self.program_counter = self.program_counter.wrapping_add(1 + offset as u16);
        }
    }

    fn brk(&mut self, _mode: &AddressingMode) {
        self.stack_push_u16(self.program_counter);
        self.stack_push(self.status.to_byte());
        self.status.break_command = true;
        self.program_counter = self.mem_read_u16(0xFFFE);
    }

    fn bvc(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if !self.status.overflow {
            self.program_counter = self.program_counter.wrapping_add(1 + offset as u16);
        }
    }

    fn bvs(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if self.status.overflow {
            self.program_counter = self.program_counter.wrapping_add(1 + offset as u16);
        }
    }

    fn clc(&mut self, _mode: &AddressingMode) {
        self.status.carry = false;
    }

    fn cld(&mut self, _mode: &AddressingMode) {
        self.status.decimal_mode = false;
    }

    fn cli(&mut self, _mode: &AddressingMode) {
        self.status.interrupt_disable = false;
    }

    fn clv(&mut self, _mode: &AddressingMode) {
        self.status.overflow = false;
    }

    fn cmp(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        let result = self.register_a.wrapping_sub(value);
        self.status.carry = self.register_a >= value;
        self.status.zero = result == 0;
        self.status.negative = result >> 7 == 1;
    }

    fn cpx(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        let result = self.register_x.wrapping_sub(value);
        self.status.carry = self.register_x >= value;
        self.status.zero = result == 0;
        self.status.negative = result >> 7 == 1;
    }

    fn cpy(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        let result = self.register_y.wrapping_sub(value);
        self.status.carry = self.register_y >= value;
        self.status.zero = result == 0;
        self.status.negative = result >> 7 == 1;
    }

    fn dcp(&mut self, _mode: &AddressingMode) {
        let addr = self.get_operand_address(_mode);
        let value = self.mem_read(addr);
        let result = value.wrapping_sub(1);
        self.mem_write(addr, result);
        if self.register_a >= result {
            self.status.carry = true;
        }
        let result = self.register_a.wrapping_sub(result);
        self.status.zero = result == 0;
        self.status.negative = result >> 7 == 1;
    }

    fn dec(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let result = value.wrapping_sub(1);
        self.mem_write(addr, result);
        self.status.zero = result == 0;
        self.status.negative = result >> 7 == 1;
    }

    fn dex(&mut self, _mode: &AddressingMode) {
        self.register_x = self.register_x.wrapping_sub(1);
        self.status.zero = self.register_x == 0;
        self.status.negative = self.register_x >> 7 == 1;
    }

    fn dey(&mut self, _mode: &AddressingMode) {
        self.register_y = self.register_y.wrapping_sub(1);
        self.status.zero = self.register_y == 0;
        self.status.negative = self.register_y >> 7 == 1;
    }

    fn eor(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        self.register_a ^= value;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn inc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let result = value.wrapping_add(1);
        self.mem_write(addr, result);
        self.status.zero = result == 0;
        self.status.negative = result >> 7 == 1;
    }

    fn inx(&mut self, _mode: &AddressingMode) {
        self.register_x = self.register_x.wrapping_add(1);
        self.status.zero = self.register_x == 0;
        self.status.negative = self.register_x >> 7 == 1;
    }

    fn iny(&mut self, _mode: &AddressingMode) {
        self.register_y = self.register_y.wrapping_add(1);
        self.status.zero = self.register_y == 0;
        self.status.negative = self.register_y >> 7 == 1;
    }

    fn isc(&mut self, mode: &AddressingMode) {
        // inc
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let result = value.wrapping_add(1);
        self.mem_write(addr, result);
        // sbc
        let value = self.get_operand(mode);
        let carry = if self.status.carry { 0 } else { 1 };
        let a_sign = self.register_a >> 7;
        let value_sign = value >> 7;
        let (result, overflow1) = self.register_a.overflowing_sub(value);
        let (result, overflow2) = result.overflowing_sub(carry);
        let result_sign = result >> 7;
        self.register_a = result;
        self.status.carry = !overflow1 && !overflow2;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
        self.status.overflow = a_sign != value_sign && a_sign != result_sign;
    }

    fn jmp(&mut self, mode: &AddressingMode) {
        self.program_counter = match mode {
            AddressingMode::Absolute => self.mem_read_u16(self.program_counter),
            AddressingMode::Indirect => {
                let mem_addr = self.mem_read_u16(self.program_counter);
                if mem_addr & 0x00FF == 0x00FF {
                    let lo = self.mem_read(mem_addr) as u16;
                    let hi = self.mem_read(mem_addr & 0xFF00) as u16;
                    (hi << 8) | lo
                } else {
                    self.mem_read_u16(mem_addr)
                }
            }
            _ => panic!("Invalid addressing mode for JMP instruction"),
        };
    }

    fn jsr(&mut self, _mode: &AddressingMode) {
        let addr = self.mem_read_u16(self.program_counter);
        // self.program_counter += 2;
        self.stack_push_u16(self.program_counter + 1);
        self.program_counter = addr;
    }

    fn las(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode) & self.stack_pointer;
        self.register_a = value;
        self.register_x = value;
        self.stack_pointer = value;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn lax(&mut self, _mode: &AddressingMode) {
        let value = self.get_operand(_mode);
        self.register_a = value;
        self.register_x = value;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn lda(&mut self, mode: &AddressingMode) {
        self.register_a = self.get_operand(mode);
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn ldx(&mut self, mode: &AddressingMode) {
        self.register_x = self.get_operand(mode);
        self.status.zero = self.register_x == 0;
        self.status.negative = self.register_x >> 7 == 1;
    }

    fn ldy(&mut self, mode: &AddressingMode) {
        self.register_y = self.get_operand(mode);
        self.status.zero = self.register_y == 0;
        self.status.negative = self.register_y >> 7 == 1;
    }

    fn lsr(&mut self, mode: &AddressingMode) {
        if let AddressingMode::Accumulator = mode {
            let carry = self.register_a & 1;
            self.status.carry = carry == 1;
            self.register_a >>= 1;
            self.status.zero = self.register_a == 0;
            self.status.negative = self.register_a >> 7 == 1;
        } else {
            let addr = self.get_operand_address(mode);
            let value = self.mem_read(addr);
            let carry = value & 1;
            self.status.carry = carry == 1;
            let result = value >> 1;
            self.mem_write(addr, result);
            self.status.zero = result == 0;
            self.status.negative = result >> 7 == 1;
        }
    }

    fn nop(&mut self, mode: &AddressingMode) {}

    fn ora(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        self.register_a |= value;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn pha(&mut self, _mode: &AddressingMode) {
        self.stack_push(self.register_a);
    }

    fn php(&mut self, _mode: &AddressingMode) {
        let status = self.status.to_byte() | 0b0011_0000;
        self.stack_push(status);
    }

    fn pla(&mut self, _mode: &AddressingMode) {
        self.register_a = self.stack_pop();
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn plp(&mut self, _mode: &AddressingMode) {
        let status = self.stack_pop();
        self.status = StatusRegister::from_byte(status);
        self.status.break_command = false;
    }

    fn rla(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let carry = if self.status.carry { 1 } else { 0 };
        self.status.carry = value >> 7 == 1;
        let result = (value << 1) | carry;
        self.mem_write(addr, result);
        self.register_a &= result;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn rol(&mut self, mode: &AddressingMode) {
        if let AddressingMode::Accumulator = mode {
            let carry = if self.status.carry { 1 } else { 0 };
            self.status.carry = self.register_a >> 7 == 1;
            let result = (self.register_a << 1) | carry;
            self.register_a = result;
            self.status.zero = result == 0;
            self.status.negative = result >> 7 == 1;
        } else {
            let addr = self.get_operand_address(mode);
            let value = self.mem_read(addr);
            let carry = if self.status.carry { 1 } else { 0 };
            self.status.carry = value >> 7 == 1;
            let result = (value << 1) | carry;
            self.mem_write(addr, result);
            self.status.zero = result == 0;
            self.status.negative = result >> 7 == 1;
        }
    }

    fn ror(&mut self, mode: &AddressingMode) {
        if let AddressingMode::Accumulator = mode {
            let carry = if self.status.carry { 1 } else { 0 };
            self.status.carry = self.register_a & 1 == 1;
            let result = (self.register_a >> 1) | (carry << 7);
            self.register_a = result;
            self.status.zero = result == 0;
            self.status.negative = result >> 7 == 1;
        } else {
            let addr = self.get_operand_address(mode);
            let value = self.mem_read(addr);
            let carry = if self.status.carry { 1 } else { 0 };
            self.status.carry = value & 1 == 1;
            let result = (value >> 1) | (carry << 7);
            self.mem_write(addr, result);
            self.status.zero = result == 0;
            self.status.negative = result >> 7 == 1;
        }
    }

    fn rra(&mut self, mode: &AddressingMode) {
        // ror
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let carry = if self.status.carry { 1 } else { 0 };
        self.status.carry = value & 1 == 1;
        let result = (value >> 1) | (carry << 7);
        self.mem_write(addr, result);
        // adc
        let value = self.get_operand(mode);
        let carry = if self.status.carry { 1 } else { 0 };
        let a_sign = self.register_a >> 7;
        let value_sign = value >> 7;
        let (result, overflow1) = self.register_a.overflowing_add(value);
        let (result, overflow2) = result.overflowing_add(carry);
        let result_sign = result >> 7;
        self.register_a = result;
        self.status.carry = overflow1 | overflow2;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
        self.status.overflow = a_sign == value_sign && a_sign != result_sign;
    }

    fn rti(&mut self, _mode: &AddressingMode) {
        let status = self.stack_pop();
        self.status = StatusRegister::from_byte(status);

        let lo = self.stack_pop() as u16;
        let hi = self.stack_pop() as u16;
        self.program_counter = (hi << 8) | lo;
    }

    fn rts(&mut self, _mode: &AddressingMode) {
        self.program_counter = self.stack_pop_u16().wrapping_add(1);
    }

    fn sax(&mut self, mode: &AddressingMode) {
        let value = self.register_a & self.register_x;
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, value);
    }

    fn sbc(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        let carry = if self.status.carry { 0 } else { 1 };
        let a_sign = self.register_a >> 7;
        let value_sign = value >> 7;
        let (result, overflow1) = self.register_a.overflowing_sub(value);
        let (result, overflow2) = result.overflowing_sub(carry);
        let result_sign = result >> 7;
        self.register_a = result;
        self.status.carry = !overflow1 && !overflow2;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
        self.status.overflow = a_sign != value_sign && a_sign != result_sign;
    }

    fn sec(&mut self, _mode: &AddressingMode) {
        self.status.carry = true;
    }

    fn sed(&mut self, _mode: &AddressingMode) {
        self.status.decimal_mode = true;
    }

    fn sei(&mut self, _mode: &AddressingMode) {
        self.status.interrupt_disable = true;
    }

    fn shx(&mut self, _mode: &AddressingMode) {
         let mem_address = self.mem_read_u16(self.program_counter) + self.register_y as u16;
         // self.program_counter += 2;
         let data = self.register_x & ((mem_address >> 8) as u8 + 1);
         self.mem_write(mem_address, data)
    }

    fn shy(&mut self, _mode: &AddressingMode) {
         let mem_address = self.mem_read_u16(self.program_counter) + self.register_x as u16;
         // self.program_counter += 2;
         let data = self.register_y & ((mem_address >> 8) as u8 + 1);
         self.mem_write(mem_address, data)
    }

    fn slo(&mut self, _mode: &AddressingMode) {
        let addr = self.get_operand_address(_mode);
        let value = self.mem_read(addr);
        self.status.carry = value >> 7 == 1;
        let result = value << 1;
        self.mem_write(addr, result);
        self.register_a |= result;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn sre(&mut self, _mode: &AddressingMode) {
        let addr = self.get_operand_address(_mode);
        let value = self.mem_read(addr);
        self.status.carry = value & 1 == 1;
        let result = value >> 1;
        self.mem_write(addr, result);
        self.register_a ^= result;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_a);
    }

    fn stp(&mut self, _mode: &AddressingMode) {}

    fn stx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_x);
    }

    fn sty(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_y);
    }

    fn tas(&mut self, _mode: &AddressingMode) {
        let mem_address = self.mem_read_u16(self.program_counter) + self.register_y as u16;
        // self.program_counter += 2;
        let data = self.register_a & self.register_x;
        self.stack_pointer = data;
        self.mem_write(mem_address, data);
    }

    fn tax(&mut self, _mode: &AddressingMode) {
        self.register_x = self.register_a;
        self.status.zero = self.register_x == 0;
        self.status.negative = self.register_x >> 7 == 1;
    }

    fn tay(&mut self, _mode: &AddressingMode) {
        self.register_y = self.register_a;
        self.status.zero = self.register_y == 0;
        self.status.negative = self.register_y >> 7 == 1;
    }

    fn tsx(&mut self, _mode: &AddressingMode) {
        self.register_x = self.stack_pointer;
        self.status.zero = self.register_x == 0;
        self.status.negative = self.register_x >> 7 == 1;
    }

    fn txa(&mut self, _mode: &AddressingMode) {
        self.register_a = self.register_x;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn txs(&mut self, _mode: &AddressingMode) {
        self.stack_pointer = self.register_x;
    }

    fn tya(&mut self, _mode: &AddressingMode) {
        self.register_a = self.register_y;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn xaa(&mut self, _mode: &AddressingMode) {
        let value = self.get_operand(_mode);
        self.register_a = self.register_x & value;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }


    fn get_operand_address(&mut self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Implicit => panic!("No operand for this instruction"),
            AddressingMode::Accumulator => panic!("No operand for this instruction"),
            AddressingMode::Immediate => {
                let pos = self.program_counter;
                // self.program_counter += 1;
                pos
            }
            AddressingMode::ZeroPage => {
                let pos = self.program_counter;
                // self.program_counter += 1;
                self.mem_read(pos) as u16
            }
            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read(self.program_counter);
                // self.program_counter += 1;
                pos.wrapping_add(self.register_x) as u16
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read(self.program_counter);
                // self.program_counter += 1;
                pos.wrapping_add(self.register_y) as u16
            }
            AddressingMode::Relative => panic!("No operand for this instruction"),
            AddressingMode::Absolute => {
                let pos = self.program_counter;
                // self.program_counter += 2;
                self.mem_read_u16(pos)
            }
            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(self.program_counter);
                // self.program_counter += 2;
                base.wrapping_add(self.register_x as u16)
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(self.program_counter);
                // self.program_counter += 2;
                base.wrapping_add(self.register_y as u16)
            }
            AddressingMode::Indirect => panic!("No operand for this instruction"),
            AddressingMode::Indirect_X => {
                let base = self.mem_read(self.program_counter);
                // self.program_counter += 1;
                let ptr: u8 = (base as u8).wrapping_add(self.register_x);
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(self.program_counter);
                // self.program_counter += 1;
                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                deref
            }
        }
    }

    fn get_operand(&mut self, mode: &AddressingMode) -> u8 {
        match mode {
            AddressingMode::Implicit => panic!("No operand for this instruction"),
            AddressingMode::Accumulator => self.register_a,
            AddressingMode::Relative => {
                let offset = self.mem_read(self.program_counter);
                // self.program_counter += 1;
                offset
            }
            AddressingMode::Indirect => {
                let ptr = self.mem_read_u16(self.program_counter);
                // self.program_counter += 2;
                let lo = self.mem_read(ptr) as u16;
                let hi = self.mem_read(ptr.wrapping_add(1)) as u16;
                let addr = (hi << 8) | lo;
                self.mem_read(addr)
            }
            _ => {
                let addr = self.get_operand_address(mode);
                self.mem_read(addr)
            }
        }
    }
}


#[derive(Debug)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
    Implicit,
    Accumulator,
    Immediate,
    ZeroPage,
    ZeroPage_X,
    ZeroPage_Y,
    Relative,
    Absolute,
    Absolute_X,
    Absolute_Y,
    Indirect,
    Indirect_X,
    Indirect_Y,
}



#[cfg(test)]
mod test {
    use super::*;
    use crate::cartridge::test;

    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.register_a, 5);
        assert!(cpu.status.to_byte() & 0b0000_0010 == 0b00);
        assert!(cpu.status.to_byte() & 0b1000_0000 == 0);
    }

    #[test]
    fn test_5_ops_working_together() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 0xc1)
    }
}
