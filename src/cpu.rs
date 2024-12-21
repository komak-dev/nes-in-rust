use crate::opcodes::{OPCODES_MAP, Mnemonic};


pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub stack_pointer: u8,
    pub status: StatusRegister,
    pub program_counter: u16,
    memory: [u8; 0xffff],
}

impl CPU {
    pub fn new() -> Self {
        Self {
            register_a: 0,
            register_x: 0,
            register_y: 0,
            stack_pointer: 0xfd,
            status: StatusRegister::from_byte(0b0000_0000),
            program_counter: 0,
            memory: [0; 0xffff],
        }
    }


    pub fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    pub fn mem_read_u16(&mut self, pos: u16) -> u16 {
        let lo = self.mem_read(pos) as u16;
        let hi = self.mem_read(pos + 1) as u16;
        (hi << 8) | lo
    }

    pub fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    pub fn mem_write_u16(&mut self, pos: u16, data: u16) {
        let lo = (data & 0xff) as u8;
        let hi = (data >> 8) as u8;
        self.mem_write(pos, lo);
        self.mem_write(pos + 1, hi);
    }

    pub fn mem_ref_mut(&mut self, addr: u16) -> &mut u8 {
        &mut self.memory[addr as usize]
    }

    fn stack_push(&mut self, data: u8) {
        self.mem_write(0x100 + self.stack_pointer as u16, data);
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
        self.mem_read(0x100 + self.stack_pointer as u16)
    }

    fn stack_pop_u16(&mut self) -> u16 {
        let lo = self.stack_pop() as u16;
        let hi = self.stack_pop() as u16;
        (hi << 8) | lo
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run();
    }

    pub fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.status = StatusRegister::from_byte(0b0000_0000);

        self.program_counter = self.mem_read_u16(0xFFFC);
    }

    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x0600..(0x0600 + program.len())]
            .copy_from_slice(&program[..]);
        self.mem_write_u16(0xFFFC, 0x0600);
    }

    pub fn run(&mut self) {
        self.run_with_callback(|_| {});
    }

    pub fn run_with_callback<F>(&mut self, mut callback: F)
    where
        F: FnMut(&mut Self),
    {
        let ref map = OPCODES_MAP;

        loop {
            callback(self);


            let opscode = self.mem_read(self.program_counter);
            self.program_counter = self.program_counter.wrapping_add(1);

            if opscode == 0x00 {
                break;
            }

            match map[&opscode].mnemonic {
                Mnemonic::ADC => self.adc(&map[&opscode].mode),
                Mnemonic::AND => self.and(&map[&opscode].mode),
                Mnemonic::ASL => self.asl(&map[&opscode].mode),
                Mnemonic::BCC => self.bcc(&map[&opscode].mode),
                Mnemonic::BCS => self.bcs(&map[&opscode].mode),
                Mnemonic::BEQ => self.beq(&map[&opscode].mode),
                Mnemonic::BIT => self.bit(&map[&opscode].mode),
                Mnemonic::BMI => self.bmi(&map[&opscode].mode),
                Mnemonic::BNE => self.bne(&map[&opscode].mode),
                Mnemonic::BPL => self.bpl(&map[&opscode].mode),
                Mnemonic::BRK => self.brk(&map[&opscode].mode),
                Mnemonic::BVC => self.bvc(&map[&opscode].mode),
                Mnemonic::BVS => self.bvs(&map[&opscode].mode),
                Mnemonic::CLC => self.clc(&map[&opscode].mode),
                Mnemonic::CLD => self.cld(&map[&opscode].mode),
                Mnemonic::CLI => self.cli(&map[&opscode].mode),
                Mnemonic::CLV => self.clv(&map[&opscode].mode),
                Mnemonic::CMP => self.cmp(&map[&opscode].mode),
                Mnemonic::CPX => self.cpx(&map[&opscode].mode),
                Mnemonic::CPY => self.cpy(&map[&opscode].mode),
                Mnemonic::DEC => self.dec(&map[&opscode].mode),
                Mnemonic::DEX => self.dex(&map[&opscode].mode),
                Mnemonic::DEY => self.dey(&map[&opscode].mode),
                Mnemonic::EOR => self.eor(&map[&opscode].mode),
                Mnemonic::INC => self.inc(&map[&opscode].mode),
                Mnemonic::INX => self.inx(&map[&opscode].mode),
                Mnemonic::INY => self.iny(&map[&opscode].mode),
                Mnemonic::JMP => self.jmp(&map[&opscode].mode),
                Mnemonic::JSR => self.jsr(&map[&opscode].mode),
                Mnemonic::LDA => self.lda(&map[&opscode].mode),
                Mnemonic::LDX => self.ldx(&map[&opscode].mode),
                Mnemonic::LDY => self.ldy(&map[&opscode].mode),
                Mnemonic::LSR => self.lsr(&map[&opscode].mode),
                Mnemonic::NOP => self.nop(&map[&opscode].mode),
                Mnemonic::ORA => self.ora(&map[&opscode].mode),
                Mnemonic::PHA => self.pha(&map[&opscode].mode),
                Mnemonic::PHP => self.php(&map[&opscode].mode),
                Mnemonic::PLA => self.pla(&map[&opscode].mode),
                Mnemonic::PLP => self.plp(&map[&opscode].mode),
                Mnemonic::ROL => self.rol(&map[&opscode].mode),
                Mnemonic::ROR => self.ror(&map[&opscode].mode),
                Mnemonic::RTI => self.rti(&map[&opscode].mode),
                Mnemonic::RTS => self.rts(&map[&opscode].mode),
                Mnemonic::SBC => self.sbc(&map[&opscode].mode),
                Mnemonic::SEC => self.sec(&map[&opscode].mode),
                Mnemonic::SED => self.sed(&map[&opscode].mode),
                Mnemonic::SEI => self.sei(&map[&opscode].mode),
                Mnemonic::STA => self.sta(&map[&opscode].mode),
                Mnemonic::STX => self.stx(&map[&opscode].mode),
                Mnemonic::STY => self.sty(&map[&opscode].mode),
                Mnemonic::TAX => self.tax(&map[&opscode].mode),
                Mnemonic::TAY => self.tay(&map[&opscode].mode),
                Mnemonic::TSX => self.tsx(&map[&opscode].mode),
                Mnemonic::TXA => self.txa(&map[&opscode].mode),
                Mnemonic::TXS => self.txs(&map[&opscode].mode),
                Mnemonic::TYA => self.tya(&map[&opscode].mode),
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

    fn and(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        self.register_a &= value;
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
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

    fn bcc(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if !self.status.carry {
            self.program_counter = self.program_counter.wrapping_add(offset as u16);
        }
    }

    fn bcs(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if self.status.carry {
            self.program_counter = self.program_counter.wrapping_add(offset as u16);
        }
    }

    fn beq(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if self.status.zero {
            self.program_counter = self.program_counter.wrapping_add(offset as u16);
        }
    }

    fn bit(&mut self, mode: &AddressingMode) {
        let value = self.get_operand(mode);
        let result = self.register_a & value;
        self.status.zero = result == 0;
        self.status.overflow = value >> 6 == 1;
        self.status.negative = value >> 7 == 1;
    }

    fn bmi(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if self.status.negative {
            self.program_counter = self.program_counter.wrapping_add(offset as u16);
        }
    }

    fn bne(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if !self.status.zero {
            self.program_counter = self.program_counter.wrapping_add(offset as u16);
        }
    }

    fn bpl(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if !self.status.negative {
            self.program_counter = self.program_counter.wrapping_add(offset as u16);
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
            self.program_counter = self.program_counter.wrapping_add(offset as u16);
        }
    }

    fn bvs(&mut self, mode: &AddressingMode) {
        let offset = self.get_operand(mode) as i8;
        if self.status.overflow {
            self.program_counter = self.program_counter.wrapping_add(offset as u16);
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

    fn jmp(&mut self, mode: &AddressingMode) {
        self.program_counter = match mode {
            AddressingMode::Absolute => self.mem_read_u16(self.program_counter),
            _ => panic!("Invalid addressing mode for JMP instruction"),
        };
    }

    fn jsr(&mut self, _mode: &AddressingMode) {
        let addr = self.mem_read_u16(self.program_counter);
        self.program_counter += 2;
        self.stack_push_u16(self.program_counter);
        self.program_counter = addr;
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

    fn nop(&mut self, _mode: &AddressingMode) {}

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
        self.stack_push(self.status.to_byte());
    }

    fn pla(&mut self, _mode: &AddressingMode) {
        self.register_a = self.stack_pop();
        self.status.zero = self.register_a == 0;
        self.status.negative = self.register_a >> 7 == 1;
    }

    fn plp(&mut self, _mode: &AddressingMode) {
        let status = self.stack_pop();
        self.status = StatusRegister::from_byte(status);
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

    fn rti(&mut self, _mode: &AddressingMode) {
        let status = self.stack_pop();
        self.status = StatusRegister::from_byte(status);

        let lo = self.stack_pop() as u16;
        let hi = self.stack_pop() as u16;
        self.program_counter = (hi << 8) | lo;
    }

    fn rts(&mut self, _mode: &AddressingMode) {
        let lo = self.stack_pop() as u16;
        let hi = self.stack_pop() as u16;
        self.program_counter = (hi << 8) | lo;
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

    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_a);
    }

    fn stx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_x);
    }

    fn sty(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_y);
    }

    fn tax(&mut self, _mode: &AddressingMode) {
        self.register_x = self.register_a;
        self.status.zero = self.register_x == 0;
    }

    fn tay(&mut self, _mode: &AddressingMode) {
        self.register_y = self.register_a;
        self.status.zero = self.register_y == 0;
    }

    fn tsx(&mut self, _mode: &AddressingMode) {
        self.register_x = self.stack_pointer;
        self.status.zero = self.register_x == 0;
    }

    fn txa(&mut self, _mode: &AddressingMode) {
        self.register_a = self.register_x;
        self.status.zero = self.register_a == 0;
    }

    fn txs(&mut self, _mode: &AddressingMode) {
        self.stack_pointer = self.register_x;
    }

    fn tya(&mut self, _mode: &AddressingMode) {
        self.register_a = self.register_y;
        self.status.zero = self.register_a == 0;
    }

    fn get_operand_address(&mut self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Implicit => panic!("No operand for this instruction"),
            AddressingMode::Accumulator => panic!("No operand for this instruction"),
            AddressingMode::Immediate => panic!("No operand for this instruction"),
            AddressingMode::ZeroPage => {
                let pos = self.program_counter;
                self.program_counter += 1;
                self.mem_read(pos) as u16
            }
            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read(self.program_counter);
                self.program_counter += 1;
                pos.wrapping_add(self.register_x) as u16
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read(self.program_counter);
                self.program_counter += 1;
                pos.wrapping_add(self.register_y) as u16
            }
            AddressingMode::Relative => panic!("No operand for this instruction"),
            AddressingMode::Absolute => {
                let pos = self.program_counter;
                self.program_counter += 2;
                self.mem_read_u16(pos)
            }
            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(self.program_counter);
                self.program_counter += 2;
                base.wrapping_add(self.register_x as u16)
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(self.program_counter);
                self.program_counter += 2;
                println!("pc += 2");
                base.wrapping_add(self.register_y as u16)
            }
            AddressingMode::Indirect => panic!("No operand for this instruction"),
            AddressingMode::Indirect_X => {
                let base = self.mem_read(self.program_counter);
                self.program_counter += 1;
                let ptr = base.wrapping_add(self.register_x) as u16;
                let lo = self.mem_read(ptr) as u16;
                let hi = self.mem_read(ptr.wrapping_add(1)) as u16;
                (hi << 8) | lo
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(self.program_counter);
                self.program_counter += 1;
                let lo = self.mem_read(base as u16) as u16;
                let hi = self.mem_read(base.wrapping_add(1) as u16) as u16;
                let deref_base = (hi << 8) | lo;
                deref_base.wrapping_add(self.register_y as u16)
            }
        }
    }

    fn get_operand(&mut self, mode: &AddressingMode) -> u8 {
        match mode {
            AddressingMode::Implicit => panic!("No operand for this instruction"),
            AddressingMode::Accumulator => self.register_a,
            AddressingMode::Immediate => {
                let value = self.mem_read(self.program_counter);
                self.program_counter += 1;
                value
            }
            AddressingMode::Relative => {
                let offset = self.mem_read(self.program_counter);
                self.program_counter += 1;
                offset
            }
            AddressingMode::Indirect => {
                let ptr = self.mem_read_u16(self.program_counter);
                self.program_counter += 2;
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
    fn from_byte(byte: u8) -> Self {
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

    fn to_byte(&self) -> u8 {
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


#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.register_a, 5);
        assert!(cpu.status.to_byte() & 0b0000_0010 == 0b00);
        assert!(cpu.status.to_byte() & 0b1000_0000 == 0);
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.load(vec![0xaa, 0x00]);
        cpu.reset();
        cpu.register_a = 10;
        cpu.run();

        assert_eq!(cpu.register_x, 10)
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 0xc1)
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.load(vec![0xe8, 0xe8, 0x00]);
        cpu.reset();
        cpu.register_x = 0xff;
        cpu.run();

        assert_eq!(cpu.register_x, 1)
    }

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x10, 0x55);

        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);

        assert_eq!(cpu.register_a, 0x55);
    }
}
