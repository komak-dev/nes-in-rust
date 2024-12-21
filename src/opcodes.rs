use crate::cpu::AddressingMode;
use std::collections::HashMap;
use once_cell::sync::Lazy;


#[derive(Debug)]
pub enum Mnemonic {
    ADC, AND, ASL, BCC, BCS, BEQ, BIT, BMI, BNE, BPL, BRK, BVC, BVS, CLC,
    CLD, CLI, CLV, CMP, CPX, CPY, DEC, DEX, DEY, EOR, INC, INX, INY, JMP,
    JSR, LDA, LDX, LDY, LSR, NOP, ORA, PHA, PHP, PLA, PLP, ROL, ROR, RTI,
    RTS, SBC, SEC, SED, SEI, STA, STX, STY, TAX, TAY, TSX, TXA, TXS, TYA,
}

pub struct OpCode {
    pub code: u8,
    pub mnemonic: Mnemonic,
    pub mode: AddressingMode,
    pub bytes: u8,
    pub cycles: u8,
}

impl OpCode {
    const fn new(code: u8, mnemonic: Mnemonic, mode: AddressingMode, bytes: u8, cycles: u8) -> OpCode {
        OpCode { code, mnemonic, mode, bytes, cycles }
    }
}


pub static OPCODES_MAP: Lazy<HashMap<u8, &OpCode>> = Lazy::new(|| {
    OPCODES.iter().map(|op| (op.code, op)).collect()
});


pub const OPCODES: [OpCode; 151] = [
    OpCode::new(0x69, Mnemonic::ADC, AddressingMode::Immediate, 2, 2),
    OpCode::new(0x65, Mnemonic::ADC, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0x75, Mnemonic::ADC, AddressingMode::ZeroPage_X, 2, 4),
    OpCode::new(0x6D, Mnemonic::ADC, AddressingMode::Absolute, 3, 4),
    OpCode::new(0x7D, Mnemonic::ADC, AddressingMode::Absolute_X, 3, 4),
    OpCode::new(0x79, Mnemonic::ADC, AddressingMode::Absolute_Y, 3, 4),
    OpCode::new(0x61, Mnemonic::ADC, AddressingMode::Indirect_X, 2, 6),
    OpCode::new(0x71, Mnemonic::ADC, AddressingMode::Indirect_Y, 2, 5),
    OpCode::new(0x29, Mnemonic::AND, AddressingMode::Immediate, 2, 2),
    OpCode::new(0x25, Mnemonic::AND, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0x35, Mnemonic::AND, AddressingMode::ZeroPage_X, 2, 4),
    OpCode::new(0x2D, Mnemonic::AND, AddressingMode::Absolute, 3, 4),
    OpCode::new(0x3D, Mnemonic::AND, AddressingMode::Absolute_X, 3, 4),
    OpCode::new(0x39, Mnemonic::AND, AddressingMode::Absolute_Y, 3, 4),
    OpCode::new(0x21, Mnemonic::AND, AddressingMode::Indirect_X, 2, 6),
    OpCode::new(0x31, Mnemonic::AND, AddressingMode::Indirect_Y, 2, 5),
    OpCode::new(0x0A, Mnemonic::ASL, AddressingMode::Accumulator, 1, 2),
    OpCode::new(0x06, Mnemonic::ASL, AddressingMode::ZeroPage, 2, 5),
    OpCode::new(0x16, Mnemonic::ASL, AddressingMode::ZeroPage_X, 2, 6),
    OpCode::new(0x0E, Mnemonic::ASL, AddressingMode::Absolute, 3, 6),
    OpCode::new(0x1E, Mnemonic::ASL, AddressingMode::Absolute_X, 3, 7),
    OpCode::new(0x90, Mnemonic::BCC, AddressingMode::Relative, 2, 2),
    OpCode::new(0xB0, Mnemonic::BCS, AddressingMode::Relative, 2, 2),
    OpCode::new(0xF0, Mnemonic::BEQ, AddressingMode::Relative, 2, 2),
    OpCode::new(0x24, Mnemonic::BIT, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0x2C, Mnemonic::BIT, AddressingMode::Absolute, 3, 4),
    OpCode::new(0x30, Mnemonic::BMI, AddressingMode::Relative, 2, 2),
    OpCode::new(0xD0, Mnemonic::BNE, AddressingMode::Relative, 2, 2),
    OpCode::new(0x10, Mnemonic::BPL, AddressingMode::Relative, 2, 2),
    OpCode::new(0x00, Mnemonic::BRK, AddressingMode::Implicit, 1, 7),
    OpCode::new(0x50, Mnemonic::BVC, AddressingMode::Relative, 2, 2),
    OpCode::new(0x70, Mnemonic::BVS, AddressingMode::Relative, 2, 2),
    OpCode::new(0x18, Mnemonic::CLC, AddressingMode::Implicit, 1, 2),
    OpCode::new(0xD8, Mnemonic::CLD, AddressingMode::Implicit, 1, 2),
    OpCode::new(0x58, Mnemonic::CLI, AddressingMode::Implicit, 1, 2),
    OpCode::new(0xB8, Mnemonic::CLV, AddressingMode::Implicit, 1, 2),
    OpCode::new(0xC9, Mnemonic::CMP, AddressingMode::Immediate, 2, 2),
    OpCode::new(0xC5, Mnemonic::CMP, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0xD5, Mnemonic::CMP, AddressingMode::ZeroPage_X, 2, 4),
    OpCode::new(0xCD, Mnemonic::CMP, AddressingMode::Absolute, 3, 4),
    OpCode::new(0xDD, Mnemonic::CMP, AddressingMode::Absolute_X, 3, 4),
    OpCode::new(0xD9, Mnemonic::CMP, AddressingMode::Absolute_Y, 3, 4),
    OpCode::new(0xC1, Mnemonic::CMP, AddressingMode::Indirect_X, 2, 6),
    OpCode::new(0xD1, Mnemonic::CMP, AddressingMode::Indirect_Y, 2, 5),
    OpCode::new(0xE0, Mnemonic::CPX, AddressingMode::Immediate, 2, 2),
    OpCode::new(0xE4, Mnemonic::CPX, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0xEC, Mnemonic::CPX, AddressingMode::Absolute, 3, 4),
    OpCode::new(0xC0, Mnemonic::CPY, AddressingMode::Immediate, 2, 2),
    OpCode::new(0xC4, Mnemonic::CPY, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0xCC, Mnemonic::CPY, AddressingMode::Absolute, 3, 4),
    OpCode::new(0xC6, Mnemonic::DEC, AddressingMode::ZeroPage, 2, 5),
    OpCode::new(0xD6, Mnemonic::DEC, AddressingMode::ZeroPage_X, 2, 6),
    OpCode::new(0xCE, Mnemonic::DEC, AddressingMode::Absolute, 3, 6),
    OpCode::new(0xDE, Mnemonic::DEC, AddressingMode::Absolute_X, 3, 7),
    OpCode::new(0xCA, Mnemonic::DEX, AddressingMode::Implicit, 1, 2),
    OpCode::new(0x88, Mnemonic::DEY, AddressingMode::Implicit, 1, 2),
    OpCode::new(0x49, Mnemonic::EOR, AddressingMode::Immediate, 2, 2),
    OpCode::new(0x45, Mnemonic::EOR, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0x55, Mnemonic::EOR, AddressingMode::ZeroPage_X, 2, 4),
    OpCode::new(0x4D, Mnemonic::EOR, AddressingMode::Absolute, 3, 4),
    OpCode::new(0x5D, Mnemonic::EOR, AddressingMode::Absolute_X, 3, 4),
    OpCode::new(0x59, Mnemonic::EOR, AddressingMode::Absolute_Y, 3, 4),
    OpCode::new(0x41, Mnemonic::EOR, AddressingMode::Indirect_X, 2, 6),
    OpCode::new(0x51, Mnemonic::EOR, AddressingMode::Indirect_Y, 2, 5),
    OpCode::new(0xE6, Mnemonic::INC, AddressingMode::ZeroPage, 2, 5),
    OpCode::new(0xF6, Mnemonic::INC, AddressingMode::ZeroPage_X, 2, 6),
    OpCode::new(0xEE, Mnemonic::INC, AddressingMode::Absolute, 3, 6),
    OpCode::new(0xFE, Mnemonic::INC, AddressingMode::Absolute_X, 3, 7),
    OpCode::new(0xE8, Mnemonic::INX, AddressingMode::Implicit, 1, 2),
    OpCode::new(0xC8, Mnemonic::INY, AddressingMode::Implicit, 1, 2),
    OpCode::new(0x4C, Mnemonic::JMP, AddressingMode::Absolute, 3, 3),
    OpCode::new(0x6C, Mnemonic::JMP, AddressingMode::Indirect, 3, 5),
    OpCode::new(0x20, Mnemonic::JSR, AddressingMode::Absolute, 3, 6),
    OpCode::new(0xA9, Mnemonic::LDA, AddressingMode::Immediate, 2, 2),
    OpCode::new(0xA5, Mnemonic::LDA, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0xB5, Mnemonic::LDA, AddressingMode::ZeroPage_X, 2, 4),
    OpCode::new(0xAD, Mnemonic::LDA, AddressingMode::Absolute, 3, 4),
    OpCode::new(0xBD, Mnemonic::LDA, AddressingMode::Absolute_X, 3, 4),
    OpCode::new(0xB9, Mnemonic::LDA, AddressingMode::Absolute_Y, 3, 4),
    OpCode::new(0xA1, Mnemonic::LDA, AddressingMode::Indirect_X, 2, 6),
    OpCode::new(0xB1, Mnemonic::LDA, AddressingMode::Indirect_Y, 2, 5),
    OpCode::new(0xA2, Mnemonic::LDX, AddressingMode::Immediate, 2, 2),
    OpCode::new(0xA6, Mnemonic::LDX, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0xB6, Mnemonic::LDX, AddressingMode::ZeroPage_Y, 2, 4),
    OpCode::new(0xAE, Mnemonic::LDX, AddressingMode::Absolute, 3, 4),
    OpCode::new(0xBE, Mnemonic::LDX, AddressingMode::Absolute_Y, 3, 4),
    OpCode::new(0xA0, Mnemonic::LDY, AddressingMode::Immediate, 2, 2),
    OpCode::new(0xA4, Mnemonic::LDY, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0xB4, Mnemonic::LDY, AddressingMode::ZeroPage_X, 2, 4),
    OpCode::new(0xAC, Mnemonic::LDY, AddressingMode::Absolute, 3, 4),
    OpCode::new(0xBC, Mnemonic::LDY, AddressingMode::Absolute_X, 3, 4),
    OpCode::new(0x4A, Mnemonic::LSR, AddressingMode::Accumulator, 1, 2),
    OpCode::new(0x46, Mnemonic::LSR, AddressingMode::ZeroPage, 2, 5),
    OpCode::new(0x56, Mnemonic::LSR, AddressingMode::ZeroPage_X, 2, 6),
    OpCode::new(0x4E, Mnemonic::LSR, AddressingMode::Absolute, 3, 6),
    OpCode::new(0x5E, Mnemonic::LSR, AddressingMode::Absolute_X, 3, 7),
    OpCode::new(0xEA, Mnemonic::NOP, AddressingMode::Implicit, 1, 2),
    OpCode::new(0x09, Mnemonic::ORA, AddressingMode::Immediate, 2, 2),
    OpCode::new(0x05, Mnemonic::ORA, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0x15, Mnemonic::ORA, AddressingMode::ZeroPage_X, 2, 4),
    OpCode::new(0x0D, Mnemonic::ORA, AddressingMode::Absolute, 3, 4),
    OpCode::new(0x1D, Mnemonic::ORA, AddressingMode::Absolute_X, 3, 4),
    OpCode::new(0x19, Mnemonic::ORA, AddressingMode::Absolute_Y, 3, 4),
    OpCode::new(0x01, Mnemonic::ORA, AddressingMode::Indirect_X, 2, 6),
    OpCode::new(0x11, Mnemonic::ORA, AddressingMode::Indirect_Y, 2, 5),
    OpCode::new(0x48, Mnemonic::PHA, AddressingMode::Implicit, 1, 3),
    OpCode::new(0x08, Mnemonic::PHP, AddressingMode::Implicit, 1, 3),
    OpCode::new(0x68, Mnemonic::PLA, AddressingMode::Implicit, 1, 4),
    OpCode::new(0x28, Mnemonic::PLP, AddressingMode::Implicit, 1, 4),
    OpCode::new(0x2A, Mnemonic::ROL, AddressingMode::Accumulator, 1, 2),
    OpCode::new(0x26, Mnemonic::ROL, AddressingMode::ZeroPage, 2, 5),
    OpCode::new(0x36, Mnemonic::ROL, AddressingMode::ZeroPage_X, 2, 6),
    OpCode::new(0x2E, Mnemonic::ROL, AddressingMode::Absolute, 3, 6),
    OpCode::new(0x3E, Mnemonic::ROL, AddressingMode::Absolute_X, 3, 7),
    OpCode::new(0x6A, Mnemonic::ROR, AddressingMode::Accumulator, 1, 2),
    OpCode::new(0x66, Mnemonic::ROR, AddressingMode::ZeroPage, 2, 5),
    OpCode::new(0x76, Mnemonic::ROR, AddressingMode::ZeroPage_X, 2, 6),
    OpCode::new(0x6E, Mnemonic::ROR, AddressingMode::Absolute, 3, 6),
    OpCode::new(0x7E, Mnemonic::ROR, AddressingMode::Absolute_X, 3, 7),
    OpCode::new(0x40, Mnemonic::RTI, AddressingMode::Implicit, 1, 6),
    OpCode::new(0x60, Mnemonic::RTS, AddressingMode::Implicit, 1, 6),
    OpCode::new(0xE9, Mnemonic::SBC, AddressingMode::Immediate, 2, 2),
    OpCode::new(0xE5, Mnemonic::SBC, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0xF5, Mnemonic::SBC, AddressingMode::ZeroPage_X, 2, 4),
    OpCode::new(0xED, Mnemonic::SBC, AddressingMode::Absolute, 3, 4),
    OpCode::new(0xFD, Mnemonic::SBC, AddressingMode::Absolute_X, 3, 4),
    OpCode::new(0xF9, Mnemonic::SBC, AddressingMode::Absolute_Y, 3, 4),
    OpCode::new(0xE1, Mnemonic::SBC, AddressingMode::Indirect_X, 2, 6),
    OpCode::new(0xF1, Mnemonic::SBC, AddressingMode::Indirect_Y, 2, 5),
    OpCode::new(0x38, Mnemonic::SEC, AddressingMode::Implicit, 1, 2),
    OpCode::new(0xF8, Mnemonic::SED, AddressingMode::Implicit, 1, 2),
    OpCode::new(0x78, Mnemonic::SEI, AddressingMode::Implicit, 1, 2),
    OpCode::new(0x85, Mnemonic::STA, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0x95, Mnemonic::STA, AddressingMode::ZeroPage_X, 2, 4),
    OpCode::new(0x8D, Mnemonic::STA, AddressingMode::Absolute, 3, 4),
    OpCode::new(0x9D, Mnemonic::STA, AddressingMode::Absolute_X, 3, 5),
    OpCode::new(0x99, Mnemonic::STA, AddressingMode::Absolute_Y, 3, 5),
    OpCode::new(0x81, Mnemonic::STA, AddressingMode::Indirect_X, 2, 6),
    OpCode::new(0x91, Mnemonic::STA, AddressingMode::Indirect_Y, 2, 6),
    OpCode::new(0x86, Mnemonic::STX, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0x96, Mnemonic::STX, AddressingMode::ZeroPage_Y, 2, 4),
    OpCode::new(0x8E, Mnemonic::STX, AddressingMode::Absolute, 3, 4),
    OpCode::new(0x84, Mnemonic::STY, AddressingMode::ZeroPage, 2, 3),
    OpCode::new(0x94, Mnemonic::STY, AddressingMode::ZeroPage_X, 2, 4),
    OpCode::new(0x8C, Mnemonic::STY, AddressingMode::Absolute, 3, 4),
    OpCode::new(0xAA, Mnemonic::TAX, AddressingMode::Implicit, 1, 2),
    OpCode::new(0xA8, Mnemonic::TAY, AddressingMode::Implicit, 1, 2),
    OpCode::new(0xBA, Mnemonic::TSX, AddressingMode::Implicit, 1, 2),
    OpCode::new(0x8A, Mnemonic::TXA, AddressingMode::Implicit, 1, 2),
    OpCode::new(0x9A, Mnemonic::TXS, AddressingMode::Implicit, 1, 2),
    OpCode::new(0x98, Mnemonic::TYA, AddressingMode::Implicit, 1, 2),
];
