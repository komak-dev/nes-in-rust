use crate::cartridge::Rom;

const RAM: u16 = 0x0000;
const RAM_MIRRORS_END: u16 = 0x1FFF;
const PPU_REGISTERS: u16 = 0x2000;
const PPU_REGISTERS_MIRRORS_END: u16 = 0x3FFF;

pub struct Bus {
    cpu_vram: [u8; 2048],
    rom: Rom,
}

impl Bus {
    pub fn new(rom: Rom) -> Self{
        Bus {
            cpu_vram: [0; 2048],
            rom,
        }
    }

    fn read_prg_rom(&self, mut addr: u16) -> u8 {
        addr -= 0x8000;
        if self.rom.prg_rom.len() == 0x4000 && addr >= 0x4000 {
            addr -= 0x4000;
        }
        self.rom.prg_rom[addr as usize]
    }

    pub fn mem_read(&self, addr: u16) -> u8 {
        match addr {
            0x0000..=0x1FFF => {
                // RAM
                let mirror_down_addr = addr & 0b00000111_11111111;
                self.cpu_vram[mirror_down_addr as usize]
            }
            0x2000..=0x3FFF => {
                // PPU registers
                let _mirror_down_addr = addr & 0b00100000_00000111;
                todo!("PPU is not supported yet")
            }
            0x8000..=0xFFFF => {
                // ROM
                self.read_prg_rom(addr)
            }
            _ => {
                println!("Ignoring mem access at {}", addr);
                0
            }
        }
    }

    pub fn mem_write(&mut self, addr: u16, data: u8) {
        match addr {
            0x0000..=0x1FFF => {
                // RAM
                let mirror_down_addr = addr & 0b11111111111;
                self.cpu_vram[mirror_down_addr as usize] = data;
            }
            0x2000..=0x3FFF => {
                // PPU registers
                let _mirror_down_addr = addr & 0b00100000_00000111;
                todo!("PPU is not supported yet");
            }
            0x8000..=0xFFFF => {
                // ROM
                panic!("Attempt to write to Cartridge ROM space: {:x}", addr);
            }
            _ => {
                println!("Ignoring mem write-access at {}", addr);
            }
        }
    }
}


#[cfg(test)]
mod test {
    use super::*;
    use crate::cartridge::test;

    #[test]
    fn test_mem_read_write_to_ram() {
        let mut bus = Bus::new(test::test_rom());
        bus.mem_write(0x01, 0x55);
        assert_eq!(bus.mem_read(0x01), 0x55);
    }
}
