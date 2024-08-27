#include "ppu.h"

namespace Emulation::Graphics
{
    PPU::PPU() :
        memoryBus(MemoryBus::Instance()),
        vramAddr(0),
        tempAddr(0),
        fineX(0),
        writeToggle(0),
        buffer(0),
        scanline(0),
        cycle(0)
    {
        vram.fill(0);
        palette.fill(0);
        paletteTable.fill(0);
        screenBuffer.fill(0);
        oam.fill(0);
        frameComplete = false;

        Utils::Logger::Info("[PPU]: Initialized...");
    }

    uint8_t PPU::ReadPPURegister(uint16_t addr) const
    {
        return memoryBus.Read(addr);
    }

    void PPU::WritePPURegister(uint16_t addr, uint8_t value)
    {
        memoryBus.Write(addr, value);
    }

    uint8_t PPU::ReadVRAM(uint16_t addr) const
    {
        addr %= VRAM_SIZE;
        return vram[addr];
    }

    void PPU::WriteVRAM(uint16_t addr, uint8_t value)
    {
        addr %= VRAM_SIZE;
        vram[addr] = value;
    }

    void PPU::Clock()
    {
        // Increment cycle
        cycle++;

        // Check if we've reached the end of a scanline
        if (cycle > 340)
        {
            cycle = 0;
            scanline++;

            // Check if we've reached the end of the frame
            if (scanline > 261)
            {
                scanline = 0;
                frameComplete = true;
            }
        }

        // Visible scanlines
        if (scanline >= 0 && scanline < 240)
        {
            if (cycle >= 1 && cycle <= 256)
            {
                RenderPixel();
            }
        }
        // Post-render scanline
        else if (scanline == 240)
        {
            // Do nothing
        }
        // Vertical blanking lines
        else if (scanline >= 241 && scanline <= 260)
        {
            if (scanline == 241 && cycle == 1)
            {
                Utils::Logger::Info("ENTERING VBLANK");
                WritePPURegister(PPUSTATUS, ReadPPURegister(PPUSTATUS) | 0x80); // Set VBlank flag

                if (ReadPPURegister(PPUCTRL) & 0x80)
                {
                    auto& cpu = Emulation::CPU::Instance();
                    Utils::Logger::Info("Triggering NMI");
                    cpu.RequestNMI();
                }
            }
        }
        // Pre-render scanline
        else if (scanline == 261)
        {
            if (cycle == 1)
            {
                Utils::Logger::Info("EXITING VBLANK");
                WritePPURegister(PPUSTATUS, ReadPPURegister(PPUSTATUS) & ~0x80); // Clear VBlank flag
            }
        }

        // Utils::Logger::Info("Scanline - ", scanline, " | Cycle - ", cycle);
    }

	void PPU::LoadCHRProgram(const std::vector<uint8_t>& chrRom)
	{
		Utils::Logger::Info("[PPU]: Loading CHR data into VRAM...");

		if (chrRom.size() > VRAM_SIZE)
		{
			Utils::Logger::Error("[PPU]: CHR ROM size exceeds VRAM capacity.");
			return;
		}

		// Copy the CHR ROM data into VRAM
		std::copy(chrRom.begin(), chrRom.end(), vram.begin());

		Utils::Logger::Info("[PPU]: CHR ROM data loaded into VRAM.");
	}

	void PPU::IncrementVRAMAddr()
	{
		if (ReadPPURegister(PPUSTATUS) & 0x04)
		{
			// Increment by 32 if the flag is set
			vramAddr += 32;
		}
		else
		{
			// Increment by 1 otherwise
			vramAddr += 1;
		}
	}

    void PPU::RenderPixel()
    {
        uint16_t x = cycle - 1;
        uint16_t y = scanline;

        if (x < 256 && y < 240)
        {
            uint8_t tileX = x / 8;
            uint8_t tileY = y / 8;
            uint16_t nametableAddr = 0x2000 | (vramAddr & 0x0FFF);
            uint8_t tileIndex = ReadVRAM(nametableAddr + tileY * 32 + tileX);

            uint16_t patternAddr = ((ReadPPURegister(PPUCTRL) & 0x10) ? 0x1000 : 0) + tileIndex * 16;
            uint8_t row = y % 8;
            uint8_t tileLow = ReadVRAM(patternAddr + row);
            uint8_t tileHigh = ReadVRAM(patternAddr + row + 8);

            uint8_t col = 7 - (x % 8);
            uint8_t pixel = ((tileLow >> col) & 1) | (((tileHigh >> col) & 1) << 1);

            // For simplicity, we'll use a grayscale palette
            uint32_t color = pixel * 85; // 0, 85, 170, or 255
            screenBuffer[y * 256 + x] = (color << 16) | (color << 8) | color;
        }
    }

	void PPU::HandleSpriteEvaluation()
	{
		
	}

    bool PPU::IsFrameComplete()
    {
        if (frameComplete)
        {
            frameComplete = false;
            return true;
        }
        return false;
    }


    const uint32_t* PPU::GetScreenBuffer() const
    {
        return screenBuffer.data();
    }

	uint8_t PPU::FetchBackgroundPixel(uint8_t x, uint8_t y)
	{
		return 0;
	}

	uint8_t PPU::FetchSpritePixel(uint8_t x, uint8_t y)
	{
		return 0;
	}
}