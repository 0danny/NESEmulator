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
		backgroundPatternTableAddr(0),
		scanline(0),
		cycle(0)
	{
		vram.fill(0);
		paletteTable.fill(0);
		screenBuffer.fill(0);
		oam.fill(0);
		frameComplete = false;

		// Example base NES palette (64 colors)
		std::array<uint8_t, 64> basePalette = {
			0x7C, 0x00, 0x10, 0x30, 0x70, 0x90, 0xA0, 0xB0,  // Example values
			0xC0, 0xD0, 0xE0, 0xF0, 0x1C, 0x2C, 0x3C, 0x4C,
			0x5C, 0x6C, 0x8C, 0x9C, 0xAC, 0xBC, 0xCC, 0xDC,
			0xEC, 0xFC, 0x0C, 0x1C, 0x2C, 0x3C, 0x4C, 0x5C,
			0x6C, 0x7C, 0x8C, 0x9C, 0xAC, 0xBC, 0xCC, 0xDC,
			0xEC, 0xFC, 0x0C, 0x1C, 0x2C, 0x3C, 0x4C, 0x5C,
			0x6C, 0x7C, 0x8C, 0x9C, 0xAC, 0xBC, 0xCC, 0xDC,
			0xEC, 0xFC, 0x0C, 0x1C, 0x2C, 0x3C, 0x4C, 0x5C
		};

		// Populate the full 256-entry palette by repeating the base palette
		for (size_t i = 0; i < palette.size(); ++i)
		{
			palette[i] = basePalette[i % 64];  // Repeat the base palette across all 256 entries
		}

		memoryBus.RegisterPPURegisterCallback([this](uint16_t address, uint8_t value, bool reading)
		{
			this->PPURegisterCallback(address, value, reading);
		});

		Utils::Logger::Info("[PPU]: Initialized...");
	}

	uint8_t PPU::ReadPPURegister(uint16_t addr) const
	{
		return memoryBus.Read(addr, false);
	}

	void PPU::WritePPURegister(uint16_t addr, uint8_t value)
	{
		memoryBus.Write(addr, value, false);
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

	void PPU::PPURegisterCallback(uint16_t address, uint16_t value, bool reading)
	{
		switch (address)
		{
		case PPUCTRL:
			// Update the base address of the background pattern table
			backgroundPatternTableAddr = (value & 0x10) ? 0x1000 : 0x0000;
			break;

		case PPUSCROLL:
			if (writeToggle == 0)
			{
				fineX = value & 0x07;
				tempAddr = (tempAddr & 0xFFE0) | (value >> 3);
			}
			else
			{
				tempAddr = (tempAddr & 0x8FFF) | ((value & 0x07) << 12);
				tempAddr = (tempAddr & 0xFC1F) | ((value & 0xF8) << 2);
			}
			writeToggle = !writeToggle;
			break;

		case PPUADDR:
			if (writeToggle == 0)
			{
				tempAddr = (tempAddr & 0x00FF) | ((value & 0x3F) << 8);
			}
			else
			{
				tempAddr = (tempAddr & 0xFF00) | value;
				vramAddr = tempAddr;
			}
			writeToggle = !writeToggle;
			break;

		case PPUDATA:
			WriteVRAM(vramAddr, value);
			IncrementVRAMAddr();
			break;
		}
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
		// Vertical blanking lines
		else if (scanline >= 241 && scanline <= 260)
		{
			if (scanline == 241 && cycle == 1)
			{
				//Utils::Logger::Info("ENTERING VBLANK");

				WritePPURegister(PPUSTATUS, ReadPPURegister(PPUSTATUS) | 0x80); // Set VBlank flag

				if (ReadPPURegister(PPUCTRL) & 0x80)
				{
					auto& cpu = Emulation::CPU::Instance();
					cpu.RequestNMI();

					Utils::Logger::Info("NMI Requested");
				}
			}
		}
		// Pre-render scanline
		else if (scanline == 261)
		{
			//Utils::Logger::Info("LEAVING VBLANK");

			if (cycle == 1)
			{
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
		if (ReadPPURegister(PPUCTRL) & 0x04)
		{
			vramAddr += 32;  // Increment by 32 if bit 2 of PPUCTRL is set
		}
		else
		{
			vramAddr += 1;   // Otherwise, increment by 1
		}
	}

	void PPU::RenderPixel()
	{
		uint16_t x = cycle - 1;    // Horizontal position (0-255)
		uint16_t y = scanline;     // Vertical position (0-239)

		if (x < 256 && y < 240)
		{
			// Calculate the coarse X and Y positions
			uint16_t coarseX = (vramAddr & 0x001F);  // bits 0-4
			uint16_t coarseY = (vramAddr & 0x03E0) >> 5; // bits 5-9

			// Calculate the fine Y position
			uint16_t fineY = (vramAddr & 0x7000) >> 12; // bits 12-14

			// Calculate the base nametable address (0x2000 or 0x2400)
			uint16_t nametableBase = 0x2000 | (vramAddr & 0x0C00);

			// Calculate the tile index from the nametable
			uint16_t tileIndex = ReadVRAM(nametableBase + (coarseY * 32) + coarseX);

			// Calculate the address of the tile in the pattern table
			uint16_t patternAddr = backgroundPatternTableAddr + (tileIndex * 16) + fineY;

			// Fetch the corresponding row of pixels from the pattern table
			uint8_t tileLow = ReadVRAM(patternAddr);
			uint8_t tileHigh = ReadVRAM(patternAddr + 8);

			// Calculate the pixel offset within the tile
			uint8_t pixelX = (x + fineX) % 8;

			// Determine the pixel color within the tile
			uint8_t pixel = ((tileLow >> (7 - pixelX)) & 1) | (((tileHigh >> (7 - pixelX)) & 1) << 1);

			// Fetch the attribute byte for this tile to determine its palette
			uint16_t attributeAddr = nametableBase + 0x03C0 + ((coarseY / 4) * 8) + (coarseX / 4);
			uint8_t attributeByte = ReadVRAM(attributeAddr);

			// Determine the quadrant within the attribute byte
			uint8_t quadrant = ((coarseY % 4) / 2) * 2 + ((coarseX % 4) / 2);
			uint8_t paletteIndex = (attributeByte >> (quadrant * 2)) & 0x03;

			// Fetch the final color from the palette
			uint32_t color = palette[(paletteIndex * 4) + pixel];

			// Write the color to the screen buffer
			screenBuffer[y * 256 + x] = color;
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