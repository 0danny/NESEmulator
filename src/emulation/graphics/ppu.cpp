#include "ppu.h"

namespace Emulation::Graphics
{
	PPU::PPU() : memoryBus(MemoryBus::Instance())
	{
		//Arrays
		vram.fill(0);
		screenBuffer.fill(0);
		primaryOam.fill(0);

		memoryBus.RegisterPPUReadCallback([this](uint16_t address) -> uint8_t
			{
				return this->PPUReadCallback(address);
			});

		memoryBus.RegisterPPUWriteCallback([this](uint16_t address, uint8_t value)
			{
				this->PPUWriteCallback(address, value);
			});

		Utils::Logger::Info("[PPU]: Initialized...");
	}

	uint8_t PPU::ReadVRAM(uint16_t addr) const
	{
		// Mirror nametables: addresses $3000 - $3EFF mirror $2000 - $2EFF
		if (addr >= 0x3000 && addr < 0x3F00)
		{
			addr -= 0x1000;
		}

		// Palette mirroring: addresses $3F20, $3F40, etc., are mirrors of $3F00 - $3F1F
		if (addr >= 0x3F00 && addr <= 0x3FFF)
		{
			addr = 0x3F00 + (addr % 0x20);
		}

		return vram[addr % VRAM_SIZE];
	}

	void PPU::WriteVRAM(uint16_t addr, uint8_t value)
	{
		// Mirror nametables: addresses $3000 - $3EFF mirror $2000 - $2EFF
		if (addr >= 0x3000 && addr < 0x3F00)
		{
			addr -= 0x1000;
		}

		// Palette mirroring: addresses $3F20, $3F40, etc., are mirrors of $3F00 - $3F1F
		if (addr >= 0x3F00 && addr <= 0x3FFF)
		{
			addr = 0x3F00 + (addr % 0x20);
		}

		vram[addr % VRAM_SIZE] = value;
	}

	inline void PPU::PreRender()
	{
		if (dot == 0)
		{
			frameComplete = true;
		}

		if (dot == 1)
		{
			// Clear the vBlank flag and sprite zero hit
			ppuStatus.vBlank = 0;
			ppuStatus.spriteZeroHit = 0;
			ppuStatus.spriteOverflow = 0;  // Clear sprite overflow as well
		}

		if (dot >= 280 && dot <= 304)
		{
			//Vertical Scroll Bits Reloaded
		}

		if (dot == 341)
		{
			scanLine = 0;
		}
	}

	inline void PPU::Rendering()
	{
		//Visible scanlines
		if (dot >= 1 && dot <= 256)
		{
			FetchTiles();
			EmitPixel();
		}

		if (dot >= 257 && dot <= 320)
		{
			//FetchSpriteData();
		}

		if (dot >= 321 && dot <= 336)
		{
			//LoadShiftRegisters();
		}
	}

	inline void PPU::Vblank()
	{
		if (dot == 1 && scanLine == 241)
		{
			ppuStatus.vBlank = 1;
			
			if (ppuCtrl.nmiEnable)
			{
				triggeredNMI = true;
			}
		}
	}

	void PPU::FetchTiles()
	{
		
	}

	void PPU::EmitPixel()
	{
		
	}


	void PPU::Clock()
	{
		//Utils::Logger::Debug("Scanline -> ", scanLine, " Dots -> ", dot);

		if (scanLine == 261)
		{
			PreRender();
		}

		if (scanLine >= 0 && scanLine <= 239)
		{
			Rendering();
		}

		if (scanLine == 240)
		{
			//Idling.
		}

		//VBlank
		if (scanLine >= 241 && scanLine <= 260)
		{
			Vblank();
		}

		// Each scanline lasts for 341 ppu clock cycles.
		if (dot == 341)
		{
			scanLine += 1;
			dot = 0;
		}
		else
			dot++;
	}

	void PPU::PPUWriteCallback(uint16_t address, uint8_t value)
	{
		uint8_t reg = address % 8;

		switch (reg)
		{

		case 0: //PPUCTRL

			ppuCtrl.val = value;
			break;

		case 1: //PPUMASK
			ppuMask.val = value;
			break;

		case 3: //OAMADDR 
			oamAddr = value;
			break;

		case 4: { //OAMDATA
			primaryOam[oamAddr] = value;

			//Increment OAM after write.
			oamAddr += 1;

			break;
		}

		case 5: //PPUSCROLL
			if (writeToggle)
			{
				//First Write (Horizontal Scroll Position)

				fineX = (value & 0x07);  // Store fine X (3 bits from the lower part of the data)
				tempAddr = (tempAddr & 0x7FE0) | ((value >> 3) & 0x1F);  // Coarse X (5 bits)

				writeToggle = 0;
			}
			else
			{
				//Second Write (Vertical Scroll Position)
				//Bits 12 - 14: Fine Y (3 bits)
				//Bits 5 - 9: Coarse Y (5 bits)

				tempAddr = (tempAddr & 0xC1F) | ((value & 0x07) << 12) | ((value >> 3) & 0x03E0);  // Fine Y, Coarse Y

				writeToggle = 1;
			}

			break;

		case 6: //PPUADDR
			if (address == 0x4014)
			{
				Utils::Logger::Info("Initiating a OAM DMA write to page -> ", Utils::Logger::Uint8ToHex(value));

				//OAM DMA (Direct Memory Access)
				uint16_t baseAddr = value * 0x100;  // Base address for DMA transfer in CPU memory

				// Perform the DMA transfer (copy 256 bytes from CPU memory to OAM)
				for (int i = 0; i < 256; ++i) 
				{
					primaryOam[oamAddr] = memoryBus.Read(baseAddr + i);  // Transfer data to OAM
					oamAddr++;  // Increment OAM address
				}

				//We are meant to be stalling the CPU here.

				break;
			}

			if (writeToggle)
			{
				//Upper byte first.
				tempAddr = (tempAddr & 0x00FF) | ((value & 0x3F) << 8);  // Masking to ensure only 6 bits are used for the upper byte

				writeToggle = 0;
			}
			else
			{
				//Lower byte second.
				tempAddr = (tempAddr & 0xFF00) | value;  // Fill the lower byte
				vramAddr = tempAddr;  // After the second write, the address is updated

				writeToggle = 1;
			}
			break;

		case 7:  //PPUDATA
			WriteVRAM(vramAddr, value);

			IncrementVRAMAddr();
			break;

		default:
			Utils::Logger::Info("CPU Writing to invalid PPU register - ", Utils::Logger::Uint8ToHex(reg));
			break;
		}
	}

	uint8_t PPU::PPUReadCallback(uint16_t address)
	{
		uint8_t reg = address % 8;

		switch (reg)
		{

		case 2: { //PPUSTATUS
			uint8_t ppuStatusCopy = ppuStatus.val;

			//VBlank is cleared on PPUSTATUS read.
			ppuStatus.vBlank = 0;

			writeToggle = 0;

			return ppuStatusCopy;
		}

		case 4: { //OAMDATA
			return primaryOam[oamAddr];
		}

		case 7: { //PPUDATA
			uint8_t returnValue = readBuffer;

			// Palette memory is handled immediately, no buffer delay
			if (vramAddr >= 0x3F00 && vramAddr <= 0x3FFF) 
			{
				returnValue = ReadVRAM(vramAddr);
			}
			else {
				// Return the buffered value, then update the buffer with current VRAM data
				readBuffer = ReadVRAM(vramAddr);
			}

			// Increment the VRAM address based on PPUCTRL's increment mode
			IncrementVRAMAddr();

			return returnValue;
		}

		default:
			Utils::Logger::Info("CPU Reading from invalid PPU register - ", Utils::Logger::Uint8ToHex(reg));
		}
	}

	void PPU::IncrementVRAMAddr()
	{
		if (ppuCtrl.vramIncrementMode == 0)
			//Going across
			vramAddr += 1;
		else
			//Going down
			vramAddr += 32;
	}

	bool PPU::ForcedBlanking()
	{
		return !ppuMask.showBg && !ppuMask.showSprites;
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

	const uint32_t* PPU::GetScreenBuffer() const
	{
		return screenBuffer.data();
	}
}