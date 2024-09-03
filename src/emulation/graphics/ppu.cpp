#include "ppu.h"

namespace Emulation::Graphics
{
	PPU::PPU() :
		memoryBus(MemoryBus::Instance()),
		vramAddr(0),
		tempAddr(0),
		fineX(0),
		writeToggle(0),
		scanLine(0),
		dot(0),
		attrShiftReg1(0),
		attrShiftReg2(0),
		attrbyte(0),
		bgShiftRegHi(0),
		bgShiftRegLo(0),
		ntbyte(0),
		patternhigh(0),
		patternlow(0)
	{
		vram.fill(0);
		paletteTable.fill(0);
		screenBuffer.fill(0);
		oam.fill(0);
		frameComplete = false;

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
		return vram[addr % VRAM_SIZE];
	}

	void PPU::WriteVRAM(uint16_t addr, uint8_t value)
	{
		vram[addr % VRAM_SIZE] = value;
	}

	void PPU::PPUWriteCallback(uint16_t address, uint8_t value)
	{
		uint8_t reg = address % 8;

		switch (reg)
		{

		case 0: //PPUCTRL
			tempAddr = (tempAddr & 0xF3FF) | (((uint16_t)value & 0x03) << 10);
			//spriteHeight = (data & 0x20) ? 16 : 8;
			ppuCtrl.val = value;
			break;

		case 1: //PPUMASK
			ppuMask.val = value;
			break;

		case 3: //OAMADDR
			oamAddr = value;
			break;

		case 4: //OAMDATA
			break;

		case 5: //PPUSCROLL
			if (writeToggle == 0)
			{
				tempAddr &= 0x7FE0;
				tempAddr |= ((uint16_t)value) >> 3;
				fineX = value & 7;
				writeToggle = 1;
			}
			else
			{
				tempAddr &= ~0x73E0;
				tempAddr |= ((uint16_t)value & 0x07) << 12;
				tempAddr |= ((uint16_t)value & 0xF8) << 2;
				writeToggle = 0;
			}

			ppuScroll = value;
			break;

		case 6: //PPUADDR
			Utils::Logger::Debug("CPU Writing to PPU ADDR [Scanline ", scanLine, ", Dot ", dot, "] (Vblank: ", ppuStatus.vBlank, ")");


			if (writeToggle == 0)
			{
				tempAddr &= 255;
				tempAddr |= ((uint16_t)value & 0x3F) << 8;
				writeToggle = 1;
			}
			else
			{
				tempAddr &= 0xFF00;
				tempAddr |= value;
				vramAddr = tempAddr;

				writeToggle = 0;
			}
			break;

		case 7:  //PPUDATA
			Utils::Logger::Debug("CPU Writing to PPU DATA [Scanline ", scanLine, ", Dot ", dot, "] (Vblank: ", ppuStatus.vBlank, ")");

			WriteVRAM(vramAddr, value);
			vramAddr += ppuCtrl.vramAddressIncrement ? 32 : 1;
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

		case 0: //PPUCTRL
			return ppuCtrl.val;

		case 1: //PPUMASK
			return ppuMask.val;

		case 2: //PPUSTATUS
			//Utils::Logger::Debug("We are saying to the CPU (Vblank: ", ppuStatus.vBlank, ")", " [Scanline ", scanLine, ", Dot ", dot, "]");
			return ppuStatus.val;

		case 3: //OAMADDR
			return oamAddr;

		case 4: //OAMDATA

			break;
		case 7: { //PPUDATA
			// Store the value in the buffer from the previous read (simulating the delayed read behavior of the PPU)
			ppuReadBuffer = ppuReadBufferCpy;

			// If the current VRAM address points to the palette memory range (0x3F00 - 0x3FFF)
			if (vramAddr >= 0x3F00 && vramAddr <= 0x3FFF)
			{
				// Pre-fetch the mirrored value from VRAM and store it in the secondary buffer
				ppuReadBufferCpy = ReadVRAM(vramAddr - 0x1000);

				// Apply the greyscale filter if the corresponding bit in ppumask is set
				ppuReadBuffer = ppuMask.greyScale ? (ReadVRAM(vramAddr) & 0x30) : ReadVRAM(vramAddr);
			}
			else
			{
				// For non-palette memory, store the current VRAM value in the secondary buffer
				ppuReadBufferCpy = ReadVRAM(vramAddr);
			}

			vramAddr += ppuCtrl.vramAddressIncrement ? 32 : 1;
			return ppuReadBuffer;
		}

		default:
			Utils::Logger::Info("CPU Reading from invalid PPU register - ", Utils::Logger::Uint8ToHex(reg));
			break;
		}
	}

	bool PPU::IsRenderingDisabled() 
	{
		return !ppuMask.showBg && !ppuMask.showSprites;
	}

	void PPU::CopyHorizontalBits() 
	{
		if (IsRenderingDisabled()) 
			return;

		vramAddr = (vramAddr & ~0x41F) | (tempAddr & 0x41F);
	}

	void PPU::CopyVerticalBits() 
	{
		if (IsRenderingDisabled()) 
			return;

		vramAddr = (vramAddr & ~0x7BE0) | (tempAddr & 0x7BE0);
	}
	
	//scanLine == 261
	inline void PPU::PreRender()
	{
		//clear vbl flag and sprite overflow
		if (dot == 2)
		{
			pixelIndex = 0;

			ppuStatus.vBlank = 0;
			ppuStatus.spriteOverflow = 0;
			ppuStatus.spriteZeroHit = 0;
		}

		//copy vertical bits
		if (dot >= 280 && dot <= 304)
		{
			CopyVerticalBits();
		}
	}

	//scanLine >= 240 && scanLine <= 260
	inline void PPU::PostRender()
	{
		//post-render, vblank
		if (scanLine == 240 && dot == 0)
		{
			frameComplete = true;
		}

		if (scanLine == 241 && dot == 1)
		{
			//set vbl flag
			ppuStatus.vBlank = 1;

			//flag for nmi
			if (ppuCtrl.generateNMI)
			{
				triggeredNMI = true;
			}
		}
	}

	void PPU::Clock()
	{
		if ((scanLine >= 0 && scanLine <= 239) || scanLine == 261) 
		{  
			//visible scanline, pre-render scanline
			if (scanLine == 261) 
			{
				PreRender();
			}

			if (scanLine >= 0 && scanLine <= 239) 
			{
				//evalSprites();
			}

			if (dot == 257) 
			{
				CopyHorizontalBits();
			}

			//main hook: fetch tiles, emit pixel, shift
			if ((dot >= 1 && dot <= 257) || (dot >= 321 && dot <= 337)) 
			{
				//reload shift registers and shift
				if ((dot >= 2 && dot <= 257) || (dot >= 322 && dot <= 337)) 
				{
					ReloadShiftersAndShift();
				}

				if (scanLine >= 0 && scanLine <= 239) 
				{
					if (dot >= 2 && dot <= 257) 
					{
						if (scanLine > 0) 
						{
							//decrementSpriteCounters();
						}

						EmitPixel();
					}
				}

				//fetch nt, at, pattern low - high
				FetchTiles();
			}
		}
		else if (scanLine >= 240 && scanLine <= 260) 
			PostRender();


		if (dot == 340)
		{
			scanLine = (scanLine + 1) % 262;
			dot = 0;
		}
		else 
			dot++;

		//Utils::Logger::Info("Scanline - ", scanline, " | Cycle - ", cycle);
	}

	void PPU::EmitPixel() 
	{
		if (IsRenderingDisabled()) 
		{
			pixelIndex++;
			return;
		}

		//Bg
		uint16_t fineSelect = 0x8000 >> fineX;
		uint16_t pixel1 = (bgShiftRegLo & fineSelect) << fineX;
		uint16_t pixel2 = (bgShiftRegHi & fineSelect) << fineX;
		uint16_t pixel3 = (attrShiftReg1 & fineSelect) << fineX;
		uint16_t pixel4 = (attrShiftReg2 & fineSelect) << fineX;
		uint8_t bgBit12 = (pixel2 >> 14) | (pixel1 >> 15);

		uint8_t paletteIndex = 0 | (pixel4 >> 12) | (pixel3 >> 13) | (pixel2 >> 14) | (pixel1 >> 15);

		if (!ppuMask.showBg) 
		{
			paletteIndex = 0;
		}

		uint8_t pindex = ReadVRAM(0x3F00 | paletteIndex) % 64;

		//Handling grayscale mode
		uint8_t p = ppuMask.greyScale ? (pindex & 0x30) : pindex;

		//Dark border rect to hide seam of scroll, and other glitches that may occur
		if (dot <= 9 || dot >= 249 || scanLine <= 7 || scanLine >= 232) 
		{
			p = 13;
		}

		screenBuffer[pixelIndex++] = palette[p];
	}

	void PPU::FetchTiles() 
	{
		if (IsRenderingDisabled()) 
			return;

		int cycle = dot % 8;

		//Fetch nametable byte
		if (cycle == 1)
		{
			ntbyte = ReadVRAM(0x2000 | (vramAddr & 0x0FFF));
			//Fetch attribute byte, also calculate which quadrant of the attribute byte is active
		}
		else if (cycle == 3) 
		{
			attrbyte = ReadVRAM(0x23C0 | (vramAddr & 0x0C00) | ((vramAddr >> 4) & 0x38) | ((vramAddr >> 2) & 0x07));
			quadrant_num = (((vramAddr & 2) >> 1) | ((vramAddr & 64) >> 5)) * 2;
			//Get low order bits of background tile
		}
		else if (cycle == 5) 
		{
			uint16_t patterAddr =
				((uint16_t)ppuCtrl.bgPatternTableAddress << 12) +
				((uint16_t)ntbyte << 4) +
				((vramAddr & 0x7000) >> 12);

			patternlow = ReadVRAM(patterAddr);
			//Get high order bits of background tile
		}
		else if (cycle == 7) 
		{
			uint16_t patterAddr =
				((uint16_t)ppuCtrl.bgPatternTableAddress << 12) +
				((uint16_t)ntbyte << 4) +
				((vramAddr & 0x7000) >> 12) + 8;

			patternhigh = ReadVRAM(patterAddr);
			//Change columns, change rows
		}
		else if (cycle == 0) 
		{
			if (dot == 256) 
			{
				YIncrement();
			}

			XIncrement();
		}
	}

	void PPU::XIncrement() 
	{
		if ((vramAddr & 0x001F) == 31) 
		{
			vramAddr &= ~0x001F;
			vramAddr ^= 0x0400;
		}
		else 
		{
			vramAddr += 1;
		}
	}

	void PPU::YIncrement() 
	{
		if ((vramAddr & 0x7000) != 0x7000) 
		{
			vramAddr += 0x1000;
		}
		else 
		{
			vramAddr &= ~0x7000;
			int y = (vramAddr & 0x03E0) >> 5;

			if (y == 29) 
			{
				y = 0;
				vramAddr ^= 0x0800;
			}
			else if (y == 31) 
			{
				y = 0;
			}
			else 
			{
				y += 1;
			}

			vramAddr = (vramAddr & ~0x03E0) | (y << 5);
		}
	}

	void PPU::ReloadShiftersAndShift() 
	{
		if (IsRenderingDisabled()) 
		{
			return;
		}

		bgShiftRegLo <<= 1;
		bgShiftRegHi <<= 1;
		attrShiftReg1 <<= 1;
		attrShiftReg2 <<= 1;

		if (dot % 8 == 1) 
		{
			uint8_t attr_bits1 = (attrbyte >> quadrant_num) & 1;
			uint8_t attr_bits2 = (attrbyte >> quadrant_num) & 2;
			attrShiftReg1 |= attr_bits1 ? 255 : 0;
			attrShiftReg2 |= attr_bits2 ? 255 : 0;
			bgShiftRegLo |= patternlow;
			bgShiftRegHi |= patternhigh;
		}
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