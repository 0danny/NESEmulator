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
		// Handle palette memory mirroring
		if (addr >= 0x3F00 && addr <= 0x3FFF)
		{
			addr = 0x3F00 + (addr % 32);  // Palette is mirrored every 32 bytes
			if (addr == 0x3F10) addr = 0x3F00; // Mirror $3F10 to $3F00
			if (addr == 0x3F14) addr = 0x3F04; // Mirror $3F14 to $3F04
			if (addr == 0x3F18) addr = 0x3F08; // Mirror $3F18 to $3F08
			if (addr == 0x3F1C) addr = 0x3F0C; // Mirror $3F1C to $3F0C
			return paletteTable[addr - 0x3F00]; // Palette address space starts at $3F00
		}

		return vram[addr % VRAM_SIZE];
	}

	void PPU::WriteVRAM(uint16_t addr, uint8_t value)
	{
		// Handle palette memory mirroring
		if (addr >= 0x3F00 && addr <= 0x3FFF)
		{
			addr = 0x3F00 + (addr % 32);  // Palette is mirrored every 32 bytes
			if (addr == 0x3F10) addr = 0x3F00; // Mirror $3F10 to $3F00
			if (addr == 0x3F14) addr = 0x3F04; // Mirror $3F14 to $3F04
			if (addr == 0x3F18) addr = 0x3F08; // Mirror $3F18 to $3F08
			if (addr == 0x3F1C) addr = 0x3F0C; // Mirror $3F1C to $3F0C
			paletteTable[addr - 0x3F00] = value; // Write to the palette table
			return;
		}

		vram[addr % VRAM_SIZE] = value;
	}

	void PPU::CopyOAM(uint8_t oamEntry, int index)
	{
		int oamSelect = index / 4;
		int property = index % 4;

		if (property == 0)
		{
			oam[oamSelect].yCoordinate = oamEntry;
		}
		else if (property == 1)
		{
			oam[oamSelect].tileIndex = oamEntry;
		}
		else if (property == 2)
		{
			oam[oamSelect].attributes = oamEntry;
		}
		else {
			oam[oamSelect].xCoordinate = oamEntry;
		}
	}

	uint8_t PPU::ReadOAM(int index)
	{
		int oamSelect = index / 4;
		int property = index % 4;

		if (property == 0)
		{
			return oam[oamSelect].yCoordinate;
		}
		else if (property == 1)
		{
			return oam[oamSelect].tileIndex;
		}
		else if (property == 2)
		{
			return oam[oamSelect].attributes;
		}
		else {
			return oam[oamSelect].xCoordinate;
		}
	}

	void PPU::PPUWriteCallback(uint16_t address, uint8_t value)
	{
		uint8_t reg = address % 8;

		switch (reg)
		{

		case 0: //PPUCTRL
			tempAddr = (tempAddr & 0xF3FF) | (((uint16_t)value & 0x03) << 10);
			spriteHeight = (value & 0x20) ? 16 : 8;
			ppuCtrl.val = value;
			break;

		case 1: //PPUMASK
			ppuMask.val = value;
			break;

		case 3: //OAMADDR
			oamAddr = value;
			break;

		case 4: //OAMDATA
			CopyOAM(value, oamAddr++);

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
			//Utils::Logger::Debug("CPU Writing to PPU ADDR [Scanline ", scanLine, ", Dot ", dot, "] (Vblank: ", ppuStatus.vBlank, ")");

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
			//Utils::Logger::Debug("CPU Writing to PPU DATA [Scanline ", scanLine, ", Dot ", dot, "] (Vblank: ", ppuStatus.vBlank, ")");

			WriteVRAM(vramAddr, value);
			vramAddr += ppuCtrl.vramAddressIncrement ? 32 : 1;
			break;

		default:
			Utils::Logger::Info("CPU Writing to invalid PPU register - ", Utils::Logger::Uint8ToHex(reg));
			break;
		}
	}

	void PPU::FetchSpritePatterns()
	{
		if (IsRenderingDisabled())
		{
			return;
		}

		for (auto& sprite : spriteRenderEntities)
		{
			if (sprite.counter > 0)
			{
				sprite.counter--;
			}
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

		case 2: { //PPUSTATUS
			//Utils::Logger::Debug("We are saying to the CPU (Vblank: ", ppuStatus.vBlank, ")", " [Scanline ", scanLine, ", Dot ", dot, "]");
			uint8_t copy = ppuStatus.val;

			ppuStatus.vBlank = 0;
			writeToggle = 0;

			return copy;
		}

		case 3: //OAMADDR
			return oamAddr;

		case 4: //OAMDATA
			return ReadOAM(oamAddr);

		case 7: { //PPUDATA
			// Store the value in the buffer from the previous read (simulating the delayed read behavior of the PPU)
			ppuReadBuffer = ppuReadBufferCpy;

			// If the current VRAM address points to the palette memory range (0x3F00 - 0x3FFF)
			if (vramAddr >= 0x3F00 && vramAddr <= 0x3FFF)
			{
				ppuReadBufferCpy = ReadVRAM(vramAddr - 0x1000); // Mirror to the name table space

				// Apply the greyscale filter if the corresponding bit in ppumask is set
				ppuReadBuffer = ppuMask.greyScale ? (ReadVRAM(vramAddr) & 0x30) : ReadVRAM(vramAddr);
			}
			else
			{
				// For non-palette memory, store the current VRAM value in the secondary buffer
				ppuReadBufferCpy = ReadVRAM(vramAddr);
			}

			// Increment the VRAM address according to the increment mode set in PPUCTRL
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
				EvaluateSprites();
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
							FetchSpritePatterns();
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

	void PPU::EvaluateSprites()
	{
		//clear secondary OAM
		if (dot >= 1 && dot <= 64) {
			if (dot == 1) {
				secondaryOAMCursor = 0;
			}

			secondaryOAM[secondaryOAMCursor].attributes = 0xFF;
			secondaryOAM[secondaryOAMCursor].tileIndex = 0xFF;
			secondaryOAM[secondaryOAMCursor].xCoordinate = 0xFF;
			secondaryOAM[secondaryOAMCursor].yCoordinate = 0xFF;

			if (dot % 8 == 0) {
				secondaryOAMCursor++;
			}
		}

		//sprite eval
		if (dot >= 65 && dot <= 256) {
			//Init
			if (dot == 65) {
				secondaryOAMCursor = 0;
				primaryOAMCursor = 0;
			}

			if (secondaryOAMCursor == 8) {
				//ppustatus |= 0x20;
				return;
			}

			if (primaryOAMCursor == 64) {
				return;
			}

			//odd cycle read
			if ((dot % 2) == 1) {
				tmpOAM = oam[primaryOAMCursor];

				if (InYRange(tmpOAM))
				{
					inRangeCycles--;
					inRange = true;
				}
				//even cycle write
			}
			else {
				//tmpOAM is in range, write it to secondaryOAM
				if (inRange) {
					inRangeCycles--;

					//copying tmpOAM in range is 8 cycles, 2 cycles otherwise
					if (inRangeCycles == 0) {
						primaryOAMCursor++;
						secondaryOAMCursor++;
						inRangeCycles = 8;
						inRange = false;
					}
					else {
						tmpOAM.id = primaryOAMCursor;
						secondaryOAM[secondaryOAMCursor] = tmpOAM;
					}
				}
				else {
					primaryOAMCursor++;
				}
			}
		}

		//Sprite fetches
		if (dot >= 257 && dot <= 320) {
			if (dot == 257) {
				secondaryOAMCursor = 0;
				spriteRenderEntities.clear();
			}

			SpriteData sprite = secondaryOAM[secondaryOAMCursor];

			int cycle = (dot - 1) % 8;

			switch (cycle)
			{
			case 0:
			case 1:
				if (!IsUninit(sprite)) {
					out = SpriteRenderEntity();
				}

				break;

			case 2:
				if (!IsUninit(sprite)) {
					out.attr = sprite.attributes;
					out.flipHorizontally = sprite.attributes & 64;
					out.flipVertically = sprite.attributes & 128;
					out.id = sprite.id;
				}
				break;

			case 3:
				if (!IsUninit(sprite)) {
					out.counter = sprite.xCoordinate;
				}
				break;

			case 4:
				if (!IsUninit(sprite)) {
					spritePatternLowAddr = GetSpritePatternAddress(sprite, out.flipVertically);
					out.lo = ReadVRAM(spritePatternLowAddr);
				}
				break;

			case 5:
				break;

			case 6:
				if (!IsUninit(sprite)) {
					spritePatternHighAddr = spritePatternLowAddr + 8;
					out.hi = ReadVRAM(spritePatternHighAddr);
				}
				break;

			case 7:
				if (!IsUninit(sprite)) {
					spriteRenderEntities.push_back(out);
				}

				secondaryOAMCursor++;
				break;

			default:
				break;
			}
		}
	}

	uint16_t PPU::GetSpritePatternAddress(const SpriteData& sprite, bool flipVertically)
	{
		uint16_t addr = 0;

		int fineOffset = scanLine - sprite.yCoordinate;

		if (flipVertically) {
			fineOffset = spriteHeight - 1 - fineOffset;
		}

		//By adding 8 to fineOffset we skip the high order bits
		if (spriteHeight == 16 && fineOffset >= 8) {
			fineOffset += 8;
		}

		if (spriteHeight == 8) {
			addr = ((uint16_t)ppuCtrl.spritePatternTableAddress << 12) |
				((uint16_t)sprite.tileIndex << 4) |
				fineOffset;
		}
		else {
			addr = (((uint16_t)sprite.tileIndex & 1) << 12) |
				((uint16_t)((sprite.tileIndex & ~1) << 4)) |
				fineOffset;
		}


		return addr;
	}

	bool PPU::IsUninit(const SpriteData& sprite)
	{
		return ((sprite.attributes == 0xFF) && (sprite.tileIndex == 0xFF) && (sprite.xCoordinate == 0xFF) && (sprite.yCoordinate == 0xFF)) || ((sprite.xCoordinate == 0) && (sprite.yCoordinate == 0) && (sprite.attributes == 0) && (sprite.tileIndex == 0));
	}

	bool PPU::InYRange(const SpriteData& oam) {
		return !IsUninit(oam) && ((scanLine >= oam.yCoordinate) && (scanLine < (oam.yCoordinate + spriteHeight)));
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

		//Sprites
		uint8_t spritePixel1 = 0;
		uint8_t spritePixel2 = 0;
		uint8_t spritePixel3 = 0;
		uint8_t spritePixel4 = 0;
		uint8_t spriteBit12 = 0;
		uint8_t paletteIndex = 0 | (pixel4 >> 12) | (pixel3 >> 13) | (pixel2 >> 14) | (pixel1 >> 15);
		uint8_t spritePaletteIndex = 0;
		bool showSprite = false;
		bool spriteFound = false;

		for (auto& sprite : spriteRenderEntities)
		{
			if (sprite.counter == 0 && sprite.shifted != 8) {
				if (spriteFound) {
					sprite.shift();
					continue;
				}

				spritePixel1 = sprite.flipHorizontally ? ((sprite.lo & 1) << 7) : sprite.lo & 128;
				spritePixel2 = sprite.flipHorizontally ? ((sprite.hi & 1) << 7) : sprite.hi & 128;
				spritePixel3 = sprite.attr & 1;
				spritePixel4 = sprite.attr & 2;
				spriteBit12 = (spritePixel2 >> 6) | (spritePixel1 >> 7);

				//Sprite zero hit
				if (!ppuStatus.spriteZeroHit && spriteBit12 && bgBit12 && sprite.id == 0 && ppuMask.showSprites && ppuMask.showBg && dot < 256)
				{
					ppuStatus.val |= 64;
				}

				if (spriteBit12) {
					showSprite = ((bgBit12 && !(sprite.attr & 32)) || !bgBit12) && ppuMask.showSprites;
					spritePaletteIndex = 0x10 | (spritePixel4 << 2) | (spritePixel3 << 2) | spriteBit12;
					spriteFound = true;
				}

				sprite.shift();
			}
		}

		//When bg rendering is off
		if (!ppuMask.showBg)
		{
			paletteIndex = 0;
		}

		uint8_t pindex = ReadVRAM(0x3F00 | (showSprite ? spritePaletteIndex : paletteIndex)) % 64;
		//Handling grayscale mode
		uint8_t p = ppuMask.greyScale ? (pindex & 0x30) : pindex;



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
			return;

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