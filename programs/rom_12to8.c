#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned char UINT8;

static void Convert12BitTo16Bit(size_t smplCnt, UINT8* data8, const UINT8* data12);

int main(int argc, char* argv[])
{
	size_t romSizeF;
	size_t curPos;
	UINT8 smplMode;
	size_t smplOfs;
	size_t smplCnt;
	UINT8* romFused;
	UINT8* romSmpl;
	FILE* hFile;
	
	if (argc < 3)
	{
		printf("Usage: %s in12.bin out8.bin\n", argv[0]);
		return 0;
	}
	
	hFile = fopen(argv[1], "rb");
	if (hFile == NULL)
	{
		printf("Error opening file %s!\n", argv[1]);
		return 1;
	}
	fseek(hFile, 0, SEEK_END);
	romSizeF = ftell(hFile);
	rewind(hFile);
	romFused = (UINT8*)malloc(romSizeF);
	fread(romFused, 1, romSizeF, hFile);
	fclose(hFile);
	
	romSmpl = (UINT8*)malloc(romSizeF);
	memset(romSmpl, 0x00, romSizeF);
	for (curPos = 0x00; curPos < 0x1800; curPos += 0x0C)
	{
		memcpy(&romSmpl[curPos], &romFused[curPos], 0x0C);
		smplMode = romFused[curPos + 0x00] & 0xC0;
		smplOfs =	(romFused[curPos + 0x00] << 16) |
					(romFused[curPos + 0x01] <<  8) |
					(romFused[curPos + 0x02] <<  0);
		smplOfs &= 0x3FFFFF;
		smplCnt =	(romFused[curPos + 0x05] <<  8) |
					(romFused[curPos + 0x06] <<  0);
		if (smplOfs < 0x1800 || ! smplCnt)
			continue;
		smplCnt ^= 0xFFFF;
		
		if (smplMode & 0x40)
		{
			Convert12BitTo16Bit(smplCnt, &romSmpl[smplOfs], &romFused[smplOfs]);
			romSmpl[curPos + 0x00] = (romFused[curPos + 0x00] & 0x3F) | 0x00;
		}
	}
	
	hFile = fopen(argv[2], "wb");
	if (hFile == NULL)
	{
		printf("Error opening file %s!\n", argv[2]);
		return 1;
	}
	fwrite(romSmpl, 1, romSizeF, hFile);
	fclose(hFile);
	
	free(romFused);	romFused = NULL;
	free(romSmpl);	romSmpl = NULL;
	
	return 0;
}

static void Convert12BitTo16Bit(size_t smplCnt, UINT8* data8, const UINT8* data12)
{
	size_t curSmpl8;
	size_t curPos12;
	
	for (curSmpl8 = 0, curPos12 = 0; curSmpl8 < smplCnt; curSmpl8 += 2, curPos12 += 3)
	{
		data8[curSmpl8 + 0] = data12[curPos12 + 0];
		data8[curSmpl8 + 1] = data12[curPos12 + 2];
	}
	//for (; curSmpl8 < curPos12; curSmpl8 ++)
	//	data8[curSmpl8] = 0x00;
	
	return;
}
