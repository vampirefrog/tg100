#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef unsigned int UINT32;

static const UINT8 VGM_HEADER[0xC0] =
{
	0x56,0x67,0x6D,0x20, 0xAA,0xAA,0xAA,0xAA, 0x61,0x01,0x00,0x00, 0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0xAA,0xAA,0xAA,0xAA, 0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00, 0x8C,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0xF0,0x41,0x73,0x00, 0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
};


int main(int argc, char* argv[])
{
	FILE* hFileIn;
	FILE* hFileOut;
	FILE* hFileROM;
	UINT32 romLen;
	UINT8* romData;
	UINT32 smplPos;
	UINT32 vgmLen;
	UINT32 tempInt32;
	UINT16 inVal;
	UINT8 cmdCount;
	size_t readEl;

	if (argc < 3)
	{
		printf("Usage: %s in.cap out.vgm [samplerom.bin]\n", argv[0]);
		return 0;
	}

	hFileIn = fopen(argv[1], "rb");
	if (hFileIn == NULL)
	{
		printf("Error opening file %s!\n", argv[1]);
		return 1;
	}
	hFileOut = fopen(argv[2], "wb");
	if (hFileOut == NULL)
	{
		printf("Error opening file %s!\n", argv[2]);
		return 1;
	}

	smplPos = 0;

	fwrite(VGM_HEADER, 0x01, 0xC0, hFileOut);
	if (argc > 3)
	{
		hFileROM = fopen(argv[3], "rb");
		if (hFileROM == NULL)
		{
			printf("Error opening file %s!\n", argv[3]);
			return 1;
		}
		fseek(hFileROM, 0, SEEK_END);
		romLen = ftell(hFileROM);

		rewind(hFileROM);
		romData = (UINT8*)malloc(romLen);
		fread(romData, 0x01, romLen, hFileROM);

		fclose(hFileROM);	hFileROM = NULL;

		fputc(0x67, hFileOut);
		fputc(0x66, hFileOut);
		fputc(0x89, hFileOut);
		tempInt32 = romLen + 0x08;	// data block size
		fwrite(&tempInt32, 0x04, 0x01, hFileOut);
		fwrite(&romLen, 0x04, 0x01, hFileOut);
		tempInt32 = 0x00;	// data start offset
		fwrite(&tempInt32, 0x04, 0x01, hFileOut);
		fwrite(romData, 0x01, romLen, hFileOut);
		free(romData);	romData = NULL;
	}

	// Find sync word
	for(int i = 0; i < 3; i++) {
		int prev = fgetc(hFileIn);
		while(!feof(hFileIn)) {
			int cur = fgetc(hFileIn);
			if(prev == 0x61 && cur == 0xb5)
				break;
			prev = cur;
		}
	}

	// printf("sync word %d\n", ftell(hFileIn));

	int cmd = 0;
	while(1)
	{
		readEl = fread(&inVal, 0x02, 1, hFileIn);
		if (readEl == 0)
			break;
		if(inVal == 0xb561) {
			// printf("sync word at %d\n", ftell(hFileIn));
			continue;
		}
//		inVal = (inVal >> 8) | ((inVal & 0xff) << 8);
		inVal *= 2;
		if (inVal > 0)
		{
			if (inVal <= 0x10)
			{
				fputc(0x70 + (inVal - 1), hFileOut);
			}
			else
			{
				fputc(0x61, hFileOut);
				fwrite(&inVal, 0x02, 0x01, hFileOut);
			}
		}
		smplPos += inVal;
		// printf("wait=%04x ", inVal);

		readEl = fread(&inVal, 0x02, 1, hFileIn);
		if (readEl == 0)
			break;
		// printf("cmd=%04x\n", inVal);
		fputc(0xB5, hFileOut);
		fputc((inVal >> 0) & 0xFF, hFileOut);
		fputc((inVal >> 8) & 0xFF, hFileOut);
	}
	fputc(0x66, hFileOut);
	vgmLen = ftell(hFileOut);

	fseek(hFileOut, 0x18, SEEK_SET);
	fwrite(&smplPos, 0x04, 0x01, hFileOut);

	vgmLen -= 0x04;
	fseek(hFileOut, 0x04, SEEK_SET);
	fwrite(&vgmLen, 0x04, 0x01, hFileOut);

	fclose(hFileIn);	hFileIn = NULL;
	fclose(hFileOut);	hFileOut = NULL;

	return 0;
}