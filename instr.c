#include <stdio.h>
#include <stdint.h>
#include <string.h>

struct tg100_instrument {
	uint8_t filler1[16];
	uint8_t name[8];
	uint8_t filler2[72];
};

void tg100_instrument_from_buffer(struct tg100_instrument *instr, uint8_t *buf) {
	memcpy(instr->filler1, buf, sizeof(instr->filler1));
	memcpy(instr->name, buf + 0x10, 8);
	memcpy(instr->filler2, buf + 0x18, sizeof(instr->filler2));
}

void tg100_instrument_dump_csv(struct tg100_instrument *instr) {
	for(int i = 0; i < 16; i++) printf("%02x\t", instr->filler1[i]);
	printf(
		"%c%c%c%c%c%c%c%c",
		instr->name[0], instr->name[1], instr->name[2], instr->name[3],
		instr->name[4], instr->name[5], instr->name[6], instr->name[7]
	);
	for(int i = 0; i < sizeof(instr->filler2); i++) printf("\t%02x", instr->filler2[i]);
	printf("\n");
}

int main(int argc, char **argv) {
	FILE *f = fopen("tg100prog.bin", "rb");
	fseek(f, 0x10410, SEEK_SET);
	uint8_t buf[96];
	for(int i = 0; i < 192; i++) {
		printf("%08x\t", (uint32_t)ftell(f));
		fread(buf, 1, sizeof(buf), f);
		struct tg100_instrument instr;
		tg100_instrument_from_buffer(&instr, buf);
		tg100_instrument_dump_csv(&instr);
	}
	fclose(f);
	return 0;
}
