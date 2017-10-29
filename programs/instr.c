#include <stdio.h>
#include <stdint.h>
#include <string.h>

struct tg100_instrument {
	uint8_t data[96];
	uint8_t is_dual_element;
	uint8_t lvl[2];
	uint8_t dtn[2];
	uint8_t pan[2];
	uint8_t grp[2]; // region group
	uint8_t name[8];
};

void tg100_instrument_from_buffer(struct tg100_instrument *instr, uint8_t *buf) {
	memcpy(instr->data, buf, 96);

	instr->is_dual_element = buf[0];
	instr->lvl[0] = buf[1];
	instr->lvl[1] = buf[2];
	instr->dtn[0] = buf[3];
	instr->dtn[1] = buf[4];
	instr->pan[0] = buf[40];
	instr->pan[1] = buf[76];
	instr->grp[0] = (buf[24] << 4) | buf[25];
	instr->grp[1] = (buf[60] << 4) | buf[61];
	memcpy(instr->name, buf + 0x10, 8);
}

void tg100_instrument_dump_csv(struct tg100_instrument *instr) {
	printf("%d\t%d\t%d\t%d\t%d\t", instr->is_dual_element, instr->lvl[0], instr->lvl[1], (int)instr->dtn[0] - 64, (int)instr->dtn[1] - 64);
	for(int i = 5; i < 16; i++) printf("%02x\t", instr->data[i]);
	printf(
		"%c%c%c%c%c%c%c%c\t",
		instr->name[0], instr->name[1], instr->name[2], instr->name[3],
		instr->name[4], instr->name[5], instr->name[6], instr->name[7]
	);
	printf("%d\t", instr->grp[0]);
	for(int i = 26; i < 40; i++) printf("%02x\t", instr->data[i]);
	if(instr->pan[0] == 0) printf("0\t");
	else printf("%c%d\t", instr->pan[0] > 7 ? 'L' : 'R', instr->pan[0] > 7 ? 0x10 - (int)instr->pan[0] : instr->pan[0]);
	for(int i = 41; i < 60; i++) printf("%02x\t", instr->data[i]);
	printf("%d\t", instr->grp[1]);
	for(int i = 62; i < 76; i++) printf("%02x\t", instr->data[i]);
	if(instr->pan[1] == 0) printf("0\t");
	else printf("%c%d\t", instr->pan[1] > 7 ? 'L' : 'R', instr->pan[1] > 7 ? 0x10 - (int)instr->pan[1] : instr->pan[1]);
	for(int i = 77; i < 95; i++) printf("%02x\t", instr->data[i]);
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
