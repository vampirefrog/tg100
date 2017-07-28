#include <stdio.h>
#include <stdint.h>

struct ymw_sound {
	int16_t pitch_offset;
	uint8_t tone_attenuate;
	uint8_t reg_attack_decay1;
	uint8_t reg_release_correction;
};

struct ymw_region {
	uint16_t sample_num;
	uint8_t key_min, key_max;
	struct ymw_sound sound;
};

void ymw_region_dump_csv(struct ymw_region *reg) {
	printf(
		"%03x\t%02x\t%02x\t%d\t%04x\t%02x\t%02x\t%02x\n",
		reg->sample_num, reg->key_min, reg->key_max,
		reg->sound.pitch_offset, reg->sound.pitch_offset, reg->sound.tone_attenuate,
		reg->sound.reg_attack_decay1, reg->sound.reg_release_correction
	);
}

void ymw_region_from_buf(struct ymw_region *reg, uint8_t *buf) {
	reg->sample_num = (buf[0] << 8) | buf[1];
	reg->key_min = buf[2];
	reg->key_max = buf[3];
	reg->sound.pitch_offset = ((buf[4] << 8) | buf[5]) >> 2;
	reg->sound.tone_attenuate = buf[6];
	reg->sound.reg_attack_decay1 = buf[7];
	reg->sound.reg_release_correction = buf[8];
}

#define NUM_OFFSETS 140
uint16_t region_offsets[NUM_OFFSETS];

int main(int argc, char **argv) {
	FILE *f = fopen("tg100prog.bin", "rb");

	fseek(f, 0x1841c, SEEK_SET);
	fread(region_offsets, 1, sizeof(region_offsets), f);
	for(int i = 0; i < NUM_OFFSETS; i++) {
		region_offsets[i] = (region_offsets[i] >> 8) | (region_offsets[i] << 8);
	}

	fseek(f, 0x176ab, SEEK_SET);
	int cur_ofs = 1;
	for(int i = 3; ;i++) {
		struct ymw_region reg;
		uint8_t buf[9];
		printf("%08x\t%d\t", ftell(f), cur_ofs);
		if(region_offsets[cur_ofs] < i) cur_ofs++;
		fread(buf, 1, sizeof(buf), f);
		ymw_region_from_buf(&reg, buf);
		if(reg.sample_num == 0) break;
		ymw_region_dump_csv(&reg);
	}

	fclose(f);

	return 0;
}
