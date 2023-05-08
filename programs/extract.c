#include <stdio.h>
#include <stdint.h>

struct ymw_sample {
	uint8_t data_bit;
	uint32_t start_address;
	uint16_t loop_offset;
	int16_t end_offset;
	uint16_t length;
	uint8_t lfo, vib;
	uint8_t ar, d1r, dl, d2r;
	uint8_t rate_correction, rr;
	uint8_t am;
};

void ymw_sample_from_buffer(struct ymw_sample *s, uint8_t *buf) {
	s->data_bit = buf[0] >> 6;
	s->start_address = buf[2] | (buf[1] << 8) | ((buf[0] & 0x3f) << 16);
	s->loop_offset = buf[4] | (buf[3] << 8);
	s->end_offset = buf[6] | (buf[5] << 8);
	s->length = 0x10000 - s->end_offset;
	s->lfo = (buf[7] >> 3) & 0x07;
	s->vib = buf[7] & 0x07;
	s->ar = buf[8] >> 4;
	s->d1r = buf[8] & 0x0f;
	s->dl = buf[9] >> 4;
	s->d2r = buf[9] & 0x0f;
	s->rate_correction = buf[0xa] >> 4;
	s->rr = buf[0xa] & 0x0f;
	s->am = buf[0xb] & 0x07;
}

void ymw_sample_dump_csv(struct ymw_sample *s) {
	printf("%x\t0x%06x\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
		s->data_bit, s->start_address, s->loop_offset, s->length, s->lfo, s->vib, s->ar, s->d1r, s->dl, s->d2r, s->rate_correction, s->rr, s->am);
}

int main(int argc, char **argv) {
	FILE *f = fopen(argv[1], "rb");
	if(!f) {
		perror(argv[1]);
		return 1;
	}
	uint8_t smplbytes[12];
	for(int i = 0; i < 512; i++) {
		fread(smplbytes, 1, 12, f);
		size_t t = ftell(f);
		if(smplbytes[0]) {
			struct ymw_sample s;
			printf("%d\t", i);
			ymw_sample_from_buffer(&s, smplbytes);
			ymw_sample_dump_csv(&s);
			char fbuf[100];
			snprintf(fbuf, sizeof(fbuf), "smpl-%03x.raw", i);
			FILE *o = fopen(fbuf, "wb");
			fseek(f, s.start_address, SEEK_SET);
			uint8_t bbuf[256];
			int l = s.length * 3 / 2;
			while(l > 0) {
				int r = fread(bbuf, 1, l > 256 ? 256 : l, f);
				if(r == 0) break;
				l -= r;
				fwrite(bbuf, r, 1, o);
			}
			fclose(o);
			fseek(f, t, SEEK_SET);
		}
	}
	fclose(f);

	return 0;
}
