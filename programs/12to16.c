#include <stdio.h>
#include <stdint.h>

int main(int argc, char **argv) {
	for(int i = 1; i < argc; i++) {
		FILE *f = fopen(argv[i], "rb");
		char buf[256];
		snprintf(buf, sizeof(buf), "%s.16", argv[i]);
		FILE *o = fopen(buf, "wb");
		uint8_t b[3];
		uint16_t c[4] = { 0, 0 };
		while(!feof(f)) {
			int r = fread(b, 1, 3, f);
			if(r == 0) break;
			c[0] = (b[0] << 8) | ((b[1] & 0x0f) << 4);
			c[1] = (b[2] << 8) | ((b[1] & 0xf0) << 0);
			fwrite(c, 2, r > 2 ? 2 : 1, o);
		}
		fclose(o);
		fclose(f);
	}
	return 0;
}
