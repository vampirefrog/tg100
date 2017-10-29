CC=gcc
CFLAGS=-Wall -O2
PROGS=12to16 extract instr patch vgmcapture

all: $(PROGS)

%: %.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(PROGS)
