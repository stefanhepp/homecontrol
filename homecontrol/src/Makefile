OPTIONS=-mmcu=atmega48p -Wall -Os
SOURCES=main.c

all: control.hex

control.elf: ${SOURCES}
	avr-gcc ${OPTIONS} -o $@ $^

control.hex: control.elf
	avr-objcopy -j .text -j .data -O ihex $^ $@

.PHONY: clean
clean:
	rm -f *.elf *.hex *.o
