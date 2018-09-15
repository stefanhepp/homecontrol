OPTIONS=-mmcu=atmega48
SOURCES=main.c

all: control.hex

control.elf: ${SOURCES}
	avr-gcc ${OPTIONS} -o $@ $^

control.hex: control.elf
	avr-objcopy -j .text -j .data -O ihex $^ $@

.PHONY: clean
clean:
	rm -f *.elf *.hex *.o
