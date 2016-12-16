all: compile

compile:
	avr-gcc -mmcu=attiny2313 -Os src/main.c -o firmware.o
	avr-objcopy -j .text -j .data -O binary firmware.o  firmware.bin

flash:
	avrdude -p attiny2313 -c avrisp -e -U flash:w:firmware.bin

install: flash

clean:
	rm -rf *.o *.hex *.bin
