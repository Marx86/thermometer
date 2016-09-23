#define F_CPU 128000L
#define space '.'
#define C '/'
#define deg ','

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>


const char SYMBOLS[14] PROGMEM = {
    0b11000110, // deg °
    0b00000010, // -
    0b00000000, // space
    0b10011100, // C
    0b11111100, // 0
    0b01100000, // 1
    0b11011010, // 2
    0b11110010, // 3
    0b01100110, // 4
    0b10110110, // 5
    0b10111110, // 6
    0b11100000, // 7
    0b11111110, // 8
    0b11110110  // 9
};

get_symbol(index) {
    return pgm_read_byte(&SYMBOLS[index]);
}

write_symbol(symbol, position) {
    char current_symbol = get_symbol(symbol - 44);

    DDRD = 1 << position;
    PORTD = 0;
    PORTB = current_symbol;
    _ms_delay(1.33);
    PORTD = DDRD;
    PORTB = ~current_symbol;
    _ms_delay(1.33);
}

read_temp() {
    return 0;
}

void main() {
    int8_t temp;
    char print_temp[6];

    DDRB = 0b11111111;
    DDRD = 0b00000000;

    while (1) {
        temp = read_temp();

        sprintf(print_temp, "%3i%c%c%c", temp, space, deg, C);

        for (uint8_t i; i<6; i++) {
            write_symbol(print_temp[i], i);
        }
    }
}
