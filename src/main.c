#define F_CPU 128000L
#define space '.'
#define C '/'
#define deg ','
#define SCL 0
#define SDA 1
#define I2C PORTA
#define SDA_mask = (1 << SDA);
#define SCL_mask = (1 << SCL);
#define set_h(port, pin) (port |= (1 << pin))
#define set_l(port, pin) (port &= ~(1 << pin))

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>


uint8_t TEMP;

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


void get_symbol(uint8_t index) {
    return pgm_read_byte(&SYMBOLS[index]);
}


void write_symbol(uint8_t symbol, uint8_t position) {
    char current_symbol = get_symbol(symbol - 44);

    DDRD = 1 << position;
    PORTD = 0;
    PORTB = current_symbol;
    _ms_delay(1.33);
    PORTD = DDRD;
    PORTB = ~current_symbol;
    _ms_delay(1.33);
}


void i2c_write_byte(char byte) {
    uint8_t i;

    I2C &= ~SCL_mask; // Set SCL pin to "0"

    for (i=8; i--; i>=0) {
        I2C &= ~SDA_mask | ((1 & byte >> i) << SDA);

        I2C ^= SCL_mask;
        I2C ^= SCL_mask;
    }

    // ACK
    I2C ^= SCL_mask;
    I2C ^= SCL_mask;
}


void i2c_read_byte() {
    uint8_t i;
    char byte = 0b00000000;

    set_l(DDRA, SDA); // Set SDA pin to read mode
    I2C &= ~SCL_mask; // Set SCL pin to "0"

    for (i=8; i--; i>=0) {
        I2C ^= SCL_mask;
        byte |= (I2C & SDA_mask) >> SDA << i;
        I2C ^= SCL_mask;
    }

    // ACK
    I2C ^= SCL_mask;
    I2C ^= SCL_mask;

    set_h(DDRA, SDA); // Set SDA pin to write mode

    return byte;
}


void i2c_start() {
    // Write start bit
    set_h(I2C, SCL);
    set_h(I2C, SDA);
    set_l(I2C, SDA);
    set_l(I2C, SCL);
}


void i2c_stop() {
    set_l(I2C, SCL);
    set_l(I2C, SDA);
    set_h(I2C, SCL);
    set_h(I2C, SDA);
}


void read_temp() {
    uint16_t result;

    i2c_start()

    // Write address and "write bit"
    i2c_write_byte(0b10000001);

    // Write command
    i2c_write_byte(0xE3);

    i2c_start()

    // Write address and "read bit"
    i2c_write_byte(0b10000000);

    _delay_ms(16);

    // Read result
    result |= i2c_read_byte() << 8;
    result |= i2c_read_byte();

    i2c_stop()

    TEMP = ((175.72 * result) / 65536) - 46.85;
}


void main() {
    int8_t temp;
    char print_temp[6];

    DDRB = 0b11111111;
    DDRD = 0b00000000;
    DDRA = 0b00000011

    while (1) {
        temp = read_temp();

        sprintf(print_temp, "%3i%c%c%c", temp, space, deg, C);

        for (uint8_t i; i<6; i++) {
            write_symbol(print_temp[i], i);
        }
    }
}
