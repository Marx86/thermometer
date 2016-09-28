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
    //abchfged
    0b11001100, // deg °
    0b00000100, // -
    0b00000000, // space
    0b10001011, // C
    0b11101011, // 0
    0b01100000, // 1
    0b11000111, // 2
    0b11100101, // 3
    0b01101100, // 4
    0b10101101, // 5
    0b10101111, // 6
    0b11100000, // 7
    0b11101111, // 8
    0b11101101  // 9
};


char get_symbol(uint8_t index) {
    return pgm_read_byte(&SYMBOLS[index]);
}


void display_print_symbol(uint8_t symbol, uint8_t position) {
    char current_symbol = get_symbol(symbol - 44);
    char m_symbol = current_symbol >> 4;
    char l_symbol = current_symbol & 0b00001111;

    if (position == 0) {
        DDRB = 0b00000000;
        PORTB = 0b00000000;

        PORTD &= 0b11100001 | (m_symbol << 1);
        DDRD = 0b01011110;

        _ms_delay(1.33);

        PORTD &= 0b10100001 | ((~m_symbol << 1) | 0b01000000);

        _ms_delay(1.33);

        DDRD = 0b00011110;

        PORTD &= 0b11100001 | (l_symbol << 1);
        DDRD = 0b00111110;

        _ms_delay(1.33);

        PORTD &= 0b11000001 | ((~l_symbol << 1) | 0b00100000);

        DDRD = 0b00011110;
    }

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


char i2c_read_byte() {
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

    i2c_start();

    // Write address and "write bit"
    i2c_write_byte(0b10000001);

    // Write command
    i2c_write_byte(0xE3);

    i2c_start();

    // Write address and "read bit"
    i2c_write_byte(0b10000000);

    _delay_ms(16);

    // Read result
    result |= i2c_read_byte() << 8;
    result |= i2c_read_byte();

    i2c_stop();

    TEMP = ((175.72 * result) / 65536) - 46.85;
}


void main() {
    char print_temp[6];

    DDRB = 0b00000000;
    DDRD = 0b00011110;
    DDRA = 0b00000011;

    while (1) {
        sprintf(print_temp, "%3i%c%c%c", TEMP, space, deg, C);

        for (uint8_t i; i<6; i++) {
            display_print_symbol(print_temp[i], i);
        }
    }
}
