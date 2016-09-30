#define F_CPU 128000L
#define SCL 0
#define SDA 1
#define I2C PORTA
#define SDA_mask (1 << SDA)
#define SCL_mask (1 << SCL)
#define set_h(port, pin) (port |= (1 << pin))
#define set_l(port, pin) (port &= ~(1 << pin))
#define set_h_SCL() I2C |= SCL_mask
#define set_h_SDA() I2C |= SDA_mask
#define set_l_SCL() I2C &= ~SCL_mask
#define set_l_SDA() I2C &= ~SDA_mask

#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <avr/pgmspace.h>


const char SYMBOLS[14] PROGMEM = {
    //abchfged
    0b00000000, // space
    0b00000100, // -
    0b11001100, // deg °
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


void display_print_first_half_of_symbol(char symbol, uint8_t position) {
    char m_symbol = symbol >> 4;

    DDRB = 0b00000000;
    PORTD &= 0b10000111 | (m_symbol << 3);
    DDRB = 1 << position;
    PORTB = 0b00000000;

    _delay_ms(1.33);

    PORTD &= 0b10000111 | (~m_symbol << 3);
    PORTB = DDRB;

    _delay_ms(1.33);
}


void display_print_second_half_of_symbol(char symbol, uint8_t position) {
    char l_symbol = symbol & 0b00001111;

    DDRB = 0b00000000;
    PORTD &= 0b10000111 | (l_symbol << 3);
    DDRB = 1 << position;
    PORTB = 0b00000000;

    _delay_ms(1.33);

    PORTD &= 0b10000111 | (~l_symbol << 3);
    PORTB = DDRB;

    _delay_ms(1.33);
}


void display_print_symbol(char symbol, uint8_t position) {
    display_print_first_half_of_symbol(symbol, position);
    display_print_second_half_of_symbol(symbol, position+1);
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

    set_h_SDA(); // Set SDA pin to write mode

    return byte;
}


void i2c_start() {
    // Write start bit
    set_h_SCL();
    set_h_SDA();
    set_l_SDA();
    set_l_SCL();
}


void i2c_stop() {
    set_l_SCL();
    set_l_SDA();
    set_h_SCL();
    set_h_SDA();
}


int8_t read_temp() {
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

    return ((175.72 * result) / 65536) - 46.85;
}


void main() {
    // , - is space
    char symbol, raw_temp[3], temp[5] = ",,";
    uint8_t i, len, start_index, position, iterations = 0;

    DDRB = 0b00000000;
    DDRD = 0b01111011;
    DDRA = 0b00000011;

    while (1) {
        if (iterations == 255) {
            itoa(read_temp(), raw_temp);
            strcat(temp, raw_temp);
            len = strlen(temp);
            start_index = len - 3;
            iterations = 0;
        }

        position = 0;
        for (i=start_index; i<len; i++) {
            symbol = pgm_read_byte(&SYMBOLS[temp[i]-44]);
            if (temp[i] == '-') {
                display_print_first_half_of_symbol(symbol, position);
                position++;
            }
            else {
                display_print_symbol(symbol, position);
                position += 2;
            }
        }

        symbol = pgm_read_byte(&SYMBOLS[2]);
        display_print_first_half_of_symbol(symbol, position);

        position++;
        symbol = pgm_read_byte(&SYMBOLS[3]);
        display_print_symbol(symbol, position);

        iterations++;
    }
}
