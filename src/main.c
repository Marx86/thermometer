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


void i2c_write_byte(char byte) {
    uint8_t i;

    I2C &= ~SCL_mask; // Set SCL pin to "0"

    for (uint8_t i=8; i--; i>=0) {
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


void convert_for_4mux_dysplay(char temp[], char TEMP[]) {
    uint8_t i, len, start_index, position = 0,  invert_position_a, invert_position_b;
    char symbol;

    len = strlen(temp);
    start_index = len - 3;

    for (i=start_index; i<len; i++) {
        symbol = pgm_read_byte(&SYMBOLS[temp[i]-44]);
        invert_position_a = 7 - position;
        invert_position_b = 6 - position;

        if (temp[i] == '-') {
            TEMP[2] |= ((0b01000000 & symbol) >> 6) << invert_position_a;
            position++;
        }
        else if (temp[i] == '.') {
            TEMP[3] |= ((0b10000000 & symbol) >> 7) << invert_position_b;
            TEMP[2] |= ((0b01000000 & symbol) >> 6) << invert_position_b;
            position++;
        }
        else {
            TEMP[3] |= ((0b00001000 & symbol) >> 3) << invert_position_a;
            TEMP[2] |= ((0b00000100 & symbol) >> 2) << invert_position_a;
            TEMP[1] |= ((0b00000010 & symbol) >> 1) << invert_position_a;
            TEMP[0] |= (0b00000001 & symbol) << invert_position_a;
            TEMP[3] |= ((0b10000000 & symbol) >> 7) << invert_position_b;
            TEMP[2] |= ((0b01000000 & symbol) >> 6) << invert_position_b;
            TEMP[1] |= ((0b00100000 & symbol) >> 5) << invert_position_b;
            TEMP[0] |= ((0b00010000 & symbol) >> 4) << invert_position_b;
            position += 2;
        }
    }
}


void print_com(uint8_t i, char TEMP[]) {
    PORTB = TEMP[i];
    set_h(DDRD, i+3);
    set_h(PORTD, i+3);
    _delay_ms(1.33);
    PORTB = ~TEMP[i];
    set_l(PORTD, i+3);
    _delay_ms(1.33);
    set_l(DDRD, i+3);
}


void main() {
    // , - is space
    char symbol, raw_temp[3], temp[5] = ",,", TEMP[4] = {0, 0, 0, 0};
    uint8_t i, iterations = 0;

    DDRB = 0b11111111;
    DDRD = 0b11110000;
    DDRA = 0b11111111;

    while (1) {
        if (iterations == 255) {
            itoa(read_temp(), raw_temp);
            strcat(temp, raw_temp);
            iterations = 0;

            convert_for_4mux_dysplay(temp, TEMP);
        }

        for (i=0; i<3; i++) {
            print_com(i, TEMP);
        }

        iterations++;
    }
}
