/**
 * Laundry Monitor System for ATmega328PB - Clean Error-Free Version
 */

 #define F_CPU 16000000UL
 #include <avr/io.h>
 #include <util/delay.h>
 #include <string.h>
 #include <stdio.h>
 #include <avr/interrupt.h>
 
 // LCD Pins (4-bit mode)
 #define LCD_RS PD0
 #define LCD_EN PD1
 #define LCD_D4 PD2
 #define LCD_D5 PD3
 #define LCD_D6 PD4
 #define LCD_D7 PD5
 
 // UART Configuration
 #define UART_BAUD 9600
 #define UART_BAUD_PRESCALE ((F_CPU / (UART_BAUD * 16UL)) - 1)
 
 // Vibration Sensor
 #define VIBRATION_PIN PC0
 #define VIBRATION_DDR DDRC
 #define VIBRATION_PORT PORTC
 #define VIBRATION_PIN_REG PINC
 
 // Status LED
 #define STATUS_LED PB5
 #define STATUS_LED_DDR DDRB
 #define STATUS_LED_PORT PORTB
 
 // Laundry Machine States
 typedef enum {
     IDLE,
     ACTIVE,
     WAITING,
     COMPLETE
 } LaundryState;
 
 // Function Prototypes
 void lcd_init(void);
 void lcd_command(uint8_t cmd);
 void lcd_data(uint8_t data);
 void lcd_string(const char *str);
 void lcd_set_cursor(uint8_t row, uint8_t col);
 void display_user_info(void);
 void uart_init(void);
 void uart_send(char data);
 void uart_send_string(const char *str);
 uint8_t read_vibration(void);
 void status_led_blink(uint8_t count);
 
 // LCD Implementation
 void lcd_init(void) {
     DDRD |= (1 << LCD_RS) | (1 << LCD_EN) | (1 << LCD_D4) | 
             (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);
     
     _delay_ms(50);
     PORTD &= ~(1 << LCD_RS);
     _delay_ms(20);
     
     // Initialization sequence
     PORTD &= ~(1 << LCD_RS);
     PORTD = (PORTD & 0xC3) | 0x30;
     _delay_ms(5);
     PORTD |= (1 << LCD_EN);
     _delay_us(1);
     PORTD &= ~(1 << LCD_EN);
     _delay_ms(1);
     
     // Repeat initialization
     for(uint8_t i = 0; i < 3; i++) {
         PORTD |= (1 << LCD_EN);
         _delay_us(1);
         PORTD &= ~(1 << LCD_EN);
         _delay_ms(1);
     }
     
     // Set 4-bit mode
     PORTD = (PORTD & 0xC3) | 0x20;
     PORTD |= (1 << LCD_EN);
     _delay_us(1);
     PORTD &= ~(1 << LCD_EN);
     _delay_ms(1);
     
     // Function set: 4-bit, 2-line, 5x8 dots
     lcd_command(0x28);
     _delay_ms(1);
     
     // Display control
     lcd_command(0x0C);
     _delay_ms(1);
     
     // Clear display
     lcd_command(0x01);
     _delay_ms(2);
     
     // Entry mode set
     lcd_command(0x06);
     _delay_ms(1);
 }
 
 void lcd_command(uint8_t cmd) {
     PORTD &= ~(1 << LCD_RS);
     PORTD = (PORTD & 0xC3) | ((cmd & 0xF0) >> 2);
     PORTD |= (1 << LCD_EN);
     _delay_us(1);
     PORTD &= ~(1 << LCD_EN);
     _delay_us(100);
     
     PORTD = (PORTD & 0xC3) | ((cmd & 0x0F) << 2);
     PORTD |= (1 << LCD_EN);
     _delay_us(1);
     PORTD &= ~(1 << LCD_EN);
     _delay_ms(2);
 }
 
 void lcd_data(uint8_t data) {
     PORTD |= (1 << LCD_RS);
     PORTD = (PORTD & 0xC3) | ((data & 0xF0) >> 2);
     PORTD |= (1 << LCD_EN);
     _delay_us(1);
     PORTD &= ~(1 << LCD_EN);
     _delay_us(100);
     
     PORTD = (PORTD & 0xC3) | ((data & 0x0F) << 2);
     PORTD |= (1 << LCD_EN);
     _delay_us(1);
     PORTD &= ~(1 << LCD_EN);
     _delay_ms(1);
 }
 
 void lcd_string(const char *str) {
     while(*str) {
         lcd_data(*str++);
     }
 }
 
 void lcd_set_cursor(uint8_t row, uint8_t col) {
     uint8_t address = (row == 0) ? 0x80 + col : 0xC0 + col;
     lcd_command(address);
 }
 
 void display_user_info(void) {
     lcd_set_cursor(0, 0);
     lcd_string("User: Ashes     ");
     lcd_set_cursor(1, 0);
     lcd_string("Mob: 0443087939");
 }
 
 // UART Functions
 void uart_init(void) {
     UBRR0H = (uint8_t)(UART_BAUD_PRESCALE >> 8);
     UBRR0L = (uint8_t)UART_BAUD_PRESCALE;
     UCSR0B = (1 << TXEN0) | (1 << RXEN0);
     UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
 }
 
 void uart_send(char data) {
     while(!(UCSR0A & (1 << UDRE0)));
     UDR0 = data;
 }
 
 void uart_send_string(const char *str) {
     while(*str) {
         uart_send(*str++);
     }
     uart_send('\r');
     uart_send('\n');
 }
 
 // Vibration Sensor
 uint8_t read_vibration(void) {
     uint8_t samples = 0;
     for(uint8_t i = 0; i < 10; i++) {
         if(!(VIBRATION_PIN_REG & (1 << VIBRATION_PIN))) {
             samples++;
         }
         _delay_ms(2);
     }
     return (samples >= 4) ? 1 : 0;
 }
 
 // Status LED
 void status_led_blink(uint8_t count) {
     for(uint8_t i = 0; i < count; i++) {
         STATUS_LED_PORT |= (1 << STATUS_LED);
         _delay_ms(100);
         STATUS_LED_PORT &= ~(1 << STATUS_LED);
         _delay_ms(100);
     }
     _delay_ms(200);
 }
 
 // Main Program
 int main(void) {
     // Initialize I/O
     VIBRATION_DDR &= ~(1 << VIBRATION_PIN);
     VIBRATION_PORT |= (1 << VIBRATION_PIN);
     
     STATUS_LED_DDR |= (1 << STATUS_LED);
     
     // Power-on test
     status_led_blink(3);
     
     // Initialize peripherals
     lcd_init();
     _delay_ms(500);
     display_user_info();
     
     uart_init();
     _delay_ms(2000);
     uart_send_string("PARTICLE:SYSTEM_READY");
     
     // Main variables
     LaundryState state = IDLE;
     uint16_t stable_counter = 0;
     uint8_t vibration_detected = 0;
     
     while(1) {
         vibration_detected = read_vibration();
         
         // LED feedback
         if(vibration_detected) {
             STATUS_LED_PORT |= (1 << STATUS_LED);
         } else {
             STATUS_LED_PORT &= ~(1 << STATUS_LED);
         }
         
         // State machine
         switch(state) {
             case IDLE:
                 if(vibration_detected) {
                     state = ACTIVE;
                     uart_send_string("PARTICLE:CYCLE_START");
                 }
                 break;
                 
             case ACTIVE:
                 if(!vibration_detected) {
                     state = WAITING;
                     stable_counter = 0;
                 }
                 break;
                 
             case WAITING:
                 if(vibration_detected) {
                     state = ACTIVE;
                 } else {
                     stable_counter++;
                     if(stable_counter > 600) {
                         state = COMPLETE;
                         for(uint8_t i = 0; i < 10; i++) {
                             STATUS_LED_PORT ^= (1 << STATUS_LED);
                             _delay_ms(50);
                         }
                         uart_send_string("PARTICLE:LAUNDRY_DONE");
                     }
                 }
                 break;
                 
             case COMPLETE:
                 _delay_ms(5000);
                 status_led_blink(2);
                 state = IDLE;
                 break;
         }
         
         _delay_ms(100);
     }
     
     return 0;
 }