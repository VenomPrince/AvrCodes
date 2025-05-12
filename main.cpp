/**
 * Laundry Monitor System - Working Version with LCD Display
 */

 #define F_CPU 16000000UL
 #include <avr/io.h>
 #include <util/delay.h>
 #include <string.h>
 #include <stdio.h>
 
 // LCD Pins (4-bit mode)
 #define LCD_RS PD0
 #define LCD_EN PD1
 #define LCD_D4 PD2
 #define LCD_D5 PD3
 #define LCD_D6 PD4
 #define LCD_D7 PD5
 
 // Vibration Sensor
 #define VIBRATION_PIN PC0
 #define VIBRATION_DDR DDRC
 #define VIBRATION_PORT PORTC
 #define VIBRATION_PIN_REG PINC
 
 // Status LED
 #define STATUS_LED PB5
 #define STATUS_LED_DDR DDRB
 #define STATUS_LED_PORT PORTB
 
 // Laundry States
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
 uint8_t read_vibration(void);
 void status_led_blink(uint8_t count);
 
 // LCD Implementation
 void lcd_init(void) {
     // Set all LCD pins as outputs
     DDRD |= (1 << LCD_RS) | (1 << LCD_EN) | 
             (1 << LCD_D4) | (1 << LCD_D5) | 
             (1 << LCD_D6) | (1 << LCD_D7);
     
     // Wait for LCD to power up
     _delay_ms(50);
     
     // Initialization sequence
     PORTD &= ~(1 << LCD_RS); // Command mode
     PORTD &= ~(1 << LCD_EN);
     
     // Step 1: Send 0x03 three times
     for(uint8_t i = 0; i < 3; i++) {
         PORTD = (PORTD & 0xC3) | 0x30; // 0x03
         PORTD |= (1 << LCD_EN);
         _delay_us(1);
         PORTD &= ~(1 << LCD_EN);
         _delay_ms(5);
     }
     
     // Step 2: Switch to 4-bit mode
     PORTD = (PORTD & 0xC3) | 0x20; // 0x02
     PORTD |= (1 << LCD_EN);
     _delay_us(1);
     PORTD &= ~(1 << LCD_EN);
     _delay_ms(1);
     
     // Function set: 4-bit, 2-line, 5x8 dots
     lcd_command(0x28);
     _delay_ms(1);
     
     // Display control
     lcd_command(0x0C); // Display on, cursor off
     _delay_ms(1);
     
     // Clear display
     lcd_command(0x01);
     _delay_ms(2);
     
     // Entry mode set
     lcd_command(0x06); // Increment, no shift
     _delay_ms(1);
 }
 
 void lcd_command(uint8_t cmd) {
     PORTD &= ~(1 << LCD_RS); // Command mode
     
     // Send high nibble
     PORTD = (PORTD & 0xC3) | ((cmd & 0xF0) >> 2);
     PORTD |= (1 << LCD_EN);
     _delay_us(1);
     PORTD &= ~(1 << LCD_EN);
     _delay_us(100);
     
     // Send low nibble
     PORTD = (PORTD & 0xC3) | ((cmd & 0x0F) << 2);
     PORTD |= (1 << LCD_EN);
     _delay_us(1);
     PORTD &= ~(1 << LCD_EN);
     
     if(cmd == 0x01 || cmd == 0x02) {
         _delay_ms(2);
     }
 }
 
 void lcd_data(uint8_t data) {
     PORTD |= (1 << LCD_RS); // Data mode
     
     // Send high nibble
     PORTD = (PORTD & 0xC3) | ((data & 0xF0) >> 2);
     PORTD |= (1 << LCD_EN);
     _delay_us(1);
     PORTD &= ~(1 << LCD_EN);
     _delay_us(100);
     
     // Send low nibble
     PORTD = (PORTD & 0xC3) | ((data & 0x0F) << 2);
     PORTD |= (1 << LCD_EN);
     _delay_us(1);
     PORTD &= ~(1 << LCD_EN);
     _delay_us(100);
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
     
     // Initialize LCD with proper delays
     lcd_init();
     _delay_ms(500);
     
     // Display user info
     display_user_info();
     
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