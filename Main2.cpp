/**
 * Laundry Monitor System for ATmega328PB
 * Detects when laundry cycle completes and sends notification via Particle Photon
 * Always displays user name and contact information without changing
 */

 #define F_CPU 16000000UL
 #include <avr/io.h>
 #include <util/delay.h>
 #include <string.h>
 #include <stdio.h>
 #include <avr/interrupt.h>
 
 // === LCD PINS (4-bit mode) ===
 #define LCD_RS PD0   // Register Select
 #define LCD_EN PD1   // Enable
 #define LCD_D4 PD2   // Data bit 4
 #define LCD_D5 PD3   // Data bit 5
 #define LCD_D6 PD4   // Data bit 6
 #define LCD_D7 PD5   // Data bit 7
 
 // === Communication with Particle Photon ===
 #define UART_BAUD 9600  // Reduced baud rate for better reliability
 #define UART_BAUD_PRESCALE ((F_CPU / (UART_BAUD * 16UL)) - 1)
 
 // === Vibration sensor ===
 #define VIBRATION_PIN PC0
 #define VIBRATION_PORT PORTC
 #define VIBRATION_PIN_PORT PINC
 
 // === Status LED ===
 #define STATUS_LED PB5  // Use onboard LED if available
 #define STATUS_LED_PORT PORTB
 #define STATUS_LED_DDR DDRB
 
 // Define states for the laundry machine
 typedef enum {
     IDLE,           // No activity detected yet
     ACTIVE,         // Vibration detected, machine running
     WAITING,        // No vibration after activity, counting stability
     COMPLETE        // Laundry cycle complete, notification sent
 } LaundryState;
 
 // === Function Prototypes ===
 void lcd_pulse_enable(void);
 void lcd_send_nibble(unsigned char);
 void lcd_command(unsigned char);
 void lcd_data(unsigned char);
 void lcd_init(void);
 void lcd_string(const char*);
 void lcd_clear(void);
 void lcd_set_cursor(uint8_t, uint8_t);
 void display_user_info(void);
 void uart_init(void);
 void uart_send(char);
 void uart_send_string(const char*);
 void uart_send_debug(const char*, uint16_t);
 void notify_laundry_done(void);
 uint8_t read_vibration(void);
 void status_led_blink(uint8_t);
 
 // === LCD Implementation ===
 void lcd_pulse_enable(void) {
     // Pulse enable pin
     PORTD |= (1 << LCD_EN);
     _delay_us(2);  // Ensure pulse is at least 450ns
     PORTD &= ~(1 << LCD_EN);
     _delay_us(100);  // Allow LCD to process command
 }
 
 void lcd_send_nibble(unsigned char nibble) {
     // Clear data pins
     PORTD &= ~((1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7));
     
     // Set data pins according to nibble
     if (nibble & 0x01) PORTD |= (1 << LCD_D4);
     if (nibble & 0x02) PORTD |= (1 << LCD_D5);
     if (nibble & 0x04) PORTD |= (1 << LCD_D6);
     if (nibble & 0x08) PORTD |= (1 << LCD_D7);
     
     lcd_pulse_enable();
 }
 
 void lcd_command(unsigned char cmd) {
     // Command mode (RS = 0)
     PORTD &= ~(1 << LCD_RS);
     
     // Send higher nibble first
     lcd_send_nibble(cmd >> 4);
     
     // Send lower nibble
     lcd_send_nibble(cmd & 0x0F);
     
     // If the command is clear or return home, need longer delay
     if (cmd == 0x01 || cmd == 0x02) {
         _delay_ms(2);  // Clear and Return home commands need 1.52ms
     } else {
         _delay_us(50);  // Most commands need 37us
     }
 }
 
 void lcd_data(unsigned char data) {
     // Data mode (RS = 1)
     PORTD |= (1 << LCD_RS);
     
     // Send higher nibble first
     lcd_send_nibble(data >> 4);
     
     // Send lower nibble
     lcd_send_nibble(data & 0x0F);
     
     _delay_us(50);  // Data writes need 37us + margin
 }
 
 void lcd_init(void) {
     // Configure data pins as outputs
     DDRD |= (1 << LCD_RS) | (1 << LCD_EN) | (1 << LCD_D4) | 
             (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);
     
     // Initial delay after power-up (longer delay for LCD to stabilize)
     _delay_ms(200);  // Increased delay for stability
     
     // Pull RS low to send commands
     PORTD &= ~(1 << LCD_RS);
     
     // Wait for more than 15ms after VCC rises to 4.5V
     _delay_ms(50);  // Increased delay
     
     // Send 0x03 command three times with specific timing
     // First try
     lcd_send_nibble(0x03);
     _delay_ms(10);  // Increased delay
     
     // Second try
     lcd_send_nibble(0x03);
     _delay_ms(10);  // Increased delay
     
     // Third try
     lcd_send_nibble(0x03);
     _delay_ms(10);  // Increased delay
     
     // Now switch to 4-bit mode
     lcd_send_nibble(0x02);
     _delay_ms(10);  // Increased delay
     
     // Now in 4-bit mode, can use regular lcd_command
     
     // Function set: 4-bit mode, 2 lines, 5x8 dots
     lcd_command(0x28);
     _delay_ms(5);  // Added delay
     
     // Display control: Display on, Cursor off, Blink off
     lcd_command(0x0C);
     _delay_ms(5);  // Added delay
     
     // Clear display
     lcd_command(0x01);
     _delay_ms(5);  // Extra delay after clear
     
     // Entry mode set: Increment cursor position, No display shift
     lcd_command(0x06);
     _delay_ms(5);  // Added delay
     
     // Extra clear display to ensure fresh start
     lcd_command(0x01);
     _delay_ms(10);  // Increased delay
 }
 
 void lcd_string(const char* str) {
     while (*str) {
         lcd_data(*str++);
     }
 }
 
 void lcd_clear(void) {
     lcd_command(0x01);  // Clear display command
     _delay_ms(5);       // Clear display needs extra time
 }
 
 void lcd_set_cursor(uint8_t row, uint8_t col) {
     uint8_t address;
     if (row == 0) {
         address = 0x80 + col;  // First line starts at 0x80
     } else {
         address = 0xC0 + col;  // Second line starts at 0xC0
     }
     lcd_command(address);
 }
 
 // Display user info on LCD (always shown)
 void display_user_info(void) {
     // First line: User name
     lcd_set_cursor(0, 0);
     lcd_string("User: Ashes     ");
     
     // Second line: Contact info
     lcd_set_cursor(1, 0);
     lcd_string("Mob: 0443087939");
 }
 
 // === Status LED Functions ===
 void status_led_blink(uint8_t count) {
     for (uint8_t i = 0; i < count; i++) {
         STATUS_LED_PORT |= (1 << STATUS_LED);
         _delay_ms(100);
         STATUS_LED_PORT &= ~(1 << STATUS_LED);
         _delay_ms(100);
     }
     _delay_ms(200);  // Pause between blink sequences
 }
 
 // === UART Functions for communicating with Particle Photon ===
 void uart_init(void) {
     // Set baud rate - Reduced to 9600 for better reliability
     UBRR0H = (uint8_t)(UART_BAUD_PRESCALE >> 8);
     UBRR0L = (uint8_t)UART_BAUD_PRESCALE;
     
     // Enable transmitter and receiver
     UCSR0B = (1 << TXEN0) | (1 << RXEN0);
     
     // Set frame format: 8 data bits, 1 stop bit, no parity
     UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
 }
 
 void uart_send(char data) {
     // Wait for empty transmit buffer
     while (!(UCSR0A & (1 << UDRE0)));
     
     // Put data into buffer, sends the data
     UDR0 = data;
 }
 
 void uart_send_string(const char* str) {
     while (*str) {
         uart_send(*str++);
     }
     // Send newline and carriage return for proper communication
     uart_send('\r');
     uart_send('\n');
 }
 
 // Send debug information with numeric value
 void uart_send_debug(const char* msg, uint16_t value) {
     char buffer[50];
     snprintf(buffer, 50, "DEBUG: %s: %d", msg, value);
     uart_send_string(buffer);
 }
 
 // Send notification to Particle Photon that laundry is done
 void notify_laundry_done(void) {
     // Protocol for Particle Photon: "PARTICLE:LAUNDRY_DONE"
     uart_send_string("PARTICLE:LAUNDRY_DONE");
     
     // Send multiple times with delay to ensure reception
     _delay_ms(100);
     uart_send_string("PARTICLE:LAUNDRY_DONE");
     _delay_ms(100);
     uart_send_string("PARTICLE:LAUNDRY_DONE");
 }
 
 // === Vibration Sensor Functions ===
 uint8_t read_vibration(void) {
     // Added multiple samples to improve reliability
     uint8_t samples = 0;
     
     // Take 10 samples and count how many are active
     for (uint8_t i = 0; i < 10; i++) {
         // Read current state (inverted logic - sensor pulls LOW when vibration detected)
         if (!(VIBRATION_PIN_PORT & (1 << VIBRATION_PIN))) {
             samples++;
         }
         _delay_ms(2);  // Short delay between samples
     }
     
     // Return 1 if at least 40% of samples detected vibration
     return (samples >= 4) ? 1 : 0;
 }
 
 // === Main Program ===
 int main(void) {
     // Initialize ports
     DDRC &= ~(1 << VIBRATION_PIN);  // Vibration sensor input
     PORTC |= (1 << VIBRATION_PIN);  // Enable pull-up resistor
     
     // Configure status LED
     STATUS_LED_DDR |= (1 << STATUS_LED);  // Set as output
     
     // Power-on test sequence
     status_led_blink(3);  // Blink 3 times to indicate power-on
     
     // Initialize peripherals - with increased delays
     lcd_init();
     _delay_ms(200);
     uart_init();
     
     // Display user information (permanent display)
     display_user_info();  // Moved to the correct location
     
     // Wait before sending startup message to Particle Photon
     _delay_ms(2000);  // Give the Particle Photon time to boot
     uart_send_string("PARTICLE:SYSTEM_READY");
     
     // Variables for detecting end of laundry cycle
     LaundryState state = IDLE;
     uint16_t stable_counter = 0;
     uint8_t vibration_detected = 0;
     uint16_t debug_counter = 0;  // For periodic debug messages
     
     // Main loop
     while (1) {
         vibration_detected = read_vibration();
         
         // Status LED feedback - lights when vibration detected
         if (vibration_detected) {
             STATUS_LED_PORT |= (1 << STATUS_LED);
         } else {
             STATUS_LED_PORT &= ~(1 << STATUS_LED);
         }
         
         // Debug counter for periodic UART messages
         debug_counter++;
         if (debug_counter >= 100) {  // Send debug info every ~10 seconds
             debug_counter = 0;
             uart_send_debug("Vibration", vibration_detected);
             uart_send_debug("State", state);
             uart_send_debug("Stable Counter", stable_counter);
         }
         
         switch (state) {
             case IDLE:
                 if (vibration_detected) {
                     state = ACTIVE;
                     uart_send_string("PARTICLE:CYCLE_START");
                 }
                 break;
                 
             case ACTIVE:
                 if (!vibration_detected) {
                     state = WAITING;
                     stable_counter = 0;
                 }
                 break;
                 
             case WAITING:
                 if (vibration_detected) {
                     // Vibration detected again, machine still running
                     state = ACTIVE;
                     uart_send_debug("Back to active", 0);
                 } else {
                     // Continue counting stability
                     stable_counter++;
                     
                     // After ~60 seconds without vibration, consider laundry done
                     // 600 * 100ms = 60 seconds
                     if (stable_counter > 600) {
                         state = COMPLETE;
                         uart_send_debug("Laundry complete", stable_counter);
                         
                         // Rapid blink LED pattern to indicate completion
                         for (uint8_t i = 0; i < 10; i++) {
                             STATUS_LED_PORT |= (1 << STATUS_LED);
                             _delay_ms(50);
                             STATUS_LED_PORT &= ~(1 << STATUS_LED);
                             _delay_ms(50);
                         }
                         
                         // Send notification to Particle Photon
                         notify_laundry_done();
                     }
                 }
                 break;
                 
             case COMPLETE:
                 // After waiting for 5 seconds, return to idle
                 _delay_ms(5000);
                 
                 // Quick flash to show returning to idle
                 status_led_blink(2);
                 
                 state = IDLE;  // Reset state for next cycle
                 break;
         }
         
         _delay_ms(100);  // Sample every 100ms
     }
     
     return 0;  // Never reached
 }