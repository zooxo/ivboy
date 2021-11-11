/*

  IVBOY - A Powerful FORTH-like Programable Scientific RPN Calculator for the Arduboy

  This software is covered by the 3-clause BSD license.
  See also: https://github.com/zooxo/ivboy

  ____________________

    PREAMBLE
  ____________________

  IVBOY is a port if IVEE respectively IVTINY (see https://github.com/zooxo/ivt)
  to the ARDUBOY hardware. Unfortunately the Arduboy has 6 keys only. So a good
  performance as pocket calculator depends on how to use the keys optimally.
  As the executable program has approximately 15 kilobytes (of possible 28)
  there is enough room to expand (ie complex numbers, matrices, graphics).

  So far the upper half of the screen shows always the top of the stack while
  the lower part gives hints to enter commands. Normally you are always in some
  kind of command mode and select one (of 111 possible) functions with the
  cursor keys and the execution key (B). Additionally you are able to reach
  important or user defined commands with the (shifting) key A and the cursor
  keys in a "fast" way (total 32 functions).

  ____________________

    COMMANDS
  ____________________

  Important commands: MNU 1 2 3 4 5 6 7 8 9 0 . DROP EE NEG DUP
  Basic operations:   + - / *
  Conditions:         = > < <>
  Other commands (alphabetic order):
    % %CH ABS ACOS ACSH ASIN ASNH ATAN ATNH BATT BEG C*F CA CM* COS COSH DEG
    ELSE EXP FRAC H* HMS* IF INT INV KG* KM* L* LN LN! LOCK LOG LR M*FT ND OVER
    P*R PC PI PICK POW PRG PW10 PV QE R*P RCL ROT S+ SCLR SIN SINH SOLV SQRT
    STAT STO SWAP TAN TANH THEN UNTL USR $A ... $Z

  For an explanation of most commands see: https://github.com/zooxo/ivt

  ____________________

    COMMAND MODE
  ____________________

  After switching on you are in the command mode. While the upper half of the
  screen shows the top of the stack, the lower part shows the selected command.
  On the left side you see the command number (one figure upon the other), the
  command name (maximum of 4 letters) and on the right side the name of the
  previous and next command.
  Select the next command with the cursor down key (and vice versa). To leap
  10 commands forward press the right key (and vice versa).
  To enter the selected command press the ENTER key (B).

  After the first 16 essential commands (figures, basic operations) all further
  commands are sorted alphabetically.
  After 85 commands the (26) programmed user commands ($A ... $Z) can be
  executed.

  But there is a much faster way to execute (16) important commands and even
  (16) user definable commands by pressing the SHIFT key (A).
  While pressing (and holding) A the lower screen shows a "star menu" with 4
  options corresponding to the four cursor keys. After typing the appropriate
  cursor key another "star menu" opens and gives the opportunity to execute one
  (of four) commands with the cursor keys.
  This procedure requires two fingers (hands), but by using the command LOCK one
  finger selection is possible.

  Last but not least you can change the "star menu" of important commands to a
  "star menu" of user defined commands (USER MENU). Enter the number of the
  desired command and the desired position in the user menu to the stack and
  execute the command USR.
  The position pattern of the "star menu" is:              4567
                                                        0123  890a
                                                           bcde

  ____________________

    PROGRAMMING
  ____________________

  Execute one (of maximal 26) user defined programs by selecting the appropriate
  command $A ... $Z.
  To edit a program enter the program number (ie 1 for $A) to the stack and
  execute PRG.
  Browse the program steps with cursor up/down and insert/delete a program step
  with the left/right cursor key. Please note that it is also possible to enter
  a desired command via the user menu.
  Leave and save the program editing with pressing A.

  ____________________

    SPECIAL KEYS
  ____________________

  Set the brightness (6 levels) by pressing (and holding) the left cursor key
  and pressing the upper (brighter) or lower cursor key (darker).
  Goto sleep mode with pressing (and holding) the left cursor key and pressing
  the right kursor key. To wake up the IVBOY press key A.
  Interrupt (endless) loops by pressing cursor up and down key simultaneousely.
  ____________________


----v----1----v----2----v----3----v----4----v----5----v----6----v----7----v----8
  TODOs: Integrate, Function plot, Complex Numbers, Matrices
----v----1----v----2----v----3----v----4----v----5----v----6----v----7----v----8

*/


// ***** I N C L U D E S

#include <avr/power.h> // Needed for power management
#include <avr/sleep.h>
#include <EEPROM.h> // For saving data to EEPROM


// ***** F O N T S

const byte fontnum [] PROGMEM = { // Numeric font 3x8
  0x08, 0x08, 0x00, // -
  0x80, 0x00, 0x00, // .
  0x00, 0x00, 0x00, // / space
  0xff, 0x81, 0xff, // 0
  0x82, 0xff, 0x80, // 1
  0xf9, 0x89, 0x8f, // 2
  0x89, 0x89, 0xff, // 3
  0x0f, 0x08, 0xff, // 4
  0x8f, 0x89, 0xf9, // 5
  0xff, 0x89, 0xf9, // 6
  0x03, 0x01, 0xff, // 7
  0xff, 0x89, 0xff, // 8
  0x8f, 0x89, 0xff, // 9
};

#define FW 4 // Font width
#define FO ' ' // Font offset
const byte font [] PROGMEM = { // Standard font 4x8
  0x00, 0x00, 0x00, 0x00, // space
  0x00, 0x5f, 0x00, 0x00, // !
  0x0b, 0x07, 0x0b, 0x07, // "
  0x6d, 0x11, 0x11, 0x6d, // ; # mean value
  0x2e, 0x6a, 0x2b, 0x3a, // $
  0x26, 0x12, 0x48, 0x64, // %
  0x76, 0x49, 0x76, 0x60, // &
  0x22, 0x72, 0x27, 0x22, // ' leftright
  0x7f, 0x41, 0x41, 0x00, // (
  0x00, 0x41, 0x41, 0x7f, // )
  0x24, 0x18, 0x18, 0x24, // *
  0x08, 0x1c, 0x08, 0x00, // +
  0x10, 0x7f, 0x01, 0x01, // , squareroot
  0x08, 0x08, 0x08, 0x08, // -
  0x00, 0x60, 0x60, 0x00, // .
  0x20, 0x10, 0x08, 0x04, // /
  0x7f, 0x41, 0x41, 0x7f, // 0
  0x00, 0x02, 0x7f, 0x00, // 1
  0x79, 0x49, 0x49, 0x4f, // 2
  0x41, 0x49, 0x49, 0x7f, // 3
  0x0f, 0x08, 0x08, 0x7f, // 4
  0x4f, 0x49, 0x49, 0x79, // 5
  0x7f, 0x49, 0x49, 0x78, // 6
  0x03, 0x01, 0x01, 0x7f, // 7
  0x7f, 0x49, 0x49, 0x7f, // 8
  0x0f, 0x49, 0x49, 0x7f, // 9
  0x00, 0x36, 0x36, 0x00, // :
  0x00, 0x5b, 0x3b, 0x00, // ;
  0x08, 0x14, 0x22, 0x41, // <
  0x14, 0x14, 0x14, 0x14, // =
  0x41, 0x22, 0x14, 0x08, // >
  0x03, 0x59, 0x09, 0x0f, // ?
  0x3e, 0x41, 0x5d, 0x5e, // @
  0x7f, 0x09, 0x09, 0x7f, // A
  0x7f, 0x49, 0x4f, 0x78, // B
  0x7f, 0x41, 0x41, 0x40, // C
  0x41, 0x7F, 0x41, 0x7F, // D
  0x7F, 0x49, 0x49, 0x41, // E
  0x7F, 0x09, 0x09, 0x01, // F
  0x7f, 0x41, 0x49, 0x79, // G
  0x7F, 0x08, 0x08, 0x7F, // H
  0x41, 0x7f, 0x41, 0x40, // I
  0x60, 0x40, 0x40, 0x7f, // J
  0x7F, 0x08, 0x0f, 0x78, // K
  0x7F, 0x40, 0x40, 0x40, // L
  0x7F, 0x07, 0x07, 0x7F, // M
  0x7F, 0x06, 0x0c, 0x7F, // N
  0x7f, 0x41, 0x41, 0x7f, // O
  0x7F, 0x09, 0x09, 0x0f, // P
  0x7f, 0x41, 0x61, 0x7f, // Q
  0x7F, 0x09, 0x79, 0x4f, // R
  0x4f, 0x49, 0x49, 0x78, // S
  0x01, 0x7f, 0x01, 0x01, // T
  0x7F, 0x40, 0x40, 0x7F, // U
  0x1F, 0x70, 0x70, 0x1F, // V
  0x7F, 0x70, 0x70, 0x7F, // W
  0x77, 0x08, 0x08, 0x77, // X
  0x4f, 0x48, 0x48, 0x7f, // Y
  0x71, 0x49, 0x49, 0x47, // Z
  0x02, 0x7f, 0x7f, 0x02, // [ arrow up
  0x2a, 0x1c, 0x1c, 0x2a, // backslash sun
  0x20, 0x7f, 0x7f, 0x20, // ] arrow down
  0x02, 0x01, 0x01, 0x02, // ^
  0x40, 0x40, 0x40, 0x40, // _
  0x3c, 0x20, 0x28, 0x20, // ` angle
  0x70, 0x54, 0x54, 0x7c, // a
  0x7F, 0x44, 0x44, 0x7c, // b
  0x7c, 0x44, 0x44, 0x44, // c
  0x7c, 0x44, 0x44, 0x7F, // d
  0x7c, 0x54, 0x54, 0x5c, // e
  0x04, 0x7f, 0x05, 0x01, // f
  0x5c, 0x54, 0x54, 0x7c, // g
  0x7F, 0x04, 0x04, 0x7c, // h
  0x40, 0x44, 0x7d, 0x40, // i
  0x40, 0x40, 0x44, 0x7d, // j
  0x7f, 0x10, 0x1c, 0x70, // k
  0x01, 0x7f, 0x40, 0x40, // l
  0x7C, 0x0c, 0x0c, 0x7c, // m
  0x7C, 0x04, 0x04, 0x7c, // n
  0x7c, 0x44, 0x44, 0x7c, // o
  0x7c, 0x14, 0x14, 0x1c, // p
  0x1c, 0x14, 0x14, 0x7c, // q
  0x7C, 0x04, 0x04, 0x04, // r
  0x5c, 0x54, 0x54, 0x74, // s
  0x04, 0x7F, 0x44, 0x40, // t
  0x7C, 0x40, 0x40, 0x7C, // u
  0x1c, 0x60, 0x60, 0x1c, // v
  0x7C, 0x60, 0x60, 0x7C, // w
  0x6c, 0x10, 0x10, 0x6c, // x
  0x5c, 0x50, 0x50, 0x7c, // y
  0x74, 0x54, 0x54, 0x5c, // z
  0x08, 0x1c, 0x3e, 0x7f, // { left arrow
  0x00, 0x7f, 0x00, 0x00, // |
  0x7f, 0x3e, 0x1c, 0x08, // } right arrow
  0x77, 0x5d, 0x49, 0x63, // ~ sum
  0x55, 0x2a, 0x55, 0x2a  // del grey rectangle
};


// ***** S Y S T E M

// PINS, PORTS
// Display
#define CS_PORT PORTD   // CS port
#define CS_BIT PORTD6   // CS physical bit number
#define DC_PORT PORTD   // DC port
#define DC_BIT PORTD4   // DC physical bit number
#define RST_PORT PORTD  // RST port
#define RST_BIT PORTD7  // RST physical bit number
// RGB LED
#define RED_LED 10           // Red LED pin
#define RED_LED_BIT PORTB6   // Red LED physical bit number
#define GREEN_LED 11         // Green LED pin
#define GREEN_LED_BIT PORTB7 // Green LED physical bit number
#define BLUE_LED 9           // Blue LED pin
#define BLUE_LED_BIT PORTB5  // Blue LED physical bit number
// Buttons
#define PIN_LEFT_BUTTON A2 // Left Button
#define LEFT_BUTTON_BIT PORTF5
#define PIN_RIGHT_BUTTON A1 // Right button
#define RIGHT_BUTTON_BIT PORTF6
#define PIN_UP_BUTTON A0
#define UP_BUTTON_BIT PORTF7
#define PIN_DOWN_BUTTON A3 // Down button
#define DOWN_BUTTON_BIT PORTF4
#define PIN_A_BUTTON 7 // A button
#define A_BUTTON_BIT PORTE6
#define PIN_B_BUTTON 8 // B button
#define B_BUTTON_BIT PORTB4
// Other
#define RAND_SEED_IN_BIT PORTF1
// SPI interface
#define SPI_MISO_BIT PORTB3
#define SPI_MOSI_BIT PORTB2
#define SPI_SCK_BIT PORTB1
#define SPI_SS_BIT PORTB0

static void bootpins(void) { // Declare and boot port pins
  PORTB |= _BV(RED_LED_BIT) | _BV(GREEN_LED_BIT) | _BV(BLUE_LED_BIT) | // Port B
           _BV(B_BUTTON_BIT);
  DDRB  &= ~(_BV(B_BUTTON_BIT) | _BV(SPI_MISO_BIT));
  DDRB  |= _BV(RED_LED_BIT) | _BV(GREEN_LED_BIT) | _BV(BLUE_LED_BIT) |
           _BV(SPI_MOSI_BIT) | _BV(SPI_SCK_BIT) | _BV(SPI_SS_BIT);
  PORTD |= _BV(CS_BIT); // Port D
  PORTD &= ~(_BV(RST_BIT));
  DDRD  |= _BV(RST_BIT) | _BV(CS_BIT) | _BV(DC_BIT);
  PORTE |= _BV(A_BUTTON_BIT); // Port E
  DDRE  &= ~(_BV(A_BUTTON_BIT));
  PORTF |= _BV(LEFT_BUTTON_BIT) | _BV(RIGHT_BUTTON_BIT) | // Port F
           _BV(UP_BUTTON_BIT) | _BV(DOWN_BUTTON_BIT);
  PORTF &= ~(_BV(RAND_SEED_IN_BIT));
  DDRF  &= ~(_BV(LEFT_BUTTON_BIT) | _BV(RIGHT_BUTTON_BIT) | _BV(UP_BUTTON_BIT) |
             _BV(DOWN_BUTTON_BIT) | _BV(RAND_SEED_IN_BIT));
  SPCR = _BV(SPE) | _BV(MSTR); // master, mode 0, MSB first, CPU clock / 2 (8MHz)
  SPSR = _BV(SPI2X);
}


// ***** D I S P L A Y

// DEFINES
#define EECONTRAST 0 // EEPROM address to save screencontrast
#define CONTRASTSTEP 48 // Step for in/decreasing contrast
#define FRAMERATE 15 // Maximal number of screen refreshes per second (>3)
#define SCREENWIDTH 64 // Virtual screen width (quarter of real screen)
#define SCREENBYTES 256 // Number of bytes to address screen (64 x 32)/8
#define MAXSTRBUF 10 // Maximal length of string buffer sbuf[]
#define DIGITS 8 // Number of digits when printing a number
#define ALMOSTZERO 1e-37 // Limits to decide if sci or fix
#define FIXMIN 1e-3 // Limits for fix display guarantee maximal
#define FIXMAX 1e7 // number of significant digits
#define MAXCHAR 4 // Maximal number of characters per command

// GLOBAL VARIABLES
static byte dbuf[SCREENBYTES]; // Buffer for virtual screen (costs 256 bytes of dynamic memory)
static byte eachframemillis, thisframestart, lastframedurationms; // Framing times
static boolean justrendered; // True if frame was just rendered
static char sbuf[MAXSTRBUF]; // Holds string to print
static boolean isnewnumber = true; // True if stack has to be lifted before entering a new number
static byte decimals = 0; // Number of decimals entered (input after decimal dot)
static boolean isdot = false; // True if dot was pressed and decimals will be entered

// MACROS
//#define _abs(x) ((x<0)?(-x):(x)) // abs()-substitute macro
#define _ones(x) ((x)%10)        // Calculates ones unit
#define _tens(x) (((x)/10)%10)   // Calculates tens unit
#define _huns(x) (((x)/100)%10)  // Calculates hundreds unit
#define _tsds(x) (((x)/1000)%10) // Calculates thousands unit

// SUBROUTINES
void dtransfer(byte data) { // Write to the SPI bus (display - MOSI pin)
  SPDR = data;
  asm volatile("nop"); // Tiny delay before wait
  while (!(SPSR & _BV(SPIF))) {} // Wait for byte to be sent
}

static void dcommandmode(void) { // Set display to command mode
  bitClear(DC_PORT, DC_BIT);
}
static void ddatamode(void) { // Set display to data mode
  bitSet(DC_PORT, DC_BIT);
}

const byte PROGMEM dinitsequence[] = { // SSD1306 boot sequence
  0x20, 0x00, // Adressing mode (0x00=horiz 0x01=vert 0x02=page)
  0x21, 0x00, 0x7F, // Set column start/end address
  0x22, 0x00, 0x07, // Set page start/end address
  0x40, // RAM start register 01fedcba (0-63)
  0x81, 0x00, // Set contrast (0-FF)
  0x8D, 0x14, // Charge Pump Setting (0x14=enable, 0x10=disabled)
  0xA0, // Normal/Inverse mode 1010011a (0xA6:a=0 normal, 0xA7:a=1 inverse)
  0xA1, // Segment remap - mirror (A0:col0=SEG0, A1:COL127=SEG0)
  0xA4, // Entire display on/off 1010010a (0xA4:a=0 Output follows RAM, 0xA5:a=1 Display ON)
  0xA8, 0x3F, // MUX ratio to N+1 **fedcba (N=3F/1F/F/7/3/1)
  0xAF, // Display on/off 1010111a (0xAE:a=0 off, 0xAF:a=1 on)
  0xB0, // Start address for page addressing mode (B0-B7)
  0xC8, // Set COM Output Scan Direction - flip (C0=normal, C8=remapped)
  0xD3, 0x00, // Display offset **fedcba (vertical shift 0-63)
  0xD5, 0xF1, // Osc_frequ/clock_divide_ratio hgfedcba (hgfe=oscFREQ, dcba=clockDIVratio)
  0xD9, 0x11, // Precharge period hgfedcba (hgfe=phase2, dcba=phase1)
  //0xDB, 0x30, // Vcom-deselect-level 0gfe0000 (30/20/10)
  0xDA, 0x12, // COMpin config 00fe0010 (e=seq/altPIN, f=disable/enableCOMleftrightremap)
};

static void dinit(void) { // Boot screen - reset the display
  bitSet(RST_PORT, RST_BIT); // Set reset pin high to come out of reset
  bitClear(CS_PORT, CS_BIT); // Select the display (permanently, since nothing else is using SPI)
  dcommandmode(); // Run customized boot-up command sequence
  for (uint8_t i = 0; i < sizeof(dinitsequence); i++)
    dtransfer(pgm_read_byte(dinitsequence + i));
  ddatamode();
}

static void setframerate(byte rate) { // Calculate frameduration
  eachframemillis = 1000 / rate;
}

static void dcontrast(byte c) { // Set display contrast
  dcommandmode();
  dtransfer(0x81); dtransfer(c);
  ddatamode();
  EEPROM.write(EECONTRAST, c);
}
static void dcontrastup(void) { // Increase display contrast
  byte c = EEPROM.read(EECONTRAST);
  c = (c <= 255 - CONTRASTSTEP) ? c + CONTRASTSTEP : 255;
  dcontrast(c);
}
static void dcontrastdown(void) { // Decrease display contrast
  byte c = EEPROM.read(EECONTRAST);
  c = (c >= CONTRASTSTEP) ? c - CONTRASTSTEP : 0;
  dcontrast(c);
}

static void dbufclr(void) { // Clear display buffer
  for (int i = 0; i < SCREENBYTES; i++) dbuf[i] = 0;
}

static void delayshort(byte ms) { // Delay (with timer) in ms with 8 bit duration
  long t = millis();
  while ((byte)(millis() - t) < ms) ;
}
static void delaylong(byte nr) { // Delay nr quarters of a second
  for (byte i = 0; i < nr; i++) delayshort(250);
}

static void bootpowersaving(void) {
  PRR0 = _BV(PRTWI) | _BV(PRADC); // Disable I2C-Interface and ADC
  PRR1 |= _BV(PRUSART1); // Disable USART1
  power_adc_disable();
  power_usart0_disable();
  power_twi_disable();
  power_timer1_disable(); // Disable timer 1...3 (0 is used for millis())
  power_timer2_disable();
  power_timer3_disable();
  power_usart1_disable();
}

static void screenoff(void) { // Shut down the display
  dcommandmode();
  dtransfer(0xAE); // Display off
  dtransfer(0x8D); // Disable charge pump
  dtransfer(0x10);
  delayshort(100);
  bitClear(RST_PORT, RST_BIT); // Set RST to low (reset state)
}
static void screenon(void) { // Restart the display after a displayOff()
  dinit();
  dcontrast(EEPROM.read(EECONTRAST));
}

static void idle(void) { // Idle, while waiting for next frame
  SMCR = _BV(SE); // Select idle mode and enable sleeping
  sleep_cpu();
  SMCR = 0; // Disable sleeping
}

static bool nextFrame(void) { // Wait (idle) for next frame
  byte now = (byte) millis(), framedurationms = now - thisframestart;
  if (justrendered) {
    lastframedurationms = framedurationms;
    justrendered = false;
    return false;
  }
  else if (framedurationms < eachframemillis) {
    if (++framedurationms < eachframemillis) idle();
    return false;
  }
  justrendered = true;
  thisframestart = now;
  return true;
}

static void wakeupnow() {} // Dummy wakeup code

static void sleepnow(void) { // Power down - wake up by pressing A (interrupt 4)
  ledsoff();
  screenoff(); // Display off only if screensaver didn't
  pinMode(PIN_A_BUTTON, INPUT_PULLUP);
  attachInterrupt(4, wakeupnow, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  attachInterrupt(4, wakeupnow, LOW);
  sleep_mode();
  // SLEEP ... till A (interrupt 4 on Leonardo) is pressed //
  sleep_disable();
  detachInterrupt(4);
  screenon(); // Display on
  delayshort(200);
  ledsoff();
}

static int rawadc() { // Measure Vcc
  power_adc_enable();
  ADMUX = (_BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1)); // Set voltage bits
  delayshort(2); // Wait for ADMUX setting to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // Measuring
  power_adc_disable();
  return (ADC);
}

static void flashmode(void) { //  Secure wait to flash software (UP is pressed when switching on)
  if (!digitalRead(PIN_UP_BUTTON)) {
    digitalWrite(BLUE_LED, LOW);
    while (true) {}; // Wait
  }
}
static void ledsoff(void) { // Disable LEDs
  digitalWrite(RED_LED, HIGH); digitalWrite(GREEN_LED, HIGH); digitalWrite(BLUE_LED, HIGH);
  TXLED1; RXLED1;
}

static double pow10(int8_t e) { // Calculates 10 raised to the power of e
  double f = 1.0F;
  if (e > 0) while (e--) f *= 10.0F;
  else while (e++) f /= 10.0F;
  return (f);
}

static byte bitpos(byte n) { // Find position of rightmost bit set
  n &= ~0x02;
  byte i = 1, pos = 1;
  while (!(i & n)) {
    i = i << 1;
    pos++;
  }
  return (pos);
}

static byte expand4bit(byte b) { // 0000abcd  Expand 4 bits (lower nibble)
  b = (b | (b << 2)) & 0x33;     // 00ab00cd
  b = (b | (b << 1)) & 0x55;     // 0a0b0c0d
  return (b | (b << 1));         // aabbccdd
}

static void ddisplay(void) { // Print display buffer (64x32) to real screen (128x64)
  for (byte l = 0; l < 4; l++) { // Four lines
    for (byte k = 0; k < 2; k++) { // Double height
      for (byte j = 0; j < SCREENWIDTH; j++) {
        byte tmp = expand4bit((dbuf[j + l * SCREENWIDTH] >> (k * 4)) & 0x0f); // Expand 0000abcd
        for (byte i = 0; i < 2; i++) dtransfer(tmp); // Double width
      }
    }
  }
}

static void printcat(byte ch, int x, int y, byte shift) { // Print character
  for (int i = 0; i < 4; i++) {
    byte tmp = pgm_read_byte(&font[4 * (ch - ' ') + i]);
    if (shift < 8) {
      dbuf[y * SCREENWIDTH + x + i] = tmp << shift; // Upper part
      if (shift > 1) dbuf[(y + 1) * SCREENWIDTH + x + i] = tmp >> (8 - shift); // Lower part
    }
    //if (shift > 8) dbuf[(y + 1) * SCREENWIDTH + x + i] = tmp << 1;
    else dbuf[(y + 1) * SCREENWIDTH + x + i] = tmp << 1; // Shift one (without 2nd part)
  }
}
static void printsat(char* s, int x, int y, byte shift) { // Print string
  for (byte i = 0; i < strlen(s); i++) printcat(s[i], x + 5 * i, y, shift);
}
static void printbigcat(byte c, byte x) { // Print big character
  byte y = 0;
  byte height = 2, width = 2;
  for (byte k = 0; k < height; k++) { // Height (1 or 2)
    for (int i = 0; i < FW; i++) {
      byte tmp = pgm_read_byte(font + FW * (c - FO) + i);
      if (height == 2) tmp = expand4bit(tmp >> (4 * k) & 0x0f);
      for (byte j = 0; j < width; j++) dbuf[x + (width * i + j) + (y + k) * SCREENWIDTH] = tmp;
    }
  }
}
static void printbigsat(char* s) { // Print big string
  for (byte i = 0; i < strlen(s); i++) {
    printbigcat(s[i], (2 * FW + 1) * i + (i > 2 ? 2 : 0));
  }
}
static void printnat(byte ch, int x) { // Print number character
  for (byte k = 0; k < 2; k++) { // Two lines
    for (byte j = 0; j < 3; j++) {
      byte tmp = expand4bit((pgm_read_byte(&fontnum[3 * (ch - '-') + j]) >> (k * 4)) & 0x0f); // Expand 0000abcd
      for (byte i = 0; i < 2; i++)
        if (x + 2 * j + i < 64)
          dbuf[k * SCREENWIDTH + x + 2 * j + i] = tmp;
    }
  }
}

static void sbufclr(void) { // Clear sbuf
  for (byte i = 0; i < sizeof(sbuf); i++) sbuf[i] = '/';
}

static void printnumsbuf(byte digits) { // Print sbuf as number (shrink ".")
  byte xshift = 0;
  for (byte i = 0; i < digits; i++) {
    printnat(sbuf[i], 8 * i - xshift);
    if (sbuf[i] == '.') xshift += 4; // Small space for '.'
    if (sbuf[i] == '-' || sbuf[i] == '/') xshift += 2; // Small space for '-' and ' '
  }
}
static void printcmdsbuf(void) { // Print sbuf as command
  byte len = strlen(sbuf);
  byte x = FW + 1 + (MAXCHAR - len) * 10 / 2; // Centered
  for (byte l = 0; l < len; l++) { // Every character of sbuf
    for (byte k = 0; k < 2; k++) { // Two lines
      for (byte j = 0; j < 4; j++) {
        byte tmp = expand4bit((pgm_read_byte(&font[FW * (sbuf[l] - ' ') + j]) << 1 >> (k * 4)) & 0x0f); // Expand 0000abcd
        //for (byte i = 0; i < 2; i++) dbuf[(2 + k) * SCREENWIDTH + 2 * 5 * l  + 2 * j + i] = tmp;
        for (byte i = 0; i < 2; i++) dbuf[(2 + k) * SCREENWIDTH + 2 * 5 * l  + 2 * j + i + x] = tmp;
      }
    }
  }
}

static void printnum(double f) { // Print number
  int8_t ee = 0; // Fixed format
  int8_t e = 1; // Exponent
  long m; // Mantissa
  sbufclr();
  if (f < 0.0F) { // # Manage sign
    f = - f;
    sbuf[0] = '-';
  }
  if (f >= ALMOSTZERO && (f < FIXMIN || f >= FIXMAX)) { // # SCI format
    ee = log10(f); // Exponent
    if (ee < 0) ee--;
    f /= pow10(ee);
  }
  if (f >= 1.0F) e = log10(f) + 1; // Calculate new exponent if (f !< 1)
  double a = pow10(7 - e); // # Calculate pre dot
  double d = (f * a + 0.5) / a; // Rounding
  m = d;
  for (byte i = e; i > 0; i--) {
    sbuf[i] = _ones(m) + '0';
    m /= 10;
  }
  sbuf[e + 1] = '.';
  if ((long)f >= (long)d) d = f; // # Calculate after dot (and suppress trailing zeros)
  m = (d - (long)d) * a + 0.5;
  boolean istrail = true;
  for (byte i = DIGITS; i > e + 1; i--) {
    byte one = _ones(m);
    if (!istrail || ((isnewnumber || i - e - 1 <= decimals) && (!isnewnumber || one != 0))) {
      sbuf[i] = one + '0'; // Assign digit
      istrail = false; // End of trailing zeros
    }
    m /= 10L;
  }
  if (ee) { // # Scientific exponent if applicable
    sbuf[6] = (ee < 0) ? '-' : '/';
    if (ee < 0) ee = -ee;
    sbuf[8] = _ones(ee) + '0';
    sbuf[7] = _tens(ee) + '0';
  }
  printnumsbuf(9);
}


// ***** B U T T O N S

#define B_A 0x01 // Bit of A
#define B_B 0x02 // Bit of B
#define B_L 0x04 // Bit of L
#define B_U 0x08 // Bit of U
#define B_R 0x10 // Bit of R
#define B_D 0x20 // Bit of D

static byte buttonscan(void) { // Scan buttons
  byte b = 0;
  if (!digitalRead(PIN_B_BUTTON)) b |= B_A; // Shift key
  if (!digitalRead(PIN_A_BUTTON)) b |= B_B;
  if (!digitalRead(PIN_LEFT_BUTTON)) b |= B_L;
  if (!digitalRead(PIN_UP_BUTTON)) b |= B_U;
  if (!digitalRead(PIN_RIGHT_BUTTON)) b |= B_R;
  if (!digitalRead(PIN_DOWN_BUTTON)) b |= B_D;
  return (b);
}


// ***** A P P L I C A T I O N

// Defines
#define RAD (180.0F/PI) // 180/PI ... used for _atan and _cos
#define CMDSTEP 10 // Page up/down step if in command mode
#define MAXCMDU 80 // End of user commands

// EEPROM dimensions and addresses
#define EELOCK 1 // EEPROM address for shift lock mode
#define EESTO 2 // Memories (MEMSTOx4 bytes)
#define MEMSTO 16 // Number of memories
#define EEMENU 66 // EEPROM address for user definable menu keys
#define MENUNR 16 // Number of definable menu keys
#define EEUSTART 83 // User programs
#define EEUEND   1024
#define EEU (EEUEND-EEUSTART) // Available user memory
#define PRGMAXNR 26 // Maximal number of user programs (A~Z)
#define PRGMAXLEN (EEU/PRGMAXNR) // Maximal length of user program

// GLOBAL VARIABLES
static byte key, oldkey; // Holds entered and old key (prevent keyrepeat)
static unsigned int mp; // MEMPOINTER (builtin and user functions)
static boolean islock = false; // Locks shift key
static boolean isprintscreen = true; // True, if screen should be printed
static byte numlevel = 0, keymult = 0;
static byte cmdselect = 0; // Selected command
static byte executedcommand; // Dispatched command
static boolean ismenu = false; // Select user menu
static boolean isprgedit = false; // True, if program is edited
static boolean isprgins = false, isshiftprgins = false; // True if program step should be inserted

static byte prgnr; // Number of user program edited
static int prgaddr; // Address of recent program in EEPROM (includes EEUSTART)
static byte prgpos; // Position in edited program
static byte prglength; // Size of program in program buffer
static byte prgbuf[PRGMAXLEN]; // Program buffer

#define DELTAX 1E-4 // Delta for solver
static byte runs; // Solver cycle runs
static boolean issolve = false; // True if solving is demanded


#define DATASTACKSIZE 24 // DATA STACK
double ds[DATASTACKSIZE];
static byte dp = 0;

#define ADDRSTACKSIZE 24 // ADDRESS STACK
static int as[ADDRSTACKSIZE];
static byte ap = 0;

byte cl = 0; // CONDITIONAL LEVEL


// Command code defines
//#define _0 0 // Intrinsic commands
#define _1 1
#define _2 2
#define _3 3
#define _4 4
#define _5 5
#define _6 6
#define _7 7
#define _8 8
#define _9 9
#define _0 10
#define _DOT 11
#define _DROP 12
#define _EE 13
#define _NEG 14
#define _DUP 15
#define _ADD 16
#define _SUB 17
#define _MULT 18
#define _DIV 19
#define _LT 22
#define _NE 23
#define _PCT 24
#define _PCTCHG 25
#define _ABS 26
#define _ACOS 27
#define _ACOSH 28
#define _ASIN 29
#define _ASINH 30
#define _ATAN 31
#define _ATANH 32
#define _BEGIN 34
#define _C2F 35
#define _CM2IN 37
#define _COS 38
#define _COSH 39
#define _DEG2RAD 40
#define _ELSE 41
#define _EXP 42
#define _FRAC 43
#define _H2HMS 44
#define _HMS2H 45
#define _IF 46
#define _INT 47
#define _INV 48
#define _KG2LBS 49
#define _KM2MI 50
#define _L2GAL 51
#define _LN 52
#define _LNGAMMA 53
#define _LR 54
#define _LOG 55
#define _M2FT 57
#define _ND 58
#define _OVER 59
#define _P2R 60
#define _PC 61
#define _PI 62
#define _PICK 63
#define _POW 64
#define _POW10 65
#define _PV 67
#define _QE 68
#define _R2P 69
#define _RCL 70
#define _ROT 71
#define _SADD 72
#define _SCLR 73
#define _SIN 74
#define _SINH 75
#define _SQRT 77
#define _STAT 78
#define _STO 79
#define _SWAP 80
#define _TAN 81
#define _TANH 82
#define _THEN 83
#define _UNTIL 84
#define _END 255 // Function delimiter

// Builtin functions (table)
const byte prgtable[] PROGMEM = {
  _PCT, _PCTCHG, _ABS, _ACOS, _ACOSH, _ASIN, _ASINH, _ATANH, _C2F, _CM2IN, // 0
  _COSH, _DEG2RAD, _FRAC, _H2HMS, _HMS2H, _KG2LBS, _KM2MI, _L2GAL, _LNGAMMA, _LOG, // 10
  _LR, _M2FT, _ND, _OVER, _P2R, _PC, _POW, _POW10, _PV, _QE, // 20
  _R2P, _SIN, _SINH, _SQRT, _SADD, _SCLR, _STAT, _TAN, _TANH // 30
};

// Builtin functions (mem)
const byte mem[] PROGMEM = {
  _END, // Necessary to prevent function starting with mp = 0

  _OVER, _SWAP, _DIV, _1, _0, _0, _MULT, _END, //0 % =x/B*100%

  _OVER, _SUB, _OVER, _DIV, _1, _0, _0, _MULT, _END, //1 %CHG =(x-B)/B*100%

  _DUP, _0, _LT, _IF, _NEG, _THEN, _END, //2 ABS

  _DUP, _MULT, _INV, _1, _SUB, _SQRT, _ATAN, _END, //3 ACOS =atan(sqrt(1/x/x-1))

  _DUP, _DUP, _MULT, _1, _SUB, _SQRT, _ADD, _LN, _END, //4 ACOSH =ln(x+sqrt(x*x-1))

  _DUP, _MULT, _INV, _1, _SUB, _SQRT, _INV, _ATAN, _END, //5 ASIN =atan(1/(sqrt(1/x/x-1))

  _DUP, _DUP, _MULT, _1, _ADD, _SQRT, _ADD, _LN, _END, //6 ASINH =ln(x + sqrt(x * x + 1))

  _DUP, _1, _ADD, _SWAP, _NEG, _1, _ADD, _DIV, _SQRT, _LN, _END, //7 ATANH =ln(sqrt((1 + x) / (1 - x)))

  _DUP, _1, _DOT, _8, _MULT, _3, _2, _ADD, _SWAP, _3, _2, _SUB, _1, _DOT, _8, _DIV, _END, //8 C<>F

  _DUP, _2, _DOT, _5, _4, _DUP, _DUP, _ROT, _SWAP, _DIV, _ROT, _ROT, _MULT, _END, //9 CM<>IN

  _EXP, _DUP, _INV, _ADD, _2, _DIV, _END, //10 COSH  =(exp(x)+exp(-x))/2

  _DUP, _PI, _MULT, _1, _8, _0, _DIV, _SWAP, _1, _8, _0, _MULT, _PI, _DIV, _END, //11 DEG<>RAD

  _DUP, _INT, _SUB, _END, //12 FRAC

  _DUP, _3, _6, _0, _0, _MULT, _DUP, _ROT, _INT, //13 H2HMS - h->s
  _SWAP, _OVER, _3, _6, _0, _0, _MULT, _SUB, _6, _0, _DIV, _INT, // hh mm
  _ROT, _OVER, _6, _0, _MULT, _SUB, _3, _PICK, _3, _6, _0, _0, _MULT, _SUB, // ss
  _1, _0, _0, _0, _0, _DIV, _SWAP, _1, _0, _0, _DIV, _ADD, _ADD, _END, // hh.mmss

  _DOT, _0, _0, _0, _0, _0, _1, _ADD,//14 HMS2H - round up to prevent leaps
  _DUP, _DUP, _INT, _SWAP, _OVER, _SUB, _1, _0, _0, _MULT, _INT, // hh mm
  _ROT, _3, _PICK, _SUB, _1, _0, _0, _MULT, _OVER, _SUB, _1, _0, _0, _MULT, // ss
  _3, _6, _0, _0, _DIV, _SWAP, _6, _0, _DIV, _ADD, _ADD, _END,// ->s ->h

  _DUP, _2, _DOT, _2, _0, _4, _6, _2, _3, _DUP, _DUP, _ROT, _MULT, _ROT, _ROT, _DIV, _END, //15 KG<>LBS

  _DUP, _1, _DOT, _6, _0, _9, _3, _4, _4, _DUP, _DUP, _ROT, _SWAP, _DIV, _ROT, _ROT, _MULT, _END, //16 KM<>MI

  _DUP, _3, _DOT, _7, _8, _5, _4, _1, _2, _DUP, _DUP, _ROT, _SWAP, _DIV, _ROT, _ROT, _MULT, _END, //17 L<>GAL


  _1, _ADD, _DUP, _DUP, _DUP, _DUP, _1, _2, _MULT, //18 LNGAMMA: ln!=(ln(2*PI)-ln(x))/2+x*(ln(x+1/(12*x-1/10/x))-1)
  _SWAP, _1, _0, _MULT, _INV, _SUB, _INV, _ADD, _LN, _1, _SUB, _MULT,
  _SWAP, _LN, _NEG, _2, _PI, _MULT, _LN, _ADD, _2, _DIV, _ADD, _END,

  _LN, _1, _0, _LN, _DIV, _END, //19 LOG log(z)=ln(z)/ln(10)

  _6, _RCL, _7, _RCL, _MULT, _8, _RCL, _9, _RCL, _MULT, _SUB, //20 LR - a
  _5, _RCL, _7, _RCL, _MULT, _8, _RCL, _DUP, _MULT, _SUB, _DIV,
  _DUP, _8, _RCL, _MULT, _NEG, _9, _RCL, _ADD, _7, _RCL, _DIV, _SWAP, _END, // b

  _DUP, _3, _DOT, _3, _7, _0, _0, _7, _9, _DUP, _DUP, _ROT, _MULT, _ROT, _ROT, _DIV, _END, //21 M<>FT

  _DUP, _DUP, _DUP, _DUP, _MULT, _MULT, _DOT, _0, _7, _MULT, //22 ND
  _SWAP, _1, _DOT, _6, _MULT, _NEG, _ADD, _EXP, _1, _ADD, _INV, _SWAP, //CDF ~ 1/(1+exp(-0.07*x^3-1.6*x))
  _DUP, _MULT, _NEG, _2, _DIV, _EXP, _2, _PI, _MULT, _SQRT, _INV, _MULT, _END, //PDF = exp(-x*x/2)/sqrt(2*PI)

  _SWAP, _DUP, _ROT, _ROT, _END, //23 OVER

  _DUP, _ROT, _DUP, _COS, _SWAP, _SIN, _ROT, _MULT, _ROT, _ROT, _MULT, _END, //24 P>R y=r*sin(a) x=r*cos(a)

  _DUP, _ROT, _SWAP, //25 PERM COMB
  _OVER, _ROT, _ROT, _SUB, _1, _ROT, _ROT, _SWAP, // PERM
  _BEGIN, _SWAP, _ROT, _1, _ROT, _ADD, _DUP, _ROT, _MULT,
  _SWAP, _ROT, _OVER, _OVER, _SWAP, _LT, _UNTIL, _DROP, _DROP,
  _DUP, _ROT, _1, _SWAP, // COMB
  _BEGIN, _ROT, _ROT, _DUP, _ROT, _SWAP, _DIV,
  _ROT, _ROT, _1, _ADD, _SWAP, _OVER, _1, _SUB,
  _OVER, _SWAP, _LT, _UNTIL, _DROP, _DROP, _END,

  _SWAP, _LN, _MULT, _EXP, _END, //26 POW a^b=exp(b*ln(a))

  _1, _SWAP, _EE, _END, //27 POW10

  _OVER, _1, _ADD, _SWAP, _POW, _DUP, _1, _SUB, _SWAP, _DIV, _SWAP, _DIV, _END, //28 PV PV(i,n)=((1+i)^n-1)/(1+i)^n/i

  _OVER, _2, _DIV, _DUP, _MULT, _SWAP, _SUB, _SQRT, _SWAP, _2, _DIV, _NEG, _SWAP, //29 QE x12=-p/2+-sqrt(p*p/4-q)
  _OVER, _OVER, _SUB, _ROT, _ROT, _ADD, _END,

  _DUP, _MULT, _SWAP, _DUP, _MULT, _DUP, _ROT, _DUP, _ROT, _ADD, _SQRT, //30 R>P r=sqrt(x*x+y*y) a=atan(y/x)
  _ROT, _ROT, _DIV, _SQRT, _ATAN, _SWAP, _END,

  _9, _0, _SWAP, _SUB, _COS, _END, //31 SIN =cos(90-x)

  _EXP, _DUP, _INV, _NEG, _ADD, _2, _DIV, _END, //32 SINH =(exp(x)-exp(-x))/2

  _DUP, _0, _NE, _IF, _LN, _2, _DIV, _EXP, _THEN, _END, //33 SQRT =exp(ln(x)/2)

  _7, _RCL, _1, _ADD, _7, _STO, //34 SUMADD - n
  _DUP, _8, _RCL, _ADD, _8, _STO, // X
  _DUP, _DUP, _MULT, _5, _RCL, _ADD, _5, _STO, // XX
  _OVER, _MULT, _6, _RCL, _ADD, _6, _STO, // XY
  _9, _RCL, _ADD, _9, _STO, _7, _RCL, _END, // Y push(n)

  _0, _DUP, _DUP, _DUP, _DUP, _DUP, //35 SUMCLR
  _5, _STO, _6, _STO, _7, _STO, _8, _STO, _9, _STO, _END,

  _8, _RCL, _7, _RCL, _DIV, //36 STAT - mean (X/n)
  _DUP, _DUP, _MULT, _7, _RCL, _MULT, _NEG, _5, _RCL, _ADD, // stddev (XX-n*m^2)/(n-1)
  _7, _RCL, _1, _SUB, _DIV, _SQRT, _SWAP, _END,

  _DUP, _SIN, _SWAP, _COS, _DIV, _END, //37 TAN =sin/cos

  _DUP, _SINH, _SWAP, _COSH, _DIV, _END, //38 TANH =sinh/cosh

};



// Command names
const char c0[] PROGMEM = "MNU"; //      PRIMARY KEYS
const char c1[] PROGMEM = "1";
const char c2[] PROGMEM = "2";
const char c3[] PROGMEM = "3";
const char c4[] PROGMEM = "4";
const char c5[] PROGMEM = "5";
const char c6[] PROGMEM = "6";
const char c7[] PROGMEM = "7";
const char c8[] PROGMEM = "8";
const char c9[] PROGMEM = "9";
const char c10[] PROGMEM = "0";
const char c11[] PROGMEM = ".";
const char c12[] PROGMEM = "DROP";
const char c13[] PROGMEM = "EE";
const char c14[] PROGMEM = "NEG";
const char c15[] PROGMEM = "DUP";
const char c16[] PROGMEM = "+";
const char c17[] PROGMEM = "-";
const char c18[] PROGMEM = "*";
const char c19[] PROGMEM = "/";
const char c20[] PROGMEM = "=";
const char c21[] PROGMEM = ">";
const char c22[] PROGMEM = "<";
const char c23[] PROGMEM = "<>";
const char c24[] PROGMEM = "%";
const char c25[] PROGMEM = "%CH";
const char c26[] PROGMEM = "ABS";
const char c27[] PROGMEM = "ACOS";
const char c28[] PROGMEM = "ACSH";
const char c29[] PROGMEM = "ASIN";
const char c30[] PROGMEM = "ASNH";
const char c31[] PROGMEM = "ATAN";
const char c32[] PROGMEM = "ATNH";
const char c33[] PROGMEM = "BATT";
const char c34[] PROGMEM = "BEG";
const char c35[] PROGMEM = "C'F";
const char c36[] PROGMEM = "CA";
const char c37[] PROGMEM = "CM'";
const char c38[] PROGMEM = "COS";
const char c39[] PROGMEM = "COSH";
const char c40[] PROGMEM = "DEG'";
const char c41[] PROGMEM = "ELSE";
const char c42[] PROGMEM = "EXP";
const char c43[] PROGMEM = "FRAC";
const char c44[] PROGMEM = "H'";
const char c45[] PROGMEM = "HMS'";
const char c46[] PROGMEM = "IF";
const char c47[] PROGMEM = "INT";
const char c48[] PROGMEM = "INV";
const char c49[] PROGMEM = "KG'";
const char c50[] PROGMEM = "KM'";
const char c51[] PROGMEM = "L'";
const char c52[] PROGMEM = "LN";
const char c53[] PROGMEM = "LN!";
const char c54[] PROGMEM = "LOCK";
const char c55[] PROGMEM = "LOG";
const char c56[] PROGMEM = "LR";
const char c57[] PROGMEM = "M'FT";
const char c58[] PROGMEM = "ND";
const char c59[] PROGMEM = "OVER";
const char c60[] PROGMEM = "P}R";
const char c61[] PROGMEM = "PC";
const char c62[] PROGMEM = "PI";
const char c63[] PROGMEM = "PICK";
const char c64[] PROGMEM = "POW";
const char c65[] PROGMEM = "PRG";
const char c66[] PROGMEM = "PW10";
const char c67[] PROGMEM = "PV";
const char c68[] PROGMEM = "QE";
const char c69[] PROGMEM = "R}P";
const char c70[] PROGMEM = "RCL";
const char c71[] PROGMEM = "ROT";
const char c72[] PROGMEM = "S+";
const char c73[] PROGMEM = "SCLR";
const char c74[] PROGMEM = "SIN";
const char c75[] PROGMEM = "SINH";
const char c76[] PROGMEM = "SOLV";
const char c77[] PROGMEM = "SQRT";
const char c78[] PROGMEM = "STAT";
const char c79[] PROGMEM = "STO";
const char c80[] PROGMEM = "SWAP";
const char c81[] PROGMEM = "TAN";
const char c82[] PROGMEM = "TANH";
const char c83[] PROGMEM = "THEN";
const char c84[] PROGMEM = "UNTL";
const char c85[] PROGMEM = "USR";
const char c86[] PROGMEM = "$A";
const char c87[] PROGMEM = "$B";
const char c88[] PROGMEM = "$C";
const char c89[] PROGMEM = "$D";
const char c90[] PROGMEM = "$E";
const char c91[] PROGMEM = "$F";
const char c92[] PROGMEM = "$G";
const char c93[] PROGMEM = "$H";
const char c94[] PROGMEM = "$I";
const char c95[] PROGMEM = "$J";
const char c96[] PROGMEM = "$K";
const char c97[] PROGMEM = "$L";
const char c98[] PROGMEM = "$M";
const char c99[] PROGMEM = "$N";
const char c100[] PROGMEM = "$O";
const char c101[] PROGMEM = "$P";
const char c102[] PROGMEM = "$Q";
const char c103[] PROGMEM = "$R";
const char c104[] PROGMEM = "$S";
const char c105[] PROGMEM = "$T";
const char c106[] PROGMEM = "$U";
const char c107[] PROGMEM = "$V";
const char c108[] PROGMEM = "$W";
const char c109[] PROGMEM = "$X";
const char c110[] PROGMEM = "$Y";
const char c111[] PROGMEM = "$Z";

const char* const cmd[] PROGMEM = {
  c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20,
  c21, c22, c23, c24, c25, c26, c27, c28, c29, c30, c31, c32, c33, c34, c35, c36, c37, c38, c39, c40,
  c41, c42, c43, c44, c45, c46, c47, c48, c49, c50, c51, c52, c53, c54, c55, c56, c57, c58, c59, c60,
  c61, c62, c63, c64, c65, c66, c67, c68, c69, c70, c71, c72, c73, c74, c75, c76, c77, c78, c79, c80,
  c81, c82, c83, c84, c85, c86, c87, c88, c89, c90, c91, c92, c93, c94, c95, c96, c97, c98, c99, c100,
  c101, c102, c103, c104, c105, c106, c107, c108, c109, c110, c111
};
byte maxcmd = sizeof(cmd) / sizeof(int); // Number of commands


// FUNCTION POINTER ARRAY
static void (*dispatch[])(void) = { // Function pointer array
  &_menu, &_num, &_num, &_num,  &_num, &_num, &_num, &_num, // 00
  &_num, &_num, &_num, &_dot, &_drop, &_ee, &_neg, &_dup,
  &_add, &_sub, &_mul, &_div, &_condeq, &_condgt, &_condlt, &_condne, // 16

  &_pct, &_pctchg, &_abs, &_acos, &_acosh, &_asin, &_asinh, &_atan,
  &_atanh, &_batt, &_begin, &_c2f, &_ca, &_cm2in, &_cos, &_cosh, // 32
  &_deg2rad, &_else, &_exp, &_frac, &_h2hms, &_hms2h, &_if, &_int,
  &_inv, &_kg2lbs, &_km2mi, &_l2gal, &_ln, &_lngamma, &_lock, &_log, // 48
  &_lr, &_m2ft, &_nd, &_over, &_p2r, &_pc, &_pi, &_pick,
  &_pow, &_prg, &_pow10,  &_pv, &_qe, &_r2p, &_rcl, &_rot, // 64
  &_sadd, &_sclr, &_sin, &_sinh, &_solve, &_sqrt, &_stat, &_sto,
  &_swap, &_tan, &_tanh, &_then, &_until, &_usrset,

  &_usr, &_usr, &_usr, &_usr, &_usr, &_usr, &_usr, &_usr, &_usr, &_usr,
  &_usr, &_usr, &_usr, &_usr, &_usr, &_usr, &_usr, &_usr, &_usr, &_usr,
  &_usr, &_usr, &_usr, &_usr, &_usr, &_usr,

};
static void _nop(void) {} // NOP - no operation
static void _pct(void) { // %
  seekmem(_PCT);
}
static void _pctchg(void) { // %CHG
  seekmem(_PCTCHG);
}
static void _abs(void) { // ABS
  seekmem(_ABS);
}
static void _acos(void) { // ACOS
  seekmem(_ACOS);
}
static void _acosh(void) { // ACOSH
  seekmem(_ACOSH);
}
static void _add(void) { // ADD +
  dpush(dpop() + dpop());
}
static void _asin(void) { // ASIN
  seekmem(_ASIN);
}
static void _asinh(void) { // ASINH
  seekmem(_ASINH);
}
static void _atan(void) { // ATAN
  dpush(atan(dpop()) * RAD);
}
static void _atanh(void) { // ATANH
  seekmem(_ATANH);
}
static void _batt(void) { // BATT
  dpush((double)(1126400L / rawadc()) / 1000.0);
}
static void _begin(void) { // BEGIN
  apush(mp);
}
static void _c2f(void) { // C<>F
  seekmem(_C2F);
}
static void _ca(void) { // CA
  dp = 0;
  _sclr();
}
static void _cm2in(void) { // CM<>IN
  seekmem(_CM2IN);
}
static void _ce(void) { // CE
  if (isdot) {
    if (decimals) {
      decimals--;
      double a = pow10(decimals);
      dpush(((long)(dpop() * a) / a));
    }
    else isdot = false;
  }
  else {
    long a = dpop() / 10.0F;
    //int64_t a = dpop() / 10.0F; // costs 350 bytes
    if (!a) isnewnumber = true;
    else dpush(a);
  }
}
static void _condeq(void) { // CONDITION =
  dpush(dpop() == dpop());
}
static void _condgt(void) { // CONDITION >
  dpush(dpop() < dpop());
}
static void _condlt(void) { // CONDITION <
  _condgt();
  dpush(!dpop());
}
static void _condne(void) { // CONDITION <>
  _condeq();
  dpush(!dpop());
}
static void _condseek(void) { // CONDITION - seek next ELSE or THEN
  boolean isloop = true;
  byte cltmp = 0; // Local conditional level
  while (isloop) {
    byte c = 0;
    if (mp < sizeof(mem)) c = pgm_read_byte(mem + mp++); // Builtin
    else if (mp < sizeof(mem) + EEU) c = EEPROM[mp++ -sizeof(mem) + EEUSTART];
    if (mp >= sizeof(mem) + EEU) isloop = false; // No corresponding ELSE or THEN
    else if (c == _IF) cltmp++; // Nested IF found
    else if (cltmp && c == _THEN) cltmp--; // Nested IF ended
    else if (!cltmp && (c == _ELSE || c == _THEN)) isloop = false;
  }
}
static void _cos(void) { // COS
  dpush(cos(dpop() / RAD));
}
static void _cosh(void) { // COSH
  seekmem(_COSH);
}
static void _deg2rad(void) { // DEG<>RAD
  seekmem(_DEG2RAD);
}
static void _div(void) { // DIV /
  _inv(); _mul();
}
static void _dot(void) { // DOT .
  if (isnewnumber) {
    dpush(0.0F); // Start new number with 0
    decimals = 0; isnewnumber = false;
  }
  isdot = true;
}
static void _drop(void) { // DROP
  if (!isnewnumber) _ce(); // Clear entry
  else if (dp) dp--; // Clear TOS
}
static void _dup(void) { // DUP
  if (isnewnumber && dp) dpush(ds[dp - 1]);
  else stacklift();
}
static void _ee(void) { // EE
  dpush(pow10(dpop())); _mul();
}
static void _else(void) { // ELSE
  _condseek(); // Seek next THEN
  cl--;
}
static void _exp(void) { // EXP
  boolean isneg = false; // True for negative x
  if (dpush(dpop()) < 0.0F) { // Taylor series ist weak for negative x ... calculate exp(-x)=1/exp(x)
    _neg();
    isneg = true;
  }
  dpush(1.0F); // Stack: x res
  for (byte i = 255; i; i--) {
    _swap(); _dup(); _rot(); // Duplicate x (TOS-1)
    dpush(i); _div(); _mul(); dpush(1.0F); _add(); // res = 1.0 + x * res / i;
  }
  if (isneg) _inv(); // Negative argument
  _swap(); _drop(); // Remove x (TOS-1)
}
static void _frac(void) { // FRAC
  seekmem(_FRAC);
}
static void _h2hms(void) { // H>HMS
  seekmem(_H2HMS);
}
static void _hms2h(void) { // HMS>H
  seekmem(_HMS2H);
}
static void _if(void) { // IF
  cl++; // Increment conditional level
  if (!dpop()) _condseek(); // FALSE-Clause - seek next ELSE or THEN
}
static void _int(void) { // INT
  dpush((long)dpop());
}
static void _inv(void) { // INV
  dpush(1.0F / dpop());
}
static void _kg2lbs(void) { // KG>LBS
  seekmem(_KG2LBS);
}
static void _km2mi(void) { // KM>MI
  seekmem(_KM2MI);
}
static void _l2gal(void) { // L>GAL
  seekmem(_L2GAL);
}
static void _ln(void) { // LN
  dpush(log(dpop()));
}
static void _lngamma(void) { // LNGAMMA
  seekmem(_LNGAMMA);
}
static void _lock(void) { // LOCK
  islock = islock ? false : true;
  numlevel = 0;
}
static void _log(void) { // LOG
  seekmem(_LOG);
}
static void _lr(void) { // LR
  seekmem(_LR);
}
static void _m2ft(void) { // M>FT
  seekmem(_M2FT);
}
static void _menu(void) { // MENU
  ismenu = ismenu ? false : true;
}
static void _mul(void) { // MULT *
  dpush(dpop()*dpop());
}
static void _nd(void) { // ND
  seekmem(_ND);
}
static void _neg(void) { // NEGATE
  dpush(-dpop());
}
static void _num(void) { // Insert number
  if (executedcommand == 10) executedcommand = 0; // Detour 0
  _numinput(executedcommand);
}
static void _numinput(byte k) { // NUM Numeric input (0...9)
  if (isdot) { // Append decimal
    dpush(k); dpush(pow10(++decimals)); _div(); _add();
  }
  else if (isnewnumber) dpush(k); // Push new numeral
  else { // Append numeral
    dpush(10.0F); _mul();
    dpush(k); _add();
  }
  isnewnumber = false;
}
static void _over(void) { // OVER
  seekmem(_OVER);
}
void _p2r(void) { // P2R
  seekmem(_P2R);
}
void _pc(void) { // PC PERMCOMB
  seekmem(_PC);
}
static void _pi(void) { // PI
  dpush(PI);
}
static void _pick(void) { // PICK
  byte n = dpop();
  if (n >= 1 && n <= dp) dpush(ds[dp - n]);
}
void _pow10(void) { // POW10
  seekmem(_POW10);
}
void _pow(void) { // POWER
  seekmem(_POW);
}
static void _prg(void) { // PRG
  if ((prgnr = dpop() - 1) >= PRGMAXNR) prgnr = 0;
  prgpos = prglength = 0;
  prgaddr = EEUSTART + prgnr * PRGMAXLEN;
  byte prgstep = EEPROM[prgaddr]; // First step
  while (prglength < PRGMAXLEN && prgstep != _END) {
    prgbuf[prglength] = prgstep; // Load step from EEPROM to prgbuf
    prgstep = EEPROM[prgaddr + ++prglength]; // Next step
  }
  isprgedit = true;
}
void _pv(void) { // PV PRESENT VALUE
  seekmem(_PV);
}
void _qe(void) { // QE QUADRATIC EQUATION
  seekmem(_QE);
}
void _r2p(void) { // R2P
  seekmem(_R2P);
}
static void _rcl(void) { // RCL
  _storcl(false);
}
static void _rot(void) { // ROT
  if (dp > 2) {
    double a = dpop(), b = dpop(), c = dpop();
    dpush(b); dpush(a); dpush(c);
  }
}
static void _sadd(void) { // SUM+
  seekmem(_SADD);
}
static void _sclr(void) { // SUMclr
  seekmem(_SCLR);
}
static void _sin(void) { // SIN
  seekmem(_SIN);
}
static void _sinh(void) { // SINH
  seekmem(_SINH);
}
static void _solve(void) { // SOLVE
  _dup(); _dup(); // 3 x0 on stack
  runs = 0;
  issolve = true;
}
static void _sqrt(void) { // SQRT
  seekmem(_SQRT);
}
static void _stat(void) { // STAT
  seekmem(_STAT);
}
static void _sto(void) { // STO
  _storcl(true);
}
static void _storcl(boolean issto) { // STORCL
  byte nr = dpop();
  byte addr = EESTO + nr * sizeof(double); // Has to be <255 (byte)
  if (nr < MEMSTO) {
    if (issto) EEwrite(addr, dpop());
    else {
      double a;
      EEread(addr, a);
      dpush(a);
    }
  }
}
static void _sub(void) { // SUB -
  _neg(); _add();
}
static void _swap(void) { // SWAP
  if (dp > 1) {
    double a = dpop(), b = dpop();
    dpush(a); dpush(b);
  }
}
static void _tan(void) { // TAN
  seekmem(_TAN);
}
static void _tanh(void) { // TANH
  seekmem(_TANH);
}
static void _then(void) { // THEN
  cl--; // Decrement conditional level
}
static void _until(void) { // UNTIL
  if (!ap) ; // No BEGIN for this UNTIL
  else if (dpop()) apop(); // Go on (delete return address)
  else apush(mp = apop()); // Go back to BEGIN
}
static void _usrset(void) { // USR
  byte pos = dpop(), cmd = dpop();
  if (pos <= MENUNR && cmd <= maxcmd) EEPROM.write(EEMENU + pos, cmd);
}
static void _usr(void) { // USER PROGRAM A~Z
  mp = (executedcommand - maxcmd + PRGMAXNR) * PRGMAXLEN + sizeof(mem);
}


// *** S T A C K

static void stacklift(void) { // End number entry
  decimals = 0; isdot = false; isnewnumber = true;
}

static void floatstack(void) { // Float stack if full
  memcpy(ds, &ds[1], (DATASTACKSIZE - 1) * sizeof(double));
  dp--;
}

static double dpush(double d) { // Push complex number to data-stack
  if (dp >= DATASTACKSIZE) floatstack(); // Float stack
  return (ds[dp++] = d);
}
static double dpop(void) { // Pop real number from data-stack
  return (dp ? ds[--dp] : 0.0F);
}

static void apush(int addr) { // Push address (int) to address-stack
  as[ap++] = addr;
}
static int apop(void) { // Pop address (int) from address-stack
  return (ap ? as[--ap] : NULL);
}



// SUBPROGRAMS

static byte seekprg(byte n) { // Seek program number in program table
  for (byte i = 0; i < sizeof(prgtable); i++)
    if (pgm_read_byte(&prgtable[i]) == n) return (i);
  return (_END);
}
static void seekmem(byte nprg) { // Find run-address (mp) of n-th builtin function
  byte n = seekprg(nprg) + 1;
  mp = 0;
  while (n) if (pgm_read_byte(&mem[mp++]) == _END) n--;
}

static void execute(byte cmd) { // Execute command
  executedcommand = cmd; // Save command for dispatching of numbers
  if (cmd > 12 && cmd != 15) { // New number - except MENU, 0-9. DROP DUP
    stacklift();
  }
  if (cmd < maxcmd) (*dispatch[cmd])(); // Dispatch intrinsic/builtin command
}

static void prgstepins(byte c) { // Insert step (command c)
  for (byte i = prglength; i > prgpos; i--) prgbuf[i] = prgbuf[i - 1];
  prgbuf[prgpos + 1] = c;
  prglength++; prgpos++;
}

template <class T> void EEwrite(int ee, const T& value) { // Write any datatype to EEPROM
  const byte* p = (const byte*)(const void*)&value;
  for (byte i = 0; i < sizeof(value); i++) EEPROM.write(ee++, *p++);
}
template <class T> void EEread(int ee, T& value) { // Read any datatype from EEPROM
  byte* p = (byte*)(void*)&value;
  for (byte i = 0; i < sizeof(value); i++) *p++ = EEPROM.read(ee++);
}

static void shortcuts() {
  boolean isb = !digitalRead(PIN_B_BUTTON) ? true : false,
          isl = !digitalRead(PIN_LEFT_BUTTON) ? true : false,
          isu = !digitalRead(PIN_UP_BUTTON) ? true : false,
          isr = !digitalRead(PIN_RIGHT_BUTTON) ? true : false,
          isd = !digitalRead(PIN_DOWN_BUTTON) ? true : false;

  if (isb && isl) _nop(); //
  else if (isb && isu) _nop(); //
  else if (isb && isr) _nop(); //
  else if (isb && isd) _nop(); //

  if (isl && isu) dcontrastup(); // Lit+
  else if (isl && isr) sleepnow(); // Sleep
  else if (isl && isd) dcontrastdown(); // Lit-

  else if (isu && isr) _nop(); //
  else if (isu && isd) {
    mp = dp = ap = 0;  // Break
    dpush(0.0);
  }

  else if (isr && isd) _nop(); //
}


// ***** P R I N T I N G

static void printstar(char* sw, char* sn, char* se, char* ss) { // Print strings as star
  printsat(sw, 20 - strlen(sw) * 5, 2, 5); // W (right-justified)
  printsat(sn, 22 + (MAXCHAR - strlen(sn)) * 5 / 2, 2, 1); // N (centered)
  printsat(se, 44, 2, 5); // E (left-justified)
  printsat(ss, 22 + (MAXCHAR - strlen(ss)) * 5 / 2, 2, 9); // S (centered)
}

static void printprgstep(void) { // Print PRG-step
  char cmdstr[MAXCHAR] = "";
  sbuf[0] = prgnr + 'A'; //sbuf[1] = ' '; // Program name
  sbuf[2] = _ones(prgpos) + '0'; sbuf[1] = _tens(prgpos) + '0'; sbuf[3] = sbuf[4] = ' '; // Program step
  if (prglength) strcpy_P(cmdstr, pgm_read_word(cmd + prgbuf[prgpos])); // Command
  byte cmdlen = strlen(cmdstr);
  for (byte i = 5; i < 7; i++) sbuf[i] = ' ';
  for (byte i = 0; i < cmdlen; i++) sbuf[i + 7 - cmdlen] = cmdstr[i];
  sbuf[7] = '\0';
  printbigsat(sbuf);
}


static boolean printscreen(void) { // Print screen due to state
  dbufclr();

  if (isprgedit) { // # Edit program
    printprgstep();
    printstar((char*)"INS", (char*)"UP", (char*)"DEL", (char*)"DOWN");
  }

  else {

    char s[4][MAXCHAR + 1]; // String array (+1 for '\0')
    if (numlevel == 1) { // Level 1
      for (byte j = 0; j < 4; j++) {
        for (byte i = 0; i < 4; i++) {
          char tmpstr[MAXCHAR];
          byte a = j * 4 + i; // Number level 1
          if (ismenu) a = EEPROM.read(EEMENU + a); // Menu level 1
          strcpy_P(tmpstr, pgm_read_word(cmd + a));
          s[j][i] = tmpstr[0];
        }
        s[j][4] = '\0';
      }
      printstar(s[0], s[1], s[2], s[3]);
    }

    else if (numlevel == 2) { // Level 2
      for (byte i = 0; i < 4; i++) {
        byte a = (keymult - 1) * 4 + i; // Number level 2
        if (ismenu) a = EEPROM.read(EEMENU + a); // Menu level 2
        strcpy_P(s[i], pgm_read_word(cmd + a));
      }
      printstar(s[0], s[1], s[2], s[3]);
    }

    else { // Command
      byte cs = cmdselect;
      byte n = (cs < maxcmd - PRGMAXNR) ? cs : cs - (maxcmd - PRGMAXNR) + 1; // Command number
      printcat((n / 10) + '0', 0, 2, 1); printcat((n % 10) + '0', 0, 3, 1);
      strcpy_P(sbuf, pgm_read_word(cmd + cmdselect)); printcmdsbuf(); // Selected command (big)
      if (cmdselect) { // Previous and next command
        strcpy_P(sbuf, pgm_read_word(cmd + cmdselect - 1));
        printsat(sbuf, 45, 2, 1);
      }
      if (cmdselect < maxcmd - 1) {
        strcpy_P(sbuf, pgm_read_word(cmd + cmdselect + 1));
        printsat(sbuf, 45, 3, 1);
      }
    }

    if (isshiftprgins) printprgstep(); // Print PRG-step
    else { // Print stack
      if (mp) { // Print "RUN"
        sbuf[0] = 'R'; sbuf[1] = 'U'; sbuf[2] = 'N'; sbuf[3] = '\0';
        printbigsat(sbuf);
      }
      else if (dp) printnum(ds[dp - 1]); // Stack
      else printcat('>', 0, 0, 3); // Empty stack prompt
    }

    if (!isnewnumber) printcat('>', 0, 0, 3); // Num input indicator
  }
  ddisplay();
  return (false); // To determine isprintscreen
}


// ***** S E T U P  &  L O O P

void setup() {
  bootpins(); // System boot procedure
  dinit();
  bootpowersaving();
  setframerate(FRAMERATE);
  ddisplay();

  flashmode(); // Secure waiting for flashing new software (UP+on)
  delaylong(3); // Needed to stop serial communication to switch LEDs off
  ledsoff();

  dcontrast(EEPROM.read(EECONTRAST)); // Set contrast
}


void loop() {

  if (isprintscreen) isprintscreen = printscreen(); // Print screen

  if (mp) { // *** Execute/run code
    if (mp < sizeof(mem)) key = pgm_read_byte(&mem[mp++]); // Builtin
    else key = EEPROM[EEUSTART + mp++ -sizeof(mem)]; // User program

    if (seekprg(key) != _END) apush(mp); // Subroutine detected - branch

    if (key == _END) { // _END reached
      if (ap) mp = apop(); // End of subroutine - return
      else { // End of run
        mp = 0;
      }
    }
    else execute(key);
  }

  else { // *** No run - get key
    if (!(nextFrame())) return; // Pause render (idle) until it's time for the next frame
    key = buttonscan(); // Get key

    if (issolve) { // # SOLVE
      if (++runs < 3) {
        if (runs == 2) { // Second run - f(x0+dx)
          _swap(); dpush(DELTAX); _add(); // x0+DELTAX ... Prepare new x-value
        }
        execute(maxcmd - PRGMAXNR); // Execute first user program
      }
      else { // Third run - x1
        _swap(); _div(); dpush(-1.0); _add(); // f1/f0-1
        dpush(DELTAX); _swap(); _div(); // diffx=DELTAX/(f1/f0-1)
        double diffx = dpush(dpop()); // Rescue diffx for exit condition
        _sub(); // x1=x0-diffx ... improved x-value
        runs = 0;
        if (diffx < DELTAX && diffx > -DELTAX) { // Exit
          isnewnumber = isprintscreen = true; issolve = false;
        }
        else { // 3 x1 on stack
          _dup(); _dup();
        }
      }
    }


    if (key != oldkey) { // New key entered
      oldkey = key; // Save as old key

      shortcuts(); // Check shortcuts

      if (key) { // Regular/valid key pressed

        if (key != B_B) { // # Regular (not shifted) key

          if (numlevel) { // Raise numlevel if >0
            numlevel++;
            if (key == B_A) numlevel = 1; // Prevent execution of command
            else if (numlevel == 2 && key != B_A) keymult = bitpos(key) - 2;
          }

          if (numlevel == 0) { // Regular cursor button entered
            if (isprgedit) { // # Program editing
              if (key == B_A) { // Exit
                for (byte i = 0; i < prglength; i++) EEPROM[prgaddr + i] = prgbuf[i];
                EEPROM[prgaddr + prglength] = _END;
                isprgedit = false;
                //printprg();
              }
              else if (key == B_L) { // Insert step by selecting command
                isprgedit = false;
                isprgins = true;
              }
              if (key == B_D && prgpos < prglength - 1) prgpos++; // Down
              else if (key == B_U && prgpos) prgpos--; // Up
              else if (key == B_R && prglength) { // Delete step
                for (byte i = prgpos; i < prglength; i++) prgbuf[i] = prgbuf[i + 1];
                prglength--;
                if (prgpos) prgpos--;
              }
            }
            else { // # Command mode
              if (key == B_D && cmdselect < maxcmd - 1) cmdselect++;
              else if (key == B_U && cmdselect > 0) cmdselect--;
              else if (key == B_L) {
                if (cmdselect > CMDSTEP) cmdselect -= CMDSTEP; else cmdselect = 0;
              }
              else if (key == B_R) {
                if (cmdselect < sizeof(cmd) / 2 - 1 - CMDSTEP) cmdselect += CMDSTEP;
                else cmdselect = sizeof(cmd) / 2 - 1;
              }
              if (key == B_A) { // Command selected
                if (isprgins) { // Insert program step
                  prgstepins(cmdselect);
                  isprgins = false; isprgedit = true; // Return to prgedit
                }
                else execute(cmdselect); // Execute selected command
              }
            }
          }

          else if (numlevel > 2) { // Enter number and execute
            byte pos = (keymult - 1) * 4 + bitpos(key) - 2 - 1;
            if (ismenu) { // Menu key selected
              if (isshiftprgins) { // Insert program step
                prgstepins(EEPROM.read(EEMENU + pos));
                isshiftprgins = false; isprgedit = true; // Return to prgedit
              }
              else execute(EEPROM.read(EEMENU + pos)); // Execute menu key
              ismenu = false; // Leave menu after execution - wanted?
            }
            else { // Number key selected
              if (isshiftprgins) { // Insert program step
                if (pos) {
                  prgstepins(pos);
                  isshiftprgins = false; isprgedit = true; // Return to prgedit
                }
                else ismenu = true;
              }
              else execute(pos); // Execute number key
            }
            if (!islock) numlevel = 0; else numlevel = 1; // Reset numlevel
          }
        }

        else { // # Shift key entered (if (key == B_B) with KEY_B=KEY_PREEND)
          if (isprgedit) { // Button B pressed, while in program insert mode
            isprgedit = false; isshiftprgins = true; //isprgins=true;
          }

          if (numlevel == 0) numlevel = 1; // First shift - raise numlevel initially
          else if (islock) numlevel = 0; // Second shift - reset numlevel if in lock mode
        }

      }
      else if (!islock) {
        numlevel = 0; // Reset numlevel if no key entered and not in lock mode
        if (isshiftprgins) {
          isprgedit = true; isshiftprgins = false;
        }
      }

      isprintscreen = true;

    }
  } // End of evaluating key

}
