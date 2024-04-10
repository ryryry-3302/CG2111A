
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

// Set the I2C address of your LCD module
#define LCD_ADDRESS 0x27

// Set the dimensions of your LCD (number of columns and rows)
#define LCD_COLUMNS 16
#define LCD_ROWS 2

hd44780_I2Cexp lcd; // declare lcd object at address 0x27 (use i2c address zero to auto locate address)

// Create a LiquidCrystal_I2C object

void setup() {
  // Initialize the LCD
  lcd.init();

  // Turn on the backlight (if your LCD has backlight control)
  lcd.backlight();

  // Print a message to the LCD
  lcd.setCursor(0, 0);
  lcd.print("Hello, Worlsadsd!");

  // Move to the second line and print another message
  lcd.setCursor(0, 1);
  lcd.print("LCD Test");
}

void loop() {
  // Your main code here
}
