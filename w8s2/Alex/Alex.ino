#include <serialize.h>
#include <math.h>
#include "packet.h"
#include "constants.h"
#include <stdarg.h>
#include <Wire.h>
#include <hd44780.h>  // Include the hd44780 library
#include <hd44780ioClass/hd44780_I2Cexp.h> // Include the I2C expander IO class

// Set the I2C address of your LCD module
#define LCD_ADDRESS 0x27

// Set the dimensions of your LCD (number of columns and rows)
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// Create an hd44780_I2Cexp I2C expander object
hd44780_I2Cexp lcd;


#define ALEX_LENGTH 26
#define ALEX_BREADTH 15
#define PI 3.141592654


// SDA: Arduino Mega pin 20 orange
// SCL: Arduino Mega pin 21 blue
// VCC: 5V pin of the Arduino Mega
// GND: GND pin of the Arduino Mega

//colour sensor 

#define S0 35 //purple
#define S1 36 //green
#define S2 37 //yellow
#define S3 38 //grey
#define sensorOut 39 //blue

// Storesrequency read by the photodiodes
volatile unsigned int redFrequency = 0;
volatile unsigned int greenFrequency = 0;
volatile unsigned int blueFrequency = 0;




// 26by15cm

float alexDiagonal =0.0;
float alexCirc = 0.0;

volatile TDirection dir;
/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder

#define COUNTS_PER_REV      4
#define COUNTS_PER_REV_left 40

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          21

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile  unsigned long leftForwardTicks; 
volatile  unsigned long rightForwardTicks;

volatile  unsigned long leftReverseTicks; 
volatile  unsigned long rightReverseTicks;

//turn
volatile  unsigned long leftForwardTicksTurns; 
volatile  unsigned long rightForwardTicksTurns;

volatile  unsigned long leftReverseTicksTurns; 
volatile  unsigned long rightReverseTicksTurns;



// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;
unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;

volatile unsigned long redColour;
volatile unsigned long greenColour;
volatile unsigned long blueColour;

/*
 * 
 * Alex Communication Routines.
 * 
 */

 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void getColour(){
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  // Reading the output frequency
  redFrequency = pulseIn(sensorOut, LOW);
  
   // Printing the RED (R) value

  delay(100);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  greenFrequency = pulseIn(sensorOut, LOW);
  
  // Printing the GREEN (G) value  

  delay(100);
 
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  blueFrequency = pulseIn(sensorOut, LOW);
  
  // Printing the BLUE (B) value 
  delay(100);
}

void sendColour(){
  getColour();
  TPacket colourPacket;
  colourPacket.packetType = PACKET_TYPE_RESPONSE;
  colourPacket.command = RESP_COLOUR;
  colourPacket.params[0] = map(redFrequency,550,50,0,255);
  colourPacket.params[1] = map(greenFrequency,513,65,0,255);
  colourPacket.params[2] = map(blueFrequency,507,48,0,255);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("identified");
  lcd.setCursor(0,1);
  lcd.print("R"+(String)colourPacket.params[0] + " G"+ (String)colourPacket.params[1] + " B"+(String)colourPacket.params[2]);
  sendResponse(&colourPacket);

}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  
  sendResponse(&statusPacket);
  
     
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}


void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 *  and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 18 and 19
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  
}


// Functions to be called by INT2 and INT3 ISRs.
void leftISR()
{
  if(dir == FORWARD){
    
  leftForwardTicks++;
  }  
  if (dir == BACKWARD){
  leftReverseTicks++;
  }
  
  if (dir == LEFT){
  leftReverseTicksTurns++;
  //rightForwardTicksTurns++;  
  }
  if (dir == RIGHT){
  leftForwardTicksTurns++;
  //rightReverseTicksTurns++;  
  }
  
  

   
  
}

void rightISR()
{
  if(dir == FORWARD){
    
  rightForwardTicks++;
  forwardDist = (unsigned long) ((float) rightForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);

  }
  
  if (dir == BACKWARD){
  rightReverseTicks++;
  reverseDist = (unsigned long) ((float) rightReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);

 
  }
  
  if (dir == LEFT){
  rightForwardTicksTurns++;
  //leftReverseTicksTurns++;  
  }
  if (dir == RIGHT){
  //leftForwardTicksTurns++;
  rightReverseTicksTurns++;  
  }
  
}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
  EICRA = 0b10100000; 
  EIMSK = 0b00001100;
  

}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.


// Implement INT2 and INT3 ISRs above.

ISR(INT2_vect){
  rightISR();
}

ISR(INT3_vect){

    leftISR();
    
}







/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  

  //forward / reverse
  leftForwardTicks=0;
  rightForwardTicks=0;
  
  leftReverseTicks=0;
  rightReverseTicks=0;

  //turns
   leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}

// Intialize Alex's internal states
void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Moving Forward");
        lcd.setCursor(7,1);
        lcd.print(":P");
        sendOK();
        forward((double) command->params[0], (float) command->params[1]);
      break;

    /*
     * Implement code for other commands here.
     * 
     */
    case COMMAND_REVERSE:
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Reversing");
        lcd.setCursor(0,1);
        lcd.print(":D");
        sendOK();
        backward((double) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Turning");
        lcd.setCursor(0,1);
        lcd.print("Left");
        sendOK();
        left((double) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Turning");
        lcd.setCursor(0,1);
        lcd.print("Right");
        sendOK();
        right((double) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_STOP:
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Stopping");
        lcd.setCursor(7,1);
        lcd.print("OwO");
        sendOK();
        stop();
      break;
    case COMMAND_GET_STATS:
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Sending Stats");
        sendStatus();
       break;   
    case COMMAND_CLEAR_STATS:
        clearOneCounter(command->params[0]);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Clearing Stats");
        sendOK();
        break; 
    
    case COMMAND_GET_COLOUR:
        sendColour();
        
        break;
    break;

    case COMMAND_SEND_MESSAGE:
      lcd.clear();
      int row = 0; // Start with the first row
      int column = 0; // Start with the first column
      for (int i = 0; command->data[i] != '\0' || i == 32; ++i) {
        lcd.setCursor(column, row); // Set the cursor to the current position
        lcd.write(message[i]); // Print the current character
        column++; // Move to the next column

        // If we reach the end of the first row, move to the second row
        if (column == 16 && row == 0) {
          row = 1;
          column = 0;
        }
      }
      break;


    default:
      sendBadCommand();
      break;
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
  cli();
  // Setting the outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  // Setting the sensorOut as an input
  pinMode(sensorOut, INPUT);
  
  // Setting frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

  
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  initializeState();
  sei();
  // Initialize the LCD with the I2C address and dimensions
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
  delay(200);
  // Print a message to the LCD
  lcd.setCursor(0,0);
  lcd.print("HELLO HELLO");



}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}
unsigned long computeDeltaTicks(float ang){
  unsigned long ticks = (unsigned long)((ang*alexCirc * COUNTS_PER_REV*1.15)  / (360.0 * WHEEL_CIRC));
  return ticks;
  
}

void left(float ang, float speed)
{
  if(ang == 0){
    deltaTicks=99999999;
  }
  else{
    deltaTicks=computeDeltaTicks(ang);
  }
  targetTicks = rightForwardTicksTurns + deltaTicks;
  ccw(ang,speed);
}
  

void right(float ang, float speed)
{
  if(ang == 0){
    deltaTicks=99999999;
  }
  else{
    deltaTicks=computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;
  cw(ang,speed);
}
  

void loop() {
// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2
  

  

 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 
      
  if(deltaDist > 0)
  {
      if(dir==FORWARD)
      {
        if(forwardDist > newDist)
           {
            deltaDist=0;
            newDist=0;
            stop();
           }
      }
      else
          if(dir == BACKWARD)
          {
            if(reverseDist > newDist)
            {
              deltaDist=0;
              newDist=0;
              stop();
            }
          }
          else
      if(dir == STOP)
        {
              deltaDist=0;
              newDist=0;
              stop();
        }
  }

  if(deltaTicks > 0){
    if(dir == LEFT){
      if(leftReverseTicksTurns >= targetTicks){
          dbprintf("HELLLLLLLLLLLLLLLLLLLLLLLOO");
          deltaTicks=0;
          targetTicks =0;
          stop();
        }
      }
      else
      if(dir == RIGHT){
      if(rightReverseTicksTurns >= targetTicks){
          deltaTicks=0;
          targetTicks =0;
          stop();
        }
      }
      else
        if(dir == STOP){
          deltaTicks =0;
          targetTicks=0;
          stop();
          }
      
    
   }
    
}
