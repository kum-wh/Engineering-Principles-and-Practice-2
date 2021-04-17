#include <serialize.h>
#include <stdarg.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "packet.h"
#include "constants.h"

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

/*
   Alex's configuration constants
*/

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV 192

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC 20.4

//Alex's width * pi
#define ALEX_CIRC 39.3

//Regardless of movement command, it will timeout after MOVE_TIMEOUT microeconds
#define MOVE_TIMEOUT 5000000

#define ULTRASONIC_TIMEOUT 100000

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin


#define RIGHT_MULTIPLIER 0.95

/*
      Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;


// Store the ticks from Alex's left and
// right encoders when turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;


// Timer counter
volatile unsigned long microsec;

volatile unsigned long timeout_target;

//for colour sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_60X);
void colour()
{
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);

  TPacket Packet;
  Packet.packetType = PACKET_TYPE_RESPONSE;
  Packet.command = RESP_COLOR;
  Packet.params[0] = r;
  Packet.params[1] = g;
  sendResponse(&Packet);
}

/*
   Alex Communication Routines.
*/

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);

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
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
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
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
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
   Setup and start codes for external interrupts and
   pullup resistors.

*/
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  switch (dir) {
    case FORWARD:
      leftForwardTicks++;
      forwardDist = (unsigned long) (((float) leftForwardTicks / (float) COUNTS_PER_REV) * WHEEL_CIRC);
      break;

    case BACKWARD:
      leftReverseTicks++;
      reverseDist = (unsigned long) (((float) leftReverseTicks / (float) COUNTS_PER_REV) * WHEEL_CIRC);
      break;

    case LEFT:
      leftReverseTicksTurns++;
      break;

    case RIGHT:
      leftForwardTicksTurns++;
      break;
  }
}

void rightISR()
{
  switch (dir) {
    case FORWARD:
      rightForwardTicks++;
      break;

    case BACKWARD:
      rightReverseTicks++;
      break;

    case LEFT:
      rightForwardTicksTurns++;
      break;

    case RIGHT:
      rightReverseTicksTurns++;
      break;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  EIMSK |= 0b00000011;
  EICRA = 0b00001010;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}


/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
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

  int count = 0;

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
   Alex's motor drivers.

*/

// Set up Alex's motors
void setupMotors()
{
  DDRD |= 0b01100000;
  DDRB |= 0b00001100;
  PORTD &= 0b10011111;
  PORTB &= 0b11110011;
  TCNT2 = 0;
  TCCR2A = 0b00000011;
  OCR2A = 0;
  OCR2B = 0;
  TIMSK2 = 0b00000111;
  TCCR2B = 0b00000100; //clk/256

  /* Our motor set up is:
        A1IN - Pin 5, PD5, OC0B, LB
        A2IN - Pin 6, PD6, OC0A, LF
        B1IN - Pin 10, PB2, OC1B, RF
        B2In - pIN 11, PB3, OC2A, RB
  */
}

// Start the PWM for Alex's motors.
void startMotors()
{
  TCNT2 = 0;
  
}

void stop() {
  dir = STOP;
  PORTB &= 0b11110011;
  PORTD &= 0b10011111;
  //TCCR0B = 0b00000000;
}


ISR(TIMER2_COMPA_vect) {
  PORTD &= 0b10011111;
}

ISR(TIMER2_COMPB_vect) {
  PORTB &= 0b11110011;
}

ISR(TIMER2_OVF_vect) {
  switch (dir) {
    case STOP:
      PORTD &= 0b10011111;
      PORTB &= 0b11110011;
      break;
    case FORWARD:
      PORTD |= 0b01000000;
      PORTD &= 0b11011111;
      PORTB |= 0b00000100;
      PORTB &= 0b11110111;
      break;
    case BACKWARD:
      PORTD |= 0b00100000;
      PORTD &= 0b10111111;
      PORTB |= 0b00001000;
      PORTB &= 0b11111011;
      break;
    case LEFT:
      PORTD |= 0b00100000;
      PORTD &= 0b10111111;
      PORTB |= 0b00000100;
      PORTB &= 0b11110111;
      break;
    case RIGHT :
      PORTD |= 0b01000000;
      PORTD &= 0b11011111;
      PORTB |= 0b00001000;
      PORTB &= 0b11111011;
      break;
  }
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD;
  if (dist > 0) {
    deltaDist = dist;
  }
  else {
    deltaDist = 9999999;
  }
  newDist = forwardDist + deltaDist;

  int val = pwmVal(speed);
  OCR2A = val;
  OCR2B = RIGHT_MULTIPLIER * val;
  startMotors();

  timeout_target = microsec + MOVE_TIMEOUT;
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  dir = BACKWARD;
  if (dist > 0) {
    deltaDist = dist;
  }
  else {
    deltaDist = 9999999;
  }
  newDist = reverseDist + deltaDist;


  int val = pwmVal(speed);
  OCR2A = val;
  OCR2B = RIGHT_MULTIPLIER * val;
  startMotors();

  timeout_target = microsec + MOVE_TIMEOUT;

}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;

  if (ang == 0) {
    deltaTicks = 99999999;
  }
  else {
    deltaTicks = (ang / 360) * ALEX_CIRC * COUNTS_PER_REV / WHEEL_CIRC;
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;

  int val = pwmVal(speed);
  OCR2A = val;
  OCR2B = val;
  startMotors();

  timeout_target = microsec + MOVE_TIMEOUT;
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;

  if (ang == 0) {
    deltaTicks = 99999999;
  }
  else {
    deltaTicks = (ang / 360) * ALEX_CIRC * COUNTS_PER_REV / WHEEL_CIRC;
  }
  targetTicks = leftForwardTicksTurns + deltaTicks;

  int val = pwmVal(speed);
  OCR2A = val;
  OCR2B = val;
  startMotors();

  timeout_target = microsec + MOVE_TIMEOUT;
}

void setupTimer() {
  TCNT0 = 0;
  TCCR0A = 0b00000010;
  OCR0A = 7;
  TIMSK0 = 0b00000010;
  TCCR0B = 0b00000010;
  microsec = 0;
}

ISR(TIMER0_COMPA_vect) {
  microsec += 4;
}

void setupUltrasonic() {
  DDRB |= 0b00000010;
  DDRB &= 0b11111110;
}

void getUltrasonic() {
  unsigned long response_time = 0;
  unsigned long echo_start;
  bool echo_detect = 0;
  bool valid_read = 0;
  PORTB |= 0b00000010;
  unsigned long pulse_start = microsec;
  int tracker = 0;
  while (microsec < pulse_start + 10);
  PORTB &= 0b11111101;
  while (!valid_read && (microsec < pulse_start + ULTRASONIC_TIMEOUT)) {
    tracker += PINB;
    if (!echo_detect && ((PINB & 0b00000001) == 0b00000001)) {
       echo_start = microsec;
       echo_detect = 1;
    }
    if (echo_detect && ((PINB & 0b00000001) == 0b00000000)) {
      valid_read = 1;
      response_time = microsec - echo_start;
    }
  }
  TPacket ultrasonicPacket;
  ultrasonicPacket.packetType = PACKET_TYPE_RESPONSE;
  ultrasonicPacket.command = RESP_ULTRASONIC_DIST;
  ultrasonicPacket.params[0] = valid_read;
  ultrasonicPacket.params[1] = 0.1715 * response_time;
  sendResponse(&ultrasonicPacket);
}



/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  forwardDist = 0;
  reverseDist = 0;
}

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_STOP:
      sendOK();
      stop();
      break;
    case COMMAND_GET_STATS:
      sendOK();

      break;
    case COMMAND_GET_ULTRASONIC:
      sendOK();
      getUltrasonic();
      break;

    case COMMAND_GET_COLOUR:
      sendOK();
      colour();
      break;
    /*
       Implement code for other commands here.
    */

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  setupTimer();
  setupUltrasonic();
  enablePullups();
  initializeState();
  sei();
  tcs.begin();
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
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

void loop() {

  if (deltaDist > 0) {
    //dbprint("%ld", microsec);
    switch (dir) {
      case FORWARD:
        if (forwardDist >= newDist) {
          deltaDist = 0;
          newDist = 0;
          stop();
        }
        break;
      case BACKWARD:
        if (reverseDist >= newDist) {
          deltaDist = 0;
          newDist = 0;
          stop();
        }
        break;
      default:
        deltaDist = 0;
        newDist = 0;
        stop();
    }
    if (microsec >= timeout_target) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0) {
    switch (dir) {
      case LEFT:
        if (leftReverseTicksTurns >= targetTicks) {
          targetTicks = 0;
          deltaTicks = 0;
          stop();
        }
        break;
      case RIGHT:
        if (leftForwardTicksTurns >= targetTicks) {
          targetTicks = 0;
          deltaTicks = 0;
          stop();
        }
        break;
      default:
        targetTicks = 0;
        deltaTicks = 0;
        stop();
    }
    if (microsec >= timeout_target) {
      targetTicks = 0;
      deltaTicks = 0;
      stop();
    }
  }


  // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }


}
