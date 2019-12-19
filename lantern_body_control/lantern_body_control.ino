//----------------------------------------------------------------------
//  File: lantern_body_control
// 
// Hardware: Arduino Pro Mini 8 MHz 3.3V
// 
// Description:
//    This load for Arduino #2 scans for input from 
//      > Rotary encoder and pushbutton
//      > Joystick 
//
//    and drives 
//      > 2 x Guage servos
//      > 2 x 9 Guage Neopixels
//      > 4 x Nixie tubes
//      > RGB Nixie LEDs
//  
#include "pins_arduino.h"
#include <FastLED.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>


//-------- I/O Pin Assignment ]--------

//  D8  PB0 ICP1           TC1 Capture input
//  SCK PB5 SCK            SPI Clock Out
//  D2  PD2 PCINT1/INT0
//  D3  PD3 OC2B/INT1
//  D4  PD4 T0
//  D5  PD5 T1
//  D6  PD6 AIN0    
//  D7  PD7 AIN1    
//  D9  PB1 OC1A  TC1 Match A Out
//  D10 PB2 OC1B  TC1 Match B Out
//  RXI PD0 RXD   Async Serial Rxd
//  TXO PD1 TXD   Async Serial Txd

#define NEOPIXEL_DATA_PIN   2
#define SPI_SS_PIN         10       // Input  (we're a slave)
#define SPI_MOSI_PIN       11       // Input  (we're a slave)
#define SPI_MISO_PIN       12       // Output (we're a slave) 
#define SPI_SCLK_PIN       13       // Input (we're a slave, also Arduino LED) 

#define HAND_SENSE_PIN      A1 
#define I2C_SD_PIN          A4 
#define I2C_SC_PIN          A5 



//--------[ NeoPixels ]--------
#define NUM_LED_ROWS    14
#define NUM_LED_COLUMNS 12   // 12 segments of 14 leds each. segments are virtical
#define BRIGHTNESS      75      

const int  num_leds_c = NUM_LED_ROWS * NUM_LED_COLUMNS;

CRGB leds[num_leds_c];


//--------[ Adafruit Motor Shield (Gears, spinner LED) ]-------- 
Adafruit_PWMServoDriver afms = Adafruit_PWMServoDriver(0x61);


//--------[ HAND MOTORS ]--------

#define MINUTE_HAND 0
#define HOUR_HAND   1

#define HAND_SENSE_THRESHOLD 0x100
#define MINUTEHAND_INCREMENT 100      // Distance for 1 month


// PWN Shield channels for windings          Minute Hand    Hour hand
const char handMotor_channel_c[2][4] = { {4, 5, 6, 7}, {0, 1, 2, 3} };  
const char spinner_servo_channel_c   = 9;



//--------[ Laser Spinner ]--------
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

Adafruit_PWMServoDriver pwm  = Adafruit_PWMServoDriver(0x40);


//--------[ Timer ISR ]--------
unsigned long int last_ms;


//---------------------------------------------------------------
//       Utilities
//---------------------------------------------------------------
uint16_t get_hex(char **p);
uint16_t get_int(char **p);



//---------------------------------------------------------------------------
class Hands_cl {                                               // Hands Class
//---------------------------------------------------------------------------
   const int motor_fullRotation_c[2] = {1630, 3842};  // min, hour
   const int motor_segmentSteps_c[2] = { motor_fullRotation_c[0]/12, motor_fullRotation_c[1]/120 };
   const int motor_sense_offset_c[2] = { 150, 340};

   const uint8_t motor_sequence_c[8][4] = { {0,0,0,1}, {0,0,1,1}, {0,0,1,0}, {0,1,1,0},
                                            {0,1,0,0}, {1,1,0,0}, {1,0,0,0}, {1,0,0,1} };

   uint8_t  motor_phase[2];
   uint16_t motor_position[2];
   uint16_t motor_target[2];
   uint16_t interval;
   uint16_t timer;

public:
   char done;

   Hands_cl(char c) {
      interval = 10;
      timer    = 0;
      done     = 0;

      for (int i=0; i<2; i++) {
          motor_position[i] = 0; 
          motor_phase[i]    = 0;
          motor_target[i]   = 0;
      }
   }

   void tick() {
      if (timer) timer--;
   }

   //--------------------------
   void update() {
   //--------------------------
      uint16_t to_up, to_down;
   
      if (timer) return;

      timer = interval; 
 
      for (int m = 0; m<2; m++) {
         if (motor_position[m] == motor_target[m] ) continue;

         //Serial.print(m);
         //Serial.print(": ");
         //Serial.print(motor_position[m]);
         //Serial.print(", ");
         //Serial.print(motor_target[m]);
         //Serial.print(": ");

         if (motor_position[m] > motor_target[m]) {
            // we're moving to a lower position, but which way is shorter?
            to_down = motor_position[m] - motor_target[m];
            to_up   = motor_target[m] + motor_fullRotation_c[m] - motor_position[m];
         //   Serial.print(to_up);
         //   Serial.print(", ");
         //   Serial.print(to_down);

            if ( to_up < to_down ) {
         //      Serial.println(" -> Step lower up");
               step(m, 1); 
            }
            else {
         //      Serial.println(" -> Step lower down");
               step(m,-1);
            }
         }
         // Moving to a higher position, but which way is shorter?
         else {
            to_up  = motor_target[m] - motor_position[m];
            to_down =  motor_position[m] + motor_fullRotation_c[m] - motor_target[m];
        //   Serial.print(to_up);
        //   Serial.print(", ");
        //   Serial.print(to_down);

            if ( to_up < to_down ) {
        //       Serial.println(" -> Step higher up");
               step(m, 1); 
            }
            else {
        //       Serial.println(" -> Step higher down");
               step(m,-1);
            }
         }
      }

      done = (    motor_position[  HOUR_HAND] == motor_target[  HOUR_HAND] 
               && motor_position[MINUTE_HAND] == motor_target[MINUTE_HAND]
             );
   }


   void calibrate() {
      // If sensor is already covered, move the hour hand a bit
     if (analogRead(HAND_SENSE_PIN) < HAND_SENSE_THRESHOLD) {
       step(HOUR_HAND, -100);
     }
     // if sensor is still covered, move minute hand a bit
     if (analogRead(HAND_SENSE_PIN) < HAND_SENSE_THRESHOLD) {
       step(MINUTE_HAND, -100);
     }

     // Hour Hand: Advance until we pass by the sensor, then go 200 further
     while (analogRead(HAND_SENSE_PIN) > HAND_SENSE_THRESHOLD) {
         step(HOUR_HAND, 1);
         delay(10);
      }
      step(HOUR_HAND, 200);

      // Hour Hand: Move backward until we pass the sensor
      while (analogRead(HAND_SENSE_PIN) > HAND_SENSE_THRESHOLD) {
          step(HOUR_HAND, -1);
          delay(10);
      }
      // Trim into position
      step(HOUR_HAND, -motor_sense_offset_c[HOUR_HAND] );
      motor_position[HOUR_HAND] = 0; 

      // Minute Hand: Advance until we cover the sensor, then move 100 beyond
      while (analogRead(HAND_SENSE_PIN) > HAND_SENSE_THRESHOLD) {
         step(MINUTE_HAND, 1);
         delay(10);
      }
      step(MINUTE_HAND, 100);

      // Minute Hand: Back up until we cover the sensor
      while (analogRead(HAND_SENSE_PIN) > HAND_SENSE_THRESHOLD) {
          step(MINUTE_HAND, -1);
          delay(10);
      }
      // Trim into position
      step(MINUTE_HAND, -motor_sense_offset_c[MINUTE_HAND] );
      motor_position[MINUTE_HAND] = 0;
   }


   char step(int m, int n) {
      int e;
      int d = abs(n);

      for (int i=0; i<d; i++) {
         if (n > 0) {
           motor_phase[m]    = (motor_phase[m]    == 7                        ) ? 0 : motor_phase[m] + 1; 
           motor_position[m] = (motor_position[m] == motor_fullRotation_c[m]-1) ? 0 : motor_position[m]+1; 
         } 
         else {
           motor_phase[m]    = (motor_phase[m]    == 0) ? 7                       : motor_phase[m]-1;
           motor_position[m] = (motor_position[m] == 0) ? motor_fullRotation_c[m]-1 : motor_position[m]-1;
         };                                                

         // Apply new step phase to all windings
         for (int w = 0; w < 4; w++) {
            e = ( motor_sequence_c[ motor_phase[m] ][w] ) ? 0 : 4095;
            pwm.setPWM( handMotor_channel_c[m][w], e, 4095);
         }
         if (d > 1) delay(10);
      }
   }

   //-----------------------------
   void set_time( char *cmd) {
   //-----------------------------
      uint16_t y, s, h;
      char *p = cmd;

      // Get year from command line
      y = get_int(&p);
     
      // Hour Hand: Translate year onto clock face (120 hand positions total)
      h =   (y < 1000) ? y / 100  
          : (y < 2000) ? (y - 900) / 10 
          : (y + 9000) / 100;

      // Convert to motor position and set target
      s = h * motor_segmentSteps_c[HOUR_HAND]; 
      motor_target[HOUR_HAND] = s;    

      
      // Minute Hand: 12 hand positions total
      if (*p != '\0') p++;
      y = get_hex(&p);

      // Convert to motor steps and set target
      s = y * motor_segmentSteps_c[MINUTE_HAND] + motor_segmentSteps_c[MINUTE_HAND]/2; 
      motor_target[MINUTE_HAND] = s;

      done = 0;

      Serial.print("[");
      Serial.print(h, DEC);
      Serial.print(", ");
      Serial.print(y, DEC);
      Serial.println("]"); 
   }

};

Hands_cl clock_hands('h');


//--------------------------------------------------------------
class spi_slave_cl {                        //   SPI Slave Class
//--------------------------------------------------------------
   uint8_t  rx_bufptr;
   uint8_t  rx_ready;
   uint8_t  txbuf[16];
   uint8_t  tx_bufptr;
   uint8_t  tx_ready;
   char     rxbuf[16];

public:

   spi_slave_cl(char f) {
      pinMode(MISO, OUTPUT);
      SPCR |= _BV(SPE);
      SPCR |= _BV(SPIE);

      rx_bufptr = 0;
      rx_ready  = 0;
      tx_bufptr = 0;
      tx_ready  = 0;
   }

   void transfer() {
      rxbuf[rx_bufptr] = SPDR;
      if (rx_bufptr < 15) rx_bufptr++;
      
      SPDR = clock_hands.done;
      if( SPDR == '\n') { 
         rxbuf[rx_bufptr-1] = '\0'; 
         rx_ready = 1;
      }
   }

   int rxRead(char *p) {
      int n=0;
      if (rx_ready) {
         n = rx_bufptr;

        for (int i=0; i<n; i++) {
           p[i] = rxbuf[i];
         }
         rx_bufptr = 0;
         rx_ready = 0;
         return n;
          
      }
      return n;
   }
};

spi_slave_cl  spi('n');



//----------------------------------------------------------------------------
class WarpCore_cl {
//----------------------------------------------------------------------------
   const CRGB fg = CRGB( 0x384000);
   const CRGB bg = CRGB( 0x002000);

   int   timer;
   int   interval;
   char  enable;
   int   index;

public:

   WarpCore_cl(char c) {
      timer    = 150;
      enable   = 0;
      index    = 0;
      interval = 70;

      for  (int i=0; i<num_leds_c; i++) {
        leds[0] = CRGB::Black;
      }
   }

   void control(char *cmd) {
      if (*cmd == '1') {
         enable = 1;
      } else {
         enable = 0;
         for (int i=0; i<num_leds_c; i++) {
            leds[i] = CRGB::Black;
         }
         FastLED.show();
      }
   }

   void tick() {
      if (timer) timer--;
   }

   void update() {
      if (timer) return;

      timer = interval; 

      if (enable==0) return;

      for (int x=0; x<NUM_LED_COLUMNS; x++) {
         leds[x*NUM_LED_ROWS + index               ] = bg;
         leds[x*NUM_LED_ROWS + NUM_LED_ROWS-1-index] = bg;
      }

      index = (index >= 6) ? 0 : index + 1;

      for (int x=0; x<NUM_LED_COLUMNS; x++) {
        leds[x*NUM_LED_ROWS + index               ] = fg;
        leds[x*NUM_LED_ROWS + NUM_LED_ROWS-1-index] = fg;
      }
      FastLED.show();
   }
};

WarpCore_cl warpcore('n');




//--------------------------------------------------------------------------
class Gears_cl {
//--------------------------------------------------------------------------
   uint16_t front_speed;
   uint16_t front_target;
   uint16_t left_speed;
   uint16_t left_target;
   uint16_t right_speed;
   uint16_t right_target;
   int interval;
   int timer;

public:
   Gears_cl(char *c) {
      front_speed  = 0;
      front_target = 0;
      left_speed   = 0;
      left_target  = 0;
      right_speed  = 0;
      right_target = 0;
      interval     = 200;
      timer        = 0;
   }

   void control(char *cmd) {
      char *p;
      p = cmd;
      front_target = get_hex(&p);
      if (*p != '\0') p++;
      left_target  = get_hex(&p);
      if (*p != '\0') p++;
      right_target = get_hex(&p);
   }
   
   tick() {
      if (timer) timer --;
   }

   update() {
      uint16_t e;
      if (timer) return;

      timer = interval;
      
      front_speed = front_target;
      left_speed  = left_target;
      right_speed = right_target;

      e = map(front_speed, 0, 255, 0, 4095); 
      afms.setPWM( 5,    0, 4095);
      afms.setPWM( 6,    e, 4095);
      afms.setPWM( 7,    0, 4095);

      e = map(left_speed, 0, 255, 0, 4095); 
      afms.setPWM( 2,    0, 4095);
      afms.setPWM( 3,    e, 4095);
      afms.setPWM( 4,    0, 4095);

      e = map(right_speed, 0, 255, 0, 4095); 
      afms.setPWM( 8,    0, 4095);
      afms.setPWM( 9,    e, 4095);
      afms.setPWM(10,    0, 4095);

   }
};

Gears_cl gears(0);


//--------------------------------------------------------------------------
class Spinner_cl {
//--------------------------------------------------------------------------
   uint16_t position;
   uint16_t target;
   int timer;
   int interval;

public:

   Spinner_cl(char c) {
      position = 128;
      target   = 128;
      timer    = 0;
      interval = 10;
   }

   void tick() {
      if (timer) timer--;
   }

   void control(char *cmd) {
      int e;
      char *p;
      p = cmd;

      // Lasers
      e = (*p=='1') ? 4095 : 0;
      afms.setPWM(12, e, 4095);

      if (*p != '\0') p++; // done with 1st parm
      if (*p != '\0') p++; // skip delimiter

      // Servo
      target = get_hex(&p);
   }

   void update() {
      uint16_t e;
      if (timer) return;
      timer = interval;

      position = target;
      e = position; 
//      Serial.print("<");
//      Serial.print(e);
//      Serial.print(">\n");
      
      pwm.setPWM(spinner_servo_channel_c, e, 4095);
   }
};

Spinner_cl spinner(0);


//-------------------------------------------------------------------------
class MySerial_cl {
//-------------------------------------------------------------------------
  char      cmd[16];
  uint8_t   index;
  char      ready;
      
public:

  mySerial_cl(char c) {
    index  = 0;
    cmd[0] = '\0';
    ready  = 0;
  }

  poll() {
    char c;
    c = Serial.read(); 
    if (c > 0 ) {
      c = c & 0x7f;
      cmd[index] = c;
      if (index < 14) {
        index++;
      }

      if (c == '\r') {
        cmd[index] = '\0';
        ready = 1; 
      }
    }
  }

  int rxRead(char *p) {
    int n=0;
    if (ready) {
      n = index;

      for (int i=0; i<n; i++) {
         p[i] = cmd[i];
      }
      p[n] = '\0';
      index = 0;
      ready = 0;
    }
    return n;
  }
};  // MySerial_cl

MySerial_cl mySerial;



//--------------------------------------------------------------------------
void setup() {
//--------------------------------------------------------------------------

   //--------[ 1ms Interrupt ]--------
   OCR0A = 0xAF;
   TIMSK0 |= _BV(OCIE0A);
   last_ms = 0;
   

   //--------[ Hand Sensor ]--------
   pinMode(HAND_SENSE_PIN, INPUT_PULLUP);
   digitalWrite(HAND_SENSE_PIN, HIGH);

  //--------[ NeoPixels ]--------
  pinMode(NEOPIXEL_DATA_PIN, OUTPUT);
  FastLED.addLeds<NEOPIXEL, NEOPIXEL_DATA_PIN>(leds, num_leds_c);
  FastLED.setBrightness( BRIGHTNESS );
  FastLED.show();


  //--------[ Motor Shield ]--------
   afms.begin();
   afms.setPWMFreq(60);
   delay(10);
   afms.setPWM(11, 0, 4095);
   afms.setPWM(12, 0, 4095);
   afms.setPWM(13, 0, 4095);

   //--------[ PWM Shield ]--------
   pwm.begin();
   pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates   
   pwm.setPWM(spinner_servo_channel_c,0,4095);
   
   // Clock Hands
   clock_hands.calibrate();
   
   //--------[ Serial debug ]--------
   Serial.begin(57600);
   Serial.print("hi\n");

}



//----------------------------------------------------------------------------
SIGNAL(TIMER0_COMPA_vect) {                             // 1 ms interrupt ISR
//----------------------------------------------------------------------------
   // Timer0 is already used for millis() - we'll just interrupt somewhere
   // in the middle and call the "Compare A" function below

   unsigned long current_ms;

   current_ms = millis();
   if (current_ms == last_ms) return;
   
   last_ms = current_ms;

   clock_hands.tick();
   warpcore.tick();
   gears.tick();
   spinner.tick(); 
}



//---------------------------------------------------------------------------
ISR (SPI_STC_vect) {                                 // SPI interrupt routine
//---------------------------------------------------------------------------
   spi.transfer();
}



//--------------------------------------------------------------------------
void loop() {
//--------------------------------------------------------------------------
  char cmd[16];
  char *cp;
  int n;
  int p1;

  // Task List
  warpcore.update();
  clock_hands.update();
  gears.update();
  spinner.update();
  mySerial.poll();


  //--------[ Command Interpreter ]--------
  n = mySerial.rxRead(cmd);
  if (n == 0) {
    n = spi.rxRead(cmd);
  }

  if (n != 0) {
    Serial.print("Received( ");
    Serial.print(n);
    Serial.print(")<");
    Serial.print(cmd);
    Serial.print(">\n");
      
    switch ( cmd[0] ) {
      case 'c': clock_hands.calibrate();            break;
      case 'e': Serial.println(cmd);                break; 
      case 'g': gears.control(        &cmd[1] );    break;
      case 'h': clock_hands.set_time( &cmd[1] );    break;
      case 'n':                                     break;   // Noop, used for polling
      case 's': spinner.control(      &cmd[1] );    break;
      case 'w': warpcore.control(     &cmd[1] );    break;
      default:                                      break;
    }
  }
}



//--------------------------------------------------------------------------
uint16_t get_hex(char **pp) {
//--------------------------------------------------------------------------
   int n = 0;
   char *p = *pp;

   while ( (*p >= '0' && *p <= '9')||(*p >='a' && *p <= 'f')) {
      if (*p <= '9') {
         n = n * 16 + (*p - '0'); 
      } else {
         n = n * 16 + (*p + 10 - 'a');
      }
      p++;
   }
   *pp = p;
   return n;
}

//--------------------------------------------------------------------------
uint16_t get_int(char **pp) {
//--------------------------------------------------------------------------
   int n = 0;
   char *p = *pp;

   while ( *p >= '0' && *p <= '9') {
      n = n * 10 + (*p - '0'); 
      p++;
   }
   *pp = p;
   return n;
}


