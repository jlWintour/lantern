//----------------------------------------------------------------------
//  File: lantern_face_control
//
//  Hardware: Arduino Pro Mini 5V 16Mhz
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
#include <Servo.h>
#include <FastLED.h>

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

#define LEFT_DATA_PIN       2  
#define LEFT_SERVO_PIN      9

#define RIGHT_DATA_PIN      4
#define RIGHT_SERVO_PIN    10   // SPI SS


#define ROTARY_DT_PIN       6      
#define ROTARY_CK_PIN       7
#define ROTARY_BUTTON_PIN   A6 

#define NIXIE_BUZZ_PIN      8
#define NIXIE_RED_PIN       3
#define NIXIE_GREEN_PIN     5 
#define NIXIE_BLUE_PIN     11    
 
#define NIXIE_SD_PIN       12   // SPI MISO 
#define NIXIE_CK_PIN       13   // SPI SCK  (Arduino LED) 
#define NIXI_BLANK_PIN     14   // * No pin available 

#define JOY_VRY_PIN         A0 
#define JOY_VRX_PIN         A1  
#define JOY_BUTTON_PIN      A2 



//--------[ NeoPixel Configuration ]--------
#define NUM_LEDS    9
#define BRIGHTNESS  100  // vs 75

CRGB leftLeds[NUM_LEDS];
CRGB rightLeds[NUM_LEDS];

Servo leftServo, rightServo;



//--------[ Global User Interface state variables ]--------
char cmd_ptr;
char cmd[16];
char cmd_rdy;

#define THIS_YEAR 2025;


//---------------------------------------------------------------
//       Utilities
//---------------------------------------------------------------
uint16_t get_hex(char **p);



//--------------------------------------------------------------------------
class NixieLeds_cl {
//--------------------------------------------------------------------------
   uint8_t  red;
   uint8_t  green; 
   uint8_t  blue;

public:

   NixieLeds_cl(char c) {
      red   = 0;
      green = 85;
      blue  = 170;
   }

   control(uint8_t r, uint8_t g, uint8_t b ) {
      red = r;
      green = g;
      blue = b;

      analogWrite(  NIXIE_RED_PIN,   red);   
      analogWrite(NIXIE_GREEN_PIN, green); 
      analogWrite( NIXIE_BLUE_PIN,  blue); 
      // analogWrite(NIXIE_BUZZ_PIN, 128); 
   }
};

NixieLeds_cl nixieLeds(0);



//--------------------------------------------------------------------------
class Nixies_cl {                           // Class for Nixie display tubes
//--------------------------------------------------------------------------
  const uint16_t poll_timer_init_c    =  10;    // in ms
  const uint16_t display_timer_init_c = 400;    // in poll intervals

  uint16_t current_year;
  uint16_t target_year;
  char     auto_mode;
  char     underway;
  char     ready;
  char     phase;
  uint16_t poll_timer;
  uint16_t display_timer;
  uint8_t  idle_r;
  uint8_t  idle_g;
  uint8_t  idle_b;

  void display(uint16_t num) {
    //         0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 2 2 2 2 2 2 2 2 2 2 3 3 3 3 3 3 3 3 3 3 4 4
    //         0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1           
    //         |      ...0         |:. |       ..0.        |       .0..        |       0...        | 
    //         9 8 7 6 5 4 3 2 1 0 * * 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
    uint16_t pixels;
    uint8_t  bit_value;
    uint16_t ln = num;

    for (int d = 0; d < 4; d++) {
       // Set the relevant pixel in this digit
       //  print(ln, b, dv, b-dv)
       pixels = 1 << (ln % 10);
       ln = ln / 10;

       for (int bit_count = 9; bit_count >= 0; bit_count--) {
          bit_value = (pixels & (1 << bit_count) ) ? HIGH : LOW;
          digitalWrite(NIXIE_SD_PIN, bit_value);
          digitalWrite(NIXIE_CK_PIN, HIGH);
          digitalWrite(NIXIE_CK_PIN, LOW);
       }

       // If we just finished digit 0, shift in 2 junk bits to represent the 2 ':' pixels
       if (d == 0) {
          digitalWrite(NIXIE_CK_PIN, HIGH);
          digitalWrite(NIXIE_CK_PIN, LOW);
          digitalWrite(NIXIE_CK_PIN, HIGH);
          digitalWrite(NIXIE_CK_PIN, LOW);
       }
    } // all digits
  } // display

  void blank() {
    //         0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 2 2 2 2 2 2 2 2 2 2 3 3 3 3 3 3 3 3 3 3 4 4
    //         0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1           
    //         |      ...0         |:. |       ..0.        |       .0..        |       0...        | 
    //         9 8 7 6 5 4 3 2 1 0 * * 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0

    digitalWrite(NIXIE_SD_PIN, LOW);
    for (int d = 0; d < 4; d++) {
      for (int bit_count = 9; bit_count >= 0; bit_count--) {
        digitalWrite(NIXIE_CK_PIN, HIGH);
        digitalWrite(NIXIE_CK_PIN, LOW);
      }

      // If we just finished digit 0, shift in 2 junk bits to represent the 2 ':' pixels
      if (d == 0) {
        digitalWrite(NIXIE_CK_PIN, HIGH);
        digitalWrite(NIXIE_CK_PIN, LOW);
        digitalWrite(NIXIE_CK_PIN, HIGH);
        digitalWrite(NIXIE_CK_PIN, LOW);
      }
    } // all digits
  } // blank


public:

   Nixies_cl(char c) {
      current_year  = THIS_YEAR;
      target_year   = THIS_YEAR; 
      underway      = 0;
      idle_r        = 0;
      idle_g        = 0;
      idle_b        = 0x80;
      poll_timer    = poll_timer_init_c;
      display_timer = 0;
      ready         = 0;
      phase         = 0;
      blank();
   }

   void tick() {
      if (poll_timer) poll_timer--;
   }

   void set_ready() {
     ready = 1;
     // Ready now
     leftLeds[0]  = CRGB::DarkMagenta;
     rightLeds[0] = CRGB::DarkMagenta;
     FastLED.show();
   }

   void launch() {
     underway = 1;
   }
   void arrive() {
      current_year = target_year;
      underway = 0;
   }

   void up(uint16_t incr) {
      target_year = (target_year < (2100 - incr)) ? target_year + incr : 2100;
   }

   void down(uint16_t decr) {
     target_year = (target_year >= decr) ? target_year - decr : 0;   
   }

   uint16_t get_target() {
      return target_year;
   }

   void set_target(uint16_t y) {
      target_year = y;
   }

   void set_current_year(uint16_t y) {
      current_year = y;
   }

   void set_idle_color(uint8_t r, uint8_t g, uint8_t b) {
     idle_r = r;
     idle_g = g;
     idle_b = b;
     idle_r = r;
   }

   void show(uint16_t num) {
     display(num);
     display_timer = display_timer_init_c;
   }

  void update() {
    if (poll_timer) return;
    poll_timer = poll_timer_init_c;

    // Update Nixie display
    if (ready) {
      if (display_timer) {
         display_timer--;
      } else {
         display(target_year);
      }
    }
    else {
      blank();
    }

    // Update Nixie LEDs
    if (!ready) {
      if (display_timer) {
        display_timer--;
      }
      else {
        switch (phase) {
        case 0: leftLeds[0]  = CRGB::Black;
                rightLeds[0] = CRGB::Black;
           break;
        case 1: leftLeds[0]  = CRGB::Black;
                rightLeds[0]  = CRGB::Green;
                break;
        case 2: leftLeds[0]  = CRGB::Green;
                rightLeds[0] = CRGB::Black;
                break;
        case 3: leftLeds[0]  = CRGB::Green;
                rightLeds[0] = CRGB::Green;
                break;
        }
        FastLED.show();
        phase = (phase >= 3) ? 0 : phase + 1;
        display_timer = 75;
      }

    } else 
      if (underway) {
        nixieLeds.control(0x00, 0x40, 0x00);   // green when underway
      } else if (current_year != target_year) {
        nixieLeds.control(0xff, 0x0, 0x00);   // red when not current
      } else {
        nixieLeds.control(idle_r, idle_g, idle_b); // idle color while idle
    }
  }  // update
};


Nixies_cl nixies(0);



//------------------------------------------------------------------------
class Joystick_cl {
//------------------------------------------------------------------------
  const   uint16_t poll_timer_init_c = 100;
  const   uint16_t repeat_timer_init_c = 10;
  const   uint16_t one_sec_timer_init_c = 1000;

   char     left, right, up, down, press;
   char     last_left, last_right, last_up, last_down, last_press;
   char     underway;
   char     ready;
   uint8_t  auto_mode;
   uint8_t  launch_interval;
   uint8_t  launch_timer;
   uint16_t one_sec_timer;
   uint16_t poll_timer;
   uint16_t repeat_timer;
   uint16_t target;

public:
   Joystick_cl(char c) {
      last_left   = 0;
      last_right  = 0;
      last_up     = 0;
      last_down   = 0;
      last_press  = 0;
      poll_timer   = 0;
      repeat_timer    = 500;
      auto_mode       = 0;
      one_sec_timer   = one_sec_timer_init_c;
      launch_interval = 30;
      launch_timer    = launch_interval;
      underway        = 0;
      ready           = 0;
   }

   void tick() {
     if (poll_timer) poll_timer--;
     if (one_sec_timer) {
       one_sec_timer--;
     } else {
       if (launch_timer) launch_timer--;
       one_sec_timer = one_sec_timer_init_c; 
     }
   }
  void set_ready() {
    ready = 1;
  }

  void arrive() {
    launch_timer = launch_interval;
    underway = 0;
  }

   void set_auto_mode(uint8_t am) {
     auto_mode = am;
     if (auto_mode) {
       nixies.set_idle_color(0xff, 0x00, 0x80);  // pink in auto mode)
       nixies.show(launch_interval);
     } else {
       nixies.set_idle_color(0x00, 0x00, 0x80);  // blue in manual mode)
     }
   }

  void launch_interval_up() {
    if (launch_interval < 255) {
      launch_interval++;
      nixies.show(launch_interval);
      launch_timer = launch_interval;
    }
   }

  void launch_interval_down() {
    if (launch_interval > 2) {
      launch_interval--;
      nixies.show(launch_interval);
      launch_timer = launch_interval;
    }
   }

   void poll() {
      uint16_t v;
      long     rn;

      if (!ready) return;
      if (poll_timer) return; 
      poll_timer = poll_timer_init_c;

      // Detect the state of the button and joystick (simple digital joystick - no proportional values)
      v = analogRead(JOY_BUTTON_PIN);
      press = (v <128);

      v = analogRead(JOY_VRX_PIN);
      right = (v > 0x280);
      left  = (v < 0x180);

      v = analogRead(JOY_VRY_PIN);
      up    = (v > 0x280);
      down  = (v < 0x180);


      if (auto_mode) {
        // we're in auto mode, so we dont scan the joystick
        // Instead, we launch whenever the launch timer expires
        if (!launch_timer && !underway) {
          // Time to launch
          rn = random(0,2100);
          target = rn;
          nixies.set_target(target);
          Serial.print("G");
          Serial.println(target, DEC);
          nixies.launch();
          underway = 1;
        }
      } else {
        // Not in auto mode, so respond to joystick
        // on joystick button press, Send Go meassage to the mother ship
        if (press==1 && last_press==0) {
          Serial.print("G");
          v = nixies.get_target();
          Serial.println(v, DEC);
          nixies.launch();
          underway = 1;
        }

        // As the joystick moves in any direction, count the year 1 tick
        if (   up ==1 && last_up   == 0) nixies.up(1); 
        if ( down ==1 && last_down == 0) nixies.down(1);
        if (right ==1 && last_right== 0) nixies.up(  100);
        if ( left ==1 && last_left == 0) nixies.down(100);

        // See if we've been holding the stick beyond the timeout
        if (up||down||left||right) {
           if (repeat_timer != 0) {
              repeat_timer--;
           } 
        } else {
           repeat_timer = repeat_timer_init_c;
        }

        // If we're still holding and the timer has expired, then repeat
        if (   up == 1 && repeat_timer == 0) nixies.up(1);
        if ( down == 1 && repeat_timer == 0) nixies.down(1);
        if ( left == 1 && repeat_timer == 0) nixies.down(100);
        if (right == 1 && repeat_timer == 0) nixies.up(100);

        last_left  = left; 
        last_right = right;
        last_up    = up;
        last_down  = down;
        last_press = press;
      }
   }
};

Joystick_cl joystick(0);
 


//-----------------------------------------------------------------------------
class Rotary_encoder_cl {
//-----------------------------------------------------------------------------
  const   uint16_t  poll_timer_init_c     = 5;   // ms
  const   uint8_t   debounce_timer_init_c = 1;    // poll_intervals
  const   uint8_t   press_timer_init_c    = 4;    // poll_intervals

  char    ck, dt, press;
  char    ck_last, press_last;
  char    press_state, press_state_last;
  char    auto_mode;
  uint16_t  poll_timer;       // in ms
  uint16_t  press_timer;      // in units of poll_intervals
  uint8_t   ck_timer;         // in poll_intervals 
  uint8_t   ck_state;     
  uint8_t   ck_state_last;     

public:

  Rotary_encoder_cl(char c) {
    ck = (digitalRead(ROTARY_CK_PIN)== HIGH);
    ck_last          = ck;
    ck_state         = ck;
    ck_state_last    = ck_state;
    poll_timer       = poll_timer_init_c;
    press_timer      = press_timer_init_c;
    press_state      = 0;
    press_last       = 0;
    auto_mode        = 0;
  }

  void tick() {
     if (poll_timer) poll_timer--;
  }

  void poll() {
    uint16_t v;
    uint16_t target;

    // See if it's time to poll
    if (poll_timer) return;
    poll_timer = poll_timer_init_c;

    // Poll button: Restart on changesm, tick down while stable, 
    // and change button state when timer expires. 
    press = (analogRead(ROTARY_BUTTON_PIN) < 128);

    if (press != press_last) {
      press_timer = press_timer_init_c;

    } else if (press_timer) {
      press_timer--; 

    } else {
      press_state = press;
    }
    press_last = press;

    // When button state changes from unpressed to pressed, toggle auto mode.
    if (press_state == 1 && press_state_last == 0) {
      auto_mode = !auto_mode;
      joystick.set_auto_mode(auto_mode);
    }
    press_state_last = press_state;

    // Poll Rotation: Debounce ck signal to form ck_state. Rise of ck state indicates movement, 
    // and dt state indicates direction. Tick launch interval up and down on ratoation steps accordingly. 
    dt = (digitalRead(ROTARY_DT_PIN)== HIGH);
    ck = (digitalRead(ROTARY_CK_PIN)== HIGH);

    if (ck != ck_last) {
      ck_timer = debounce_timer_init_c;
    } else if (ck_timer) {
      ck_timer--;
    } else {
      ck_state = ck;
    }
    ck_last = ck;

    if (ck_state == 1 && ck_state_last == 0) {
      if (dt == 0) {
        joystick.launch_interval_up();
      } else {
        joystick.launch_interval_down();
      }
    }
    ck_state_last = ck_state;
  } // poll()
};  // Rotary_encoder class

Rotary_encoder_cl rotary_encoder(0);




 
//--------[ Timer ISR ]--------
unsigned long int last_ms;




//--------------------------------------------------------------------------
void setup() {
//--------------------------------------------------------------------------
   //--------[ 1ms Interrupt ]--------
   OCR0A = 0xAF;
   TIMSK0 |= _BV(OCIE0A);
   last_ms = 0;
   
   FastLED.addLeds<NEOPIXEL,  LEFT_DATA_PIN>( leftLeds, NUM_LEDS);
   FastLED.addLeds<NEOPIXEL, RIGHT_DATA_PIN>(rightLeds, NUM_LEDS);
   FastLED.setBrightness( BRIGHTNESS );

 //  leftServo.attach(LEFT_SERVO_PIN);
 //  leftServo.write(150);
 //  rightServo.attach(RIGHT_SERVO_PIN);
 //  rightServo.write(50);

   pinMode(ROTARY_DT_PIN,     INPUT_PULLUP);
   digitalWrite(ROTARY_DT_PIN, HIGH);
   
   pinMode(     ROTARY_CK_PIN, INPUT_PULLUP);
   digitalWrite(ROTARY_CK_PIN, HIGH);

   pinMode(     ROTARY_BUTTON_PIN, INPUT_PULLUP);
   digitalWrite(ROTARY_BUTTON_PIN, HIGH);
   
   pinMode(NIXIE_RED_PIN, OUTPUT);
   pinMode(NIXIE_GREEN_PIN, OUTPUT);
   pinMode(NIXIE_BLUE_PIN, OUTPUT);
   pinMode(NIXIE_BUZZ_PIN, OUTPUT);
   
   pinMode(NIXIE_SD_PIN, OUTPUT); 
   pinMode(NIXIE_CK_PIN, OUTPUT);
   
   cmd_ptr = 0;
   cmd_rdy = 0;
   Serial.begin(57600);
   nixies.set_current_year(THIS_YEAR);
   Serial.print("R\n");

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

   nixies.tick();
   //nixieLeds.tick();
   joystick.tick();
   rotary_encoder.tick();
}



//---------------------------------------------------------------------------
//ISR (SPI_STC_vect) {                               // SPI interrupt routine
//---------------------------------------------------------------------------
//   spi.transfer();
//}



//--------------------------------------------------------------------------
void loop() {
//--------------------------------------------------------------------------
   int command;
   char c;

   //do_left_gaugeServo();
   //do_right_gaugeServo();

   joystick.poll();
   rotary_encoder.poll();
   nixies.update();

   //--------[ Command interpreter ]--------
   
   command = Serial.read(); 
   if (command > 0 ) {
     c = command & 0x7f;
     if (cmd_ptr < 14) {
       cmd[cmd_ptr++] = c;
     }
     if (c == '\r') {
       cmd[cmd_ptr++] = c;
       cmd[cmd_ptr] = 0;
       cmd_rdy = 1; 
     }
   }

   if (cmd_rdy) {
     if (cmd[0] == 'A') {      // Arrive command
        nixies.arrive();
        joystick.arrive();
     } 
     else if (cmd[0] == 'R') { // Ready command (seed)
       nixies.set_ready();
       nixies.arrive();
       joystick.set_ready();
       Serial.print("r\n");
     }
     else if (cmd[0] == 'E') {   // Echo command
       Serial.println(cmd);
     }
     cmd_rdy = 0;
     cmd_ptr = 0;
   }  
 }

 


//---------------------------------
void do_left_gaugeServo() {
//---------------------------------
   for  (int pos=10; pos<=80; pos++) {
 //     leftServo.write(pos);
      delay(50);
   }
   for (int pos=80; pos>10; pos--) {
//     leftServo.write(pos);
      delay(50);
   }
}


//---------------------------------
void do_right_gaugeServo() {
//---------------------------------
   for  (int pos=10; pos<=80; pos++) {
//      rightServo.write(pos);
      delay(50);
   }
   for ( int pos=80; pos>10; pos--) {
//      rightServo.write(pos);
      delay(50);
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


