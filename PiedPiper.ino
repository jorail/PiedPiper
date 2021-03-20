/* PiedPiper
  This code is published with GNU General Public License GPL v3, J. Ruppert, 2021-02
  
  The purpose is to control a model train motor with whistle tone signals.
  The work is inspired by the project 'free your model train' proposed at https://freie-software.org/free-your-model-train/
  Code and layout are developed for ESP32 (before Teensy 4.0) but can easily modified for other micro controllers.
  Sound evaluation is performed by FFT analysis in the microprocessor.
  Identified tone signals are processed to commands for change of the speed level and transfered via pulse width modulation (PWM) output to a H-bridge motor control IC.
  The idea for the tone evaluation and part of the functions are based on 'Audio Tone Input' (toneinput.ino) by Tony DiCola, which is published with MIT License (see below) 
  as part of the ardafruit learning guide and examples at http://learn.adafruit.com/fft-fun-with-fourier-transforms/. The corresponding function headings are marked by 'MIT License'.
    
  GNU General Public License Version 3, GPLv3
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.


  The MIT License (MIT) for marked sections with copyright (c) 2013 Tony DiCola
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.


  Version track:
  001 define pins
  002
  003 adjust power led method to flashing with loop count, setting flash frequency and unlit period of it, define morsecode and commands
  004 2020-10-17 morse code parsing to do/speed_command
  005 speed_command and speedAdjust subroutine with speed_wait_countdown for delay of next step of speed adjustment
  006 monitor speed_command in subroutines and main loop: // ### do not use String in front of speed_command when passing command in subroutine and if loop, as this will define a new and !non global vairable, which becomes unavailable outside theis if loop!
  007 2020-10-18 evaluate first speed_command character here: speed_command.charAt(0) bei switch case loop based on char() value
  008 2020-10-18 consolidated version for morse command parsing
  009 2020-10-18 further consolidation, descirption, transfer to PiedPiper sketch, indication of license and copy rights
  010 2020-10-18 further consolidation
  011 2020-10-18 resolve long delay in morse code parsing, due to condition of if loop
  012 2020-10-18 integrate PWM motor driver from 'whistle004.ino'
  013 2020-10-19 consolidation (MotorIC error flag output damaged)
  014 2020-10-19 consolidated morse code interpretation and motor ic commands, includin '?' as request for sketch info
  015 2020-10-19 consolidated analogWrite(motor2,(16-v)*(1-speedoffset)*16), for start with exact speedoffset at speedlevel 1, speedoffset of pulse width modulation (PWM) for speed_level 1 as ratio of maximum pulse width
  016 2020-10-19 morse code from switch interpretation integrated as subroutine MorseCodeDecoderSwitch
  017 2020-10-20 pass morsecode to speed_command, improve code and Serial.print commands
  018 2020-10-20 error resolved for passing currentSignal from switch to MORSE_CODE
  019 2020-10-20 Reorganize Main Loop
  020 2020-10-21 Main loop fixed with fractions of LOOP_COUNT for monitoring, MorseCodeDecoderSwitch001 fixed, debugging MONITOR added
  021 2020-10-21 consolidated version
  022 2020-10-22 consolidated, fix backward acceleration failures/mismatch for motor1 PWM calculation, 2xlongSignal as additional signal for emergency brake, 3x'-' for code/sketch information and motor setting indicated by LEDs, LED information output on ? or '---' request, reordering subroutines
  023 2020-10-23 consolidated 
  024 2020-10-23 test tone detection at 1000 Hz, successful with flute, red LED indicates tone_signal
  025 2020-10-23 test tone detection with tone loop and tone signal counting ag 1100 HZ, using tone_array for gliding average/threshold of tone signals
  026 2020-10-24 valarray fixed, tone_signal according to evaluation of gliding average on tone_array(10) > TONE_THRESHOLD2_DB, include red and green LED to indicate tone detection on instataneous pass of TONE_THRESHOLD1_DB, or average pass of TONE_THRESHOLD2_DB
  027
  028 2020-10-25 Tone 0 h = 988 Hz, THRESHOLD1 = 12 dB, THRESHOLD2 = 14 dB, tone_array size = 7, increased resolution 20 Hz by SAMPLE_RATE_HZ = 5120 Hz, with TONE_ERROR_MARTING = 50 Hz i.e. evaluation of 5 frequency bands with center 988 Hz; Frequency Table, 
                 TONE_FREQ_OFFSET for omission of low frequency energy, when comparing the energy content in the tone window to other frequencies
  029 2020-10-27 increase of number of speed levels from 16 to 32 for smoother speed adjustments, *command number of symbols doubled accordingly 
  030 2020-10-27 consolidated version, reviewed and tested  
  031 2020-11-01 LIGHT INPUT via photo LED signa: pin 20 = LOW
  032 2020-11-15 adjusted tone thresholds  
  033 2020-11-15 reorganise TONE_THRESHOLD2_DB   
  034 2020-11-15 monitor clarity of active tone signal in tone_loop, comment out for faster processing
  035 2020-11-22 License clarification: GNU GPL v3, some marked sections under MIT license
  036 2020-12-06 ESP32 adaptation
  037 2020-12-07 ESP32 adaptation with LEDC
  038 2020-12-07 ESP32 adaptation for mic signal input
  040 2020-12-08 ESP32 solve ESP32 specific defitions with #ifdef
  041 2020-12-11 ESP32 comment out tone sampling for first test of integrated mounting with L293D H-bridge motor IC
  042 2020-12-12 ESP32 test full equippment
  043 2020-12-12 ESP32 change motor output to output pins 32, 33, 25
  044 2020-12-22 ESP32 brown out problem
  045
  046 2021-01-10 ESP32 & TLE5206-2S; ESP startup with touch sensor, (internal temperatur reading not working, because hardware was disontinued in new ESP32 versions) 
  047 2021-01-10 ESP32 & TLE5206-2S: Adjust PWM channel managment for TLE5206-2S, consolidated
  048 2021-01-10 ESP32 & TLE5206-2S testing
  049 2021-01-10 ESP32 & TLE5206-2S consolidated, use redLED as powerLED with very short blink
  050 2021-01-12 2nd test...  of burnout prevention with ESP32 no4, add reset analysis at startup, correct ESP & TLE5206 speed setting, flash frequency calibrated to 1 s
  051 2021-01-16 adjust ESP32 touch read and TLE5206 motor_ic_error_flag readings
  052 2021-02-27 modified ESP32 LDO protection with TVS diode and 1kOhm resistor at 3.3V output for minimum current
  054 2021-02-28 assess TLE5206 error flag for emergency bereak
  055 2021-02-28 modify error display on LED
  056 2021-03-20 for GitHub repository: https://github.com/jorail/PiedPiper
      
*/

#define ARM_MATH_CM4
//#include <arm_math.h>
#include <valarray>
#include <arduinoFFT.h>
//#include <dsps_fft4r.h> 
//#include <wavelib.h>

////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION
// <=######## These values can be changed to alter the behavior of the tone analysis.
// Other values relate to the spectrum display.
////////////////////////////////////////////////////////////////////////////////

String SKETCH_INFO = "PiedPiper_056.ino, GNU General Public License Version 3, GPLv3, J. Ruppert, 2021-03";
#define ESP32      //option to adjust code for interaction with different type of microProcessor (default or comment out or empty, i.e. the else option in the if statement = Teensy4.0)
#define TLE5206    //option to adjust code for interaction with different type of MotorIC        (default or comment out or empty, i.e. the else option in the if statement = L293D)

//toneLoop, parameters for tone signal detection
int SAMPLE_RATE_HZ = 5120;             // Sample rate of the audio in hertz. Default= 9000 Hz => resolution 37 Hz; increased resolution = 5120 => resolution = 20 Hz => highest detectable frequency = 2500 Hz
const int FFT_SIZE = 256;              // Size of the FFT.  Realistically can only be at most 256 = default: Frequency bin siez (Hz) = SAMPLE_RATE_HZ/FFT_SIZE
float TONE_THRESHOLD1_DB = 11.0;       // <=########## Sensitivity threshold (in decibels) for energy in tone window, which must be above energy in other frequencies for successful detection, default 10.0. in quite room, default 12.0 with speach in the environment
std::valarray<int> tone_array(8);      // <=########## setting room for gliding average of tones detected in tone loop, default tone_array size = 10, lower values for faster response time
float TONE_THRESHOLD2_DB = 9.0;       // <=########## Sensitivity threshold (in decibels) for the gliding average of tones, which must be above other frequencies for successful, default 15.0 in quite room, default 14.0 in noisy environment
int  LIGHT_THRESHOLD = 20;            // <=######## if below threshold then the light signal is active, choose analog input threshold value between 1 and 1023
int TONE_FREQ_OFFSET = int(200/(SAMPLE_RATE_HZ/FFT_SIZE)); // Default 200 Hz converted into number of low frequency bins to be skipped for 'windowMean' evaluation and difference of energy in window vs. in other frequencies
const int TONE_LOWS[] = {              // <=########## Lower bound (in hz) of each tone in the input tone window (or sequence). Default single tone = h'' = 988 Hz. Default tone sequence: 1723, 1934, 1512, 738, 1125. 
  988
};
const int TONE_HIGHS[] = {             // <=########## Upper bound (in hz) of each tone in the input tone window (or sequence). Default single tone = h'' = 988 Hz. Default tone sequence: 1758, 1969, 1546, 773, 1160. 
  988
};
// Margin for calculation of the low and high frequency bin limites for the currently expected tone.
int TONE_ERROR_MARGIN_HZ = 50;         // <=########## Allowed fudge factor above and below the bounds for each tone input. Default = 50 Hz

/* Tone frequency table
Tone Hertz      Tone Hertz      Tone Hertz        Tone    Hertz       Tone   Hertz
''A  27,50      C   65,41       c'    261,63      c'''    1.046,50    c''''' 4.186,01
''B  29,14      Des 69,30       des'  277,18      des'''  1.108,73           
''H  30,87      D   73,42       d'    293,66      d'''    1.174,66           
'C   32,70      Es  77,78       es'   311,13      es'''   1.244,51           
'Des 34,65      E   82,41       e'    329,63      e'''    1.318,51           
'D   36,71      F   87,31       f'    349,23      f'''    1.396,91           
'Es  38,89      Ges 92,50       ges'  369,99      ges'''  1.479,98           
'E   41,20      G   98,00       g'    392,00      g'''    1.567,98           
'F   43,65      As  103,83      as'   415,30      as'''   1.661,22           
'Ges 46,25      A   110,00      a'    440,00      a'''    1.760,00           
'G   49,00      B   116,54      b'    466,16      b'''    1.864,66           
'As  51,91      H   123,47      h'    493,88      h'''    1.975,53           
'A   55,00      c   130,81      c''   523,25      c''''   2.093,00           
'B   58,27      des 138,59      des'' 554,37      des'''' 2.217,46           
'H   61,74      d   146,83      d''   587,33      d''''   2.349,32           
                es  155,56      es''  622,25      es''''  2.489,02           
                e   164,81      e''   659,26      e''''   2.637,02           
                f   174,61      f''   698,46      f''''   2.793,83           
                ges 185,00      ges'' 739,99      ges'''' 2.959,96           
                g   196,00      g''   783,99      g''''   3.135,96           
                as  207,65      as''  830,61      as''''  3.322,44           
                a   220,00      a''   880,00      a''''   3.520,00           
                b   233,08      b''   932,33      b''''   3.729,31           
                h   246,94      h''   987,77      h''''   3.951,07           
*/
// Tone sequence detection
int tonePosition = 0;                  // start with first TONE_LOWS[] and TONE_HIGH[], multible values in the array allow for tone sequence code detection
int TONE_WINDOW_MS = 4000;             // Maximum amount of milliseconds allowed to enter the full tone sequence detection, default 4000. without running out of memory for buffers and other state

// Micor Processor and Development Board Pin-out definition

#ifdef ESP32 

  //#include <WiFi.h>
  //#include <WiFiClient.h>
  //#include <WiFiAP.h>
  // input output defintions
  const int AUDIO_INPUT_PIN = 14;        // Input D14 = ADC16 pin for audio signal.
  const int LIGHT_INPUT_PIN = 27;        // Input D27 = ADC17 pin for light signal.
  const int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
  const int ANALOG_READ_AVERAGING = 8;  // Number of samples to average with each ADC reading. Recommanded for ESP32 =8. For Teensy 4.0 successfully tested with =16
  const int POWER_LED_PIN = 4;          // optional output pin for power LED (use ESP32 GPIO pin D15 next to D02 onboard LED pin), alternatively, if no seperate power LED is attached, redLED pin can be used with only minor interference
  const int greenLED      = 2;           // PIN of on-board blue LED for simple digitalWrite, D02 pin
  const int redLED        = 4;           // PIN of on-board red LED for simple digitalWrite, after hardware modification by connection via 1kOhm resistero to D04 pin
  const int error_motoric = 25;          // motor H-bridge ic error flag, for TLE5206 on pin 25 must be defined as input, alternatively: D12 
  const int input_switch  = 13;          // D13 (touch switch input: HIGH=steady state; LOW < 50 = activated, Pin 13 = Touch4 = T4 => if((touchRead(input_switch) < 50)
  const int input_switch_level = 50 ;    // default LOW < 50 = activated
  #define LEDC_TIMER_BITS   8            // PWM properties, use 8 bit precission for LEDC timer with 256 levels
  #define LEDC_BASE_FREQ    5000         // PWM properties, use 5000 Hz as a LEDC base frequency
  const int pwmChannel1    = 0;          // required for ESP32 in combination with either L293D or TLE5206 motor ic
  const int pwmChannel2    = 1;          // only required in combination of ESP32 and TLE5206 motor ic
  int dutyCycle           = 200;
  const int motor2        = 32;          // D32 digital output to motor H-bridge ic input 2 setting motor direction4
  const int motor1        = 33;          // D33 digital output to motor H-bridge ic input 1 setting motor direction
  const int motor_enable  = 25;          // D32 PWM output to motor H-bridge ic enable setting motor speed

#else  //uP not definied, defualt for Teensy 4.0
  // input output defintions
  const int AUDIO_INPUT_PIN = 21;        // Input ADC pin for audio signal.
  const int LIGHT_INPUT_PIN = 20;        // Input ADC pin for light signal.
  const int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
  const int ANALOG_READ_AVERAGING = 16 ; // Number of samples to average with each ADC reading. Recommanded for ESP32 =8. For Teensy 4.0 successfully tested with =16
  const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 4.0's onboard LED).
  const int error_motoric =  4;          // motor H-bridge ic error flag
  const int input_switch  =  5;          // manual switch input: HIGH=steady state, LOW=activated
  const int motor1        =  9;          // PWM output to motor H-bridge ic input 1
  const int motor2        = 10;          // PWM output to motor H-bridge ic input 2
  const int greenLED      = 11;          // PIN of LED for simple digitalWrite
  const int redLED        = 12;          // PIN of LED for simple digitalWrite

#endif

// mainloop and monitoring
const int MONITOR_FREQ =   10;         // Frequency of monitoring switch Morse signal input (ca. 10 Hz or 0.1 second, defined by division of main LOOP COUNT)
const int FLASH_FREQ   =   18815;      // Frequency for power LED flashes while void loop running (ca. 1 to 2 seconds, defined by value for main LOOP_COUNT), default 18815 on ESP32
const int FLASH_FREQ_DARK = int(FLASH_FREQ * 0.98); // flash frequency period with unlit power LED, use number close to 1.00, when redLED is used as alternative powerLED

//MorseCodeDecoder
const int dotMinLength   =  200; //dots are more than 100ms long and shorter than dashMinLength
const int dashMinLength  = 1000; //dashes are more than 300ms long
const int TerminalLength = 1500; //wait after last dash/dot for termination of command
const int   morsecode_size = 12;
const char *morsecode[morsecode_size] = {".",  "..",       "...",                              "....", ".....", ".-", ".--",       ".---",             "-", "--", "---", "?"};
const char *commands[morsecode_size]  = {"--", "--------", "--------------------------------", "0",    "00<",   "++",  "++++++++", "++++++++++++++++", "0", "00", "?",   "?"};
const char *meaning[morsecode_size]   = {"decrease speed", "slow down", "break to halt", "fast brake and stop", "fast brake, stop, reverse direction", "increase speed", "speed up", "go fast", "fast brake and stop", "fast brake and stop", "info?", "info?"};
//acceleration is limited to half full speed, i.e. 8x '+' character, while full break is from max and any speed to halt, i.e. 16x '-' character
//use fast brake to immediatly stop the motor by command charcter '0',
//'<'   stands for reverse of direction, which can only be done after a safe full halt, thus multiple leading '0' characters are introduced in front for train inertia
const int MAX_CHARS     = 65;          // Max size of the input command buffer

//speedAdjustment
const int speed_wait_loops = FLASH_FREQ * 0.5;    // Number of loops for delay of next step of speed adjustment, default = 1000000 = ca. 0.5 seconds
const float speedoffset = 0.6;         // offset of pulse width modulation (PWM) for speed_level 1 as ratio of maximum pulse width, default 0.60 @ 9.5 V dc, 0.50 @ 10 Vdc, 0.30 @ 12Vdc, default 0.25 @ 16Vdc

////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

//toneLoop
/* ###### comment out tone sampling
IntervalTimer samplingTimer;      //###TODO correct for ESP32
float samples[FFT_SIZE * 2];
float magnitudes[FFT_SIZE];

int sampleCounter = 0;
unsigned long toneStart = 0;
*/
char commandBuffer[MAX_CHARS];

// mainloop and monitoring
int LOOP_COUNT = 0;            // Monitor the void loop running by flashing power LED
int FLASH_COUNT = 0;

int TONE_LOOP_COUNT =0; 
int TONE_ACTIVE_COUNT =0;  
String speed_command = "";     // variable for parsing the execution of a morse command
int speed_wait_countdown = 0;  // loop count for delay of next step of speed adjustment, set to 0 for immediate execution of new speed_command
int speed_level = 0;           // speed_level for PWD output to motor
int max_speed_level = 32;      // maximum speed level, default =32 in odrder to get smooth speed increase/decrease, adjust speed_wait_loops and number of *commands speed adjust symbols in appropriately 
                               // (max_speed_level=16 as simplified method up to version 028 with half the number of command symbols)
bool speed_direction = true;   // true=forward, false=backward

// variables from whistle004.ino
int Intensity = 0;
int IntensityStep = 8;
int switch_ok = HIGH;          // (not used in version 030)
int motor_ok = HIGH;           // motor ic error flag (not used in version 030 to 45)
int sound = 0;

// variables from MorseCodeDecoder
bool switch_signal = HIGH;
bool tone_signal = false;
bool light_signal = false;
// int  light_level = 0;       //disabled light level reading
unsigned long SignalTimer = 0; //for recording the time how long the button was pressed
int SignalDuration = 0;
bool SignalActive = false;
bool shortSignalActive = false;
bool longSignalActive = false;
bool verylongSignalActive = false; //2xlongSignal as additional signal for emergency brake
bool SignalTerminated = true;
String currentSignal = ""; //string to hold what is currently being signalled, complete Morse command after termination

////////////////////////////////////////////////////////////////////////////////
// MAIN SKETCH FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(input_switch, INPUT);  
 
#ifdef ESP32 
  setCpuFrequencyMhz(80); //reduce CPU frequency for power saving
  Serial.begin(115200);   // Set up serial port.

  // check for brownout or other startup reason
  esp_reset_reason_t reset_reason = esp_reset_reason();
  Serial.println();
  Serial.print("Startup reset_reason: ");
  Serial.print(reset_reason);
  Serial.print(" = ");
  switch (reset_reason)
    {
    case ESP_RST_UNKNOWN:    Serial.println("Reset reason can not be determined"); break;
    case ESP_RST_POWERON:    Serial.println("Reset due to power-on event"); break;
    case ESP_RST_EXT:        Serial.println("Reset by external pin (not applicable for ESP32)"); break;
    case ESP_RST_SW:         Serial.println("Software reset via esp_restart"); break;
    case ESP_RST_PANIC:      Serial.println("Software reset due to exception/panic"); break;
    case ESP_RST_INT_WDT:    Serial.println("Reset (software or hardware) due to interrupt watchdog"); break;
    case ESP_RST_TASK_WDT:   Serial.println("Reset due to task watchdog"); break;
    case ESP_RST_WDT:        Serial.println("Reset due to other watchdogs"); break;
    case ESP_RST_DEEPSLEEP:  Serial.println("Reset after exiting deep sleep mode"); break;
    case ESP_RST_BROWNOUT:   Serial.println("Brownout reset (software or hardware)"); break;
    case ESP_RST_SDIO:       Serial.println("Reset over SDIO"); break;
    default :                Serial.println("Reset reason not recognized"); break;
    }
 
  if (reset_reason == ESP_RST_BROWNOUT) {
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
    delay(180000); // 3 min delay for cool down and power recovery before continuation in deep_sleep_mode
    pinMode(greenLED, OUTPUT);
    for (int i = 0; i < 150; i++) { //indicate brown out error by fast flashing of green LED for 30 seconds
      digitalWrite(greenLED, HIGH);
      delay(100);
      digitalWrite(greenLED, LOW);
      delay(100);
    }
  }

  delay(2000); //wait 2 s at startup in order to alliviate start-up brown-out cycle
  // Initialise LEDs.
  pinMode(POWER_LED_PIN, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  
  digitalWrite(POWER_LED_PIN, LOW);
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, HIGH);
  for (int i = 0; i < reset_reason;i++) {
    digitalWrite(greenLED, LOW);
    delay(480);
    digitalWrite(greenLED, HIGH);
    delay(20);
  }
    
  while (!(touchRead(input_switch) < 50)) { //wait for touch signal at startup in order to alliviate start-up brown-out cycle
    Serial.print("Waiting for startup signal at GPIO pin ");
    Serial.print(input_switch);
    Serial.print(" TOUCH read= ");
    Serial.println(touchRead(input_switch)); 
    
    delay(950); 
    digitalWrite(greenLED, LOW);
    delay(50);
    digitalWrite(greenLED, HIGH);
  }

  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  pinMode(LIGHT_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION); 
  //analogReadAveraging(ANALOG_READ_AVERAGING);  //#####TODO test if adjustment to ESP analog input reading commands is functioning
  analogSetCycles(ANALOG_READ_AVERAGING);        //#####TODO test if adjustment to ESP analog input reading commands is functioning
  
  /*
  // test motor1&2 digital output
  digitalWrite(motor1, HIGH);
  digitalWrite(motor2, HIGH);
  delay (2000);
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW); 
  delay (500);
  digitalWrite(motor1, HIGH);
  digitalWrite(motor2, HIGH);
  delay (2000);
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW); 
  */

#else  //mP not definied, defualt for Teensy 4.0
  // Set up serial port.
  Serial.begin(38400);

  // Initialise LEDs.
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, LOW);
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, HIGH);
  pinMode(greenLED, OUTPUT);
  digitalWrite(greenLED, HIGH);

  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  pinMode(LIGHT_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);
  
#endif

#if defined(ESP32) && defined(TLE5206) //motor IC, TLE5206 is using only 2 input signals but offers an error flag output
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(error_motoric, INPUT_PULLUP); //use internal 45 kOhm pullup resistor
  // configure ESP32 LEDC PWM functionalitites for TLE5206
  ledcSetup(pwmChannel1, LEDC_BASE_FREQ, LEDC_TIMER_BITS);
  ledcAttachPin(motor1, pwmChannel1); // attach the channel to the GPIO to be controlled
  ledcSetup(pwmChannel2, LEDC_BASE_FREQ, LEDC_TIMER_BITS);
  ledcAttachPin(motor2, pwmChannel2); // attach the channel to the GPIO to be controlled

#elif defined(ESP32)&& !defined(TLE5206)  //motorIC not defined, default L293D, using motor_enable for PWM signal
  //pinMode(error_motoric, INPUT);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor_enable, OUTPUT);
  // configure ESP32 LEDC PWM functionalitites
  ledcSetup(pwmChannel1, LEDC_BASE_FREQ, LEDC_TIMER_BITS);
  ledcAttachPin(motor_enable, pwmChannel1); // attach the channel to the GPIO to be controlled

#else   //assume setting for Teensy 4.0 with TLE5206 motor IC
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(error_motoric, INPUT);
  
#endif

  // Clear the input command buffer
  memset(commandBuffer, 0, sizeof(commandBuffer));
  speed_command = "0?0"; //initial brake speed_command in order to check routines and LED indication

/* ###### comment out tone sampling
  // Begin sampling audio
  samplingBegin();
*/

  //finish startup
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);
  delay(500);
  Serial.print("Startup ");
  Serial.println(SKETCH_INFO);
  Serial.println();
  for (int i = 0; i < reset_reason;i++) {
    digitalWrite(greenLED, HIGH);
    delay(20);
    digitalWrite(greenLED, LOW);
    delay(480);
  }
}

////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP
////////////////////////////////////////////////////////////////////////////////

void loop() {
  // Indicate void loop i.e. program is running by frequent flashes
  ++LOOP_COUNT;
  if (LOOP_COUNT == FLASH_FREQ_DARK) {
    digitalWrite(POWER_LED_PIN, HIGH);
  }
  if (LOOP_COUNT >= FLASH_FREQ) {
    digitalWrite(POWER_LED_PIN, LOW);
    monitor_global_variables();
    LOOP_COUNT = 0;
  }

/* ###### comment out tone sampling
  // Calculate FFT if a full sample is available.
  if (samplingIsDone()) {
    // Run FFT on sample data.
    
    arm_cfft_radix4_instance_f32 fft_inst;
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&fft_inst, samples);
    // Calculate magnitude of complex numbers output by the FFT.
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);
    
    
    // Detect tone sequence.
    toneLoop();

    // Restart audio sampling.
    samplingBegin();
  }
*/ 

  // Parse any pending commands.
  parserLoop();

  if (LOOP_COUNT % MONITOR_FREQ == 0) {

    #if defined(ESP32) && defined(TLE5206) //motor IC TLE5206 offers a comprehensive error flag output
      if (!digitalRead(error_motoric)) { //TLE5206 error indicated if error_motoric = LOW
        motor_set(0);                   // emergency break
        speed_level = 0;
        Serial.println("");
        Serial.println("============================================");
        Serial.println("=== TLE5206 motor IC error indicated !!! ===");
        Serial.println("============================================");
        for (int i = 0; i < 50; i++) { //indicate motor error by fast flashing of red and green LED for 10 seconds
          digitalWrite(redLED, HIGH);
          digitalWrite(greenLED, HIGH);
          delay(100);
          digitalWrite(redLED, LOW);
          digitalWrite(greenLED, LOW);
          delay(100);
        }
        while (!(touchRead(input_switch) < 50)) { //wait for touch signal at startup in order to alliviate start-up brown-out cycle
          Serial.print("Waiting for restart signal at GPIO pin ");
          Serial.print(input_switch);
          Serial.print(" TOUCH read= ");
          Serial.println(touchRead(input_switch)); 
          digitalWrite(redLED, HIGH);
          digitalWrite(greenLED, HIGH);
          delay(950);
          digitalWrite(redLED, LOW);
          digitalWrite(greenLED, LOW);
          delay(50);
        }
      }
    #endif
    
    // evaluate manual switch digital reading (or any alternative digital HIGH/LOW signal) for MorseCode
    MorseCodeDecoder();
  }

  // use speed_command information for motor 1. speed adjustment and 2. motor PWM setting
  if (!speed_command.equals("")) {
    adjustSpeed();
  }

  // end of main loop
}

////////////////////////////////////////////////////////////////////////////////
// SPECTRUM DISPLAY FUNCTIONS (J Ruppert 2020, built on principles by Tony DiCola, Copyright 2013, MIT License)
///////////////////////////////////////////////////////////////////////////////
/* ###### comment out tone sampling
void toneLoop() {
  ++TONE_LOOP_COUNT;
  tone_array = tone_array.shift(-1);
  int lowBin = frequencyToBin(TONE_LOWS[tonePosition] - TONE_ERROR_MARGIN_HZ);
  int highBin = frequencyToBin(TONE_HIGHS[tonePosition] + TONE_ERROR_MARGIN_HZ);
  // Get the average intensity of frequencies inside and outside the tone window.
  float window, other;
  windowMean(magnitudes, lowBin, highBin, &window, &other);
  window = intensityDb(window);
  other = intensityDb(other);
  // Check if tone intensity is above the threshold to detect a step in the sequence.
  if ((window - other) >= TONE_THRESHOLD1_DB) {
    // Start timing the window if this is the first in the sequence.
    digitalWrite(redLED, HIGH);
    //toneDetected(); //disable for morse signal detection
    ++TONE_ACTIVE_COUNT;  //just for monitoring
    tone_array[0]= int(window-other);
      
  }
  else {
    digitalWrite(redLED, LOW);
    //tone_array[0] = int((window - other)/2);  // record value if below TONE_THRESHOLD1_DB; alternatively use half of TONE_THRESHOLD1_DB (or 0) for adjusting the sensitivity of the gliding average assessment
    tone_array[0] = int(TONE_THRESHOLD1_DB/2);  // record value if below TONE_THRESHOLD1_DB; alternatively use half of TONE_THRESHOLD1_DB (or 0) for adjusting the sensitivity of the gliding average assessment
  }
  //is the gliding average over time of multiple events, which are above TONE_THRESHOLD1_DB, also above TONE_THRESHOLD2_DB?
  if (tone_array.sum()/tone_array.size()>= TONE_THRESHOLD2_DB) { 
    tone_signal=true;
    digitalWrite(greenLED, HIGH);

    //monitor clarity of active tone signal, comment out for faster processing
    if (TONE_LOOP_COUNT % 8 == 0) {
       Serial.print  ("Average tone_array= ");
       Serial.print  (tone_array.sum()/tone_array.size());
       Serial.print  (" Tone_array: ");
       for (int j = 0; j < 8;++j) {
         Serial.print(tone_array[j]);
         Serial.print(", ");
       }     
    Serial.println(" ");
    }
   
  }
  else {
    tone_signal=false;
    digitalWrite(greenLED, LOW);
  }
}

// monitor successful toneSequenceDetection after sequence of tones (not used in version 030)
void toneSequenceDetected() { //do not call, if uninterrupted main loop is required for morse signal detection
  // Flash the green LED multiple times, when the tone was detected.
  digitalWrite(redLED, HIGH);
  int pause = 50;
  for (int i = 0; i < 10; ++i) {
    digitalWrite(greenLED, HIGH);
    delay(pause);
    digitalWrite(greenLED, LOW);
    delay(pause);
  }
  digitalWrite(redLED, LOW);
}


////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS (by Tony DiCola, Copyright 2013, MIT License)
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  
  // disabled light level reading
  // light_level = analogRead(LIGHT_INPUT_PIN);
  
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter + 1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE * 2) {
    samplingTimer.end();
  }
}

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000 / SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE * 2;
}
*/

////////////////////////////////////////////////////////////////////////////////
// Recording morse code commands active input signals (switch_signal, tone_signal)
////////////////////////////////////////////////////////////////////////////////

void MorseCodeDecoder() {
  //input signal active here by input_switch=LOW, i.e. switch_signal=!switch_switch
  //use alternative inputs (light detection, tone detection together with the switch_signal by logical OR combination '||'

#ifdef ESP32 
    // switch_signal = !digitalRead(input_switch);      //for use with connected normally closed micro switch connected to HIGH and pull-down 10 kOhm resistor
    switch_signal = (touchRead(input_switch) < 50);      //for ESP32 touch sensor reading
  
#else  // uP = Teensy and normally closed microswitch connected to input_switch
    switch_signal = !digitalRead(input_switch);         //for use with connected normally closed micro switch connected to HIGH and pull-down 10 kOhm resistor

#endif
  
  //light sampling analysis
  //light_signal =  (light_level < LIGHT_THRESHOLD);    // for disabling the assessment of light level readings
  light_signal = false;                                 // for disabling the assessment of light level readings
  
  if (switch_signal || tone_signal || light_signal) {  //Morse code can be transmitted via switch or tone or light
    if (SignalActive == false) {
      delay(5);// delay(button.debounce);
      // additional update status in case of bounce
      // button.currentState = digitalRead(button.pin);
      //reference to millis() function usage:
      //https://www.baldengineer.com/millis-tutorial.html
      //https://www.baldengineer.com/detect-short-long-button-press.html
      SignalTimer = millis();
      SignalActive = true;
      SignalTerminated = false;
    }
    SignalDuration = int (millis() - SignalTimer);
    if ((SignalDuration > dotMinLength) && (shortSignalActive == false)) {
      shortSignalActive = true;
    }
    if ((SignalDuration > dashMinLength) && (longSignalActive == false)) {
      longSignalActive = true;
    }
    if ((SignalDuration > 2*dashMinLength) && (longSignalActive == true)) {
      //2xlongSignal as additional signal for emergency brake, immediate activity
      SignalActive = false;
      longSignalActive = false;
      shortSignalActive = false;
      SignalTerminated = true;
      currentSignal="--";
      Serial.println("Morse signal: emergency brake by very long signal " + currentSignal);
      MORSE_CODE(currentSignal.c_str());
      currentSignal = "";
    }
  }
  //signal not active
  else {
    //evaluate length of last signal
    if (SignalActive) {
      SignalTimer = millis();
      SignalActive = false;
      if (longSignalActive) {
        currentSignal += ("-");
        Serial.println("Recognized: " + currentSignal);
        longSignalActive = false;
        shortSignalActive = false;
      }
      if (shortSignalActive) {
        currentSignal += (".");
        Serial.println("Recognized: " + currentSignal);
        //longSignalActive = false;
        shortSignalActive = false;
      }
    }
    else {
      if (SignalTerminated == false) {
        SignalDuration = int (millis() - SignalTimer);
        if ((SignalDuration > TerminalLength)) {
          SignalTerminated = true; //command completed
          Serial.println("Morse signal: " + currentSignal);
          // translate currentSignal Morse code to speed_command; input to MORSE_CODE as: const char *
          MORSE_CODE(currentSignal.c_str());
          currentSignal = "";
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Parse MORSE_CODE and translation to speed_command, sizeof(*morsecode)
////////////////////////////////////////////////////////////////////////////////

void MORSE_CODE (const char* command) {
  for (unsigned int i = 0 ; i < morsecode_size ; ++i) {
    if (strcmp(command, morsecode[i]) == 0) {
      Serial.println();
      Serial.print("GOT MORSE CODE: ");
      Serial.print(morsecode[i]);
      speed_command = commands[i]; // do not use 'String' in front of speed_command when passing command in subroutine and in if loop, as this will define a new and !non global vairable, which becomes unavailable outside this if loop!
      Serial.print(" | SPEED COMMAND: ");
      Serial.print(speed_command);
      Serial.print(" | ACTIVITY: ");
      Serial.println(meaning[i]);
      speed_wait_countdown = 0;    // don't wait but immedeatly (countdown value = 0) start adjusting the motor speed
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// MOTOR SPEED ADJUSTMENT
////////////////////////////////////////////////////////////////////////////////

void adjustSpeed() {
  // Serial.print("Speed_command_received. Speed_wait_countdown = "); //### for monitoring ###
  // Serial.println(speed_wait_countdown);                            //### for monitoring ###
  if (speed_wait_countdown <= 0) {
    Serial.print("adjust speed: ");
    Serial.print(speed_command.charAt(0)); //evaluate first speed_command character
    Serial.print(" | ");

    //###### evaluate first speed_command character here: speed_command.charAt(0)
    switch (char(speed_command.charAt(0))) {
      case '0':        //fast brake, speed_level =0
        speed_level = 0;
        digitalWrite(redLED, HIGH);
        digitalWrite(greenLED, HIGH);
        delay(1000);
        Serial.print("fast brake, speed level set to 0 | ");
        break;
      case '-':        //decrease speed
        --speed_level;
        if (speed_level < 0) {
          speed_level = 0;
          break;
        }
        digitalWrite(redLED, HIGH);
        Serial.print("speed level decreased - | ");
        delay(300);
        break;
      case '+':        //increase speed
        ++speed_level;
        if (speed_level > max_speed_level) {
          speed_level = max_speed_level;
          break;
        }
        digitalWrite(greenLED, HIGH);
        Serial.print("speed level increased + | ");
        delay(300);
        break;
      case '<':        //reverse direction
        speed_direction = !speed_direction;
        if (speed_direction) {
          digitalWrite(greenLED, HIGH);
          delay (200);
          digitalWrite(greenLED, LOW);
          digitalWrite(redLED, HIGH);
          delay (200);
          digitalWrite(redLED, LOW);
          digitalWrite(greenLED, HIGH);
          delay (200);
          digitalWrite(greenLED, LOW);
          delay (200);
          digitalWrite(greenLED, HIGH);
          delay (600);
          Serial.print("direction reversed forward >>> | ");
        }
        else {
          digitalWrite(redLED, HIGH);
          delay (200);
          digitalWrite(redLED, LOW);
          digitalWrite(greenLED, HIGH);
          delay (200);
          digitalWrite(greenLED, LOW);
          digitalWrite(redLED, HIGH);
          delay (200);
          digitalWrite(redLED, LOW);
          delay (200);
          digitalWrite(redLED, HIGH);
          delay (600);
          Serial.println("direction reversed backward <<< | ");
        }
        break;
      case '?':        //question
        Serial.println();
        Serial.println();
        Serial.print("Running arduino/c++ sketch is: ");
        Serial.println(SKETCH_INFO); //print sketch name, license, author
        Serial.println();
        
        //indicate motor direction by green and red LED long flash
        if (speed_direction) {
          digitalWrite(greenLED, HIGH);
          delay(1500);
          digitalWrite(greenLED, LOW);
        }
        else {
          digitalWrite(redLED, HIGH);
          delay(1500);
          digitalWrite(redLED, LOW);
        }
        delay(100);

        //Serial.print morse code information
        Serial.println("Morse code  | speed adjustment                   | meaning ");
        for (int i = 0; i < morsecode_size ; ++i) {
          Serial.print(morsecode[i]);
          int l=(10 - strlen(morsecode[i]));
          for (int i=0; i < l; i++) {
            Serial.print(' ');
          }
          Serial.print("  |  ");
          Serial.print(commands[i]);
          l=(max_speed_level - strlen(commands[i]));
          for (int i=0; i < l; i++) {
            Serial.print(' ');
          }
          Serial.print("  |  ");
          Serial.println(meaning[i]);
        }
        Serial.println();

        //indicate motor speed level by green and red LED very short flashes
        Serial.println("LED INDICATORS"); 
        Serial.println("Information on motor direction:   forward = 1 = green LED; backward = 0 = red LED"); 
        Serial.println("Information on motor speed_level: 0 to 16 short LED flashes");
        Serial.println("Adjustment of motor speed:        green LED flash = speed level + 1; red LED flash = speed level - 1; green & red LEDs long flash = brake, speed level = 0");
        Serial.println("Adjustment of motor direction:    green/red/green + green LED flash = forward = 1;  red/green/red + red LED flash = forward = 1;");
        Serial.println("Tone signal detected:             Single FFT with frequency window > THRESHOLD1: red LED; averaged FFTs frequency windows > THRESHOLD2: green LED = tone signal active");  
        Serial.print  ("Program loop frequencies:         orange flash after ");     
        Serial.print  (FLASH_FREQ);
        Serial.print  (" main loop cycles and "); 
        Serial.print  (FLASH_FREQ/MONITOR_FREQ);
        Serial.println(" times monitoring for input activity");
        for (int i = 0; i < speed_level/2; ++i) {
          delay(480);
          if (speed_direction) {
            digitalWrite(greenLED, HIGH);
            delay(20);
            digitalWrite(greenLED, LOW);
          }
          if (!speed_direction) {
            digitalWrite(redLED, HIGH);
            delay(20);
            digitalWrite(redLED, LOW);
          }
        }    
        delay (1000);
        Serial.println();
        break;
      default:
        // if nothing else matches, do the default
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, HIGH);
        delay (200);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, LOW);
        delay (200);
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, HIGH);
        delay (200);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, LOW);
        delay (200);
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, HIGH);
        delay (200);
        Serial.println("unknown command ??? no change of speed");
        break;
    } // end of switch case

    //switch_ok = digitalRead(input_switch);  // (no condition and value not used in version 030)
    //motor_ok = digitalRead(error_motoric);  // (no condition and value not used in version 030)
    
    // forward new speed_level to motor set
    motor_set(speed_level);
    
    // prepare for next round of speed_command
    speed_wait_countdown = speed_wait_loops;
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, LOW);
    speed_command.remove(0, 1); //ersase first charcter of speed_command, i.e. the command, which was just executed
    //speed_command = ""; //empty speed_command after execution
  } //end of speed wait countdown loop
  --speed_wait_countdown;
}

////////////////////////////////////////////////////////////////////////////////
// MOTOR SET: set motor speed level through PWM output
////////////////////////////////////////////////////////////////////////////////

void motor_set (int v) {
  #if defined(ESP32) && defined(TLE5206)
      if (v > 0 && v <= max_speed_level) {  // speed level 1 to max_speed_level (default=32), not stopped, motor running
                                            //for TLE5206 and start with exact speedoffset at speedlevel 1 use: ledcWrite(pwmChannel2, (1 - static_cast<float>(v)/max_speed_level) * (1-speedoffset) * 256);
        if (speed_direction) { //forward direction
          ledcWrite(pwmChannel1, 255);  // for setting motor1 = HIGH      
          ledcWrite(pwmChannel2, (1 - static_cast<float>(v)/max_speed_level) * (1-speedoffset) * 256);   // for setting motor 2 to PWM settinig for TLE5206 H-Bridge IC motor driver
          Serial.print("motor set: speed_level ");
          Serial.print(v);
          Serial.print(" forward | PWM analog write motor1 = 255, motor2 = ");
          Serial.println( (1 - static_cast<float>(v)/max_speed_level) * (1-speedoffset) * 256);     
        }
        else { //backward direction
          ledcWrite(pwmChannel1, (1 - static_cast<float>(v)/max_speed_level) * (1-speedoffset) * 256);   // for setting motor 2 to PWM settinig for TLE5206 H-Bridge IC motor driver
          ledcWrite(pwmChannel2, 255);  // for setting motor2 = HIGH      
          Serial.print("motor set: speed level ");
          Serial.print(v);
          Serial.print(" backward | PWM analog write motor2 = 255, motor1 = ");
          Serial.println( (1 - static_cast<float>(v)/max_speed_level) * (1-speedoffset) * 256);          
        }
      }
      else { // speed level 0 or out of range 1 to max_speed_level: stop motor
        ledcWrite(pwmChannel1, 0);  // for setting motor 1 = LOW
        ledcWrite(pwmChannel2, 0);  // for setting motor 2 = LOW
        Serial.print("motor set: ============== STOP ============== | ");
        Serial.print(v);
        Serial.print(" | PWM analog write motor1&2 = LOW, PWM = ");
        Serial.println(0);    
    }

  
  #elif defined(ESP32) && !defined(TLE5206) // for ESP32 in combination with L293D Motor IC
      if (v > 0 && v <= max_speed_level) {  // speed level 1 to max_speed_level (default=32), not stopped, motor running
                                            //for L293D and start with exact speedoffset at speedlevel 1 use: ledcWrite(pwmChannel1, ((static_cast<float>(v)/max_speed_level) * (1 - speedoffset) + speedoffset)* 256);
        if (speed_direction) { //forward direction
          digitalWrite(motor1, HIGH);
          digitalWrite(motor2, LOW);          
          ledcWrite(pwmChannel1, ((static_cast<float>(v)/max_speed_level) * (1 - speedoffset) + speedoffset)* 256);   // PWM settinig for L293D H-Bridge IC motor driver
          Serial.print("motor set: speed_level ");
          Serial.print(v);
          Serial.print(" forward | PWM analog write motor1 = HIGH, motor2 = LOW, PWM = ");
          Serial.println(((static_cast<float>(v)/max_speed_level) * (1 - speedoffset) + speedoffset)* 256);     
        }
        else { //backward direction
          digitalWrite(motor1, LOW);
          digitalWrite(motor2, HIGH);   
          ledcWrite(pwmChannel1, ((static_cast<float>(v)/max_speed_level) * (1 - speedoffset) + speedoffset)* 256);   // PWM settinig for L293D H-Bridge IC motor driver
          Serial.print("motor set: speed level ");
          Serial.print(v);
          Serial.print(" backward | PWM analog write motor1 = LOW, motor2 = HIGH, PWM = ");
          Serial.println(((static_cast<float>(v)/max_speed_level) * (1 - speedoffset) + speedoffset)* 256);          
        }
      }
      else { // speed level 0 or out of range 1 to max_speed_level: stop motor
        digitalWrite(motor1, LOW);
        digitalWrite(motor2, LOW);
        ledcWrite(pwmChannel1, 0);  
        Serial.print("motor set: ============== STOP ============== | ");
        Serial.print(v);
        Serial.print(" | PWM analog write motor1&2 = LOW, PWM = ");
        Serial.println(0);    
    }
    
  #else  // mP= default e.g. Teensy 4.0 or Arduino assumed to be in combination with TLE5206 motor IC
    if (v > 0 && v <= max_speed_level) {  // speed level 1 to max_speed_level (default=32), not stopped, motor running
                                          //for start with exact speedoffset at speedlevel 1 use: analogWrite(motor2, (max_speed_level - v) * (1 - speedoffset) * 256/max_speed_level);
                                          //approximate and for maximum of 16 speed levels use:   analogWrite(motor2,(16-v)*(1-speedoffset)*16); 
      if (speed_direction) { //forward direction
        analogWrite(motor1, 255);
        analogWrite(motor2, (max_speed_level - v) * (1 - speedoffset) * 256/max_speed_level);   // Teensy 4.0 PWM settinig for TLE5206S H-Bridge IC motor driver
        Serial.print("motor set: speed_level ");
        Serial.print(v);
        Serial.print(" forward | PWM analog write motor2 = ");
        Serial.println((max_speed_level - v) * (1 - speedoffset) * 256/max_speed_level);
      }
      else { //backward direction
        analogWrite(motor1, (max_speed_level - v) * (1 - speedoffset) * 256/max_speed_level);   // Teensy 4.0 PWM settinig for TLE5206S H-Bridge IC motor driver
        analogWrite(motor2, 255);
        Serial.print("motor set: speed level ");
        Serial.print(v);
        Serial.print(" backward | PWM analog write motor1 = ");
        Serial.println(((max_speed_level - v) * (1 - speedoffset) * 256/max_speed_level));
      }
    }
    else { // speed level 0 or out of range 1 to max_speed_level: stop motor
      analogWrite(motor1, 0);
      analogWrite(motor2, 0);
      Serial.print("motor set: ============== STOP ============== | ");
      Serial.print(v);
      Serial.print(" | PWM analog write motor1&2 = ");
      Serial.println(0);
    }
  #endif
}

////////////////////////////////////////////////////////////////////////////////
// Monitoring global variables in main loop
////////////////////////////////////////////////////////////////////////////////

void monitor_global_variables() {
  Serial.print("Loop count: ");
  Serial.print(LOOP_COUNT);
  Serial.print("! Restart at 0.");
  Serial.print(" Speed_level: ");
  Serial.print(speed_level);
  Serial.print(" Direction: ");
  Serial.print(speed_direction);
  Serial.print(" | ");
  for (int i = 1; i <= max_speed_level; ++i) {
    if (i <= speed_level) {
      if (speed_direction) {
        Serial.print(">");
      }
      else {
        Serial.print("<");
      }
    }
    else {
      Serial.print("~");
    }
  }
  Serial.print(" | ");
  Serial.print("Speed_command: " + speed_command);
  //Serial.println("commandBuffer="+commandBuffer+"| ");
  
  #ifdef ESP32
    switch_ok = !(touchRead(input_switch) < 50);
  #else
    switch_ok = digitalRead(input_switch);
  #endif

  #ifdef TLE5206
   motor_ok = digitalRead(error_motoric);
  #endif
  
  Serial.print("switch_ok= ");
  Serial.print(switch_ok);
  Serial.print(" | motor_ok= ");
  Serial.print(motor_ok);
  Serial.print(" | tone_signal= ");
  Serial.print(tone_signal);
  Serial.print(" | Tone_loop_count= ");
  Serial.print(TONE_LOOP_COUNT);
  TONE_LOOP_COUNT =0; 
  Serial.print(" | Tone_signal_activ= ");
  Serial.print(TONE_ACTIVE_COUNT);
  TONE_ACTIVE_COUNT =0;   
  
  //disabled light_level reading
  //Serial.print(analogRead(LIGHT_INPUT_PIN));  
  //int light_level = analogRead(LIGHT_INPUT_PIN);  
  //Serial.print(" | light_level= ");
  //Serial.print(light_level);  
  //Serial.print(" | light_signal= ");
  //Serial.print(light_signal);  

  #ifdef ESP32
    /*ESP32 internal temperature monitoring (hardware temperature sensor discontinued in new ESP32 versions
    Serial.print(" | ESP32_Temp= ");
    Serial.print((temprature_sens_read() - 32) / 1.8); // Convert raw temperature in F to Celsius degrees
    Serial.print(" C");
    */
    Serial.print(" | GPIO pin ");
    Serial.print(input_switch);
    Serial.print(" TOUCH read= ");
    Serial.print(touchRead(input_switch)); 
    
  #endif
  
  Serial.println( " | send '?' for info");
}

////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS (by Tony DiCola, Copyright 2013, MIT License)
////////////////////////////////////////////////////////////////////////////////

// Compute the average magnitude of a target frequency window vs. all other frequencies.
void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
  *windowMean = 0;
  *otherMean = 0;
  // Notice the first magnitude bin is skipped because it represents the
  // average power of the signal.
  for (int i = 1+TONE_FREQ_OFFSET; i < FFT_SIZE / 2; ++i) { // TONE_FREQ_OFFSET in order to avoid high loads of low frequency noise in peak detection of window vs. other 
    if (i >= lowBin && i <= highBin) {
      *windowMean += magnitudes[i];
    }
    else {
      *otherMean += magnitudes[i];
    }
  }
  *windowMean /= (highBin - lowBin) + 1;
  *otherMean /= (FFT_SIZE / 2 - (highBin - lowBin) - TONE_FREQ_OFFSET);
}

// Convert a frequency to the appropriate FFT bin it will fall within.
int frequencyToBin(float frequency) {
  float binFrequency = float(SAMPLE_RATE_HZ) / float(FFT_SIZE);
  return int(frequency / binFrequency);
}

// Convert intensity to decibels
float intensityDb(float intensity) {
  return 20.0 * log10(intensity);
}

////////////////////////////////////////////////////////////////////////////////
// COMMAND PARSING FUNCTIONS (by Tony DiCola, Copyright 2013, MIT License, amended J. Ruppert 2020)
// These functions allow parsing simple commands input on the serial port.
// Commands allow reading and writing variables that control the device.
// The code also allows interaction with a Python program for Spectrogram display by Tony DiCola
// See instruction and download at http://learn.adafruit.com/fft-fun-with-fourier-transforms/
//
// The code interpretes commands defined in morsecode[] and evaluated in the function MORSE_CODE to a speed_command
// Example morse code commands are:
// ...
// .---
// ---
//
// Direct commands must end with a semicolon character.
//
// Example commands are:
// GET SAMPLE_RATE_HZ;
// - Get the sample rate of the device.
// SET SAMPLE_RATE_HZ 400;
// - Set the sample rate of the device to 400 hertz.
//
// Written Morse Code commands just end with a return after the last dash or dot
// Send '?' character for more information.
//
////////////////////////////////////////////////////////////////////////////////

void parserLoop() {
  // Process any incoming characters from the serial port
  while (Serial.available() > 0) {
    char c = Serial.read();
    // Add any characters that aren't the end of a command (semicolon) to the input buffer.
    if (c != ';') {
      c = toupper(c);
      strncat(commandBuffer, &c, 1);
    }
    else if (c == ';') {  //criterion changed in order to allow additional MORSE_CODE parsing
      parseCommand(commandBuffer); // Parse the command because an end of command token was encountered.
      memset(commandBuffer, 0, sizeof(commandBuffer)); // Clear the input buffer
    }
  }
  // Parse MORSE_CODE commands, which are not terminated with ';' character
  // detect availability of commandBuffer characters by first element in command buffer not equal to 0
  if (commandBuffer[0] != 0) {
    MORSE_CODE(commandBuffer);
    // Clear the input buffer
    memset(commandBuffer, 0, sizeof(commandBuffer));
  }
}

/* ###### comment out tone sampling 
// Macro used in parseCommand function to simplify parsing get and set commands for a variable
#define GET_AND_SET(variableName) \
  else if (strcmp(command, "GET " #variableName) == 0) { \
    Serial.println(variableName); \
  } \
  else if (strstr(command, "SET " #variableName " ") != NULL) { \
    variableName = (typeof(variableName)) atof(command+(sizeof("SET " #variableName " ")-1)); \
  }
*/

void parseCommand(char* command) {

/* ###### comment out tone sampling 
  if (strcmp(command, "GET MAGNITUDES") == 0) {
    for (int i = 0; i < FFT_SIZE; ++i) {
      Serial.println(magnitudes[i]);
    }
  }
  else if (strcmp(command, "GET SAMPLES") == 0) {
    for (int i = 0; i < FFT_SIZE * 2; i += 2) {
      Serial.println(samples[i]);
    }
  }
  else if (strcmp(command, "GET FFT_SIZE") == 0) {
    Serial.println(FFT_SIZE);
  }

  
  GET_AND_SET(SAMPLE_RATE_HZ)
  GET_AND_SET(TONE_ERROR_MARGIN_HZ)
  GET_AND_SET(TONE_WINDOW_MS)
  GET_AND_SET(TONE_THRESHOLD1_DB)
  GET_AND_SET(TONE_THRESHOLD2_DB)
*/
  
}
