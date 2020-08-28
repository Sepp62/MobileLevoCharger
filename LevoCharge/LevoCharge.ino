/*
    Name:       LevoCharge.ino
    Created:    25.07.2020 11:55:07
    Author:     Bernd Woköck

      Version history:
        0.90   07/25/2020  created

    Permission is hereby granted, free of charge, to any person obtaining
    a copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
    OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
    IN THE SOFTWARE.
  
    Can bus timing
    http://www.bittiming.can-wiki.info/
*/

// https://github.com/tonton81/FlexCAN_T4
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0; // CAN1 is Pin 23 (RX) and 22 (TX)
CAN_message_t _canFrame;

#include <ADC.h>
#include <ADC_util.h>

#include "MedianFilter.h"   // https://github.com/daPhoosa/MedianFilter
#include "OneButton.h"      // https://github.com/mathertel/OneButton

// display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display;

// Pins
#define BUTTON_PIN 9 // one button control
#define FET_SWITCH 6 // Smart FET switch
#define LEVO_ON    7 // Levo battery enable

//--------------------------------
// analog input value calculation
// Current: 3152 = 0.0A, 3472 = 4,0A
// Voltage: 17   = 0.0V, 3225 = 41.0V
float _Ui0 = 3152.0f;
float _Uc0 = 17.0f;
float _fact_Ui     = (3472.0f - _Ui0)/4.0f;
float _fact_Uchg   = (3225.0f - _Uc0)/41.0f;
#define MAKE_Ui(a) ((a - _Ui0)/_fact_Ui)
#define MAKE_Uchg(a) ((a - _Uc0)/_fact_Uchg)

//------------
// Teensy ADC
const int PIN_Ui   = A7; // ADC0
const int PIN_Uchg = A6; // ADC1

ADC *_adc = new ADC(); // adc object;

//---------------------
// filtering AD values
int _analogUi   = 0;
int _analogUchg = 0;

MedianFilter _medianUi(31, 2048);
MedianFilter _medianUchg(31, 2000);

// ----------------------------
// builtin ampere and voltmeter results
struct InternalMultimeterST
{
  float current;
  float voltage;
  float energy;
  unsigned long nMeasureCnt = 0L;
}
_multiMeter = { 0.0, 0.0, 0.0, 0L };

// -------------
// LEVO battery
struct LevoBatteryST
{
  float voltage;
  float current;
  int   chStatePercent; 
  float currentEnergy;  // multiply with 1.12
  int   healthPercent;
  float maxCapacity;    // 450Wh instead of 504Wh, multiply with 1.12
  bool  bCharging;
  unsigned long newValueMask;
}
_levoBatt = { 0.0f, 0.0f, 0, 0.0, 0, 0.0, false, 0L };

enum msgMask // bitmask to tag new messages from CAN ---> _levoBatt.newValueMask
{
    EN_CLEAR = 0,
    EN_MSG_400 = 1 << 0,
    EN_MSG_401 = 1 << 1,
    EN_MSG_402 = 1 << 2,
    EN_MSG_403 = 1 << 3,
};

//--------------
// on/off button
bool _bOn = false;         // charging is on
bool _bDebugMode = false;  // show debug/adjust output

OneButton _btn = OneButton(
  BUTTON_PIN,  // Input pin for the button
  true,        // Button is active LOW
  true         // Enable internal pull-up resistor
);

//--------------
// safety checks
const unsigned long GRACE_PERIOD = 1000L; // wait until values are settled after switching on
const float MAX_VOLTAGE = 42.0f; // V
const float MAX_CURRENT= 6.5f; // A
unsigned long _tiGracePeriod = 0L;
enum errors
{
  EN_NONE = 0,
  EN_INT_HICURRENT = 1,
  EN_INT_HIVOLTAGE = 2,
  EN_BAT_HICURRENT = 3,
  EN_BAT_HIVOLTAGE = 4,
};
int _safetyError = EN_NONE;

// -------------
// switch off
void SwitchOff()
{
  digitalWrite( FET_SWITCH, LOW );
  digitalWrite( LEVO_ON, LOW );
  _bOn = false;
}

// -------------------------------------------------------------
// Handler function for a single and long click:
static void handleClick()
{
  if( _btn.isLongPressed())
  {
    // switch off
    if( _bOn )
    {
      SwitchOff();
    }
    
    // reset error 
    if( _safetyError != EN_NONE )
    {
       _safetyError = EN_NONE;
      _levoBatt.voltage = 0.0;
      _levoBatt.current = 0.0;
      _levoBatt.bCharging = false;
    }
    Serial.println("Long pressed!");
  }
  else
  {
    if( !_bOn) 
    {
      // switch on
      _Ui0 = _medianUi.out(); // remember "zero" current for calibration

      digitalWrite( LEVO_ON, HIGH );
      delay( 1000 );
      // todo: wait for Levo battery on
      digitalWrite( FET_SWITCH, HIGH );
      
      _bOn = true;
      _tiGracePeriod = millis() + GRACE_PERIOD;
      _safetyError = EN_NONE;
    }
    
    Serial.println("Clicked!");
  }
}

static void handleDoubleClick()
{
    _bDebugMode ^= 1;
}

// -------------------------------------------------------------
void ShowOutput() // from internal multimeter
{
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);

    if( _bOn )
    {
      display.print( String(_multiMeter.current, 1).c_str() );
      display.print( "A ");
    }
    else
    {  
      display.print("off ");
    }
    display.print( String( _multiMeter.voltage, 1).c_str() );
    display.println( "V");
    display.print( (int)(_multiMeter.energy/36000.0f) ); // every 100ms --> 1h is 3600*10
    display.print( "Wh ");
    if( _multiMeter.nMeasureCnt < 600 )
    {
      display.print( _multiMeter.nMeasureCnt/10 );
      display.print( "s");
    }
    else
    {
      display.print( _multiMeter.nMeasureCnt/600 );
      display.print( "m");
    }
    
    display.display();
}

void ShowOutputLevo()
{
    if( !_levoBatt.bCharging)
    {
      ShowOutput(); // show valöues from internal multimeter when no CAN data is available
      return;
    }

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);

    if( _bOn )
    {
      display.print( String(_levoBatt.current, 1).c_str() );
      display.print( "A ");
    }
    else
    {  
      display.print("off ");
    }
    display.print( String( _levoBatt.voltage, 1).c_str() );
    display.println( "V");

    if( _multiMeter.nMeasureCnt < 600 )
    {
      display.print( _multiMeter.nMeasureCnt/10 );
      display.print( "s ");
    }
    else
    {
      display.print( _multiMeter.nMeasureCnt/600 );
      display.print( "m ");
    }

    if( (_multiMeter.nMeasureCnt/20) % 2 == 0  )
    {
      display.print( (int)(_levoBatt.currentEnergy) ); 
      display.print( "Wh ");
    }
    else
    {
      display.print( _levoBatt.chStatePercent ); 
      display.print( "%  ");
    }

    display.display();
}

void ShowOutputDebugMode()
{
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
  
    if( _bOn )
    {
      display.print( String(_multiMeter.current, 1).c_str() );
      display.print( "A/");
      display.println( _analogUi ); // show AD output to calculate calibration factor
    }
    else
    {  
      display.print("off/");
      display.println( _analogUi );
    }
    display.print( String( _multiMeter.voltage, 1).c_str() );
    display.print( "V/");
    display.println( _analogUchg ); // show AD output to calculate calibration factor

    display.display();
}

// -------------------------------------------------------------
bool ShowSafetyError()
{
    if( _safetyError != EN_NONE )
    {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);

      display.println("Safety err" );
      display.print(_safetyError ); // todo: show friendly text
      display.print(" Restart!" );

      display.display();
      return true;
    }
    return false;
}

// -------------------------------------------------------------
bool CheckSafetyError()
{
  // no check during grace period
  if( _tiGracePeriod != 0L && _tiGracePeriod < millis() )
    return false;
    
  _tiGracePeriod = 0L;

  if( _multiMeter.voltage > MAX_VOLTAGE  )
  {
    SwitchOff();
    _safetyError = EN_INT_HIVOLTAGE;
  }
  else if( _levoBatt.voltage > MAX_VOLTAGE )
  {
    SwitchOff();
    _safetyError = EN_BAT_HIVOLTAGE;
  }
  else if( _multiMeter.current > MAX_CURRENT  )
  {
    SwitchOff();
    _safetyError = EN_INT_HICURRENT;
  }
  else if( _levoBatt.current > MAX_CURRENT )
  {
    SwitchOff();
    _safetyError = EN_BAT_HICURRENT;
  }

  return ShowSafetyError();
}

// -------------------------------------------------------------
void dumpLevoBattData() // dump values from CAN
{
    if( _levoBatt.newValueMask & EN_MSG_401 )
      Serial.print("U "); Serial.print(_levoBatt.voltage); Serial.print(", I "); Serial.println(_levoBatt.current);
    if( _levoBatt.newValueMask & EN_MSG_402)
      Serial.print("% "); Serial.print(_levoBatt.chStatePercent); Serial.print(", Energy "); Serial.println(_levoBatt.currentEnergy);
    if( _levoBatt.newValueMask & EN_MSG_403 )
      Serial.print("health% "); Serial.print(_levoBatt.healthPercent); Serial.print(", max. cap. "); Serial.println( _levoBatt.maxCapacity );
    if( _levoBatt.newValueMask & EN_MSG_400 )
       Serial.print("Charging "); Serial.println(_levoBatt.bCharging);
}

// -------------------------------------------------------------
void canReceive(const CAN_message_t &msg)
{
  // voltage and current
  if( msg.id == 0x401 )
  {
    _levoBatt.voltage = (msg.buf[0] + (msg.buf[1]<<8))/1000.0;
    _levoBatt.current = (msg.buf[4] + (msg.buf[5]<<8) + (msg.buf[6]<<16) + (msg.buf[7]<<24) )/-1000.0; // neg. value is "charging"
    _levoBatt.newValueMask |= EN_MSG_401;
  }
  // charging state (% and energy)
  else if( msg.id == 0x402 )
  {
    _levoBatt.chStatePercent = msg.buf[0];
    _levoBatt.currentEnergy = (msg.buf[4] + (msg.buf[5]<<8) + (msg.buf[6]<<16) + (msg.buf[7]<<24) )/1000.0;
    _levoBatt.newValueMask |= EN_MSG_402;
  }
  // battery capacity and health ?
  else if( msg.id == 0x403 )
  {
    _levoBatt.healthPercent = msg.buf[0];
    _levoBatt.maxCapacity = (msg.buf[4] + (msg.buf[5]<<8) + (msg.buf[6]<<16) + (msg.buf[7]<<24) )/1000.0;
    _levoBatt.newValueMask |= EN_MSG_403;
  }
  // status 
  else if( msg.id == 0x400 )
  {
    _levoBatt.bCharging = (bool)(msg.buf[2] & 0x0C);
    _levoBatt.newValueMask |= EN_MSG_400;
  }

  if( _bDebugMode )
  {
    Serial.print("MB "); Serial.print(msg.mb);
    Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
    Serial.print(" BUS ");  Serial.print(msg.bus);   
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" EXT: "); Serial.print(msg.flags.extended);
    Serial.print(" TS: "); Serial.print(msg.timestamp);
    Serial.print(" ID: "); Serial.print(msg.id, HEX);
    Serial.print(" Buffer: ");
    for ( uint8_t i = 0; i < msg.len; i++ ) {
      Serial.print(msg.buf[i], HEX); Serial.print(" ");
    } Serial.println();
  }
}

// -------------------------------------------------------------

void setup() {

    Serial.begin(115200);

    // display init
    display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
    display.display();
    delay(2000);
    display.clearDisplay();
    display.display();

    // can init
    Can0.begin();
    Can0.setBaudRate(250000);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(canReceive);
    Can0.mailboxStatus();

    // set ADC
    pinMode(PIN_Ui, INPUT);
    pinMode(PIN_Uchg, INPUT);
    _adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
    _adc->adc0->setAveraging(32);
    _adc->adc0->setResolution(16);
    _adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);
    _adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);
    
    _adc->adc1->setReference(ADC_REFERENCE::REF_3V3);
    _adc->adc1->setAveraging(32);
    _adc->adc1->setResolution(16);
    _adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);
    _adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);
    
    // FET switch off
    pinMode(FET_SWITCH, OUTPUT);
    digitalWrite( FET_SWITCH, LOW );

    // switch LEVO off
    pinMode(LEVO_ON, OUTPUT);
    digitalWrite( LEVO_ON, LOW );

    // setup button
    _btn.attachClick(handleClick);
    _btn.attachLongPressStart(handleClick);
    _btn.attachDoubleClick(handleDoubleClick);
    _btn.setDebounceTicks(50);  // Period of time in which to ignore additional level changes.
    _btn.setPressTicks(1500);   // Duration to hold a button to trigger a long press.

    // show CPU freq
    Serial.print("F_CPU_ACTUAL=");
    Serial.println(F_CPU_ACTUAL);    
}

// -------------------------------------------------------------

void loop() {

  static unsigned long ti = millis() + 100;

  Can0.events();

  // read ADC all 100ms
  if( millis() > ti )
  {
    ti = millis() + 100;
    
    _analogUi   = _medianUi.in( _adc->adc0->analogRead(PIN_Ui) );
    _analogUchg = _medianUchg.in( _adc->adc1->analogRead(PIN_Uchg) );
    _multiMeter.current = max( MAKE_Ui(_analogUi), 0.0f );
    _multiMeter.voltage = max( MAKE_Uchg(_analogUchg), 0.0f );
    _multiMeter.energy += _multiMeter.current * _multiMeter.voltage; 

    if( _multiMeter.current > 0.5f ) // 0.5A --> threshold for time measuring
       _multiMeter.nMeasureCnt++;

    if( !CheckSafetyError() )
    {
      if( !_bDebugMode )
        ShowOutputLevo();
      else
        ShowOutputDebugMode();

      dumpLevoBattData();
      _levoBatt.newValueMask = EN_CLEAR; // all values are seen
    }
  }
  _btn.tick();
}
