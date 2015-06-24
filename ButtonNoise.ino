//////////////////////////////////////////////////////////////////
// Copyright ©2014 Graeme Jury ZL2APV
// Released under the lgpl License - Please alter and share.
// Controller for the EB104.ru Auto Antenna Tuner
// Coarse stepping through L & C for best SWR
/////////////////////////////////////////////////////////////////

/*
This programme is to test the noise of a button when pressed or released. The analog input reads as fast
as it can when a button is pressed and prints the value of each read together with the time in uSec from
first change detected. This will continue for 50 mSec and is effectively a poor man's storage scope.
*/

#define LEDpin        13   // A LED is connected to this pin, use for heartbeat

#define DEBUG_BUTTON_ARRAY
//#define DEBUG_BUTTON_INFO
#define DEBUG_BUTTONS
//#define DEBUG_STEP_BUTTON

// Analog pushbutton settings
#define analog_buttons_pin A6
#define num_of_analog_buttons 2
#define analog_buttons_r1 20  // Resistor value connected to button chain in "K's"
#define analog_buttons_r2 1.2 // Value of each resistor in the chain in "K's"
#define LONG_PRESS_TIME 800 //msec before button considered a long press
#define analog_Button_Debounce_Millis 10

// Global variables always start with an underscore
int _Button_array_max_value[num_of_analog_buttons];
int _Button_array_min_value[num_of_analog_buttons];

/**********************************************************************************************************/

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  digitalWrite(analog_buttons_pin, HIGH);  // set pullup (Value 20K ohm)
  initialize_analog_button_array();
}

/**********************************************************************************************************/

void loop(){
  byte buttonNumber;

  buttonNumber = getAnalogButton();
  if(buttonNumber != 0) { // 0x00 is returned with no button press
    if(buttonNumber <= num_of_analog_buttons) {
      // A short press trailing edge detected
      processShortPressTE(buttonNumber);

    }
    else if(buttonNumber <= (num_of_analog_buttons + num_of_analog_buttons)) {
      // A long press leading edge detected
      buttonNumber = buttonNumber - num_of_analog_buttons;
      processLongPressLE(buttonNumber);
#ifdef DEBUG_BUTTON_INFO
      Serial.print(F("Loop:  A long press leading edge detected on button "));
      Serial.println(buttonNumber);
#endif
    }
    else {
      // A long press trailing edge detected
      buttonNumber = buttonNumber - (num_of_analog_buttons + num_of_analog_buttons);
      processLongPressTE(buttonNumber);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutines start here
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initialize_analog_button_array()
{
  /*
   typical button values:

   0: -56 - 46
   1: 47 - 131
   2: 132 - 203
   3: 203 - 264
   */
  int button_value;
  int lower_button_value;
  int higher_button_value;

  for (int x = 0;x < num_of_analog_buttons;x++) {
    button_value = int(1023 * (float(x * analog_buttons_r2)/float((x * analog_buttons_r2) + analog_buttons_r1)));
    lower_button_value = int(1023 * (float((x-1) * analog_buttons_r2)/float(((x-1) * analog_buttons_r2) + analog_buttons_r1)));
    higher_button_value = int(1023 * (float((x+1) * analog_buttons_r2)/float(((x+1) * analog_buttons_r2) + analog_buttons_r1)));

    _Button_array_min_value[x] = (button_value - ((button_value - lower_button_value)/2));
    _Button_array_max_value[x] = (button_value + ((higher_button_value - button_value)/2));

#ifdef DEBUG_BUTTON_ARRAY
    Serial.print(F("initialize_analog_button_array: "));
    Serial.print(x);
    Serial.print(F(":  "));
    Serial.print(_Button_array_min_value[x]);
    Serial.print(F(" - "));
    Serial.println(_Button_array_max_value[x]);
#endif //DEBUG_BUTTON_ARRAY/*
  }
}

/**********************************************************************************************************/
byte getAnalogButton()
{
  // Buttons are checked for either a short or a long press. The first time the poll detects a button press it saves
  // the button info and waits 10 msec to re-sample the button. A short press is determined by a release being
  // detected within LONG_PRESS_TIME. A Long press by checking button is held for more than LONG_PRESS_TIME.
  // Returns: 0 if no button is pressed
  //          0 for short pressed button leading edge or 'short press time' button hold.
  //          button number if short press trailing edge
  //          button Number plus Number of buttons if Long press leading edge
  //          button Number plus (Number of buttons * 2) if Long press trailing edge

  static unsigned long lastButtonTime = 0;
  static unsigned long longPressTimer = 0;
  static boolean longPress = false;
  static byte lastButtonValue = 0;
  static byte currentButton = 0;
  int analogButtonValue = 0;
  int analog_read_temp = 0;
  byte thisButton;
  byte cnt;
  byte retVal;

  //sample the analog input only every 'analog_Button_Debounce_Millis' intervals
  if((millis() - lastButtonTime) < analog_Button_Debounce_Millis){
    return 0;
  }

  // OK we are over 'analog_Button_Debounce_Millis' since the last button read so process button.
  lastButtonTime = millis(); // Set timer for next sample period
  //See if a button was pressed
  //  if (analogRead(analog_buttons_pin) <= button_array_high_limit[num_of_analog_buttons-1]) {

  // 32 reads of button effectively averages it
  for (cnt = 0; cnt < 32; cnt++){
    analogButtonValue = analogButtonValue + analogRead(analog_buttons_pin);
  }
  analogButtonValue = analogButtonValue / cnt;
#ifdef DEBUG_BUTTONS
  if(analogButtonValue < 1020) {
    Serial.print(F("The raw button press value is "));
    Serial.println(analogButtonValue);
  }
#endif
  // Now determine which button was pressed if any, else assign button value of 0
  if (analogButtonValue <= _Button_array_max_value[num_of_analog_buttons-1]) {
    for (cnt = 0; cnt < num_of_analog_buttons; cnt++) {
      if  ((analogButtonValue > _Button_array_min_value[cnt]) &&
        (analogButtonValue <=  _Button_array_max_value[cnt])) {
        thisButton = cnt + 1;
      }
    }
  }
  else thisButton = 0; // End of "Now determine which button was pressed if any ..."

  // See if we got 2 identical samples in a row
  if(thisButton != lastButtonValue) {
    lastButtonValue = thisButton; // No but setting up now for next sample match.
  }
  else { // We have a valid button press or a valid button release
    if(thisButton != 0) { // It is a press so save the button and check for a long press
      if(currentButton != thisButton) {
        currentButton = thisButton;
        longPressTimer = millis();
      }
      if((millis() - longPressTimer) > LONG_PRESS_TIME) {
        retVal = currentButton + num_of_analog_buttons;
        longPress = true;
      }
      else retVal = 0;
    }
    else { // We are releasing the button so check if it is from a short or long press
      if(longPress) {
        //        Serial.println(F("At ... if((millis() - longPressTimer) > LONG_PRESS_TIME)"));
        retVal = currentButton + num_of_analog_buttons + num_of_analog_buttons;
        currentButton = 0;
        longPress = false;
      }
      else {
        retVal = currentButton;
        currentButton = 0;
        longPressTimer = 0;
      }
    }
  } // End of "See if we got 2 identical samples in a row"

  return retVal;
}

/**********************************************************************************************************/

void processShortPressTE(byte button)
{

}

/**********************************************************************************************************/
void processLongPressLE(byte button)
{

}

/**********************************************************************************************************/
void processLongPressTE(byte button)
{

}

/**********************************************************************************************************/

