/**
 * @file        PKR_251_VACUUM_METER.ino
 * @brief       Program to decode the output of a PFEIFFER PKR251 vacuum gauge. 
 * @details     This program for Arduino Nano converts the voltage output of a PKR 251 pressure sensor to display it on a scientific format OLED 
                !! screen and send it to the serial port. 'Out of Range' measurements and sensor errors are taken into account by the program.
                !! In the event of measurements returning a sensor error, the Interlock pin goes to the logic high state. A pressure setpoint
                !! can be defined to activate the Interlock pin.
                !! The sensor output voltage is converted by an external ADS1115 analog to digital converter. The display is performed on a 128
                !! x 32 pixel OLED screen.
                !! This program uses the ADS1X15.h librarie from Copyright (c) 2013-2023 Rob Tillaart and the LED_I2C.h librarie from Copyright
                !! (c) 2010-2023 Rinky-Dink Electronics, Henning Karlsen.
 * @author      Frederic Salach <frederic.salach@gmail.com>
 * @version     1.2 dev
 * @copyright   PKR_251_Vacuum_Meter.ino © 2023 by Frédéric Salach is licensed under Attribution-NonCommercial-ShareAlike 4.0 International 
 */

// Libraries import
#include <ADS1X15.h>   // External ADC ADS1115 - Copyright (c) 2013-2023 Rob Tillaart - MIT licence
#include <OLED_I2C.h>  // External OLED display - Copyright (c) 2010-2023 Rinky-Dink Electronics, Henning Karlsen - Creative Commons (CC BY-NC-SA 3.0) licence
#include <EEPROM.h>

// Librarie config
ADS1115 ADS(0x48);  // Set ADS1115

OLED OLED_Display(SDA, SCL);  // Set OLED Display
extern uint8_t SmallFont[];   // OLED font import

// Variable declaration
uint16_t Current_Time = 0;   // Storage of the current time from millis()
uint16_t Previous_Time = 0;  // Storage of the previous time from millis()
int16_t ADC_Voltage;         // Storage of the ADC voltage measurement value
float Pressure;              // Storage of the calculated pressure value from the ADC Voltage
int8_t Error_State;          // Storage of the gauge error state
int8_t Range_State;          // Storage of the gauge range state
bool Pressure_State;         // Storage of the Pressure Interlock state
bool Interlock_State;        // Storage of the Interlock output state
/////////////////////
int State_SW;                   //
int State_CLK;                  //
int State_DT;                   //
int Previous_State_SW;          // Cette variable nous permettra de stocker le dernier état de la ligne SW lu, afin de le comparer à l'actuel
int Previous_State_CLK;         // Cette variable nous permettra de stocker le dernier état de la ligne CLK lu, afin de le comparer à l'actuel
uint16_t compteur;                   //
uint16_t Current_Time_Bp = 0;   // Storage of the current time from millis()
uint16_t Previous_Time_Bp = 0;  // Storage of the previous time from millis()



// Input/Output declatation
// Arduino Board
const uint8_t INTERLOCK_PIN = 3;  // Definition of the interlock output pin (3)
const uint8_t ON_BOARD_LED = 13;  // Definition of the interlock output pin (13)
const uint8_t Bp_SW = 5;          // La pin D2 de l'Arduino recevra la ligne SW du module KY-040
const uint8_t Bp_CLK = 6;         // La pin D3 de l'Arduino recevra la ligne CLK du module KY-040
const uint8_t Bp_DT = 7;          // La pin D4 de l'Arduino recevra la ligne DT du module KY-040
// ADC Board
const uint8_t GAUGE_SIGNAL_PIN = 0;  // Definition of the analog input pin for the gauge output (1)

// User configurable variables
const uint16_t ADC_RESOLUTION = 32767;         // Definition of the ADC resolution (For ADS1115, 32767)
const uint16_t ADC_RANGE = 4096;               // Definition of the ADC range [mV] (For ADS115 with a gain of 1, 4096)
const uint16_t ADC_GAIN = 1;                   // Definition of the ADC Gain (For ADS115 1 for input up 4096 mV)
const uint8_t INPUT_ATTENUATION = 3;           // Defination of the input attenuation (3)
const uint16_t SENSOR_ERROR_LOW = 500;         // Definition of the low limit output for gauge error detection [mV] (For PKR 251, 500)
const uint16_t SENSOR_ERROR_HIGH = 9500;       // Definition of the high limit output for gauge error detection [mV] (For PKR 251, 9500)
const uint16_t OUT_OFF_RANGE_LOW = 1820;       // Definition of the low limit output for gauge out of range detection [mV] (For PKR 251, 1820)
const uint16_t OUT_OFF_RANGE_HIGH = 8600;      // Definition of the low limit output for gauge error [mV] (For PKR 251, 500)
const float SENSOR_PRESSURE_CONSTANT = 1.667;  // Definition of the sensor constant for pressure conversion (For PKR 251, 1.667)
const float SENSOR_UNIT_FACTOR = 9.333;        // Definition of the sensor constant for pressure unit conversion (For PKR 251 and Pa unit, 9.333)
//const float INTERLOCK_PRESSURE_LOW = 80;       // Definition of the sensor constant for interlock low pressure value detection [Pa] (80)
uint16_t INTERLOCK_PRESSURE_LOW;
//const float INTERLOCK_PRESSURE_HIGH = 100.0;     // Definition of the sensor constant for interlock high pressure value detection [Pa] (100)
uint16_t INTERLOCK_PRESSURE_HIGH;
const char PRESSURE_UNIT[] = "Pa";               // Definition of the pressure unit displayed ("Pa")
const char INIT_MESSAGE[] = "PFEIFFER PKR 251";  // Definition of the initialization message ("PFEIFFER PKR 251")
const uint16_t SERIAL_SPEED = 9600;              // Definition of the serial connection speed [bps] (9600)
const uint16_t SCREEN_REFRESH_DELAY = 500;       // Definition of the refresh screen delay [ms] (500)
const uint16_t DEBOUNCE_DELAY = 150;             //

void setup() {
  /**
  * @fn          setup()
  * @brief       Function setup.
  */

  // Serial connection initialization
  Serial.begin(SERIAL_SPEED);
  Serial.println(INIT_MESSAGE);

  //ADS1115 ADC initialization
  ADS.begin();
  ADS.setGain(ADC_GAIN);
  ADS.setMode(1);
  ADS.setDataRate(1);
  ADS.readADC(0);

  // OLED screen initialization
  if (!OLED_Display.begin(SSD1306_128X32))
    while (1)
      ;

  OLED_Display.clrScr();
  OLED_Display.setFont(SmallFont);
  OLED_Display.print(String(INIT_MESSAGE), CENTER, 15);
  OLED_Display.update();

  // Pin initialization
  // Output
  pinMode(INTERLOCK_PIN, OUTPUT);
  pinMode(ON_BOARD_LED, OUTPUT);
  digitalWrite(INTERLOCK_PIN, HIGH);
  digitalWrite(ON_BOARD_LED, HIGH);
  // Input
  pinMode(Bp_SW, INPUT);
  pinMode(Bp_DT, INPUT);
  pinMode(Bp_CLK, INPUT);

  Previous_State_SW = digitalRead(Bp_SW);
  Previous_State_CLK = digitalRead(Bp_CLK);

  // Variables in EEPROM
  EEPROM.get(112, INTERLOCK_PRESSURE_HIGH);
  EEPROM.get(128, INTERLOCK_PRESSURE_LOW);

  Serial.println(INTERLOCK_PRESSURE_HIGH);
  Serial.println(INTERLOCK_PRESSURE_LOW);



  // Initialization delay
  delay(2500);
}

void loop() {
  Pressure_Main();

  Read_Rotaty_Encoder();

  Current_Time_Bp = millis();
  if ((Current_Time_Bp - Previous_Time_Bp) > DEBOUNCE_DELAY) {

    if (State_SW != Previous_State_SW) {
      Previous_State_SW = State_SW;
      Menu();
    }
  }
}

void Menu() {

  bool Menu_Boucle_State;
  int Read_EEPROM;

  OLED_Display.clrScr();
  OLED_Display.setFont(SmallFont);
  OLED_Display.print("MENU", CENTER, 15);
  OLED_Display.update();

  delay(1000);

///// LEVEL 1 MENU
  EEPROM.get(112, compteur);

  Menu_Boucle_State = true;
  Read_Rotaty_Encoder();
  Previous_State_SW = State_SW;

  while (Menu_Boucle_State == true) {
    Read_Rotaty_Encoder();

    Current_Time_Bp = millis();
    if ((Current_Time_Bp - Previous_Time_Bp) > DEBOUNCE_DELAY) {
      if (State_SW != Previous_State_SW) {
        Previous_State_SW = State_SW;
        Menu_Boucle_State = false;
      }
    }

    // Lecture CLK et DT
    if (State_CLK != Previous_State_CLK) {
      Previous_State_CLK = State_CLK;
      if (State_CLK == LOW) {
        if (State_CLK != State_DT) {
          compteur++;
        } else {
          compteur--;
        }
        delay(10);
      }
    }


    OLED_Display.clrScr();
    OLED_Display.setFont(SmallFont);
    OLED_Display.print("SET HIGH", RIGHT, 5);
    OLED_Display.print(String(compteur), RIGHT, 15);
    OLED_Display.update();
  }

  OLED_Display.clrScr();
  OLED_Display.setFont(SmallFont);
  OLED_Display.print("HIGH SETING", RIGHT, 5);
  OLED_Display.print(String(compteur), RIGHT, 15);
  OLED_Display.update();

  delay(1000);

  EEPROM.put(112, compteur);

  EEPROM.get(112, Read_EEPROM);

  OLED_Display.clrScr();
  OLED_Display.setFont(SmallFont);
  OLED_Display.print("VERIFY", RIGHT, 5);
  OLED_Display.print(String(Read_EEPROM), RIGHT, 15);
  OLED_Display.update();

  delay(1000);

///// LEVEL 2 MENU
  EEPROM.get(128, compteur);

  Menu_Boucle_State = true;
  Read_Rotaty_Encoder();
  Previous_State_SW = State_SW;

  while (Menu_Boucle_State == true) {
    Read_Rotaty_Encoder();

    Current_Time_Bp = millis();
    if ((Current_Time_Bp - Previous_Time_Bp) > DEBOUNCE_DELAY) {
      if (State_SW != Previous_State_SW) {
        Previous_State_SW = State_SW;
        Menu_Boucle_State = false;
      }
    }

    // Lecture CLK et DT
    if (State_CLK != Previous_State_CLK) {
      Previous_State_CLK = State_CLK;
      if (State_CLK == LOW) {
        if (State_CLK != State_DT) {
          compteur++;
        } else {
          compteur--;
        }
        delay(10);
      }
    }


    OLED_Display.clrScr();
    OLED_Display.setFont(SmallFont);
    OLED_Display.print("SET LOW", RIGHT, 5);
    OLED_Display.print(String(compteur), RIGHT, 15);
    OLED_Display.update();
  }

  OLED_Display.clrScr();
  OLED_Display.setFont(SmallFont);
  OLED_Display.print("LOW SETING", RIGHT, 5);
  OLED_Display.print(String(compteur), RIGHT, 15);
  OLED_Display.update();

  delay(1000);

  EEPROM.put(128, compteur);

  EEPROM.get(128, Read_EEPROM);

  OLED_Display.clrScr();
  OLED_Display.setFont(SmallFont);
  OLED_Display.print("VERIFY", RIGHT, 5);
  OLED_Display.print(String(Read_EEPROM), RIGHT, 15);
  OLED_Display.update();

  delay(1000);

///// RETURN MENU

  Read_Rotaty_Encoder();
  Previous_State_SW = State_SW;
}


void Read_Rotaty_Encoder() {

  State_SW = digitalRead(Bp_SW);
  State_CLK = digitalRead(Bp_CLK);
  State_DT = digitalRead(Bp_DT);
}
























//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// Function for Pressure Processing ///////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Pressure_Main() {
  /**
  * @fn          Pressure_Main()
  * @brief       Main function for pressure measurment and processing.
  */

  // Read ADC_Voltage
  ADC_Voltage = ADC_Read(GAUGE_SIGNAL_PIN, ADC_RESOLUTION, ADC_RANGE, INPUT_ATTENUATION);

  // Convert ADC_Voltage to Pressure
  Pressure = Pressure_Convertion(ADC_Voltage, SENSOR_PRESSURE_CONSTANT, SENSOR_UNIT_FACTOR);

  // Send mesurement data to serial port
  Send_Serial();

  // Calculate measurement state
  Error_State = Threshold_Comparison(ADC_Voltage, SENSOR_ERROR_LOW, SENSOR_ERROR_HIGH);
  Range_State = Threshold_Comparison(ADC_Voltage, OUT_OFF_RANGE_LOW, OUT_OFF_RANGE_HIGH);

  // Calculate pressure state with hysteresis
  Pressure_State = Hyst_Threshold_Comparison(Pressure, INTERLOCK_PRESSURE_LOW, INTERLOCK_PRESSURE_HIGH, Pressure_State);

  // Interlock Test
  if (Error_State != 0 || Pressure_State != 0) {
    digitalWrite(INTERLOCK_PIN, HIGH);
    Interlock_State = 1;
  } else {
    digitalWrite(INTERLOCK_PIN, LOW);
    Interlock_State = 0;
  }

  // Display Pressure and State on screen
  Current_Time = millis();
  if ((Current_Time - Previous_Time) > SCREEN_REFRESH_DELAY) {
    digitalWrite(ON_BOARD_LED, HIGH);

    Previous_Time = Current_Time;
    OLED_Display.clrScr();
    OLED_Display.setFont(SmallFont);
    Display_Result(Pressure, 2, 5);
    Display_State(25);
    OLED_Display.update();

    digitalWrite(ON_BOARD_LED, LOW);
  }

  // Send end to serial port
  Serial.println("End");
}


bool Hyst_Threshold_Comparison(float FI_Value, float FI_Threshold_Low, float FI_Threshold_High, bool Previous_State) {
  /**
  * @fn          Hyst_Threshold_Comparison(FI_Value, FI_Threshold_Low, FI_Threshold_High, Previous_State)
  * @brief       Function to compare a value to a threshold with hysteresis.
  * @param       float FI_Value, Value compared to the thresholds.
  * @param       float FI_Threshold_Low, Low threshold.
  * @param       float FI_Threshold_High, High threshold.
  * @param       bool Previous_State, Memorizing previous output state.
  * @return      Returns false if the value is less than the low threshold, and true if the value is greater than the high threshold.
  */

  bool FO_State = Previous_State;

  if (FI_Value < FI_Threshold_Low) {
    FO_State = false;
  }
  if (FI_Value > FI_Threshold_High) {
    FO_State = true;
  }

  return (FO_State);
}


void Display_State(uint8_t FI_Display_Line) {
  /**
  * @fn          Display_State(FI_Display_Line)
  * @brief       Function to send the measurement state string in the OLED screen buffer.
  * @param       uint8_t FI_Display_Line, Line to display the measurement state string.
  */

  char FV_Displayed_Data[25];

  // Displayed_Data recovery
  char *Displayed_Data = Format_State(Error_State, Range_State, Pressure_State, Interlock_State);
  strcpy(FV_Displayed_Data, Displayed_Data);
  free(Displayed_Data);

  strcat(FV_Displayed_Data, " ");

  // Send measurement state string to the OLED screen buffer
  OLED_Display.print(String(FV_Displayed_Data), RIGHT, FI_Display_Line);
}


char *Format_State(int8_t FI_Error_State, int8_t FI_Range_State, bool FI_Pressure_State, bool FI_Interlock_State) {
  /**
  * @fn          Format_State(FI_Error_State, FI_Range_State, FI_Pressure_State, FI_Interlock_State);
  * @brief       Function to format the state string.
  * @param       int8_t FI_Error_State, 
  * @param       int8_t FI_Range_State, 
  * @param       bool FI_Pressure_State, 
  * @param       bool FI_Interlock_State, 
  * @return      Return a string.
                 !! [ ]     Sensor Error Flag :     [E]   Error
                 !! [ ]     Sensor Range Flag :     [+]   Out of Range High,  [-]    Out of Range Low
                 !! [ ]     Pressure Limit Flag :   [P]   Pressure Interlock 
                 !! [ ]     Interlock State Flag :  [*]   Interlock activ
  */
  char *FO_Result = (char *)malloc(25);  //   Memory allocation for the output result

  // Sensor Error Detection
  if (FI_Error_State == -1 || FI_Error_State == +1) {
    strcat(FO_Result, "[E]");
  } else {
    strcat(FO_Result, "[ ]");
  }

  // Sensor Out of Range Detection
  if (FI_Range_State == -1) {
    strcat(FO_Result, "[-]");
  } else if (FI_Range_State == 1) {
    strcat(FO_Result, "[+]");
  } else {
    strcat(FO_Result, "[ ]");
  }

  // Pressure interlock Detection
  if (FI_Pressure_State == true) {
    strcat(FO_Result, "[P]");
  } else {
    strcat(FO_Result, "[ ]");
  }

  // Interlock Detection
  if (FI_Interlock_State == true) {
    strcat(FO_Result, "[*]");
  } else {
    strcat(FO_Result, "[ ]");
  }

  return FO_Result;
}


void Display_Result(float FI_Value, uint8_t FI_Dot_Place, uint8_t FI_Display_Line) {
  /**
  * @fn          Display_Result(FI_Value, FI_Dot_Place, FI_Display_Line);
  * @brief       Function to send the measurement string in the OLED screen buffer.
  * @param       float FI_Value, 
  * @param       int FI_Dot_Place, 
  */

  char FV_Displayed_Data[25];

  // Displayed_Data recovery
  char *Displayed_Data = Format_Result(FI_Value, FI_Dot_Place);
  strcpy(FV_Displayed_Data, Displayed_Data);
  free(Displayed_Data);

  strcat(FV_Displayed_Data, " ");
  strcat(FV_Displayed_Data, PRESSURE_UNIT);
  strcat(FV_Displayed_Data, " ");


  //Serial.print(FV_Displayed_Data);

  // Display output
  OLED_Display.print(String(FV_Displayed_Data), RIGHT, FI_Display_Line);
}


char *Format_Result(float FI_Value, int FI_Dot_Place) {
  /**
  * @fn          Format_Result(FI_Value, FI_Dot_Place);
  * @brief       Function to format the FI_Value in scientific format.
  * @param       float FI_Value, 
  * @param       int FI_Dot_Place, 
  * @return      Return a string.
                 !! X.XXXX E+XX
  */

  int8_t FV_Calc_Exp;
  float FV_Calc_Base;
  char FV_Base[25];
  char FV_Exp[25];
  char *FO_Result = (char *)malloc(25);  //   Memory allocation for Output_Result

  // Base and Exponent Extraction
  FV_Calc_Exp = int(log10(FI_Value));
  FV_Calc_Base = FI_Value / pow(10, FV_Calc_Exp);
  if (FV_Calc_Base < 1) {
    FV_Calc_Base = FV_Calc_Base * 10;
    FV_Calc_Exp = FV_Calc_Exp - 1;
  }

  // Formatting Output
  dtostrf(FV_Calc_Base, 1, FI_Dot_Place, FV_Base);
  dtostrf(FV_Calc_Exp, 1, 0, FV_Exp);
  strcat(FO_Result, FV_Base);
  if (FV_Calc_Exp >= 0) {
    strcat(FO_Result, " E+");
  } else {
    strcat(FO_Result, " E");
  }
  strcat(FO_Result, FV_Exp);

  return FO_Result;
}


int8_t Threshold_Comparison(float FI_Measured_Value, float FI_Low_Limit, float FI_High_Limit) {
  /**
  * @fn          Threshold_Comparison(FI_Measured_Value, FI_Low_Limit, FI_High_Limit);
  * @brief       Function to compare FI_Measured_Value to FI_Low_Limit and FI_High_Limit as thresholds.
  * @param       float FI_Measured_Value, 
  * @param       float FI_Low_Limit, 
  * @param       float FI_High_Limit, 
  * @return      Return -1 if FI_Measured_Value <= FI_Low_Limit, +1 if FI_Measured_Value >= FI_High_Limit, or else return 0.
  */

  // Variables
  int8_t FO_Code;

  // Check to the limits
  if (FI_Measured_Value <= FI_Low_Limit) {
    FO_Code = -1;
  } else if (FI_Measured_Value >= FI_High_Limit) {
    FO_Code = +1;
  } else {
    FO_Code = 0;
  }

  // Return ADC_Output_Voltage
  return (FO_Code);
}


void Send_Serial() {
  /**
  * @fn          Send_Serial();
  * @brief       Function to send data to serial port.
  */

  Serial.print(ADC_Voltage);
  Serial.print(";");
  Serial.print(Pressure, 8);
  Serial.print(";");
}


float Pressure_Convertion(int16_t FI_Voltage, float FI_Sensor_Constant, float FI_Unit_Constant) {
  /**
  * @fn          Pressure_Convertion(FI_Voltage, FI_Sensor_Constant, FI_Unit_Constant);
  * @brief       Function to convert FI_Voltage in Pressure with FI_Sensor_Constant and FI_Unit_Constant as sensor parameters.
  * @param       int16_t FI_Voltage, 
  * @param       float FI_Sensor_Constant, 
  * @param       float FI_Unit_Constant, 
  * @return      Return the Pressure calculated from : Pressure = pow(10, (FI_Sensor_Constant * FI_Voltage / 1000) - FI_Unit_Constant)
  */

  // Variables
  float FO_Pressure;

  // Convert Voltage to Pressure
  FO_Pressure = pow(10, (FI_Sensor_Constant * FI_Voltage / 1000) - FI_Unit_Constant);

  // Return ADC_Output_Voltage
  return (FO_Pressure);
}


int16_t ADC_Read(uint8_t FI_ADC_Imput_Number, uint16_t FI_ADC_Resolution, uint16_t FI_ADC_Range, uint8_t FI_ADC_Attenuation) {
  /**
  * @fn          ADC_Read(FI_ADC_Imput_Number, FI_ADC_Resolution, FI_ADC_Range, FI_ADC_Attenuation);
  * @brief       Function to read the ADC input FI_ADC_Imput_Number with FI_ADC_Resolution, FI_ADC_Range and FI_ADC_Attenuation parameters.
  * @param       uint8_t FI_ADC_Imput_Number, 
  * @param       uint16_t FI_ADC_Resolution, 
  * @param       uint16_t FI_ADC_Range, 
  * @param       uint8_t FI_ADC_Attenuation, 
  * @return      Return the ADC voltage.
  */

  // Variables
  int16_t FV_ADC_Raw_Value;
  int16_t FO_ADC_Output_Voltage;

  // Read ADC_Raw_Value
  FV_ADC_Raw_Value = ADS.readADC(FI_ADC_Imput_Number);

  // Convert ADC_Raw_Value in  ADC_Output_Voltage
  FO_ADC_Output_Voltage = map(FV_ADC_Raw_Value, 0, FI_ADC_Resolution, 0, FI_ADC_Range) * FI_ADC_Attenuation;

  // Return ADC_Output_Voltage
  return (FO_ADC_Output_Voltage);
}
