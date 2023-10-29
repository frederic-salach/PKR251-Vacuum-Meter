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
                // (c) 2010-2023 Rinky-Dink Electronics, Henning Karlsen.
 * @author      Frederic Salach <frederic.salach@gmail.com>
 * @version     1.0
 * @copyright   PKR_251_Vacuum_Meter.ino © 2023 by Frédéric Salach is licensed under Attribution-NonCommercial-ShareAlike 4.0 International 
 */

// Libraries import
#include "ADS1X15.h"   // External ADC ADS1115 - Copyright (c) 2013-2023 Rob Tillaart - MIT licence
#include <OLED_I2C.h>  // External OLED display - Copyright (c) 2010-2023 Rinky-Dink Electronics, Henning Karlsen - Creative Commons (CC BY-NC-SA 3.0) licence

// Librarie config
ADS1115 ADS(0x48);  // Set ADS1115

OLED OLED_Display(SDA, SCL);  // Set OLED Display
extern uint8_t SmallFont[];   // OLED font import

// Variable declaration
uint16_t Current_Time = 0;          // Storage of the current time from millis()
uint16_t Previous_Time = 0;         // Storage of the previous time from millis()
const uint8_t INTERLOCK_PIN = 5;    // Definition of the interlock output pin
int16_t ADC_Voltage;                // Storage of the ADC voltage measurement value
float Pressure;                     // Storage of the calculated pressure value from the ADC Voltage
int8_t Error_Limit_Comparison;      // Storage of the calculated gauge error value
int8_t Range_Limit_Comparison;      // Storage of the calculated gauge error value
int8_t Interlock_Limit_Comparison;  // Storage of the calculated gauge error value

// User configurable variables
const uint8_t GAUGE_SIGNAL_PIN = 1;              // Definition of the analog input pin for the gauge output (1)
const uint16_t ADC_RESOLUTION = 32767;           // Definition of the ADC resolution (For ADS1115, 32767)
const uint16_t ADC_RANGE = 4096;                 // Definition of the ADC range [mV] (For ADS115 with a gain of 1, 4096)
const uint8_t INPUT_ATTENUATION = 3;             // Defination of the input attenuation (3)
const uint16_t SENSOR_ERROR_LOW = 500;           // Definition of the low limit output for gauge error detection [mV] (For PKR 251, 500)
const uint16_t SENSOR_ERROR_HIGH = 9500;         // Definition of the high limit output for gauge error detection [mV] (For PKR 251, 9500)
const uint16_t OUT_OFF_RANGE_LOW = 1820;         // Definition of the low limit output for gauge out of range detection [mV] (For PKR 251, 1820)
const uint16_t OUT_OFF_RANGE_HIGH = 8600;        // Definition of the low limit output for gauge error [mV] (For PKR 251, 500)
const float SENSOR_PRESSURE_CONSTANT = 1.667;    // Definition of the sensor constant for pressure conversion (For PKR 251, 1.667)
const float SENSOR_UNIT_FACTOR = 9.333;          // Definition of the sensor constant for pressure unit conversion (For PKR 251 and Pa unit, 9.333)
const float INTERLOCK_PRESSURE = 10.0;           // Definition of the sensor constant for interlock pressure value detection [Pa] (10)
const char PRESSURE_UNIT[] = "Pa";               // Definition of the pressure unit displayed ("Pa")
const char INIT_MESSAGE[] = "PFEIFFER PKR 251";  // Definition of the initialization message ("PFEIFFER PKR 251")
const uint16_t SERIAL_SPEED = 9600;              // Definition of the serial connection speed [bps] (9600)
const uint16_t SCREEN_REFRESH_DELAY = 500;       // Definition of the refresh screen delay [ms] (500)

void setup() {
  //Setup function

  // Serial connection initialization
  Serial.begin(SERIAL_SPEED);
  Serial.println(INIT_MESSAGE);

  //ADS1115 ADC initialization
  ADS.begin();
  ADS.setGain(1);
  ADS.setMode(1);
  ADS.setDataRate(3);
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
  pinMode(INTERLOCK_PIN, OUTPUT);
  digitalWrite(INTERLOCK_PIN, HIGH);

  // Initialization delay
  delay(2500);
}

void loop() {
  // Loop function

  // Read ADC_Voltage
  ADC_Voltage = ADC_Read(GAUGE_SIGNAL_PIN, ADC_RESOLUTION, ADC_RANGE, INPUT_ATTENUATION);

  // Convert ADC_Voltage to Pressure
  Pressure = Pressure_Convertion(ADC_Voltage, SENSOR_PRESSURE_CONSTANT, SENSOR_UNIT_FACTOR);

  // Send mesurement data to serial port
  Send_Serial();

  // Calculate measurement State
  Error_Limit_Comparison = Limit_Comparison(ADC_Voltage, SENSOR_ERROR_LOW, SENSOR_ERROR_HIGH);
  Range_Limit_Comparison = Limit_Comparison(ADC_Voltage, OUT_OFF_RANGE_LOW, OUT_OFF_RANGE_HIGH);
  Interlock_Limit_Comparison = Limit_Comparison(Pressure, 0, INTERLOCK_PRESSURE);

  // Interlock Test
  if (Error_Limit_Comparison != 0 || Interlock_Limit_Comparison != 0) {
    digitalWrite(INTERLOCK_PIN, HIGH);
  } else {
    digitalWrite(INTERLOCK_PIN, LOW);
  }

  Current_Time = millis();
  if ((Current_Time - Previous_Time) > SCREEN_REFRESH_DELAY) {
    Previous_Time = Current_Time;
    // Display
    OLED_Display.clrScr();
    OLED_Display.setFont(SmallFont);
    Display_Result(Pressure, 2, 5);
    Display_State(25);
    OLED_Display.update();
  }
  Serial.println("End");
}


void Display_State(uint8_t FI_Display_Line) {
  // Display_State function
  // This function displays on the OLED screen the line corresponding to the measurement status
  // This function work with the Format_State function
  char FV_Displayed_Data[25];

  // Displayed_Data recovery
  char *Displayed_Data = Format_State(Error_Limit_Comparison, Range_Limit_Comparison, Interlock_Limit_Comparison);
  strcpy(FV_Displayed_Data, Displayed_Data);
  free(Displayed_Data);

  strcat(FV_Displayed_Data, " ");

  //Serial.print(FV_Displayed_Data);

  // Display output
  OLED_Display.print(String(FV_Displayed_Data), RIGHT, FI_Display_Line);
}


char *Format_State(int8_t FI_Error_State, int8_t FI_Range_State, int8_t FI_Interlock_State) {
  // Format_State function
  // This function formats the measurement status before display
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
  if (FI_Interlock_State == -1 || FI_Interlock_State == +1) {
    strcat(FO_Result, "[I]");
  } else {
    strcat(FO_Result, "[ ]");
  }

  return FO_Result;
}


void Display_Result(float FI_Value, uint8_t FI_Dot_Place, uint8_t FI_Display_Line) {
  // Display_Result function
  // This function displays on the OLED screen the line corresponding to the measured pressure the in scientific format
  // This function work with the Format_Result function
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
  // Format_Result function
  // This function formats the measured pressure scientific format before display
  // Format result
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


int8_t Limit_Comparison(float FI_Measured_Value, float FI_Low_Limit, float FI_High_Limit) {
  // Limit_Comparison function
  // This function compares data to low and high limits
  // Imput:
  //  float FI_Measured_Value
  //  float FI_Low_Limit
  //  float FI_High_Limit
  // Return:
  //  uint8_t FO_Code (-1 = Under Range, 0 = In Range, +1 = Over Range)

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
  // Send_Serial function
  // This function send measurement data (ADC_Voltage and pressure) to serial port

  Serial.print(ADC_Voltage);
  Serial.print(";");
  Serial.print(Pressure, 8);
  Serial.print(";");
}


float Pressure_Convertion(int16_t FI_Voltage, float FI_Sensor_Constant, float FI_Unit_Constant) {
  // Pressure_Convertion function
  // This function convert mesured Voltage to Pressure
  // Input:
  //  int16_t FI_Voltage
  //  float FI_Sensor_Constant
  //  float FI_Unit_Constant
  // Return:
  //  float FO_Pressure

  // Variables
  float FO_Pressure;

  // Convert Voltage to Pressure
  FO_Pressure = pow(10, (FI_Sensor_Constant * FI_Voltage / 1000) - FI_Unit_Constant);

  // Return ADC_Output_Voltage
  return (FO_Pressure);
}


int16_t ADC_Read(uint8_t FI_ADC_Imput_Number, uint16_t FI_ADC_Resolution, uint16_t FI_ADC_Range, uint8_t FI_ADC_Attenuation) {
  // ADC_Read function
  // This function read the ADC Input and return the voltage
  // Input:
  //  uint8_t FI_ADC_Imput_Number
  //  uint16_t FI_ADC_Resolution
  //  uint16_t FI_ADC_Range
  //  uint8_t FI_ADC_Attenuation
  // Return:
  //  int16_t FO_ADC_Output_Voltage

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
