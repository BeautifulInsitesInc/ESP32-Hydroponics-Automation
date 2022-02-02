#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <SimpleTimer.h>
#include <esp_adc_cal.h>
//#include <DFRobot_ESP_PH.h> // library to calcuate ph from voltage

//DFRobot_ESP_PH ph;

SimpleTimer timer;

// ----- LCD SETTINGS --------
int lcdColumns = 20; // LCD Columns
int lcdRows = 4; // LCD Rows

// ----- DEFAULT SETTINGS ------
bool temp_in_c = true; // Tempurature defaults to C
int pump_init_delay = .5; // Minutes - Initial time before starting the pump on startup
int pump_on_time = .5; // Minutes - how long the pump stays on for
int pump_off_time = 1; // Minutes -  how long the pump stays off for

// ----- SET PINS ------------------
const int temp_pin = 16; // Tempurature sensor pin
const int tds_pin = 34; // TDS sensor pin - try 13 if 26 doesnt work
const int ph_pin = 35; // pin the Ph sensor is connected to
const int pump_pin = 36; // set pump pin
const int heater_pin = 39;

// *************** CALIBRATION FUNCTION ******************
// To calibrate actual votage read at pin to the esp32 reading

uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

// =======================================================
// ======= TEMPURATURE SENSOR DS18B20 ====================
// =======================================================
#define TEMPERATURE_PRECISION 10
OneWire oneWire(temp_pin);// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature waterTempSensor(&oneWire); // Pass our oneWire reference to Dallas Temperature.

float tempC; // tempurature in Celsius
float tempF; // tempurature in Fahrenheit

void getWaterTemp()
  {
    waterTempSensor.requestTemperatures();    // send the command to get temperatures
    tempC = waterTempSensor.getTempCByIndex(0);  // read temperature in °C
    tempF = tempC * 9 / 5 + 32; // convert °C to °F
    //Serial.print("tempC : ");
    //Serial.print(tempC);
    //Serial.print("  tempF : ");
    //Serial.println(tempF);
    
    if (tempC == DEVICE_DISCONNECTED_C) // Something is wrong, so return an error
      {
        Serial.println("Houston, we have a problem");
        tempC = -100; 
        tempF = tempC;
      }
    //delay(1000);
  }
// =======================================================
// ======= PH SENSOR =====================================
// =======================================================
float ph_value; // actual pH value to display on screen
//-----------------------------------------
float calibration_adjustment_ph = 0; // adjust this to calibrate
//float voltage_adjustment = 0;
int voltage_input_ph = 3.3; // voltage can be 5 or 3.3
//-----------------------------------------
float adc_resolution = 4095;
float calibration_value_ph = 21.34 + calibration_adjustment_ph;
unsigned long int average_value_ph;
int buffer_array_ph[10],temp;

void getPH()
  {
    // Use the for loop we take samples, then arrange the values, and finally take the average.
    //Serial.print("Loop readings = ");
    for(int i=0;i<10;i++) // take 10 readings
      { 
        buffer_array_ph[i]=analogRead(ph_pin);
        //Serial.print(buffer_array_ph[i]); // print the voltage readout in the Serial Monitor
        //Serial.print(" / ");
        delay(30);
      }
    for(int i=0;i<9;i++)
      {
        for(int j=i+1;j<10;j++)
          {
            if(buffer_array_ph[i]>buffer_array_ph[j])
              {
                temp=buffer_array_ph[i];
                buffer_array_ph[i]=buffer_array_ph[j];
                buffer_array_ph[j]=temp;
              }
          }
      }
    average_value_ph=0;
    for(int i=2;i<8;i++)
      {
        average_value_ph += buffer_array_ph[i];
      }
    average_value_ph = average_value_ph / 6;

    //float voltage_ph = (float)average_value_ph * voltage_input_ph / adc_resolution / 6; 
    //voltage_ph = voltage_ph * (1+(voltage_adjustment/voltage_ph)); //  This line is to try to bring voltage to actual
    

    //float voltage_ph = analogRead(ph_pin) * voltage_input_ph / adc_resolution;
    
    float voltage_ph = average_value_ph * voltage_input_ph / adc_resolution;
    float calibrated_voltage = readADC_Cal(average_value_ph);
    calibrated_voltage = calibrated_voltage/1000;

    ph_value = voltage_input_ph * calibrated_voltage;
    //ph_value = (-5.70 * calibrated_voltage) + calibration_value_ph; // Calculate the actual pH
    //int temp_plug = 25;
    //float calibrated_ph_value = ph.readPH(calibrated_voltage, temp_plug);
/*
    Serial.print("analogRead(ph_pin : ");
    Serial.print(analogRead(ph_pin));
    Serial.print("    average_value_ph = ");
    Serial.print(average_value_ph);
    Serial.print("      voltage_ph = ");
    Serial.print(voltage_ph);
    Serial.print("     calibration_value_ph = ");
    Serial.print(calibration_value_ph);
    Serial.print("     ph_value = ");
    Serial.print(ph_value);
    Serial.print("     calibtraded_voltage = ");
    Serial.print(calibrated_voltage);
    Serial.print("     calibtraded_ph_value = ");
    Serial.println(calibrated_ph_value);

    delay(0); // pause between serial monitor output - can be set to zero after testing
 */
  }



// =======================================================
// ======= PPM OCEAN TDS METER SENSOR ====================
// =======================================================

#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0;
int tdsValue = 0;
float temperature = 25;

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

void getTDSReading()
  {
    // get current tempurature
    if (tempC == -100) temperature = 25;
    else temperature = tempC;
    static unsigned long analogSampleTimepoint = millis();
    if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
      {
        analogSampleTimepoint = millis();
        analogBuffer[analogBufferIndex] = analogRead(tds_pin);    //read the analog value and store into the buffer

        analogBufferIndex++;
        if(analogBufferIndex == SCOUNT) 
            analogBufferIndex = 0;
      }   
    static unsigned long printTimepoint = millis();
    if(millis()-printTimepoint > 800U)
      {
          printTimepoint = millis();
          for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
          analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
          float medianRead = getMedianNum(analogBuffer,SCOUNT);
          averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0; // ESP32 - 1-4096 - Ardurino mega 0-1023read the analog value more stable by the median filtering algorithm, and convert to voltage value
          
          //float calibrated_voltage = readADC_Cal(medianRead);
          //calibrated_voltage = calibrated_voltage/1000;
          
          float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
          float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
          tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
          
          Serial.print("   VREF : ");
          Serial.print(VREF);
          Serial.print("   analogRead(tds_pin) : ");
          Serial.print(analogRead(tds_pin));
          Serial.print("   Medianread : ");
          Serial.print(medianRead);
          Serial.print("   averageVoltage: ");
          Serial.print(averageVoltage,2);
          //Serial.print("   calibrated_voltage: ");
          //Serial.print(calibrated_voltage,2);
          Serial.print("   TtdsValue: ");
          Serial.print(tdsValue, 0);
          Serial.print("   Tempurature: ");
          Serial.print(temperature);
          Serial.print("   compensationVoltage: ");
          Serial.println(compensationVolatge);
        
      }
    delay(0);
  }
  

// =================================================
// ========== LCD DISPLAY ==========================
// =================================================
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  // set the LCD address to 0x27 for a 16 chars and 2 line display
void displaySplashscreen()// Display the splash screen
  {
    lcd.init(); // Initialize LCD
    lcd.backlight(); // Turn on the LCD backlight
    lcd.setCursor(1,0);
    lcd.print("CONCIERGE GROWERS");
    lcd.setCursor(5,1);
    lcd.print("Eat Good");
    lcd.setCursor(4,2);
    lcd.print("Feel Good");
    lcd.setCursor(4,3);
    lcd.print("Look Good");
    delay(1000);
    lcd.clear();
  }
void displayMainscreenstatic()// Display the parts that don't change
  {
    lcd.setCursor(0,0);
    lcd.print("Temp:");
    lcd.setCursor(1,1);
    lcd.print("TDS:");
    lcd.setCursor(2,2);
    lcd.print("PH:");
    lcd.setCursor(0,3);
    lcd.print("Pump:");
    lcd.setCursor(14,3);
    lcd.print("(Time)");
  }

void displayMainscreenData() // Display the data that changes on main screen
  {
    // ---- Display tempurature
    lcd.setCursor(5,0);
    //getWaterTemp(); // will return -100 if there is an issue -  not needed variable tempC is already set
    if (tempC == -100) lcd.print("(error)   ");
    else
      {
        if (temp_in_c == true)
          {
            lcd.print(tempC);
            lcd.print((char)223);
            lcd.print("C");
          }
        else
          {
            lcd.print(tempF);
            lcd.print((char)223);
            lcd.print("F");
          }
      }
    //----- Display TDS reading
    lcd.setCursor(5,1);
    if (tdsValue > 1000) lcd.print("(error)  ");
    else 
    {
      lcd.print(tdsValue);
      lcd.println("(PPM)    ");
    }
    // ---- Display pH Reading
    
    lcd.setCursor(5,2);
    lcd.print(ph_value); 
    //----Display  Pump status
    lcd.setCursor(5,3);
    if (digitalRead(pump_pin) == 0) lcd.print("ON     ");
    else lcd.print("OFF    ");
    
  }


// ==================================================
// ===========  PUMP CONTROL ========================
// ==================================================
int pump_init_delay_mills = pump_init_delay*1000;
int pump_on_time_mills = pump_off_time *1000; // pump turns on after the time off delay
int pump_off_time_mills = pump_off_time *1000; // pump turns off after teh time on delay


// ==================================================
// ===========  HEATER CONTROL ========================
// ==================================================

// ==================================================
// ===========  MAIN SETUP ==========================
// ==================================================
void setup(void)
  {
    Serial.begin(115200);// start serial port 115200
    Serial.println("Starting Hydroponics Automation Controler");

    // Initialize Sensors
    waterTempSensor.begin(); // initalize water temp sensor
    pinMode(tds_pin,INPUT); // setup TDS sensor pin
    //ph.begin(); // starts the ph api 

    // Set relay pins
    pinMode(pump_pin, OUTPUT);

    // Prepare screen
    displaySplashscreen();
    displayMainscreenstatic();
  }

// ====================================================
// ===========  MAIN LOOP =============================
// ====================================================

void loop(void)
{
  timer.run(); // Initiates SimpleTimer

  // Get readings
  getWaterTemp(); // sets tempC and tempF
  getTDSReading(); // sets tdsValue
  getPH();

  displayMainscreenData();
}

// ----------------- END MAIN LOOP ------------------------


 

