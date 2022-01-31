#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
//#include <SimpleTimer.h>
//SimpleTimer timer;

// Settings
int lcdColumns = 20; // LCD Columns
int lcdRows = 4; // LCD Rows

// Defaults
bool temp_in_c = true; // Tempurature defaults to C
int pump_init_delay = 10; // Initial time before starting the pump on startup
int pump_on_time = 10; // how long the pump stays on for
int pump_off_time = 20; // how long the pump stays off for

// ----- SET PINS ------------------
const int temp_pin = 12; // Tempurature sensor pin
const int tds_pin = 35; // TDS sensor pin
/*
const int ph_in = 25; // pin the Ph sensor is connected to
const int pump_out = 28; // set pump pin

// ---- DEFAULT SETTINGS ----------


// =======================================================
// ======= PH SENSOR =====================================
// =======================================================
float adc_resolution = 1024.0;
//-----------------------------------------
float calibration_adjustment_ph = -.08; // adjust this to calibrate
int voltage_input_ph = 4.98; // voltage can be 5 or 3.3
//-----------------------------------------
float calibration_value_ph = 21.34 + calibration_adjustment_ph;
unsigned long int average_value_ph;
int buffer_array_ph[10],temp;
float ph_value; // actual pH value to display on screen
float getPH()
  {
    timer.run(); // Initiates SimpleTimer
    // Use the for loop we take samples, then arrange the values, and finally take the average.
    //Serial.print("Loop readings = ");
    for(int i=0;i<10;i++) // take 10 readings
      { 
        buffer_array_ph[i]=analogRead(ph_in);
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
  float voltage_ph = (float)average_value_ph * voltage_input_ph / 1024 / 6; 
  ph_value = (-5.70 * voltage_ph) + calibration_value_ph; // Calculate the actual pH
  
 delay(0); // pause between serial monitor output - can be set to zero after testing
 return ph_value;
}
*/

// =======================================================
// ======= PPM OCEAN TDS METER SENSOR ====================
// =======================================================
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;
int getMedianNum(int bArray[], int iFilterLen) 
  {
    int bTab[iFilterLen];
    for (byte i = 0; i<iFilterLen; i++)
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
int getTDSReading()
  {
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
          averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
          float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
          float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
          tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
          
      }
    return tdsValue;
  }

// =======================================================
// ======= TEMPURATURE SENSOR DS18B20 ====================
// =======================================================
#define TEMPERATURE_PRECISION 11
OneWire oneWire(temp_pin);// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature waterTempSensor(&oneWire); // Pass our oneWire reference to Dallas Temperature.

float tempC; // tempurature in Celsius
float tempF; // tempurature in Fahrenheit

void getWaterTemp()
  {
    waterTempSensor.requestTemperatures();    // send the command to get temperatures
    tempC = waterTempSensor.getTempCByIndex(0);  // read temperature in °C
    tempF = tempC * 9 / 5 + 32; // convert °C to °F
    if (tempC == DEVICE_DISCONNECTED_C) // Something is wrong, so return an error
      {
        tempC = -100; 
        tempF = tempC;
      }
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
    getWaterTemp(); // will return -100 if there is an issue
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
    int tds = getTDSReading();
    //Serial.print(tds);
    if (tds > 1000) lcd.print("(error)     ");
    else 
    {
      lcd.print(tds);
      lcd.print("(PPM)     ");
    }
    // ---- Display pH Reading
    /*
    lcd.setCursor(5,2);
    float ph = getPH();
    lcd.print(ph); 
    //----Display  Pump status
    lcd.setCursor(5,3);
    if (digitalRead(pump_out) == 0) lcd.print("ON     ");
    else lcd.print("OFF    ");
    */
  }

/*
// ==================================================
// ===========  PUMP CONTROL ========================
// ==================================================
int pump_init_delay_mills = pump_init_delay*1000;
int pump_on_time_mills = pump_off_time *1000; // pump turns on after the time off delay
int pump_off_time_mills = pump_off_time *1000; // pump turns off after teh time on delay
int pump_on_timerID; // setup the on and off timer ids
int pump_off_timerID;
int pump_previous_state; // used to check if state of pump has changed
int pump_pin_value = digitalRead(pump_out);

void turnOnPump()
{
  Serial.print("Turning on pump...  ");
  digitalWrite(pump_out, LOW); // turn on pump
  Serial.print("Pump is on : ");
  Serial.println(digitalRead(pump_out));
 }

void turnOffPump()
{
  Serial.print("Turning off pump...  ");
  digitalWrite(pump_out, HIGH); // turn off pump
  Serial.print("Pump is off : ");
  Serial.println(digitalRead(pump_out));
}

void pumpTimerFunction()
{
  pump_pin_value = digitalRead(pump_out);
  Serial.print("pin value : ");
  Serial.print(pump_pin_value);
  if (pump_pin_value == 1)  // is it off?
    {
      Serial.print("   Pump is off : ");
      Serial.println(pump_pin_value);
      if (pump_previous_state == 0) // If it was previously on
        {
          Serial.println("pump has changed state to off ... ");
          timer.setInterval(pump_off_time_mills, turnOnPump); // start timer to turn it off
          pump_previous_state = 1;
        }
    }
  if (pump_pin_value == 0)
    {
      Serial.print("Pump is on : ");
      Serial.println(pump_pin_value);
      if (pump_previous_state == 1) // if it just changed from off
        {
          Serial.println("pump has changed state to on... ");
          timer.setInterval(pump_on_time_mills, turnOffPump);
          pump_previous_state = 1;
        }
    }
}

void initalizePumpTimer() // Only run once at startup
{
  Serial.print("initalizePumpTimer Started");
  pinMode(pump_out, OUTPUT); // setup Motor control pin to output
  timer.setTimeout(pump_off_time_mills, pumpTimerFunction);
  pump_previous_state = 1;
  Serial.print("pump previous state set to current : ");
  Serial.println(pump_previous_state);
}

void checkPumpStatus()
{
  
  pump_pin_value = digitalRead(pump_out);
  Serial.print("pin value : ");
  Serial.println(pump_pin_value);
  
    
    
}

// --- Serial print counter -----
void printDigits(int digits) {//For displaying timer
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void pumpTimer()
{
  int h,m,s;
  s = millis() / 1000;
  m = s / 60;
  h = s / 3600;
  s = s - m * 60;
  m = m - h * 60;
  Serial.print(h);
  printDigits(m);
  printDigits(s);
  Serial.println();
}

// ==================================================
// ===========  RTC STUFF ===========================
// ==================================================
void rtcSetup()
{


}

*/

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

    displaySplashscreen();
    displayMainscreenstatic();
  
  }

// ====================================================
// ===========  MAIN LOOP =============================
// ====================================================

void loop(void)
{
  //sensors.requestTemperatures(); // request tempuratures
  //getTDSReading(); // request TDS reasding
  //getPH(); // get pH reading
  //timer.run();
 
  //pumpTimerFunction();
  
 

 // displayMainscreenData();

}

// ----------------- END MAIN LOOP ------------------------



