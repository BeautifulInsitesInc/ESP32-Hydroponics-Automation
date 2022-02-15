#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <SimpleTimer.h>
#include <esp_adc_cal.h>
#include <RTClib.h>
#include <SPI.h>
#include <millisDelay.h> // part of the SafeString Library
#include <AiEsp32RotaryEncoder.h> //Rotary encodder library
#include <AiEsp32RotaryEncoderNumberSelector.h> // helper to set the acceleration and limits of rotary encoder
#include <WiFi.h>  // for ota update
#include <AsyncTCP.h> // for ota update
#include <ESPAsyncWebServer.h>// for ota update
#include <AsyncElegantOTA.h>// for ota update
#include <SPIFFS.h> // for uploading webfiles to 
//#include <Arduino_JSON.h> // to make handling json files easier
#include <Adafruit_ADS1X15.h> // For Adafruit 4 channel ADC Breakout board SFD1015
#include <DHT.h> // Humidity and tempurature sensor

LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
SimpleTimer timer;
Adafruit_ADS1115 ads; // Use this for the 16-bit version ADC

const char* ssid = "Office 2.4";
//const char* ssid = "Free Viruses";
//const char* ssid = "TekSavvy";
const char* password = "cracker70";

AsyncWebServer server(80);

// ----- DEFAULT SETTINGS ------
bool temp_in_c = true; // Tempurature defaults to C
float heater = 25; // Tempurature that shuts off the heater
float heater_delay = .5; // Delay heater power on initiation in minutes
float moisture_delay = 2; // Delay between moisture sensing

bool twelve_hour_clock = true; // Clock format

float pump_init_delay = .5; // Minutes - Initial time before starting the pump on startup
float pump_on_time = 1; // Minutes - how long the pump stays on for
float pump_off_time = 2; // Minutes -  how long the pump stays off for

float ph_set_level = 6.2; // Desired pH level
float ph_delay_minutes = 0.5;// miniumum period allowed between doses in minutes
float ph_dose_seconds = 1; // Time Dosing pump runs per dose in seconds;
float ph_tolerance = 0.5; // how much can ph go from target before adjusting

float ppm_set_level = 400; // Desired nutrient level
float ppm_delay_minutes = .5; //period btween readings/doses in minutes
float ppm_dose_seconds = 1; // Time Dosing pump runs per dose

float blink_delay = 1; // blinking indicator blink speed in seconds

// ----- SET PINS ------------------
// Pin 21 - SDA - RTC and LCD screen
// Pin 22 - SCL - RTC and LCD screen
//const int tds_pin = 34; // TDS sensor pin - try 13 if 26 doesnt work - This is now using adc1
//const int ph_pin = 35; // pH sensor pin
#define dht_pin 17 // Humidity and air tempurature sensor
OneWire oneWire(16);// Tempurature pin - Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
// ADC Board
int16_t adc0; //pH sensor
int16_t adc1; //TDS sensor
int16_t adc2; //Moisture Sensor

const int pump_pin = 32; // pump relay
const int heater_pin = 33; // heater relay

const int ph_up_pin = 25; //pH up dosing pump
const int ph_down_pin = 26; // pH down dosing pump
const int ppm_a_pin = 19; // nutrient part A dosing pump
const int ppm_b_pin = 18; // nutrient part B dosing pump

#define ROTARY_ENCODER_A_PIN 36 // CLK pin
#define ROTARY_ENCODER_B_PIN 39  // DT pin
#define ROTARY_ENCODER_BUTTON_PIN 27 // SW (Button pin)
#define ROTARY_ENCODER_VCC_PIN -1  // Set to -1 if connecting to VCC (otherwise any output in)
#define ROTARY_ENCODER_STEPS 4

// ==================================================
// ===========  OTA UPDATES =========================
// ==================================================
void setupWebServer()
  {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("");

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "hello world");
    });
    AsyncElegantOTA.begin(&server);    // Start ElegantOTA
    server.begin();
    Serial.println("HTTP server started");
  }

void testFileUpload() {
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  
  File file = SPIFFS.open("/test.txt");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
  
  Serial.println("File Content:");
  while(file.available()){
    Serial.print(file.read());
  }
  file.close();
}
// *************** CALIBRATION FUNCTION ******************
// To calibrate actual votage read at pin to the esp32 reading
uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

float convertCtoF(float c)
  {
    float f = c*1.8 + 32;
    return f;
  }

// =======================================================
// ======= MOISTURE SENSOR ====================
// =======================================================
int moisture_value;
millisDelay moistureDelay;

void moistureReading()
  {
    if (moistureDelay.justFinished())
    {
      int16_t reading = ads.readADC_SingleEnded(2);
      moisture_value = map(reading, 9500, 0, 100, 0);
      moistureDelay.repeat();
      //Serial.print("reading : "); Serial.print(reading);
      //Serial.print("     moisture_value = "); Serial.print(moisture_value); Serial.println("%");

    }
    
  }

// =======================================================
// ========== DHT Sensor ==============================
// =======================================================
#define DHTTYPE DHT11   // DHT 11
DHT dht(dht_pin, DHTTYPE);
millisDelay dhtDelay;
float dht_tempC = 0;
float dht_tempF = 0;
float dht_humidity = 0;

void dhtIntilization()
  {
    dht.begin(); // initialize humidity sensor
    dhtDelay.start(3000); // sensor can only sample every 1 second
    Serial.print("dht Sensor initilized");
    pinMode(dht_pin, INPUT_PULLUP);
    //pinMode(35,INPUT); // INPUT_PULLUP INPUT_PULLDOWN
    //pinMode(34,INPUT); // INPUT_PULLUP INPUT_PULLDOWN
  }

void dhtReadings()
  {
    if (dhtDelay.justFinished())
      {
        dht_tempC = dht.readTemperature();
        dht_tempF = dht.readTemperature(true);
        dht_humidity = dht.readHumidity();
        dhtDelay.repeat();
        
        Serial.print("reading from pin 34 :"); Serial.print(digitalRead(34));
        Serial.print("   reading from pin 35 :"); Serial.print(digitalRead(35));
        Serial.print("   Temperature = "); Serial.print(dht_tempC); Serial.print("       F: ");Serial.print(dht_tempF);
        Serial.print("       Humidity = "); Serial.println(dht_humidity);
        
      }
  }
void displayDHTmain()
  {
    if (temp_in_c == true) {lcd.print(" "); lcd.print(dht_tempC,0); lcd.print(" "); lcd.print(dht_humidity,0); lcd.print("%");}
    else {lcd.print(" "); lcd.print(dht_tempF,0); lcd.print(" "); lcd.print(dht_humidity,0); lcd.print("%");}
  }
// =======================================================
// ========== RTC Functions ==============================
// =======================================================
RTC_DS3231 rtc; 
DateTime uptime;
DateTime now;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
bool display_seconds = false;
int year, month, day, dayofweek, minute, second, hour,twelvehour; // hour is 24 hour, twelvehour is 12 hour format
bool ispm; // yes = PM
String am_pm;

void initalize_rtc()
  {
    if (! rtc.begin()) 
      {
        Serial.println("Couldn't find RTC");
        Serial.flush();
        while (1) delay(10);
      }
    if (rtc.lostPower()) 
      {
        Serial.println("RTC lost power, let's set the time!");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //sets the RTC to the date & time this sketch was compiled
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0)); //January 21, 2014 at 3am you would call:
      } 
    DateTime uptime = rtc.now(); // Set time at boot up
  }

void setTimeVariables()
  {
    DateTime now = rtc.now();
    year = now.year(); month = now.month(); day = now.day(); dayofweek = now.dayOfTheWeek(); hour = now.hour(); twelvehour = now.twelveHour(); ispm =now.isPM(); minute = now.minute(); second = now.second();
  }

void printDigits(int digit) // To alwasy display time in 2 digits
  {
      lcd.print(":");
      if(digit < 10) lcd.print('0');
      lcd.print(digit);
  }

void displayTime()  // Displays time in proper format
  {
    if (twelve_hour_clock == true)
      {
        hour = twelvehour;
        if (ispm == true) am_pm = "PM";
        else am_pm = "AM";
      }
    lcd.print(hour);
    printDigits(minute);
    if (twelve_hour_clock == true) lcd.print(am_pm);
    if (display_seconds == true) printDigits(second);
  }

// =======================================================
// ======= TEMPURATURE SENSOR DS18B20 ====================
// =======================================================
float tempC; // tempurature in Celsius
float tempF; // tempurature in Fahrenheit
millisDelay heaterTimer;
#define TEMPERATURE_PRECISION 10
DallasTemperature waterTempSensor(&oneWire); // Pass our oneWire reference to Dallas Temperature.

void getWaterTemp()
  {
    waterTempSensor.requestTemperatures();    // send the command to get temperatures
    tempC = waterTempSensor.getTempCByIndex(0);  // read temperature in °C
    tempF = tempC * 9 / 5 + 32; // convert °C to °F
    if (tempC == DEVICE_DISCONNECTED_C) // Something is wrong, so return an error
      {
        //Serial.println("Houston, we have a problem");
        tempC = -1; 
        tempF = tempC;
      }
  }

void checkHeater()
  {
      if (heaterTimer.remaining()==0) // if delay is done, start heater if needed
        {
          if (tempC < heater) digitalWrite(heater_pin, LOW);
          else digitalWrite(heater_pin, HIGH);
        }
  }

// =======================================================
// ======= PH SENSOR =====================================
// =======================================================
float ph_value; // actual pH value to display on screen
float ph_calibration_adjustment = -1.26; // adjust this to calibrate
//float calibration_value_ph = 21.34 + ph_calibration_adjustment;

void getPH()
  {
    float voltage_input = 3.3; // voltage can be 5 or 3.3
    unsigned long int average_reading ;
    unsigned long int buffer_array_ph[10],temp;
    float calculated_voltage; // voltage calculated from reading

    for(int i=0;i<10;i++) // take 10 readings to get average
      { 
        buffer_array_ph[i]=ads.readADC_SingleEnded(0); // read the voltage
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
    average_reading = 0;
    for(int i=2;i<8;i++)
      {
        average_reading  += buffer_array_ph[i];
      }
    average_reading  = average_reading  / 6;
    calculated_voltage = ads.computeVolts(average_reading);
    ph_value = voltage_input * calculated_voltage + ph_calibration_adjustment;
    //if (ph_value < 2 || ph_value > 11) ph_value = -10; // make this an error condition


    /*
    Serial.print("    average_reading  = "); Serial.print(average_reading );
    Serial.print("      calculated_voltage = "); Serial.print(calculated_voltage);
    Serial.print("     ph_value = "); Serial.println(ph_value);
    adc0 =ads.readADC_SingleEnded(0);
    Serial.print("   ADC Reading : "); Serial.print(adc0); Serial.print("   ADC voltage : "); Serial.println(We get it; you have been heard! We need way more funding for education!);
    delay(0); // pause between serial monitor output - can be set to zero after testing
    */
  }

// =======================================================
// ======= PPM OCEAN TDS METER SENSOR ====================
// =======================================================
int tds_value = 0;
const int sample_count = 30;    // sum of sample point
int analogBuffer[sample_count]; // store the analog value in the array, read from ADC
int analogBufferTemp[sample_count];
int analogBufferIndex = 0,copyIndex = 0;

// Function to get median
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

void getTDSReading()
  {
    //const float voltage_input = 3.3;  // analog reference voltage(Volt) of the ADC
    //const float adc_resolution = 4095;
    float average_voltage = 0;
    unsigned long int average_reading;
    float temperature = 25;
   
    // get current tempurature
    if (tempC == -100) temperature = 25;
    else temperature = tempC;
    static unsigned long analogSampleTimepoint = millis();
    if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
      {
        analogSampleTimepoint = millis();
        //analogBuffer[analogBufferIndex] = analogRead(tds_pin);    //read the analog value and store into the buffer
        analogBuffer[analogBufferIndex] = ads.readADC_SingleEnded(1);    //read the analog value and store into the buffer
        analogBufferIndex++;
        if(analogBufferIndex == sample_count) 
            analogBufferIndex = 0;
      }   
    static unsigned long printTimepoint = millis();
    if(millis()-printTimepoint > 800U)
      {
        printTimepoint = millis();
        for(copyIndex=0;copyIndex<sample_count;copyIndex++)
          analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
        average_reading = getMedianNum(analogBuffer,sample_count);
        average_voltage = ads.computeVolts(average_reading);
        float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
        float compensationVolatge=average_voltage/compensationCoefficient;  //temperature compensation
        tds_value=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      
        /*
        Serial.print("Average Read: "); Serial.print(average_reading);
        Serial.print("   average Voltage: "); Serial.print(average_voltage);
        Serial.print("   Temp: "); Serial.print(temperature);
        Serial.print("   compensationVoltage: "); Serial.print(compensationVolatge);
        Serial.print("   TtdsValue: "); Serial.println(tds_value, 0);

        Serial.print("current read : "); Serial.print(ads.readADC_SingleEnded(1));
        Serial.print("   current voltage : "); Serial.println(ads.computeVolts(ads.readADC_SingleEnded(1)));
        delay(0);
        */
      }
  }

// =================================================
// ========== DOSING PUMPS =========================
// =================================================
int ph_dose_pin; //  used to pass motor pin to functions
millisDelay phDoseTimer; // the dosing amount time
millisDelay phDoseDelay; // the delay between doses - don't allow another dose before this
millisDelay phBlinkDelay; // used to blink the indicator if dosing is happening

millisDelay ppmDoseTimerA;
millisDelay ppmDoseTimerB;
millisDelay ppmDoseDelay;
bool next_ppm_dose_b = false;

void phDose(int motor_pin) // turns on the approiate ph dosing pump
  {
    digitalWrite(motor_pin, LOW); // turn on dosing pump
    phDoseTimer.start(ph_dose_seconds*1000); // start the pump
    phBlinkDelay.start(blink_delay*1000); // start delay for blinking indicator
    ph_dose_pin = motor_pin;
    phDoseDelay.start(ph_delay_minutes * 60 * 1000); // start delay before next dose is allowed
    Serial.print("A ph dose has been started, timer is runnning. Dose pin : "); Serial.println(ph_dose_pin);
  }

void phBalanceCheck() //this is to be called from pump turning on function
  {
    if (phDoseTimer.justFinished()) digitalWrite(ph_dose_pin, HIGH);// dosing is done, turn off, and start delay before next dose is allowed
    if (phDoseDelay.remaining()<=0)  
      {
        //Serial.println("PH Dose timer is not running checking if adjustment is needed");
        if (ph_value < ph_set_level - ph_tolerance)
          {
            phDose(ph_up_pin);
          }  // ph is low start ph up pump
        if (ph_value > ph_set_level + ph_tolerance) phDose(ph_down_pin); // ph is high turn on lowering pump
      }
    //else Serial.println("Dose pause timer is runnning - not allowing another dose");
  }

void ppmDoseA()
  {
    digitalWrite(ppm_a_pin, LOW); // turn on ppm dosing pump
    ppmDoseTimerA.start(ph_dose_seconds*1000); // start the pump
    ppmDoseDelay.start(ppm_delay_minutes * 60 * 1000); // start delay before next dose is allowed
    Serial.println("Nutrient dose A has been started, timer is runnning");
    next_ppm_dose_b = true; // run ppm dose B next
  }

void ppmDoseB()
  {
    digitalWrite(ppm_b_pin, LOW); // turn on ppm dosing pump
    ppmDoseTimerB.start(ph_dose_seconds*1000); // start the pump
    ppmDoseDelay.start(ppm_delay_minutes * 60 * 1000); // start delay before next dose is allowed
    Serial.println("Nutrient dose A has been started, timer is runnning");
  }
void ppmBlanceCheck()
  {
    // check if its time to turn off the doseing pump
    if (ppmDoseTimerA.justFinished()) digitalWrite(ppm_a_pin, HIGH);// dosing is done, turn off, and start delay before next dose is allowed
    if (ppmDoseTimerB.justFinished()) digitalWrite(ppm_b_pin, HIGH);// dosing is done, turn off, and start delay before next dose is allowed

    // check if ppm dosing is required
    if (ppmDoseDelay.isRunning()) //check if ppm dose delay is running
      {
        //Serial.print("PPM Dose delay timer is running, not alloweed to dose yet");
      }
    else
      {
        Serial.print("PPM dose delay timer is not running, dosing can happen");
        if (next_ppm_dose_b == true) 
          {
            ppmDoseB();
            next_ppm_dose_b = false;
            Serial.println("Sending Dose B");
          }
        
        else 
          {
            if (tds_value < ppm_set_level) ppmDoseA();
            next_ppm_dose_b = true;
            Serial.println("Sending Dose A");
          }
      }
  }
void doseTest()
  {
    Serial.print("Starting dosing test in 10 seconds");
    delay(5);
    
    digitalWrite(ph_up_pin, LOW); Serial.println("PH UP is LOW - motor on");
    delay(500);
    digitalWrite(ph_up_pin, HIGH); Serial.println("PH UP is HIGH - motor off");
    delay(1000);

    digitalWrite(ph_down_pin, LOW); Serial.println("PH down is LOW");
    delay(500);
    digitalWrite(ph_down_pin, HIGH); Serial.println("PH down is HIGH");
    delay(1000);

    digitalWrite(ppm_a_pin, LOW); Serial.println("Nutrient A is LOW - motor on");
    delay(500);
    digitalWrite(ppm_a_pin, HIGH); Serial.println("Nutrient A  is HIGH - motor off");
    delay(1000);

    digitalWrite(ppm_b_pin, LOW); Serial.println("Nutrient B is LOW");
    delay(500);
    digitalWrite(ppm_b_pin, HIGH); Serial.println("Nutrient B is HIGH");
    delay(1000);
  }

// ==================================================
// ===========  PUMP CONTROL ========================
// ==================================================
int pump_seconds; // current seconds left
int pump_minutes;
millisDelay pumpOnTimer;
millisDelay pumpOffTimer;
bool clear_screen = false;

void setPumpSeconds() // Change seconds into minutes and seconds
  { 
    pump_minutes = pump_seconds / 60;
    if (pump_seconds < 60) pump_minutes = 0;
    else pump_seconds = pump_seconds - (pump_minutes * 60);
  }

void pumpTimer()
  {
    if (digitalRead(pump_pin) == 1 ) // pump is off, check timer
      {
        pump_seconds = pumpOffTimer.remaining() / 1000;
        if (pumpOffTimer.justFinished()) // off delay is done, start pump
          {
            digitalWrite(pump_pin, LOW); // turn on pump
            Serial.print("pump_on_time : ");
            Serial.print(pump_on_time);
            pumpOnTimer.start(pump_on_time * 60 * 1000);
            pump_seconds = pumpOnTimer.remaining() / 1000;
            clear_screen = true;
            //phBalanceCheck(); // check to see if ph dose is needed
            
          }
      }
    else // pump is on, check timing
      {
        pump_seconds = pumpOnTimer.remaining() /1000;
        if (pumpOnTimer.justFinished()) // on time is done turn off
          {
            digitalWrite(pump_pin, HIGH); // turn off pump
            pumpOffTimer.start(pump_off_time * 60 * 1000);
            pump_seconds = pumpOffTimer.remaining() /1000;
          }
      }
    setPumpSeconds();
  }

// ==================================================
// ===========  ROTARY ENCODER ======================
// ==================================================
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoderNumberSelector numberSelector = AiEsp32RotaryEncoderNumberSelector();

void IRAM_ATTR readEncoderISR() {rotaryEncoder.readEncoder_ISR();}

void initilizeRotaryEncoder()
  {
    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    
    //rotaryEncoder.setBoundaries(0, 1000, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    rotaryEncoder.setAcceleration(0); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  }

void rotaryChanged()
  {
    int rotaryValue = rotaryEncoder.readEncoder();
    lcd.setCursor(15,1); lcd.print("    ");
    lcd.setCursor(15,1); lcd.print(rotaryValue);
  }

void rotaryClicked()
  {
    lcd.setCursor(15,1); lcd.print("    ");
    lcd.setCursor(15,1); lcd.print("C");
    delay(1000);
    lcd.setCursor(15,1);lcd.print("    ");
  }

void loopRotaryEncoder()
  {
    if (rotaryEncoder.encoderChanged())
      {
        rotaryChanged();
      }
    if (rotaryEncoder.isEncoderButtonClicked())
      {
        rotaryClicked();
      }
  }

// =================================================
// ========== LCD DISPLAY ==========================
// =================================================
bool ph_blink_on = false;

void displaySplashscreen()// Display the splash screen
  {
    lcd.init(); // Initialize LCD
    lcd.backlight(); // Turn on the LCD backlight
    lcd.setCursor(1,0); lcd.print("CONCIERGE GROWERS");
    lcd.setCursor(5,1); lcd.print("Eat Good");
    lcd.setCursor(4,2); lcd.print("Feel Good");
    lcd.setCursor(4,3); lcd.print("Look Good");
    delay(1000);
  }

void displayMainscreenstatic()// Display the parts that don't change
  {
    lcd.clear();
    lcd.setCursor(1,0); lcd.print("PH:");
    lcd.setCursor(0,1); lcd.print("TDS:");
    lcd.setCursor(0,2); lcd.print("TMP:");
    lcd.setCursor(0,3); lcd.print("PMP:");
  }

// --- Functions for main screen 
void displayPhUorD() // used for flashing or displaying U or D
  {
    if (ph_value < ph_set_level) {lcd.setCursor(18,0); lcd.print("U");}
    else {lcd.setCursor(18,0); lcd.print("D");}
  }
// --- MAIN SCREEN --------------
void displayMainscreenData() // Display the data that changes on main screen
  {
    // ---- PH READING
    lcd.setCursor(4,0); lcd.print(ph_value); 
    lcd.setCursor(12,0); lcd.print("["); lcd.print(ph_set_level,1); lcd.print("]");
    // Display brackets if blancing is happening
    lcd.setCursor(17,0); lcd.print("["); 
    lcd.setCursor(19,0); lcd.print("]");
    if (ph_value < ph_set_level - ph_tolerance || ph_value > ph_set_level + ph_tolerance)
      {
        
        if (phDoseTimer.isRunning()) displayPhUorD();
        else // ph is in waiting period flash indicator
          {
            if (phBlinkDelay.justFinished())
            {if (ph_blink_on == false) 
              {
                lcd.setCursor(18,0); lcd.print(" ");
                phBlinkDelay.repeat();
                ph_blink_on = true;
              }
              else {
                displayPhUorD();
                ph_blink_on = false;
                phBlinkDelay.repeat();
              }
            }
          }
      }
    else {lcd.setCursor(17,0); lcd.print("   ");}
    
    // --- TEMPURATURE
    lcd.setCursor(4,2);
    if (tempC == -10) {lcd.print("(err)  ");}// -1 is an error
    else 
      {
        lcd.setCursor(4,2);
        if (temp_in_c == true) {
          lcd.print(tempC,1); lcd.print((char)223); lcd.print("C");
          lcd.setCursor(14,2); lcd.print(heater,0);
        }
        else {
          lcd.print(tempF,1); lcd.print((char)223); lcd.print("F");
          lcd.setCursor(14,2); lcd.print(convertCtoF(heater),0);
        }
      }
    lcd.setCursor(13,2); lcd.print("[");
    lcd.setCursor(16,2); lcd.print("]");
    lcd.setCursor(17,2); lcd.print("["); // print the heater brackets
    lcd.setCursor(19,2); lcd.print("]");
    if (digitalRead(heater_pin) == 0) {lcd.setCursor(18,2); lcd.print("H");}
    else {lcd.setCursor(18,2); lcd.print(" ");}

    // Display TDS reading
    lcd.setCursor(4,1);
    if (tds_value > 1000) lcd.print("(error)    ");
    else {lcd.print(tds_value); lcd.print(" PPM    ");}
   
    //----Display  Pump status
    lcd.setCursor(4,3);
    if (digitalRead(pump_pin) == 0) lcd.print("ON ");
    else lcd.print("OFF");
    lcd.setCursor(16,3); lcd.print(pump_minutes); printDigits(pump_seconds); // use time fuction to print 2 digit seconds

    // ---- Display time
    //lcd.setCursor(14,0); displayTime();
  }

void displayTempurature()
  {
    lcd.clear();
    lcd.setCursor(5,0); lcd.print("Tempurature");
  }

void displayPH()
  {
    lcd.clear();
    lcd.setCursor(5,0); lcd.print("PH");
  }

void displayTDS()
  {
    lcd.clear();
    lcd.setCursor(5,0); lcd.print("TDS");
  }

void displayPump()
  {
    lcd.clear();
    lcd.setCursor(5,0); lcd.print("Pump");
  }

void displaySettings()
  {
    lcd.clear();
    lcd.setCursor(5,0); lcd.print("Settings");
  }


// ==================================================
// ===========  MAIN SETUP ==========================
// ==================================================
void setup(void)
  {
    Serial.begin(115200);// start serial port 115200
    Serial.println("Starting Hydroponics Automation Controler");
    timer.run(); // Initiates SimpleTimer
    setupWebServer();

    // Check to see if ADS initalized
    if (!ads.begin()) {
      Serial.println("Failed to initialize ADS.");
      while (1);
  }

    // Initialize Sensors
    waterTempSensor.begin(); // initalize water temp sensor
    dhtIntilization();

    // MOISTURE SENSOR
    moistureDelay.start(moisture_delay*1000); // change to minutes later
  
    //Initalize RTC
    initalize_rtc();
    setTimeVariables();

    //Initalize Pump
    pinMode(pump_pin, OUTPUT); digitalWrite(pump_pin,HIGH);
    pumpOffTimer.start(pump_init_delay*60*1000); // start initilization period
    pump_seconds = pumpOffTimer.remaining() /1000;
       
    Serial.print("pump start timer : "); Serial.println(pump_seconds);

    //Initalize Heater
    pinMode(heater_pin, OUTPUT); digitalWrite(heater_pin, HIGH);
    heaterTimer.start(heater_delay *60 * 1000); // start heater initilization delay
    
    // Initalize dosing pumps
    pinMode(ph_up_pin, OUTPUT); digitalWrite(ph_up_pin, HIGH);
    pinMode(ph_down_pin, OUTPUT); digitalWrite(ph_down_pin, HIGH);
    pinMode(ppm_a_pin, OUTPUT); digitalWrite(ppm_a_pin, HIGH);
    pinMode(ppm_b_pin, OUTPUT); digitalWrite(ppm_b_pin, HIGH);
    ppmDoseDelay.start(ppm_delay_minutes*60*100); // start delay before ppm dosing can start
    phDoseDelay.start(ph_delay_minutes*60*10); // start ph delay before dosing can start

    // Initilize Rotary Encoder
    initilizeRotaryEncoder();

    // Prepare screen
    displaySplashscreen();
    displayMainscreenstatic();

    //doseTest(); //used to test ph dosing motors

    testFileUpload();
  }

// ====================================================
// ===========  MAIN LOOP =============================
// ====================================================
void loop(void)
{
  // --- READ SENSORS
  getWaterTemp(); // sets tempC and tempF
  getTDSReading(); // sets tds_value
  getPH();

  // --- HEATER CONTROL
  checkHeater();

  // --- CLOCK
  setTimeVariables();

  // --- PUMP TIMER
  pumpTimer(); // uncomment this to turn on functioning pump timer
  if (clear_screen == true) // clear noise from the screen when pump turns on
    {
      displayMainscreenstatic(); 
      clear_screen = false;;
    } 

  // --- DISPLAY SCREEN
  displayMainscreenData();

  // --- HUMIDITY SENSOR
  dhtReadings();
  
  // - MOISTURE SENSOR
  moistureReading();

  // --- PH BALANCER
  //phBalanceCheck(); // move this to pump on function once testing is complete so it only runs when pump is on

  // --- NUTRIENT BALANCER
  ppmBlanceCheck(); // move this to pump on function once testing is complete so it only runs when pump is on

  // --- ROTARY ENCODER
  loopRotaryEncoder();

}

// ----------------- END MAIN LOOP ------------------------