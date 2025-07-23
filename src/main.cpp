/**
 * @file      ModemSleep.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-05-24
 * @record    https://youtu.be/2cjNsYcU6TU
 * @note      T-A7608 & T-A7608-S3 VBUS of the modem is connected to VUSB.
 *            When using USB power supply, the modem cannot be set to sleep mode. Please see README for details.
 */

// example payload: AT+HTTPPARA="URL","http://124.176.216.142:8889/?id=864643060013606&lat=-28.025770&lon=153.387802&speed=0.20&sats=10&pdop=1.41&mov=1&dbatt=100.00&dinputv=13.01&vrpm=4051epoch=1729129629"
// example payload: AT+HTTPPARA="URL","http://124.176.216.142:8889/?id=864643060013606&lat=-28.025709&lon=153.387680&speed=0.17&sats=8&pdop=1.18&mov=0&dbatt=100.00&dinputv=12.70&vrpm=0&timestamp=1732229085"

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGsmClient.h>
#include <driver/gpio.h>
#include <time.h>
#include "MAX1704.h"
#include "SparkFun_MMA8452Q.h"    // Click here to get the library: http://librarymanager/All#SparkFun_MMA8452Q
#include <Wire.h>

#include "utilities.h"
#include "secrets.h"

#define TINY_GSM_RX_BUFFER          1024 // Set RX buffer to 1Kb

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

#define uS_TO_S_FACTOR      1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP       600          /* Time ESP32 will go to sleep (in seconds) */

TwoWire MYI2C1 = TwoWire(1); 
MMA8452Q accel;                   // create instance of the MMA8452 class

MAX1704 fuelGauge;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, 38, NEO_RGB + NEO_KHZ800); 

#define TINY_GSM_DEBUG SerialAT

#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

String getURL;
time_t now;                         // this is the epoch
tm myTimeInfo;                      // the structure tm holds time information in a more convient way

bool firstBoot = true;
bool gpsUpdateSuccess;
float SOC;
float newAccuracy = 100;
float lat2      = 0;
float lon2      = 0;
float speed2    = 0;
float alt2      = 0;
int   vsat2     = 0;
int   usat2     = 0;
float accuracy2 = 0;
int   year2     = 0;
int   month2    = 0;
int   day2      = 0;
int   hour2     = 0;
int   min2      = 0;
int   sec2      = 0;

// Variable to save current epoch time
unsigned long epochTime; 

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

void showTime() {
    time(&now);                               // read the current time
    localtime_r(&now, &myTimeInfo);           // update the structure tm with the current time
    Serial.print("year:");
    Serial.print(myTimeInfo.tm_year + 1900);  // years since 1900
    Serial.print("\tmonth:");
    Serial.print(myTimeInfo.tm_mon + 1);      // January = 0 (!)
    Serial.print("\tday:");
    Serial.print(myTimeInfo.tm_mday);         // day of month
    Serial.print("\thour:");
    Serial.print(myTimeInfo.tm_hour);         // hours since midnight  0-23
    Serial.print("\tmin:");
    Serial.print(myTimeInfo.tm_min);          // minutes after the hour  0-59
    Serial.print("\tsec:");
    Serial.print(myTimeInfo.tm_sec);          // seconds after the minute  0-61*
    Serial.print("\twday");
    Serial.print(myTimeInfo.tm_wday);         // days since Sunday 0-6
    if (myTimeInfo.tm_isdst == 1)             // Daylight Saving Time flag
        Serial.print("\tDST");
    else
        Serial.print("\tstandard");
    Serial.println();
}

void enableAccel()
{
    delay(2000);
    pinMode(12, ACCEL_INT2);
    pinMode(11, ACCEL_INT2);
    pinMode(ACCEL_INT1, INPUT_PULLDOWN);
    pinMode(ACCEL_INT2, INPUT_PULLDOWN);

    MYI2C1.begin(ACCEL_SDA, ACCEL_SCL);

    delay(1000);

    if (accel.begin(MYI2C1) == false) {
       Serial.println("Not Connected. Please check connections and read the hookup guide.");
    }

    accel.setDataRate(ODR_100);
    accel.setScale(SCALE_2G);

    while (1) {
        // do something to wake on accel int
        if (accel.available()) {      // Wait for new data from accelerometer
            // Orientation of board (Right, Left, Down, Up);
            float calcX = accel.getCalculatedX();
            float calcY = accel.getCalculatedY();
            float calcZ = accel.getCalculatedZ();
            if (accel.isRight() == true) {
                Serial.println("Right");
            }
            else if (accel.isLeft() == true) {
                Serial.println("Left");
            }
            else if (accel.isUp() == true) {
                Serial.println("Down");
            }
            else if (accel.isDown() == true) {
                Serial.print("Up, "); Serial.print(calcX); Serial.print(", "); Serial.print(calcY); Serial.print(", "); Serial.print(calcZ);
                if ((calcX < -0.2) || (calcX > 0.2)) {
                    Serial.println(" Moving left or right!");
                }
                if ((calcY < 0.8) || (calcY > 1.2)) {
                    Serial.println(" Moving up and down!");
                }
                if ((calcZ < -0.2) || (calcZ > 0.2)) {
                    Serial.println(" Moving right or left!");
                }
            }
            else if (accel.isFlat() == true) {
                Serial.println("Flat");
            }
            Serial.println();
        }
        delay(1000);
    }
}

void enableModem()
{
    Serial.println("Enabling modem!");
    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(1000);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    Serial.println("Done enabling modem!");
}

void wakeModem()
{
    Serial.println("Waking modem!");
    // Need to cancel GPIO hold if wake from sleep
    gpio_hold_dis((gpio_num_t )MODEM_DTR_PIN);

    // Pull down DTR to wake up MODEM
    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, LOW);
    delay(2000);
    modem.sleepEnable(false);

    // Delay sometime ...
    delay(5000);
    Serial.println("Done waking modem!");
}

void sleepMCU()
{
    Serial.println("Enter esp32 goto deepsleep!");
    strip.setPixelColor(0, 0, 0, 0); // off
    strip.show();
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    delay(200);
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
}

void sleepModem()
{
    Serial.println("Enter modem sleep mode!");

    // Pull up DTR to put the modem into sleep
    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, HIGH);
    // Set DTR to keep at high level, if not set, DTR will be invalid after ESP32 goes to sleep.
    gpio_hold_en((gpio_num_t )MODEM_DTR_PIN);
    gpio_deep_sleep_hold_en();

    delay(5000);

    Serial.println("Check modem response...");

    int breakCheckModemResponse = 0;
    while (modem.testAT()) {
        breakCheckModemResponse++;
        Serial.printf("Modem asked to sleep, not sleeping yet, retrying %i of 15 in 500ms...\n", breakCheckModemResponse); delay(500);
        if (breakCheckModemResponse > 14) {
            Serial.println("Red LED: Modem still awake");
            strip.setPixelColor(0, 255, 99, 71); // orange, should only happen when charging
            strip.show();
            break;
        }
    }
    if (breakCheckModemResponse > 14) {
        Serial.println("Modem did not go to sleep, maybe the USB cable is connected? Giving up and moving on!");
    } else {
        Serial.println("Modem is not responding to 'AT', modem is asleep!");
        // might not need the below
        if (modem.sleepEnable(true) != true) {
        Serial.println("modem sleep failed!");
        } else {
            Serial.println("Modem enter sleep modem!");
        }
    }
}

void checkModemAwake()
{
    Serial.println("Check modem online .");
    int retry = 0;
    while (!modem.testAT(1000)) {
        Serial.println(".");
        if (retry++ > 10) {
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            delay(100);
            digitalWrite(BOARD_PWRKEY_PIN, HIGH);
            delay(1000);
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            retry = 0;
        }
    }
    Serial.println("Modem is online !");
}

void checkModemOnline()
{
    // Check if SIM card is online
    SimStatus sim = SIM_ERROR;
    while (sim != SIM_READY) {
        sim = modem.getSimStatus();
        switch (sim) {
        case SIM_READY:
            Serial.println("SIM card online");
            break;
        case SIM_LOCKED:
            Serial.println("The SIM card is locked. Please unlock the SIM card first.");
            // const char *SIMCARD_PIN_CODE = "123456";
            // modem.simUnlock(SIMCARD_PIN_CODE);
            Serial.println("Red LED: SIM locked");
            strip.setPixelColor(0, 255, 0, 0); // red
            strip.show();
            break;
        default:
            break;
        }
        delay(1000);
    }

    // Check network registration status and network signal status
    int16_t sq ;
    Serial.print("Wait for the modem to register with the network.");
    RegStatus status = REG_NO_RESULT;
    while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED) {
        status = modem.getRegistrationStatus();
        switch (status) {
        case REG_UNREGISTERED:
        case REG_SEARCHING:
            Serial.println("Red LED: Modem searching");
            strip.setPixelColor(0, 255, 255, 0); // yellow
            strip.show();
            sq = modem.getSignalQuality();
            Serial.printf("[%lu] Signal Quality:%d", millis() / 1000, sq);
            delay(1000);
            break;
        case REG_DENIED:
            Serial.println("Network registration was rejected, please check if the APN is correct");
            Serial.println("Red LED: Modem registration denied");
            strip.setPixelColor(0, 255, 0, 0); // red
            strip.show();
            return ;
        case REG_OK_HOME:
            Serial.println("Green LED: Modem registration successful");
            Serial.println("Online registration successful");
            strip.setPixelColor(0, 0, 0, 255); // blue
            strip.show();
            break;
        case REG_OK_ROAMING:
            Serial.println("Network registration successful, currently in roaming mode");
            break;
        default:
            Serial.printf("Registration Status:%d\n", status);
            delay(1000);
            break;
        }
    }
    Serial.println();

    Serial.printf("Registration Status:%d\n", status);

    String ueInfo;
    if (modem.getSystemInformation(ueInfo)) {
        Serial.print("Inquiring UE system information: ");
        Serial.println(ueInfo);
    }

    if (!modem.enableNetwork()) {
        Serial.println("Enable network failed!");
    }

    String ipAddress = modem.getLocalIP();
    Serial.print("\nNetwork IP:"); Serial.println(ipAddress); Serial.println();

    //Serial.printf("GSM Time: %s\n", modem.getGSMDateTime(DATE_FULL).c_str());
    int     GSMyear = 0;
    int     GSMmonth = 0;
    int     GSMdate = 0;
    int     GSMhours = 0;
    int     GSMminutes = 0;
    int     GSMseconds = 0;
    float   GSMtimezone = 0;
    time_t  GSMUTCtime = 0;

    if (modem.getNetworkTime(&GSMyear, &GSMmonth, &GSMdate, &GSMhours, &GSMminutes, &GSMseconds, &GSMtimezone))
    {
        struct tm s;
        s.tm_sec  = (GSMseconds);
        s.tm_min  = (GSMminutes);
        s.tm_hour = (GSMhours);
        s.tm_mday = (GSMdate);
        s.tm_mon  = (GSMmonth - 1);
        s.tm_year = (GSMyear - 1900);
        GSMUTCtime   = mktime(&s);
        Serial.printf("GSM Time:    %s",  ctime(&GSMUTCtime));

        if (GSMUTCtime > 1615155060) {      //  check for valid time, not 1939!
            setenv("TZ", "AEST-10,M3.5.0/02:00:00,M10.5.0/03:00:00", 1);
            tzset();
            struct timeval tv;
            memset(&tv, 0, sizeof(struct timeval));
            tv.tv_sec = GSMUTCtime;
            settimeofday(&tv, NULL);

            Serial.print("Local Time:\t");
            showTime();  
            epochTime = getTime();
            Serial.printf("Epoch Time:    %i\n",  epochTime);
        }
    }
}

void enableGPS()
{
    Serial.println("Enabling GPS/GNSS/GLONASS");
    int GPSEnableBreakLoop = 0;
    while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO)) {
        Serial.println("Waiting for GPS...");
        GPSEnableBreakLoop++;
        if (GPSEnableBreakLoop > 10) {
            Serial.println("GPS Error!!! Non-recoverable!\n");
            break;
        }
    }
}

void disableGPS()
{
    Serial.println("Disabling GPS");
    modem.disableGPS(MODEM_GPS_ENABLE_GPIO);
}

void createGetURL()
{

    // create GET URL
    getURL += secretServerURL;
    getURL += "?";
    getURL += "id=";
    getURL += String(modem.getIMEI());
    getURL += "&lat=";
    getURL += String(lat2, 6);
    getURL += "&lon=";
    getURL += String(lon2, 6);
    getURL += "&speed=";
    getURL += String(speed2);
    getURL += "&sats=";
    getURL += String(vsat2);
    getURL += "&pdop=";
    getURL += String(accuracy2);
    getURL += "&mov=";
    getURL += String(0); // 0 for no movement, 1 for moving, as per accelerometer
    getURL += "&dbatt=";
    getURL += String(SOC); // was SOC
    getURL += "&dinputv=";
    getURL += String(12.70); // 0 for no movement, 1 for moving, as per accelerometer
    getURL += "&vrpm=";
    getURL += String(0); // 0 for no movement, 1 for moving, as per accelerometer
    getURL += "&timestamp=";
    getURL += String(epochTime);
    Serial.printf("URL: %s\n", getURL.c_str());
}

void getData()
{
    // Initialize HTTPS
    modem.https_begin();

    // Set GET URL
    if (!modem.https_set_url(getURL.c_str())) {
        Serial.println("Failed to set the URL. Please check the validity of the URL!");
        return;
    }

    modem.https_add_header("Accept-Language", "en-AU,en-GB;q=0.9,en-US;q=0.8,en;q=0.7");
    modem.https_add_header("Accept-Encoding", "gzip, deflate, br");
    modem.https_set_accept_type("application/json");
    modem.https_set_user_agent("TinyGSM/AE-4GTv001");


    int httpCode = modem.https_get();

    if (httpCode == 200) {
        Serial.println("HTTPS GET success!"); 
        Serial.println("Green LED: HTTPS GET success");
        strip.setPixelColor(0, 0, 255, 0); // green
        strip.show();
    } else {
        Serial.println("Red LED: HTTPS GET failed");
        Serial.print("HTTPS post failed ! error code = "); 
        Serial.println(httpCode); 
        Serial.println(); 
        strip.setPixelColor(0, 255, 0, 0); // red
        strip.show();
        return;
    }
}

void getSOC() {
    SOC = 0;
    Wire.begin(3,2); 
    delay(1000);
    fuelGauge.reset();
    fuelGauge.quickStart();
    //fuelGauge.showConfig();
    delay(1000);
    SOC = fuelGauge.stateOfCharge(), 2;
    if (SOC > 100) {
      SOC = 100;
    }
    Wire.end();

    Serial.printf("Battery SOC: %f%\n", SOC, 2);
}

void getTemperature() {
     Serial.printf("temperature: %f\n", modem.getTemperature());
}

void clearGPSData()
{
    lat2      = 0;
    lon2      = 0;
    speed2    = 0;
    alt2      = 0;
    vsat2     = 0;
    usat2     = 0;
    accuracy2 = 0;
    year2     = 0;
    month2    = 0;
    day2      = 0;
    hour2     = 0;
    min2      = 0;
    sec2      = 0;
}

void updateGPSLocation()
{
    int breakUpdateGPSLocation = 0;
    int breakAccuracyRequirement = 0;
    clearGPSData();
    uint8_t    fixMode   = 0;
    for (int8_t i = 30; i; i--) { // change this back to 15 after implementing an accuracy increasing counter
        Serial.println("Requesting current GPS/GNSS/GLONASS location");
        if (modem.getGPS(&fixMode, &lat2, &lon2, &speed2, &alt2, &vsat2, &usat2, &accuracy2,
                        &year2, &month2, &day2, &hour2, &min2, &sec2)) {
            if (speed2 > 0) { // here to stop some weird bug where the speed seems to be negative
                Serial.println("Blue LED: GPS online, has lock, waiting for good HDOP");
                strip.setPixelColor(0, 0, 0, 255); // blue
                strip.show();
                gpsUpdateSuccess = true;
                Serial.print("FixMode:"); Serial.println(fixMode);
                Serial.print("Latitude:"); Serial.println(lat2, 6); 
                Serial.print("Longitude:"); Serial.println(lon2, 6);
                Serial.print("Speed:"); Serial.println(speed2); 
                Serial.print("Altitude:"); Serial.println(alt2);
                Serial.print("Visible Satellites:"); Serial.println(vsat2); 
                Serial.print("Accuracy:"); Serial.println(accuracy2);
                Serial.println();

                if (breakAccuracyRequirement > 9)
                {
                   newAccuracy = 0; // make the follow iftatement choose else
                }

                if (accuracy2 <= newAccuracy) {
                    breakAccuracyRequirement++;
                    newAccuracy = accuracy2;
                    Serial.print("Accuracy increasing, continuing...\n");
                } else {
                    newAccuracy = 100; // reset the value from above
                    Serial.print("Disabling GPS...");
                    disableGPS();
                    Serial.print("Accuracy decreasing, updating location\n");
                    //Serial.print("Raw GPS: "); Serial.println(modem.getGPSraw());
                    checkModemAwake();
                    getSOC();
                    checkModemOnline();
                    createGetURL();
                    getData();
                    break;
                }
            } else {
                Serial.println("GPS parse error, retrying in 2s.");
                delay(2000L);
                Serial.println("Yellow LED: GPS parse issue, temporary");
                strip.setPixelColor(0, 255, 255, 0); // yellow
                strip.show();
            }
        } else {
            breakUpdateGPSLocation++;
            Serial.printf("Couldn't get GPS location, retrying %i of 15 in 15s...", breakUpdateGPSLocation);
            Serial.println("Yellow LED: No GPS lock");
            strip.setPixelColor(0, 255, 255, 0); // yellow
            strip.show();
            if (breakUpdateGPSLocation > 14) {
                gpsUpdateSuccess = false;
                break;
            }
            delay(15000L);
        }
    }
}

void setup()
{
    Serial.begin(115200); // Set console baud rate

    strip.begin();
    strip.setBrightness(25);
    Serial.println("Blue LED: Awake!");
    strip.setPixelColor(0, 0, 0, 255); // blue
    strip.show();

    //enableAccel();

    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    delay(5000);

    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println("Firstboot...");
        enableModem();
        enableGPS();
        updateGPSLocation();

        Serial.println("Watch the LED...");
        delay(5000);
        digitalWrite(33, HIGH);
        delay(5000);
        digitalWrite(33, LOW);
        delay(5000);
        digitalWrite(33, HIGH);
        delay(5000);
        digitalWrite(33, LOW);

    } else {
        Serial.println("Woken from sleep by timer...");
        wakeModem();
        enableGPS();
        updateGPSLocation();
    }
    getTemperature();
    sleepModem();
    sleepMCU();
}

void loop()
{
    //this should never be printed:
    Serial.println("Something is very wrong!");
    strip.setPixelColor(0, 128, 0, 128); // purple
    strip.show();
    delay(1000);
    strip.setPixelColor(0, 255, 0, 0); // red
    strip.show();
}



