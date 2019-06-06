// Code Revision History:
//
// MM-DD-YYYY---Version---Description-----------------------------------------------------------------------
// 06-06-2019   v0.1.0    Cleaned up code for publication to https://M5stack.hackster.io
// 05-31-2019   v0.0.13   Added ability to have a power on and off splash screen.  They are /PowerOn.jpg and
//                        /PowerOff.jpg stored on the SD card.  If they are missing, the M5Stack logo will
//                        be displayed instead from the jpeg file /M5_Logo.jpg.  This meant that I could
//                        remove the bmp_map.c file from the project which saved 153572 bytes when compiling.
//                        The .jpg files must not be larger than 320 x 240 in size.
// 05-29-2019   v0.0.12   Finally added Read_Config_File() to read external configuration from SD card.
// 05-28-2019   v0.0.11   Added code to make the OTA Update process nicer on the M5Stack.
// 05-27-2019   v0.0.10   Merged the code to work with either the M5Stack module or a simple ESP32 module.
//                        Created the Fritzing diagram, schematic and PCB board.
// 05-24-2019   v0.0.9    Added code to display images in a lightbox or popup modal window.  You select
//                        which method from the /Config.txt file.  When disabled, you get the popup version, 
//                        otherwise the lightbox version.
// 05-17-2019   v0.0.8    Added Get_Location() function when viewing a waypoint along a route.
//                        Corrected bug where it was not properly disconnecting from WiFi AP after
//                        upgrading Espressif's ESP32 libraries from 1.0.1 to version 1.0.2
// 05-15-2019   v0.0.7    Removed function GetStops() and added code to GPX_FileInfo() function.  This way I
//                        only need to scan the GPX file once when sending the Daily Ride page.  Added code
//                        to display a web page with an iframe containing a stop location along a daily ride.
//                        This uses Google's Reverse Geocoding and requires you to sign up for it.  Check your
//                        Google Dashboard here; https://console.cloud.google.com/google/maps-apis/overview
// 05-11-2019   v0.0.6    Added code for oil change.  Modified some calendar formatting and added code to
//                        list stops in the Daily Ride web page.  Added condition to only send link to
//                        weather location if current location is not home location.  This variable will
//                        to be added to the /Config.txt file.
// 05-02-2019   v0.0.5    Made modifications to TinyGPS++ and moved to local directory to better track and 
//                        keep a GPS fix up to date.
// 05-01-2019   v0.0.4    Statistics page completed.
// 04-27-2019   v0.0.3    Finished adding the motorcycle alarm code.  After playing with it for a while,
//                        I decided I did not want to combine the two projects together.  There's enough
//                        CPU power to handle both codes but it would mean that I would need to build an
//                        expansion unit that would connect to the M5Stack using the side pin connectors.
//                        This would make it messy again like the A9 Pudding board did.  Not something I
//                        want right now.
// 04-24-2019   v0.0.2    Added motorcycle alarm functions to code.  This was another project I completed
//                        over the winter and wanted to see how it would work with the M5Stack or a simple
//                        ESP32 board.
// 04-20-2019   v0.0.1    Re-Start of project (forth version).  Simplified version of the M5Stack GPSTracker
//                        code.  Also decided to drop the A9 Pudding board as I could simply use the M5Stack
//                        built-in WiFi to connect to my cell phone (mobile hotspot).  This required the use
//                        of the M5Stack GPS module board, which made the finished product much nicer and
//                        simpler to use for end users.
// 05-09-2018   v0.0.0    Start of M5Stack project.  Worked on it all summer to get it ready for an
//                        East Coast Roadtrip I had been planning for the month of August.  The first
//                        version involved working with a GPRS/GSM module called a 'A9 Pudding' board.
//                        I wanted to be able to post my location on Dweet.io so that my worry-wart 
//                        sister would know where I am while on this roadtrip.  Had that working pretty
//                        well and started making enhancements to the web-interface.  Over the winter I
//                        did two complete re-write of the code for better performance and tracking.
//
// NOTE: Working with the DOIT ESP32 DEVKIT V1 board I've had to modify the boards.txt file
//       and change the line;
//         esp32doit-devkit-v1.build.board=ESP32_DEV
//       to
//         esp32doit-devkit-v1.build.board=DOIT_ESP32_DEVKIT
//       so that I can differentiate between boards.  Some boards don't have the builtin LED defined.

//
// Define firmware version and device type
//
#define FIRMWARE    "v0.1.0"
#define DEVICETYPE  0xCB06      // Unique project identifier for EEPROM data structures

// Uncomment next line(s) for debugging...
#define SHOW_LOGFILE_MSG      // Output log file messages to serial comm port
//#define TIMINGS               // Display function call timing information
//#define WEB_REQUEST           // Display web interface calls
//#define DEBUG                 // Additional debug info

//
// If using a U-BLOX GPS module, uncomment next line(s) as needed
// M5Stack GPS module uses a U-Blox NEO-M8N-0 module.
//
#define UBLOX         // You're using a U-Blox GPS module
//#define UBLOX_DEBUG   // Output data sent and received from U-Blox module during InitUBLOX()
  
//
// Include these to disable brownouts conditions and to get reset reason.
// This was added because of the DOIT ESP_32 Dev Kit board which is *very* sensitive to brownouts.
// Comment out or remove these lines if your board does not have brownout issues when turning on WiFi.
//
#include <soc/rtc_cntl_reg.h>   // For brownouts
#include <rom/rtc.h>            // For reset reason

//
// Include files needed
//
#ifdef ARDUINO_M5Stack_Core_ESP32
#include <M5Stack.h>

// Make sure UBLOX is defined for the M5Stack GPS module
#ifndef UBLOX
  #define UBLOX
#endif

// Include these for the M5Stack startup music.
extern const unsigned char m5stack_startup_music[];
#endif

//#include <SoftwareSerial.h>           // Include this file if you are using Software Serial for the GPS port
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SD.h>
#include <TimeLib.h>                  // Repository: https://github.com/PaulStoffregen/Time
#include <Timezone.h>                 // Repository: https://github.com/JChristensen/Timezone
#include "TinyGPSMod.h"               // Modified TinyGPS++ library in project directory
#include "polylineencoder.h"
extern "C" { unsigned short CRC16(const char* data, int length); }    // Part of SD card libary

//
// Eastern Time Zone rule (this will need to be changed for other time zones and daylight savings rules)
//
TimeChangeRule myEDT = {"EDT", Second, Sun, Mar, 2, -240};  // Eastern Daylight Time = UTC - 4 hours
TimeChangeRule myEST = {"EST", First, Sun, Nov, 2, -300};   // Eastern Standard Time = UTC - 5 hours
Timezone myTZ(myEDT, myEST);

//
// Define PINs used by sketch
//
#ifdef ARDUINO_ESP32_DEV || ARDUINO_DOIT_ESP32_DEVKIT
  #define SDCard        A13       // SD Card detect pin (input) labeled D15 or IO15
  #define Buzzer        A14       // Output buzzer pin (output) labeled D13 or IO13
  #define Power_GPS     A10       // Pin to power GPS (input) labeled D4 or IO4
  #define BUTTON        A11       // Load Button labeled IO0 or not exposed on other boards
  #define GPS_RX        3         // Pin used for SoftwareSerial RX
  #define GPS_TX        4         // Pin used for SoftwareSerial TX
#ifndef LED_BUILTIN               // Some boards don't have the LED defined in the pin_arduino.h file
  #define LED_BUILTIN   2         // Pin 2 is what's used most often
  #define LED_ON        0         // Define logic level that turns ON the LED
  #define LED_OFF       1         // Define logic level that turns OFF the LED
#endif  
#elif ARDUINO_M5Stack_Core_ESP32
  #define GPS_Port      Serial2
#else
  #error Unsupported board type!
#endif

#ifdef ARDUINO_M5Stack_Core_ESP32
//
// Define M5Stack warning tones
//
enum WARNING_TONES
{
  NoMotionTone = 0,
  TrackingTone,
  ConnectedWiFiTone,
  LostWiFiTone,
  NoSDCardTone,
  NoGPSFixTone,
  EEPROMTone
};

// Since the .beep() function on the M5Stack is non blocking, you need to call M5.update() in order to have it stopped.
// This I think is bad programming as a beep tone should be blocking and not allow other processes to run.
// But to each their own...  Define a blocking beep here.
#define Beep()    { M5.Speaker.tone(1000); delay(100); M5.Speaker.mute(); }
#define LowBeep() { M5.Speaker.tone(400); delay(250); M5.Speaker.mute(); }

#else
//
// Define error beeps (long beep followed by error beep code)
//
enum ERROR_BEEPS
{
  SETUP_BEEP = 1, 
  NO_GPS_ERROR,
  SD_CARD_ERROR,
  EEPROM_ERROR
};

//
// Define status beeps (one short beep + one long beep followed by status beep code)
//
enum STATUS_BEEPS
{
  DEEPSLEEP = 0,  
  TRACKING,
  NOT_TRACKING,
  WIFI_CONNECTED,
  WIFI_DISCONNECT
};
#endif

//
// Define GPX file state
//
enum GPX_FILE_STATES
{
  FILE_CREATED = 1,         // Daily GPX file has been created but nothing is in it
  TRACK_SEGMENT,            // Writing track segments to file
  TRACK_WAYPOINT,           // Way point was just written
  FILE_CLOSED               // Final tracking info written to file
};

//
// Define constants
//
#define OneSecond           1000                // One second
#define OneMinute           (60 * OneSecond)    // One minute
#define OneHour             (60 * OneMinute)    // One hour
#define OneDay              (24 * OneHour)      // One day
#define DisplayDelay        (2 * OneMinute)     // Delay before turning the display off (M5Stack)
#define ScreenDelay         (20 * OneSecond)    // Delay waiting for a button press in a display screen
#define HTTP_Timeout        (15 * OneSecond)    // Defaut timeout for HTTP requests
#define BlinkDelay          (2 * OneSecond)     // Delay between status blinks
#define MotionDelay         (3 * OneMinute)     // Delay when stopped before writing a "No motion" waypoint
#define TrackDelay          (10 * OneSecond)    // Delay between tracking entries in GPX file
#define DweetDelay          (30 * OneSecond)    // Delay between post to Dweet.io
#define WiFiDelay           (45 * OneSecond)    // Delay between WiFi network scans
#define SDCardDelay         (15 * OneSecond)    // Delay between error tones when no SD card present
#define ForecastDelay       (30 * OneMinute)    // Time to wait before getting a new weather forecast
#define OilChangeInterval   (5000 * 1000.0)     // Oil change interval in meters
#define MinSatellites       5                   // Minimum satellites needed for a valid GPS fix

//
// Define data structure stored in EEPROM memory
//
struct CONFIG_RECORD
{
  unsigned int      Init;             // Set to DEVICETYPE if valid data in structure
  unsigned long     Mileage;          // Odometer reading in meters
  unsigned long     DailyRiding;      // Daily riding time in milliseconds
  unsigned long     OilChange;        // Mileage of last oil change in meters
  float             AvgSpeed;         // Average riding speed
  unsigned long     AvgCntr;          // Counter to calculate average speed
  float             MaxSpeed;         // Maximum riding speed
  char              WiFi_SSID[32];    // Wifi SSID name
  char              WiFi_PASS[64];    // Wifi password
  double            HomeLatitude;     // Home latitude position
  double            HomeLongitude;    // Home longitude position
  unsigned int      HomeRadius;       // Distance from home to be considered no longer home
  byte              GPXState;         // State of the GPX file
  time_t            TripStart;        // Trip start date
  unsigned long     TripDistance;     // Trip distance in meters
  unsigned long     TripRiding;       // Trip riding time in seconds
  float             TripAvgSpeed;     // Trip Average speed
  unsigned long     TripAvgCntr;      // Trip Counter to calculate average speed
  float             TripMaxSpeed;     // Trip Maximum speed
  float             TripExpenses;     // Trip expenses recorded
  time_t            SeasonStart;      // Season start date
  unsigned long     SeasonDistance;   // Trip distance in meters
  unsigned long     SeasonRiding;     // Season riding time in seconds
  float             SeasonAvgSpeed;   // Season Average speed
  unsigned long     SeasonAvgCntr;    // Season Counter to calculate average speed
  float             SeasonMaxSpeed;   // Season Maximum speed
  float             SeasonExpenses;   // Sum of all trip expenses
  unsigned short    CRC;
};

struct DATA_RECORD
{
  unsigned int      Init;             // Set to DEVICETYPE if valid data in structure
  char              HostName[20];     // OTA HostName
  char              OTA_Pass[20];     // OTA Password
  char              SSID1[20];        // Cell phone access point
  char              PASS1[20];        // Cell phone password
  char              SSID2[20];        // Home access point
  char              PASS2[20];        // Home password
  char              UserName[32];     // User name
  char              Bike[32];         // Motorcycle type
  char              DweetName[20];    // dweet.io posting name  
  char              OWappid[64];      // OpenWeatherMap.org credentials
  char              GoogleAPI[64];    // Google API key
  bool              DailyShutdown;    // Shutdown device after running nightly scan?
  bool              LightBox;         // View pictures with a lightbox style display?
  unsigned int      MinDailyDistance; // Minimum daily distance to keep GPX file
  unsigned short    CRC;
};

//
// Define EEPROM memory offsets for EEPROM.put function
//
#define CONFIG_OFFSET     0
#define CONFIG_CRC        ((int)&Config.CRC - (int)&Config.Init)
#define DATA_OFFSET       sizeof(CONFIG_RECORD)
#define DATA_CRC          ((int)&DataRec.CRC - (int)&DataRec.Init)

// 
// Create objects needed
//
//SoftwareSerial    GPS_Port(GPS_RX, GPS_TX);   // Using SoftwareSerial
#ifndef ARDUINO_M5Stack_Core_ESP32
  HardwareSerial    GPS_Port(2);              // Hardware serial uses pin 16 as RX and 17 as TX
#endif
TinyGPSPlus       GPS;
HTTPClient        http;
WiFiServer        WebServer(80);
WiFiClient        WebClient;

//
// Define global variables
//
int             TrackSec;
char            DailyGPXFile[32];
String          DailyForecast = "";
String          CurrentWeather = "";
String          OTAError = "";
String          City = "Location Unknown";
String          CityCode = "";
String          PGM = "";
byte            GPXDay, GPSCntr, GPSSymbols[] = "|/-\\";
unsigned long   CurrentTime, BlinkTime, WiFiTime, DweetTime, ForecastTime, TrackTime, QuietTime, GPSTime;
unsigned long   SDCardTime, WebClientTime, DisplayTime, MovingTime, UITime;
double          Latitude, Longitude, LastLAT, LastLON, Speed, LastSpeed, Distance, DailyDistance, DistanceHome, DweetSpeed;
bool            SDCardMissing, OTA_Update, Tracking, WiFiConnected, WasConnected, DailyScan, DailyWeather;
bool            GPSEncoding, LostGPS, ShowNMEA, DisplayOn, TextDisplay;
File            Running;
CONFIG_RECORD   Config;
DATA_RECORD     DataRec;

//
// Write line to log file
//
void WriteLogFile(String Msg, bool Time=true)
{
  // Add time to message
  if(Time) 
  {
    if(timeStatus() == timeSet)
      Msg = "[" + String(getTime(false)) + "] " + Msg;
    else
      Msg = "[??:??:??] " + Msg;
  }

  // Print message to serial output
#ifdef SHOW_LOGFILE_MSG || TIMINGS || WEB_REQUESTS || DEBUG
  Serial.println(Msg);
#endif

  // Is the SD card still inserted
#ifdef ARDUINO_M5Stack_Core_ESP32
  SDCardMissing = !SD.exists("/Weather");
#else  
  SDCardMissing = !digitalRead(SDCard);
#endif  
  if(SDCardMissing) return;

  // Now write to file
  File file = SD.open("/LogFile.txt", FILE_APPEND);
  if(!file) return;
  file.println(Msg);
  file.close();
}

//
// Setup function
//
void setup() 
{
  char            StartTime[32], EndTime[32], Buf[64];
  time_t          t;
  TimeElements    tm;
  int             Cntr, Segments, Tracks, Stops;
  double          Meters, AvgSpeed, MaxSpeed, OilChange;
  unsigned long   Riding, Elapsed;
  String          Card, StopStr;
  
  // Disable brownout detector
#ifdef RTC_CNTL_BROWN_OUT_REG  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif  

#ifndef ARDUINO_M5Stack_Core_ESP32
  // Set pin modes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(SDCard, INPUT_PULLUP);
  pinMode(Power_GPS, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  // Beep to indicate startup
  delay(1000);
  Buzzer_Beep(SETUP_BEEP, true);
#else
  // Initialize M5Stack
  delay(1000);
  M5.begin();
  startupLogo("/PowerOn.jpg");
  M5.Speaker.begin();
  delay(500);

  // Initialize display
  M5.lcd.setBrightness(200);
  M5.Lcd.clearDisplay();
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextSize(2);
#endif
  
  // Initialize serial ports
  Serial.begin(115200); while(!Serial);
  GPS_Port.begin(9600); while(!GPS_Port);

  // Initialize some variables
  SDCardMissing = OTA_Update = ShowNMEA = GPSEncoding = TextDisplay = false;
  CurrentTime = millis();  

#ifndef ARDUINO_M5Stack_Core_ESP32
  // Initialize SD card module
  if(!SD.begin() || !digitalRead(SDCard)) SDCardMissing = true;
#endif

  // Ok, let's start...
  WriteLogFile("\n--------------------------------------------------", false);
#ifdef _ROM_RTC_H_
  WriteLogFile("CPU0 reset reason = " + Get_Reset_Reason(rtc_get_reset_reason(PRO_CPU_NUM)), false);
  WriteLogFile("CPU1 reset reason = " + Get_Reset_Reason(rtc_get_reset_reason(APP_CPU_NUM)), false);
#endif  
  PGM = __FILE__; PGM = PGM.substring(PGM.lastIndexOf("\\")+1); PGM = PGM.substring(0, PGM.indexOf("."));
  WriteLogFile("\n" + PGM + " " + String(FIRMWARE) + " dated " + String(__DATE__), false);
  WriteLogFile("Running on   : " + String(ARDUINO_BOARD), false);

  // Print the type of card
  Card = "SD Card Type : ";
  switch(SD.cardType()) 
  {
    case CARD_NONE:
      Card += "SD Card Missing!";
      SDCardMissing = true;
      break;    
    case CARD_MMC:
      Card += "MMC";
      break;
    case CARD_SD:
      Card += "SD";
      break;
    case CARD_SDHC:
      Card += "SDHC";
      break;
    default:
      Card += "UNKNOWN";
      break;
  }
  WriteLogFile(Card, false);
  if(!SDCardMissing)
  {
    sprintf(Buf, "Total space  : %ul MB", SD.totalBytes() / (1024 * 1024));
    Card = String(Buf);
    sprintf(Buf, "\nUsed space   : %ul MB", SD.usedBytes() / (1024 * 1024));
    Card += String(Buf) + "\n";
    WriteLogFile(Card, false);
  }
  else
  {
    // Failed to initialize the SD card reader or card not inserted.  Hang here beeping...
#ifndef ARDUINO_M5Stack_Core_ESP32
    for(int i=0; i<5; i++) { Buzzer_Beep(SD_CARD_ERROR, true); delay(1000); }
#else
    strcpy(Buf, "SD Card Missing!");
    M5.Lcd.clearDisplay();
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setTextSize(3);
    M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 60);
    for(int i=0; i<5; i++) { WarningTone(NoSDCardTone); delay(1000); }
#endif    
    PowerOff();
    while(1) {}
  }

  // Initialize EEPROM memory
  if(!EEPROM.begin(sizeof(Config)+sizeof(DataRec)))
  {
    // Unable to allocate EEPROM memory
    Card = "Failed to allocate EEPROM memory (" + String(sizeof(Config)+sizeof(DataRec)) + " bytes)!";
    WriteLogFile(Card, false);
#ifndef ARDUINO_M5Stack_Core_ESP32
    for(int i=0; i<5; i++) { Buzzer_Beep(EEPROM_ERROR, true); delay(1000); }
#else    
    M5.Lcd.clearDisplay();
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setTextSize(3);
    strcpy(Buf, "Insufficient");
    M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 60);
    strcpy(Buf, "EEPROM Memory!");
    M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 90);
    for(int i=0; i<5; i++) { WarningTone(EEPROMTone); delay(1000); }
#endif  
    PowerOff();
    while(1) {}
  }

  // Get EEPROM data
  memset(&Config, NULL, sizeof(Config));
  memset(&DataRec, NULL, sizeof(DataRec));
  EEPROM.get(CONFIG_OFFSET, Config);
  EEPROM.get(DATA_OFFSET, DataRec);
  
  // Make sure config record is valid
  unsigned short CRC = CRC16((char *)&Config, CONFIG_CRC);
  if(Config.Init != DEVICETYPE || Config.CRC != CRC)
  {
    // Initialize configuration record
    WriteLogFile("Configuration record initialized (" + String(sizeof(Config)) + " bytes)", false);
    memset(&Config, NULL, sizeof(Config));
    Config.Init = DEVICETYPE;
    Save_Config();
  }

  // Check if I have a /Config.txt file to read
  if(!Read_Config_File())
  {
    // I don't have a file (or it's not in the proper format)
    // make sure the one I have from EEPROM is okay.  Otherwise hang...
    unsigned short CRC = CRC16((char *)&DataRec, DATA_CRC);
    if(DataRec.Init != DEVICETYPE || DataRec.CRC != CRC)
    {
      Card = "Invalid DataRec in EEPROM memory (" + String(sizeof(DataRec)) + " bytes)!";
      WriteLogFile(Card, false);
#ifndef ARDUINO_M5Stack_Core_ESP32
      for(int i=0; i<5; i++) { Buzzer_Beep(EEPROM_ERROR, true); delay(1000); }
#else    
      M5.Lcd.clearDisplay();
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.setTextSize(3);
      strcpy(Buf, "Invalid DataRec!");
      M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 60);
      for(int i=0; i<5; i++) { WarningTone(EEPROMTone); delay(1000); }
#endif  
      PowerOff();
      while(1) {}
    }
  }
  
/* Reset Values
  WriteLogFile("Resetting Config_Rec values...", false);
  memset(&tm, NULL, sizeof(tm));
  tm.Year = 2019 - 1970;
  tm.Month = 4;
  tm.Day = 18;
  tm.Hour = 16;
  tm.Minute = 03;
  tm.Second = 58;
  Config.HomeLatitude = 45.533192;     // Parking = 45.533192 / House = 45.533316
  Config.HomeLongitude = -73.479172;   // Parking = -73.479172 / House = -75.479257
  Config.HomeRadius = 25;
  Config.TripAvgSpeed = Config.SeasonAvgSpeed = 41.2;
  Config.TripAvgCntr = Config.SeasonAvgCntr = 1;
  Config.TripMaxSpeed = Config.SeasonMaxSpeed = 90.6;
  Config.TripStart = Config.SeasonStart = makeTime(tm);
  Config.TripExpenses = Config.SeasonExpenses = 157.29;
  Config.Mileage = (unsigned long)(63012.3 * 1000.0);     // Current odometer reading
  Config.OilChange = (unsigned long)(61894 * 1000.0);   // Last time an oil change was done
  Config.TripRiding = Config.SeasonRiding = ((10 * OneHour) + (32 * OneMinute) + (12 * OneSecond)) / 1000;
  Config.TripDistance = Config.SeasonDistance = Config.Mileage - Config.OilChange;
  Save_Config();
*/

  // Make sure default access point is the cell phone
  strcpy(Config.WiFi_SSID, DataRec.SSID1);
  strcpy(Config.WiFi_PASS, DataRec.PASS1);
  
#ifndef ARDUINO_M5Stack_Core_ESP32
  // Power up GPS
  WriteLogFile("Turning on GPS...", false);
  digitalWrite(Power_GPS, HIGH);
  delay(1000);
#endif

#ifdef UBLOX
  Card = "Initializing GPS module";
  WriteLogFile(Card, false);
#ifdef ARDUINO_M5Stack_Core_ESP32
  M5.Lcd.println(Card);
  Beep();
#endif  
  if(!InitUBLOX()) WriteLogFile("Failed to set all NMEA messages...", false);
#endif  
  
  // Loop here until I have a valid GPS fix
  unsigned long entry = millis();
  ShowNMEA = false;
  Card = "Waiting for GPS fix...";
  WriteLogFile(Card, false);
#ifdef ARDUINO_M5Stack_Core_ESP32
  M5.Lcd.clearDisplay();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("  " + Card);
#endif  
  while(!GPS.location.isValid() || !GPS.date.isValid() || !GPS.time.isValid() || 
    GPS.satellites.value() < MinSatellites || GPS.sentencesWithFix() < 3 || millis() - entry < (5 * OneSecond))
  {
    // Feed TinyGPS++ waiting for a GPS fix
    while(GPS_Port.available() > 0) 
    {
      char c = GPS_Port.read();
      if(ShowNMEA) Serial.print(c);
      GPSEncoding = (GPS.sentencesWithFix() > 3);
      if(GPS.encode(c)) { if(GPS.sentenceType() == GPS_SENTENCE_GPRMC) if(++GPSCntr > 3) GPSCntr = 0; }
    }

#ifdef ARDUINO_M5Stack_Core_ESP32
    M5.update();
    M5.Lcd.setCursor(0, 0); M5.Lcd.printf("%c", GPSSymbols[GPSCntr]); 
    M5.Lcd.setCursor(0, 20); 
    if((millis() - entry) % 1000 == 0 || millis() - entry < 500) 
    {
      M5.Lcd.printf("  Time: %s\n", UpTime(millis() - entry, false));
      M5.Lcd.printf("  Sats: %02d\n\n", (int)GPS.satellites.value());
      M5.Lcd.printf("  WiFi: %-20s\n", Config.WiFi_SSID);
      M5.Lcd.printf("   Ver: %s %s\n", FIRMWARE, ShowNMEA ? "NMEA" : "    ");
    }
#else
    // Blink status LED
    if((millis() - CurrentTime) >= BlinkDelay) 
    { 
      Blink_LED(GPSEncoding ? 1 : 3, false); 
      digitalWrite(Buzzer, HIGH); delay(100); digitalWrite(Buzzer, LOW);
      CurrentTime = millis(); 
    }
#endif

    // If it's been more than 5 minutes and still no GPS fix, error beep
    if(millis() - entry >= (5 * OneMinute))
    {
      Card = "Unable to get GPS fix...";
      WriteLogFile(Card, false);
#ifdef ARDUINO_M5Stack_Core_ESP32
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.setTextSize(3);
      strcpy(Buf, "Unable to get");
      M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 160);
      strcpy(Buf, "valid GPS fix!");
      M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 190);
      for(int i=0; i<5; i++) { WarningTone(NoGPSFixTone); delay(1000); }
#else
      for(int i=0; i<5; i++) { Buzzer_Beep(NO_GPS_ERROR, true); delay(1000); }
#endif      
      PowerOff();
      while(1) {}
    }

    // Was a button pressed
#ifdef ARDUINO_M5Stack_Core_ESP32
    if(M5.BtnA.wasPressed())
#else
    if(!digitalRead(BUTTON)) 
#endif    
    { 
      // Switch between Home and Cell WiFi APs
      if(strcmp(Config.WiFi_SSID, DataRec.SSID2) != 0)
      {
        strcpy(Config.WiFi_SSID, DataRec.SSID2); 
        strcpy(Config.WiFi_PASS, DataRec.PASS2); 
      }
      else
      {
        strcpy(Config.WiFi_SSID, DataRec.SSID1); 
        strcpy(Config.WiFi_PASS, DataRec.PASS1); 
      }
#ifdef ARDUINO_M5Stack_Core_ESP32
      Beep();
#else
      digitalWrite(Buzzer, HIGH);
#endif        
      entry = millis();
    }
#ifdef ARDUINO_M5Stack_Core_ESP32
    if(M5.BtnB.wasPressed()) { Beep(); ShowNMEA = !ShowNMEA; }
#else
    else digitalWrite(Buzzer, LOW);
#endif    
  } 
    
  // Set time from GPS module
  Card = "GPS fix took " + String(UpTime(millis() - entry, false)) + " to get " + String(GPS.satellites.value()) + " satellites.";
  WriteLogFile(Card, false);
  memset(&tm, NULL, sizeof(tm));
  if(GPS.date.isValid() && GPS.time.isValid())
  {
    tm.Year = GPS.date.year() - 1970;
    tm.Month = GPS.date.month();
    tm.Day = GPS.date.day();
    tm.Hour = GPS.time.hour();
    tm.Minute = GPS.time.minute();
    tm.Second = GPS.time.second();
    t = makeTime(tm);
    t = myTZ.toLocal(t);
    setTime(t);
    WriteLogFile("Clock set to " + getDateTime(), false);

    // If the config record was just initialized, set Trip and Season start to now()
    if(Config.TripStart == 0) Config.TripStart = now();
    if(Config.SeasonStart == 0) Config.SeasonStart = now();   
  } else WriteLogFile("Unable to set date/time...");
    
  // Initialize GPX file for today
  GPXDay = GPX_StartDaily();
  GPX_FileInfo(year(), month(), day(), &Meters, &Segments, &Tracks, &Stops, &Riding, &Elapsed, StartTime, EndTime, &AvgSpeed, &MaxSpeed, &StopStr);
  Card = "GPX file state  = ";  
  switch(Config.GPXState)
  {
    case FILE_CREATED:    Card += "File created"; break; 
    case TRACK_SEGMENT:   Card += "Tracking"; break;
    case TRACK_WAYPOINT:  Card += "Waypoint"; break;
    case FILE_CLOSED:     Card += "File closed"; break;
    default:              Card += "Unknown"; break;    
  }
  WriteLogFile(Card);
  Card = "Daily distance  = ";
  if(DailyDistance >= 1000.0) 
    Card += String(DailyDistance / 1000.0, 1) + " Km";
 else
    Card += String(DailyDistance) + " m";
  WriteLogFile(Card);

  // Log mileage info
  Card = "Total mileage   = ";
  if(Config.Mileage + (Meters * 1000.0) >= 1000) 
    Card += String((Config.Mileage + (Meters * 1000.0)) / 1000.0, 1) + " Km";
  else
    Card += String(Config.Mileage + (Meters * 1000.0)) + " m";
  WriteLogFile(Card);
  OilChange = OilChangeInterval - (Config.Mileage + (Meters * 1000.0) - Config.OilChange);
  Card = "Next oil change = ";
  if(OilChange >= 1000) 
    Card += String(OilChange / 1000.0, 1) + " Km";
  else
  {
    if(OilChange >= 0)
      Card += String(OilChange, 0) + " m";
    else
    {
      Card += "DUE ";
      if(abs(OilChange) >= 1000)
        Card += String(abs(OilChange) / 1000.0, 1) + " K";
      else
        Card += String(abs(OilChange), 0) + " ";
      Card += "m ago!";
    }
  }
  WriteLogFile(Card);
  Card = "WiFi Network    = " + String(Config.WiFi_SSID);
  WriteLogFile(Card);
  Card = "OTA HostName    = " + String(DataRec.HostName);
  WriteLogFile(Card);

  // Init WiFi
  WiFi.disconnect();    // Clear Wifi Credentials
  WiFi.mode(WIFI_STA);  // Make WiFi mode is Station 

  // Initialize OTA callbacks
  InitOTA();
  
  // Default some program variables
  BlinkTime = DweetTime = DisplayTime = UITime = millis();
  SDCardTime = (SDCardMissing ? 0 : millis());
  WasConnected = WiFiConnected = Tracking = false;
  DisplayOn = true;
  LastLAT = Latitude = GPS.location.lat();
  LastLON = Longitude = GPS.location.lng();
  WiFiTime = MovingTime = Speed = Distance = DistanceHome  = 0;
    
  // We're done, run main loop
#ifdef ARDUINO_M5Stack_Core_ESP32
  M5.Lcd.clearDisplay();
  UI();
#endif  
  WriteLogFile("Running main program loop...");
  WriteLogFile("--------------------------------------------------\n", false);
}

//
// Main program loop
//
void loop() 
{
  int     WiFiStatus;
  
  // Save time
  CurrentTime = millis();

  // Handle OTA process and do nothing else if I'm in an OTA Update
  ArduinoOTA.handle(); 
  if(OTA_Update) return;

  // Time to blink status LED?
  if((millis() - BlinkTime) >= BlinkDelay) 
  {
    if(GPS.location.age() >= (15 * OneSecond) && GPSTime != WiFiTime)
    {
      // Assume here that either I'm no longer getting NMEA messages or that the GPS location is invalid
      if(!LostGPS) 
      {
        WriteLogFile("Lost GPS signal!");
#ifndef ARDUINO_M5Stack_Core_ESP32
        digitalWrite(Buzzer, HIGH); delay(100); digitalWrite(Buzzer, LOW);
#endif        
        LostGPS = true;
        GPSTime = BlinkTime = millis();
      }
#ifndef ARDUINO_M5Stack_Core_ESP32
      else Blink_LED(3, false);
#endif      
    }
    else
    {
      // Have I restored the GPS signal yet?
      if(LostGPS && GPS.satellites.value() >= MinSatellites && GPS.sentencesWithFix() > 3) 
      {
        WriteLogFile("GPS signal restored in " + String(UpTime(millis() - GPSTime, false)));
        LostGPS = false;
      } 
#ifndef ARDUINO_M5Stack_Core_ESP32
      else Blink_LED(GPSEncoding ? (Tracking ? 2 : 1) : 3, false);
      if(!WiFiConnected && GPSEncoding && !Tracking) { delay(150); Blink_LED(2, true); }
#endif      
    }
    BlinkTime = millis();

    // Check if card is still there?
#ifdef ARDUINO_M5Stack_Core_ESP32
    if(!SDCardMissing) SDCardMissing = !SD.exists("/Weather");
#else  
    if(!SDCardMissing) SDCardMissing = !digitalRead(SDCard);
#endif  
  }

  // Check if I've lost my WiFi connection
  WiFiStatus = WiFi.status();
  if(WiFiStatus != WL_CONNECTED && WiFiConnected) 
  { 
    WriteLogFile("WiFi connection lost!");
#ifdef ARDUINO_M5Stack_Core_ESP32
    WarningTone(LostWiFiTone);
#else
    Buzzer_Beep(WIFI_DISCONNECT, false);
#endif      
    WebServer.stop();
    WiFi.disconnect();
    WiFiConnected = false; 
    WasConnected = true;
    WiFiTime = millis();
  }     

  // If not connected to WiFi and not tracking, try to connect
  if(WiFiStatus != WL_CONNECTED && (millis() - WiFiTime >= WiFiDelay || WiFiTime == 0) && !Tracking)
  {
#ifdef ARDUINO_M5Stack_Core_ESP32
    StatusLine("WiFi Scan");
    Hide_Buttons();
#endif    
    int net = WiFi.scanNetworks();
    for(int i=0; i<net; i++)
    {
      if(WiFi.SSID(i) == String(Config.WiFi_SSID))
      {
#ifdef ARDUINO_M5Stack_Core_ESP32
        if(!DisplayOn) { M5.Lcd.wakeup(); TextDisplay = false; UI(); M5.Lcd.setBrightness(200); }
        DisplayOn = true; DisplayTime = millis();
        StatusLine(Config.WiFi_SSID);
#endif    
        int Cntr = 0;
        WriteLogFile(String(WasConnected ? "Re-c" : "C") + "onnecting to \"" + String(Config.WiFi_SSID) + "\"");
        if(WasConnected)
          WiFi.reconnect();
        else
          WiFiStatus = WiFi.begin(Config.WiFi_SSID, Config.WiFi_PASS);
        while(WiFiStatus != WL_CONNECTED)
        {
          delay(500);
          WiFiStatus = WiFi.status();
          if(++Cntr > 10 || WiFiStatus == WL_CONNECTED) break;
        }
        if(WiFiStatus != WL_CONNECTED) 
        {
          WriteLogFile("Failed to connect (err=" + String(WiFiStatus) + ")");
          WiFi.disconnect(true);
          break;
        }
        else
        {
          WriteLogFile("IP Address: " + WiFi.localIP().toString() + " (" + String(WiFi.RSSI()) + " dBm)");
#ifdef ARDUINO_M5Stack_Core_ESP32
          WarningTone(ConnectedWiFiTone);
#else
          Buzzer_Beep(WIFI_CONNECTED, false);
#endif          
          WiFiConnected = WasConnected = true;
          ForecastTime = 0; // This will force a new weather fetch
          Get_Location(Latitude, Longitude, &City, &CityCode);
          WriteLogFile("After WiFi connect, you are in \"" + City + " / " + CityCode + "\"");
          WebServer.begin();
        }
      }
    }

    // If GPX file is in tracking, start timers to continue tracking
    // This condition will only occur after a device reboot while it was in tracking mode.
    if(Config.GPXState == TRACK_SEGMENT)
    {
      Speed = (GPS.speed.isValid() ? GPS.speed.kmph() : 0);
      WriteLogFile("Tracking after reboot at " + String(Speed, 1) + " Km/h");
      MovingTime = millis();
      TrackTime = 0;
      Tracking = true;
      LastSpeed = Speed;
      DweetPost(Speed);
    } else DweetPost(-1);
    DweetTime = millis();

    // Clean up memory used by scan
    WiFi.scanDelete();
    WiFiTime = millis();
#ifdef ARDUINO_M5Stack_Core_ESP32
    Display_Buttons();
#endif    
  }
  
  // Feed GPS
  while(GPS_Port.available() > 0) 
  {
    char c = GPS_Port.read();
    if(ShowNMEA) Serial.print(c);
    GPSEncoding = (GPS.sentencesWithFix() > 3);
    if(GPS.encode(c))
    {
      // A NMEA sentence has been properly encoded
      if(GPS.location.isValid()) 
      { 
        Latitude = GPS.location.lat(); Longitude = GPS.location.lng(); 
        DistanceHome = TinyGPSPlus::distanceBetween(Latitude, Longitude, Config.HomeLatitude, Config.HomeLongitude);
        if(LastLAT != 0.0 && LastLON != 0.0)
          Distance = TinyGPSPlus::distanceBetween(Latitude, Longitude, LastLAT, LastLON);
        else
          Distance = 0;
      }
      if(GPS.speed.isValid()) Speed = GPS.speed.kmph();

      // If Speed is < 5 Km/h, set speed and distance to zero
      if(Speed < 5.0) Speed = Distance = 0.0;

      // On speed sentences if speed is not zero, increment tracking counter
      // This will trigger the Tracking process if I've had more than 5 seconds of movement
      if(GPS.sentenceType() == GPS_SENTENCE_GPRMC)
      {
        if(Speed != 0) ++TrackSec; else TrackSec = 0;
        if(++GPSCntr > 3) GPSCntr = 0;
      }

      // Calc avg speed and check if I have a new maximum speed
      if(Speed != 0.0 && Tracking)
      {
        Config.AvgSpeed += Speed; ++Config.AvgCntr;
        Config.TripAvgSpeed += Speed; ++Config.TripAvgCntr;
        Config.SeasonAvgSpeed += Speed; ++Config.SeasonAvgCntr;
        if(Config.MaxSpeed < Speed) Config.MaxSpeed = Speed;
        if(Config.TripMaxSpeed < Speed) Config.TripMaxSpeed = Speed;
        if(Config.SeasonMaxSpeed < Speed) Config.SeasonMaxSpeed = Speed;
      }
    }
  }

  // Is there a web client to process? (or update UI on M5Stack)
  if(millis() - WebClientTime >= OneSecond) 
  {
    if(WiFiConnected && !Tracking) ProcessClient();
#ifdef ARDUINO_M5Stack_Core_ESP32
    UI();
#endif
    WebClientTime = millis();
  }

  // If I've been stopped for a while, log a "No motion" waypoint
  if(Speed != 0 || LastSpeed != 0 || LostGPS) QuietTime = millis();
  if(millis() - QuietTime >= MotionDelay)
  { 
    // No motion, was I tracking?
    if(Tracking)
    {
#ifdef ARDUINO_M5Stack_Core_ESP32
      WarningTone(NoMotionTone);
#else
      Buzzer_Beep(NOT_TRACKING, false);
#endif      
      // Get my current location after stopping
      if(WiFiConnected)
      {
        Get_Location(Latitude, Longitude, &City, &CityCode);
        WriteLogFile("After tracking, you are in \"" + City + " / " + CityCode + "\"");
      }
      
      // Write waypoint and reset some variables
      if(MovingTime != 0) Config.DailyRiding += millis() - MovingTime;
      GPX_WayPoint("No motion");  // Do not change the title of this entry.  It is used as a special marker
      Tracking = false;
      TrackSec = TrackTime = WiFiTime = 0;
      ForecastTime = 0; // This will force a new weather fetch
      LastLAT = Latitude;
      LastLON = Longitude;
      DweetPost(-1);
      DweetTime = millis();
    }
  }
  else
  {
    // If I have motion for more than 5 seconds and not tracking, start tracking
    if(!Tracking && TrackSec >= 5 && Speed != 0 && Distance != 0 && GPS.location.isValid()) 
    { 
      WriteLogFile("Tracking at " + String(Speed, 1) + " Km/h with " + String(GPS.satellites.value()) + " satllites");
#ifdef ARDUINO_M5Stack_Core_ESP32
      WarningTone(TrackingTone);
      Hide_Buttons();
#else
      Buzzer_Beep(TRACKING, false);
#endif      
      TrackTime = DweetTime = 0;
      MovingTime = millis();
      Tracking = true;
    }
  }

  // Time to log current position in GPX file?
  if(Tracking && GPS.location.isValid() && millis() - TrackTime >= TrackDelay && (Speed != 0 || (Speed == 0 && LastSpeed != 0)))
  {
    // Save new position and increment distance travelled
    GPX_TrackPoint();
    DailyDistance += Distance;
    LastSpeed = Speed;
    LastLAT = Latitude;
    LastLON = Longitude;
    TrackTime = millis();
  }

  // Time to post to dweet.io?
  if(Tracking && GPS.location.isValid() && millis() - DweetTime >= DweetDelay && (Speed != 0 || (Speed == 0 && DweetSpeed != 0)))
  {
    DweetPost(Speed);
    DweetSpeed = Speed;
    DweetTime = millis();
  }

  // Time to get a new weather forecast?
  if(WiFiConnected && (millis() - ForecastTime >= ForecastDelay || ForecastTime == 0))
  {
    if(Get_Forecast()) WriteLogFile(CurrentWeather);

    // Save today's forecast as today's weather
    if(!DailyWeather && DailyForecast != "")
    {
      // Extract min/max temperature for the day.  If still the same, don't save.
      // OpenWeatherMap.org only updates today's forecast an hour or so after sunrise.
      // And don't save if the forecast is "Mist", "Fog" or "Smoke" (Icon: 50), 
      // this is usually before the real daily forecast.
      double Min = DailyForecast.substring(DailyForecast.indexOf("Min:")+5).toFloat();
      double Max = DailyForecast.substring(DailyForecast.indexOf("Max:")+5).toFloat();
      if(Min != Max && DailyForecast.indexOf("Icon: 50") == -1)
      {
        // Extract today's icon and make sure it's a day icon and not a night icon
        String Icon = DailyForecast.substring(DailyForecast.indexOf("Icon:")+6, DailyForecast.indexOf("Icon:")+9);
        if(Icon.indexOf("n") == -1) 
        {
          if(Save_Weather()) 
          {
            WriteLogFile("Daily weather saved...");
            DailyWeather = true;
          }
        }
      }
    }
    ForecastTime = millis();
  }
  
  // If it's 11:55pm, run daily scan to update monthly travel file (Rides.txt)
  if(!Tracking && hour() == 23 && minute() == 55 && !DailyScan)
  {
    GPX_DailyScan(year(), month());
    DailyScan = true;
  }
  
  // If this is a new day, close active GPX file and reset daily variables
  if(!Tracking && DailyScan && GPXDay != day())
  {
    // Close GPX file and clear daily variables
    GPX_EndDaily();
    DailyDistance = 0.00;
    Config.DailyRiding = Config.AvgSpeed = Config.AvgCntr = Config.MaxSpeed = 0;
    DailyScan = DailyWeather = false;
    DailyForecast = "";
    Save_Config();

    // Check size of LogFile.  If too big, delete old backup and rename current file
    File file = SD.open("/LogFile.txt");
    if(file)
    {
      // Get file size
      if(file.size() / 1024 > 100)
      {
        // LogFile is bigger than 100KB
        file.close();
        SD.remove("/LogFile.bak");
        SD.rename("/LogFile.txt", "/LogFile.bak");
        WriteLogFile("LogFile backup performed...");
      } else file.close();
    }
    
    // Log start of new day
    WriteLogFile("New day...\n\n-----> " + String(getDate(false)) + " <-----\n");

    // Am I configured to shutdown on a daily basis?
    if(DataRec.DailyShutdown)
    {
      DweetPost(-2);
      PowerOff();
    }

    // Not shutting down on a daily basis (or deep sleep failed), initialize for a new day
    GPXDay = GPX_StartDaily();
    DweetPost(-1);
  }

  // Is the SD card missing?  Typically popped out because of bumps while riding...
  // Put scotch tape over the SD card to fix the problem...
  if(millis() - SDCardTime >= SDCardDelay && SDCardMissing)
  {
    // Try to re-initialize the SD file system
    SD.end();
#ifdef ARDUINO_M5Stack_Core_ESP32
    if(SD.begin(TFCARD_CS_PIN, SPI, 40000000))
#else    
    if(SD.begin())
#endif    
    {
      WriteLogFile("SD Card mounted...");
      SDCardMissing = false;
    } 
#ifdef ARDUINO_M5Stack_Core_ESP32
    else WarningTone(NoSDCardTone);
#else
    else Buzzer_Beep(SD_CARD_ERROR, true);
#endif
    SDCardTime = millis();
  }  

#ifdef ARDUINO_M5Stack_Core_ESP32
  // Run the M5Stack update process
  M5.update();
  
  // Time to turn off the display?
  if(millis() - DisplayTime >= DisplayDelay && !SDCardMissing)
  {
    M5.Lcd.setBrightness(0); 
    M5.Lcd.sleep();
    DisplayOn = false;    
  }

  // Now check for a button press
  if(M5.BtnA.wasPressed())
  {
    // Weather Forecast
    Beep(); delay(250);
    if(DisplayOn) 
    {
      // If I'm tracking, do an error beep
      if(!Tracking || SDCardMissing)
      {
        if(DailyForecast != "") Weather(); else LowBeep();
      } else LowBeep();
    } else { M5.Lcd.wakeup(); TextDisplay = false; UI(); M5.Lcd.setBrightness(200); }
    DisplayOn = true;
    DisplayTime = millis();
  }

  if(M5.BtnB.wasPressed())
  {
    // Currently not assigned    
    if(!DisplayOn) Beep();
    if(DisplayOn) 
    {

    } else { M5.Lcd.wakeup(); TextDisplay = false; UI(); M5.Lcd.setBrightness(200); }
    DisplayOn = true;
    DisplayTime = millis();
  }

  if(M5.BtnC.wasPressed())
  {
    // System Menu
    Beep(); delay(250);
    if(DisplayOn) 
    {
      // If I'm tracking, do an error beep
      if(!Tracking) Menu(); else LowBeep();
    } else { M5.Lcd.wakeup(); TextDisplay = false; UI(); M5.Lcd.setBrightness(200); }
    DisplayOn = true;
    DisplayTime = millis();
  }  
#else
  // Check for button press to go into deep sleep
  if(!digitalRead(BUTTON)) { Buzzer_Beep(DEEPSLEEP, false); DweetPost(-2); PowerOff(); }
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// DATE FUNCTIONS
//
////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Get current time
//
char *getTime(bool Short)
{
  static char   Buffer[10];

  // Default buffer
  if(Short) strcpy(Buffer, "00:00"); else strcpy(Buffer, "00:00:00");
  
  // Check if I have the time set
  if(timeStatus() != timeSet) return Buffer;

  // Return short or long time format
  if(Short)
    sprintf(Buffer, "%02d:%02d", hour(), minute());
  else
    sprintf(Buffer, "%02d:%02d:%02d", hour(), minute(), second());
  return Buffer;
}

//
// Get current date
//
char *getDate(bool Short)
{
  static char   Buffer[50];
  String        Date;

  // Default buffer
  strcpy(Buffer, "??? ??? ?? ????");
  
  // Check if I have the time set
  if(timeStatus() != timeSet) return Buffer;

  // I have a valid time...
  if(Short)
    Date = String(dayShortStr(weekday())) + " " + String(monthShortStr(month())) + " " + String(day()) + " " + String(year());
  else
    Date = String(dayStr(weekday())) + " " + String(monthStr(month())) + "-" + String(day()) + ", " + String(year());
  strcpy(Buffer, Date.c_str());
  return Buffer;
}

//
// Return a String containing Date/Time for logging purposes
//
String  getDateTime(void)
{
  return(String(getDate(true)) + " " + String(getTime(false)));  
}

//
// Return UTC time from GPS
// This function is no longer used...  A proper GPX file has the <time> element formatted
// as this function returns.  But I encode the date/time in the <time> element using the now() function.
// Left here for possible future use.
//
char *GetUTCTime(bool utc)
{
  static char   Buffer[50];

  // Default buffer to local date/time
  sprintf(Buffer, "%04d-%02d-%02dT%02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
  if(!GPS.date.isValid() || !utc) return(Buffer);

  // Return GPS UTC date/time
  sprintf(Buffer, "%04d-%02d-%02dT%02d:%02d:%02dZ", GPS.date.year(), GPS.date.month(), GPS.date.day(), GPS.time.hour(), GPS.time.minute(), GPS.time.second());
  return(Buffer);
}

//
// Return the total uptime
//
char  *UpTime(unsigned long CT, bool Short)
{
  int d, h, m, s;
  static char   Buffer[64];
  
  d = CT / OneDay;
  CT -= (d * OneDay);
  h = CT / OneHour;
  CT -= (h * OneHour);
  m = CT / OneMinute;
  CT -= (m * OneMinute);
  s = CT / OneSecond;
  if(!Short)
  {
    if(d > 0)
      sprintf(Buffer, "%d day%s %02d:%02d:%02d", d, (d == 1 ? "" : "s"), h, m, s);
    else
      sprintf(Buffer, "%02d:%02d:%02d", h, m, s);
  }
  else
  {
    // Do I have more than 99 hours?
    if((d * 24) + h < 100)
      sprintf(Buffer, "%02d:%02d:%02d", (d*24) + h, m, s);
    else
      sprintf(Buffer, "%02d|%02d:%02d", d, h, m);
  }
  
  return(Buffer);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// GOOGLE'S GEOCODING FUNCTION (must enable Geolocation API)
//
////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Get current location from Google by doing a Reverse Geocoding look-up
// See: https://developers.google.com/maps/documentation/geocoding/intro#ReverseGeocoding
// Example: https://maps.googleapis.com/maps/api/geocode/json?latlng=48.1074066,-69.2076721&key=YOUR_AI_KEY&result_type=administrative_area_level_5
// Example: https://maps.googleapis.com/maps/api/geocode/json?latlng=48.1074066,-69.2076721&key=YOUR_API_KEY
// Read first compound_code and first global_code, result would be something like "Grève-Morency, QC, Canada/87WG4Q4R+XW"
//
bool Get_Location(double Lat, double Lon, String *C, String *CC)
{
  WiFiClientSecure  client;
  const char*       server = "maps.googleapis.com";
  String            Location, Result, line;
  int               Response, i;
  unsigned long     entry = millis();

  // Init variables
  Result = "Location Unknown"; Location = ""; 
  if(!WiFiConnected) { *C = Result; *CC = Location; return(false); }

#ifdef ARDUINO_M5Stack_Core_ESP32
  StatusLine("Getting Location");
  Hide_Buttons();
#endif
  
  // Try to connect to secure server
  if(!client.connect(server, 443)) 
  { 
    WriteLogFile("[Get_Location] Failed to connect!"); 
    *C = Result; *CC = Location;
    return(false); 
  }

  // Send request
  Location = "GET https://" + String(server) + "/maps/api/geocode/json?latlng=" + String(Lat, 7) + "," + String(Lon, 7) + "&result_type=administrative_area_level_5&key=" + String(DataRec.GoogleAPI);  
#ifdef DEBUG
  WriteLogFile("[Get_Location] query =\n" + Location);
#endif  
  client.printf("%s HTTP/1.1\n", Location.c_str());
  client.printf("Host: %s\n", server);
  client.print("Connection: close\n");
  client.println();
  Response = 0;

  // Read response headers 
  while(client.connected())
  {
    line = client.readStringUntil('\n');
#ifdef DEBUG
    WriteLogFile(line);
#endif    
    if(line.indexOf("HTTP/1.1") != -1)
    {
      line = line.substring(8);
      Response = line.toInt();
    }
    if(line == "\r") break;
  }
#ifdef DEBUG
  WriteLogFile("[Get_Location] Response = " + String(Response));
#endif  
  
  // Read content body
  while(client.available())
  {
    line = client.readStringUntil('\n');
#ifdef DEBUG
    WriteLogFile(line);
#endif    
    i = line.indexOf("compound_code");
    if(i != -1) Result = line.substring(i+26, line.indexOf("\","));        
    i = line.indexOf("global_code");
    if(i != -1) Location = line.substring(i+16, line.indexOf("+")+3);
  }

  // Close connection
  client.stop();
  *C = Result;
  *CC = Location;
#ifdef DEBUG
  WriteLogFile("[Get_Location] Location = " + Result + "/" + Location);
#endif  
#ifdef TIMINGS
  WriteLogFile("Get_Location() = " + String(millis() - entry) + " ms");
#endif  
  return(Response == 200);  
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// WEATHER FUNCTIONS
//
////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Get weather forecast from OpenWeatherMap.org
// See: https://openweathermap.org/api documentation
//
bool Get_Forecast(void)
{
  int           Ret = 404;
  int           Days = 4;
  int           Humidity;
  String        Result = "";
  String        Current = "";
  String        OWCity = "Unknown";
  String        response, Forecast, Text, Icon;
  float         MinTemp, MaxTemp;
  time_t        Sunrise, Sunset;
  char          SunUp[10], SunDown[10];
  unsigned long entry = millis();
  TimeElements  tm1, tm2;

  // Do I have WiFi
  if(!WiFiConnected) return(false);
  
#ifdef ARDUINO_M5Stack_Core_ESP32
  StatusLine("Getting Weather");
  Hide_Buttons();
#endif
  
  // Build request with API key
  String Site = "api.openweathermap.org";
  String Request = "/data/2.5/forecast/daily?cnt=" + String(Days) + "&lat=" + String(Latitude, 3) + "&lon=" + String(Longitude, 3) + "&units=metric&APPID=" + String(DataRec.OWappid);
  String RequestNow = "/data/2.5/weather?lat=" + String(Latitude, 3) + "&lon=" + String(Longitude, 3) + "&units=metric&APPID=" + String(DataRec.OWappid);

  // If I don't have a GPS fix, use home location
  if(Latitude == 0 || Longitude == 0)
  {
    Request = "/data/2.5/forecast/daily?cnt=" + String(Days) + "&lat=" + String(Config.HomeLatitude, 3) + "&lon=" + String(Config.HomeLongitude, 3) + "&units=metric&APPID=" + String(DataRec.OWappid);
    RequestNow = "/data/2.5/weather?lat=" + String(Config.HomeLatitude, 3) + "&lon=" + String(Config.HomeLongitude, 3) + "&units=metric&APPID=" + String(DataRec.OWappid);
  }
  
  // First get current readings    
  http.setTimeout(HTTP_Timeout);
  http.begin("http://" + Site + RequestNow);
  http.setReuse(DailyForecast == "" ? true : false);
  Ret = http.GET();
  if(Ret == HTTP_CODE_OK) response = http.getString();
#ifdef DEBUG
  WriteLogFile("Get_Forecast() Weather response =\n" + response);
#endif  
  http.end();
  
  if(Ret == HTTP_CODE_OK) 
  {
    // Extract fields
    Forecast = response.substring(response.indexOf(F("main\":"))+7, response.indexOf(F(",\"desc"))-1);
    Text = response.substring(response.indexOf(F("description"))+14, response.indexOf(F(",\"icon"))-1);
    Icon = response.substring(response.indexOf(F("icon\":"))+7, response.indexOf(F("icon\":"))+10);
    OWCity = response.substring(response.indexOf("name\":")+7, response.indexOf("\",\"cod\""));
    OWCity += ", " + response.substring(response.indexOf("country\":")+10, response.indexOf("\",\"sunrise"));
    response = response.substring(response.indexOf("temp\":")+6);
    MinTemp = response.toFloat();
    response = response.substring(response.indexOf("humidity")+10);
    Humidity = response.toInt();
    response = response.substring(response.indexOf("sunrise")+9);
    Sunrise = response.toInt();
    response = response.substring(response.indexOf("sunset")+8);
    Sunset = response.toInt();
    breakTime(myTZ.toLocal(Sunrise), tm1);
    breakTime(myTZ.toLocal(Sunset), tm2);
    sprintf(SunUp, "%02d:%02d", tm1.Hour, tm1.Minute);
    sprintf(SunDown, "%02d:%02d", tm2.Hour, tm2.Minute);
    CurrentWeather = (City != "" ? City : OWCity) + ", Temp " + String(MinTemp, 1) + "C, Humidity " + String(Humidity) + "%, ";
    CurrentWeather += Forecast + " : " + Text;
  }
  else
  {
    WriteLogFile("Failed to get current weather (" + String(Ret) + ") " + http.errorToString(Ret));
#ifdef TIMINGS
    WriteLogFile("Get_Forecast() = " + String(millis() - entry) + " ms");
#endif  
    return(false);
  }

  // If I've already saved my daily forecast, exit now...
  if(DailyWeather) 
  {
#ifdef TIMINGS
    WriteLogFile("Get_Forecast() = " + String(millis() - entry) + " ms");
#endif  
    return(true);
  }
    
  // Now get forecast 
  http.setTimeout(HTTP_Timeout);
  http.begin("http://" + Site + Request);
  http.setReuse(false);
  Ret = http.GET();
  if(Ret == HTTP_CODE_OK) response = http.getString();
#ifdef DEBUG
  WriteLogFile("Get_Forecast() Forecast response =\n" + response);
#endif  
  http.end();
  
  if(Ret == HTTP_CODE_OK) 
  {
    // Extract fields
    response = response.substring(response.indexOf("dt\":"));
    for(int i=0; i<Days; i++)
    {      
      if(response.indexOf("dt\":") != 0) break;
      response = response.substring(response.indexOf("min\":")+5);
      MinTemp = response.toFloat();
      response = response.substring(response.indexOf("max\":")+5);
      MaxTemp = response.toFloat();
      response = response.substring(response.indexOf("humidity")+10);
      if(i != 0) 
      {
        Humidity = response.toInt();
        Forecast = response.substring(response.indexOf("main\":")+7, response.indexOf(",\"desc")-1);
        Text = response.substring(response.indexOf("description")+14, response.indexOf(",\"icon")-1);
        Icon = response.substring(response.indexOf("icon\":")+7, response.indexOf("}],")-1);
      }
      Result += String(day()+i) + ") " + (City != "" ? City : OWCity) + ", Min: " + String(MinTemp, 1) + ", Max: " + String(MaxTemp, 1) + ", Humidity: " + String(Humidity) + "%, ";
      Result += Forecast + " : " + Text;
      if(i == 0) Result += ", Sunrise " + String(SunUp) + " / Sunset " + String(SunDown);
      Result += ", Icon: " + Icon + "\n";
      response = response.substring(response.indexOf("dt\":"));
    }
  }
  else
  {
    WriteLogFile("Failed to get weather forecast (" + String(Ret) + ") " + http.errorToString(Ret));
#ifdef TIMINGS
    WriteLogFile("Get_Forecast() = " + String(millis() - entry) + " ms");
#endif  
    return(false);
  }
  
  // Save my DailyForecast and exit
  DailyForecast = Result;
#ifdef TIMINGS
  WriteLogFile("Get_Forecast() = " + String(millis() - entry) + " ms");
#endif  
  return(true);
}

//
// Save daily weather forecast in Weather.txt file in current month directory
//
bool Save_Weather(void)
{
  char            YearPath[10];
  char            WeatherFile[32];
  String          Line;
  unsigned long   entry = millis();

  // Is the SD card missing?
  if(SDCardMissing) return(false);
  
  // Set Year directory and assign file path for new weather file
  sprintf(YearPath, "/%04d", year());
  sprintf(WeatherFile, "/%04d/%02d/Weather.txt", year(), month());
  
  // Now check if the Year directory exist by trying to create it
  if(!SD.mkdir(YearPath)) 
  {
    WriteLogFile("[Save_Weather] Unable to create year directory " + String(YearPath));
    return(false);
  }

  // Now check if the Month directory exist by trying to create it
  sprintf(YearPath, "/%04d/%02d", year(), month());
  if(!SD.mkdir(YearPath)) 
  {
    WriteLogFile("[Save_Weather] Unable to create month directory " + String(YearPath));
    return(false);
  }

  // Try to open the daily weather file for reading
  File file = SD.open(WeatherFile, FILE_READ);
  if(file)
  {
    // The file exist, check to see if today's forecast is already in it
    while(file.available())
    {
      Line = file.readStringUntil('\n');
      if(Line.substring(0, 3) == DailyForecast.substring(0, 3))
      {
        // Today's weather already saved.  Close file and exit
        file.close();
        return(true);
      }
    }

    // Today's weather not there, close file and re-open for append
    file.close();
    file = SD.open(WeatherFile, FILE_APPEND);
    if(!file)
    {
      WriteLogFile("[Save_Weather] Failed to re-open weather file");
      return(false);
    }
  }
  else
  {
    // File doesn't exist.  Open it for WRITE which will create it.
    file = SD.open(WeatherFile, FILE_WRITE);
    if(!file)
    {
      WriteLogFile("[Save Weather] Failed to create weather file " + String(WeatherFile));
      return(false);
    }
  }

  // Add today's weather forecast
  Line = DailyForecast.substring(0, DailyForecast.indexOf("\n"));
  file.println(Line);
  file.close();
#ifdef TIMINGS
  WriteLogFile("Save_Weather() = " + String(millis() - entry) + " ms");
#endif  
  return(true);
}

//
// Get daily weather forecast from file
//
String Get_Weather(int Year, int Month, int Day)
{
  char            WeatherFile[32];
  String          Line;
  unsigned long   entry = millis();
      
  // Set file name
  sprintf(WeatherFile, "/%04d/%02d/Weather.txt", Year, Month);

  // Try to open the daily weather file for reading
  File file = SD.open(WeatherFile, FILE_READ);
  if(file)
  {
    // The file exist, scan for day requested
    while(file.available())
    {
      Line = file.readStringUntil('\n');
      if(Line.toInt() == Day)
      {
        // Got it.  Close file and return weather line
        file.close();
#ifdef TIMINGS
        WriteLogFile("Get_Weather(" + String(Day) + ") = " + String(millis() - entry) + " ms");
#endif  
        return(Line);
      }
    }
    file.close();
  }
  
  // File doesn't exist or day not found
#ifdef TIMINGS
  WriteLogFile("Get_Weather(" + String(Day) + ") = " + String(millis() - entry) + " ms");
#endif  
  return("");
}


////////////////////////////////////////////////////////////////////////////////////////////////////
//
// GPX FUNCTIONS
//
////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Start a new daily GPX file
//
byte GPX_StartDaily(void)
{
  char            YearPath[10];
  unsigned long   entry = millis();
  
  // Check if the card is still inserted
#ifdef ARDUINO_M5Stack_Core_ESP32
  SDCardMissing = !SD.exists("/Weather");
#else  
  SDCardMissing = !digitalRead(SDCard);
#endif  
  if(SDCardMissing) return(0);
  
  // Set Year directory and assign file path for new GPX file
  sprintf(YearPath, "/%04d", year());
  sprintf(DailyGPXFile, "/%04d/%02d/%02d.gpx", year(), month(), day());
  
  // Now check if the Year directory exist by trying to create it
  if(!SD.mkdir(YearPath)) 
  {
    WriteLogFile("[GPX_StartDaily] Unable to create year directory " + String(YearPath));
    return(0);
  }

  // Now check if the Month directory exist by trying to create it
  sprintf(YearPath, "/%04d/%02d", year(), month());
  if(!SD.mkdir(YearPath)) 
  {
    WriteLogFile("[GPX_StartDaily] Unable to create month directory " + String(YearPath));
    return(0);
  }

  // Try to open the daily GPX file for reading
  File file = SD.open(DailyGPXFile, FILE_READ);
  if(!file)
  {
    // Failed to open for read, create file by opening for write
    file = SD.open(DailyGPXFile, FILE_WRITE);
    if(!file)
    {
      WriteLogFile("Failed to create daily GPX file " + String(DailyGPXFile));
      return(0);
    }

    // Write GPX header info
    WriteLogFile("Creating daily GPX file " + String(DailyGPXFile));
    file.println("<?xml version=\"1.1\" encoding=\"UTF-8\" standalone=\"yes\" ?>");
    file.printf("<gpx version=\"1.1\" creator=\"%s %s\"\n", PGM.c_str(), FIRMWARE);
    file.println("\txmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"");
    file.println("\txmlns=\"http://www.topografix.com/GPX/1/1\" xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">");
    file.println("\t<metadata>");
    file.printf("\t\t<name>GPS log for %s</name>\n", getDate(false));
    file.printf("\t\t<desc>Out riding on the %s</desc>\n", DataRec.Bike);
    file.printf("\t\t<author><name>%s</name></author>\n", DataRec.UserName);
    file.println("\t\t<keywords>Motorcycle, Riding, Trips, Exploring, Having Fun</keywords>");
    file.println("\t</metadata>");

    // Clear daily variables
    DailyDistance = Config.DailyRiding = 0;
    Config.AvgSpeed = Config.AvgCntr = Config.MaxSpeed = 0;
  }
  else
    WriteLogFile("Daily GPX file " + String(DailyGPXFile) + " already exist");
  
  // Close file and save config record
  file.close();
  Config.GPXState = FILE_CREATED;
  Save_Config();
#ifdef TIMINGS  
  WriteLogFile("GPX_StartDaily() = " + String(millis() - entry) + " ms");
#endif  

  // Return current day number
  return(day());
}

//
// End daily track file
//
void GPX_EndDaily(void)
{
  unsigned long   entry = millis();
  
  // Check if the card is still inserted
#ifdef ARDUINO_M5Stack_Core_ESP32
  SDCardMissing = !SD.exists("/Weather");
#else  
  SDCardMissing = !digitalRead(SDCard);
#endif  
  if(SDCardMissing) return;
  
  // If GPX state is already FILE_CLOSED, exit.
  if(Config.GPXState == FILE_CLOSED || strlen(DailyGPXFile) == 0) return;

  // Open file for append
  File file = SD.open(DailyGPXFile, FILE_APPEND);
  if(!file)
  {
    WriteLogFile("[GPX_CloseFile] Error opening file " + String(DailyGPXFile));
    return;
  }

  // If GPX state is in TRACK_SEGMENT, close current track segment
  if(Config.GPXState == TRACK_SEGMENT) file.println("\t</trkseg></trk>");

  // Write final GPX entry in file
  file.println("</gpx>");
  file.close();
  Config.GPXState = FILE_CLOSED;

  // Save config record
  Save_Config();
  GPXDay = 0;
#ifdef TIMINGS  
  WriteLogFile("GPX_EndDaily() = " + String(millis() - entry) + " ms");
#endif  
  WriteLogFile("Closed daily GPX file " + String(DailyGPXFile));
}

//
// Return file info for GPX file
//
bool GPX_FileInfo(int Year, int Month, int Day, double *D, int *Sg, int *T, int *St, unsigned long *R, unsigned long *E, char *STime, char *ETime, double *AvgSpeed, double *MaxSpeed, String *StopStr)
{
  char            GPXFile[32], buf[32];
  double          Meters = 0.00;
  double          Lat, Lon, L_Lat, L_Lon, S_Lat, S_Lon, B_Lat, B_Lon, Speed;
  double          Avg = 0;
  double          Max = 0;
  unsigned long   S_Time, St_Time, E_Time;
  unsigned long   Riding = 0;
  unsigned long   entry = millis();
  int             Segments = 0;
  int             Tracks = 0;
  int             Stops = 0;
  int             i;
  int             Cntr = 0;
  byte            FileState;
  String          StartTime, EndTime, Line, Type, Str, Tmp;
        
  // Open file for reading
  sprintf(GPXFile, "/%04d/%02d/%02d.gpx", Year, Month, Day);
  File file = SD.open(GPXFile, FILE_READ);
  if(!file) return(false);

  // Read line by line 
  Lat = Lon = L_Lat = L_Lon = 0.00;
  S_Time = St_Time = E_Time = 0;
  StartTime = EndTime = Str = "";
  FileState = FILE_CREATED;
  sprintf(buf, "&Day=%04d-%02d-%02d", Year, Month, Day);
  while(file.available())
  {
    Type = "";
    Line = file.readStringUntil('\n');
    if(Line.indexOf("<trk>") != -1) 
    { 
      // A new segment started
      ++Segments; 
      i = Line.indexOf("started at");
      S_Time = Line.substring(i+11).toInt() * OneHour;
      S_Time += Line.substring(i+14).toInt() * OneMinute;
      S_Time += Line.substring(i+17).toInt() * OneSecond;
      if(StartTime == "") { St_Time = S_Time; StartTime = Line.substring(i+11, Line.indexOf("</name>")); }

      // Read next line and extract Lat/Lon from first entry to save beginning of segment
      Line = file.readStringUntil('\n');
      Line = Line.substring(Line.indexOf("lat=") + 5);
      B_Lat = (double)Line.toFloat();
      Line = Line.substring(Line.indexOf("lon=") + 5);
      B_Lon = (double)Line.toFloat();
      Tmp = "/Start&Time=" + StartTime + String(buf) + "&Location=" + String(B_Lat, 7) + "," + String(B_Lon, 7);
      if(Str == "") Str = "<a href=\"" + Tmp + "\">Start&nbsp</a>&nbsp&nbsp ";      
    } 
    if(Line.indexOf("<trkpt lat") != -1) { ++Tracks; FileState = TRACK_SEGMENT; }
    if(Line.indexOf("<wpt lat") != -1)
    {
      // Extract lat/lon from waypoint entry
      Line = Line.substring(Line.indexOf("lat=") + 5);
      S_Lat = (double)Line.toFloat();
      Line = Line.substring(Line.indexOf("lon=") + 5);
      S_Lon = (double)Line.toFloat();
      FileState = TRACK_WAYPOINT;
    }
    if(Line.indexOf("<name>No motion") != -1) 
    {
      ++Stops;
      Type = "Stop";
      i = Line.indexOf("No motion");
      EndTime = Line.substring(i+13, Line.indexOf("</name>"));      
      E_Time = Line.substring(i+13).toInt() * OneHour;
      E_Time += Line.substring(i+16).toInt() * OneMinute;
      E_Time += Line.substring(i+19).toInt() * OneSecond;
      Riding += (E_Time - S_Time);

      // Add to return string for stops
      Tmp = "/" + Type + "=" + String(Stops) + "&Time=" + EndTime + String(buf) + "&Location=" + String(S_Lat, 7) + "," + String(S_Lon, 7);
      Str += "<a href=\"" + Tmp + "\">" + Type + "&nbsp" + String(Stops) + "</a>&nbsp&nbsp ";
    }
    i = Line.indexOf("<distance>");
    if(i != -1) Meters += Line.substring(i+10).toFloat();
    i = Line.indexOf("<speed>");
    if(i != -1)
    {
      // Calc AvgSpeed and MaxSpeed
      Speed = Line.substring(i+7).toFloat();
      if(Speed != 0) { Avg += Speed; ++Cntr; }
      if(Max < Speed) Max = Speed;
    }
    if(Line.indexOf("</gpx>") != -1) FileState = FILE_CLOSED;

    // If no distance in line (TrackSolid file), calculate distance
    // This file type is no longer used.  Left here so that I can still view old GPX files
    if(Line.indexOf("<trkpt") != -1 && Line.indexOf("<distance>") == -1)
    {
      // Extract lat/lon from line
      Line = Line.substring(Line.indexOf("lat=") + 5);
      Lat = (double)Line.toFloat();
      Line = Line.substring(Line.indexOf("lon=") + 5);
      Lon = (double)Line.toFloat();

      // Calculate distance from last position
      if(L_Lat != 0.00 && L_Lon != 0.00) Meters += TinyGPSPlus::distanceBetween(Lat, Lon, L_Lat, L_Lon);
      L_Lat = Lat;
      L_Lon = Lon;
    }
  }
  
  // Close file and return values
  file.close();
  *St = (Stops >= 2 ? Stops - 1 : Stops);
  *D = Meters / 1000.0;
  *Sg = Segments;
  *T = Tracks;
  *E = (E_Time - St_Time) / 1000;
  *R = (Riding / 1000) - (Stops * (MotionDelay / 1000));
  strcpy(STime, StartTime.c_str());
  strcpy(ETime, EndTime.c_str());
  if(Cntr != 0) *AvgSpeed = Avg / Cntr; else *AvgSpeed = 0;
  *MaxSpeed = Max;

  // Change last stop text to be "End"
  Str.replace("Stop&nbsp" + String(*St >= 2 ? *St+1 : *St), "End");
  Str.replace("Stop=" + String(*St >= 2 ? *St+1 : *St), "End");
  *StopStr = Str;

  // If request was for today, update config record
  if(Year == year() && Month == month() && Day == day())
  {
    Config.DailyRiding = *R * 1000;
    DailyDistance = *D * 1000.0;
    Config.AvgSpeed = *AvgSpeed;
    Config.AvgCntr = 1;
    Config.MaxSpeed = *MaxSpeed;
    Config.GPXState = FileState;
  }
#ifdef TIMINGS  
  WriteLogFile("GPX_FileInfo() = " + String(millis() - entry) + " ms");
#endif  
  return(true);
}

//
// Polyline encode a segment number
//
String GPX_EncodeSegment(int Year, int Month, int Day, int Tracks)
{
  PolylineEncoder encoder;
  char            GPXFile[32];
  double          T_Lat, T_Lon, W_Lat, W_Lon, Meters;
  unsigned long   entry = millis();
  int             Segments = 0;
  int             Points = 0;
  int             Stop = 0;
  float           Skip, Count;
  String          StartEnd, Stops, LastStop, Result, Marker;
  
  // Start by clearing the polyline
  encoder.clear();
  Result = StartEnd = Stops = LastStop = "";
  T_Lat = T_Lon = W_Lat = W_Lon = 0.00;
    
  // Open file for reading
  sprintf(GPXFile, "/%04d/%02d/%02d.gpx", Year, Month, Day);
  File file = SD.open(GPXFile, FILE_READ);
  if(!file) return("");

  // Calculate skip factor.  This is to end up with an encoded track that Google Map can handle.
  // The encoded string must be less than 8K which represents about 1800 encoded points
  if(Tracks != 0) Skip = 1800.0 / Tracks; else Skip = 1.0;
  Count = 0.00;

  // Read line by line 
  while(file.available())
  {
    String Line = file.readStringUntil('\n');
    if(Line.indexOf("<trk>") != -1) ++Segments;
    if(Line.indexOf("<trkpt lat") != -1)
    {
      // Extract latitude and longitude from string
      Line = Line.substring(Line.indexOf("lat=") + 5);
      T_Lat = (double)Line.toFloat();
      Line = Line.substring(Line.indexOf("lon=") + 5);
      T_Lon = (double)Line.toFloat();

      // Add point
      if(Skip >= 1.0 || Count == 0.00 || Count >= 1.0) { encoder.addPoint(T_Lat, T_Lon); ++ Points; Count -= 1.0; }
      Count += Skip;
      if(Points == 1) StartEnd = "&markers=color:green|label:B|" + String(T_Lat, 7) + "," + String(T_Lon, 7);
    }
    if(Line.indexOf("<wpt lat") != -1)
    {
      // Extract LAT/LOG
      Line = Line.substring(Line.indexOf("lat=") + 5);
      W_Lat = (double)Line.toFloat();
      Line = Line.substring(Line.indexOf("lon=") + 5);
      W_Lon = (double)Line.toFloat();      
    }
    if(Line.indexOf("<name>No motion") != -1) 
    {
      // Only add if the LastStop is not empty.  This will prevent the very last
      // "No motion" waypoint from being added to the encoding.
      if(++Stop < 10) Marker = String(Stop); else Marker = String((char)('A' + (Stop - 10)));
      if(LastStop.length() != 0) Stops += LastStop;
      LastStop = "&markers=color:yellow|size:mid|label:" + Marker + "|" + String(W_Lat, 7) + "," + String(W_Lon, 7);
    }

    // Next two lines are for GPSBabel GPX encoded files
    // No longer used but I want to be able to read and display old files.
    if(Line.indexOf("<name>start point") != -1) StartEnd = "&markers=color:green|label:B|" + String(W_Lat, 7) + "," + String(W_Lon, 7);
    if(Line.indexOf("<name>end point") != -1) StartEnd += "&markers=color:red|label:E|" + String(W_Lat, 7) + "," + String(W_Lon, 7);
  }

  // Add last point if needed
  if(Skip < 1.0 && Points > 1 && Count < 1.0) { encoder.addPoint(T_Lat, T_Lon); ++Points; }
  
  // Close file and return values
  file.close();
  StartEnd += "&markers=color:red|label:E|" + String(T_Lat, 7) + "," + String(T_Lon, 7);
  if(Points != 0) Result = StartEnd + Stops + "&path=color:blue|weight:5|enc:" + encoder.Result();
#ifdef DEBUG  
  WriteLogFile("Encoded length (%d points / %d)" + String(Points) + String(Result.length()));
#endif  
#ifdef TIMINGS  
  WriteLogFile("GPX_EncodeSegment() = " + String(millis() - entry) + " ms");
#endif  
  return(Result);
}

//
// Daily scanning of monthly trips
//
void GPX_DailyScan(int Year, int Month)
{
  char            RideFile[32], GPXFile[32], StartTime[32], EndTime[32];
  bool            GotData;
  double          Meters, AvgSpeed, MaxSpeed;
  float           C_Gas, C_Lodging, C_Maintenance, C_Other;
  int             Day, Segments, Tracks, Stops;
  unsigned long   Riding, Elapsed, entry;
  String          StopStr;
    
  // Check if the card is still inserted
#ifdef ARDUINO_M5Stack_Core_ESP32
  SDCardMissing = !SD.exists("/Weather");
#else  
  SDCardMissing = !digitalRead(SDCard);
#endif  
  if(SDCardMissing) return;

  // Read daily ride file to get last day recorded
  entry = millis();
  Day = 0;
  sprintf(RideFile, "/%04d/%02d/Rides.txt", Year, Month);
  File file = SD.open(RideFile, FILE_READ);
  if(file) 
  {
    // Read line by line 
    while(file.available())
    {
      String Line = file.readStringUntil('\n');
      Day = Line.toInt();
    }
    file.close();
  }

  // Increment Day by one and start scanning from there
  ++Day;
  WriteLogFile("Starting daily scan on day " + String(Day));
  
  // Open file for append (will be created if it doesn't exist)
  file = SD.open(RideFile, FILE_APPEND);
  if(!file)
  {
    WriteLogFile("[GPX_DailyScan] Error opening file " + String(RideFile));
    return;
  }

  // Now scan each day to get trip info and write to monthly file
  for(int i=Day; i<=day(); i++)
  {
    if(GPX_FileInfo(Year, Month, i, &Meters, &Segments, &Tracks, &Stops, &Riding, &Elapsed, StartTime, EndTime, &AvgSpeed, &MaxSpeed, &StopStr))
    {
      // Only write to file if I have more than MinDailyDistance travelled
      if(Meters >= DataRec.MinDailyDistance)
      {
        file.printf("%02d: ", i);
        file.print(Meters, 1);
        file.printf(" Km, %d Segments, %d Tracks, %d Stops, %d Riding\n", Segments, Tracks, Stops, Riding);
        
        // Commit values to configuration record
        Config.Mileage += (Meters * 1000.0);
        Config.TripDistance += (Meters * 1000.0); 
        Config.TripRiding += Riding;
        Config.SeasonDistance += (Meters * 1000.0); 
        Config.SeasonRiding += Riding;
      }
      else
      {
        // GPX file has less than MinDailyDistance in it, delete GPX file
        sprintf(GPXFile, "/%04d/%02d/%02d.gpx", Year, Month, i);        
        if(strcmp(GPXFile, DailyGPXFile) == 0) strcpy(DailyGPXFile, "");
        if(SD.remove(GPXFile)) 
          WriteLogFile("Removed GPX file " + String(GPXFile) + " with only " + String(Meters * 1000.0) + " m and " + String(UpTime(Riding*1000, false)) + " of riding.");
        else
          WriteLogFile("Failed to remove GPX file " + String(GPXFile));

        // Now make sure the text, media and expense file are removed as well
        sprintf(GPXFile, "/%04d/%02d/%02d_Text.txt", Year, Month, i);
        if(SD.remove(GPXFile)) WriteLogFile("Removed text file...");
        sprintf(GPXFile, "/%04d/%02d/%02d_Media.txt", Year, Month, i);
        if(SD.remove(GPXFile)) WriteLogFile("Removed media file...");

        // Get daily expenses and remove them from trip and season
        Get_Expenses(Year, Month, i, &C_Gas, &C_Lodging, &C_Maintenance, &C_Other);
        Config.TripExpenses -= (C_Gas + C_Lodging + C_Maintenance + C_Other);
        Config.SeasonExpenses -= (C_Gas + C_Lodging + C_Maintenance + C_Other);
        sprintf(GPXFile, "/%04d/%02d/%02d_Exp.txt", Year, Month, i);
        if(SD.remove(GPXFile)) WriteLogFile("Removed expenses file...");

        // Only commit mileage to config record as I still want to maintain proper values
        // for overall mileage, season mileage and season riding time
        Config.Mileage += (Meters * 1000.0);        
        Config.TripDistance += (Meters * 1000.0); 
        Config.SeasonDistance += (Meters * 1000.0); 
        Config.SeasonRiding += Riding;
      }
    }
  }

  // Close file
  file.close();
  Save_Config();
  WriteLogFile("Daily scan took " + String(millis() - entry) + " ms");
}

//
// Start a new track segment
//
void GPX_StartTrack(void)
{
  unsigned long   entry = millis();
  
  // Check if the card is still inserted
#ifdef ARDUINO_M5Stack_Core_ESP32
  SDCardMissing = !SD.exists("/Weather");
#else  
  SDCardMissing = !digitalRead(SDCard);
#endif  
  if(SDCardMissing) return;
  
  // If GPX state is already FILE_CLOSED, exit.
  if(Config.GPXState == FILE_CLOSED) return;

  // Open file for append
  File file = SD.open(DailyGPXFile, FILE_APPEND);
  if(!file)
  {
    WriteLogFile("[GPX_StartTrack] Error opening file " + String(DailyGPXFile));
    return;
  }

  // Start new track segment
  file.printf("\t<trk><name>Segment started at %s</name>", getTime(false));
  file.print("<location>" + City + "/" + CityCode + "</location>");
  file.print("<trkseg>\n");
  file.close();
  Config.GPXState = TRACK_SEGMENT;
  
  // Save config record
  Save_Config();
#ifdef TIMINGS  
  WriteLogFile("GPX_StartTrack() = " + String(millis() - entry) + " ms");
#endif  
}

//
// Write a segment track point
//
void GPX_TrackPoint(void)
{
  unsigned long   entry = millis();
  
  // Check if the card is still inserted
#ifdef ARDUINO_M5Stack_Core_ESP32
  SDCardMissing = !SD.exists("/Weather");
#else  
  SDCardMissing = !digitalRead(SDCard);
#endif  
  if(SDCardMissing) return;
  
  // Make sure Latitude and Longitude are set
  if(Latitude == 0.00 || Longitude == 0.00)
  {
    WriteLogFile("[GPX_TrackPoint] Attempt to write invalid track point!");
    return;
  }

  // If Distance is zero and Speed is non-zero, consider it GPS noise and ignore
  if(Distance == 0 && Speed != 0) return;
  
  // Create new file if needed
  if(GPXDay == 0) GPXDay = GPX_StartDaily();
  
  // Start track segment if needed
  if(Config.GPXState != TRACK_SEGMENT) GPX_StartTrack();
  
  // Open file for append
  File file = SD.open(DailyGPXFile, FILE_APPEND);
  if(!file)
  {
    WriteLogFile("[GPX_TrackPoint] Error opening file " + String(DailyGPXFile));
    return;
  }

  // Write track segment
  file.print("\t\t<trkpt lat=\""); file.print(Latitude, 7); 
  file.print("\" lon=\""); file.print(Longitude, 7);
  file.print("\"><distance>");
  file.print(Distance, 1);
  file.printf("</distance><speed>%s</speed></trkpt>\n", String(Speed, 1).c_str());
  file.close();
  
  // Save config record if needed.
  if(Config.GPXState != TRACK_SEGMENT)
  {
    Config.GPXState = TRACK_SEGMENT;
    Save_Config();  
  }
#ifdef TIMINGS  
  WriteLogFile("GPX_TrackPoint() = " + String(millis() - entry) + " ms");
#endif  
}

//
// Write a waypoint
//
void GPX_WayPoint(String Title)
{
  unsigned long   entry = millis();
  
  // Check if the card is still inserted
#ifdef _M5STACK_H_
  SDCardMissing = !SD.exists("/Weather");
#else  
  SDCardMissing = !digitalRead(SDCard);
#endif  
  if(SDCardMissing) return;
  
  // Make sure Latitude and Longitude are set
  if(Latitude == 0.00 || Longitude == 0.00)
  {
    WriteLogFile("[GPX_WayPoint] Attempt to write invalid waypoint!");
    return;
  }
  
  // If GPX state is already FILE_CLOSED, exit.
  if(Config.GPXState == FILE_CLOSED) return;

  // Open file for append
  File file = SD.open(DailyGPXFile, FILE_APPEND);
  if(!file)
  {
    WriteLogFile("[GPX_WayPoint] Error opening file " + String(DailyGPXFile));
    return;
  }

  // If GPX state is in TRACK_SEGMENT, close current track segment to write waypoint
  if(Config.GPXState == TRACK_SEGMENT) file.println("\t</trkseg></trk>");

  // Write waypoint
  file.print("\t<wpt lat=\""); file.print(Latitude, 7); 
  file.print("\" lon=\""); file.print(Longitude, 7); file.println("\">");
  file.printf("\t\t<name>%s at %s</name>\n", Title.c_str(), getTime(false));
  file.print("\t\t<location>" + City + "/" + CityCode + "</location>\n");
  file.print("\t</wpt>\n");
  file.close();
  Config.GPXState = TRACK_WAYPOINT;  
  
  // Save config record
  Save_Config();
#ifdef TIMINGS  
  WriteLogFile("GPX_Waypoint() = " + String(millis() - entry) + " ms");
#endif  
  WriteLogFile("Waypoint \"" + Title + "\" added");
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// HELPER FUNCTIONS
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ARDUINO_M5Stack_Core_ESP32
//
// Blink builtin LED
//
void Blink_LED(int count, bool fast)
{
  for(int i=0; i<count; i++) 
  { 
    digitalWrite(LED_BUILTIN, LED_ON); delay(fast ? 125 : 200); 
    digitalWrite(LED_BUILTIN, LED_OFF); delay(fast ? 125 : 200);
  }
}

//
// Beep buzzer
//
void Buzzer_Beep(int count, bool Error)
{
  if(!Error) 
  { 
    digitalWrite(Buzzer, HIGH); delay(150); 
    digitalWrite(Buzzer, LOW); delay(150); 
  }
  digitalWrite(Buzzer, HIGH); delay(750); 
  digitalWrite(Buzzer, LOW); delay(250); 

  for(int i=0; i<count; i++) 
  { 
    digitalWrite(Buzzer, HIGH); delay(150); 
    digitalWrite(Buzzer, LOW); delay(150); 
  }
  if(Error) CurrentTime = millis();
}
#endif

//
// Return daily expenses
//
void Get_Expenses(int Year, int Month, int Day, float *G, float *L, float *M, float *O)
{
  char    MediaFile[32];
  float   C_Gas, C_Lodging, C_Maintenance, C_Other;
  int     i;
  String  Line;
  
  // Init variables
  C_Gas = C_Lodging = C_Maintenance = 0; C_Other = 0.00;

  // Check if I have a daily file
  sprintf(MediaFile, "/%04d/%02d/%02d_Exp.txt", Year, Month, Day);
  File file = SD.open(MediaFile, FILE_READ);
  if(file) 
  {
    // Yup there is.  Read file extracting values
    while(file.available())
    {
      Line = file.readStringUntil('\n');
      i = Line.indexOf("Gas = "); if(i != -1) C_Gas = Line.substring(i+6).toFloat();
      i = Line.indexOf("Lodging = "); if(i != -1) C_Lodging = Line.substring(i+10).toFloat();
      i = Line.indexOf("Maintenance = "); if(i != -1) C_Maintenance = Line.substring(i+14).toFloat();
      i = Line.indexOf("Other = "); if(i != -1) C_Other = Line.substring(i+8).toFloat();
    }
    file.close();
  }

  // Return values
  *G = C_Gas;
  *L = C_Lodging;
  *M = C_Maintenance;
  *O = C_Other;
}

//
// Initialize OTA callback functions
//
void  InitOTA(void)
{
  // Set the hostname and OTA update password
  if(strlen(DataRec.HostName) != 0) ArduinoOTA.setHostname(DataRec.HostName); else ArduinoOTA.setHostname(PGM.c_str());
  if(strlen(DataRec.OTA_Pass) != 0) ArduinoOTA.setPassword(DataRec.OTA_Pass); else ArduinoOTA.setPassword("admin");

  // Create OTA call backs
  ArduinoOTA.onStart([]() 
  { 
    OTA_Update = true;
    WebServer.stop();
    WriteLogFile("OTA update starting...");
    Save_Config();
    DweetPost(-3);
#ifdef ARDUINO_M5Stack_Core_ESP32
    Beep();
    if(!DisplayOn) { M5.Lcd.wakeup(); M5.Lcd.setBrightness(200); }
    M5.Lcd.fillScreen(TFT_BLUE);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
    M5.Lcd.drawRoundRect(40, 105, 240, 30, 8, TFT_GREEN);
    M5.Lcd.drawRoundRect(39, 104, 242, 32, 8, TFT_GREEN);
    M5.Lcd.drawRoundRect(38, 103, 244, 34, 8, TFT_GREEN);
    M5.Lcd.setTextSize(1);
    M5.Lcd.drawCentreString("OTA Update", 160, 75, 4);    
#endif    
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) 
  { 
    int   p = (progress / (total / 100));
    static int lp;
    char  buf[40];

#ifdef ARDUINO_M5Stack_Core_ESP32
    // Fill the progress bar
    M5.Lcd.fillRoundRect(38, 103, 244 * (((float)p) / 100.0), 34, 8, TFT_GREEN);
    sprintf(buf, "%d%%", p);
    M5.Lcd.drawCentreString(buf, 160, 140, 4);
#endif

    // Write to logfile every 10% done
    if(p % 10 == 0 && p != lp)
    {
      lp = p;
      WriteLogFile(String(p) + "% done...");
    }
  });
  ArduinoOTA.onEnd([]() 
  { 
#ifdef ARDUINO_M5Stack_Core_ESP32
    M5.Lcd.drawCentreString("Done!", 160, 140, 4);
#endif    
    WriteLogFile("OTA update done!");
    GPS_Port.end();
  });
  ArduinoOTA.onError([](ota_error_t error) 
  {
    OTAError = "OTA error [" + String(error) + "] ";
    if(error == OTA_AUTH_ERROR) OTAError += "Auth Failed";
    else if(error == OTA_BEGIN_ERROR) OTAError += "Begin Failed";
    else if(error == OTA_CONNECT_ERROR) OTAError += "Connect Failed";
    else if(error == OTA_RECEIVE_ERROR) OTAError += "Receive Failed";
    else if(error == OTA_END_ERROR) OTAError += "End Failed";
    else OTAError += "Unknown Error";
    WriteLogFile(OTAError);
    ESP.restart();
  });

  // Start the OTA process
  ArduinoOTA.begin();  
}

//
// Return CPU reset reason
//
#ifdef _ROM_RTC_H_
String Get_Reset_Reason(RESET_REASON reason)
{
  switch(reason)
  {
    case 1:   return("POWERON_RESET");          /**<1, Vbat power on reset*/
    case 2:   return("UNKNOWN");
    case 3:   return("SW_RESET");               /**<3, Software reset digital core*/
    case 4:   return("OWDT_RESET");             /**<4, Legacy watch dog reset digital core*/
    case 5:   return("DEEPSLEEP_RESET");        /**<5, Deep Sleep reset digital core*/
    case 6:   return("SDIO_RESET");             /**<6, Reset by SLC module, reset digital core*/
    case 7:   return("TG0WDT_SYS_RESET");       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8:   return("TG1WDT_SYS_RESET");       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9:   return("RTCWDT_SYS_RESET");       /**<9, RTC Watch dog Reset digital core*/
    case 10:  return("INTRUSION_RESET");        /**<10, Instrusion tested to reset CPU*/
    case 11:  return("TGWDT_CPU_RESET");        /**<11, Time Group reset CPU*/
    case 12:  return("SW_CPU_RESET");           /**<12, Software reset CPU*/
    case 13:  return("RTCWDT_CPU_RESET");       /**<13, RTC Watch dog Reset CPU*/
    case 14:  return("EXT_CPU_RESET");          /**<14, for APP CPU, reseted by PRO CPU*/
    case 15:  return("RTCWDT_BROWN_OUT_RESET"); /**<15, Reset when the vdd voltage is not stable*/
    case 16:  return("RTCWDT_RTC_RESET");       /**<16, RTC Watch dog reset digital core and rtc module*/
    default:  return("UNKNOWN");
  }
}
#endif

//
// Put ESP32 into deep sleep
//
void PowerOff(void)
{
  // ESP32 into deep sleep
  WriteLogFile("Going into deep sleep...", (timeStatus() == timeNotSet ? false : true));
#ifdef ARDUINO_M5Stack_Core_ESP32
  M5.Lcd.setBrightness(0); 
  M5.Lcd.sleep();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_A_PIN, LOW);
  while(digitalRead(BUTTON_A_PIN) == LOW) { delay(10); }
#else  
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON, LOW);
  while(digitalRead(BUTTON) == LOW) { delay(10); }
#endif  
  esp_deep_sleep_start();
  WriteLogFile("Deep sleep failed!");
#ifdef ARDUINO_M5Stack_Core_ESP32
  M5.Lcd.wakeup();  
  M5.Lcd.setBrightness(200); 
#endif  
}

//
// Save updated config record to EEPROM
//
void Save_Config(void)
{
/*  
  char * data = (char *)&Config;
  for(int i=0; i<sizeof(Config); i++)
  {
    char c = data[i];
    if(c < 16) Serial.print("0");
    Serial.print(c, HEX); Serial.print(" ");
    if(i % 26 == 0 && i != 0) Serial.println();
  }
  Serial.println();  
*/
//Serial.print("C = "); Serial.print(Config.CRC, HEX); Serial.print(" / ");
  Config.CRC = CRC16((char *)&Config, CONFIG_CRC);  
//Serial.print(Config.CRC, HEX); Serial.print(" / ");  
  EEPROM.put(CONFIG_OFFSET, Config);
//unsigned short CRC = CRC16((char *)&Config, CONFIG_CRC);
//Serial.println(CRC, HEX);
  if(!EEPROM.commit()) WriteLogFile("Error writing Config EEPROM memory!");
}

//
// Save updated data record to EEPROM
//
void Save_Data(void)
{
/*
  char * data = (char *)&DataRec;
  for(int i=0; i<sizeof(DataRec); i++)
  {
    char c = data[i];
    if(c < 16) Serial.print("0");
    Serial.print(c, HEX); Serial.print(" ");
    if(i % 26 == 0 && i != 0) Serial.println();
  }
  Serial.println();  
*/
//Serial.print("D = "); Serial.print(DataRec.CRC, HEX); Serial.print(" / ");
  DataRec.CRC = CRC16((char *)&DataRec, DATA_CRC);  
//Serial.print(DataRec.CRC, HEX); Serial.print(" / ");  
  EEPROM.put(DATA_OFFSET, DataRec);
//unsigned short CRC = CRC16((char *)&DataRec, DATA_CRC);
//Serial.println(CRC, HEX);
  if(!EEPROM.commit()) WriteLogFile("Error writing DataRec to EEPROM memory!");
}

//
// Read device configuration from /Config.txt on SD Card
//
bool Read_Config_File(void)
{
  bool            Okay = true;
  int             i;
  String          Line, Item, Value;
  unsigned long   entry = millis();

  // Create list of valid items that can be in the file
  static const char* Items[] = 
  { "USERNAME", "MOTORCYCLE", "MILEAGE", "OILCHANGE", "DWEETNAME", "HOMELATITUDE", "HOMELONGITUDE",
  "HOMERADIUS", "CELLWIFI", "CELLPASS", "HOMEWIFI", "HOMEPASS", "HOSTNAME", "HOSTPASS", "DAILYSHUTDOWN",
  "MINDAILYDISTANCE", "OWAPPID", "GOOGLEAPI", "LIGHTBOX" };
  static byte Size[] = { 32, 32, 0, 0, 20, 0, 0, 0, 20, 20, 20, 20, 20, 20, 0, 0, 64, 64, 0 } ;
  
  // Check if the configuration file exist
  File file = SD.open("/Config.txt", FILE_READ);
  if(!file) return(false);

  // Clear DataRec and set incorrect values for bool fields
  memset(&DataRec, NULL, sizeof(DataRec));
  DataRec.DailyShutdown = DataRec.LightBox = 0xF0;  
  
  // It exist, read line by line
  WriteLogFile("Processing /Config.txt file...", false);
  while(file.available())
  {
    Line = file.readStringUntil('\n');
#ifdef DEBUG    
    Serial.print("Line = '" + Line + "'\n");
#endif    

    // If the line does not contain an '=' or the line is a comment, skip it
    if(Line.indexOf("=") == -1 || Line.indexOf("//") == 0) continue;

    // Extract item and value from line
    Item = Line.substring(0, Line.indexOf("="));
    Item.toUpperCase();
    Value = Line.substring(Line.indexOf("=")+1);
    Value.replace('\n', 0x00); Value.replace('\r', 0x00);
#ifdef DEBUG
    Serial.print("Item = '" + Item + "', Value = '" + Value + "'\n");
#endif

    // Now place in proper field of DataRec or Config record
    for(i=0; i<sizeof(Size); i++) if(Item == Items[i]) break;
    if(i == sizeof(Size))
    {
      // Item not found in list...  Exit with error.
      WriteLogFile("Invalid item '" + Item + "' with value of '" + Value + "'", false);
      file.close();
      return(false);    
    }

    // Make sure Value length is good
    if(Value.length() > Size[i] && Size[i] != 0)
    {
      // Invalid size for Value...  Exit with error.
      WriteLogFile("Invalid size for item '" + Item + "' with value of '" + Value + "', Len = " + String(Value.length()) + ", Max = " + String(Size[i]), false);
      file.close();
      return(false);      
    }

    // Place in appropriate variable
    switch(i+1)
    {
      case 1: // UserName
        strcpy(DataRec.UserName, Value.c_str());
        break;
      case 2: // Motorcycle
        strcpy(DataRec.Bike, Value.c_str());
        break;
      case 3: // Mileage
        Config.Mileage = (unsigned long)(Value.toFloat() * 1000.0);
        break;
      case 4: // Oil Change
        Config.OilChange = (unsigned long)(Value.toFloat() * 1000.0);
        break;
      case 5: // DweetName
        strcpy(DataRec.DweetName, Value.c_str());
        break;
      case 6: // Home Latitude
        Config.HomeLatitude = (double)Value.toFloat();
        break;
      case 7: // Home Longitude
        Config.HomeLongitude = (double)Value.toFloat();
        break;
      case 8: // Home Radius
        Config.HomeRadius = (unsigned int)Value.toInt();
        break;
      case 9: // Cell Wifi
        strcpy(DataRec.SSID1, Value.c_str());
        break;
      case 10: // Cell Password
        strcpy(DataRec.PASS1, Value.c_str());
        break;
      case 11: // Home WiFi
        strcpy(DataRec.SSID2, Value.c_str());
        break;
      case 12: // Home password
        strcpy(DataRec.PASS2, Value.c_str());
        break;
      case 13: // HostName
        strcpy(DataRec.HostName, Value.c_str());
        break;
      case 14: // OTA Password
        strcpy(DataRec.OTA_Pass, Value.c_str());
        break;
      case 15: // DailyShutdown
        Value.toUpperCase();
        if(Value == "YES" || Value == "ON" || Value == "TRUE") DataRec.DailyShutdown = true;
        else if(Value == "NO" || Value == "OFF" || Value == "FALSE") DataRec.DailyShutdown = false;
        else Okay = false;
        break;
      case 16: // Minimum Daily Distance
        DataRec.MinDailyDistance = (unsigned int)Value.toInt();
        if(DataRec.MinDailyDistance > 50) Okay = false;
        break;
      case 17: // OpenWeatherMap.org credentials
        strcpy(DataRec.OWappid, Value.c_str());
        break;
      case 18: // Google API key
        strcpy(DataRec.GoogleAPI, Value.c_str());
        break;
      case 19: // LightBox
        Value.toUpperCase();
        if(Value == "YES" || Value == "ON" || Value == "TRUE") DataRec.LightBox = true;
        else if(Value == "NO" || Value == "OFF" || Value == "FALSE") DataRec.LightBox = false;
        else Okay = false;
        break;
    }
  }

  // Done reading file, make sure I have my mandatory fields
  file.close();
  if(strlen(DataRec.SSID1) == 0) Okay = false;
  if(strlen(DataRec.PASS1) == 0) Okay = false;
  if(strlen(DataRec.OWappid) == 0) Okay = false;
  if(strlen(DataRec.GoogleAPI) == 0) Okay = false;
  if(Config.HomeLatitude == 0) Okay = false;    
  if(Config.HomeLongitude == 0) Okay = false;    

  // Default some values not set
  if(strlen(DataRec.HostName) == 0) strcpy(DataRec.HostName, "M5-Tracker");
  if(strlen(DataRec.OTA_Pass) == 0) strcpy(DataRec.OTA_Pass, "admin");
  if(strlen(DataRec.SSID2) == 0) strcpy(DataRec.SSID2, DataRec.SSID1);
  if(strlen(DataRec.PASS2) == 0) strcpy(DataRec.PASS2, DataRec.PASS1);
  if(Config.HomeRadius == 0) Config.HomeRadius = 25;
  if(DataRec.MinDailyDistance == 0) DataRec.MinDailyDistance = 5;
  if(DataRec.DailyShutdown == 0xF0) DataRec.DailyShutdown = true;
  if(DataRec.LightBox == 0xF0) DataRec.LightBox = true;

  // Were there any mandatory fields not set?
  if(!Okay) 
  {
    WriteLogFile("One of the mandatory fields not set.\nCheck your /Config.txt file!", false); 
    return(false); 
  }
  else
  {
    // Everything set, rename the config.txt file
    WriteLogFile("All fields processed properly.\n", false);
    SD.rename("/Config.txt", "/Config.bak");
    Save_Config(); 
    DataRec.Init = DEVICETYPE;
    Save_Data();
  }

  // Return result
  return(Okay);
}

//
// Encode URL
//
String URLEncode(String URL)
{
  String Encoded = URL;
  Encoded.replace(" ", "+");
  Encoded.replace("(", "%28");
  Encoded.replace(")", "%29");
  Encoded.replace("°", "%C2%B0");
  return Encoded;
}

//
// Decode URL
//
String URLDecode(String str)
{    
  String encodedString="";
  char c;
  char code0;
  char code1;
 
  for (int i =0; i < str.length(); i++)
  {
    c=str.charAt(i);
    if (c == '+')
    {
      encodedString+=' ';  
    }
    else if (c == '%') 
    {
      i++;
      code0=str.charAt(i);
      i++;
      code1=str.charAt(i);
      c = (h2int(code0) << 4) | h2int(code1);
      encodedString+=c;
    } 
    else
      encodedString+=c;  
  }
  return encodedString;
}

unsigned char h2int(char c)
{
  if(c >= '0' && c <= '9') return((unsigned char)c - '0');
  if(c >= 'a' && c <= 'f') return((unsigned char)c - 'a' + 10);
  if(c >= 'A' && c <= 'F') return((unsigned char)c - 'A' + 10);
  return(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// DWEET.IO FUNCTION
//
////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Post position to dweet.io
//
void DweetPost(int StatusSpeed)
{
  int           ret;
  String        Time, Data, response;
  String        Spd = StatusSpeed >= 0 ? (StatusSpeed == 0 ? "Stopped" : (String((int)StatusSpeed) + " Km%2Fh")) : (StatusSpeed == -1 ? "Not Tracking" : "Deep Sleep");
  unsigned long entry = millis();

  // Do I have WiFi
  if(!WiFiConnected || strlen(DataRec.DweetName) == 0) return;

#ifdef ARDUINO_M5Stack_Core_ESP32
  StatusLine("Dweet Post");
  Hide_Buttons();
#endif
  
  // Is this an OTA update dweet?
  if(StatusSpeed == -3) Spd = "OTA Update";
  
  // Get time
  Time = String(getTime(false));

  // Build data string to get
  Data = "/dweet/for/" + String(DataRec.DweetName) + 
    "?Latitude=" +  String(DistanceHome <= Config.HomeRadius ? Config.HomeLatitude : (StatusSpeed <= 0 ? LastLAT : Latitude), 6) + 
    "&Longitude=" + String(DistanceHome <= Config.HomeRadius ? Config.HomeLongitude : (StatusSpeed <= 0 ? LastLON : Longitude), 6) + 
    "&Speed=" + Spd + "&Home=";
  if(DistanceHome <= Config.HomeRadius)
    Data += "0 ";
  else
  {
    if(DistanceHome >= 1000) 
      Data += String(DistanceHome / 1000.0) + " K";
    else 
      Data += String((unsigned int)DistanceHome) + " ";  
  }
  Data += "m&Daily=";
  if(DailyDistance >= 1000) 
    Data += String(DailyDistance / 1000.0) + " K";
  else 
    Data += String((unsigned int)DailyDistance) + " ";  
  Data += "m&TimeStamp=" + Time;
  Data += "&IP=" + WiFi.localIP().toString();
  
  // Post data to dweet.io
  Data = URLEncode(Data);
  Data = "http://dweet.io" + Data;
  http.setReuse(Speed > 0 ? true : false);
  http.setTimeout(HTTP_Timeout);
  http.begin(Data);
  ret = http.GET();
  if(ret == HTTP_CODE_OK) response = http.getString();
  http.end();

  // Was it posted properly?
  if(response.indexOf("succeeded") == -1 || ret != HTTP_CODE_OK) 
    WriteLogFile("No success on DweetPost (" + String(ret) + ") " + http.errorToString(ret));
#ifdef TIMINGS
  WriteLogFile("DweetPost() = " + String(millis() - entry) + " ms");
#endif  
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// WEBSITE FUNCTIONS
//
////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Process a web client request
//
void  ProcessClient(void)
{
  char        MediaFile[32];
  bool        Icon = false;
  bool        Post = false;
  bool        Video = false;
  bool        Image = false;
  bool        GPX = false;
  bool        Text = false;
  int         PageType, Day, Month, Year, i;
  float       Gas, Lodging, Maintenance, Other, C_Gas, C_Lodging, C_Maintenance, C_Other;
  double      S_Lat, S_Lon;
  String      Val, Title, Location, CC, Line = "";

  // Do I have a client to process?
  if(!WebServer.hasClient()) return;
  WebClient = WebServer.available();
  unsigned long entry = millis();

#ifdef ARDUINO_M5Stack_Core_ESP32
  if(!DisplayOn) { M5.Lcd.wakeup(); TextDisplay = false; UI(); M5.Lcd.setBrightness(200); }
  DisplayOn = true;
  DisplayTime = millis();
  StatusLine("Web Client");
  Hide_Buttons();
#endif
  
  // Init day, month and year
  Month = month();
  Year = year();
  Day = day();
  PageType = 1;
  
  // Process client
  while(WebClient.connected())
  {
    // Check for rogue client.  Don't know where these comes from...
    if(millis() - entry >= 30000)
    {
      WriteLogFile("Rogue web client... IP = " + WebClient.remoteIP().toString());
      break;
    }
    if(WebClient.available()) 
    {
      Line = WebClient.readStringUntil('\n');
#ifdef DEBUG      
      WriteLogFile(Line);
#endif      
      if(Line == "\r")
      {
        // Is browser asking for a media file?
        if(Icon || Video || Image || GPX || Text) { SendMedia(WebClient, Val); break; }
        
        // Send the HTML header information
        WebPageHeader(WebClient, PageType);

        // Send HTML body start
        if(PageType == 1 && !Post)
        {
          if(strlen(DataRec.DweetName) != 0)
            WebClient.printf("<body onload=\"ClearForecast()\"><center><H2><a href=\"https://dweet.io/follow/%s\" target=\"_blank\">%s</a></H2></center>", DataRec.DweetName, DataRec.DweetName);
          else
            WebClient.printf("<body onload=\"ClearForecast()\"><center><H2>%s</H2></center>", PGM.c_str());
        }
        else
          WebClient.print("<body>");
        if(SDCardMissing && PageType == 1 && !Post) WebClient.println("<center><font color=\"#FF0000\"><b><h1>SD Card Missing!</h1></b><font color=\"#000000\"></center><br>");
      
        // Was this a POST command?
        if(Post)
        {
          // Now save data sent back from POST
          PageType = 2;          
          String Msg = WebClient.readStringUntil('\n');
#ifdef DEBUG
          WriteLogFile("POST DATA = " + Msg);
#endif          

          // Was this a NOTES or MEDIA update?
          if(Msg.indexOf("Notes=") != -1 || Msg.indexOf("Media=") != -1)
          {
            sprintf(MediaFile, "/%04d/%02d/%02d_%s.txt", Year, Month, Day, Msg.indexOf("Notes=") != -1 ? "Text" : "Media");
            Msg = URLDecode(Msg);
            Msg = Msg.substring(6);
  
            // Am I deleting the media file?
            if(Msg.length() == 0)
            {
              SD.remove(MediaFile);
              WriteLogFile("File " + String(MediaFile) + " removed...");
            }
            else
            {
              // Save the new media file data
              File file = SD.open(MediaFile, FILE_WRITE);
              if(file) 
              {
                file.print(Msg);
                file.close();
              }
              else
                WriteLogFile("[ProcessClient] Error opening file " + String(MediaFile));
            }
          }

          // Was this a EXPENSES update?
          if(Msg.indexOf("GAS=") != -1 && Msg.indexOf("C_Gas=") != -1)
          {
            // Init fields and file name
            Msg.toUpperCase();
            Gas = Lodging = Maintenance = Other = C_Gas = C_Lodging = C_Maintenance = C_Other = 0.00;
            sprintf(MediaFile, "/%04d/%02d/%02d_Exp.txt", Year, Month, Day);

            // Extract field from data string
            i = Msg.indexOf("GAS="); if(i != -1) Gas = Msg.substring(i+4).toFloat();
            i = Msg.indexOf("LODGING="); if(i != -1) Lodging = Msg.substring(i+8).toFloat();
            i = Msg.indexOf("MAINTENANCE="); if(i != -1) Maintenance = Msg.substring(i+12).toFloat();
            i = Msg.indexOf("OTHER="); if(i != -1) Other = Msg.substring(i+6).toFloat();
            i = Msg.indexOf("C_GAS="); if(i != -1) C_Gas = Msg.substring(i+6).toFloat();
            i = Msg.indexOf("C_LODGING="); if(i != -1) C_Lodging = Msg.substring(i+10).toFloat();
            i = Msg.indexOf("C_MAINTENANCE="); if(i != -1) C_Maintenance = Msg.substring(i+14).toFloat();
            i = Msg.indexOf("C_OTHER="); if(i != -1) C_Other = Msg.substring(i+8).toFloat();

            // Do I have anything to update?
            if(Gas + Lodging + Maintenance + Other + C_Gas + C_Lodging + C_Maintenance + C_Other != 0.00)
            {
              // Open file for WRITE (will re-create file) and write new values
              File file = SD.open(MediaFile, FILE_WRITE);
              if(file) 
              {
                file.print("Gas = " + String(Gas + C_Gas, 2) + "\n");
                file.print("Lodging = " + String(Lodging + C_Lodging, 2) + "\n");
                file.print("Maintenance = " + String(Maintenance + C_Maintenance, 2) + "\n");
                file.print("Other = " + String(Other + C_Other, 2) + "\n");
                file.close();
  
                // Update Trip and season expense fields if it's for the current year
                if(Year == year())
                {
                  Config.TripExpenses += (Gas + Lodging + Maintenance + Other);
                  Config.SeasonExpenses += (Gas + Lodging + Maintenance + Other);
                  Save_Config();
                }
#ifdef WEB_REQUEST
                WriteLogFile("Updated daily expenses for " + String(MediaFile));
#endif              
              }
              else
                WriteLogFile("[ProcessClient] Error opening file " + String(MediaFile));
            }
          }
        }

        // Send requested page type
        switch(PageType)
        {
          case 1: 
            // Refresh weather forecast and send monthly calendar
            if(ShowNMEA) WebClient.print("<center><b><h3>NMEA Messages On Serial Port</h3></b></center>");
            if(CurrentWeather != "") WebClient.printf("<center>%s</center>", CurrentWeather.c_str());
            SendCalendar(WebClient, Year, Month);
            break;
          case 2: // View ride
          case 3: // Edit ride notes
          case 4: // Edit ride media
            DailyRide(WebClient, Year, Month, Day, PageType);
            break;  
          case 5: // Dump log file
            WebClient.printf("<center><h2><b>%s Log File</b></h2></center>", PGM.c_str());
            DumpLogFile(WebClient);
            break;
          case 6: // New road trip
            Config.TripStart = now();
            Config.TripDistance = 0;
            Config.TripRiding = 0;
            Config.TripAvgSpeed = 0;
            Config.TripAvgCntr = 0;
            Config.TripMaxSpeed = 0;
            Config.TripExpenses = 0.00;
            Save_Config();
            WriteLogFile("New road trip started...");
            WebClient.print("<center><h2><font color=\"#FF0000\"><b>New Road Trip Started!</b><font color=\"#000000\"></h2></center>");
            SendCalendar(WebClient, Year, Month);
            break;            
          case 7: // New riding season
            Config.SeasonStart = now();
            Config.SeasonDistance = 0;
            Config.SeasonRiding = 0;
            Config.SeasonAvgSpeed = 0;
            Config.SeasonAvgCntr = 0;
            Config.SeasonMaxSpeed = 0;
            Config.SeasonExpenses = 0.00;
            Save_Config();
            WriteLogFile("New riding season started...");
            WebClient.print("<center><h2><font color=\"#FF0000\"><b>New Riding Season Started!</b><font color=\"#000000\"></h2></center>");
            SendCalendar(WebClient, Year, Month);
            break; 
          case 8: // ESP32 Tracker Statistics
            SendStatistics(WebClient, Year, Month);
            break;   
          case 9:  // Send route stops map
            StopLocation(WebClient, Year, Month, Day, Title, Location);
            break;
          case 10: // Send daily expense form
            DailyExpenses(WebClient, Year, Month, Day);
            break;
        }

        // Send web page bottom info
        WebClient.printf("<hr><center>Created by Claude G. Beaudoin<br>");
        WebClient.printf("%s / %s<br>%s / Firmware %s<br>UpTime: %s</center><br><br></body></html>", getDate(false), getTime(false), ARDUINO_BOARD, FIRMWARE, UpTime(millis(), false));
        WebClient.println();
        WebClient.println();
        break;
      } 
      else 
      {
        // Check what the client GET request was for
        if(Line.indexOf("GET") != -1) 
        {
          Val = Line.substring(0, Line.indexOf("HTTP")-1);
#ifdef WEB_REQUEST          
          WriteLogFile("WebRequest() = " + Val);
#endif          
          Line.toUpperCase();
          if(Line.indexOf("/FAVICON.ICO") != -1) Icon = true;
          if(Line.indexOf(".TXT") != -1 || Line.indexOf(".BAK") != -1) Text = true;
          if(Line.indexOf(".HTM") != -1) Text = true;
          if(Line.indexOf(".GPX") != -1) GPX = true;
          if(Line.indexOf(".MP4") != -1) Video = true;
          if(Line.indexOf(".PGN") != -1 || Line.indexOf(".JPG") != -1) Image = true;
          if(Line.indexOf("MONTH=") != -1)
          {
            // Extract month and year
            Month = Line.substring(11,13).toInt();
            Year = Line.substring(19,23).toInt();
            PageType = 1;
          }
          if(Line.indexOf("DAY=") != -1)
          {
            // Extract year, month and day
            Year = Line.substring(9,13).toInt();
            Month = Line.substring(14,16).toInt();
            Day = Line.substring(17,19).toInt();
            PageType = 2;
          }
          if(Line.indexOf("DATEPICKER?DATE=") != -1)
          {
            if(Line.indexOf("-") != -1)
            {
              // Extract month and year
              Year = Line.substring(21,25).toInt();
              Month = Line.substring(26,28).toInt();
            }
            PageType = 1;
          }
          if(Line.indexOf("NOTES=") != -1)
          {
            // Extract year, month and day
            Year = Line.substring(11,15).toInt();
            Month = Line.substring(16,18).toInt();
            Day = Line.substring(19,21).toInt();
            PageType = 3;
          }
          if(Line.indexOf("MEDIA=") != -1)
          {
            // Extract year, month and day
            Year = Line.substring(11,15).toInt();
            Month = Line.substring(16,18).toInt();
            Day = Line.substring(19,21).toInt();
            PageType = 4;
          }
          if(Line.indexOf("EXPEN=") != -1)
          {
            // Extract year, month and day
            Year = Line.substring(11,15).toInt();
            Month = Line.substring(16,18).toInt();
            Day = Line.substring(19,21).toInt();
            PageType = 10;
          }
          if(Line.indexOf("/START") != -1 || Line.indexOf("/STOP") != -1 || Line.indexOf("/END") != -1)
          { 
            // Extract fields and get location of stop
            Year = Line.substring(Line.indexOf("DAY=")+4).toInt();
            Month = Line.substring(Line.indexOf("DAY=")+9).toInt();
            Day = Line.substring(Line.indexOf("DAY=")+12).toInt();
            S_Lat = (double)Line.substring(Line.indexOf("LOCATION=")+9, Line.indexOf(",")).toFloat();
            S_Lon = (double)Line.substring(Line.indexOf(",")+1, Line.indexOf("HTTP")-1).toFloat();
            Get_Location(S_Lat, S_Lon, &Location, &CC);
            if(Line.indexOf("/START") != -1) Title = "Start";
            if(Line.indexOf("/STOP") != -1) Title = "Stop " + Line.substring(Line.indexOf("STOP=")+5, Line.indexOf("&TIME")); 
            if(Line.indexOf("/END") != -1) Title = "End";
            if(Line.indexOf("&TIME=") != -1) Title += " in " + Location + " at " + Line.substring(Line.indexOf("&TIME=")+6, Line.indexOf("&DAY"));                  
            Location = Line.substring(Line.indexOf("LOCATION=")+9, Line.indexOf("HTTP")-1);
            PageType = 9;
          }
          if(Line.indexOf("/STATS") != -1) 
          {
            Year = Line.substring(Line.indexOf("STATS=")+6).toInt();
            Month = Line.substring(Line.indexOf("STATS=")+11).toInt();
            PageType = 8;
          }
          if(Line.indexOf("/LOGFILE") != -1) PageType = 5;
          if(Line.indexOf("/NEWTRIP") != -1) PageType = 6;
          if(Line.indexOf("/NEWSEASON") != -1) PageType = 7;
          if(Line.indexOf("/SHOWNMEA") != -1) { ShowNMEA = !ShowNMEA; PageType = 1; }
          if(Line.indexOf("/OILCHANGE") != -1) 
          {
            Config.OilChange = Config.Mileage;
            Save_Config();
            PageType = 8;
          }
          if(Line.indexOf("/CLEARLOG") != -1) 
          {
            // Erase log file
            if(!SDCardMissing) 
            {
              SD.remove("/LogFile.txt");
              WriteLogFile(String(getDate(false)) + " log file was erased!");
              PageType = 5;
            }
          }
          if(Line.indexOf("/DELEXP=") != -1) 
          {
            // Erase expense file
            if(!SDCardMissing) 
            {
              // Extract year, month and day
              Year = Line.substring(12,16).toInt();
              Month = Line.substring(17,19).toInt();
              Day = Line.substring(20,22).toInt();
              Other = Line.substring(23).toFloat();
              sprintf(MediaFile, "/%04d/%02d/%02d_Exp.txt", Year, Month, Day);
              SD.remove(MediaFile);

              // Remove expenses from trip and season expenses if it's for a day/month in this year.
              PageType = 2;
              if(Year == year()) 
              { 
                Config.TripExpenses -= Other; 
                Config.SeasonExpenses -= Other; 
                Save_Config();
#ifdef WEB_REQUEST          
                WriteLogFile("Expense file " + String(MediaFile) + " erased!");
#endif              
              }
            }
          }
        }

        // Was this a POST method?
        if(Line.indexOf("POST") != -1) 
        {
          Val = Line.substring(0, Line.indexOf("HTTP")-1);
#ifdef WEB_REQUEST          
          WriteLogFile("WebRequest() = " + Val);
#endif          
          Year = Line.substring(12,16).toInt();
          Month = Line.substring(17,19).toInt();
          Day = Line.substring(20,22).toInt();
          Post = true;
        }
      }
    }
    else
    {
      // No data available from remote.  Disconnect after 5 seconds
      if(millis() - entry > (5 * OneSecond))
      {
        WriteLogFile("Silent web client... IP = " + WebClient.remoteIP().toString());
        break;
      }
    }
  };

  // Disconnect client
  WebClient.flush();
  delay(500);
  WebClient.stop();
  WebClient = NULL;
#ifdef WEB_REQUEST  
  WriteLogFile("WebRequest() = " + String(millis()-entry) + " ms");
#endif  
}

//
// Send back media stored on the SD card
//
void SendMedia(WiFiClient WebClient, String MediaFile)
{
  // Increasing the bufsize value may start to cause crashes because of stack overflow  
  // They will appear as;
  // Guru Meditation Error: Core  1 panic'ed (Unhandled debug exception)
  // Debug exception reason: Stack canary watchpoint triggered (loopTask) 
  // This is an Espressif issue that's still open.
  // See: https://github.com/espressif/arduino-esp32/issues/1260
  // So keep bufsize at only 1024 for now...
  
  int   bufsize = 1024;   
  byte  clientBuf[bufsize];
  int   readSize;
  char  *itemBytes = reinterpret_cast<char*>(&clientBuf);
  unsigned long entry, totalSize;
  String Media;

  // Open request file and send data
  if(SDCardMissing) return;
  Media = MediaFile.substring(4);
  File file = SD.open(Media, FILE_READ);
  if(file)
  {
    // Send response header
    String FileBytes = " KB)";
    double FileSize = file.size() / 1024;
    if(FileSize > 1024) { FileSize = FileSize / 1024; FileBytes = " MB)"; }
    if(FileSize > 1024) { FileSize = FileSize / 1024; FileBytes = " GB)"; }
#ifdef WEB_REQUEST    
    WriteLogFile("Sending media file '" + Media + "' (" + String(FileSize, 0) + FileBytes);
#endif    
    WebClient.println("HTTP/1.1 200 OK");
    WebClient.print("Content-type: ");
    if(MediaFile.indexOf(".txt") != -1) WebClient.println("text/html");
    if(MediaFile.indexOf(".htm") != -1) WebClient.println("text/html");
    if(MediaFile.indexOf(".gpx") != -1) WebClient.println("application/gpx+xml");
    if(MediaFile.indexOf(".mp4") != -1) WebClient.println("video/mp4");
    if(MediaFile.indexOf(".jpg") != -1) WebClient.println("image/jpeg");
    if(MediaFile.indexOf(".png") != -1) WebClient.println("image/png");
    if(MediaFile.indexOf(".ico") != -1) WebClient.println("image/x-icon");
    WebClient.print("Content-Length: "); WebClient.println(file.size());
    WebClient.println("Connection: close");
    WebClient.println();

    // Send content
    entry = millis();
    totalSize = 0;
    while(file.available())
    {
      readSize = file.readBytes(itemBytes, bufsize);
      WebClient.write(clientBuf, readSize);
      totalSize += readSize;
    }
    WebClient.println();
    
    // Close file
    file.close();      
#ifdef TIMINGS
    WriteLogFile("SendMedia() = " + String((millis() - entry)) + " ms");
#endif    
  } 
  else 
  {
    // File does not exist!  Send back a page not found error.
#ifdef WEB_REQUEST    
//    if(Media != "/favicon.ico" ) 
    WriteLogFile("Failed to open file \"" + Media + "\"");
#endif    
    String HTML = "<!DOCTYPE html><html><head></head><body>Not Found</body></html>";
    WebClient.println("HTTP/1.1 404 Not Found");
    WebClient.println("Content-type: text/html");
    WebClient.printf("Content-Length: %d\n", HTML.length());
    WebClient.println("Connection: close");
    WebClient.println();
    WebClient.println(HTML);
    WebClient.println();  
#ifdef TIMINGS
    WriteLogFile("SendMedia() = " + String((millis() - entry)) + " ms");
#endif    
  }
}

//
// Send the web page header information to client
//
void  WebPageHeader(WiFiClient WebClient, int PageType)
{
  // Send response header
  WebClient.println("HTTP/1.1 200 OK");
  WebClient.println("Content-type: text/html");
  WebClient.println("Cache-Control: no-cache");
  WebClient.println("Connection: close");
  WebClient.println();
  WebClient.print("<!DOCTYPE html>\n<html lang=\"en-US\">\n<head><title>ESP32 Tracker</title>\n");
  WebClient.println("<META http-equiv=\"Content-Type\" content=\"text/html;charset=ISO-8859-1\">\n<META charset=\"utf-8\">");
//  if(PageType == 8) WebClient.println("<META HTTP-EQUIV=\"refresh\" CONTENT=\"15\">"); // Auto refresh page every 5 seconds
  WebClient.println("<link rel=\"icon\" href=\"/favicon.ico\" type=\"image/x-icon\">");
  WebClient.println("<style type=\"text/css\"> body { color:#000000; background-color:#C0C0C0; } a  { color:#0000FF; } a:visited { color:#800080; } a:hover { color:#008000; } a:active { color:#FF0000; }");

  if(DataRec.LightBox)
  {
    // Insert lightbox style sheet
    WebClient.println("* { box-sizing: border-box; }");
    WebClient.println(".row > .column { padding: 2px 2px; }");
    WebClient.println(".row:after { content: \"\"; display: table; clear: both; }");
    WebClient.println(".column { float: left; width: 100%; }");
    WebClient.println(".modal { display: none; position: fixed; z-index: 1; padding-top: 30px; left: 0; top: 0; width: 100%; height: 100%; overflow: auto; background-color: black; font-family: Verdana, sans-serif; margin: 0; } ");
    WebClient.println(".modal-content { position: relative; background-color: #fefefe; margin: auto; padding: 0; width: 90%; max-width: 960px; }");
    WebClient.println(".close { color: white; position: absolute; top: 10px; right: 25px; font-size: 35px; font-weight: bold; }");
    WebClient.println(".close:hover, .close:focus { color: #999; text-decoration: none; cursor: pointer; }");
    WebClient.println(".mySlides { display: none; }");
    WebClient.println(".cursor { cursor: pointer; }");
    WebClient.println(".prev, .next { cursor: pointer; position: absolute; top: 50%; width: auto; padding: 16px; margin-top: -50px; color: white; font-weight: bold; font-size: 20px; transition: 0.6s ease; border-radius: 0 3px 3px 0; user-select: none; -webkit-user-select: none; }");
    WebClient.println(".next { right: 0; border-radius: 3px 0 0 3px; }");
    WebClient.println(".prev:hover, .next:hover { background-color: rgba(0, 0, 0, 0.8); }");
    WebClient.println(".numbertext { color: #f2f2f2; font-size: 12px; padding: 8px 12px; position: absolute; top: 0; }");
    WebClient.println(".caption-container { text-align: center; background-color: black; padding: 2px 16px; color: white; }");
    WebClient.println(".demo { opacity: 0.5; }");
    WebClient.println(".active, .demo:hover { opacity: 1; }");
    WebClient.println(".hover-shadow:hover { box-shadow: 0 5px 20px 0 rgba(0, 0, 0, 1.0), 0 10px 30px 0 rgba(0, 0, 0, 1.0); }");
    WebClient.println("img { margin-bottom: -4px; }");
    WebClient.println("img.hover-shadow { transition: 0.3s; padding: 3px; }");
  }
  else
  {
    // Insert popup style sheet
    WebClient.println("img.hover-shadow { border-radius: 5px; cursor: pointer; transition: 0.3s; padding: 3px; }");
    WebClient.println(".hover-shadow:hover { box-shadow: 0 5px 20px 0 rgba(0, 0, 0, 1.0), 0 10px 30px 0 rgba(0, 0, 0, 1.0); }");
    WebClient.println(".modal { display: none; position: fixed; z-index: 1; padding-top: 60px; left: 0; top: 0; width: 100%; height: 100%; overflow: auto; background-color: rgb(0,0,0); background-color: rgba(0,0,0,1.0); }");
    WebClient.println(".modal-content { margin: auto; display: block; width: 80%; max-width: 960px; }");
    WebClient.println(".modal-content, #caption { -webkit-animation-name: zoom; -webkit-animation-duration: 0.6s; animation-name: zoom; animation-duration: 0.6s; }");
    WebClient.println(".close { position: absolute; top: 15px; right: 35px; color: #f1f1f1; font-size: 40px; font-weight: bold; transition: 0.3s; }");
    WebClient.println(".close:hover, .close:focus { color: #bbb; text-decoration: none; cursor: pointer; }");
    WebClient.println("#caption { margin: auto; display: block; width: 80%; max-width: 960px; text-align: center; color: #ccc; padding: 10px 0; height: 150px; font-family: Verdana, sans-serif; }");
    WebClient.println("@-webkit-keyframes zoom { from {-webkit-transform:scale(0)} to {-webkit-transform:scale(1)} }");
    WebClient.println("@keyframes zoom { from {transform:scale(0)} to {transform:scale(1)} }");
    WebClient.println("@media only screen and (max-width: 700px) { .modal-content { width: 100%; } }");
  }
  WebClient.println("</style>");      
  
  // Insert Java scripts used by ESP32 Tracker
  WebClient.println("<script>");
  WebClient.println("function DayForecast(text) { document.getElementById(\"Forecast\").innerHTML = text; }");
  WebClient.println("function ClearForecast() { document.getElementById(\"Forecast\").innerHTML = \"&nbsp\"; }");
  WebClient.println("function goBack() { window.history.back(); }");
  WebClient.println("function myFunction(Value) { window.location.href=Value; }");

  if(DataRec.LightBox)
  {
    // Insert lightbox java scripts
    WebClient.println("var slideIndex = 1;");
    WebClient.println("showSlides(slideIndex);");
    WebClient.println("function openModal() { document.getElementById(\"myModal\").style.display = \"block\"; }");
    WebClient.println("function closeModal() { document.getElementById(\"myModal\").style.display = \"none\"; }");
    WebClient.println("function plusSlides(n) { showSlides(slideIndex += n); }");
    WebClient.println("function currentSlide(n) { showSlides(slideIndex = n); }");
    WebClient.println("function showSlides(n) {");
    WebClient.println("var i;");
    WebClient.println("var slides = document.getElementsByClassName(\"mySlides\");");
    WebClient.println("var dots = document.getElementsByClassName(\"demo\");");
    WebClient.println("var captionText = document.getElementById(\"caption\");");
    WebClient.println("if (n > slides.length) {slideIndex = 1}");
    WebClient.println("if (n < 1) {slideIndex = slides.length}");
    WebClient.println("for (i = 0; i < slides.length; i++) { slides[i].style.display = \"none\"; }");
    WebClient.println("for (i = 0; i < dots.length; i++) { dots[i].className = dots[i].className.replace(\" active\", \"\"); }");
    WebClient.println("slides[slideIndex-1].style.display = \"block\";");
    WebClient.println("dots[slideIndex-1].className += \" active\";");
    WebClient.println("captionText.innerHTML = dots[slideIndex-1].alt; }");    
  }
  WebClient.println("</script></head>");
}

//
// Send monthly calender
//
void  SendCalendar(WiFiClient WebClient, int Year, int Month)
{
  char          GPXFile[32], MediaFile[32], StartTime[32], EndTime[32];
  bool          GotData, Media, Expenses;
  int           i, j, FileDay, Segments, Tracks, Stops;
  double        Meters, MonthlyMeters, AvgSpeed, MaxSpeed;
  unsigned long Riding, Elapsed, MonthlyRiding;
  String        Title, Line, Icon, ForecastText, DayLine, Location, StopStr;
  time_t        t;
  TimeElements  tm;

  // Send page top
  Title = String(monthStr(Month)) + " - " + String(Year);
  WebClient.print("<center><table border=\"1\" width=\"90%\"<tr>");
  WebClient.printf("<td align=\"center\"><a href=\"/Month=%02d,Year=%d\">Previous</a></td>", (Month - 1 < 1 ? 12 : Month - 1), (Month - 1 < 1 ? Year - 1 : Year));
  WebClient.printf("<td colspan=\"3\" align=\"center\"><b><font color=\"#000000\" size=\"+2\">%s<font color=\"#000000\" size=\"-2\"></b></td>", Title.c_str());
  WebClient.printf("<td colspan=\"2\" align=\"center\"><form action=\"DatePicker\"><input type=\"month\" name=\"Date\">&nbsp<input type=\"submit\">&nbsp&nbsp&nbsp<a href=\"/Stats=%04d-%02d\">Stats</a></td>", Year, Month);

  if(Month == month() && Year == year())
    WebClient.print("<td>&nbsp</td></tr><tr>");
  else
    WebClient.printf("<td align=\"center\"><a href=\"/Month=%02d,Year=%d\">Next</a></td></tr><tr>", (Month + 1 > 12 ? 1 : Month + 1), (Month + 1 > 12 ? Year + 1 : Year));

  // Print days
  for(i=0; i<7; i++) WebClient.printf("<td align=\"center\" width=\"10%%\">%s</td>", dayStr(i+1));
  WebClient.println("</tr><tr>");

  // Build a time_t for the month requested
  memset(&tm, NULL, sizeof(tm));
  tm.Year = Year - 1970;
  tm.Month = Month;
  tm.Day = 1;
  t = makeTime(tm);
  breakTime(t, tm);
  MonthlyMeters = 0.0;
  MonthlyRiding = 0;
  
  // Open monthly ride file
// TODO: Open monthly weather file as well  
  sprintf(GPXFile, "/%04d/%02d/Rides.txt", Year, Month);
  File file = SD.open(GPXFile, FILE_READ);
  if(file)
  {
    // Read first line and extract day from it
    Line = file.readStringUntil('\n');
    FileDay = Line.toInt();        
  } else FileDay = 0;
  
  // Print first day of the month spanning columns if needed
  for(i=1, j=1; i<32; i++, j++)
  {
    Meters = 0.0; Riding = 0;
    if((FileDay == 0 && !file) || (Year == year() && Month == month() && i == day()))
    {
      if(!SDCardMissing)
        GotData = GPX_FileInfo(Year, Month, i, &Meters, &Segments, &Tracks, &Stops, &Riding, &Elapsed, StartTime, EndTime, &AvgSpeed, &MaxSpeed, &StopStr);
      else GotData = false;
    }
          
    // Span columns if first day of the month is not a Sunday
    if(i == 1 && tm.Wday != 1)
    {
      WebClient.printf("<td align=\"right\" colspan=\"%d\"></td>", tm.Wday-1); 
      j = tm.Wday; 
    }

    // If I have a forecast, get fields from it
    Icon = ""; ForecastText = ""; DayLine = "";
    if(Year == year() && Month == month() && i >= day()) 
    {
      // Get DayLine from current weather forecast string
      DayLine = DailyForecast.substring(DailyForecast.indexOf(String(i)+")"));
      DayLine = DayLine.substring(0, DayLine.indexOf("\n"));
    } 
    else if(!SDCardMissing) DayLine = Get_Weather(Year, Month, i);
    if(DayLine != "") 
    {
      DayLine = DayLine.substring(0, DayLine.indexOf("\n"));
      Icon = DayLine.substring(DayLine.indexOf("Icon:")+6, DayLine.indexOf("Icon:")+9);
      ForecastText = DayLine.substring(DayLine.indexOf(")")+2, DayLine.indexOf("Icon")-2);
      Location = DayLine.substring(DayLine.indexOf(")")+2, DayLine.indexOf("Min")-2);
    } 

    // Do I have any media files for today?
    sprintf(MediaFile, "/%04d/%02d/%02d_Media.txt", Year, Month, i);
    if(!SDCardMissing) Media = SD.exists(MediaFile); else Media = false;
    if(!Media)
    {
      // No media file, what about a text file?
      sprintf(MediaFile, "/%04d/%02d/%02d_Text.txt", Year, Month, i);
      if(!SDCardMissing) Media = SD.exists(MediaFile); else Media = false;
    }

    // Check for expenses to change day number colour
    sprintf(MediaFile, "/%04d/%02d/%02d_Exp.txt", Year, Month, i);
    if(!SDCardMissing) Expenses = SD.exists(MediaFile); else Expenses = false;

    // Insert weather icon if I have one
    if(Icon == "")
      WebClient.printf("<td align=\"right\"><b><font color=\"%s\">%d<font color=\"#000000\"></b>", 
        (i == day() && Month == month() && Year == year() ? "#FF0000" : (Expenses ? "#00FFFF" : Media ? "#FFFF00" : "#000000")), i);
    else
      WebClient.printf("<td align=\"right\"><img onmouseover=\"DayForecast('%s')\" onmouseout=\"ClearForecast()\" src=\"https://openweathermap.org/img/w/%s.png\" style=\"float:left;width:50px;height:50px\" alt=\"Weather\"></a><b><font color=\"%s\">%d&nbsp<font color=\"#000000\"></b>",
        ForecastText.c_str(), Icon.c_str(), (i == day() && Month == month() && Year == year() ? "#FF0000" : (Expenses ? "#00FFFF" : Media ? "#FFFF00" : "#000000")), i);
    
    // If I have a FileDay, extract info from it and read next line
    if(FileDay != 0 && FileDay == i && Line != "")
    {
      Line = Line.substring(Line.indexOf(":")+1);
      Meters = (unsigned long)Line.toFloat();
      if(Line.indexOf("Stops,") != -1) Riding = (unsigned long)Line.substring(Line.indexOf("Stops,")+7).toInt();
      GotData = true;
      Line = file.readStringUntil('\n');
      FileDay = Line.toInt();          
    }
    if(GotData && Meters != 0.0)
    {
      MonthlyMeters += Meters;
      MonthlyRiding += Riding;
      WebClient.printf("<b><p><a href=\"/Day=%d-%02d-%02d\">", Year, Month, i);
      if(Meters >= 1.00) 
        Icon = String(Meters, 1) + "&nbspKm";
      else
        Icon = String(Meters * 1000.0) + "&nbspm";
      WebClient.printf("%s</a></p></b></td>", Icon.c_str());
      GotData = false;
    } else WebClient.print("<p>&nbsp</p></td>");
    if(j % 7 == 0) WebClient.println("</tr><tr>");
    t += SECS_PER_DAY;
    breakTime(t, tm);
    if(tm.Month != Month) break;
  }
  if(j % 7 != 0) WebClient.printf("<td align=\"right\" colspan=\"%d\"></td></tr>", 7 - (j % 7)); 
  WebClient.println("</table></center><br>");
  WebClient.print("<center><b>Monthly Mileage = " + String(MonthlyMeters, 1) + " Km");
  if(MonthlyRiding != 0) WebClient.print(", Riding Time = " + String(UpTime(MonthlyRiding*1000, false)));
  WebClient.print("</b><p id=\"Forecast\"></p></center>");  

  // Close file if opened
  if(file) file.close();
}

//
// Send Daily Expenses form
//
void  DailyExpenses(WiFiClient WebClient, int Year, int Month, int Day)
{
  char          MediaFile[32], buf[50], buf1[50];
  int           i;
  String        Line;
  float         C_Gas, C_Lodging, C_Maintenance, C_Other;
  time_t        t;
  TimeElements  tm;

  // Build a time_t for the date requested
  memset(&tm, NULL, sizeof(tm));
  tm.Year = Year - 1970;
  tm.Month = Month;
  tm.Day = Day;
  t = makeTime(tm);
  breakTime(t, tm);

  // Check if I have daily expences already logged for that day
  Get_Expenses(Year, Month, Day, &C_Gas, &C_Lodging, &C_Maintenance, &C_Other);
  
  // Send top page info
  WebClient.print("<center><h1><font color=\"#000000\"><b>Daily Expenses</b><font color=\"#000000\"></h1></center>");
  WebClient.printf("<center><b><h2>%s ", dayStr(tm.Wday));
  WebClient.printf("%s-%d, %d</h2></b>", monthStr(Month), Day, Year);
  WebClient.print("<center><table border=\"1\" cellpadding=\"10\"><form method=\"POST\">");

  // Insert input fields and current values
  WebClient.printf("<tr><td align=\"right\">Gas</td><td><input type=\"number\" name=\"GAS\" min=\"0.00\" step=\"0.01\"></td>");
  WebClient.printf("<td align=\"right\"><b>%s</b></td></tr>", String(C_Gas, 2).c_str());
  WebClient.printf("<tr><td align=\"right\">Lodging</td><td><input type=\"number\" name=\"LODGING\" min=\"0.00\" step=\"0.01\"></td>");
  WebClient.printf("<td align=\"right\"><b>%s</b></td></tr>", String(C_Lodging, 2).c_str());
  WebClient.printf("<tr><td align=\"right\">Maintenance</td><td><input type=\"number\" name=\"MAINTENANCE\" min=\"0.00\" step=\"0.01\"></td>");
  WebClient.printf("<td align=\"right\"><b>%s</b></td></tr>", String(C_Maintenance, 2).c_str());
  WebClient.printf("<tr><td align=\"right\">Food / Other</td><td><input type=\"number\" name=\"OTHER\" min=\"0.00\" step=\"0.01\"></td>");
  WebClient.printf("<td align=\"right\"><b>%s</b></td></tr>", String(C_Other, 2).c_str());

  // Insert hidden fields
  WebClient.printf("<input type=\"hidden\" name=\"C_Gas\" value=\"%s\">", String(C_Gas, 2).c_str());
  WebClient.printf("<input type=\"hidden\" name=\"C_Lodging\" value=\"%s\">", String(C_Lodging, 2).c_str());
  WebClient.printf("<input type=\"hidden\" name=\"C_Maintenance\" value=\"%s\">", String(C_Maintenance, 2).c_str());
  WebClient.printf("<input type=\"hidden\" name=\"C_Other\" value=\"%s\">", String(C_Other, 2).c_str());

  // Send bottom page "Submit", "Cancel" and "Delete" buttons
  // TODO: Make the "Delete" button display a confirm dialog as; https://www.w3schools.com/jsref/tryit.asp?filename=tryjsref_confirm3
  WebClient.printf("</table><hr><input type=\"submit\">&nbsp&nbsp<input type=\"button\" onclick=\"goBack()\" value=\"Cancel\">", Year, Month, Day);
  WebClient.printf("&nbsp&nbsp<input type=\"button\" onclick=\"myFunction('/DelExp=%02d-%02d-%02d-%s')\" value=\"Delete\">", Year, Month, Day, String(C_Gas + C_Lodging + C_Maintenance + C_Other, 2).c_str());
  WebClient.println("</form></center><script>function myFunction(Value) { window.location.href=Value; }</script>");    
}

//
// Send ESP32 Tracker Statistics
//
void  SendStatistics(WiFiClient WebClient, int Year, int Month)
{
  char          StartTime[32], EndTime[32], buf[50], buf1[50];
  int           Segments, Tracks, Stops;
  double        Meters, AvgSpeed, MaxSpeed, OilChange;
  unsigned long Riding, Elapsed;
  String        Line, StopStr;
  TimeElements  tm;

  // Send top page info
  WebClient.printf("<center><b><h2>%s ", dayStr(weekday()));
  WebClient.printf("%s-%d, %d</h2></b>", monthStr(month()), day(), year());
  WebClient.print("<center><table border=\"0\" cellpadding=\"10\"><tr>");

  // Day Statistics
  GPX_FileInfo(year(), month(), day(), &Meters, &Segments, &Tracks, &Stops, &Riding, &Elapsed, StartTime, EndTime, &AvgSpeed, &MaxSpeed, &StopStr);
  WebClient.print("<td align=\"center\" valign=\"top\"><table border=\"1\" cellpadding=\"10\">");
  WebClient.print("<tr><td align=\"center\" colspan=\"2\"><b>Daily Statistics</b></td></tr>");
  if(Meters >= 1) 
    Line = String(Meters, 1) + "&nbspKm";
  else
    Line = String(Meters * 1000) + "&nbspm";
  WebClient.printf("<tr><td align=\"right\">Daily Distance</td><td align=\"left\">%s</td></tr>", Line.c_str());
  WebClient.printf("<tr><td align=\"right\">Riding Time</td><td align=\"left\">%s</td></tr>", UpTime(Riding*1000, false));
  WebClient.printf("<tr><td align=\"right\">Avg Speed</td><td align=\"left\">%s&nbspKm/h</td></tr>", String(AvgSpeed, 1).c_str());
  WebClient.printf("<tr><td align=\"right\">Max Speed</td><td align=\"left\">%s&nbspKm/h</td></tr>", String(MaxSpeed, 1).c_str());
  if(DistanceHome >= 1000) 
    Line = String(DistanceHome / 1000.0, 1) + "&nbspKm";
  else
    Line = String(DistanceHome) + "&nbspm";
  WebClient.printf("<tr><td align=\"right\">Distance Home</td><td align=\"left\">%s</td></tr>", Line.c_str());
  if(Config.Mileage + (Meters * 1000.0) >= 1000) 
    Line = String((Config.Mileage + (Meters * 1000.0)) / 1000.0, 1) + "&nbspKm";
  else
    Line = String(Config.Mileage + (Meters * 1000.0)) + "&nbspm";
  WebClient.printf("<tr><td align=\"right\">Total Mileage</td><td align=\"left\">%s</td></tr>", Line.c_str());
  OilChange = OilChangeInterval - (Config.Mileage + (Meters * 1000.0) - Config.OilChange);
  if(OilChange >= 1000) 
    Line = String(OilChange / 1000.0, 1) + "&nbspKm";
  else
  {
    if(OilChange >= 0)
      Line = String(OilChange) + "&nbspm";
    else
    {
      Line = "DUE ";
      if(abs(OilChange) >= 1000)
        Line += String(abs(OilChange) / 1000.0, 1) + " K";
      else
        Line += String(abs(OilChange)) + " ";
      Line += "m ago!";
    }
  }
  WebClient.printf("<tr><td align=\"right\">Oil Change In</td><td align=\"left\">%s</td></tr>", Line.c_str());
  WebClient.print("</table></td>");

  // GPS Info
  WebClient.print("<td align=\"center\" valign=\"top\"><table border=\"1\" cellpadding=\"10\">");
  WebClient.print("<tr><td align=\"center\" colspan=\"2\"><b>GPS Information</b></td></tr>");
  WebClient.printf("<tr><td align=\"right\">GPS Fix</td><td align=\"left\">%s</td></tr>", GPS.location.isValid() ? "True" : "False");
  WebClient.printf("<tr><td align=\"right\">Satellites</td><td align=\"left\">%d</td></tr>", GPS.satellites.value());
  WebClient.print("<tr><td align=\"right\">Latitude</td><td align=\"left\">");
  WebClient.print(Latitude, 7);
  WebClient.print("</td></tr><tr><td align=\"right\">Longitude</td><td align=\"left\">");
  WebClient.print(Longitude, 7);
  WebClient.print("</td></tr>");
  WebClient.printf("<tr><td align=\"right\">Speed</td><td align=\"left\">%s Km/h</td></tr>", GPS.speed.isValid() ? String(GPS.speed.kmph(), 1).c_str() : "???");
  WebClient.printf("<tr><td align=\"right\">Course</td><td align=\"left\">%s</td></tr>", GPS.course.isValid() ? String(String(TinyGPSPlus::cardinal(GPS.course.deg())) + "&nbsp&nbsp(" + String(GPS.course.deg(), 1) + ")").c_str() : "???");
  WebClient.printf("<tr><td align=\"right\">Altitude</td><td align=\"left\">%s m</td></tr>", GPS.altitude.isValid() ? String(GPS.altitude.meters(), 1).c_str() : "???");
  WebClient.print("</table></td>");

  // Comment next line to have the 4 tables side by side
//  WebClient.print("</tr>");
  
  // Trip Statistics
  breakTime(Config.TripStart, tm);
  sprintf(buf, "%s ", dayShortStr(tm.Wday));
  sprintf(buf1, "%s-%02d/%04d %02d:%02d", monthShortStr(tm.Month), tm.Day, tm.Year+1970, tm.Hour, tm.Minute);
  strcat(buf, buf1);
  WebClient.print("<td align=\"center\" valign=\"top\"><table border=\"1\" cellpadding=\"10\">");
  WebClient.printf("<tr><td align=\"center\" colspan=\"2\"><b>Road Trip<p>%s</b></td></tr>", buf);
  if(Config.TripDistance + (Meters * 1000.0) >= 1000) 
    Line = String((Config.TripDistance + (Meters * 1000.0)) / 1000.0, 1) + "&nbspKm";
  else
    Line = String(Config.TripDistance + (Meters * 1000)) + "&nbspm";
  WebClient.printf("<tr><td align=\"right\">Distance</td><td align=\"left\">%s</td></tr>", Line.c_str());
  WebClient.printf("<tr><td align=\"right\">Riding Time</td><td align=\"left\">%s</td></tr>", UpTime((Config.TripRiding + (Config.DailyRiding/1000))*1000, false));
  WebClient.printf("<tr><td align=\"right\">Avg Speed</td><td align=\"left\">%s&nbspKm/h</td></tr>", String(Config.TripAvgSpeed / max((unsigned long)1, Config.TripAvgCntr), 1).c_str());
  WebClient.printf("<tr><td align=\"right\">Max Speed</td><td align=\"left\">%s&nbspKm/h</td></tr>", String(Config.TripMaxSpeed, 1).c_str());
  WebClient.printf("<tr><td align=\"right\">Expenses</td><td align=\"left\">$&nbsp%s</td></tr>", String(Config.TripExpenses, 2).c_str());
  WebClient.print("</table></td>");

  // Season Statistics
  breakTime(Config.SeasonStart, tm);
  sprintf(buf, "%s ", dayShortStr(tm.Wday));
  sprintf(buf1, "%s-%02d/%04d %02d:%02d", monthShortStr(tm.Month), tm.Day, tm.Year+1970, tm.Hour, tm.Minute);
  strcat(buf, buf1);
  WebClient.print("<td align=\"center\" valign=\"top\"><table border=\"1\" cellpadding=\"10\">");
  WebClient.printf("<tr><td align=\"center\" colspan=\"2\"><b>Riding Season<p>%s</b></td></tr>", buf);
  if(Config.SeasonDistance + (Meters * 1000) >= 1000) 
    Line = String((Config.SeasonDistance + (Meters * 1000.0)) / 1000.0, 1) + "&nbspKm";
  else
    Line = String(Config.SeasonDistance + (Meters * 1000)) + "&nbspm";
  WebClient.printf("<tr><td align=\"right\">Distance</td><td align=\"left\">%s</td></tr>", Line.c_str());
  WebClient.printf("<tr><td align=\"right\">Riding Time</td><td align=\"left\">%s</td></tr>", UpTime((Config.SeasonRiding + (Config.DailyRiding/1000))*1000, false));
  WebClient.printf("<tr><td align=\"right\">Avg Speed</td><td align=\"left\">%s&nbspKm/h</td></tr>", String(Config.SeasonAvgSpeed / max((unsigned long)1, Config.SeasonAvgCntr), 1).c_str());
  WebClient.printf("<tr><td align=\"right\">Max Speed</td><td align=\"left\">%s&nbspKm/h</td></tr>", String(Config.SeasonMaxSpeed, 1).c_str());
  WebClient.printf("<tr><td align=\"right\">Expenses</td><td align=\"left\">$&nbsp%s</td></tr>", String(Config.SeasonExpenses, 2).c_str());
  WebClient.print("</table></td></tr></table></center>");

  // Send bottom page "Back" button
  WebClient.println("<hr><center><form id=\"form\" method=\"get\" enctype=\"application/x-www-form-urlencoded\" action=\"/\">");
  WebClient.printf("<input type=\"button\" onclick=\"goBack()\" value=\"Back\"></form></center>", Month, Year);
}

//
// Send a stop location
//
void  StopLocation(WiFiClient WebClient, int Year, int Month, int Day, String Title, String Location)
{
  String          C, CC;
  time_t          t;
  TimeElements    tm;
  
  // Build a time_t for the date requested
  memset(&tm, NULL, sizeof(tm));
  tm.Year = Year - 1970;
  tm.Month = Month;
  tm.Day = Day;
  t = makeTime(tm);
  breakTime(t, tm);

  // Send page top
  WebClient.printf("<center><h2><b>%s ", dayStr(tm.Wday));
  WebClient.printf("%s-%d, %d<br>%s</b></h2></center>", monthStr(Month), Day, Year, Title.c_str());

  // Now insert iframe to get google map location
  WebClient.printf("<hr><center><iframe src=\"https://www.google.com/maps/embed/v1/place?key=%s&q=%s\" width=960 height=540 frameborder=\"0\" style=\"border:0\" allowfullscreen></iframe></center>", DataRec.GoogleAPI, Location.c_str());

  // Insert BACK button
  WebClient.printf("<hr><center><input type=\"button\" onclick=\"goBack()\" value=\"Back\"></center>", Year, Month, Day);
}

//
// Build lightbox or popup string for pictures
// Code based on https://www.w3schools.com/howto/howto_js_lightbox.asp and https://www.w3schools.com/howto/howto_css_modal_images.asp
//
String LightBox(int Year, int Month, int Day)
{
  char    MediaFile[32];
  String  Line, Text, Str1, Str2, Str3;
  int     ImageCount, ImageCntr, Count, i;

  // Check if there's a media file for that day
  sprintf(MediaFile, "/%04d/%02d/%02d_Media.txt", Year, Month, Day);
  File file = SD.open(MediaFile, FILE_READ);
  if(!file) return("");
  
  // Now check if there's any images and get count
  ImageCount = Count = ImageCntr = 0;
  Str1 = Str2 = Str3 = "";
  while(file.available())
  {
    // First get number if images to display
    Line = file.readStringUntil('\n');
    if(Line.indexOf(".mp4") == -1) ++ImageCount;
  }

  // If no images, close file and return an empty string
  if(ImageCount == 0) { file.close(); return(""); }

  // I have images, seek to BOF and insert the images
  file.seek(0L);
  while(file.available())
  {
    Line = file.readStringUntil('\n');
    if(Line.indexOf(".mp4") == -1)
    {
      // I have an image, remove caption text if any
      ++Count;
      Text = Line; Text.toUpperCase();
      i = Text.indexOf("TEXT=");
      if(i != -1)
      {
        Text = Line.substring(i+5);
        Line = Line.substring(0, i);
        while(Line.lastIndexOf(" ") != -1) Line = Line.substring(0, Line.lastIndexOf(" "));
      } else Text = "&nbsp";

      // Do I need to insert a break?
      if(ImageCntr++ == 7) 
      { 
        Str1 += "<br>\n"; 
        if(DataRec.LightBox) Str3 += "<br><br>\n"; 
        ImageCntr = 1; 
      }
      if(Count == 1) Str1 = "<br>\n<center>\n";

      // Backward compatibility with old DropBox files
      if(Line.indexOf("https://www.dropbox.com") != -1) 
      {
        Line.replace("?dl=0", "?dl=1");
        if(Line.indexOf("?dl=1") == -1) Line += "?dl=1";
      }

      // Hack for Google Drive files
      if(Line.indexOf("https://drive.google.com") != -1)
      {
        Line.replace("/open?id=", "/uc?authuser=0&id=");
        Line += "&export=download";
      }
      if(DataRec.LightBox)
      {
        // Insert it into the three strings
        Str1 += "\t<img src=\"" + Line + "\" style=\"width:100%;max-width:128px;height:100%;max-height:80px\" onclick=\"openModal();currentSlide(" + String(Count) + ")\" class=\"hover-shadow cursor\">\n";
        if(Count == 1) Str2 = "<div id=\"myModal\" class=\"modal\">\n\t<span class=\"close cursor\" onclick=\"closeModal()\">&times;</span>\n\t<div class=\"modal-content\">\n";
        Str2 += "\t\t<div class=\"mySlides\">\n\t\t\t<div class=\"numbertext\">" + String(Count) + " / " + String(ImageCount) + "</div>\n";
        Str2 += "\t\t\t<img src=\"" + Line + "\" width=\"100%\">\n\t\t</div>\n";
        if(Count == 1) Str3 = "\t\t<center>\n\t\t\t<div class=\"column\">\n";
        Str3 += "\t\t\t\t<img class=\"demo cursor\" src=\"" + Line + "\" width=128 height=64 onclick=\"currentSlide(" + String(Count) + ")\" alt=\"" + Text + "\">\n";
      }
      else
        Str1 += "\t<img id=\"Pic" + String(Count) + "\" src=\"" + Line + "\" alt=\"" + Text + "\" style=\"width:100%;max-width:128px;height:100%;max-height:80px\" onclick=\"OpenModal('Pic" + String(Count) + "')\" class=\"hover-shadow cursor\">\n";
    }
  }
  
  // Insert string terminators
  if(DataRec.LightBox)
  {
    Str1 += "</center>\n";
    Str2 += "\t\t<a class=\"prev\" onclick=\"plusSlides(-1)\">&#10094;</a>\n";
    Str2 += "\t\t<a class=\"next\" onclick=\"plusSlides(1)\">&#10095;</a>\n";
    Str2 += "\t\t<div class=\"caption-container\">\n\t\t\t<p id=\"caption\"></p>\n\t\t</div>\n";
    Str3 += "<br><br>\t\t\t</div>\n\t\t</center>\n\t</div>\n</div>";
  }
  else
  {
    Str1 += "</center>\n<div id=\"myModal\" class=\"modal\">\n\t<span class=\"close\">&times;</span>\n\t<img class=\"modal-content\" id=\"img01\">\n\t<div id=\"caption\"></div>\n</div>\n";
    Str1 += "<script>\n\tvar modal = document.getElementById(\"myModal\");\n";
    Str1 += "\tvar modalImg = document.getElementById(\"img01\");\n";
    Str1 += "\tvar captionText = document.getElementById(\"caption\");\n";
    Str1 += "\tvar span = document.getElementsByClassName(\"close\")[0];\n";
    Str1 += "\tfunction OpenModal(Pic) { var img = document.getElementById(Pic); modal.style.display = \"block\"; modalImg.src = img.src; captionText.innerHTML = img.alt; }\n";
    Str1 += "\tspan.onclick = function() { modal.style.display = \"none\"; }\n</script>";
  }

  // Close file and return the three string as one
  file.close();
  return(Str1 + Str2 + Str3);
}

//
// Send daily ride page
//
void  DailyRide(WiFiClient WebClient, int Year, int Month, int Day, int PageType)
{
  char            MediaFile[32], StartTime[32], EndTime[32];
  double          Meters, Lat, Lon, AvgSpeed, MaxSpeed;
  float           Gas, Lodging, Maintenance, Other;
  int             FileDay, Segments, Tracks, Stops, ImageCntr;
  unsigned long   Riding, Elapsed;
  String          Image, Line, StopStr;
  time_t          t;
  TimeElements    tm;
  
  // Build a time_t for the date requested
  memset(&tm, NULL, sizeof(tm));
  tm.Year = Year - 1970;
  tm.Month = Month;
  tm.Day = Day;
  t = makeTime(tm);
  breakTime(t, tm);
  ImageCntr = 0;
  
  // Get daily info
  GPX_FileInfo(Year, Month, Day, &Meters, &Segments, &Tracks, &Stops, &Riding, &Elapsed, StartTime, EndTime, &AvgSpeed, &MaxSpeed, &StopStr);
  WebClient.printf("<center><b><h2>%s ", dayStr(tm.Wday));
  WebClient.printf("%s-%d, %d</h2>", monthStr(Month), Day, Year);
  if(Meters >= 1.00) 
    Line = String(Meters, 1) + "&nbspKm";
  else
    Line = String(Meters * 1000.0) + "&nbspm";
  WebClient.printf("%s, %d Segments, %d Stops\r\n", Line.c_str(), Segments, Stops);
  if(Riding != 0) 
  {
    WebClient.printf("<br>Start Time = %s, End Time = %s\r\n", StartTime, EndTime);
    WebClient.printf("<br>Riding Time = %s, Elapsed Time = ", UpTime(Riding*1000, false));
    WebClient.printf("%s\r\n", UpTime(Elapsed*1000, false));
    WebClient.printf("<br>Avg Speed = %s Km/h, Max Speed = %s Km/h\r\n", String(AvgSpeed, 1).c_str(), String(MaxSpeed, 1).c_str());
  }
  
  // Initialize the Google image string
  Image = "https://maps.googleapis.com/maps/api/staticmap?size=640x400&scale=2&maptype=terrain&key=" + String(DataRec.GoogleAPI);

  // Encode track points in Google image
  Image += GPX_EncodeSegment(Year, Month, Day, Tracks);

  // Send Google Map link and enclose image within a 960x540 image
  WebClient.println("</b><hr><img src=\"" + Image + "\" width=960 alt=\"Today's ride\"></img></center>\n");

  // Now add a link to all the stops if I'm not editing
  if(PageType == 2 && Stops != 0) WebClient.print("<center>" + StopStr + "</center>\n");

  // Insert daily expenses table if I have any epenses
  Get_Expenses(Year, Month, Day, &Gas, &Lodging, &Maintenance, &Other);
  if(Gas + Lodging + Maintenance + Other != 0.00 && PageType == 2)
  {
    WebClient.print("<hr><center><b>Daily Expenses</b><br><table border=\"1\" cellpadding=\"5\"><tr>");
    WebClient.print("<td align=\"center\">Gas<br>$&nbsp" + String(Gas, 2) + "</td>");
    WebClient.print("<td align=\"center\">Lodging<br>$&nbsp" + String(Lodging, 2) + "</td>");
    WebClient.print("<td align=\"center\">Maintenance<br>$&nbsp" + String(Maintenance, 2) + "</td>");
    WebClient.print("<td align=\"center\">Food / Other<br>$&nbsp" + String(Other, 2) + "</td>");
    WebClient.print("<td align=\"center\">Daily Total<br><b>$&nbsp" + String(Gas + Lodging + Maintenance + Other, 2) + "</b></td>");
    WebClient.println("</tr></table></center>");
  }
  
  // If editing, start input form
  if(PageType == 3 || PageType == 4)
  {
    WebClient.printf("<hr><center><h3><u>%s</u></h3>", PageType == 3 ? "Riding Notes" : "Media Files");
    WebClient.printf("<form method=\"POST\"><textarea name=\"%s\" rows=\"20\" cols=\"120\">", PageType == 3 ? "Notes" : "Media");
  }
  
  // Check if there's a media file for that day
  sprintf(MediaFile, "/%04d/%02d/%02d_Media.txt", Year, Month, Day);
  File file = SD.open(MediaFile, FILE_READ);
  if(file) 
  {
    // Yup there is.  Read file looking for movies first
    while(file.available())
    {
      Line = file.readStringUntil('\n');
      if(Line.indexOf(".mp4") != -1)
      {
        if(PageType == 2) 
        {
          // Backward compatibility with old DropBox files
          if(Line.indexOf("https://www.dropbox.com") != -1) 
          {
            Line.replace("?dl=0", "?dl=1");
            if(Line.indexOf("?dl=1") == -1) Line += "?dl=1";
          }

          // Google Drive hack
          if(Line.indexOf("https://drive.google.com") != -1)
          {
            Line = Line.substring(0, Line.indexOf(".mp4"));
            while(Line.lastIndexOf(" ") != -1) Line = Line.substring(0, Line.lastIndexOf(" "));
            Line.replace("/open?id=", "/uc?authuser=0&id=");
            Line += "&export=download";
          }
          WebClient.printf("<hr><br><center><video width=720 controls><source src=\"%s\" type=\"video/mp4\">Your browser does not support the video tag.</source></video></center>", Line.c_str());
        }
        if(PageType == 4) WebClient.print(Line);
      }
    }

    // Insert movie lines if I'm editing
    file.seek(0L);
    while(file.available())
    {
      Line = file.readStringUntil('\n');
      if(Line.indexOf(".mp4") == -1 && PageType == 4)
        WebClient.print(Line);
    }
    file.close();
  }

  // Insert picture viewer if I'm not editing
  if(PageType == 2) 
  {
    Line = LightBox(Year, Month, Day);
    WebClient.println(Line);
  } else Line = "";
  
  // Now check if there's riding notes for that day
  if(PageType == 2 || PageType == 3)
  {
    sprintf(MediaFile, "/%04d/%02d/%02d_Text.txt", Year, Month, Day);
    file = SD.open(MediaFile, FILE_READ);
    if(file) 
    {
      // Send the text file
      if(PageType == 2) WebClient.println("<br><hr><blockquote><h2><font color=\"#FF0000\"><u>Riding Notes</u><font color=\"#000000\"></h2>");
      while(file.available())
      {
        Line = file.readStringUntil('\n');
        WebClient.print(Line);
        if(PageType == 2) WebClient.println("<br>");
      }
      if(PageType == 2) WebClient.print("</blockquote>");
      file.close();
    }  
  }

  // Add bottom buttons
  if(PageType == 2)
  {
    // Add buttons to edit
    WebClient.println("<hr><center><form id=\"form\" method=\"get\" enctype=\"application/x-www-form-urlencoded\" action=\"/\">");
    WebClient.printf("<input type=\"button\" onclick=\"myFunction('/Month=%02d,Year=%04d')\" value=\"Back\">", Month, Year);
    WebClient.printf("&nbsp&nbsp<input type=\"button\" onclick=\"myFunction('/Notes=%04d-%02d-%02d')\" value=\"Notes\">", Year, Month, Day);
    WebClient.printf("&nbsp&nbsp<input type=\"button\" onclick=\"myFunction('/Media=%02d-%02d-%02d')\" value=\"Media\">", Year, Month, Day);
    WebClient.printf("&nbsp&nbsp<input type=\"button\" onclick=\"myFunction('/Expen=%02d-%02d-%02d')\" value=\"Expenses\">", Year, Month, Day);
    WebClient.println("</form></center>");    
  }
  else
  {
    // I'm editing, end input form
    WebClient.print("</textarea><br><center><input type=\"submit\">");
    WebClient.printf("&nbsp&nbsp<input type=\"button\" onclick=\"goBack()\" value=\"Cancel\"></center>", Year, Month, Day);
    WebClient.println("<script>function myFunction(Value) { window.location.href=Value; }</script>");    
  }  
}

//
// Dump log file
//
void DumpLogFile(WiFiClient WebClient)
{
  // Open file for reading
  File file = SD.open("/LogFile.txt", FILE_READ);
  if(!file) 
  {
    WebClient.println("<hr>Unable to open log file!<br>");
    WriteLogFile("Unable to open log file!");
    return;
  }

  // Read line by line 
  WebClient.println("<hr>");
  while(file.available())
  {
    String Line = file.readStringUntil('\n');
    WebClient.println(Line + "<br>");
  }
  
  // Close file
  file.close();
}

#ifdef UBLOX
////////////////////////////////////////////////////////////////////////////////////////////////////
//
// U-BLOX GPS FUNCTIONS
//
////////////////////////////////////////////////////////////////////////////////////////////////////
 
//
// Send a message to U-BLOX GPS module
//
bool GPSMsg(const char *nmea, uint8_t Rate)
{
  char    c, ubx[512];
  int     i, j;
  bool    B5 = false;
  uint8_t CK_A, CK_B;
  CK_A = CK_B = 0;

  // Make sure NMEA type is 3 characters
  if(strlen(nmea) != 3) return false;

  // Build UBX message
  memset(&ubx, NULL, sizeof(ubx));
  ubx[0] = 0xB5; ubx[1] = 0x62; ubx[2] = 0x06; ubx[3] = 0x01; ubx[4] = 0x08; ubx[5] = 0x00; 
  ubx[6] = 0xF0; ubx[8] = 0x00; ubx[10] = 0x00; ubx[11] = 0x00; ubx[12] = 0x00; ubx[13] = 0x00;
  if(strcmp("GGA", nmea) == 0) ubx[7] = 0x00;
  else if(strcmp("GLL", nmea) == 0) ubx[7] = 0x01;
  else if(strcmp("GSA", nmea) == 0) ubx[7] = 0x02;
  else if(strcmp("GSV", nmea) == 0) ubx[7] = 0x03;
  else if(strcmp("RMC", nmea) == 0) ubx[7] = 0x04;
  else if(strcmp("VTG", nmea) == 0) ubx[7] = 0x05;
  else if(strcmp("GRS", nmea) == 0) ubx[7] = 0x06;
  else if(strcmp("GST", nmea) == 0) ubx[7] = 0x07;
  else if(strcmp("ZDA", nmea) == 0) ubx[7] = 0x08;
  else if(strcmp("GBS", nmea) == 0) ubx[7] = 0x09;
  else if(strcmp("DTM", nmea) == 0) ubx[7] = 0x0A;
  else if(strcmp("GNS", nmea) == 0) ubx[7] = 0x0D;
  else if(strcmp("VLW", nmea) == 0) ubx[7] = 0x0F;
  else { WriteLogFile("Invalid U-Blox NMEA message (" + String(nmea) + ")", false); return(false); }
  ubx[9] = Rate; 

  // Calc checksum and insert into string
  for(int i=0; i<12; i++) { CK_A = CK_A + ubx[i+2]; CK_B = CK_B + CK_A; }
  ubx[14] = CK_A; ubx[15] = CK_B;

  // Flush out GPS port
  while(GPS_Port.available()) GPS_Port.read();
  
  // Now send data to module
#ifdef UBLOX_DEBUG  
  Serial.printf("%s = %d ( ", nmea, Rate);
#endif  
  for(int i=0; i<16; i++) 
  { 
#ifdef UBLOX_DEBUG  
    if(ubx[i] < 16) Serial.print("0"); 
    Serial.print(ubx[i], HEX); Serial.print(" "); 
#endif    
    GPS_Port.write(ubx[i]);
  }

  // Now get result, should be 10 bytes long
#ifdef UBLOX_DEBUG  
  Serial.print(")\nResult = ");
#endif  
  unsigned long entry = millis();
  memset(&ubx, NULL, sizeof(ubx));
  i = 0; j = sizeof(ubx);
  while(millis() - entry < 2000 && i < sizeof(ubx))
  {
    if(GPS_Port.available())
    {
      c = GPS_Port.read();
#ifdef UBLOX_DEBUG 
      if(B5 && i < 10) 
      { 
        if(i == 1) { Serial.print(ubx[0], HEX); Serial.print(" "); }
        if(c < 16) Serial.print("0"); 
        Serial.print(c, HEX); Serial.print(" ");
      }
#endif    
      if(c == 0xB5) B5 = true;
      if(B5) ubx[i++] = c; 
    }
    if(B5 && i == 10) break;
  }
#ifdef UBLOX_DEBUG  
  Serial.println((B5 && ubx[2] == 0x05 && ubx[3] == 0x01) ? " OK" : " Failed");
#endif  

  // Now check response...
  // B5 62 05 01 xx yy is a ACK response
  // B5 62 05 00 xx yy is a NAK response, typically sent by a CASIC knock-off U-Blox module
  return((ubx[2] == 0x05 && ubx[3] == 0x01));
}

//
// Get U-Blox version information
//
double Get_UBlox_Version(String *Software, String *Hardware, String *Module)
{
  char      c, ubx[512], buf[5], SW[30], HW[10], Ext[30];
  int       i, j;
  double    Version = 0.0;
  bool      B5 = false;
  uint8_t   CK_A, CK_B;
  CK_A = CK_B = 0;

  // Send to module using UBX protocol
  memset(&ubx, NULL, sizeof(ubx));
  ubx[0] = 0xB5; ubx[1] = 0x62; ubx[2] = 0x0A; ubx[3] = 0x04; ubx[4] = 0x00; ubx[5] = 0x00;

  // Calc checksum and insert into string
  for(int i=0; i<4; i++) { CK_A = CK_A + ubx[i+2]; CK_B = CK_B + CK_A; }
  ubx[6] = CK_A; ubx[7] = CK_B;

  // Flush out GPS port
  while(GPS_Port.available()) GPS_Port.read();
  
  // Now send data to module
#ifdef UBLOX_DEBUG  
  Serial.print("Getting U-Blox version ( ");
#endif  
  for(i=0; i<8; i++) 
  { 
#ifdef UBLOX_DEBUG  
    if(ubx[i] < 16) Serial.print("0"); 
    Serial.print(ubx[i], HEX); Serial.print(" "); 
#endif    
    GPS_Port.write(ubx[i]);
  }

  // Read data coming back until I get a 0xB5 character which is the begining of the reply or I time out
#ifdef UBLOX_DEBUG  
  Serial.print(")\nResult = ");
#endif  
  unsigned long entry = millis();
  memset(&ubx, NULL, sizeof(ubx));
  i = 0; j = sizeof(ubx);
  while(millis() - entry < 2000 && i < sizeof(ubx))
  {
    if(GPS_Port.available())
    {
      c = GPS_Port.read();
#ifdef UBLOX_DEBUG 
      if(B5 && i < 10) 
      { 
        if(i == 1) { Serial.print(ubx[0], HEX); Serial.print(" "); }
        if(c < 16) Serial.print("0"); 
        Serial.print(c, HEX); Serial.print(" ");
      }
#endif    
      if(c == 0xB5) B5 = true;
      if(B5) ubx[i++] = c; 
    }
    if(B5 && i == 10)
    {
      // Now check response...
      // B5 62 0A 04 xx yy is a version info message
      // B5 62 05 00 xx yy is a NAK response, typically sent by a CASIC knock-off U-Blox module
#ifdef UBLOX_DEBUG  
      Serial.println((B5 && ubx[2] == 0x0A && ubx[3] == 0x04) ? " OK" : " Failed");
#endif  
      j = ubx[4] + ubx[5]*255;                      // Message length
      if(ubx[2] == 0x05 && ubx[3] == 0x00) break;   // NAK message
    }
    if(B5 && i-6 > j) break;
  }
  
  // If I have a valid response, extract Software, Hardware versions and module model
  memset(&SW, NULL, sizeof(SW));
  memset(&HW, NULL, sizeof(HW));
  memset(&Ext, NULL, sizeof(Ext));
  if(B5 && ubx[2] == 0x0A && ubx[3] == 0x04)
  {
    strcpy(buf, "MOD=");
    memcpy(&SW, &ubx[6], 30);
    memcpy(&HW, &ubx[36], 10);
#ifdef UBLOX_DEBUG    
    Serial.printf("S/W = %s, H/W = %s\n", SW, HW);
#endif    

    // Here I'm assuming the hardware string is major (4 chars) plus minor (4 chars)
    // Resulting in something like; 8.1 or 6.0200
    Version = atoi(HW) / 10000.00;
#ifdef UBLOX_DEBUG    
    Serial.printf("Version = %.4f", Version);
#endif    
    i = 46;
    while(i < j)
    {
      memcpy(&Ext, &ubx[i], 30);
      if(strstr(&Ext[0], &buf[0]) != NULL) 
      {
        memcpy(&Ext[0], &ubx[i+4], 26);
#ifdef UBLOX_DEBUG
        Serial.printf(", Module = %s\n", Ext);        
#endif        
        break;
      }
      i += 30;
    }
#ifdef UBLOX_DEBUG
    if(strlen(Ext) == 0) Serial.println();
#endif        
  }
  
  // Return values
  *Software = String(SW);
  *Hardware = String(HW);
  *Module = String(Ext);
  return(Version);
}

//
// Initialize U-BLOX GPS module
//
bool InitUBLOX(void)
{
  double          Version;
  String          Software, Hardware, Module;
  unsigned long   entry = millis();
    
  // Get version of U-Blox module
  Version = Get_UBlox_Version(&Software, &Hardware, &Module);
  if(Version == 0) Version = Get_UBlox_Version(&Software, &Hardware, &Module); // Call a second time if first failed
  if(Version == 0)
  {
    WriteLogFile("You have a fake U-Blox module!", false);
#ifdef TIMINGS
    WriteLogFile("InitUBLOX() = " + String(millis() - entry) + " ms", false);
#endif  
    return(false);
  }

  // Print module and hardware version
  WriteLogFile(Module + " (" + String(Version, 4) + "), S/W = " + Software, false);
  
  // Disable certain NMEA messages since TinyGPS++ only uses GxGGA and GxRMC  
  if(!GPSMsg("GGA", 1)) return(false); // GPS Fixed Data
  if(!GPSMsg("RMC", 1)) return(false); // Recommended Minimum Specific GNSS Data
  if(!GPSMsg("VTG", 0)) return(false); // Course Over Ground and Ground Speed (true and magnetic)
  if(!GPSMsg("GSV", 0)) return(false); // GNSS Satellites in View
  if(!GPSMsg("GSA", 0)) return(false); // GNSS DOP and Active Satellites
  if(!GPSMsg("GLL", 0)) return(false); // Geographic Position - Latidute/Longitude
  if(!GPSMsg("GST", 0)) return(false); // GNSS Pseudorange Error Statistics

  // These messages are typically not send but you can turn them on (TinyGPS++ will not decode them)
//  if(!GPSMsg("ZDA", 1)) return(false); // Time & Date
//  if(!GPSMsg("DTM", 1)) return(false); // Datum Reference
//  if(!GPSMsg("GBS", 1)) return(false); // Satellite default Detection
//  if(!GPSMsg("GNS", 1)) return(false); // GNSS Fix Data
//  if(!GPSMsg("GRS", 1)) return(false); // GNSS Range Residuals
//  if(!GPSMsg("VLW", 1)) return(false); // Dual Ground/Water Distance

  // All good...
#ifdef TIMINGS
  WriteLogFile("InitUBLOX() = " + String(millis() - entry) + " ms", false);
#endif  
  return(true);
}
#endif


#ifdef ARDUINO_M5Stack_Core_ESP32
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  M5STACK FUNCTIONS
//
////////////////////////////////////////////////////////////////////////////////////////////////////

//
// M5Stack start-up logo.  Because I love it!
//
void startupLogo(const char* Logo) 
{
    static uint8_t brightness, pre_brightness;
    uint32_t length = strlen((char*)m5stack_startup_music);
    M5.Lcd.setBrightness(0);
    if(SD.exists(Logo)) 
      M5.Lcd.drawJpgFile(SD, Logo);
    else
      if(SD.exists("/M5_Logo.jpg")) M5.Lcd.drawJpgFile(SD, "/M5_Logo.jpg"); else return;

    // Play sound
    for(int i=0; i<length; i++) 
    {
      dacWrite(SPEAKER_PIN, m5stack_startup_music[i]>>2);
      delayMicroseconds(40);
      brightness = (i/157);
      if(pre_brightness != brightness) 
      {
        pre_brightness = brightness;
        M5.Lcd.setBrightness(brightness);
      }
    }

    // Fade picture to black
    for(int i=255; i>=0; i--) 
    {
      M5.Lcd.setBrightness(i);
      delay(10);
    }
    M5.Lcd.fillScreen(BLACK);
}

//
// Play M5Stack startup music
//
void Music(void)
{
  uint32_t length = strlen((char*)m5stack_startup_music);
  for(int i=0; i<length; i++) 
  {
    dacWrite(SPEAKER_PIN, m5stack_startup_music[i]>>2);
    delayMicroseconds(40);
  }  
  M5.Speaker.begin();  
}

//
// Warning Tone
//
void WarningTone(byte Cntr)
{
  M5.Speaker.tone(1000); delay(50); M5.Speaker.mute(); delay(200);
  M5.Speaker.tone(750); delay(50); M5.Speaker.mute(); delay(200);
  M5.Speaker.tone(1000); delay(50); M5.Speaker.mute(); delay(200);
  for(int i=0; i<Cntr; i++) { LowBeep() delay(250); }
}

//
// Display status line
//
void StatusLine(char *Buf)
{
  M5.Lcd.fillRect(0, 27, 320, 25, TFT_CYAN);
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 32);
}

//
// Display main screen buttons
//
void Display_Buttons(void)
{
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setCursor(25, 220); M5.Lcd.print("Weather          Menu");  
}

//
// Hide buttons text
//
void Hide_Buttons(void)
{
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setCursor(25, 220); M5.Lcd.print("                      ");  
}

//
// Display RSSI icon
//
void renderRSSIBars(void)
{
  int   X, Y, Height, Width;

  Height = 8; Width = 4;
  X = 295;
  Y = 11;

  M5.Lcd.fillRect(X, Y - 6, (5 * Width) + 2, Height + 6, TFT_CYAN);
  if(!WiFiConnected) return;
  int quality = rssiToQuality(WiFi.RSSI());
  if(quality != 0) M5.Lcd.fillRect(X, Y, Width, Height, RED);
  if(quality >= 45) M5.Lcd.fillRect(X + (2 * Width) - 2, Y - 2, Width, Height + 2, RED);      
  if(quality >= 70) M5.Lcd.fillRect(X + (3 * Width), Y - 4, Width, Height + 4, RED);      
  if(quality >= 90) M5.Lcd.fillRect(X + (4 * Width) + 2, Y - 6, Width, Height + 6, RED);
}

//
// Convert RSSI value to percentage
//
int rssiToQuality(int rssi)
{
  int quality;
  
  if(rssi <= -100) 
    quality = 0;
  else 
    if(rssi >= -51) quality = 100; else quality = 2 * (rssi + 100);
  return quality;
}

//
// Display UI screen
//
void UI(void)
{
  double        OilChange;
  unsigned long entry = millis();
  char          buf[80], buf1[20];
  static int    Cntr;

  // Have I displayed the static text yet?
  if(!TextDisplay)
  {
    // Nope, display all text that doesn't change at each update
    M5.Lcd.fillScreen(TFT_BLUE);
    M5.Lcd.fillRect(0, 0, 320, 52, TFT_CYAN);
    M5.Lcd.fillRect(0, 215, 320, 240, TFT_CYAN);
    M5.Lcd.drawLine(0, 25, 320, 25, BLACK);
    M5.Lcd.drawLine(154, 58, 154, 210, WHITE);
    M5.Lcd.drawLine(155, 58, 155, 210, WHITE);
    
    // Distance / Moving
    M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 60); M5.Lcd.print("Distance");
    M5.Lcd.setCursor(170, 60); M5.Lcd.print("Riding Time");
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(WHITE, TFT_BLUE);
    M5.Lcd.setCursor(10, 80); M5.Lcd.print("0000 ");
    M5.Lcd.setTextSize(2); M5.Lcd.print("Km");
    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(170, 80); M5.Lcd.print("00:00:00");
    M5.Lcd.drawLine(0, 107, 320, 107, WHITE);
    M5.Lcd.drawLine(0, 108, 320, 108, WHITE);

    // Avg / Max Speed
    M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 112); M5.Lcd.print("Avg Speed");
    M5.Lcd.setCursor(170, 112); M5.Lcd.print("Max Speed");
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(WHITE, TFT_BLUE);
    M5.Lcd.setCursor(10, 132); M5.Lcd.print("000 ");
    M5.Lcd.setTextSize(2); M5.Lcd.print("Km/h");
    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(170, 132); M5.Lcd.print("000 ");
    M5.Lcd.setTextSize(2); M5.Lcd.print("Km/h");
    M5.Lcd.drawLine(0, 159, 320, 159, WHITE);
    M5.Lcd.drawLine(0, 160, 320, 160, WHITE);

    // Distance Home / GPS info
    M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 164); M5.Lcd.print("Home");
    M5.Lcd.setCursor(170, 164); M5.Lcd.print("Sats  Alt");
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(WHITE, TFT_BLUE);
    M5.Lcd.setCursor(10, 184); M5.Lcd.print("0000 ");
    M5.Lcd.setTextSize(2); M5.Lcd.print("Km");
    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(170, 184); M5.Lcd.print("00 0000");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("m");

    // Display button
    Display_Buttons();
    TextDisplay = true;
    Cntr = 0;
  }

  // Now display text that changes each update
  // Top info line and location or speed if tracking
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(5, 5);
  M5.Lcd.printf("%c", GPSSymbols[GPSCntr]);
  M5.Lcd.printf(" %s ", dayShortStr(weekday()));
  M5.Lcd.printf("%s-%02d %02d:%02d %c%d  %c", monthShortStr(month()), day(), hour(), minute(),  Tracking ? 'T' : ' ', Config.GPXState, WiFiConnected ? 'W' : ' ');

  // Display WiFi signal strength
  renderRSSIBars();

  // If not tracking display City, otherwise display current speed
  // Alternate the City name with local IP number every 5 seconds for one second
  int i = City.indexOf(",");
  if(i != -1) strcpy(buf, City.substring(0, i).c_str()); else strcpy(buf, City.c_str());
  if(!Tracking)
  {
    if(++Cntr % 5 == 0 && WiFiConnected) sprintf(buf, "%s", WiFi.localIP().toString().c_str());
  } else sprintf(buf, "%s Km/h", String(Speed, 0).c_str());
  if(SDCardMissing) strcpy(buf, "SD CARD MISSING!");
  StatusLine(buf);

  // Distance and riding time
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  if(DailyDistance < 1000.0)
    { sprintf(buf, "%4d ", (int)DailyDistance); strcpy(buf1, "m "); }
  else
    { sprintf(buf, "%4d ", (int)(DailyDistance / 1000.0)); strcpy(buf1, "Km"); }
  M5.Lcd.setCursor(10, 80); M5.Lcd.print(buf);
  M5.Lcd.setTextSize(2); M5.Lcd.print(buf1); 
  
  // If Speed is 0 then increment moving time by one second (hack)
  // This will act as if the clock is stopped.
  if(Speed == 0) 
  {
    MovingTime += millis() - UITime;
    if(MovingTime > millis()) MovingTime = millis();
  }
  sprintf(buf, UpTime(Tracking ? (Config.DailyRiding + (millis() - MovingTime)) : Config.DailyRiding, true));
  M5.Lcd.setTextSize(3);  
  M5.Lcd.setCursor(170, 80); M5.Lcd.print(buf);

  // Avg and Max speeds
  sprintf(buf, "%3d", (int)(Config.AvgSpeed / max((unsigned long)1, Config.AvgCntr)));
  M5.Lcd.setCursor(10, 132); M5.Lcd.print(buf);
  sprintf(buf, "%3d", (int)Config.MaxSpeed);
  M5.Lcd.setCursor(170, 132); M5.Lcd.print(buf);

  // Distance home and GPS info  
  M5.Lcd.setTextSize(2); M5.Lcd.setCursor(300, 164); M5.Lcd.print(GPSCntr % 2 == 0 ? "*" : " ");
  if(DistanceHome < 1000.0)
    { sprintf(buf, "%4d ", (int)DistanceHome); strcpy(buf1, "m "); }
  else
    { sprintf(buf, "%4d ", (int)(DistanceHome / 1000.0)); strcpy(buf1, "Km"); }
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(10, 184); M5.Lcd.print(buf); 
  M5.Lcd.setTextSize(2); M5.Lcd.print(buf1); M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(170, 184); 
  sprintf(buf, "%2d %4d", GPS.satellites.value(), 
    (int)(GPS.altitude.meters() < 0 ? 0 : (GPS.altitude.meters() > 9999 ? 9999 : GPS.altitude.meters())));
  M5.Lcd.print(buf);

  // Reset color, text size and save UI time
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setTextSize(2);  
  if(!Tracking) Display_Buttons();
//  Serial.print("UI() = "); Serial.print(millis()-entry); Serial.println(" ms");
  UITime = millis();
}


//
// Display menu
//
void Menu(void)
{
  char          Buf[32];
  unsigned long entry = millis();

  // Clear display area and prompt user
  StatusLine("Menu Options");
  M5.Lcd.fillRect(0, 53, 320, 165, TFT_BLUE);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.drawString("->", 0, 80);
  M5.Lcd.drawString("Road Trip Statistics", 40, 80);
  M5.Lcd.drawString("Season Statistics", 40, 100);
  M5.Lcd.drawString("Get Weather Forecast", 40, 120);
  M5.Lcd.drawString("Perform Oil Change", 40, 140);
  M5.Lcd.drawString("M5Stack Power Down", 40, 160);  
  M5.Lcd.drawString("Exit Menu", 40, 180);
  M5.Lcd.fillRect(0, 215, 320, 240, TFT_CYAN);
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setCursor(25, 220); M5.Lcd.print("  Up     Down   Select");
  M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
  int Option = 1;
  int Pos = 80;
  int Max = 6;
  while(millis() - entry < ScreenDelay)
  {
    // Run M5 update
    M5.update();
    if(M5.BtnA.wasPressed()) 
    {
      M5.Lcd.drawString("  ", 0, Pos);
      Pos -= 20;
      if(--Option < 1) 
      { 
          Option = Max; 
          Pos = 180; 
      }
      M5.Lcd.drawString("->", 0, Pos);
      Beep(); 
      entry = millis();
    }
    if(M5.BtnB.wasPressed()) 
    {
      M5.Lcd.drawString("  ", 0, Pos);
      Pos += 20;
      if(++Option > Max) { Option = 1; Pos = 80; }
      M5.Lcd.drawString("->", 0, Pos);
      Beep(); 
      entry = millis();
    }
    if(M5.BtnC.wasPressed()) 
    {
      Beep(); 
      switch(Option)
      {
        case 1: // Road Trip Statistics
          Road_Trip(true);
          break;
        case 2: // Season Statistics
          Road_Trip(false);
          break;
        case 3: // Update weather forecast
          M5.Lcd.setTextColor(RED, TFT_CYAN);
          if(WiFiConnected)
            { Get_Forecast(); Weather(); }
          else
            { delay(250); LowBeep(); }
          break;
        case 4: // Oil Change
          Oil_Change();
          break;
        case 5: // Power Down
          if(Confirm("Turn off M5Stack", "POWER DOWN"))
          {
            DweetPost(-2);
            startupLogo("/PowerOff.jpg");
            PowerOff();
            M5.Lcd.setBrightness(200);
          }
          break;
        case 6: // Exit
          break;        
      }
      break;
    }
  }

  // Reset timers
  GPSTime = DweetTime = WiFiTime = millis();
  TextDisplay = false;
}

//
// Display Road Trip / Season Statistics
//
void Road_Trip(bool RoadTrip)
{
  unsigned long entry = millis();
  double        OilChange;
  char          buf[50], buf1[50];
  TimeElements  tm;

  // Display static text
  M5.Lcd.fillScreen(TFT_BLUE);
  M5.Lcd.fillRect(0, 0, 320, 52, TFT_CYAN);
  M5.Lcd.fillRect(0, 215, 320, 240, TFT_CYAN);
  M5.Lcd.drawLine(0, 25, 320, 25, BLACK);
  M5.Lcd.drawLine(154, 58, 154, 210, WHITE);
  M5.Lcd.drawLine(155, 58, 155, 210, WHITE);
  
  // Distance / Moving
  M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 60); M5.Lcd.print("Distance");
  M5.Lcd.setCursor(170, 60); M5.Lcd.print("Riding Time");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  M5.Lcd.setCursor(10, 80); M5.Lcd.print("00000 ");
  M5.Lcd.setTextSize(2); M5.Lcd.print("Km");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(170, 80); M5.Lcd.print("00:00:00");
  M5.Lcd.drawLine(0, 107, 320, 107, WHITE);
  M5.Lcd.drawLine(0, 108, 320, 108, WHITE);

  // Avg / Max Speed
  M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 112); M5.Lcd.print("Avg Speed");
  M5.Lcd.setCursor(170, 112); M5.Lcd.print("Max Speed");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  M5.Lcd.setCursor(10, 132); M5.Lcd.print("000 ");
  M5.Lcd.setTextSize(2); M5.Lcd.print("Km/h");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(170, 132); M5.Lcd.print("000 ");
  M5.Lcd.setTextSize(2); M5.Lcd.print("Km/h");
  M5.Lcd.drawLine(0, 159, 320, 159, WHITE);
  M5.Lcd.drawLine(0, 160, 320, 160, WHITE);

  // Expenses / Oil Change
  M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 164); M5.Lcd.print("Expenses");
  M5.Lcd.setCursor(170, 164); M5.Lcd.print("Oil Change");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  M5.Lcd.setCursor(10, 184); M5.Lcd.print("0000.00");
  M5.Lcd.setTextSize(2); M5.Lcd.print("$");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(170, 184); M5.Lcd.print("0000");
  M5.Lcd.setTextSize(2); M5.Lcd.print("Km");

  // Now display text that changes
  // Top info line, start of road trip
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setTextSize(2);
  if(RoadTrip) strcpy(buf, "Road Trip Started On"); else strcpy(buf, "Season Started On");
  M5.Lcd.drawString(buf, 160 - (M5.Lcd.textWidth(buf)/2), 5);
  if(RoadTrip) breakTime(Config.TripStart, tm); else breakTime(Config.SeasonStart, tm);
  sprintf(buf, "%s ", dayShortStr(tm.Wday));
  sprintf(buf1, "%s-%02d %04d %02d:%02d", monthShortStr(tm.Month), tm.Day, tm.Year+1970, tm.Hour, tm.Minute);
  strcat(buf, buf1);
  StatusLine(buf);
 
  // Distance and moving time
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  if((int)((RoadTrip ? Config.TripDistance : Config.SeasonDistance) + DailyDistance) / 1000 == 0)
    { sprintf(buf, "%5d ", (int)(RoadTrip ? Config.TripDistance : Config.SeasonDistance) + DailyDistance); strcpy(buf1, "m "); }
  else
    { sprintf(buf, "%5d ", (int)((RoadTrip ? Config.TripDistance : Config.SeasonDistance) + DailyDistance) / 1000); strcpy(buf1, "Km"); }
  M5.Lcd.setCursor(10, 80); M5.Lcd.print(buf);
  M5.Lcd.setTextSize(2); M5.Lcd.print(buf1); M5.Lcd.setTextSize(3);
  sprintf(buf, UpTime(((RoadTrip ? Config.TripRiding : Config.SeasonRiding) + (Config.DailyRiding / 1000)) * 1000, true));
  M5.Lcd.setCursor(170, 80); M5.Lcd.print(buf);

  // Avg and Max speeds
  sprintf(buf, "%3d", (int)((RoadTrip ? Config.TripAvgSpeed : Config.SeasonAvgSpeed) / max((unsigned long)1, (RoadTrip ? Config.TripAvgCntr : Config.SeasonAvgCntr))));
  M5.Lcd.setCursor(10, 132); M5.Lcd.print(buf);
  sprintf(buf, "%3d", (int)(RoadTrip ? Config.TripMaxSpeed : Config.SeasonMaxSpeed));
  M5.Lcd.setCursor(170, 132); M5.Lcd.print(buf);

  // Trip/Season expenses and oil change  
  sprintf(buf, "%s", String((RoadTrip ? Config.TripExpenses : Config.SeasonExpenses), 2).c_str());
  while(strlen(buf) < 7) strcat(buf, " ");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(10, 184); M5.Lcd.print(buf); 
  OilChange = OilChangeInterval - (Config.Mileage + DailyDistance - Config.OilChange);
  if((int)(OilChange / 1000) == 0)
    { sprintf(buf, "%4d ", (int)OilChange); strcpy(buf1, "m "); }
  else
    { sprintf(buf, "%4d ", (int)(OilChange / 1000)); strcpy(buf1, "Km"); }
  if(OilChange < 500) M5.Lcd.setTextColor(RED, TFT_BLUE);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(170, 184); M5.Lcd.print(buf); 
  M5.Lcd.setTextSize(2); M5.Lcd.print(buf1); M5.Lcd.setTextSize(3);
  
  // Display button options
  M5.Lcd.setTextSize(2);
  M5.Lcd.fillRect(0, 215, 320, 240, TFT_CYAN);
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setCursor(25, 220); M5.Lcd.print("  NEW            EXIT");
  while(millis() - entry < ScreenDelay)
  {
    // Run M5 update
    M5.update();
    if(M5.BtnA.wasPressed())
    {
      Beep();
      strcpy(buf, (RoadTrip ? "New Road Trip" : "Riding Season"));
      if(Confirm(buf, buf))
      {
        if(RoadTrip)
          Exclamation("Cool man!", "We're going on a", "Road Trip...");
        else
          Exclamation("Another Season", "Live To Ride", "Ride To Live");
        Music();
        delay(2000);
        if(RoadTrip)
        {
          Config.TripStart = now();
          Config.TripDistance = 0;
          Config.TripRiding = 0;
          Config.TripAvgSpeed = 0;
          Config.TripAvgCntr = 0;
          Config.TripMaxSpeed = 0;
          Config.TripExpenses = 0.00;
          WriteLogFile("New road trip started...");
        }
        else
        {
          Config.SeasonStart = now();
          Config.SeasonDistance = 0;
          Config.SeasonRiding = 0;
          Config.SeasonAvgSpeed = 0;
          Config.SeasonAvgCntr = 0;
          Config.SeasonMaxSpeed = 0;
          Config.SeasonExpenses = 0.00;
          WriteLogFile("New riding season started...");
        }
        Save_Config();
      }
      break;
    }
    if(M5.BtnC.wasPressed()) { Beep(); break; }
  }
}

//
// Confirm oil change
//
void Oil_Change(void)
{
  char          Buf[32], Buf1[32];
  double        OilChange;
  unsigned long entry = millis();

  // Am I due for an oil change?  I am if less then 500 Km
  OilChange = OilChangeInterval - (Config.Mileage - Config.OilChange);
  if(OilChange > (500 * 1000.0)) 
  {
    // Prompt user if he still wants to do an oil change
    if(!Not_Due())
    {
      // Reset timers and exit
      GPSTime = DweetTime = WiFiTime = millis();
      TextDisplay = false;
      return;    
    }
  }
  
  // Continue with oil change.
  M5.Lcd.fillRect(0, 53, 320, 165, TFT_BLUE);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  M5.Lcd.setTextSize(3);
  strcpy(Buf, "Confirm?");
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 70);
  M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
  strcpy(Buf, "Oil Change at");
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 110);
  sprintf(Buf, "%s Km", String(Config.Mileage/1000.0,  1).c_str());
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 150);

  // Display mileage of last oil change
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  sprintf(Buf, "Last was %s Km", String(Config.OilChange/1000.0, 1).c_str());
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 190);
  M5.Lcd.fillRect(0, 215, 320, 240, TFT_CYAN);

  // Display buttons
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setCursor(25, 220); M5.Lcd.print("  YES             NO");
  delay(500);
  WarningTone(0);  
  while(millis() - entry < ScreenDelay)
  {
    // Run M5 update
    M5.update();
    if(M5.BtnA.wasPressed())
    {
      Config.OilChange = Config.Mileage;
      Save_Config();
      WriteLogFile("Oil change performed at " + String(Buf1));
      break;
    }
    if(M5.BtnC.wasPressed()) break;
  }
  M5.update();
  Beep(); 

  // Reset timers
  GPSTime = DweetTime = WiFiTime = millis();
  TextDisplay = false;
}

//
// Oil change is not due yet, do one anyways?
//
bool Not_Due(void)
{
  char          Buf[32];
  double        OilChange;
  unsigned long entry = millis();

  // Clear display text area
  M5.Lcd.fillScreen(TFT_BLUE);
  M5.Lcd.fillRect(0, 0, 320, 52, TFT_CYAN);
  M5.Lcd.fillRect(0, 215, 320, 240, TFT_CYAN);
  M5.Lcd.drawLine(0, 25, 320, 25, BLACK);
  M5.Lcd.fillRect(0, 53, 320, 165, TFT_BLUE);
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setTextSize(2);
  sprintf(Buf, "Firmware %s", FIRMWARE);
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 5);
  StatusLine("Oil Change");

  // Display when oil changne is due
  OilChange = OilChangeInterval - (Config.Mileage - Config.OilChange);  
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  M5.Lcd.setTextSize(3);
  strcpy(Buf, "Oil Change");
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 70);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  strcpy(Buf, "only due in");
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 100);
  if(OilChange >= 1000.0)
    sprintf(Buf, "%s Km", String(OilChange/1000.0, 1).c_str());
  else
    sprintf(Buf, "%s m", String(OilChange, 0).c_str());
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 130);
  strcpy(Buf, "Continue anyway?");
  M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 175);

  // Display buttons
  M5.Lcd.fillRect(0, 215, 320, 240, TFT_CYAN);
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(25, 220); M5.Lcd.print("  YES             NO");
  delay(250); LowBeep();
  while(millis() - entry < ScreenDelay)
  {
    // Run M5 update
    M5.update();
    if(M5.BtnA.wasPressed()) { M5.update(); Beep(); return(true); }
    if(M5.BtnC.wasPressed()) break;
  }
  M5.update(); 
  Beep(); 
  return(false);
}

//
// Confirm dialog
//
bool Confirm(String Title, String Msg)
{
  char          Buf[64];
  unsigned long entry = millis();

  // Clear display area and prompt user
  M5.Lcd.fillScreen(TFT_BLUE);
  M5.Lcd.fillRect(0, 0, 320, 52, TFT_CYAN);
  M5.Lcd.fillRect(0, 215, 320, 240, TFT_CYAN);
  M5.Lcd.drawLine(0, 25, 320, 25, BLACK);
  M5.Lcd.fillRect(0, 53, 320, 165, TFT_BLUE);
  M5.Lcd.setTextColor(RED, TFT_CYAN);

  sprintf(Buf, "Firmware %s", FIRMWARE);
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 5);
  strcpy(Buf, Title.c_str());
  StatusLine(Buf);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  M5.Lcd.setTextSize(3);
  strcpy(Buf, "Confirm?");
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 90);
  M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
  M5.Lcd.drawString(Msg.c_str(), 160 - (M5.Lcd.textWidth(Msg.c_str())/2), 130);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  
  // Draw buttons
  M5.Lcd.setTextSize(2);
  M5.Lcd.fillRect(0, 215, 320, 240, TFT_CYAN);
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setCursor(25, 220); M5.Lcd.print("  YES             NO");
  delay(500);
  WarningTone(0);  
  while(millis() - entry < ScreenDelay)
  {
    // Run M5 update
    M5.update();
    if(M5.BtnA.wasPressed()) { Beep(); return(true); }
    if(M5.BtnC.wasPressed()) { Beep(); return(false); }
  }
}

//
// Exclamation dialog
//
bool Exclamation(String Msg1, String Msg2, String Msg3)
{
  char          Buf[32];
  unsigned long entry = millis();

  // Clear display area display text
  M5.Lcd.fillRect(0, 53, 320, 165, TFT_BLUE);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
  M5.Lcd.drawString(Msg1.c_str(), 160 - (M5.Lcd.textWidth(Msg1.c_str())/2), 70);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  M5.Lcd.drawString(Msg2.c_str(), 160 - (M5.Lcd.textWidth(Msg2.c_str())/2), 110);
  M5.Lcd.drawString(Msg3.c_str(), 160 - (M5.Lcd.textWidth(Msg3.c_str())/2), 150);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  Hide_Buttons();
}

//
// Display weather
//
void Weather(void)
{
  String        Text;
  char          Buf[64];
  unsigned long entry = millis();

  // Display city and current conditions
  M5.Lcd.fillRect(0, 0, 320, 24, TFT_CYAN);
  int i = City.indexOf(",");
  if(i != -1) strcpy(Buf, City.substring(0, i).c_str()); else strcpy(Buf, City.c_str());
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 5);
  Text = CurrentWeather.substring(CurrentWeather.indexOf(" : ")+3, CurrentWeather.indexOf(", Sunrise"));
  strcpy(Buf, Text.c_str());
  StatusLine(Buf);
  M5.Lcd.fillRect(0, 53, 320, 165, TFT_BLUE);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  M5.Lcd.drawLine(0, 25, 320, 25, BLACK);
  M5.Lcd.drawLine(154, 58, 154, 210, WHITE);
  M5.Lcd.drawLine(155, 58, 155, 210, WHITE);
  M5.Lcd.drawLine(0, 107, 320, 107, WHITE);
  M5.Lcd.drawLine(0, 108, 320, 108, WHITE);
  M5.Lcd.drawLine(0, 159, 320, 159, WHITE);
  M5.Lcd.drawLine(0, 160, 320, 160, WHITE);

  // Draw weather icon
  M5.Lcd.fillRect(2, 55, 150, 50, WHITE);
  Text = DailyForecast.substring(DailyForecast.indexOf("Icon: ")+6, DailyForecast.indexOf("\n"));
  Text = "/Weather/" + Text + ".bmp";
  if(SD.exists(Text.c_str()))
    M5.Lcd.drawBmpFile(SD, Text.c_str(), 50, 55);
  else
    WriteLogFile("Missing weather icon " + Text);    

  // Temperature/Humidity
  M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(170, 60); M5.Lcd.print("Temp    Humi");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  M5.Lcd.setCursor(170, 80); 
  Text = CurrentWeather.substring(CurrentWeather.indexOf("Temp ")+5, CurrentWeather.indexOf("C, Hum"));
  if(Text.length() < 4) M5.Lcd.print(" ");
  M5.Lcd.print(Text);
  M5.Lcd.setTextSize(2); M5.Lcd.print("C ");
  M5.Lcd.setTextSize(3);
  Text = CurrentWeather.substring(CurrentWeather.indexOf("dity ")+5, CurrentWeather.indexOf("%, "));
  if(Text.length() < 2) M5.Lcd.print(" ");
  M5.Lcd.print(Text);
  if(Text.length() < 3) { M5.Lcd.setTextSize(2); M5.Lcd.print("%"); }

  // Today's Min/Max temperatures
  M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 112); M5.Lcd.print("MinTemp");
  M5.Lcd.setCursor(170, 112); M5.Lcd.print("MaxTemp");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  M5.Lcd.setCursor(10, 132);
  Text = DailyForecast.substring(DailyForecast.indexOf("Min: ")+5, DailyForecast.indexOf(", Max"));
  if(Text.length() < 4) M5.Lcd.print(" ");
  M5.Lcd.print(Text);
  M5.Lcd.setTextSize(2); M5.Lcd.print("C");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(170, 132);
  Text = DailyForecast.substring(DailyForecast.indexOf("Max: ")+5, DailyForecast.indexOf(", Hum"));
  if(Text.length() < 4) M5.Lcd.print(" ");
  M5.Lcd.print(Text);
  M5.Lcd.setTextSize(2); M5.Lcd.print("C");

  // Sunrise/Sunset
  M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 164); M5.Lcd.print("Sunrise");
  M5.Lcd.setCursor(170, 164); M5.Lcd.print("Sunset");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  Text = DailyForecast.substring(DailyForecast.indexOf("Sunrise ")+8, DailyForecast.indexOf(" / Sunset"));
  M5.Lcd.setCursor(10, 184); M5.Lcd.print(Text);
  Text = DailyForecast.substring(DailyForecast.indexOf("Sunset ")+7, DailyForecast.indexOf(", Icon"));
  M5.Lcd.setCursor(170, 184); M5.Lcd.print(Text);
  M5.Lcd.fillRect(0, 215, 320, 240, TFT_CYAN);
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(25, 220); M5.Lcd.print("Forecast         EXIT");
  delay(500);

  // Loop here waiting for a button press or timeout
  while(millis() - entry < ScreenDelay)
  {
    // Run M5 update
    M5.update();
    if(M5.BtnA.wasPressed()) { Beep(); Forecast(); break; }
    if(M5.BtnC.wasPressed()) break;
  }
  M5.update();
  Beep(); 

  // Reset timers
  GPSTime = DweetTime = WiFiTime = millis();
  TextDisplay = false;
}

//
// Display 3 day weather forecast
//
void Forecast(void)
{
  String        Text, Forecast;
  int           Offset = 54;
  char          Buf[64];
  unsigned long entry = millis();
  
  // Extract city and weather condition text
  M5.Lcd.fillRect(0, 0, 320, 24, TFT_CYAN);
  int i = City.indexOf(",");
  if(i != -1) strcpy(Buf, City.substring(0, i).c_str()); else strcpy(Buf, City.c_str());
  M5.Lcd.drawString(Buf, 160 - (M5.Lcd.textWidth(Buf)/2), 5);
  StatusLine("3 day forecast");
  M5.Lcd.fillRect(0, 53, 320, 165, TFT_BLUE);
  M5.Lcd.setTextColor(WHITE, TFT_BLUE);
  M5.Lcd.drawLine(0, 25, 320, 25, BLACK);
  M5.Lcd.drawLine(100, 58, 100, 210, WHITE);
  M5.Lcd.drawLine(101, 58, 101, 210, WHITE);
  M5.Lcd.drawLine(0, 107, 320, 107, WHITE);
  M5.Lcd.drawLine(0, 108, 320, 108, WHITE);
  M5.Lcd.drawLine(0, 159, 320, 159, WHITE);
  M5.Lcd.drawLine(0, 160, 320, 160, WHITE);

  // Loop for each day
  Forecast = DailyForecast;
  for(int i=0; i<3; i++)
  {    
    // Draw weather icon
    Forecast = Forecast.substring(Forecast.indexOf("\n")+1);
    M5.Lcd.fillRect(2, 55 + (i * Offset), 96, 50, WHITE);
    Text = Forecast.substring(Forecast.indexOf("Icon: ")+6, Forecast.indexOf("\n"));
    Text = "/Weather/" + Text + ".bmp";
    if(SD.exists(Text.c_str()))
      M5.Lcd.drawBmpFile(SD, Text.c_str(), 28, 55 + (i * Offset));
    else
      WriteLogFile("Missing weather icon \"" + Text + "\"");    
  
    // Min/Max Temperature
    M5.Lcd.setTextColor(YELLOW, TFT_BLUE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(110, 60 + (i * Offset)); M5.Lcd.print("MinTemp  MaxTemp");
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(WHITE, TFT_BLUE);
    M5.Lcd.setCursor(110, 80 + (i * Offset)); 
    Text = Forecast.substring(Forecast.indexOf("Min: ")+5, Forecast.indexOf(", Max"));
    if(Text.length() < 4) M5.Lcd.print(" ");
    M5.Lcd.print(Text);
    M5.Lcd.setTextSize(2); M5.Lcd.print("C  ");
    M5.Lcd.setTextSize(3);
    Text = Forecast.substring(Forecast.indexOf("Max: ")+5, Forecast.indexOf(", Hum"));
    if(Text.length() < 4) M5.Lcd.print(" ");
    M5.Lcd.print(Text);
    M5.Lcd.setTextSize(2); M5.Lcd.print("C");
  }

  // Draw botton
  M5.Lcd.fillRect(0, 215, 320, 240, TFT_CYAN);
  M5.Lcd.setTextColor(RED, TFT_CYAN);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(25, 220); M5.Lcd.print("                 EXIT");

  // Loop here waiting for button press or timeout
  while(millis() - entry < ScreenDelay)
  {
    // Run M5 update
    M5.update();
    if(M5.BtnC.wasPressed()) break;
  }
  M5.update();
}
#endif

//
// END OF CODE
//
