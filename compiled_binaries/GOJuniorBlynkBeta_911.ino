/*************************************************************
 GOfermentor JR
 3/14/20 derived from NET930
 3/24/20 v902 partially working
 3/35/20 v904 SDLog ok but EEPROM n
 3/28/20 v905 EEPROM log ok also inflation fail warning
 3/30/20 v906 minor bug fixes
 4/1/20 v907 corrected to NET945
 4/3/20 v909 fixed github file bug
 4/4/20 v910 fixed provioning and reconnect issues. Ok for Thomas test
 4/8/20 v911 added logo, auto temp enable and better start for Thomas test

 *************************************************************/
//-------------------------- Blynk Init -------------------------------------------------
int BlynkLED;                               // provides current blynk connection status
#define USE_CUSTOM_BOARD                    // See "Custom board configuration" in Settings.h
// #define APP_DEBUG                           // Comment this out to disable debug prints
#define BLYNK_SSL_USE_LETSENCRYPT           // Comment this out, if using public Blynk Cloud
//#define BLYNK_PRINT Serial
#include "BlynkProvisioning.h"
BlynkTimer timer;                           // upto 16 
//------------------------------------------------------------
int const ver=911;
int const FERM_TYPE = 1;                    // 0 = net 1 = JR
#include <M5ez.h>
#include <M5Stack.h>
#include <ezTime.h>
#include "MegunoLink.h"                    // added smoothing filter for pressure
#include "filter.h"
ExponentialFilter<float> psiFilter(5, 0);   // not used - use instant reading
ExponentialFilter<float> tempFilter(5, 0);
#include "RTClib.h"
RTC_PCF8523 rtc;
#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1015 ads;     
#include <math.h>
#include <stdlib.h>
#include <Arduino.h>
// ----------------------BLYNK  -------------------------------------
WidgetLED blwrLED(V10);
WidgetLED ventLED(V11);
WidgetLED valvLED(V12);
WidgetTable table;
BLYNK_ATTACH_WIDGET(table, V0);                 // logs are stored here - 100 entries with autorollover
char tagName[13];                               // 12 char name from app
char sendTo[40];                                // 40 char email address for notifications
char textTo[40];                                // 40 char phone + carrier email
char phoneNum[11];                              // phone number for text message
#define WIFI_RETRY 120                          // seconds to retry connection to WiFi if lost
#define BLYNK_BACKGROUND "#212226"
boolean lastAppConnectState;                   // set to true if connected
// --------------------  I/O pins --------------- 

const int VACPin = 5;                          //  VAC R1 HIGH => VAC SELECTED
const int AIRPin = 2;                          //  AIR R2 HIGH => PUMP ON
const int VALVEPin = 13;                         //  HX valve HIGH => OPEN

// -------------------- flags
#define ALL_OFF 0
#define INFLATE 1
#define DEFLATE 2
boolean AIR_ON = false;                        // flag to switch airpump on
boolean VAC_ON = false;                        // flag to set vacuum 
boolean VALV_ON = false;                       // flag to open HX valve

//  ---------    EEPROM -----------------------------------------
int MAXLOG=12;                                // maximum stored events 5 bytes= (4time+1eventcode) MAXLOG=12 for EEPROM
#define MAXCODES 24                           // maximum defined event codes
#define SSIDADROFFSET 64                      // SSID stored 64 - 95  - 32 bytes
#define LOGADDRSTART 96                       // start of log data in EEPROM - leave 64 bytes  + 32 bytes oldSSID for parameters
#define SPROWS 5                              // editable SP data rows
#include "EEPROM.h"
int addr = 0;                                 // EEPROM starting address
#define EEPROM_SIZE 256                       // 48 bytes used for parameters
int SP[6][8]={{1,0,4,0,0,0,1,1},              // factory default configuration            
              {2,3,7,5,0,0,0,0},              // punch settings
              {15,10,10,5,0,0,0,0},           // press settings 
              {123,0,0,0,1,0,0,0},            // stored time data
              {0,150,0,0,0,0,0,0},            // temperature settings    
              {0,0,0,0,0,0,0,0}};             // misc - not editable                
              
int SP_EEPROM[][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}}; // shadowed in RAM

// --------- Wifi/BLE communication variables -----------------------------------              
int PV[8] = {0,0,0,0,0,0,0,0};
const char* PVName[8] = {"State","PsigP","PsigV","BLWR","AIR_ON","VAC_ON","CDwnsecs","NextPmins"};

const char* SPName[][8]= {{"ID","AUTO","P/24hr","Operation","","","AutoWiFi","WiFiEnbl"},        
                          {"PunchInfMins","PunchPsigx10","PunchDefMins","PunchTOmins","","","",""},            // punch
                          {"PressInfMins","PressPsigx10","PressDefMins","PressTOmins","","","",""},            // press
                          {"TZhr","TZmin","DST","HR","MIN","MONTH","DAY","YY"},                                  // time          
                          {"TempControl_ON","TSPx5","DEGF","TempEnbl","TCTRLMode","","",""},       // temperature
                          {"LOGcnt","WiFiRst","UpdateESP","StartBtn","","","",""}};                         // misc configuration - not editable
int SPmin[][8]={{1,0,1,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{104,0,0,0,0,1,1,19},{0,25,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};     // not stored
int SPmax[][8]={{99,1,8,2,0,0,1,1},{10,5,20,30,0,0,0,0},{30,20,60,30,0,0,0,0},{152,59,1,24,59,12,31,30},{1,200,1,1,2,0,0,0},{MAXLOG,1,0,0,0,0,0,0}};    // not stored
int PunchT[]={0,0,0,0,0,0,0,0,0,0,0,0};         // minute of day for punch schedule
char* Statename[12]={"READY       ","","INFLATING..  ","INFLATING..  ","","DEFLATING..  ","","","","COMPLETE   ","CANCELLING..","FAILING.."};
const char* Alarmname[8]= {"NOWIFI","BAD_TEMP","HI_TEMP","LO_TEMP","BAD_psi","HI_psi","TIMEOUT","NOCLOUD"};   // these are current conditions
const char* Codename[MAXCODES]= {"NULL","PunchStarted","PunchComplete","PunchCancelled","PunchFailed","autoPunchStarted",
                    "PressStarted","PressComplete","PressCancelled",
                    "temp failed","tempHiALM","tempLoALM","TConON","TConOFF","temperatureOK",
                    "autoPunchON","autoPunchOFF",
                    "PsiFailALM","PoweredON","overPressure!","AppConnected","AppDisconnected","Requires CONFIRM on UNIT",""};
// log codes:  1 =punch started 2=punch complete 3=punch cancelled 4= punchfailed 5=autopunchstarted
// logcodes:   6 = press started 7=press complete 8=press cancelled
// log codes: 9=temp failed 10=temp highalm 11=temp lowalm 12=tempcntrlON 13=tempcntrOFF 14=sensorOK
// log codes  15=autopunch on 16 autopunchOFF
// log codes  17=psiFailALM 18= power ON 19= overpressure 20=app connected 21 app disconnected, 22 press release22-23 ... spare
// TZ must be positive 128 = 0  123 = UTC-5 ...
// TCTRLMode = 0 no temperature control function, = 1 cooling only, = 2 heating only
// TSP is stored in degCx5 so 40C = 200

// ----  constants
#define x_datum 120                             // left edge of bag in graphic
#define y_datum 30
#define PSIX10_DB 1                             // pressure control db in psix10
#define SERIESRESISTOR 10000                    // resistor used for thermistor
#define MINCOOLINGVLVSECS 30                    // minimum secs between cooling valve cycles
#define FILTER_T_TIME_CONST   5.0                 // time constant: time in samples to reach ~ 63% of a steady value
                                                // the rise time of a signal to > 90% is approx 2.5 times this value
                                                // at 3kHz sampling a value of 100 == rise time to 95% of approx 100mS                                        
#define FILTER_WEIGHT_T (FILTER_T_TIME_CONST/(FILTER_T_TIME_CONST +1.0))
#define SPLASHSECS 30                           // time splash screen is displayed

// ------------------- variables
int State=0;                                    // state machine 0=initializing 1=ready 2=venting 3=pressing 4=resetting 5=cancelling
int Operation=0;                                // 0=punch 1=press
int nextPunch_mins;                             // time to next punch in minutes
int psix10_PV;                                  // pressure sensor readings
int psix10_SP;                                  // pressure SP
boolean highPressure;
int incT;                                       // temperature incrementor  3 for F and 5 for C
int tempCx5;                                    // convert temp to intx5
int pressCycle=1;                               // press cycle number
// -------------------- analog variables
float tempC=0.0;                                // bag temperature in deg C averaged
float psi=0.00;                                 // pressure reading psi
float zeroPoffset=0.0;                          // zero offset in pressure - set to read value on power up
// -------            housekeeping --------------------------------
int msgnum = 0;
char buffer[40]="";                             // display string
char outStr[40]="";
boolean debug=true;                             // send output to serial
boolean simulate=false;                         // speed up cycle for testing
boolean RTCFound=false;                         // will be autodetected
boolean SDFound= false;                         // will be autodetected
boolean WiFiEnabled=true;                       // in setup 
boolean updateApp=true;                         // update Blynk app
boolean BlynkAppConnected=false;
boolean BlynkAppLatch=false;                    // for app message
int counter=0;
int historyCtr=0;                               // update Blynk temp history
int startsecs=500;                              // startup delay 0.1 secs
boolean latchStart = false;                     // holds off countdpown until at set pressure
boolean presshold = false;                      // set true when waiting for permission to start PRESS
uint64_t chipid;                                // unique ESP chip ID
byte mac[6];                                    // the MAC address of M5stack
char chipStr[40];
boolean firstpass = true;                       // for initializing on blynk start
boolean displayOff = false;                     // setting true disables the displayloop
int netWorksFound=0;                              // numNetworks detected
int netNumber = -1;                             // index number of saved network in network list -1 means no network found
String savedSSIDStr;
// ------------     timing    ---------------------------------------    
int BLWROffsecs;                                // high pressure blower off countdown
int CVLVOffsecs=30;                             // 
int minutes_today=0;                            // minutes since midnight 0 - 1440
int elapsedsecs=0;                              // counts up in seconds from each state change
long countdownsecs;                             // counts down in seconds from each state change
int onesec = 1000;
int secspermin = 60;                            // seconds per minute
boolean gotTimeSync = false;                    // set to true if internet time OK
Timezone myTZ;                                  // set the local timezone
int TZoffset;
time_t ts_secs;                                 // time since Jan 1 1970
unsigned long Esecs = millis();                 // startup elapsed timer
// --------------  alarms ---------------------------------------------
boolean tempLow = false;boolean tempLowLatch=false;
boolean tempHigh = false;boolean tempHighLatch=false;
boolean overPressure=false;boolean overPressureLatch=false;
boolean psiSensorFailed=false;boolean psiFailLatch=false;
boolean tempSensorFailed=false;boolean tempSensorLatch=false;

boolean WiFiConnected=false;
boolean internetAvailable=false;
boolean BlynkConnected=false;                   // will be refreshed -connected to cloud not app
int BlynkOffCount = 0;                          // seconds blynk connection is off
// ---------  logging
unsigned long logSecs[102];
int logCode[102];
int logOrder[102];
int logIndex = 0;                               // for blynk table
// -----------  include files ----------------------------------------
#include "GOEEPROMsupport.h";
#include "GO_IOTblynk.h"
#include "SD_Utilities.h"
#include "GO_JR.h"
// -----------------------------  SETUP ------------------------------

void setup() { 
  if (debug) Serial.begin(115200);                            // initialize serial communication debugger
  delay(1000);
  pinMode(VACPin,OUTPUT);digitalWrite(VACPin,LOW);          // LOW => off
  pinMode(AIRPin,OUTPUT);digitalWrite(AIRPin,LOW);          // LOW => off`
  pinMode(VALVEPin,OUTPUT);digitalWrite(VALVEPin,LOW);        // LOW => off
  State = 0;                                                  // always power up in idle 
  // ---- start up ADC and RTC on I2C
  ads.setGain(GAIN_ONE);                                      // 1x gain   +/- 4.096V  1 bit = 2mV     
  ads.begin();                                                // start the ADC
  rtc.begin();                                                // start RTC. does NOT detect if RTC is present  
  delay(1000);
  DateTime RTCpowerup = rtc.now();                            // get time from RTC
  if ((RTCpowerup.year() > 0) && (RTCpowerup.year() < 2155 )) RTCFound = true;              // check if rtc is present  
  // --- start M5ez -----
  ez.begin();  
  ez.header.remove("clock");                                  // remove the clock widget - will replace with own in header
  ez.canvas.scroll(false);                                    // turn scroll off to prevent crashes// start M5ez engine 
  M5.Lcd.fillScreen(TFT_BLACK); // Black screen fill
  M5.Lcd.drawXBitmap(70, 30, logo, logoWidth, logoHeight, TFT_WHITE);     
  // --------- get M5stack ID ------
  chipid=ESP.getEfuseMac();                                   // The chip ID is essentially its MAC address(length: 6 bytes)
  sprintf(chipStr,"%lu",chipid);

//  ---   read all EEPROM data ------------------
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM"); delay(1000000);
  }
  // !!!!!!!!!!!!!!!    uncomment next 2 lines to force EEPROM rewrite
  //eraseEEPROM();                                          // uncomment to erase EEPROM
  //eraseEEPROMLog();
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  int ID = EEPROM.read(0);                                    // read the ID to detect blank EEPROM 
  // uncomment below to force default reprovision
  //ID= 0;
  
  SDFound = detectSD(SD, "/", 0); 
  if (SDFound) {
    boolean fileFound = false;
    fileFound =  checkFile(SD, "/eventlog.txt");
    // fileFound = false;                                        // uncommento erase SD card !!!!!!!!!!!!!!!!!
    if (!fileFound) {                                         // check eventlog.txt exists. If not create it
      strcpy(buffer,"ESP32# ");
      strcat(buffer,chipStr);                                 // put unique chp ID in first line
      strcat(buffer," \r\n");
      Serial.print ("new file->"); Serial.println(buffer);      writeFile(SD, "/eventlog.txt", buffer);                     // create file if new 
    }  
    MAXLOG = 12;                                              // increase log capacity due to SD
    Serial.println("------");
  }
  
  Serial.print("ID");Serial.print(ID);
  Serial.print(" RTC= "); Serial.print(RTCFound); 
  Serial.print(" SD = ");Serial.println(SDFound);             // 

  
  if ((ID < SPmin[0][0])| (ID > SPmax[0][0])){                // check if EEPROM was blank = ID invalid
    defaultEEPROM();                                          // write into the EEPROM value in the default menuSP array
    SP[5][1] = 1;                                             // needs to be provisioned - new device
    saveEEPROM();
    if (debug) Serial.println("set to default");
  }
  
  readEEPROM();                                               // reads the EEPROM inoto SP_EEPROM array
  loadEEPROM();                                               // read the SP_EEPROM array into SP array
//  printSPArray();                                             // list SP array
  Serial.print("RESET   =");Serial.println(SP[5][1]);  
  Serial.print("AUTO_ON =");Serial.println(SP[0][6]); 
  Serial.print("WIFI_ON =");Serial.println(SP[0][7]);   
  WiFiEnabled = SP[0][7];                                   // read WiFi option at power up
  SP[5][3]= false;                                          // force start button off
 

  netWorksFound = WiFi.scanNetworks(); 
  savedSSIDStr = getSSID_EEPROM();
  Serial.print("read SSID");Serial.println(savedSSIDStr);
  if (savedSSIDStr.length() < 1) {                          // no saved SSID at all                                     
    if (netWorksFound > 0) SP[5][1];                        // networks are available - setup for reset
    else WiFiEnabled = false;                               // switch to pure non WiFi operation
  }
  else {
    netNumber = checkNetworks(savedSSIDStr);                // gets -1 if saved network not detected    
  }
  Serial.print("networks=");Serial.println(netWorksFound);
  Serial.print("savedSSID=");Serial.println(savedSSIDStr);
  Serial.print("netnumber=");Serial.println(netNumber);
  showSplashScreen();                                       // give a chance for user to change SETUP
  // ================================================================== 
  if (WiFiEnabled) { 
    if (netNumber > -1) {                               // found the stored SSID so connect
      if (SP[5][1]==0) {
        BlynkProvisioning.begin();       // provision - network exists RESET not selected
      }
      else {
        if (netWorksFound >0) {                 // force reprovision from scratch - must have at least one network available
          BlynkProvisioning.begin();
          BlynkState::set(MODE_RESET_CONFIG); 
          ez.screen.clear();
          ez.msgBox("Add device to App: ","In App | +add New Device | Click Ready","" ,false, &FreeSans18pt7b, TFT_BLUE);      
        }
      }
    }   
    else {                                    // saved network was not found
      if (SP[5][1]) {
        if (netWorksFound >0) {                 // force reprovision from scratch - must have at least one network available
          BlynkProvisioning.begin();
          BlynkState::set(MODE_RESET_CONFIG); 
          ez.screen.clear();
          ez.msgBox("Add device to App: ","In App | +add New Device | Click Ready","" ,false, &FreeSans18pt7b, TFT_BLUE);      
        }
      }
      else if (SP[0][6]) WiFiEnabled = false;    // switch to no WiFi for now                                                     
    }
  }                             // end WiFi enabled code
  if (!WiFiEnabled) {           // no WiFi so finish setup now and get time from RTC
    if (RTCFound) {                             // must have an RTC to use it
      DateTime RTCnow = rtc.now();                // get time from RTC
      myTZ.setTime(RTCnow.hour(),RTCnow.minute(),RTCnow.second(),RTCnow.day(),RTCnow.month(),RTCnow.year());                // set the ESP clock 
    }  
    if (checkEEPROM()) {
      saveEEPROM();
      readEEPROM();                                               // reads the EEPROM inoto SP_EEPROM array
      loadEEPROM();                                               // read the SP_EEPROM array into SP array
    }
 // -----  calculate punch timings and IO
    calcPunch();
    zeroPoffset = readPressure(false);                       // get the pressure offset
    zeroPoffset = 0;
    if (!SP[4][3]) SP[4][3] = checkForTempProbe();                      // probe found then autoenable                                  
    if (SP[4][3]) {
      for (int i=0;i<100;i++) {
        tempC = readTemp();                                  // readTemp and load up filter
        float tempInstant = readTemp();
        tempFilter.Filter(tempInstant);
        tempC = tempFilter.Current();    
      }
    }   
    setupDisplayandTimers(); 
    Serial.println("NON-WIFI SETUP DONE");
    add2Log(UTC.now(),18);                              // record time powered ON in TZ even though set as UTC !   
  }
  timer.setInterval(1000L, checkBlynk);              // make sure connection is running 
}



//========================================= MAIN LOOP ==============================================
void loop() { 
if (WiFiEnabled) {
    BlynkProvisioning.run();
    if (BlynkLED < 5) Serial.println(BlynkLED);
    if (BlynkLED == 0) Esecs = millis();
    if (BlynkLED == 5) {                                    // Blynk is running
      if (SP[0][6]) SP[0][7] = true;                        // autoenable if connected
      if (firstpass) {
        if (SP[5][1] | !(SP[0][0])) { 
          Serial.println("-- reset parameters");
          SP[0][0]=1;                                       // reset if newly provisioned
          SP[5][1]=0;                                       // reset if newly provisioned
          saveEEPROM();                                     // save immediately
          readEEPROM();                                     // reads the EEPROM inoto SP_EEPROM array
          loadEEPROM();                                     // read the SP_EEPROM array into SP array
        }
        int n = saveSSID_EEPROM();                          // save the Blynk SSID for  next boot  
        ez.msgBox("Connecting to internet: ","Waiting for sync...","" ,false, &FreeSans18pt7b, TFT_RED);
        setInternetTime(60);
        Serial.print("LOCAL=");Serial.println(myTZ.dateTime("Y,m,d,H,i,s"));
        if (WiFi.status() == 3) WiFiConnected = true;
        if( debug) printWiFiStatus();
    //    timer.setInterval(1000L, checkBlynk);              // make sure connection is running
        if (SP[5][2]) mainmenu_ota();                             // firmware download
        // -----  calculate punch timings and IO
        calcPunch();
        zeroPoffset = readPressure(false);                       // get the pressure offset
        zeroPoffset = 0;
        if (!SP[4][3]) SP[4][3] =  checkForTempProbe();         // autocheck for temp probe                                  
        if (SP[4][3]) {
          for (int i=0;i<100;i++) {
            tempC = readTemp();                                  // readTemp and load up filter
            float tempInstant = readTemp();
            tempFilter.Filter(tempInstant);
            tempC = tempFilter.Current();    
          }
        }   
        setupDisplayandTimers();
        Serial.println("BLYNK SETUP DONE");
        add2Log(UTC.now(),18);                              // record time powered ON in TZ even though set as UTC !             
        if (Blynk.connected()) {
          initBlynk();
          updateBlynkSP(true);
          Blynk.syncAll();
        }
        firstpass = false;
      }     // end first pass
    }       // end LED=5
    if (!firstpass) pollButtons();                         // keep checking buttons 
    timer.run();
    Blynk.run();
  }
  else {                                                   // WiFi not enabled option
    pollButtons();
    timer.run();  
  }
}

//==================================  END LOOP ===========================================


// ------ physical IO -----------------------------------------
float readPressure(boolean UseOffset) {                          // reads pressure sensor and returns in psi
 // if UseOfset is true then compensate for zero offset
  int16_t mV;
  float psi = 0;
  psiSensorFailed= false;    
  overPressure = false; 
  mV = ads.readADC_SingleEnded(0)*2.0;
  if ((mV > 100) && (mV < 4900)) {
    psi = 1.25 * (mV - 500)/1000.0;    
    if (UseOffset) psi = psi - zeroPoffset;               // remove the zero offset
    if (psi <0.0) psi =0.0; 
    if (psi > (SP[1][2]+2)) overPressure=true; else overPressure=false; // high pressure warning psi above setpoint
  }
  else psiSensorFailed = true;
  return psi;
}

float readTemp() {          // reads 10K thermistor and uses Steinhar-Hart eqn
 // uses Steinhart-Hart A,B,C for omega type 44031 thermistor
  tempSensorFailed = true;
  int16_t mV =0;
  int16_t mVsum = 0;
  float Temp = 0;
  float R;                  // resistance in ohm
  boolean invalid = true;
  int cnt = 0;
  for (int i = 0;i<5;i++) {
    mV = ads.readADC_SingleEnded(2)*2.0;                // read ADC2 on JR
    if (mV < 4092)  {     // only valid readings
      cnt++;
      mVsum = mVsum + mV;
    }
 //   Serial.print("mV=");Serial.print(mV);Serial.print(" ");Serial.println(cnt);
  }
  if (cnt >0) {                                               // ensure no 0 divide                        
    mV = mVsum/cnt;                                           // average the valid readings 
    if ((mV > 100) && (mV < 4500)) invalid=false;             // check range
    
    if (!invalid) {                                            // valid reading
   //   Serial.print(" Temp_mV= ");Serial.println(mV);
      float voltage = mV/1000.0;
      R = SERIESRESISTOR * (5.0/voltage - 1.0);
   //   Serial.print("R= ");Serial.println(R);
      float logR = log(R);
      Temp = 1/ (0.001032 + 0.0002387 * logR + 0.0000001580 * logR *logR * logR); 
      Temp -= 273.15;                              // convert to degC
   //   Serial.print("TC= ");Serial.println(Temp);   
      if ((Temp > 1.0) && (Temp < 50.0)) tempSensorFailed = false;   // temp valid
    }
  }   
  return Temp;
}

int checkForTempProbe() {
  // gives true if sensor found
  int found = false;
  float TempC = readTemp();
  if (!tempSensorFailed) found = true;
  return found;
}


void checkAlarms() {                          // check and notify alarms
  if (psiSensorFailed && !psiFailLatch) {
    add2Log(UTC.now(),17);
    psiFailLatch = true;
  }
  if (!psiSensorFailed) psiFailLatch = false;
  if (overPressure && !overPressureLatch) {
    add2Log(UTC.now(),19);
    overPressureLatch = true;
  }
  if (!overPressure) overPressureLatch = false;
  if (SP[4][3]) {                             // temperature sensor enabled
    if (tempSensorFailed && !tempSensorLatch) {
      add2Log(UTC.now(),9);
      tempSensorLatch = true;
    }
    if (!tempSensorFailed && tempSensorLatch) {
      add2Log(UTC.now(),14 );                 // recovered
      tempSensorLatch = false;
    }
    if (!tempSensorFailed) {                  // temp valid
      if (SP[4][4]) {                         // control on
        if (tempHigh && !tempHighLatch){
          add2Log(UTC.now(),10);
          tempHighLatch = true;
        }
        if (!tempHigh && tempHighLatch) {
          add2Log(UTC.now(),14 );            // recovered
          tempHighLatch = false;
        }  
        if (tempLow && !tempLowLatch){
          add2Log(UTC.now(),11);
          tempLowLatch = true;
        }
        if (!tempLow && tempLowLatch) {
          add2Log(UTC.now(),14 );            // recovered
          tempLowLatch = false;
        }  
      }
    }
  }
}

void Outputs(int action) {  
  switch(action) {
    case 0: // idle state
       AIR_ON=false;
       VAC_ON=false;                
       break;
    case 1: // inflate
       AIR_ON=true;
       VAC_ON=false;
       break;
    case 2: // deflate 
       AIR_ON=true;
       VAC_ON=true;
       break;
  }    
}


// =================================  SUBROUTINES ====================================
void setupDisplayandTimers() {                               // setup main screen and start all timers
  ez.screen.clear();
  State=0;
  showHeaderez();
  showStatusez();
  showMainBtn();
  showIndicators();                                 // display the status indicators
  timer.setInterval(10000L, sec10Loop);
  timer.setInterval(1000L, onesecLoop);
  timer.setInterval(500L, fastLoop);
  timer.setInterval(1000L, processLoop);
  timer.setInterval(1000L, displayLoop);
  showBag(); 
}


void pollButtons() {
  String btnpressed = ez.buttons.poll();
  if (btnpressed != "") {
    Serial.print("BUTTON pressed= ");Serial.println(btnpressed);   
    if (btnpressed =="MODE") {
      if (SP[0][1]){
         SP[0][1]=false;
      }
      else {
         SP[0][1]=true;        
      }
    }
    if (btnpressed == "START") {
      if (State == 0) {
        SP[5][3]=1;                                // trigger punch or press 
      }
    }
    if ((btnpressed == "CANCEL") & (State>1) & (State<10) & (elapsedsecs > 2)) {
      countdownsecs = 5;
      ez.buttons.show(" # WAIT # ");
      State = 10;                                 // cancel step
    }  
    if (btnpressed == "OPER") {                   // select punch or press or deflate operation
       if (SP[0][3] < SPmax[0][3]) SP[0][3]++;else SP[0][3] = SPmin[0][3];
    }
    if (btnpressed == "TC") {
      if (SP[4][3]==1) {                               // temperature enabled
        setTempC();
      }
    }
    if (btnpressed == "CHANGE") {
      if (SP[0][2] < SPmax[0][2]) SP[0][2]++;else SP[0][2] = SPmin[0][2];
      showNextPunch();                        // update display
    }
    if (btnpressed == "LOG") {
      if (debug) printOrderedLog();
      showLog();
      showBag();
    }
    if (btnpressed == "OK") {
      if (State == 11) countdownsecs = 0;      // inflation timeout hold
    }
  }    
}                                               // end pollButton
// --------------------------------------------- NEW -----
int checkNetworks(String SSIDStr) {
  // scans al available networks returns the number of the network == to the savedSSID
  //  return -1 if not found
//  String SSIDStr = getSSID_EEPROM();
  int netsFound=-1;
// Serial.print("config");Serial.println(configStore.wifiSSID);

//  Serial.print("savedSSID=");Serial.print(SSIDStr);Serial.println("|");
//  Serial.println(SSIDStr.length());
  netWorksFound = WiFi.scanNetworks();          // rescan to get fresh list
//  Serial.println(netWorksFound);  
  for (int i=0;i<netWorksFound;i++) {
//    Serial.println(WiFi.SSID(i));
    if (SSIDStr.equals(WiFi.SSID(i))) {
      netsFound = i;                                        // get matching network -1 if none
    }
  }
  return netsFound;
}  

int saveSSID_EEPROM() {
  // get the current BLYNK SSID and save it in EEPROM
//  String SSID = configStore.wifiSSID;
  char buffer[40];
//  configStore.wifiSSID.toCharArray(buffer, configStore.wifiSSID.length());
strcpy(buffer,configStore.wifiSSID);
//Serial.print("stored================================");Serial.println(buffer);
  int n=strlen(buffer);
//  int n = SSIDStr.length();
  Serial.println(n);
  if (n > 0 ) {
    for (int i=0;i<n;i++) {
      EEPROM.write(SSIDADROFFSET+i,buffer[i]);
//      Serial.print(i);Serial.print("=");Serial.print(buffer[i]);
    }  
    for (int i=n;i<32;i++) {
      EEPROM.write(SSIDADROFFSET+i,0);
    }
    EEPROM.commit();
    delay(100);
 
  }  
//  Serial.println("|");
  return n;                             // 0 if empty 
}

String getSSID_EEPROM () {
  char buffer[40];
  byte ch;
  int n = 0;
  for (int i=0;i<32;i++) {
      ch = byte(EEPROM.read(SSIDADROFFSET+i));
      if ((ch < 127) && (ch > 31)) {
        buffer[i]=ch;
        n = i;
    //    Serial.print(i);Serial.print("=");Serial.print(buffer[i]);
   //     readString += char(ch);
      }
        else break;
   //    Serial.print(i);Serial.print("=");Serial.println(ch);
  }
 // Serial.println();
  buffer[n+1] = '\0';
  String readString = buffer;
//  Serial.print("buffer=");Serial.println(buffer);
//  Serial.print("string=");Serial.println(readString);

  return readString;
}

//----------------------------------------------  TIMERS ----------------------------------------

void checkBlynk() {
  if (BlynkOffCount > 0) {
 //   Serial.print("Blynk counter=");Serial.println(BlynkOffCount);
  }
  if (!Blynk.connected()) {
    if (BlynkOffCount < WIFI_RETRY) BlynkOffCount++;else BlynkOffCount = 0;
    if (BlynkOffCount >= WIFI_RETRY) {
      Serial.println("retrying..");   
      int netnum = checkNetworks(savedSSIDStr); 
      Serial.println(netnum);
      if ((checkNetworks(savedSSIDStr) > -1)&& SP[0][6]) {         // check if network saved is up
        Serial.println("trying to connect...");
        BlynkProvisioning.begin();
        WiFiEnabled = true;
      }  
    }   
    if ((BlynkAppConnected) & (!lastAppConnectState)) {
        initBlynk();
        updateBlynkSP(true);
        lastAppConnectState = BlynkAppConnected;
    }
  }
  else BlynkOffCount = 0;  
}


void sec10Loop() {
  if (Blynk.connected()) updateBlynkTempPressPV();
  historyCtr++;
  if (historyCtr > 10) {
    historyCtr = 0;
    if (Blynk.connected()) updateBlynkTempHistory();
  }
}

void onesecLoop() {                        // one second loop
  // runs every second to get time , check Wifi and countdown, elapsed timers
  int period = 1000;
  boolean needUpdate = false;
  if (!simulate) {
    minutes_today = minutesSinceMidnight();
    secspermin =60;                           // 60 seconds per minute
  }
  else {                                      // simulate time 1 sec = 1 minute
    if (minutes_today > 1440) minutes_today = 0;
    else minutes_today++;
    secspermin = 1;                           // 1 sec per minute     
  } 
  if (latchStart) countdownsecs--;            // countdown the timer
  elapsedsecs++;                              // count up the elapsed 
  if (WiFiEnabled) {
    if (Blynk.connected()) {
      updateBlynkDisplay();
      updateBlynkSP(false);                          // update Blynk SP parameters
      /*
      if (BlynkAppConnected & !BlynkAppLatch) {
        logIndex = -1;
        BlynkAppLatch = true;
        add2Log(UTC.now(),20);                    // record app connected
      }
      if (!BlynkAppConnected &BlynkAppLatch) {
        BlynkAppLatch = false;
        add2Log(UTC.now(),21);
      }
      */
    }
  }  
  if (SP[0][1] != SP_EEPROM[0][1]) {
      if (SP[0][1] == 1) add2Log(UTC.now(),15);else add2Log(UTC.now(),16);
  }
  if (SP[4][3]) {
    if (SP[4][4] != SP_EEPROM[4][4]) {
        if (SP[4][4] == 1) add2Log(UTC.now(),12);else add2Log(UTC.now(),13);
    }    
  }
  needUpdate = checkEEPROM();                              // see if EEPROM needs updating 
  if (needUpdate) {
    saveEEPROM();   
    readEEPROM();                                               // reads the EEPROM inoto SP_EEPROM array
    loadEEPROM();                                               // read the SP_EEPROM array into SP array// save updated values
    Serial.println("EEPROM updated");
  }
  if (needUpdate) calcPunch();                              // recalculate the autopunch schedule
  if (WiFi.status() == 3) WiFiConnected = true; else WiFiConnected = false;    // check the WiFi connection 
// if (debug) printState(); 
}


void processLoop() {
  // runs every second - check if autopunch time and run any sequences
  int period = 1000;
  // ----  check autopunch --  
  if ((SP[0][1]>0) & (State == 0) & (SP[0][3] == 0)) {           // if autopunch is active and waiting
//     Serial.print("APSP");Serial.println(SP[0][2]);
//     Serial.print("minute=");Serial.println(minutes_today);
     nextPunch_mins = 1440;
     for (int i=0;i<SP[0][2];i++) {
        int delta = PunchT[i] - minutes_today;
 //       Serial.print(PunchT[i]);Serial.print(" ");
 //       Serial.println(delta);
        if (delta == 0) {
           SP[5][3] = true;                    // set punch now
           add2Log(UTC.now(),5);               // record start of autopunch 
        }
        else {                                // calculate minutes to next
          if ((delta >0) & (delta < nextPunch_mins)) nextPunch_mins = delta;         
        }
     }
     if (nextPunch_mins == 1440) nextPunch_mins = 1440 - minutes_today;   // time to midnight    
  }  // end autopunch check
  switch (SP[0][3]) {                             // different sequences based on SP[0][3] (operation)
    case 0:                                       // punch operation    
      punchSequence();
      break;
    case 1:
      pressSequence();
      break;
    case 2:  
      deflateSequence();     
      break; 
  }  
}


void displayLoop() {
    int period = 1000;
    showPsi();      
    showHeaderez();  
    showStatusez();
    showMainBtn();
    showIndicators(); 
    showBag();                   
}

void fastLoop() {
  // gets pressure readings and sets digital outputs
  int period = 500;                                      // execute every period ms
  int tempCx5;
  float tempSPC = SP[4][1]/5.0;                 // remember TSP is stored in degCx5
  psi = readPressure(0);                           // read pressure on sensor 0 (top)
//  psiFilter.Filter(psi);                                 // run it through filter
//  psix10_PV = psiFilter.Current()*10;                   // use filtered value and convert  to x10 integer
  psix10_PV = psi*10;                                 // use instant value
  int ERR = psix10_SP - psix10_PV; 
  if (ERR>PSIX10_DB) highPressure = false;              // work in db
  if (ERR<-PSIX10_DB) highPressure = true;
  if (AIR_ON) {                                         // shut airpump off if highpressure
    if ((highPressure)&(!VAC_ON)) {
      if (latchStart) digitalWrite(AIRPin,LOW);         // do not shut off until latchStart reached
    }
    else digitalWrite(AIRPin,HIGH);
  }
  else digitalWrite(AIRPin,LOW); 
  if (VAC_ON) digitalWrite(VACPin,HIGH);else digitalWrite(VACPin,LOW);
  // ---- temperature code ----
  if (SP[4][3]) {                                // temperature sensor enabled
    int newVal= readTemp();
    tempC = (FILTER_WEIGHT_T * tempC ) + (1.0-FILTER_WEIGHT_T) * newVal ;       // exponentail filter
    if (SP[4][4]>0) {                            // HX control enabled
      if (SP[4][0] & !tempSensorFailed) {               // temperature control ON and valid temperature reading
         if (SP[4][4] == 1) {                    // set for cooling
            if (tempC > (tempSPC+2.0)) tempHigh = true;            
           if (tempC <= (tempSPC+0.5)) tempHigh = false;
           if (tempC < (tempSPC-2.0)) tempLow = true; 
           if (tempC > (tempSPC-0.5)) tempLow = false;
           if (tempC > tempSPC) {
              VALV_ON = true;
           }
           else if (tempC <(tempSPC-1.0)) VALV_ON = false;
         }
         if (SP[4][4] == 2) {                    // set for heating
           if (tempC > (tempSPC+2.0)) tempHigh = true;            
           if (tempC <= (tempSPC+0.5)) tempHigh = false;
           if (tempC < (tempSPC-2.0)) tempLow = true; 
           if (tempC > (tempSPC-0.5)) tempLow = false;
           if (tempC < tempSPC) {
              VALV_ON = true;
           }
           else if (tempC >(tempSPC+1.0)) VALV_ON = false;
         } 
         else VALV_ON = false;             // temperature control disabled kill valve if open         
      }
      else VALV_ON = false;                     // keep HX valve shut
    } 
    if (VALV_ON) digitalWrite(VALVEPin,HIGH);else digitalWrite(VALVEPin,LOW);// end temp enabled
  } 
}



// -----------------------------------------  SEQUENCES -----------------------------------------

void punchSequence() {
  switch (State) {
    case 0:                                     // idle
      countdownsecs = 0;
      Outputs(ALL_OFF);             
      if (SP[5][3]) State = 1;                  // autopunch or manual punch start
      break;
    case 1:
      psix10_SP = SP[1][1];                     // assign pressure control setpoint
      countdownsecs=SP[1][0]*secspermin;        // inflate timer        
      elapsedsecs=0;                            // just to prevent immediate cancel
      if (simulate) latchStart = true; else latchStart = false; // force the latchstart
      if (psix10_SP > 1) Outputs(INFLATE);      // inflate if there is a valid pressure SP
      State=2; 
      add2Log(UTC.now(),1);                    // record start of punch in TZ even though set as UTC !!
      break; 
    case 2:  
      if (!latchStart) {
        if (psix10_PV > psix10_SP) latchStart = true;   // release it to start countdown
     //   Serial.print(elapsedsecs);Serial.print(" ");Serial.println(SP[1][3]);
        if (elapsedsecs > SP[1][3]*60) {        // failed to inflate in time
          ez.screen.clear();                          // clear the display
   //$$       ez.removeEvent(displayLoop);
          displayOff= true;
          add2Log(UTC.now(),4);                       // record punch failed
          countdownsecs = 100;
          State = 11;                                 // failing               
        }
      }                          
      if (countdownsecs < 1) {                       // finished inflation
        latchStart = false;                          // reset the latchstart       
        countdownsecs=SP[1][2]*secspermin;           // deflate timer  
        Outputs(DEFLATE);                
        elapsedsecs=0;                               // just to prevent immediate cancel
        State=5;
      }   
      break;
    case 5:                                         // deflating
      if (!latchStart) {
        if (psix10_PV < 1) latchStart = true;       // release it to start deflation countdown
      }
      if (countdownsecs < 1) {                      //           
        countdownsecs = 5;
        Outputs(ALL_OFF);
        elapsedsecs = 0;
        State=9;                                    // go to complete
      } 
      break; 
    case 9:                                         // COMPLETE
      Outputs(ALL_OFF);
      latchStart = true;
      if (countdownsecs < 1) {                      // allow time 
        SP[5][3] = false;
        add2Log(UTC.now(),2);                       // record completion of punch
        latchStart = false;
        showHeaderez();                             // redraw the clock
        State=0;                                    // back to idle
      }  
      break;
    case 10:                                        // CANCELLING
      Outputs(ALL_OFF);
      latchStart = true;
      if (countdownsecs < 1) {                      // allow time 
        ez.canvas.clear();                          // clear any message
        SP[5][3] = false;
        add2Log(UTC.now(),3);                       // record cancellation of punch
        latchStart = false;
        showHeaderez();                             // redraw the clock
        State=0;                                    // back to idle
      }  
      break;      
    case 11:                                        // FAILING
      Outputs(ALL_OFF);
      latchStart = true;
      String btnpressed = ez.msgBox("FAILED TO INFLATE !","Check tubing connections","OK",false,&FreeSans18pt7b, TFT_RED);        
      if (countdownsecs < 1) {                      // allow time 
        ez.screen.clear();                          // clear any message
        delay(10);
  //$$      ez.addEvent(displayLoop);
        displayOff = false;
        SP[5][3] = false;
        latchStart = false;
        showHeaderez();                             // redraw the clock
        State=0;                                    // back to idle
      }  
      break;
    }      
}


void pressSequence() {
  String btnpressed;
  switch (State) {
  case 0:
    countdownsecs = 0;
    presshold = false;
    if (SP[5][3])  State = 1;                     // manual press start
    break;
  case 1:  
    elapsedsecs=0;                                // just to prevent immediate cancel
    if (!presshold) {
      displayOff = true;
      add2Log(UTC.now(),22);
      ezProgressBar pb ("Ready To Press?","VERIFY|1 relief valve removed? |2 harvest tube connected? |3 lid clamp secured?","YES # CANCEL #");
      int startsecs = 1000;                            // 5 secs startup to hit SETUP
      float percent = 100;
      String btnpressed;
      unsigned long lastTime = millis();
      while (startsecs > 0) {
        if ((millis() - lastTime) >10) {
          lastTime = millis();
          percent = (1000 - startsecs)/10.0;          // pb goes 0 to 100
          startsecs--;
          btnpressed = ez.buttons.poll();
          pb.value(percent);                                                      // show a startup progress bar
          if ((btnpressed == "YES")|(btnpressed == "CANCEL")) {
            ez.canvas.clear(); 
            break;
          }
        }  
      }
      if (startsecs == 0)  {
         ez.canvas.clear();      
         btnpressed = "CANCEL";                                // kill the press   
      }
      presshold = true;
      displayOff = false;
      if (btnpressed =="YES") {
        psix10_SP = SP[2][1];                     // press pressure control setpoint
        Outputs(INFLATE);                         // inflate cuff 
        countdownsecs=SP[2][0]*secspermin;        // press inflate timer 
        if (simulate) latchStart = true; else latchStart = false; 
        add2Log(UTC.now(),6);                     // record start of press in TZ even though set as UTC !!      
        State = 3;      // was 2 !!
      }
      else if (btnpressed == "CANCEL") {
        countdownsecs = 5;
        ez.buttons.show(" # WAIT # ");
        showHeaderez();
        showMainBtn();
        State = 10;                               // goto cancel
      }          
      showHeaderez();                             // update all the dispalys
    }  
    break; 
  case 3:                                         // inflating ...
    if (!latchStart) {
      if (psix10_PV > psix10_SP) latchStart = true;   // release it to start countdown
      if (elapsedsecs > SP[2][3]*60) {                // failed to inflate in time
        ez.screen.clear();                          // clear the display
  //      ez.removeEvent(displayLoop);
        displayOff = true;
        add2Log(UTC.now(),9);                       // record press failed
        countdownsecs = 100;
        State = 11;                                   // cancel the SP[0][3]
      }
    }                                
    if (countdownsecs < 1) {                          // done pressurizing
      latchStart = true;                              // no need to latch
      Outputs(DEFLATE);                               // now deflating ....
      countdownsecs = SP[2][2]*secspermin;            // set the press deflate time             
      elapsedsecs=0;                                  // just to prevent immediate cancel
      State=4;        // was 3
    }   
    break;
  case 4:   
//    if (!latchStart) {
//        if (psix10_PV < 5) latchStart = true;        // release it to start countdown
//    }
    if (countdownsecs < 1)  {
      Outputs(ALL_OFF);
      latchStart = false;                           // reset
      countdownsecs=5;                              // some timer   
      elapsedsecs=0;                                // just to prevent immediate cancel
      State=9;
    }   
    break;
  case 9:                                           // COMPLETE
    Outputs(ALL_OFF);
    latchStart = true;
    if (countdownsecs < 1) {                        
      State=0;    
      showHeaderez();
      SP[5][3] = false;
      add2Log(UTC.now(),7);                       // record press complete                                     
    }  
    break;
  case 10:                                        // CANCELLING
    Outputs(ALL_OFF);
    latchStart = true;
     if (countdownsecs < 1) {                        
      State=0;                                    // back to idle
      showHeaderez();
      SP[5][3] = false;
      add2Log(UTC.now(),8);                       // record press cancelled
    }  
    break;
  case 11:                                        // FAILING
      Outputs(ALL_OFF);
      latchStart = true;
      String btnpressed = ez.msgBox("FAILED TO INFLATE !","Check tubing connections","OK",false,&FreeSans18pt7b, TFT_RED);        
      if (countdownsecs < 1) {                      // allow time 
        ez.screen.clear();                          // clear any message
        delay(10);
  //      ez.addEvent(displayLoop);
        displayOff = false;
        SP[5][3] = false;
        latchStart = false;
        showHeaderez();                             // redraw the clock
        State=0;                                    // back to idle
      }  
      break;
  }                                
}


void deflateSequence() { 
  switch (State) {
  case 0:
    countdownsecs = 0;
    Outputs(ALL_OFF);
    if (SP[5][3]) State = 1;                               // manual deflate start
    break;
  case 1:
    Outputs(DEFLATE);
//    showOperationBtnez();
    countdownsecs=SP[2][2]*secspermin;        //  press deflate timer  
    latchStart = false; 
    elapsedsecs=0;                            // just to prevent immediate cancel
    State=5; 
    break; 
  case 5:   
    if (!latchStart) {
        if (psix10_PV < 1) latchStart = true;       // release it to start countdown
    }
    if (countdownsecs < 1)  {
      Outputs(ALL_OFF);
      latchStart = false;                // reset
      countdownsecs=5;                   // just to pad   
      elapsedsecs=0;                     // just to prevent immediate cancel
      State=9;
    }   
    break;
  case 9:                               // COMPLETE
    Outputs(ALL_OFF);
    latchStart = true;
    if (countdownsecs < 1) {                       
      State=0;     
      SP[5][3] = false;                 // back to idle
    }  
    break;
  case 10:                               // CANCELLING
    Outputs(ALL_OFF);
    latchStart = true;
    if (countdownsecs < 1) {             // allow time for vent valve to close
      State=0; 
      SP[5][3] = false;                  // back to idle
    }  
    break;  
  }                               
}


//====================================   DISPLAYS =================================
void showSetupez() {
  ezMenu setupmenu("GOfermentor JR SETUP");
  setupmenu.addItem("Brightness", ez.backlight.menu);
  setupmenu.addItem("Timezone and DST", timemenu);
  setupmenu.addItem("Set Process parameters", parametermenu);
  setupmenu.addItem("Reset WiFi", resetmenu);           // force reprovisioning
  setupmenu.addItem("Firmware Update",request_ota);
  setupmenu.upOnFirst("last|up");
  setupmenu.downOnLast("first|down");
  setupmenu.addItem("Exit | Go back to main menu");
  setupmenu.buttons(" up # Done # select # # down #");
  setupmenu.run();
}


void showSplashScreen() {
 String cr = (String)char(13);
  char headerStr[80];
  char SSIDStr[255];
  char buffer[40];
  unsigned long lastTime = millis();
  strcpy(headerStr,"JR V ");sprintf(buffer,"%0d",ver);strcat(headerStr,buffer);
  if (savedSSIDStr.length()>0) {
    strcpy(SSIDStr,"... ");
    savedSSIDStr.toCharArray(buffer,32);
    strcat(SSIDStr,buffer);   
  }
  else {
    strcpy(SSIDStr,"NO WiFi");  
  }
  strcat(headerStr,"   ");
  strcat(headerStr,SSIDStr);  
  ezProgressBar pb (headerStr,"www.GOfermentor.com|Press SETUP to change settings","SETUP");
  int startsecs = 500;                            // 5 secs startup to hit SETUP
  float percent = 100;
  while (startsecs > 0) {
    if ((millis() - lastTime) >10) {
      lastTime = millis();
      percent = (500 - startsecs)/5.0;          // pb goes 0 to 100
      startsecs--;
      String btnpressed = ez.buttons.poll();
      pb.value(percent);                                                      // show a startup progress bar
      if (btnpressed == "SETUP") {
        showSetupez();                               // show the setup menu
        break;
      }
    }  
  }
}


void showResetScreen() {
  String btnpressed = ez.msgBox("ADD DEVICE", "SETUP NEW DEVICE", "",false);
}

void showSettings() {                                 // shows current press/punch parameters
  char outStr[80];
  char buffer[80];
  ezMenu settingmenu("View Settings");
  settingmenu.txtSmall();
  strcpy(outStr,"UNIT ID ");
  sprintf(buffer,"%0d",SP[0][0]);
  strcat(outStr,buffer);
  strcat(outStr,"\t");
  strcat(outStr,"version: ");
  sprintf(buffer,"%0d",ver);
  strcat(outStr,buffer);
  settingmenu.addItem(outStr);
  strcpy(outStr,"MAC ");
  strcat(outStr,"\t");
  WiFi.macAddress(mac);
  sprintf(buffer, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  strcat(outStr,buffer);
  settingmenu.addItem(outStr);
  strcpy(outStr,"RTC");strcat(outStr,"\t");  
  if (RTCFound) strcat(outStr,"YES");else  strcat(outStr," NO");
  settingmenu.addItem(outStr);
  strcpy(outStr,"SDCARD");strcat(outStr,"\t");  
  if (SDFound) strcat(outStr,"YES");else  strcat(outStr," NO");
  settingmenu.addItem(outStr);
  strcpy(outStr,"TZone");strcat(outStr,"\t"); 
  sprintf(buffer,"%0d",SP[3][0]-128);
  strcat(outStr,"UTC-");
  strcat(outStr,buffer); 
  settingmenu.addItem(outStr);
  strcpy(outStr,"DST");strcat(outStr,"\t");  
  if (SP[3][2]) strcat(outStr,"YES");else  strcat(outStr," NO");
  settingmenu.addItem(outStr);
  if (WiFiEnabled) {  
    if (WiFiConnected) {
      strcpy(outStr,"SSID ");strcat(outStr,"\t");  
      WiFi.SSID().toCharArray(buffer, WiFi.SSID().length());
      strcat(outStr,buffer);
      settingmenu.addItem(outStr);
      strcpy(outStr,"IP ");strcat(outStr,"\t");  
      sprintf (buffer,WiFi.localIP().toString().c_str());
      strcat(outStr,buffer);
      settingmenu.addItem(outStr);
      strcpy(outStr,"RSSI ");strcat(outStr,"\t");  
      sprintf(buffer,"%0d",WiFi.RSSI());
      strcat(outStr,buffer);strcat(outStr," dBm"); 
      settingmenu.addItem(outStr);
      strcpy(outStr,"TIME sync'd");strcat(outStr,"\t");  
      if (gotTimeSync) strcat(outStr,"YES");else  strcat(outStr," NO");
      settingmenu.addItem(outStr);
      strcpy(outStr,"INTERNET");strcat(outStr,"\t");  
      if (internetAvailable) strcat(outStr,"YES");else  strcat(outStr," NO");
      settingmenu.addItem(outStr);
    }
    else {
      strcpy(outStr,"WiFI NO CONNECTION");
      settingmenu.addItem(outStr);
    }     
  }
  else {
    strcpy(outStr,"WiFI");strcat(outStr,"\t"); strcat(outStr,"DISABLED");
    settingmenu.addItem(outStr);
  }  
  strcpy(outStr,"");
  // now print parameter settings
  for (int i=1;i<5;i++) {         // 5 rows
    if ((i <3) | (i == 4  & SP[4][3])) {                   // do not show the time row or temp row if enabled
      for (int j=0;j<8;j++) {      
        if (strlen(SPName[i][j]) > 0) {           // only show non-blank parameters  
          strcpy(outStr,"");
          strcat(outStr,SPName[i][j]);
          strcat(outStr,"\t");
          if ((SPmax[i][j]-SPmin[i][j])==1) {     // use yes no prompt
            if (SP[i][j]) strcpy(buffer,"YES");else strcpy(buffer," NO");
          }
          else sprintf(buffer,"%0d",SP[i][j]);
          strcat(outStr,buffer);
          settingmenu.addItem(outStr); 
        }
      } 
    }  
  }     
  settingmenu.buttons("up # exit # down ");
  settingmenu.run();
}  


void showMainBtn() {
  if (State ==0 ) {
    switch(SP[0][3]) {
      case 0:
        if (SP[4][4]) {         // temperature control enabled
          if (SP[0][1]) ez.buttons.show("MODE # LOG # CHANGE # # OPER # TC ");
          else ez.buttons.show("MODE # LOG # START # # OPER # TC "); 
        }
        else {
          if (SP[0][1]) ez.buttons.show("MODE # LOG # CHANGE # # OPER # ");
          else ez.buttons.show("MODE # LOG # START # # OPER # "); 
        }  
        break;
      case 1:
        ez.buttons.show("# START # OPER");
        break;
      case 2:
        ez.buttons.show("# START # OPER");
        break;
    }
  } 
  else if (State < 6) {
    ez.buttons.show("# CANCEL # ");    
  }
}


void showIndicators() {
  int botEdge = 25;
  m5.lcd.fillRect (215, ez.canvas.bottom() -botEdge-8, 100, 30, TFT_YELLOW);
  m5.Lcd.setFont(&FreeSansBold9pt7b);
  m5.Lcd.setCursor(225,ez.canvas.bottom()-botEdge+12);
  m5.Lcd.setTextColor(TFT_RED,TFT_BLACK);
  switch(SP[0][3]) {
    case 0: m5.Lcd.print(" PUNCH ");
            if (SP[0][1]) {
               m5.lcd.fillRect (110, ez.canvas.bottom() -botEdge-8, 95, 30, TFT_YELLOW);
               m5.lcd.fillRect (5, ez.canvas.bottom() -botEdge-8, 95, 30, TFT_YELLOW);
               m5.Lcd.setCursor(21,ez.canvas.bottom()-botEdge+12);
               m5.Lcd.print(" AUTO ");         // autopunch settings                 
               m5.Lcd.setCursor(130,ez.canvas.bottom()-botEdge+12);
               ez.canvas.color(TFT_RED);
               m5.Lcd.print(SP[0][2]);         // autopunch settings
               m5.Lcd.print(" /24hr");         // autopunch settings
            }
            else {
               m5.lcd.fillRect (5, ez.canvas.bottom() -botEdge-8,95, 30, TFT_YELLOW);
               m5.Lcd.setCursor(12,ez.canvas.bottom()-botEdge+12);
               m5.Lcd.setTextColor(TFT_RED,TFT_BLACK);
               m5.Lcd.print("MANUAL");         // autopunch settings  
               m5.lcd.fillRect (110, ez.canvas.bottom() -botEdge-8, 95, 30,ez.theme->background);  
            }
            break;
    case 1: m5.lcd.fillRect (5, ez.canvas.bottom() -botEdge-8,95, 30, ez.theme->background);
            m5.lcd.fillRect (215, ez.canvas.bottom() -botEdge-8, 100, 30, TFT_RED);
            m5.Lcd.setTextColor(TFT_YELLOW,TFT_RED);
            m5.Lcd.print(" PRESS ");
            m5.lcd.fillRect (110, ez.canvas.bottom() -botEdge-8, 95, 30,ez.theme->background); 
            break;
    case 2: m5.lcd.fillRect (5, ez.canvas.bottom() -botEdge-8,95, 30, ez.theme->background);
            m5.Lcd.print("DEFLATE");
            m5.lcd.fillRect (110, ez.canvas.bottom() -botEdge-8, 95, 30,ez.theme->background); 
            break;  
  }
}



/*

void showIndicators() {
  int botEdge = 25;
  M5.lcd.fillRect (110, ez.canvas.bottom() -botEdge-8, 95, 30,ez.theme->background);
  M5.lcd.fillRect (5, ez.canvas.bottom() -botEdge-8, 95, 30, ez.theme->background);
  if (SP[0][3] == 0) {                                                   // punch only
    if (SP[0][1] & (SP[0][3] ==0)) {
       M5.lcd.fillRect (5, ez.canvas.bottom() -botEdge-8, 95, 30, TFT_YELLOW);
       M5.Lcd.setCursor(21,ez.canvas.bottom()-botEdge+12);
       M5.Lcd.setFont(&FreeSansBold9pt7b);
       M5.Lcd.setTextColor(TFT_RED,TFT_YELLOW);
       M5.Lcd.print(" AUTO ");         // autopunch settings                 
       M5.Lcd.setCursor(130,ez.canvas.bottom()-botEdge+12);
       m5.lcd.fillRect (110, ez.canvas.bottom() -botEdge-8,95, 30, TFT_YELLOW);
       M5.Lcd.setTextColor(TFT_RED,TFT_YELLOW);
       M5.Lcd.print(SP[0][2]);         // autopunch settings
       M5.Lcd.print(" /24hr");         // autopunch settings
    }
    if (!SP[0][1]) {
       m5.lcd.fillRect (5, ez.canvas.bottom() -botEdge-8,95, 30, TFT_YELLOW);
       M5.Lcd.setCursor(12,ez.canvas.bottom()-botEdge+12);
       M5.Lcd.setTextColor(TFT_RED,TFT_YELLOW);
       M5.Lcd.setFont(&FreeSansBold9pt7b);
       M5.Lcd.print("MANUAL");         // autopunch settings    
  //     M5.lcd.fillRect (110, ez.canvas.bottom() -botEdge-8, 95, 30,ez.theme->background);
    } 
  }  
  if ((SP[0][3] == 0) & !SP[0][1]) {
    m5.lcd.fillRect (215, ez.canvas.bottom() -botEdge-8,100, 30, TFT_YELLOW);
    M5.Lcd.setCursor(240,ez.canvas.bottom()-botEdge+12);
    M5.Lcd.setTextColor(TFT_RED,TFT_YELLOW);
    M5.Lcd.setFont(&FreeSansBold9pt7b); 
    M5.Lcd.print("PUNCH");         // autopunch settings    
//     M5.lcd.fillRect (110, ez.canvas.bottom() -botEdge-8, 95, 30,ez.theme->background);  
  }
  else if (SP[0][3] == 1) {
    m5.lcd.fillRect (215, ez.canvas.bottom() -botEdge-8,100, 30, TFT_RED);
    M5.Lcd.setCursor(240,ez.canvas.bottom()-botEdge+12);
    M5.Lcd.setTextColor(TFT_YELLOW,TFT_RED);
    M5.Lcd.setFont(&FreeSansBold9pt7b); 
    M5.Lcd.print("PRESS");         // autopunch settings    
//     M5.lcd.fillRect (110, ez.canvas.bottom() -botEdge-8, 95, 30,ez.theme->background);      
  }  
}
*/
void drawBag() {
  M5.Lcd.drawRoundRect(x_datum,y_datum,80,80,20,TFT_BLACK);
}


void showNextPunch() {                          // in auto show next punch timing
  int hh;
  int mm;
  int ss;
  long punchSecs = nextPunch_mins * 60;  
  char outStr[40];
  M5.lcd.fillRect (218, 160, 100, 20,ez.theme->background);                   // blanking rectangle
  ez.setFont(&FreeSansBold12pt7b);
  M5.Lcd.setCursor(10,175);M5.Lcd.setTextColor(TFT_BLACK,TFT_WHITE);
  M5.Lcd.print("Next punch in ...");  
  secondsToHMS(punchSecs,hh,mm,ss);               // convert to hh mm ss
  sprintf(buffer,"%02d:%02d",hh,mm);
  M5.Lcd.setCursor(245,156);
  ez.setFont(&FreeMono9pt7b);
  M5.Lcd.print(" hh:mm");
  M5.Lcd.setCursor(250,178);
  ez.setFont(&FreeSansBold12pt7b);
  if (punchSecs > 0) M5.Lcd.print(buffer);else M5.Lcd.print("   ");  
}


void setTempC() {
   int currentValue;
  int k;
//$$  ez.removeEvent(displayLoop);
  displayOff = true;                     // setting true disables the displayloop
  String btnpressed = ez.msgBox("Temperature Control", "Set Temp Control?", "Cancel#OK# ",true);
  if (btnpressed=="OK") {
    ez.header.show("Temperature Controller Settings..");
    m5.Lcd.setFont(&FreeSans12pt7b);
    m5.Lcd.setTextColor(TFT_BLACK,TFT_WHITE); 
    ez.buttons.show("no # Done  # yes");
    currentValue = showYesNoButton(SP[4][0],SPName[4][0]);
    if (currentValue!= SP[4][0]) SP[4][0] = currentValue;                       // set the new value
    if (SP[4][0]) {                                                             // only ask if control is ON
      M5.Lcd.fillRect(10,25,250,45,ez.theme->background); 
      ez.buttons.show("up # Done  # down");
      // -- now for SP 
      if (SP[4][2]) {                         // need to convert to F
        float SPdegF = CToF(SP[4][1]/5.0);
        int TempTSP = round(SPdegF);
        int TempSPmin = CToF(SPmin[4][1]/5.0);
        int TempSPmax = CToF(SPmax[4][1]/5.0); 
        currentValue = showAnalogButton(TempTSP,TempSPmin,TempSPmax,"SETPOINT degF");
        if (currentValue!= TempTSP) {
          float SPdegC = FToC(currentValue);
          SP[4][1] = round(SPdegC * 5.0);    // convert back to degCx5 for storage
        }
      }
      else {                                   // was in C
        int temp = SP[4][1]/5;
        int tempmin = SPmin[4][1]/5;
        int tempmax = SPmax[4][1]/5;
        currentValue = showAnalogButton(temp,tempmin,tempmax,"SETPOINT deg C");
        currentValue = currentValue*5;
        if (currentValue!= SP[4][1]) SP[4][1] = currentValue;                       // set the new value
      }   
    }  
  } 
  if (btnpressed=="Cancel") {
    
  }
  ez.screen.clear();
  displayOff = false;                     // setting true disables the displayloop
//$$ ez.addEvent(displayLoop);
  showBag();
}

void showBag() {
  M5.Lcd.drawRoundRect(x_datum,y_datum,80,110,10,TFT_BLACK);        
  showCuffpressure();                   // show cuff pressure only if debug
  if (AIR_ON) {
    if (VAC_ON) showDeflating();
    else showInflating();
  }
  else  M5.Lcd.fillRect(x_datum-90,y_datum+20,80,40,ez.theme->background);            // erase
}

void showHX() {
  int color = TFT_ORANGE;
  int x_off= 220;
  int y_off= 115;
//  M5.Lcd.fillRect(x_off-10,y_off-12,90,62,ez.theme->background);            // blanking rectangle
  M5.Lcd.fillRect(x_off-10,y_off-12,105,42,ez.theme->background);            // blanking rectangle
  if (SP[4][3] & (SP[4][4]>0) & !tempSensorFailed) {                               // temp enabled tctrlmode enabled and temp reading valid
    M5.Lcd.fillRect(x_off+60,y_off-10,40,38,TFT_YELLOW); 
    M5.Lcd.setFont(&FreeSansBold9pt7b);
    M5.Lcd.setCursor(x_off+62,y_off+12);
    M5.Lcd.setTextColor(TFT_RED,TFT_YELLOW);
    if (SP[4][0]) M5.Lcd.print(" ON");else M5.Lcd.print("OFF");   // control status
  }
  else {
    M5.Lcd.fillRect(x_off+60,y_off-10,40,38,ez.theme->background);
  }
  boolean ON = digitalRead(VALVEPin);
  if (ON) {
    color = TFT_GREEN; 
    M5.Lcd.fillTriangle(x_off,y_off,x_off-10,y_off+10,x_off,y_off+20,color);// coolant in arrow
    M5.Lcd.fillRect(x_off,y_off+5,20,10,color);
    M5.Lcd.fillRect(152,50,15,85,TFT_GREEN);                              // show cooling tube
  }
  else  M5.Lcd.fillRect(152,50,15,85,TFT_DARKGREY);                              // show cooling tube
  M5.Lcd.fillRect(x_off+9,y_off-10,50,38,color); 
  ez.setFont(&FreeSansBold9pt7b);
  M5.Lcd.setTextColor(TFT_BLACK,color);
  M5.Lcd.setCursor(x_off+13,y_off+5);
  M5.Lcd.print("VLV"); // legend
  M5.Lcd.setCursor(x_off+11,y_off+20);
  if (ON) M5.Lcd.print("open"); else M5.Lcd.print("clsd"); 
}

void showInflating() { 
  M5.Lcd.fillRect(x_datum-90,y_datum+20,80,40,ez.theme->background);            // erase
  M5.Lcd.fillTriangle(x_datum-20,y_datum+30,x_datum-10,y_datum+40,x_datum-20,y_datum+50,TFT_RED);// air in arrow
  M5.Lcd.fillRect(x_datum-40,y_datum+35,20,10,TFT_RED);            // air in rectangle
  ez.setFont(&FreeSansBold12pt7b);
  M5.Lcd.setTextColor(TFT_RED,TFT_WHITE);
  M5.Lcd.setCursor(x_datum-85,y_datum+45);M5.Lcd.print("AIR"); // legend
  M5.lcd.fillRect (0, 145, TFT_W-75, 35, ez.theme->background);  // blanking rectangle
  ez.setFont(&FreeSansBold12pt7b);
  M5.Lcd.setCursor(10,178);M5.Lcd.setTextColor(TFT_BLACK,TFT_WHITE);
  M5.Lcd.print("INFLATING ..");
}

void showDeflating() {
  M5.Lcd.fillRect(x_datum-90,y_datum+25,90,40,ez.theme->background);            // erase
//  M5.Lcd.fillTriangle(x_datum-30,y_datum+30,x_datum-40,y_datum+40,x_datum-30,y_datum+50,TFT_BLACK);// air out arrow
  M5.Lcd.fillTriangle(x_datum-22,y_datum+30,x_datum-32,y_datum+40,x_datum-22,y_datum+50,TFT_BLACK);// air out arrow
  M5.Lcd.fillRect(x_datum-25,y_datum+35,15,10,TFT_BLACK);           
  ez.setFont(&FreeSansBold12pt7b);
  M5.Lcd.setTextColor(TFT_BLACK,TFT_WHITE);
  M5.Lcd.setCursor(x_datum-85,y_datum+45);M5.Lcd.print("VAC"); // legend
  M5.lcd.fillRect (0, 145, TFT_W-75, 35, ez.theme->background);  // blanking rectangle for legend
  ez.setFont(&FreeSansBold12pt7b);
  M5.Lcd.setCursor(10,178);M5.Lcd.setTextColor(TFT_BLACK,TFT_WHITE);
  M5.Lcd.print("DEFLATING ..");
}

void showCuffpressure(){
  int color = TFT_BLACK;
  M5.Lcd.fillRect(x_datum+5,y_datum+35,25,60,ez.theme->background);       // erase old cuffs
  M5.Lcd.fillRect(x_datum+50,y_datum+35,25,60,ez.theme->background);       // erase old cuffs
  if (psix10_PV < 1) color= TFT_BLACK;
  if (psix10_PV > 1) {
    if (psix10_PV <= 10) color= TFT_GREEN;
    if (psix10_PV > 10) color= TFT_YELLOW;
    if (psix10_PV > 15) color= TFT_RED;
    int press_inpixel = psix10_PV *15/20;                                      // show cuff pressure
    M5.Lcd.fillRect(x_datum+5,y_datum+35,press_inpixel,60,color);           // top left
    M5.Lcd.fillRect(x_datum+75-press_inpixel,y_datum+35,press_inpixel,60,color);            // top right 
  }
}


void showPsi() {
  char outStr[32];
  char buffer[32];
  float psig = psix10_PV/10.0;
  strcpy(outStr,"");strcpy(buffer,"");   
  M5.Lcd.setTextColor(TFT_BLACK,TFT_YELLOW);
  ez.setFont(&FreeSansBold12pt7b);
  M5.Lcd.setCursor(20,120);
  if (psig < 0) psig = 0.0;
  dtostrf(psig,2,1,buffer);
  strcat(outStr,buffer);
  strcat(outStr," psi"); 
  M5.Lcd.fillRect(15,100,100,30,ez.theme->background);       // erase old PV
  M5.Lcd.print(outStr);
}


void showPressTube() {
  M5.Lcd.drawRect(240,35,20,123,TFT_WHITE);    // center tube
}


void erasePressTube() {
  M5.Lcd.fillRect(240,35,20,123,TFT_BLACK);    // center tube
}



/*
void showCycleNum(boolean show) {
  // show the cycle number if true
  char buffer[40];
  char outStr[40];
  M5.lcd.fillRect (5, 128, 150, 32,ez.theme->background);  // blanking rectangle 
//  M5.lcd.fillRect (5, 128, 150, 32,TFT_RED);  // blanking rectangle 
  if (show) {
    strcpy(outStr,"CYCLE ");
    M5.Lcd.setCursor(10,150);M5.Lcd.setTextColor(TFT_BLACK,TFT_WHITE);
    sprintf(buffer,"%d",pressCycle);
    strcat(outStr,buffer);
    strcat(outStr," of ");
    sprintf(buffer,"%d",SP[2][0]);
    strcat(outStr,buffer);
    M5.Lcd.print(outStr);
  }
}

void CycleNum() {
  char buffer[40];
  char outStr[40];
  strcpy(outStr,"CYCLE ");
  M5.lcd.fillRect (5, 120, 160, 32,TFT_RED);  // blanking rectangle
  M5.Lcd.setCursor(10,148);M5.Lcd.setTextColor(TFT_BLACK,TFT_WHITE);
  sprintf(buffer,"%d",pressCycle);
  strcat(outStr,buffer);
  strcat(outStr," of ");
  sprintf(buffer,"%d",SP[2][0]);
  strcat(outStr,buffer);
  M5.Lcd.print(outStr);
}

*/
void showHeaderez() {
  char outStr[60];
  char buffer[40];
  ez.setFont(&FreeMono9pt7b);
  strcpy(outStr,"TAG ");
  sprintf(buffer,"% 12s",tagName);
  strcat(outStr,buffer);
  myTZ.dateTime("     H:i").toCharArray(buffer,30);
  strcat(outStr,buffer);
  if(WiFiConnected) {               
    if (internetAvailable & WiFiConnected) strcat(outStr,"  i"); else strcat(outStr,"   ");
    if (BlynkOffCount < 10) strcat(outStr," C"); else strcat(outStr,"  ");
    if (BlynkAppConnected & Blynk.connected()) strcat(outStr,"A"); else strcat(outStr," ");   // show app connected
  }
  else strcat(outStr," No WiFi     ");                 // no WiFi
  ez.header.show(outStr);
}


void showStatusez() { 
  int hh;
  int mm;
  int ss;
  char outStr[10];
  char buffer[10];
  int tempint;
  if (SP[4][3]) {                                   // temperature display
    ez.setFont(&FreeSansBold12pt7b);
    M5.lcd.fillRect (200, 25, 110, 31,ez.theme->background);  // blanking rectangle
    if (!tempSensorFailed) {    
      M5.lcd.fillRect (220, 28, 90, 30, TFT_YELLOW);  // PV rectangle
      if (SP[4][0]) {                                 // controller ON so show if alarms
        if (tempHigh) {                              // HIGH alarm
          M5.lcd.fillRect (220, 28, 90, 30, TFT_RED);  // PV rectangle 
          M5.Lcd.setCursor(202,47);M5.Lcd.setTextColor(TFT_RED,TFT_YELLOW);
          M5.Lcd.print("H");
        }
        if (tempLow) {                               // LOW alarm
          M5.lcd.fillRect (220, 28, 90, 30, TFT_CYAN);  // PV rectangle
          M5.Lcd.setCursor(202,47);M5.Lcd.setTextColor(TFT_CYAN,TFT_YELLOW);
          M5.Lcd.print("L");  
        }     
      } 
      M5.Lcd.setCursor(232,47);M5.Lcd.setTextColor(TFT_BLACK,TFT_YELLOW);
      float tempdisp = tempC;
   //   Serial.print("tempC"); Serial.println(tempC);
      if (SP[4][2]) tempdisp = CToF(tempC);
   //   Serial.print("tempF"); Serial.println(tempdisp);
      dtostrf(tempdisp,3,1,buffer);
      strcpy(outStr,buffer);
      M5.Lcd.print(outStr);
      M5.Lcd.setCursor(280,47);
      if (SP[4][2]) M5.Lcd.print(" F"); else M5.Lcd.print(" C");  // show temp units     
      if (SP[4][4]>0) {                                // HX enabled
        showHX();                                // show HX and valve status
        M5.lcd.fillRect (210, 62, 110, 25,ez.theme->background);  // blank out old SP value
        M5.Lcd.setCursor(232,77);M5.Lcd.setTextColor(TFT_BLACK,TFT_YELLOW);
        ez.setFont(&FreeSansBold12pt7b);
        float tempSP = SP[4][1]/5.0;              // remember TSP in degCx5
        if (SP[4][2]) tempSP = CToF(tempSP);
        dtostrf(tempSP,3,0,buffer);
        strcpy(outStr,buffer);
        M5.Lcd.print(outStr);
        M5.Lcd.setCursor(280,77);
        M5.Lcd.print ("SP");
       // now show the heating/cooling mode
        ez.setFont(&FreeSansBold9pt7b);
        M5.Lcd.setCursor(240,95);
        if (SP[4][4] == 1) {
          M5.lcd.fillRect (220, 80, 90, 20,TFT_CYAN);  // blank out old TCTRLMode value
          M5.Lcd.print("COOL");        
        }
        if (SP[4][4] == 2) {
          M5.lcd.fillRect (220, 80, 90, 20,TFT_RED);  // blank out old TCTRLMode value
          M5.Lcd.print("HEAT");       
        }
      }
      else {                                          // do not show HX is disabled
        M5.lcd.fillRect (210, 58, 110, 88,ez.theme->background);  
        M5.Lcd.fillRect(152,50,15,85,ez.theme->background);      // hide cooling tube
      }
    }     
    else {
      M5.lcd.fillRect (220, 28, 90, 30, TFT_MAGENTA);  // PV rectangle  
      M5.Lcd.setCursor(232,47);M5.Lcd.setTextColor(TFT_BLACK,TFT_YELLOW);
      M5.Lcd.print("ERR!");     
    }
  }
  ez.setFont(&FreeSansBold12pt7b);
  M5.Lcd.setCursor(10,178);M5.Lcd.setTextColor(TFT_BLACK,TFT_WHITE);  
  M5.lcd.fillRect (0, 155, 200, 26,ez.theme->background);                   // blanking rectangle
  if (State==0) {
    if (!SP[0][1]) { 
      M5.Lcd.print("READY");
      M5.lcd.fillRect (218, 140, 100, 40,ez.theme->background);                   // blanking rectangle
    }
    else showNextPunch();  // show autopunch if punch selected
  }
  else M5.Lcd.print(Statename[State]);
  
  if (State > 1) {                          // if running - show countdown  
    M5.Lcd.setCursor(250,178);
    secondsToHMS(countdownsecs,hh,mm,ss);
    sprintf(buffer,"%02d:%02d",mm,ss);
    M5.lcd.fillRect (250, 158, 70, 25,ez.theme->background);  // blanking rectangle
    if (countdownsecs > 0) M5.Lcd.print(buffer);else M5.Lcd.print("   ");
    ez.setFont(&FreeSans12pt7b);
  }  
}


// --------------------------------- LOGS ---------------------------

void add2Log(unsigned long timesecs,int code) { 
  time_t lt;
  if (timesecs > 1577836800){                 // must be greater than 2020
    Serial.print("Adding to Log@ ");Serial.print(SP[5][0]);Serial.print(" ");
   // lt = myTZ.tzTime(timesecs,UTC_TIME);
   // lt = timesecs;
    Serial.print(timesecs);Serial.print(" ");Serial.print(myTZ.dateTime(timesecs,"m/d/y H:i"));Serial.print(" ");Serial.println(Codename[code]);
    if (SDFound) {
      add2SDLog(timesecs,code);
    }
    else {
      add2EEPROMLog(timesecs,code);
    }  
    if (Blynk.connected())  {
      newMsg(code); 
    }
//    printOrderedLog();
  }  
}


void add2EEPROMLog(unsigned long timesecs,int code) { 
  // write time and event code to EEPROM
  int addr = SP[5][0]*5 + LOGADDRSTART;         // get the offset addr
//  Serial.print("SP=");Serial.print(SP[5][0]);Serial.print(" ");Serial.print("addr=");Serial.println(addr);
  EEPROMWritelong(addr,timesecs);
  addr = addr + 4 ;                    // code value register
  EEPROM.write(addr,code); 
  EEPROM.commit();
  delay(10);
  if (SP[5][0] < MAXLOG-1) SP[5][0]++; else SP[5][0] = 0;  
}


void readLog() {
  int filelines;
  if (SDFound)  {
    readSDLog();
  }
  else {
    readEEPROMLog();                           // will return in chron reverse chron
  }
}

void readEEPROMLog() {
  int offset = LOGADDRSTART;                   // start of log eeprom
  int address;
//  Serial.println("--- readLog");
  for (int i=0;i<=MAXLOG-1;i++) {  // read up MAXLOG entries   
    address = offset+i*5;
//    Serial.print("read");    Serial.print(address);
    logSecs[i]  = EEPROMReadlong(address) ;
    address = address + 4;
    logCode[i] = EEPROM.read(address);  
//    Serial.print(i);Serial.print(" ");Serial.print(logSecs[i]);
//    Serial.print("=");Serial.println(logCode[i]);
  }  
  orderLog();                               // put the EEPROM data in logOrder array  
}  

void eraseEEPROMLog() {
  int addr;
  for (int i=0;i<=MAXLOG-1;i++) {  // read up MAXLOG entries   
    addr = LOGADDRSTART+i*5;
    EEPROMWritelong(addr,0);
    addr = addr + 4 ;                    // code value register
    EEPROM.write(addr,0);   
  } 
//  SP[5][0]=0;
//  saveEEPROM();
  EEPROM.commit();
  delay(100);   
  Serial.println("LOG erased");
}  

void eraseBlynkTable() {
  Blynk.virtualWrite(V0,"clr");
}

void orderLog() {
  //  int topIndex = 0;
//  Serial.print("SP5,0");Serial.println(SP[5][0]);
  int topIndex = SP[5][0]-1;
  if (topIndex < 0) topIndex = 0;
//  Serial.print("top2=");Serial.println(topIndex);
  int j=-1;
  for (int i=topIndex;i>=0;i--) {
    j++;
    logOrder[j]=i;
  }  
  // now look at wraparound
  for (int i=(MAXLOG-1);i> topIndex;i--) {
    j++;
    logOrder[j]=i;  
  }
//  for (j=0;j<MAXLOG;j++) {
//    Serial.print(j);Serial.print(" ");Serial.println(logOrder[j]);
//  }
}


void add2SDLog(unsigned long timesecs,int codevalue) {      // already chron order
  int hh;
  int mm;
  int ss;
  char buffer[10];       
  char SDbuffer[80];
  char codeBuffer[8];
  sprintf(SDbuffer,"%lu",timesecs);
  strcat(SDbuffer,",LOG,");
  sprintf(codeBuffer,"%02d",codevalue);
  strcat(SDbuffer,codeBuffer);
  strcat(SDbuffer,"\n");
  appendFile(SD, "/eventlog.txt", SDbuffer);        // write to SD card
  if (SP[5][0] < MAXLOG-1) SP[5][0]++; else SP[5][0] = 0;   // move the pointer for table app
}


void readSDLog() {                    // read up to MAXLOG items from end of file into LogMsg string array
  String outStr;
//  time_t lt; 
  unsigned long endLine;
//  readFile(SD, "/eventlog.txt");
//  outStr = getFileLine(SD, "/eventlog.txt",1);
//  Serial.print("outStr=");Serial.println(outStr);
//  outStr = getFileLine(SD, "/eventlog.txt",2);
//  Serial.print("outStr=");Serial.println(outStr);
  for (int i=0;i<MAXLOG;i++) {
     logOrder[i] = i;                 // reverse chron order already  
     outStr = getFileLine(SD, "/eventlog.txt",i+1);
 //    Serial.print(i);Serial.print("-");Serial.print(outStr);Serial.print("|");Serial.println(outStr.length());
     if (outStr.length() == 17) {
       int commaIndex = outStr.indexOf(',');
       int secondCommaIndex = outStr.indexOf(',',commaIndex+1);
       String epochStr = outStr.substring(0,commaIndex);
  //     Serial.print("epoch=");Serial.println(epochStr);
       logSecs[i] = epochStr.toInt();
 //      Serial.println(logSecs[i]);
       String codeStr = outStr.substring(secondCommaIndex+1);
       logCode[i]= codeStr.toInt(); 
    //   lt = logSecs[logOrder[i]];     
    //   outStr = String(i) + myTZ.dateTime(lt,"m/d/y H:i") + " " + Codename[logCode[logOrder[i]]];    
     } 
     else {                 // NULL record
       logSecs[i]=0;
       logCode[i]=0; 
     }
      
  }
}


void showLog() {                         // shows thwe Log file and show connection/parameters submenu
  String outStr;
  time_t lt;
//$$  ez.removeEvent(displayLoop);           // have to stop the process loop
  displayOff = true;
  ezMenu logmenu("View log");
  logmenu.txtSmall();
  for (int j=0;j<MAXLOG;j++) {
    if (logCode[logOrder[j]] > 0 ) {       // valid logitem
      lt = logSecs[logOrder[j]];
      outStr = myTZ.dateTime(lt,"m/d/y H:i") + " " + Codename[logCode[logOrder[j]]];
      logmenu.addItem(outStr);
    }  
  }  
  logmenu.buttons("up #  # exit # settings # down # ");
  while (logmenu.pickButton() != "exit") {
    logmenu.runOnce();
    if (logmenu.pickButton() == "settings") {
      readEEPROM();
      loadEEPROM();
      showSettings();  //$$
    }
  }  
  displayOff = false;
//$$  ez.addEvent(displayLoop);               // restart the scheduler
}


// ----------------------------------  MISC ROUTINES --------------
String IPAddress2String(IPAddress address) {
 return String(address[0]) + "." + 
        String(address[1]) + "." + 
        String(address[2]) + "." + 
        String(address[3]);
}

float CToF(float degC) {
  float degF;
  degF = degC * 9.0/5.0 + 32.0;
  return degF;
}

float FToC(float degF) {
  float degC;
  degC = (degF - 32.0)* 5.0/9.0;
  return degC;
}

// ---- time routines ---
int minutesSinceMidnight() {
  int mins_midnight;
  mins_midnight = myTZ.hour() * 60 + myTZ.minute();                  // gets minutes since midnight local time
  return mins_midnight;
}

void calcPunch() {
// calculate minutes of day for each punch starting 1 minute after midnight  
  int punchinterval_mins = 1440/SP[0][2];
  PunchT[0] = 1;                                // first punch of the day always at 1 minute past midnight
  for (int i=1;i<SPmax[0][2];i++) {             // upto max number of punches
    if (i<SP[0][2]) PunchT[i] = PunchT[i-1] + punchinterval_mins;else PunchT[i]=0;
  }
}

void secondsToHMS(long &seconds,int &h,int &m, int &s) {
  long t=seconds;
  s = t % 60;
  t = (t-s)/60;
  m = t% 60;
  t = (t-m)/60;
  h = t; 
}  

boolean setInternetTime(int timeout) {
  Serial.println("waiting for time");
  gotTimeSync = waitForSync(timeout);               // wait timeout secs to get time from internet timeserver   
  Serial.print("sync = ");Serial.println(gotTimeSync);
  if (gotTimeSync) {
    TZoffset = SP[3][0];                          // get from EEPROM
    String posixStr = "UTC";
    if (TZoffset < 128) {                         // UTC = 0 = 128. 
      TZoffset = 128 - TZoffset-SP[3][2];         // convert to +/- 23 and adjust DST
      posixStr = "UTC+"+String(TZoffset)+":"+String(SP[3][1]);      // syntax is reverse sign !
    }
    else if (TZoffset > 128) {
      TZoffset = TZoffset - 128-SP[3][2];
      posixStr = "UTC-"+String(TZoffset)+":"+String(SP[3][1]); 
    }
    Serial.println(posixStr);
    Serial.println(SP[3][0]);
    myTZ.setPosix(posixStr);
    myTZ.setDefault();
    // next set the RTC just in case
    if (RTCFound) rtc.adjust(DateTime(myTZ.year(), myTZ.month(), myTZ.day(), myTZ.hour(), myTZ.minute(), myTZ.second()));
  }
  else { 
    Serial.println("No Time - using RTC if available");
    if (RTCFound) {                             // must have an RTC to use it
      DateTime RTCnow = rtc.now();                // get time from RTC
      myTZ.setTime(RTCnow.hour(),RTCnow.minute(),RTCnow.second(),RTCnow.day(),RTCnow.month(),RTCnow.year());                // set the ESP clock 
    }        
  }     
  internetAvailable = gotTimeSync;              // if you get time then internet connection ok
}

// -----------------------------  DEBUG ROUTINES -------------------------------
void printState() {
  char outStr[80];
  char buffer[80];
   Serial.println("--------------------------------"); 
   Serial.print(msgnum);Serial.print(" ->"); Serial.print(myTZ.dateTime("H:i "));
//   Serial.print(minutes_today);Serial.println("  minsToday");
//   if (digitalRead(BLWRPin)) Serial.print("BLWR_ ON "); else Serial.print("BLWR_OFF ");
//   if (digitalRead(VENTPin)) Serial.print(" VENT_ ON "); else Serial.print(" VENT_OFF ");
//   if (digitalRead(VALVEPin)) Serial.print(" VALV_ ON"); else Serial.println(" VALV_OFF ");
   msgnum++;
   if (msgnum > 9999) msgnum = 0;
   
   Serial.print("Operation= ");Serial.print(SP[0][3]);Serial.print(" BLYNK= "),Serial.println(Blynk.connected());  
   Serial.print("State=");Serial.print(State);
   Serial.print(" Elap= ");Serial.print(elapsedsecs);Serial.print(" Cdown= ");Serial.print(countdownsecs);
   Serial.print(" L=");Serial.println(latchStart);
   Serial.print("tempC= ");Serial.print(tempC);Serial.print(" Press= ");Serial.println(psix10_PV/10.0);
}


void printOrderedLog() {
  time_t lt;
  String outStr;
  readLog();
  Serial.println("---- LOG -------");
  for (int j=0;j<MAXLOG;j++) {
    if (logCode[logOrder[j]] > 0) {
      lt = logSecs[logOrder[j]];
      outStr = String(j) + " "+ myTZ.dateTime(lt,"m/d/y H:i") + " " + Codename[logCode[logOrder[j]]];
      Serial.println(outStr);
    }    
  }  
}

   
void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}



// ---------------------------------- MENUS -------------------------

void mainmenu_ota() {
  // if (ez.msgBox("Get Update via OTA", "This will update the firmware with the latest version. Internet connection required.", "Cancel#OK#") == "OK",true) {
    SP[5][2]=false;
    saveEEPROM();                                 // reset so does not happen on next restart
    Serial.println("update starting");
    ezProgressBar progress_bar("OTA update in progress", "Downloading ...", "Abort");
    String url = "https://raw.githubusercontent.com/skysingh/GOfermJR/master/compiled_binaries/GOJunior_M5.bin";
  //  String url = "https://raw.githubusercontent.com/skysingh/GOferm_NET/master/compiled_binaries/GONet_M5.bin";
    #include "raw_githubusercontent_com.h" // the root certificate is now in const char * root_cert
    if (ez.wifi.update(url, root_cert, &progress_bar)) {
      ez.msgBox("Over The Air updater", "OTA download successful. Reboot to new firmware", "Reboot");
      ESP.restart();
    } else {
      ez.msgBox("OTA error", ez.wifi.updateError(), "OK");
    }
 // }
}

void request_ota() {
  SP[5][2] = false;
  if (ez.msgBox("Get Update via OTA", "This will update the firmware with the latest version. Internet connection required.", "Cancel#OK#") == "OK",true) {
    SP[5][2] = true;
    saveEEPROM();                       // save for next restart
    ESP.restart();
  }  
}


void resetmenu() {
  // reset to new blynk device AP provisioning
  String btnpressed = ez.msgBox("Parameters", "Reset WiFi?", "Cancel#OK#",true);
  if (btnpressed=="OK") {
     Serial.println("RESET");
     SP[5][1]=1;                          // arm for reset
     saveEEPROM();                        // save for restart
     delay(100);
     ESP.restart();                       // need to restart to get new AP 
  } 
}  

void timemenu() {
  if (ez.msgBox("Set Timezone", "Set local timezone", "Cancel#OK#") == "OK") {
     m5.Lcd.setFont(&FreeSans12pt7b);
     ez.header.show("Set TZ/DST from UTC");
     ez.buttons.show("up # Done  # down");
     int currentTZ = SP[3][0] - 128;                                               // load the current timezone
     int TZ = currentTZ;
     currentTZ = showAnalogButton(currentTZ,-23,23,"Enter TZ hours from to UTC.     Example EST is -5");
     if (currentTZ != TZ) {
        TZ = currentTZ;                       // set the new timezone
        SP[3][0] = TZ + 128;                  // unsigned for storage
     }
  }
  if (ez.msgBox("Set DST", "Set daylight saving time now ?", "YES#NO#") == "YES") SP[3][2] = 1; else SP[3][2] = 0;  
}

int showAnalogButton(int parameter,int minval, int maxval,String prompt) {
  char outStr[40];
  char buffer[40];
  m5.Lcd.setFont(&FreeSans12pt7b);
  m5.Lcd.setTextColor(TFT_BLACK,TFT_BLACK); 
  m5.Lcd.setCursor(20, ez.canvas.bottom() - 170); M5.Lcd.println(prompt);
  m5.Lcd.setCursor(20, ez.canvas.bottom() - 90);  M5.Lcd.print("min: "); M5.Lcd.println(minval);   
  m5.Lcd.setCursor(220, ez.canvas.bottom() - 90); M5.Lcd.print("max: ");   M5.Lcd.println(maxval);  
  m5.lcd.fillRect (130, ez.canvas.bottom() - 115, 80, 40, TFT_YELLOW); 
  m5.Lcd.setCursor(160, ez.canvas.bottom() - 90);  M5.Lcd.println(parameter); 
  while (true) {
    String btnpressed = ez.buttons.poll();
    if (btnpressed == "Done") break;
    if (btnpressed == "up") {
      if (parameter < maxval) parameter++;  
      m5.lcd.fillRect (130, ez.canvas.bottom() - 115, 80, 40, TFT_YELLOW);   
      m5.Lcd.setCursor(160, ez.canvas.bottom() - 90);  M5.Lcd.println(parameter); 
    }
    if (btnpressed == "down") {
      if (parameter > minval) parameter--; 
      m5.lcd.fillRect (130, ez.canvas.bottom() - 115, 80, 40, TFT_YELLOW); 
      m5.Lcd.setCursor(160, ez.canvas.bottom() - 90);  M5.Lcd.println(parameter); 
    }  
  }  
  m5.Lcd.setFont(&FreeSans12pt7b);
  m5.Lcd.setTextColor(TFT_BLACK,TFT_BLACK); 
  return parameter;
}

int showYesNoButton(int parameter,String prompt) {
  m5.Lcd.setFont(&FreeSans12pt7b);
  m5.Lcd.setTextColor(TFT_BLACK,TFT_BLACK); 
  m5.Lcd.setCursor(20, ez.canvas.bottom() - 170); M5.Lcd.println(prompt);
  m5.lcd.fillRect (130, ez.canvas.bottom() - 115, 80, 40, TFT_YELLOW); 
  m5.Lcd.setCursor(160, ez.canvas.bottom() - 90);  
  if (parameter) M5.Lcd.println("YES"); else M5.Lcd.println("NO"); 
  while (true) {
    String btnpressed = ez.buttons.poll();
    if (btnpressed == "Done") break;
    if (btnpressed == "yes") {
      parameter=1;  
      m5.lcd.fillRect (130, ez.canvas.bottom() - 115, 80, 40, TFT_YELLOW);   
      m5.Lcd.setCursor(160, ez.canvas.bottom() - 90); 
      M5.Lcd.println("YES"); 
    }
    if (btnpressed == "no") {
      parameter = 0; 
      m5.lcd.fillRect (130, ez.canvas.bottom() - 115, 80, 40, TFT_YELLOW); 
      m5.Lcd.setCursor(160, ez.canvas.bottom() - 90); 
      M5.Lcd.println("NO");  
    }  
  }     
  m5.Lcd.setFont(&FreeSans12pt7b);
  m5.Lcd.setTextColor(TFT_BLACK,TFT_BLACK); 
  return parameter;
}

void parametermenu() {
  int currentValue;
  int k;
  String btnpressed = ez.msgBox("Parameters", "Set device parameters", "Cancel#OK#Default",true);
  if (btnpressed=="OK") {
    ez.header.show("Settings..");
    m5.Lcd.setFont(&FreeSans12pt7b);
    m5.Lcd.setTextColor(TFT_BLACK,TFT_WHITE); 
    for (int i=0;i<5;i++) {                    
      k = 0;
      if (i==3) {                         // allow for manual time entry  - will be overridden later if synced
        k = 3;                            // skip timezone and DST entry
        DateTime RTCpre = rtc.now();
        SP[3][3]=RTCpre.hour();
        SP[3][4]=RTCpre.minute();
        SP[3][5]=RTCpre.month();
        SP[3][6]=RTCpre.day();
        SP[3][7]=RTCpre.year() - 2000;   // 2 digit year 
      }
      for (int j= k;j<8;j++) {      
        currentValue = SP[i][j];
        if (strlen(SPName[i][j]) > 0) {           // only show non-blank parameters
          ez.canvas.clear();
          if (i==0) ez.header.show("DEVICE settings");
          if (i==1) ez.header.show("PUNCH settings");
          if (i==2) ez.header.show("PRESS settings");
          if (i==3)  {                            // on time display
            if (RTCFound) ez.header.show("CLOCK settings"); else j = 8;                 // skip if no RTC 
          }
          if (i==4) ez.header.show("TEMPERATURE settings");
          if (SPmax[i][j] > SPmin[i][j]) {        // skip if min == max
            if ((SPmax[i][j]-SPmin[i][j])==1) {     // use yes no prompt         
              ez.buttons.show("no # Done  # yes");
              currentValue = showYesNoButton(SP[i][j],SPName[i][j]);
              if (currentValue!= SP[i][j]) SP[i][j] = currentValue;                       // set the new value  
            }
            else {
              ez.buttons.show("up # Done  # down");
              currentValue = showAnalogButton(SP[i][j],SPmin[i][j],SPmax[i][j],SPName[i][j]);
              if (currentValue!= SP[i][j]) SP[i][j] = currentValue;                       // set the new value
            }
          }  
        }  
      }
    }
    if (RTCFound) {
      myTZ.setTime(SP[3][3],SP[3][4],0,SP[3][6],SP[3][5],(SP[3][7]+2000)); 
      rtc.adjust(DateTime(myTZ.year(), myTZ.month(), myTZ.day(), myTZ.hour(), myTZ.minute(), myTZ.second()));
      Serial.print("DT=");Serial.println(myTZ.dateTime("Y,m,d,H,i,s"));                              // show the ezTime
      DateTime RTC1 = rtc.now();
      Serial.println("setting clock...");
      Serial.print("RT=");Serial.print(RTC1.year()); Serial.print(",");Serial.print(RTC1.month());    
      Serial.print(",");Serial.print(RTC1.day());Serial.print(",");Serial.print(RTC1.hour()); Serial.print(",");Serial.println(RTC1.minute());     // show the RTC clock UTC       
    }
    saveEEPROM();
    ESP.restart();                                // need to restart to get time sync right
  } 
  if (btnpressed == "Default") {
    String btnpressed2 = ez.msgBox("Restore to Factory Default", "Are You Sure ?", "YES # CANCEL #",true);
    if (btnpressed2 == "YES") {
      eraseEEPROM();                              // erase the EEPROM parameters
      ESP.restart();                              // restart
    }  
  }
}
