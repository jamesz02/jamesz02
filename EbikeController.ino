#include <thermistor.h>
#include <RTClib.h>
#include <Adafruit_Fingerprint.h>
#include <ACS712.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#define clk 27
#define dt 32
#define sw 33
#define NTC_PIN 35
#define maxOverheatValue 120
#define switchingFreq 24000
#define Buzzer_on digitalWrite(buzzer,HIGH)
#define Buzzer_off digitalWrite(buzzer,LOW)
uint16_t resistance = 4600;
THERMISTOR thermistor(NTC_PIN,        // Analog pin
                      10000,          // Nominal resistance at 25 C
                      3435,           // thermistor's beta coefficient
                      resistance);         // Value of the series resistor
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,20,4);
float wheelradius = 0.5;
byte currentSpeed = 0;
float pulseInSec,pulseInMillis,circumfence;
float distanceCalc;
unsigned long countingTime = 0;
int magnetCount = 2;
const float Pi = 3.1415926;
const int throttle = 36;
float floatOdo = 0.000;
byte dutyCycle = 0;
int driveSignal = 0;
int currentThrottle = 0;
int throttleCalibrationMax;
int throttleCalibrationMin;
float battVolt;
float battVoltChr;
int voltAdj = 80;
int powerLimit = 70;
float battPercentage;
bool charging = false;
const int voltmeter = 39;
const int voltmeter_charging = 14;
const int BMS = 26;
const int bbconverter = 25;
bool motorEnable = true;
bool drivingSystem = true;
int motorTemp;
int overheatProtect = 75;
int criticalProtect = 125;
const int buzzer = 5;
const int modeselect = 19;
const int hallsensor = 34;
bool mainScreenExe = false;
bool ecoMode = false;
bool tripView = false;
bool lowvoltCutoff = false;
bool regenBrake = true;
bool Overheat = false;
bool forceEco = false;
float lowvoltCutoff_threshold = 33.0;
float lowvoltRecovery_threshold = 36.0;
float fullCharge_threshold = 43.2;
float normalChargeVoltage = 40.8;
bool fullCharge = false;
volatile boolean TurnDetected = false;
volatile boolean up = false;
volatile boolean button = false;
volatile boolean buttonHold = false;
const long ubt = 500;
const long returnTime = 7000;
const long distanceUpdateInterval = 1000;
unsigned long TimerA = 0;
unsigned long TimerB = 0;
unsigned long TimerC = 0;
unsigned long swHoldTimer = 0;
bool timerSetB = false;
bool timerSetC = false;
int arrowpos = 0;
bool advancedSetting = false;
bool subSetting = false;
bool specificSetting = false;
bool ubstate = false;
bool secondSet = false;
bool thirdSet = false;
bool verticalSet = false;
bool adjValue = false;
//--------------------------EEPROM Address-------------------------<
int odoAddr = 0;
int lvCAddr = 5;
int lvRAddr = 25;
int fcVAddr = 50;
int pwLimitAddr = 75;
int nrChrAddr = 80;
int vAdjAddr = 95;
int wheelAddr = 100;
int magnetCountAddr = 115;
int thrCaliMxAddr = 118;
int thrCaliMnAddr = 125;
int ohTempAddr = 130;
int crtcTempAddr = 140; 
int regBrkAddr = 150;
int resistAddr = 160;
int resetWatcherAddr = 256;
bool EEPROMResetState;
#define Name2Addr 300
#define Name3Addr 310
#define Name4Addr 320
#define Name5Addr 330
#define Name6Addr 340
#define Name7Addr 350
#define Name8Addr 360
#define Name9Addr 370
//------------------------------------------->
float tripmeter = 0;
int odometer = 0;
int mA;
float Amp;
int a;
int c;
float d;
float e;
int i;
bool BH = false;
const char Character[36] = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'
,'1','2','3','4','5','6','7','8','9','0'};
char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
float mapf2i(float val1, float in_min1, float in_max1, float out_min1, float out_max1) {
    return (val1 - in_min1) * (out_max1 - out_min1) / (in_max1 - in_min1) + out_min1;
}
#define mySerial Serial2
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);
RTC_DS3231 rtc;
byte setMonth = 0;
byte setDay = 0;
byte setHour = 0;
byte setMin = 0;
byte setYear = 0;
ACS712  ACS(13, 5.0, 4095, 66);
int fingerinputnum = 1;
int fingerprintID = 0;
uint8_t id;
bool authpass = false;
const char* host = "ERide1000W36V";
const char* ssid = "ERideAP";
const char* password = "1010101010";
char commandRecv;
char commandSend;
char queryResp;
char inputCharacterSelector;
bool commandInput = false;
bool serialCommStatus;
bool sendingReady = false;
bool adminVerifySecurity = false;
const char* incomingCommandList[8] = {"00000000","GUESTDEL","GETBATTV","GETSPEED","SHUTDOWN","LOCKDOWN","FDBCLEAR","FFFFFFFF"}; // Max 8 char
const char* outgoingCommandList[] = {"CHECKSTATE","ENABLE","DISABLE","SETTINGREQ","","","","RECVOK","RESTART","FIRMWAREUP",}; // Max 10 char
const char* queryProcessingList[] = {"STATEON","STATEOFF","SETTINGREADY","SETTINGOK","SETTINGSAVE","ACK","","","","",};

class FingerName {
	public:
		char Name2;
		char Name3;
		char Name4;
		char Name5;
		char Name6;
		char Name7;
		char Name8;
		char Name9;
		char fingerNameInput;
};
FingerName FingerID;

int accessLevelControl(){
	if(fingerprintID == 1){
		return 1;
	} else if (fingerprintID >= 2 && fingerprintID <= 9){
		return 2;
	} else if (fingerprintID >= 10){
		return 3;
	}
}

WebServer server(80);
//Start Webpage line-------------------------------//
const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='jamesz02')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>Progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('Progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('Progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";
//End Webpage line----------------------------------//
byte batteryBase[] = {
  B11111,
  B10000,
  B10111,
  B10111,
  B10111,
  B10111,
  B10000,
  B11111
};
byte batteryFilled[] = {
  B11111,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B00000,
  B11111
};
byte batteryDeplete[] = {
  B11111,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111
};
byte batteryTop[] = {
  B11000,
  B01000,
  B01110,
  B01110,
  B01110,
  B01110,
  B01000,
  B11000
};
byte fingerFragment1[] = {0x00,0x18,0x1B,0x14,0x0B,0x0A,0x05,0x03};
byte fingerFragment2[] = {0x0D,0x15,0x0B,0x13,0x05,0x09,0x1E,0x18};
byte fingerFragment3[] = {0x00,0x18,0x14,0x0A,0x16,0x0D,0x13,0x09};
byte fingerFragment4[] = {0x00,0x03,0x04,0x08,0x17,0x1C,0x0B,0x14};

int EEPROMSaveNameCheck(uint8_t y){
	if(y == 2){
		return Name2Addr;
	} else if(y == 3){
		return Name3Addr;
	} else if(y == 4){
		return Name4Addr;
	} else if(y == 5){
		return Name5Addr;
	} else if(y == 6){
		return Name6Addr;
	} else if(y == 7){
		return Name7Addr;
	} else if(y == 8){
		return Name8Addr;
	} else if(y == 9){
		return Name9Addr;
	}
}

void fingerNameCheck(uint8_t x){
	if(x == 2){
		lcd.print(FingerID.Name2);
	} else if(x == 3){
		lcd.print(FingerID.Name3);
	} else if(x == 4){
		lcd.print(FingerID.Name4);
	} else if(x == 5){
		lcd.print(FingerID.Name5);
	} else if(x == 6){
		lcd.print(FingerID.Name6);
	} else if(x == 7){
		lcd.print(FingerID.Name7);
	} else if(x == 8){
		lcd.print(FingerID.Name8);
	} else if(x == 9){
		lcd.print(FingerID.Name9);
	}
}

void inputCharCheck(){
	if(inputCharacterSelector > 35){
		inputCharacterSelector = 0;
	}
}

void adminFingerVerify(){
	fingerprintID = 0;
  lcd.setCursor(0,0);
    lcd.print("---Authentication---");
  lcd.setCursor(0,1);
    lcd.print("---Admin Required---");
  lcd.setCursor(0,2);
    lcd.print("[");
    lcd.write(3);
    lcd.write(2);
  lcd.setCursor(3,2);
    lcd.print("] Fingerprint");
  lcd.setCursor(0,3);
    lcd.print("[");
    lcd.write(0);
    lcd.write(1);
  lcd.setCursor(3,3);
    lcd.print("] Awaiting...");
	while(adminVerifySecurity == false){
    fingerprintID = getFingerprintID();
    if(fingerprintID == 1){
      lcd.setCursor(3,3);
      lcd.print("] [Pass]ID Admin");
      Buzzer_on;
      delay(200);
      Buzzer_off;
	  delay(200);
	  Buzzer_on;
	  delay(200);
	  Buzzer_off;
		adminVerifySecurity = true;
		return;
	} else if(fingerprintID > 1) {
		lcd.setCursor(3,3);
		lcd.print("] [Fail]Admin Req");
		delay(800);
		break;
	}
	}
	return;
}

void serialCommListening(){
	if (Serial.available() > 0) {
    commandRecv = Serial.read();
	if(commandRecv == *incomingCommandList[0]){
		
	} else if(commandRecv == *incomingCommandList[1]){
		
	} else if(commandRecv == *incomingCommandList[2]){
		
	} else if(commandRecv == *incomingCommandList[3]){
		
	} else if(commandRecv == *incomingCommandList[4]){
		
	} else if(commandRecv == *incomingCommandList[5]){
		
	} else if(commandRecv == *incomingCommandList[6]){
		
	} else if(commandRecv == *incomingCommandList[7]){
		
	}
	}
}

void serialCommSending(){
	if(sendingReady){
	Serial.print(commandSend);
	sendingReady = false;
	}
}

void EEPROMSaveAll(){
	EEPROM.put(odoAddr,odometer);
	EEPROM.put(pwLimitAddr,powerLimit);
	EEPROM.put(wheelAddr,wheelradius);
	EEPROM.put(lvCAddr,lowvoltCutoff_threshold);
	EEPROM.put(lvRAddr,lowvoltRecovery_threshold);
	EEPROM.put(nrChrAddr,normalChargeVoltage);
	EEPROM.put(vAdjAddr,voltAdj);
	EEPROM.put(magnetCountAddr,magnetCount);
	EEPROM.put(thrCaliMxAddr,throttleCalibrationMax);
	EEPROM.put(thrCaliMnAddr,throttleCalibrationMin);
	EEPROM.put(ohTempAddr,overheatProtect);
	EEPROM.put(crtcTempAddr,criticalProtect);
	EEPROM.put(regBrkAddr,regenBrake);
	EEPROM.put(resistAddr,resistance);
	EEPROM.commit();
	EEPROM.end();
}

void lockScreen(){
	fingerprintID = 0;
  lcd.setCursor(0,0);
    lcd.print("---Authentication---");
  lcd.setCursor(0,1);
    lcd.print("------Required------");
  lcd.setCursor(0,2);
    lcd.print("[");
    lcd.write(3);
    lcd.write(2);
  lcd.setCursor(3,2);
    lcd.print("] Fingerprint");
  lcd.setCursor(0,3);
    lcd.print("[");
    lcd.write(0);
    lcd.write(1);
  lcd.setCursor(3,3);
    lcd.print("] Scanning...");
    while(authpass == false){
    fingerprintID = getFingerprintID();
    if(fingerprintID >= 1){
      lcd.setCursor(3,3);
      lcd.print("] [Pass] ID = ");
      lcd.print(fingerprintID);
      digitalWrite(buzzer,HIGH);
      delay(500);
      authpass = true;
      digitalWrite(buzzer,LOW);
      delay(600);
      lcd.clear();
      mainScreenExe = true;
      mainScreen();
    }
    }
}

void adminFingerprintRegistration(){
	id = 0;
	specificSetting = true;
  while(specificSetting == true){
  lcd.setCursor(0,0);
    lcd.print("----FINGERPRINTS----");
  lcd.setCursor(0,1);
    lcd.print("----REGISTRATION----");
  lcd.setCursor(0,2);
    lcd.print("[");
    lcd.write(3);
    lcd.write(2);
  lcd.setCursor(3,2);
    lcd.print("] Connect Device");
  lcd.setCursor(0,3);
    lcd.print("[");
    lcd.write(0);
    lcd.write(1);
  lcd.setCursor(3,3);
    lcd.print("] In Progress...");
    delay(500);
  if (finger.verifyPassword()) {
    lcd.setCursor(5,3);
    lcd.print("Connecting     ");
    delay(400);
    lcd.setCursor(5,2);
    lcd.print("Admin ID = 1   ");
    lcd.setCursor(5,3);
    lcd.print("               ");
    lcd.setCursor(5,3);
    lcd.print("Push to Registr");
    while (id == 0){
    if(button){
	id = 1;
    button = false;
    delay(500);
    }
    }
  while (!  getFingerprintEnroll() );
}
  else {
    lcd.setCursor(5,3);
    lcd.print("Connecting Fail");
    ESP.restart();
  }
}
}

int getFingerprintID() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -1;
  
  // found a match!
  return finger.fingerID; 
}

uint8_t getFingerprintEnroll() {
  int p = -1;
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    lcd.setCursor(5,2);
    lcd.print("Place Finger   ");
    lcd.setCursor(5,3);
    lcd.print("Scanning...    ");
    switch (p) {
    case FINGERPRINT_OK:
      lcd.setCursor(5,2);
    lcd.print("Fingerprint OK ");
    lcd.setCursor(5,3);
    lcd.print("Pass ID = ");
    delay(500);
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      lcd.setCursor(5,2);
    lcd.print("Comm Error     ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
      break;
    case FINGERPRINT_IMAGEFAIL:
      lcd.setCursor(5,2);
    lcd.print("Image Failed   ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(1);
  switch (p) {
    case FINGERPRINT_OK:
      lcd.setCursor(5,2);
    lcd.print("Fingerprint OK ");
    lcd.setCursor(5,3);
    lcd.print("Image Converted");
    delay(500);
      break;
    case FINGERPRINT_IMAGEMESS:
      lcd.setCursor(5,2);
    lcd.print("Image Too Messy");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      lcd.setCursor(5,2);
    lcd.print("Comm Error     ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      lcd.setCursor(5,2);
    lcd.print("Feature Fail   ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      lcd.setCursor(5,2);
    lcd.print("Invalid Image  ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
      return p;
  }

  lcd.setCursor(5,2);
    lcd.print("Remove Finger  ");
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }
    lcd.setCursor(5,3);
    lcd.print(id);
  p = -1;
  lcd.setCursor(5,2);
    lcd.print("Place Finger Ag");
    lcd.setCursor(5,3);
    lcd.print("Scanning...    ");
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      lcd.setCursor(5,2);
    lcd.print("Fingerprint OK ");
    lcd.setCursor(5,3);
    lcd.print("Pass 2");
    delay(800);
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      lcd.setCursor(5,2);
    lcd.print("Comm Error     ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
      break;
    case FINGERPRINT_IMAGEFAIL:
      lcd.setCursor(5,2);
    lcd.print("Image Fail     ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(2);
  switch (p) {
    case FINGERPRINT_OK:
      lcd.setCursor(5,2);
    lcd.print("Fingerprint OK ");
    lcd.setCursor(5,3);
    lcd.print("Image Converted");
    delay(700);
      break;
    case FINGERPRINT_IMAGEMESS:
      lcd.setCursor(5,2);
    lcd.print("Image Too Messy");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      lcd.setCursor(5,2);
    lcd.print("Comm Error     ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      lcd.setCursor(5,2);
    lcd.print("Feature Fail   ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      lcd.setCursor(5,2);
    lcd.print("Invalid Image  ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
      return p;
  }

  // OK converted!
  lcd.setCursor(5,2);
    lcd.print("Creating Model ");
    lcd.setCursor(5,3);
    lcd.print("               ");
    lcd.setCursor(5,3);
    lcd.print("For ID ");
    lcd.print(id);
    delay(1500);

  p = finger.createModel();
  if (p == FINGERPRINT_OK) {
    lcd.setCursor(5,2);
    lcd.print("Print Matched  ");
    lcd.setCursor(5,3);
    delay(800);
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    lcd.setCursor(5,2);
    lcd.print("Comm Error     ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
    return p;
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    lcd.setCursor(5,2);
    lcd.print("Print ! Matched");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
    return p;
  } else {
    lcd.setCursor(5,2);
    lcd.print("Unknown Error  ");
    lcd.setCursor(5,3);
    lcd.print("Please Restart ");
    return p;
  }

  Serial.print("ID "); Serial.println(id);
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    lcd.setCursor(5,2);
    lcd.print("Stored Success ");
    lcd.setCursor(5,3);
    lcd.print("               ");
    lcd.setCursor(5,3);
    lcd.print("ID ");
    lcd.print(id);
    delay(1500);
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    lcd.setCursor(5,2);
    lcd.print("Comm Error     ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
    return p;
  } else if (p == FINGERPRINT_BADLOCATION) {
    lcd.setCursor(5,2);
    lcd.print("Bad Location   ");
    lcd.setCursor(5,3);
    lcd.print("Try Again      ");
    return p;
  } else if (p == FINGERPRINT_FLASHERR) {
    lcd.setCursor(5,2);
    lcd.print("Flash Error    ");
    lcd.setCursor(5,3);
    lcd.print("Please Restart ");
    return p;
  } else {
    lcd.setCursor(5,2);
    lcd.print("Unknown Error  ");
    lcd.setCursor(5,3);
    lcd.print("Please Restart ");
    return p;
  }
  
  delay(500);
  lcd.clear();
  authpass = false;
  specificSetting = false;
  lockScreen();
  return true;
}

uint8_t deleteFingerprint(uint8_t id) {
  uint8_t p = -1;

  p = finger.deleteModel(id);

	lcd.setCursor(5,2);
    lcd.print("               ");
    lcd.setCursor(5,2);
  if (p == FINGERPRINT_OK) {
    lcd.print("Deleted");
	delay(500);
	fingerprintManagement();
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    lcd.print("COMM ERR");
	delay(500);
	fingerprintDetele();
    return p;
  } else if (p == FINGERPRINT_BADLOCATION) {
    lcd.print("DEL FAIL");
	delay(500);
	fingerprintDetele();
    return p;
  } else if (p == FINGERPRINT_FLASHERR) {
    lcd.print("WRITE ERR");
	delay(500);
	fingerprintDetele();
    return p;
  } else {
    lcd.print("ERR 0x??");
	delay(500);
	fingerprintDetele();
    return p;
  }
}

void universalBlinkerState(){
	if (millis() - TimerA >= ubt) {
    TimerA = millis();
    if (ubstate == LOW){
      ubstate = HIGH;
    } else {
      ubstate = LOW;
    }
  }
}

void chargingMonitor(){
	battVoltChr = analogRead(voltmeter_charging)*(voltAdj/4095);
	if(battVoltChr > 10 && battVoltChr < (fullCharge_threshold+2) && fullCharge == false){
		charging = true;
		if(battVoltChr <= normalChargeVoltage){
		digitalWrite(bbconverter,HIGH);
		} else {
		digitalWrite(bbconverter,LOW);
		}
	} else {
		charging = false;
	}
	if(battVoltChr > fullCharge_threshold+2){
		digitalWrite(BMS,LOW);
		charging = false;
	}
	if(charging == true){
		if(timerSetB == false){
			TimerB = millis();
			timerSetB = true;
		}
		if(millis() - TimerB > 1000){
			digitalWrite(BMS,HIGH);
		}
	} else {
		digitalWrite(BMS,LOW);
		timerSetB = false;
	}
}

void BMSSystem(){
	if(battVolt < lowvoltCutoff_threshold){
    lowvoltCutoff = true;
	}
	if(battVolt > lowvoltRecovery_threshold){
	lowvoltCutoff = false;
	}
	if(battVolt >= fullCharge_threshold && charging == true){
	fullCharge = true;
	}
	if(battVolt < (fullCharge_threshold-0.8) && fullCharge == true){
	fullCharge = false;
	}
}

void batterySoC(){
  battVolt = analogRead(voltmeter)*voltAdj/4095;
  battPercentage = mapf2i(battVolt,lowvoltCutoff_threshold,fullCharge_threshold,0.0,100.0);
}

void battStateReport(){
	if(charging){
		if(ubstate == HIGH){
			lcd.setCursor(6,1);
			lcd.write(7);
		}else{
			lcd.setCursor(6,1);
			lcd.print(" ");
		}
	} else {
		lcd.setCursor(6,1);
		lcd.write(7);
	}
  if (battPercentage <= 90 && battPercentage > 70){
    lcd.setCursor(0,1);
    lcd.write(4);
    lcd.write(5);
    lcd.write(5);
    lcd.write(5);
    lcd.write(5);
    lcd.write(5);
  }
  else if (battPercentage <= 70 && battPercentage > 50){
    lcd.setCursor(0,1);
    lcd.write(4);
    lcd.write(5);
    lcd.write(5);
    lcd.write(5);
    lcd.write(5);
    lcd.write(6);
  }
  else if (battPercentage <= 50 && battPercentage > 30){
    lcd.setCursor(0,1);
    lcd.write(4);
    lcd.write(5);
    lcd.write(5);
    lcd.write(5);
    lcd.write(6);
    lcd.write(6);
  }
  else if (battPercentage <= 30 && battPercentage > 10){
    lcd.setCursor(0,1);
    lcd.write(4);
    lcd.write(5);
    lcd.write(5);
    lcd.write(6);
    lcd.write(6);
    lcd.write(6);
  }
  else if (battPercentage <= 10 && battPercentage > 5){
    lcd.setCursor(0,1);
    lcd.write(4);
    lcd.write(5);
    lcd.write(6);
    lcd.write(6);
    lcd.write(6);
    lcd.write(6);
  }
  else if (battPercentage <= 5){
	  if(ubstate == HIGH){
			lcd.setCursor(0,1);
			lcd.write(4);
		}else{
			lcd.setCursor(0,1);
			lcd.write(6);
		}
    lcd.setCursor(1,1);
    lcd.write(6);
    lcd.write(6);
    lcd.write(6);
    lcd.write(6);
    lcd.write(6);
  }
}

void currentMonitor(){
	mA = ACS.mA_DC();
	Amp = mA/1000;
	if(Amp > 0.0 ){
	lcd.setCursor(15,1);
	lcd.print(Amp, 1);
	}else if(Amp < 0.0){
	lcd.setCursor(14,1);
	lcd.print(Amp, 1);
	}
	if(Amp < 10.0 || Amp > -10.0){
	lcd.setCursor(18,1);
	lcd.print(" ");
	}
	lcd.setCursor(19,1);
	lcd.print("A");
}

void motorCurrent(){
	lcd.setCursor(6,0);
	lcd.print("[");
	lcd.setCursor(11,0);
	lcd.print("|");
	lcd.setCursor(19,0);
	lcd.print("]");
	if(mA > 30000){
	lcd.setCursor(7,0);
	lcd.print("    ");
	lcd.setCursor(12,0);
	lcd.print("#######");
	}
	else if(mA > 24000 && mA < 30000){
	lcd.setCursor(7,0);
	lcd.print("    ");
	lcd.setCursor(12,0);
	lcd.print("###### ");
	}
	else if(mA > 18000 && mA < 24000){
	lcd.setCursor(7,0);
	lcd.print("    ");
	lcd.setCursor(12,0);
	lcd.print("#####  ");
	}
	else if(mA > 13000 && mA < 18000){
	lcd.setCursor(7,0);
	lcd.print("    ");
	lcd.setCursor(12,0);
	lcd.print("####   ");
	}
	else if(mA > 8000 && mA < 13000){
	lcd.setCursor(7,0);
	lcd.print("    ");
	lcd.setCursor(12,0);
	lcd.print("###    ");
	}
	else if(mA > 4000 && mA < 8000){
	lcd.setCursor(7,0);
	lcd.print("    ");
	lcd.setCursor(12,0);
	lcd.print("##     ");
	}
	else if(mA > 2000 && mA < 4000){
	lcd.setCursor(7,0);
	lcd.print("    ");
	lcd.setCursor(12,0);
	lcd.print("#      ");
	}
	else if(mA > -1000 && mA <= 2000){
	lcd.setCursor(7,0);
	lcd.print("    ");
	lcd.setCursor(12,0);
	lcd.print("       ");
	}
	else if(mA < -1000 && mA > -4000){
	lcd.setCursor(7,0);
	lcd.print("   #");
	lcd.setCursor(12,0);
	lcd.print("       ");
	}
	else if(mA < -4000 && mA > -8000){
	lcd.setCursor(7,0);
	lcd.print("  ##");
	lcd.setCursor(12,0);
	lcd.print("       ");
	}
	else if(mA < -8000 && mA > -12000){
	lcd.setCursor(7,0);
	lcd.print(" ###");
	lcd.setCursor(12,0);
	lcd.print("       ");
	}
	else if(mA < -12000){
	lcd.setCursor(7,0);
	lcd.print("####");
	lcd.setCursor(12,0);
	lcd.print("       ");
	}
}

void defaultLine(){
	secondSet = false;
	thirdSet = false;
	verticalSet = false;
}

void secondLine(){
	secondSet = true;
	thirdSet = false;
	verticalSet = false;
}

void thirdLine(){
	secondSet = false;
	thirdSet = true;
	verticalSet = false;
}
	
void verticalLine(){
	secondSet = false;
	thirdSet = false;
	verticalSet = true;
}
	
void arrowpos1(){
    lcd.setCursor(0,1);
    lcd.print("=>");
    lcd.setCursor(0,2);
    lcd.print("  ");
    lcd.setCursor(0,3);
    lcd.print("  ");
    lcd.setCursor(9,1);
    lcd.print("  ");
    lcd.setCursor(9,2);
    lcd.print("  ");
    lcd.setCursor(9,3);
    lcd.print("  ");
}
void arrowpos2(){
	if(secondSet == true){
	lcd.setCursor(0,2);
    lcd.print("=>");
    lcd.setCursor(0,3);
    lcd.print("  ");
    lcd.setCursor(9,2);
    lcd.print("  ");
    lcd.setCursor(9,3);
    lcd.print("  ");
	}
	if(verticalSet == true){
	lcd.setCursor(0,2);
    lcd.print("=>");
    lcd.setCursor(0,3);
    lcd.print("  ");
	} 
	if (secondSet == false && verticalSet == false) {
	lcd.setCursor(0,1);
    lcd.print("  ");
    lcd.setCursor(0,2);
    lcd.print("=>");
    lcd.setCursor(0,3);
    lcd.print("  ");
    lcd.setCursor(9,1);
    lcd.print("  ");
    lcd.setCursor(9,2);
    lcd.print("  ");
    lcd.setCursor(9,3);
    lcd.print("  ");
	}
}
void arrowpos3(){
    if(secondSet == true){
	lcd.setCursor(0,2);
    lcd.print("  ");
    lcd.setCursor(0,3);
    lcd.print("=>");
    lcd.setCursor(9,2);
    lcd.print("  ");
    lcd.setCursor(9,3);
    lcd.print("  ");
	}
	if(thirdSet == true){
	lcd.setCursor(0,3);
    lcd.print("=>");
    lcd.setCursor(9,3);
    lcd.print("  ");
	}
	if(verticalSet == true){
	lcd.setCursor(0,2);
    lcd.print("  ");
    lcd.setCursor(0,3);
    lcd.print("=>");
	} 
	if (secondSet == false && thirdSet == false && verticalSet == false) {
	lcd.setCursor(0,1);
    lcd.print("  ");
    lcd.setCursor(0,2);
    lcd.print("  ");
    lcd.setCursor(0,3);
    lcd.print("=>");
    lcd.setCursor(9,1);
    lcd.print("  ");
    lcd.setCursor(9,2);
    lcd.print("  ");
    lcd.setCursor(9,3);
    lcd.print("  ");
	}
}
void arrowpos4(){
      lcd.setCursor(0,1);
    lcd.print("  ");
    lcd.setCursor(0,2);
    lcd.print("  ");
    lcd.setCursor(0,3);
    lcd.print("  ");
    lcd.setCursor(9,1);
    lcd.print("=>");
    lcd.setCursor(9,2);
    lcd.print("  ");
    lcd.setCursor(9,3);
    lcd.print("  ");
}
void arrowpos5(){
    if(secondSet == true){
	lcd.setCursor(0,2);
    lcd.print("  ");
    lcd.setCursor(0,3);
    lcd.print("  ");
    lcd.setCursor(9,2);
    lcd.print("=>");
    lcd.setCursor(9,3);
    lcd.print("  ");
	} else {
	lcd.setCursor(0,1);
    lcd.print("  ");
    lcd.setCursor(0,2);
    lcd.print("  ");
    lcd.setCursor(0,3);
    lcd.print("  ");
    lcd.setCursor(9,1);
    lcd.print("  ");
    lcd.setCursor(9,2);
    lcd.print("=>");
    lcd.setCursor(9,3);
    lcd.print("  ");
	}
}
void arrowpos6(){
    if(secondSet == true){
	lcd.setCursor(0,2);
    lcd.print("  ");
    lcd.setCursor(0,3);
    lcd.print("  ");
    lcd.setCursor(9,2);
    lcd.print("  ");
    lcd.setCursor(9,3);
    lcd.print("=>");
	}
	if(thirdSet == true){
	lcd.setCursor(0,3);
    lcd.print("  ");
    lcd.setCursor(9,3);
    lcd.print("=>");
	}
	if (secondSet == false && thirdSet == false) {
	lcd.setCursor(0,1);
    lcd.print("  ");
    lcd.setCursor(0,2);
    lcd.print("  ");
    lcd.setCursor(0,3);
    lcd.print("  ");
    lcd.setCursor(9,1);
    lcd.print("  ");
    lcd.setCursor(9,2);
    lcd.print("  ");
    lcd.setCursor(9,3);
    lcd.print("=>");
	}
}

void motorTempProtect(){
  if (motorTemp >= overheatProtect){
    motorEnable = true;
	Overheat = true;
  }
  else if (motorTemp >= criticalProtect){
    motorEnable = false;
	Overheat = true;
  }
}

void logicalOperation1(){
	if(motorEnable == true && Overheat == true){
		forceEco = true;
		drivingSystem = true;
	}
	else if(motorEnable == false){
		drivingSystem = false;
	}
}

void logicalOperation2(){
	if(lowvoltCutoff == true){
		motorEnable == false;
	} else {
		motorEnable == true;
	}
}

void motorDriveOperation(){
	currentThrottle = analogRead(throttle);
	if(ecoMode == true || forceEco == true){
		d = float(powerLimit) / 100;
		e = 255.0 * d;
		i = e;
		driveSignal = map(currentThrottle,throttleCalibrationMin,throttleCalibrationMax,0,i);
		driveSignal = constrain(driveSignal,0,i);
	} else {
		driveSignal = map(currentThrottle,throttleCalibrationMin,throttleCalibrationMax,0,255);
		driveSignal = constrain(driveSignal,0,255);
	}
	if(drivingSystem == true || motorEnable == true){
		ledcWrite(0,driveSignal);
	}
	dutyCycle = map(driveSignal,0,255,0,99);
}

void speedCalc(){
	if(countingTime != 0){
		pulseInMillis = countingTime*0.001;
		circumfence = 2*Pi*wheelradius;
		currentSpeed = 3600*circumfence/pulseInMillis/magnetCount;
	} else {
		currentSpeed = 0;
	}
}

void odoCounting(){
  if(millis() - TimerC >= distanceUpdateInterval){
	  TimerC = millis();
	  distanceCalc = (currentSpeed/3600)*(distanceUpdateInterval);
	  floatOdo = floatOdo+distanceCalc;
	  tripmeter = floatOdo;
	  odometer = floatOdo;
  }
}

void IRAM_ATTR isr1() {
  if (digitalRead(sw) == LOW) {
    button = true;
	buttonHold = true;
  }
}

void IRAM_ATTR isr0()  {
  TurnDetected = true;
  up = (digitalRead(clk) == digitalRead(dt));
}

void buttonHoldMeasure(){
	if(buttonHold){
		swHoldTimer = millis();
		buttonHold = false;
	}
	if(!buttonHold && millis() - swHoldTimer >= 1500 && digitalRead(sw) == LOW){
		BH = true;
		button = false;
	}
}

void timeDisplay(){
	lcd.setCursor(5,0);
	DateTime now = rtc.now();
	if(now.day() < 10){
		lcd.print("0");
	lcd.print(now.day(),DEC);
	} else {
	lcd.print(now.day(),DEC);
	}
	lcd.print("/");
	if(now.month() < 10){
		lcd.print("0");
	lcd.print(now.month(),DEC);
	} else {
		lcd.print(now.month(),DEC);
	}
	lcd.print(" ");
	if(now.hour() < 10){
		lcd.print("0");
	lcd.print(now.hour(),DEC);
	} else {
		lcd.print(now.hour(),DEC);
	}
	lcd.print(":");
	if(now.minute() < 10){
		lcd.print("0");
	lcd.print(now.minute(),DEC);
	} else {
		lcd.print(now.minute(),DEC);
	}
	lcd.print(" ");
	lcd.print(daysOfTheWeek[now.dayOfTheWeek()]);
}

void fingerprintManagement(){
	arrowpos = 2;
  while(subSetting == true){
  lcd.home();
  lcd.print("Authentication Setup");
    lcd.setCursor(0,1);
  lcd.print("FingerpringManagemnt");
    lcd.setCursor(2,2);
  lcd.print("Back");
    lcd.setCursor(2,3);
  lcd.print("Regist");
    lcd.setCursor(11,2);
  lcd.print("Delete");
    lcd.setCursor(11,3);
  lcd.print("ClearData");
  secondLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
    if(arrowpos >= 6){
      arrowpos = 2;
    }
    if(arrowpos < 2){
      arrowpos = 5;
    }
    switch(arrowpos){
      case 2:
      arrowpos2();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
    commLinkSetting();
    }
      break;
      case 3://---------------Finger Enroll-------------
      arrowpos3();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
    fingerprintRegistration();
    }
      break;
      case 4://---------------Delete Finger---------------
      arrowpos5();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	fingerprintDetele();
    }
      break;
      case 5://------------------Wipe Database----------------
      arrowpos6();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	fingerprintEraseDatabase();
    }
      break;
    }
  }
}

void fingerprintRegistration(){
	id = 0;
	specificSetting = true;
  while(specificSetting == true){
  lcd.setCursor(0,0);
    lcd.print("----FINGERPRINTS----");
  lcd.setCursor(0,1);
    lcd.print("----REGISTRATION----");
  lcd.setCursor(0,2);
    lcd.print("[");
    lcd.write(3);
    lcd.write(2);
  lcd.setCursor(3,2);
    lcd.print("] Connect Device");
  lcd.setCursor(0,3);
    lcd.print("[");
    lcd.write(0);
    lcd.write(1);
  lcd.setCursor(3,3);
    lcd.print("] In Progress...");
    delay(500);
  if (finger.verifyPassword()) {
    lcd.setCursor(5,3);
    lcd.print("Connecting     ");
    delay(400);
    lcd.setCursor(5,2);
    lcd.print("OVWrN:");
    lcd.setCursor(5,3);
    lcd.print("               ");
    lcd.setCursor(5,3);
    lcd.print("ID # ");
    while (id == 0){
		universalBlinkerState();
      lcd.setCursor(10,3);
    lcd.print(fingerinputnum);
	if(ubstate == true){
		lcd.cursor();
	} else {
		lcd.noCursor();
	}
    if(TurnDetected){
      if(up){
        fingerinputnum = fingerinputnum + 1;
		lcd.setCursor(11,2);
		lcd.print("         ");
        TurnDetected = false;
      }
      if(!up){
        fingerinputnum = fingerinputnum - 1;
		lcd.setCursor(11,2);
		lcd.print("         ");
        TurnDetected = false;
      }
    }
	lcd.setCursor(11,2);
	fingerNameCheck(fingerinputnum);
    fingerinputnum = min(fingerinputnum, 99);
  fingerinputnum = max(fingerinputnum, 2);
    if(button){
		button = false;
		id = fingerinputnum;
		lcd.noCursor();
		lcd.setCursor(5,2);
		lcd.print("               ");
		lcd.setCursor(5,2);
		lcd.print("ID Confirm");
		delay(500);
	if(id > 1 && id < 10){
		lcd.setCursor(5,2);
		lcd.print("Enter Name ID ");
		lcd.print(id);
		lcd.setCursor(5,3);
		lcd.print("               ");
		FingerID.fingerNameInput = 0;
		inputCharacterSelector = 0;
	while(id != 0){
		lcd.setCursor(5,3);
		lcd.print("Name:");
		lcd.print(FingerID.fingerNameInput);
		lcd.print(inputCharacterSelector);
		if(TurnDetected){
      if(up){
        inputCharacterSelector++;
        TurnDetected = false;
      }
      if(!up){
        inputCharacterSelector--;
        TurnDetected = false;
      }
		}
		if(BH && inputCharacterSelector == 35){
			BH = false;
			button = false;
			FingerID.fingerNameInput = 0;
		} else if(BH){
			BH = false;
			button = false;
			lcd.setCursor(5,2);
			lcd.print("Name Comfirm   ");
			delay(400);
			EEPROM.put(EEPROMSaveNameCheck(id),FingerID.fingerNameInput);
			EEPROM.commit();
			EEPROM.end();
			break;
		} else if(button && digitalRead(sw) == HIGH) {
			if(FingerID.fingerNameInput == 0){
				FingerID.fingerNameInput = Character[inputCharacterSelector];
				button = false;
			} else {
				FingerID.fingerNameInput += Character[inputCharacterSelector];
				button = false;
			}
		}
	}
	} else if(id >= 10){
		break;
	}
   }
  }
  while (!  getFingerprintEnroll() );
}
  else {
    lcd.setCursor(5,3);
    lcd.print("Connecting Fail");
    ESP.restart();
  }
}
}

void fingerprintDetele(){
	id = 0;
	while(specificSetting == true){
  lcd.setCursor(0,0);
    lcd.print("----FINGERPRINTS----");
  lcd.setCursor(0,1);
    lcd.print("-------DELETE-------");
  lcd.setCursor(0,2);
    lcd.print("[");
    lcd.write(3);
    lcd.write(2);
  lcd.setCursor(3,2);
    lcd.print("] Connect Device");
  lcd.setCursor(0,3);
    lcd.print("[");
    lcd.write(0);
    lcd.write(1);
  lcd.setCursor(3,3);
    lcd.print("] In Progress...");
    delay(500);
  if (finger.verifyPassword()) {
    lcd.setCursor(5,3);
    lcd.print("Connecting     ");
    delay(400);
    lcd.setCursor(5,2);
    lcd.print("Name = ");
    lcd.setCursor(5,3);
    lcd.print("               ");
    lcd.setCursor(5,3);
    lcd.print("ID # ");
    while (id == 0){
		universalBlinkerState();
      lcd.setCursor(10,3);
    lcd.print(fingerinputnum);
	if(ubstate = true){
      lcd.cursor();
	} else {
		lcd.noCursor();
	}
    if(TurnDetected){
      if(up){
        fingerinputnum = fingerinputnum + 1;
        TurnDetected = false;
      }
      if(!up){
        fingerinputnum = fingerinputnum - 1;
        TurnDetected = false;
      }
    }
    fingerinputnum = min(fingerinputnum, 99);
  fingerinputnum = max(fingerinputnum, 2);
    lcd.setCursor(12,2);
	fingerNameCheck(fingerinputnum);
    if(button){
    id = fingerinputnum;
    lcd.noCursor();
    lcd.setCursor(5,2);
    lcd.print("               ");
    lcd.setCursor(5,2);
    lcd.print("Delete ID");
    button = false;
    delay(500);
	deleteFingerprint(id);
	id = 0;
	fingerprintManagement();
    }
    }
  }
  }
}

void fingerprintEraseDatabase(){
	arrowpos = 1;
	while(specificSetting == true){
	lcd.home();
  lcd.print("---Erase Database---");
	lcd.setCursor(0,1);
  lcd.print("Fingerprint Clear DB");
	lcd.setCursor(0,2);
  lcd.print("    Are you Sure?   ");
	lcd.setCursor(2,3);
  lcd.print("Yes");
    lcd.setCursor(11,3);
  lcd.print("No");
  thirdLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos3();
		if (button){
		lcd.setCursor(0,2);
        lcd.print("   Finger Wiping    ");
		finger.emptyDatabase();
		delay(500);
		lcd.setCursor(0,2);
        lcd.print("  Memory Clear Done ");
		delay(500);
		specificSetting = false;
		subSetting = true;
		button = false;
		lcd.clear();
		adminFingerprintRegistration();
		}
		break;
		case 1:
		arrowpos6();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		lcd.clear();
		fingerprintManagement();
		}
		break;
	}
  }
}

void odoSpdSetting(){
		arrowpos = 2;
  while(subSetting == true){
    lcd.home();
  lcd.print("------Setting-------");
    lcd.setCursor(0,1);
  lcd.print("Odo & Spd Calibrate ");
    lcd.setCursor(2,2);
  lcd.print("Back");
    lcd.setCursor(2,3);
  lcd.print("Magnet");
    lcd.setCursor(11,2);
  lcd.print("Wheel");
    lcd.setCursor(11,3);
  lcd.print("Trip");
  secondLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
    if(arrowpos >= 6){
      arrowpos = 2;
    }
    if(arrowpos < 2){
      arrowpos = 5;
    }
    switch(arrowpos){
      case 2:
      arrowpos2();
      if (button){
    subSetting = false;
    button = false;
    lcd.clear();
    settingScreen();
    }
      break;
      case 3://---------------Motor Temperature Calibration----
      arrowpos3();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	magnetSetting();
    }
      break;
      case 4://-----------------Battery Voltage Calibration-----
      arrowpos5();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	wheelDiameterSetting();
    }
      break;
      case 5://----------------Throttle Calibration---------
      arrowpos6();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
    TimerC = millis();
    tripReset();
    }
      break;
    }
  }
}

void magnetSetting(){
		arrowpos = 0;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("Hall Sensor Quantity");
	lcd.setCursor(2,2);
  lcd.print("Magnet = ");
	lcd.setCursor(2,3);
  lcd.print("Save & Exit");
  verticalLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos2();
		if(button){
			lcd.setCursor(18,2);
			lcd.print("<-");
			button = false;
			adjValue = true;
			while(adjValue == true){
				lcd.setCursor(14,2);
				lcd.print(magnetCount);
				magnetCount = min(magnetCount,12);
				magnetCount = max(magnetCount,1);
				if(TurnDetected){
					if(up){
						magnetCount = magnetCount + 1;
						TurnDetected = false;
					}
					if(!up){
						magnetCount = magnetCount - 1;
						TurnDetected = false;
					}
				}
				if(button){
					adjValue = false;
					lcd.setCursor(18,2);
					lcd.print("  ");
					button = false;
					break;
				}
			}
		}
		break;
		case 1:
		arrowpos3();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		EEPROM.put(magnetCountAddr,magnetCount);
		EEPROM.commit();
		lcd.clear();
		odoSpdSetting();
		}
		break;
	}
	}
}

void wheelDiameterSetting(){
	arrowpos = 0;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("Wheel Size in Meter ");
	lcd.setCursor(2,2);
  lcd.print("Diameter = ");
	lcd.setCursor(2,3);
  lcd.print("Save & Exit");
  verticalLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos2();
		if(button){
			lcd.setCursor(18,2);
			lcd.print("<-");
			button = false;
			adjValue = true;
			while(adjValue == true){
				lcd.setCursor(12,2);
				lcd.print(wheelradius,2);
				if(TurnDetected){
					if(up){
						wheelradius = wheelradius + 0.01;
						TurnDetected = false;
					}
					if(!up){
						wheelradius = wheelradius - 0.01;
						TurnDetected = false;
					}
				}
				if(button){
					adjValue = false;
					lcd.setCursor(18,2);
					lcd.print("  ");
					button = false;
					break;
				}
			}
		}
		break;
		case 1:
		arrowpos3();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		EEPROM.put(wheelAddr,wheelradius);
		EEPROM.commit();
		lcd.clear();
		odoSpdSetting();
		}
		break;
	}
	}
}

void tripReset(){
	arrowpos = 1;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("--Trip Meter Reset--");
	lcd.setCursor(0,2);
  lcd.print("    Are you Sure?   ");
	lcd.setCursor(2,3);
  lcd.print("Yes");
    lcd.setCursor(11,3);
  lcd.print("No");
  thirdLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos3();
		if (button){
		lcd.setCursor(0,2);
		tripmeter = 0;
        lcd.print("     Trip Reset     ");
		delay(500);;
		specificSetting = false;
		subSetting = true;
		button = false;
		lcd.clear();
		odoSpdSetting();
		}
		break;
		case 1:
		arrowpos6();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		lcd.clear();
		odoSpdSetting();
		}
		break;
	}
  }
}

void sensorSetting(){
	arrowpos = 2;
  while(subSetting == true){
    lcd.home();
  lcd.print("------Setting-------");
    lcd.setCursor(0,1);
  lcd.print("Sensor & Calibration");
    lcd.setCursor(2,2);
  lcd.print("Back");
    lcd.setCursor(2,3);
  lcd.print("MtTemp");
    lcd.setCursor(11,2);
  lcd.print("BattVolt");
    lcd.setCursor(11,3);
  lcd.print("Throttle");
  secondLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
    if(arrowpos >= 6){
      arrowpos = 2;
    }
    if(arrowpos < 2){
      arrowpos = 5;
    }
    switch(arrowpos){
      case 2:
      arrowpos2();
      if (button){
    subSetting = false;
    button = false;
    lcd.clear();
    settingScreen();
    }
      break;
      case 3://---------------Motor Temperature Calibration----
      arrowpos3();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	motorTempCalibration();
    }
      break;
      case 4://-----------------Battery Voltage Calibration-----
      arrowpos5();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	battVoltCalibration();
    }
      break;
      case 5://----------------Throttle Calibration---------
      arrowpos6();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
    TimerC = millis();
    throttleCalibration();
    }
      break;
    }
  }
}

void motorTempCalibration(){
	arrowpos = 0;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("Motor Temp Sense Adj");
	lcd.setCursor(2,2);
  lcd.print("ResistV =");
	lcd.setCursor(2,3);
  lcd.print("Save & Exit");
  verticalLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos2();
		if(button){
			lcd.setCursor(18,2);
			lcd.print("<-");
			button = false;
			adjValue = true;
			while(adjValue == true){
				lcd.setCursor(12,2);
				lcd.print(resistance);
				if(TurnDetected){
					if(up){
						resistance = resistance + 10;
						TurnDetected = false;
					}
					if(!up){
						resistance = resistance - 10;
						TurnDetected = false;
					}
				}
				if(button){
					adjValue = false;
					lcd.setCursor(18,2);
					lcd.print("  ");
					button = false;
					break;
				}
			}
		}
		break;
		case 1:
		arrowpos3();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		EEPROM.put(resistAddr,resistance);
		EEPROM.commit();
		lcd.clear();
		sensorSetting();
		}
		break;
	}
	}
}

void battVoltCalibration(){
	arrowpos = 0;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("Voltmeter Offset Adj");
	lcd.setCursor(2,2);
  lcd.print("Reference =");
	lcd.setCursor(2,3);
  lcd.print("Save & Exit");
  verticalLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos2();
		if(button){
			lcd.setCursor(18,2);
			lcd.print("<-");
			button = false;
			adjValue = true;
			while(adjValue == true){
				lcd.setCursor(14,2);
				lcd.print(voltAdj);
				if(TurnDetected){
					if(up){
						voltAdj = voltAdj + 1;
						TurnDetected = false;
					}
					if(!up){
						voltAdj = voltAdj - 1;
						TurnDetected = false;
					}
				}
				if(button){
					adjValue = false;
					lcd.setCursor(18,2);
					lcd.print("  ");
					button = false;
					break;
				}
			}
		}
		break;
		case 1:
		arrowpos3();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		EEPROM.put(vAdjAddr,voltAdj);
		EEPROM.commit();
		lcd.clear();
		sensorSetting();
		}
		break;
	}
	}
}

void throttleCalibration(){
  lcd.home();
  lcd.print("------Setting-------");
  lcd.setCursor(0,1);
  lcd.print("Throttle Calibration");
  throttleCalibrationMin = 4095;
  throttleCalibrationMax = 0;
  while (millis() - TimerC <= returnTime) {
    currentThrottle = analogRead(throttle);
	universalBlinkerState();
	if(ubstate == true){
    lcd.setCursor(0,2);
    lcd.print(" Pls Twist the Grip ");
	} else {
	lcd.setCursor(0,2);
    lcd.print("                    ");
	}
    if(currentThrottle > throttleCalibrationMax){
      throttleCalibrationMax = currentThrottle;
    }
    if(currentThrottle < throttleCalibrationMin){
      throttleCalibrationMin = currentThrottle;
    }
    lcd.setCursor(1,3);
    lcd.print("Min ");
    lcd.print(throttleCalibrationMin);
    lcd.setCursor(9,3);
    lcd.print("Max ");
    lcd.print(throttleCalibrationMax);
  }
	if(throttleCalibrationMin > 20){
		throttleCalibrationMin -= 20;
	}
  lcd.setCursor(0,2);
    lcd.print("  Calibration Done  ");
    lcd.setCursor(0,3);
    lcd.print("   Save to EEPROM   ");
  EEPROM.put(thrCaliMnAddr,throttleCalibrationMin);
  EEPROM.put(thrCaliMxAddr,throttleCalibrationMax);
  EEPROM.commit();
  delay(1000);
  lcd.clear();
  subSetting = true;
  sensorSetting();
}

void motorDriveParameter(){
	arrowpos = 2;
  while(subSetting == true){
    lcd.home();
  lcd.print("------Setting-------");
    lcd.setCursor(0,1);
  lcd.print("Motor & Power Config");
    lcd.setCursor(2,2);
  lcd.print("Back");
    lcd.setCursor(2,3);
  lcd.print("EcoMode");
    lcd.setCursor(11,2);
  lcd.print("TempProtc");
    lcd.setCursor(11,3);
  lcd.print("RegenBrak");
  secondLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
    if(arrowpos >= 6){
      arrowpos = 2;
    }
    if(arrowpos < 2){
      arrowpos = 5;
    }
    switch(arrowpos){
      case 2:
      arrowpos2();
      if (button){
    subSetting = false;
    button = false;
    lcd.clear();
    settingScreen();
    }
      break;
      case 3://----------------Eco Mode-----------------
      arrowpos3();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	ecoModeParameter();
    }
      break;
      case 4://-------------------Temperature Protect Para----
      arrowpos5();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	motorTempParameter();
    }
      break;
      case 5:
      arrowpos6();//---------------Regenerative Brake-----------
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
	arrowpos = 0;
    lcd.clear();
	regenerativeBrake();
    }
      break;
    }
  }
}

void ecoModeParameter(){
	arrowpos = 0;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("Motor PowerCapLimit ");
	lcd.setCursor(2,2);
  lcd.print("Max Power =");
	lcd.setCursor(2,3);
  lcd.print("Save & Exit");
  verticalLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos2();
		if(button){
			lcd.setCursor(18,2);
			lcd.print("<-");
			button = false;
			adjValue = true;
			while(adjValue == true){
				lcd.setCursor(15,2);
				lcd.print(powerLimit);
				if(TurnDetected){
					if(up){
						powerLimit = powerLimit + 1;
						TurnDetected = false;
					}
					if(!up){
						powerLimit = powerLimit - 1;
						TurnDetected = false;
					}
				}
				if(button){
					adjValue = false;
					lcd.setCursor(18,2);
					lcd.print("  ");
					button = false;
					break;
				}
			}
		}
		break;
		case 1:
		arrowpos3();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		EEPROM.put(pwLimitAddr,powerLimit);
		EEPROM.commit();
		lcd.clear();
		motorDriveParameter();
		}
		break;
	}
	}
}

void motorTempParameter(){
	arrowpos = 0;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("Motor Overheat Temps");
	lcd.setCursor(2,2);
  lcd.print("Temp =");
	lcd.setCursor(2,3);
  lcd.print("Save & Exit");
  verticalLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos2();
		if(button){
			lcd.setCursor(18,2);
			lcd.print("<-");
			button = false;
			adjValue = true;
			while(adjValue == true){
				lcd.setCursor(9,2);
				lcd.print(overheatProtect);
				min(overheatProtect,maxOverheatValue);
				if(TurnDetected){
					if(up){
						overheatProtect = overheatProtect + 1;
						TurnDetected = false;
					}
					if(!up){
						overheatProtect = overheatProtect - 1;
						TurnDetected = false;
					}
				}
				if(button){
					adjValue = false;
					lcd.setCursor(18,2);
					lcd.print("  ");
					button = false;
					break;
				}
			}
		}
		break;
		case 1:
		arrowpos3();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		EEPROM.put(ohTempAddr,overheatProtect);
		EEPROM.commit();
		lcd.clear();
		motorDriveParameter();
		}
		break;
	}
	}
}

void regenerativeBrake(){
	arrowpos = 0;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("Regenerative Braking");
	lcd.setCursor(0,2);
  lcd.print(" Status = ");
	lcd.setCursor(2,3);
  lcd.print("Enable");
    lcd.setCursor(11,3);
  lcd.print("Disable");
  thirdLine();
  if(regenBrake == true){
	  lcd.setCursor(10,2);
	  lcd.print("Enable ");
  } else {
	  lcd.setCursor(10,2);
	  lcd.print("Disable");
  }
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos3();
		if (button){
			regenBrake = true;
		specificSetting = false;
		subSetting = true;
		button = false;
		lcd.clear();
		EEPROM.put(regBrkAddr,regenBrake);
		EEPROM.commit();
		motorDriveParameter();
		}
		break;
		case 1:
		arrowpos6();
		if (button){
			regenBrake = false;
		specificSetting = false;
		subSetting = true;
		button = false;
		lcd.clear();
		EEPROM.put(regBrkAddr,regenBrake);
		EEPROM.commit();
		motorDriveParameter();
		}
		break;
	}
	}
}

void BMSParameterSetting(){
	arrowpos = 2;
  while(subSetting == true){
    lcd.home();
  lcd.print("------Setting-------");
    lcd.setCursor(0,1);
  lcd.print("BMS Parameter Config");
    lcd.setCursor(2,2);
  lcd.print("Back");
    lcd.setCursor(2,3);
  lcd.print("CutoffV");
    lcd.setCursor(11,2);
  lcd.print("FullChrgV");
    lcd.setCursor(11,3);
  lcd.print("NormChrgV");
  secondLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
    if(arrowpos >= 6){
      arrowpos = 2;
    }
    if(arrowpos < 2){
      arrowpos = 5;
    }
    switch(arrowpos){
      case 2:
      arrowpos2();
      if (button){
    subSetting = false;
    button = false;
    lcd.clear();
    settingScreen();
    }
      break;
      case 3://-----------------Cutoff V-----------------
      arrowpos3();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	cutoffVoltage();
    }
      break;
      case 4://----------------Full Charge V-----------
      arrowpos5();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	fullchargeVoltage();
    }
      break;
      case 5:///----------------Normal V--------------
      arrowpos6();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	chargeCheckVoltage();
    }
      break;
    }
  }
}

void cutoffVoltage(){
	arrowpos = 0;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("BMS - Cutoff Voltage");
	lcd.setCursor(2,2);
  lcd.print("Voltage = ");
	lcd.setCursor(2,3);
  lcd.print("Save & Exit");
  verticalLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos2();
		if(button){
			lcd.setCursor(18,2);
			lcd.print("<-");
			button = false;
			adjValue = true;
			while(adjValue == true){
				lcd.setCursor(12,2);
				lcd.print(lowvoltCutoff_threshold);
				if(TurnDetected){
					if(up){
						lowvoltCutoff_threshold = lowvoltCutoff_threshold + 0.1;
						TurnDetected = false;
					}
					if(!up){
						lowvoltCutoff_threshold = lowvoltCutoff_threshold - 0.1;
						TurnDetected = false;
					}
				}
				if(button){
					adjValue = false;
					lcd.setCursor(18,2);
					lcd.print("  ");
					button = false;
					break;
				}
			}
		}
		break;
		case 1:
		arrowpos3();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		EEPROM.put(lvCAddr,lowvoltCutoff_threshold);
		EEPROM.commit();
		lcd.clear();
		BMSParameterSetting();
		}
		break;
	}
	}
}

void fullchargeVoltage(){
	arrowpos = 0;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("BMS Full Chr Voltage");
	lcd.setCursor(2,2);
  lcd.print("Voltage = ");
	lcd.setCursor(2,3);
  lcd.print("Save & Exit");
  verticalLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos2();
		if(button){
			lcd.setCursor(18,2);
			lcd.print("<-");
			button = false;
			adjValue = true;
			while(adjValue == true){
				lcd.setCursor(12,2);
				lcd.print(fullCharge_threshold);
				if(TurnDetected){
					if(up){
						fullCharge_threshold = fullCharge_threshold + 0.1;
						TurnDetected = false;
					}
					if(!up){
						fullCharge_threshold = fullCharge_threshold - 0.1;
						TurnDetected = false;
					}
				}
				if(button){
					adjValue = false;
					lcd.setCursor(18,2);
					lcd.print("  ");
					button = false;
					break;
				}
			}
		}
		break;
		case 1:
		arrowpos3();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		EEPROM.put(fcVAddr,fullCharge_threshold);
		EEPROM.commit();
		lcd.clear();
		BMSParameterSetting();
		}
		break;
	}
	}
}

void chargeCheckVoltage(){
	arrowpos = 0;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("BMS Norminal Voltage");
	lcd.setCursor(2,2);
  lcd.print("Voltage = ");
	lcd.setCursor(2,3);
  lcd.print("Save & Exit");
  verticalLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos2();
		if(button){
			lcd.setCursor(18,2);
			lcd.print("<-");
			button = false;
			adjValue = true;
			while(adjValue == true){
				lcd.setCursor(12,2);
				lcd.print(normalChargeVoltage);
				if(TurnDetected){
					if(up){
						normalChargeVoltage = normalChargeVoltage + 0.1;
						TurnDetected = false;
					}
					if(!up){
						normalChargeVoltage = normalChargeVoltage - 0.1;
						TurnDetected = false;
					}
				}
				if(button){
					adjValue = false;
					lcd.setCursor(18,2);
					lcd.print("  ");
					button = false;
					break;
				}
			}
		}
		break;
		case 1:
		arrowpos3();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		EEPROM.put(nrChrAddr,normalChargeVoltage);
		EEPROM.commit();
		lcd.clear();
		BMSParameterSetting();
		}
		break;
	}
	}
}

void firmwareSetting(){
	arrowpos = 2;
  while(subSetting == true){
  lcd.home();
  lcd.print("------Setting-------");
    lcd.setCursor(0,1);
  lcd.print("Firmware & ROMConfig");
    lcd.setCursor(2,2);
  lcd.print("Back");
    lcd.setCursor(2,3);
  lcd.print("Clock");
    lcd.setCursor(11,2);
  lcd.print("FactReset");
    lcd.setCursor(11,3);
  lcd.print("Update");
  secondLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
    if(arrowpos >= 6){
      arrowpos = 2;
    }
    if(arrowpos < 2){
      arrowpos = 5;
    }
    switch(arrowpos){
      case 2:
      arrowpos2();
      if (button){
    subSetting = false;
    button = false;
    lcd.clear();
    settingScreen();
    }
      break;
      case 3:///---------------Clock----------------
      arrowpos3();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	timeSetting();
    }
      break;
      case 4:///---------------Factory Reset-----------
      arrowpos5();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	factoryReset();
    }
      break;
      case 5:///----------Update---------------
      arrowpos6();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
    firmwareUpdate();
    }
      break;
    }
  }
}

void timeSetting(){
	arrowpos = 0;
	lcd.home();
  lcd.print("====Time Setting====");
    lcd.setCursor(2,1);
  lcd.print("Save");
    lcd.setCursor(2,2);
  lcd.print("H");
    lcd.setCursor(2,3);
  lcd.print("M");
    lcd.setCursor(11,1);
  lcd.print("Day");
    lcd.setCursor(11,2);
  lcd.print("Month");
    lcd.setCursor(11,3);
  lcd.print("Y");
  defaultLine();
  DateTime now = rtc.now();
	setMonth = now.month();
	setDay = now.day();
	setHour = now.hour();
	setMin = now.minute();
  while(specificSetting == true){
	if(TurnDetected){
    if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
		}
    if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
		}
    }
	if(arrowpos >= 6){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 5;
    }
	switch (arrowpos){
    case 0:
    arrowpos1();
    if (button){
    mainScreenExe = true;
	specificSetting = false;
    button = false;
    lcd.clear();
	rtc.adjust(DateTime(setYear, setMonth, setDay, setHour, setMin, 00));
    mainScreen();
    }
    break;
    case 1: //Hour
    arrowpos2();
    if (button){
    button = false;
	adjValue = true;
	while(adjValue == true){
		lcd.setCursor(8,2);
		lcd.print("<");
		if(setHour < 10){
		lcd.setCursor(4,2);
		lcd.print("0");
		lcd.print(setHour);
		} else {
			lcd.setCursor(4,2);
			lcd.print(setHour);
		}
		if(TurnDetected){
		if(up){
			setHour = setHour + 1;
			TurnDetected = false;
			}
		if(!up){
			setHour = setHour - 1;
			TurnDetected = false;
		}
		}
				if(button){
					adjValue = false;
					lcd.setCursor(8,2);
					lcd.print(" ");
					button = false;
					break;
				}
	}
    }
    break;
    case 2: //Minute
    arrowpos3();
    if (button){
    button = false;
	adjValue = true;
	while(adjValue == true){
		lcd.setCursor(8,3);
		lcd.print("<");
	if(setMin < 10){
		lcd.setCursor(4,3);
		lcd.print("0");
		lcd.print(setMin);
		} else {
			lcd.setCursor(4,3);
			lcd.print(setMin);
		}
	if(TurnDetected){
    if(up){
        setMin = setMin + 1;
        TurnDetected = false;
		}
    if(!up){
        setMin = setMin - 1;
        TurnDetected = false;
		}
    }
				if(button){
					adjValue = false;
					lcd.setCursor(8,3);
					lcd.print(" ");
					button = false;
					break;
				}
	}
    }
    break;
    case 3: //Day
    arrowpos4();
    if (button){
    button = false;
	adjValue = true;
	while(adjValue == true){
	lcd.setCursor(19,1);
	lcd.print("<");
	if(setDay < 10){
	lcd.setCursor(15,1);
	lcd.print("0");
	lcd.print(setDay);
	} else {
		lcd.setCursor(15,1);
		lcd.print(setDay);
	}
	if(TurnDetected){
    if(up){
        setDay = setDay + 1;
        TurnDetected = false;
		}
    if(!up){
        setDay = setDay - 1;
        TurnDetected = false;
		}
    }
				if(button){
					adjValue = false;
					lcd.setCursor(19,1);
					lcd.print(" ");
					button = false;
					break;
				}
	}
    }
    break;
    case 4: //Month
    arrowpos5();
    if (button){
    button = false;
	adjValue = true;
	while(adjValue == true){
	lcd.setCursor(19,2);
	lcd.print("<");
	if(setMonth < 10){
	lcd.setCursor(17,2);
	lcd.print("0");
	lcd.print(setMonth);
	} else {
		lcd.setCursor(17,2);
		lcd.print(setMonth);
	}
	if(TurnDetected){
    if(up){
        setMonth = setMonth + 1;
        TurnDetected = false;
		}
    if(!up){
		setMonth = setMonth - 1;
        TurnDetected = false;
		}
    }
				if(button){
					adjValue = false;
					lcd.setCursor(19,2);
					lcd.print(" ");
					button = false;
					break;
				}
	}
    }
    break;
    case 5: //Year
    arrowpos6();
    if (button){
    button = false;
	adjValue = true;
	while(adjValue == true){
	lcd.setCursor(19,3);
	lcd.print("<");
	if(setYear < 10){
	lcd.setCursor(13,3);
	lcd.print("200");
	lcd.print(setYear);
	lcd.print(" ");
	} else {
	lcd.setCursor(13,3);
	lcd.print("20");
	lcd.print(setYear);
	lcd.print(" ");
	}
	if(TurnDetected){
    if(up){
        setYear = setYear + 1;
        TurnDetected = false;
		}
    if(!up){
        setYear = setYear - 1;
        TurnDetected = false;
		}
    }
				if(button){
					adjValue = false;
					lcd.setCursor(19,3);
					lcd.print(" ");
					button = false;
					break;
				}
	}
    }
    break;
    }
  }
}

void factoryReset(){
	arrowpos = 1;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("Factory Reset EEPROM");
	lcd.setCursor(0,2);
  lcd.print("    Are you Sure?   ");
	lcd.setCursor(2,3);
  lcd.print("Yes");
    lcd.setCursor(11,3);
  lcd.print("No");
  thirdLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos3();
		if (button){
		lcd.setCursor(0,2);
        lcd.print("   EEPROM Wiping    ");
		for (int z = 0 ; z < 512 ; z++) {
		EEPROM.write(z, 0);
		EEPROM.commit();
		}
		finger.emptyDatabase();
		delay(500);
		lcd.setCursor(0,2);
        lcd.print("  Memory Clear Done ");
		delay(1000);
		lcd.setCursor(0,2);
        lcd.print("  System Restart..  ");
		delay(1000);
		ESP.restart();
		}
		break;
		case 1:
		arrowpos6();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		lcd.clear();
		firmwareSetting();
		}
		break;
	}
  }
}

void firmwareUpdate(){
  lcd.setCursor(0,0);
    lcd.print("  Firmware Updater  ");
  lcd.setCursor(0,1);
    lcd.print("------[Status]------");
  lcd.setCursor(0,2);
    lcd.print(" Soft AP Setup");
  lcd.setCursor(0,3);
    lcd.print(" MDNS Initialized");
    WiFi.softAP(ssid, password);
    delay(1500);
  if (!MDNS.begin(host)) {//http://esp32.local
    lcd.setCursor(0,3);
    lcd.print(" MDNS Setup Failed");
  } else {
    lcd.setCursor(0,2);
    lcd.print(" Server Online");
  lcd.setCursor(0,3);
    lcd.print(" Connection Ready");
  }
    server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      lcd.setCursor(0,2);
    lcd.print(" Uploading....");
  lcd.setCursor(0,3);
    lcd.print(" Do not turn off");
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {

      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        lcd.setCursor(0,2);
    lcd.print(" Update Success");
  lcd.setCursor(0,3);
    lcd.print(" System Reboot...");
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();
  for(;;){
    server.handleClient();
	delay(1);
  }
}

void bootup(){
  //System Booting/////////////////////
  lcd.home();
  lcd.print(">System Booting....<");
  //Voltage Check///////////////////////////
  lcd.setCursor(0,1);
  lcd.print("BIOS Checking       ");
  delay(200);
  lcd.setCursor(0,2);
  lcd.print("         OK         ");
  delay(100);
  lcd.setCursor(0,2);
  lcd.print("                    ");
  //Throttle Calibration Load////////////////////////
  lcd.setCursor(0,1);
  lcd.print("Throttle Setting    ");
  delay(150);
  lcd.setCursor(0,2);
  lcd.print("        LOAD        ");
  delay(150);
  lcd.setCursor(0,2);
  lcd.print("                    ");
  //Interface Check
  lcd.setCursor(0,1);
  lcd.print("Interface Checking  ");
  if (finger.verifyPassword()) {
  delay(160);
  lcd.setCursor(0,2);
  lcd.print("  Finger Bus Begin  ");
  }else{
    delay(160);
  lcd.setCursor(0,2);
  lcd.print("        ERR         ");
  lcd.home();
  lcd.print(">System Restart....<");
  delay(500);
  ESP.restart();
  }
  delay(300);
  lcd.setCursor(0,2);
  lcd.print("                    ");
  //Sensor Check
  lcd.setCursor(0,1);
  lcd.print("Variable Set to RAM ");
  delay(250);
  lcd.setCursor(0,2);
  lcd.print("         OK         ");
  delay(200);
  lcd.setCursor(0,2);
  lcd.print("                    ");
  //EEPROM Loaded
  lcd.setCursor(0,1);
  lcd.print("EPROM Setting Loaded");
  EEPROM.get(resetWatcherAddr,EEPROMResetState);
  if(EEPROMResetState == 0){
	EEPROM.put(odoAddr,odometer);
	EEPROM.put(pwLimitAddr,powerLimit);
	EEPROM.put(wheelAddr,wheelradius);
	EEPROM.put(lvCAddr,lowvoltCutoff_threshold);
	EEPROM.put(lvRAddr,lowvoltRecovery_threshold);
	EEPROM.put(nrChrAddr,normalChargeVoltage);
	EEPROM.put(vAdjAddr,voltAdj);
	EEPROM.put(magnetCountAddr,magnetCount);
	EEPROM.put(thrCaliMxAddr,throttleCalibrationMax);
	EEPROM.put(thrCaliMnAddr,throttleCalibrationMin);
	EEPROM.put(ohTempAddr,overheatProtect);
	EEPROM.put(crtcTempAddr,criticalProtect);
	EEPROM.put(regBrkAddr,regenBrake);
	EEPROM.put(resistAddr,resistance);
	EEPROMResetState = true;
	EEPROM.put(resetWatcherAddr,EEPROMResetState);
	EEPROM.commit();
	EEPROM.end();
  } else {
	EEPROM.get(odoAddr,odometer);
	EEPROM.get(pwLimitAddr,powerLimit);
	EEPROM.get(wheelAddr,wheelradius);
	EEPROM.get(lvCAddr,lowvoltCutoff_threshold);
	EEPROM.get(lvRAddr,lowvoltRecovery_threshold);
	EEPROM.get(nrChrAddr,normalChargeVoltage);
	EEPROM.get(vAdjAddr,voltAdj);
	EEPROM.get(magnetCountAddr,magnetCount);
	EEPROM.get(thrCaliMxAddr,throttleCalibrationMax);
	EEPROM.get(thrCaliMnAddr,throttleCalibrationMin);
	EEPROM.get(ohTempAddr,overheatProtect);
	EEPROM.get(crtcTempAddr,criticalProtect);
	EEPROM.get(regBrkAddr,regenBrake);
	EEPROM.get(resistAddr,resistance);
		EEPROM.get(Name2Addr,FingerID.Name2);
		EEPROM.get(Name3Addr,FingerID.Name3);
		EEPROM.get(Name4Addr,FingerID.Name4);
		EEPROM.get(Name5Addr,FingerID.Name5);
		EEPROM.get(Name6Addr,FingerID.Name6);
		EEPROM.get(Name7Addr,FingerID.Name7);
		EEPROM.get(Name8Addr,FingerID.Name8);
		EEPROM.get(Name9Addr,FingerID.Name9);
  }
  delay(400);
  lcd.setCursor(0,2);
  lcd.print("       Done         ");
  delay(60);
  lcd.setCursor(0,0);
  lcd.print("---System Started---");
  delay(1000);
  lcd.clear();
  finger.getTemplateCount();
if (finger.templateCount == 0) {
    adminFingerprintRegistration();
  } else {
    lockScreen();
  }
}

void mainScreen(){
  while (mainScreenExe == true) {
	universalBlinkerState();
	countingTime = pulseIn(hallsensor,HIGH,1500000);
	speedCalc();
	odoCounting();
	motorTemp = thermistor.read();
	motorDriveOperation();
	motorTempProtect();
	batterySoC();
	lcd.setCursor(0,0);////////////// 1 /////////////
    lcd.print("T");
    motorTemp = min(motorTemp,150);
    motorTemp = max(motorTemp,-40);
    lcd.print(motorTemp);
    lcd.print("c");
    if (motorTemp <= 99){
      lcd.setCursor(4,0);
      lcd.print(" ");
    }
    ////////////////////////////////// 2 //////////////
	battStateReport();
    lcd.setCursor(8,1);
	lcd.print(battVolt, 1);
	if(battVolt < 10.0){
	lcd.setCursor(11,1);
	lcd.print(" "); 
	}
	lcd.setCursor(12,1);
	lcd.print("V");
	currentMonitor();
    lcd.setCursor(0,2);/////////////// 3 ///////////////
    lcd.print("PWM ");
    lcd.print(dutyCycle);
    if(dutyCycle <= 9){
      lcd.setCursor(5,2);
      lcd.print(" ");
    }
	lcd.setCursor(6,2);
    lcd.print("%");
    lcd.setCursor(8,2);
    lcd.print("SPD [");
	if(currentSpeed < 10){
		lcd.print("0");
		lcd.print(currentSpeed);
	} else {
    lcd.print(currentSpeed);
	}
    lcd.setCursor(15,2);
    lcd.print("]km/h");
    lcd.setCursor(0,3);//////////////// 4 ///////////////
    EEPROM.put(odoAddr,odometer);
	EEPROM.commit();
    if (TurnDetected){
      if(up && tripView == false){
        tripView = true;
        TurnDetected = false;
		lcd.setCursor(0,3);
        lcd.print("        ");
      } else if(!up && tripView == false){
        tripView = true;
        TurnDetected = false;
		lcd.setCursor(0,3);
        lcd.print("        ");
      } else {
        tripView = false;
        TurnDetected = false;
		lcd.setCursor(0,3);
        lcd.print("        ");
      }
    }
    if (tripView == false){
		lcd.setCursor(0,3);
      lcd.print("Odo");
      lcd.setCursor(4,3);
      lcd.print(odometer);
	  motorCurrent();
	  lcd.setCursor(5,0);
      lcd.print(" ");
    } else {
		lcd.setCursor(0,3);
      lcd.print("Trp");
      lcd.setCursor(4,3);
      lcd.print(tripmeter, 1);
	  timeDisplay();
    }
    lcd.setCursor(8,3);
    lcd.print("km");
    ecoMode = digitalRead(modeselect);
    if(ecoMode == true){
	lcd.setCursor(10,3);
    lcd.print(" [EcoMode]");
    } else {
	lcd.setCursor(10,3);
    lcd.print("[MaxPower]");
    }
    if(button && accessLevelControl() == 1 && accessLevelControl() == 2){
      button = false;
      mainScreenExe = false;
      lcd.clear();
      settingScreen();
    }
  }
}

void settingScreen(){
	arrowpos = 0;
  lcd.home();
  lcd.print("======Setting=======");
    lcd.setCursor(2,1);
  lcd.print("Back");
    lcd.setCursor(2,2);
  lcd.print("Sensor");
    lcd.setCursor(2,3);
  lcd.print("Motor");
    lcd.setCursor(11,1);
  lcd.print("Odometer");
    lcd.setCursor(11,2);
  lcd.print("BMSParamt");
    lcd.setCursor(11,3);
  lcd.print("Firmware");
  defaultLine();
  while(mainScreenExe == false){
	if(BH){
		BH = false;
		button = false;
		advancedSetting = true;
		lcd.clear();
		commLinkSetting();
	}
    if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
  switch (arrowpos){
    case 0:  //Back to Main Screen////////////////////
    arrowpos1();
    if (button){
    mainScreenExe = true;
    button = false;
    lcd.clear();
	EEPROM.end();
    mainScreen();
    }
    break;
    case 1:  //Sensor Setting////////////////
    arrowpos2();
    if (button){
      lcd.clear();
      button = false;
      subSetting = true;
      sensorSetting();
    }
    break;
    case 2:  //Motor Drive Setting////////////////////
    arrowpos3();
    if (button){
      lcd.clear();
      button = false;
      subSetting = true;
      motorDriveParameter();
    }
    break;
    case 3:  //Speedometer Parameter/////////////////
    arrowpos4();
    if (button){
      lcd.clear();
      button = false;
      subSetting = true;
      odoSpdSetting();
    }
    break;
    case 4:  //BMS Parameter/////////////////////////
    arrowpos5();
    if (button && accessLevelControl() == 1 || adminVerifySecurity){
      lcd.clear();
      button = false;
      subSetting = true;
      BMSParameterSetting();
    } else {
		adminFingerVerify();
		if(adminVerifySecurity){
			if(accessLevelControl() == 3){
			adminVerifySecurity = false;
			}
			BMSParameterSetting();
		} else {
			settingScreen();
		}
	}
    break;
    case 5:  //Firmware Update//////////////////////
    arrowpos6();
    if (button && accessLevelControl() == 1 || adminVerifySecurity){
    lcd.clear();
    button = false;
    subSetting = true;
    firmwareSetting();
    } else {
		adminFingerVerify();
		if(adminVerifySecurity){
			if(accessLevelControl() == 3){
			adminVerifySecurity = false;
			}
			firmwareSetting();
		} else {
			settingScreen();
		}
	}
    break;
    }
    if(arrowpos >= 6){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 5;
    }
  }
  lcd.clear();
  mainScreenExe = true;
  mainScreen();
}

void commLinkSetting(){
	arrowpos2();
	while(advancedSetting == true){
  lcd.home();
  lcd.print("--Advanced Setting--");
    lcd.setCursor(0,1);
  lcd.print("InterComm Bus Config");
    lcd.setCursor(2,2);
  lcd.print("Back");
    lcd.setCursor(2,3);
  lcd.print("Auth");
    lcd.setCursor(11,2);
  lcd.print("AntiThief");
    lcd.setCursor(11,3);
  lcd.print("SentrySys");
  secondLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
    if(arrowpos >= 6){
      arrowpos = 2;
    }
    if(arrowpos < 2){
      arrowpos = 5;
    }
    switch(arrowpos){
      case 2:
      arrowpos2();
      if (button){
    advancedSetting = false;
    button = false;
    lcd.clear();
    settingScreen();
    }
      break;
      case 3:///---------------Authentication Control System----------------
      arrowpos3();
      if (button){
    button = false;
	subSetting = true;
    lcd.clear();
	fingerprintManagement();
    }
      break;
      case 4:///---------------GPS and Alarm System-----------
      arrowpos5();
      if (button){
    button = false;
	subSetting = true;
    lcd.clear();
	securityComm();
    }
      break;
      case 5:///----------Camera & Storage Setting---------------
      arrowpos6();
      if (button){
    button = false;
	subSetting = true;
    lcd.clear();
    sentrySystem();
    }
      break;
    }
  }
}

void securityComm(){
	arrowpos = 2;
  while(subSetting == true){
  lcd.home();
  lcd.print("Serial Comm Setting ");
    lcd.setCursor(0,1);
  lcd.print("Alarm & Tracking Sys");
    lcd.setCursor(2,2);
  lcd.print("Back");
    lcd.setCursor(2,3);
  lcd.print("State");
    lcd.setCursor(11,2);
  lcd.print("Parameter");
    lcd.setCursor(11,3);
  lcd.print("Command");
  secondLine();
  if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
    if(arrowpos >= 6){
      arrowpos = 2;
    }
    if(arrowpos < 2){
      arrowpos = 5;
    }
    switch(arrowpos){
      case 2:
      arrowpos2();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
    commLinkSetting();
    }
      break;
      case 3://---------------ON OFF Function-------------
      arrowpos3();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	securityState();
    }
      break;
      case 4://---------------Parameter---------------
      arrowpos5();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	securityParameter();
    }
      break;
      case 5://------------------Send Command----------------
      arrowpos6();
      if (button){
    subSetting = false;
    specificSetting = true;
    button = false;
    lcd.clear();
	commandPanel();
    }
      break;
    }
  }
}

void securityState(){
	arrowpos = 0;
	while(specificSetting == true){
	lcd.home();
  lcd.print("------Setting-------");
	lcd.setCursor(0,1);
  lcd.print("Alarm Security State");
	lcd.setCursor(0,2);
  lcd.print(" Status = ");
	lcd.setCursor(2,3);
  lcd.print("Enable");
    lcd.setCursor(11,3);
  lcd.print("Disable");
  thirdLine();
	
/*	if(regenBrake == true){
	  lcd.setCursor(10,2);
	  lcd.print("Enable ");
	} else {
	  lcd.setCursor(10,2);
	  lcd.print("Disable");
	}*/
	if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos3();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		lcd.clear();
//		EEPROM.put(regBrkAddr,regenBrake);
//		EEPROM.commit();
//		motorDriveParameter();
		}
		break;
		case 1:
		arrowpos6();
		if (button){
		specificSetting = false;
		subSetting = true;
		button = false;
		lcd.clear();
//		EEPROM.put(regBrkAddr,regenBrake);
//		EEPROM.commit();
//		motorDriveParameter();
		}
		break;
	}
	}
}

void securityParameter(){
	
}

void commandPanel(){
	arrowpos = 0;
	commandSend = 0;
	inputCharacterSelector = 0;
	bool inputSomething;
	while(specificSetting == true){
	universalBlinkerState();
	thirdLine();
	lcd.home();
	lcd.print("CtrSec Command Panel");
	lcd.setCursor(0,1);
	lcd.print("Respond = ");
	lcd.print(queryResp);
	lcd.setCursor(0,2);
	lcd.print("INPUT = ");
	lcd.setCursor(2,3);
	lcd.print("Send/En");
	lcd.setCursor(11,3);
	lcd.print("Clear/Ext");
	if(commandSend == 0){
		inputSomething = false;
	} else {
		inputSomething = true;
	}
	if(TurnDetected){
      if(up){
        arrowpos = arrowpos + 1;
        TurnDetected = false;
      }
      if(!up){
        arrowpos = arrowpos - 1;
        TurnDetected = false;
      }
    }
	if(arrowpos >= 2){
      arrowpos = 0;
    }
    if(arrowpos < 0){
      arrowpos = 1;
    }
	switch(arrowpos){
		case 0:
		arrowpos3();
		if (button && !inputSomething){
		button = false;
		commandInput = true;
		lcd.setCursor(0,3);
		lcd.print("  ");
		} else if(button && inputSomething){
			delay(100);
			serialCommSending();
		}
		break;
		case 1:
		arrowpos6();
		if (button && !inputSomething){
		button = false;
		specificSetting = false;
		securityComm();
		} else if(button && inputSomething){
			delay(100);
			commandSend = 0;
		}
		break;
	}
	while(commandInput){
		if(BH){
			BH = false;
			button = false;
			commandInput = false;
			lcd.setCursor(8,2);
			lcd.print(" ");
			lcd.setCursor(19,2);
			lcd.print(" ");
		} else if(button && digitalRead(sw) == HIGH) {
			if(commandSend == 0){
				commandSend = Character[inputCharacterSelector];
				button = false;
			} else {
				commandSend = commandSend + Character[inputCharacterSelector];
				button = false;
			}
		}
	if(ubstate == true){
	lcd.setCursor(8,2);
	lcd.print(">");
	lcd.setCursor(19,2);
	lcd.print("<");
	} else {
	lcd.setCursor(8,2);
	lcd.print(" ");
	lcd.setCursor(19,2);
	lcd.print(" ");
	}
	lcd.setCursor(9,2);
	lcd.print(commandSend);
	lcd.setCursor(2,3);
	lcd.print("Char> ");
	lcd.print(Character[inputCharacterSelector]);
	inputCharCheck();
	lcd.setCursor(11,3);
	lcd.print("Hold2Exit");
	if(TurnDetected){
      if(up){
        inputCharacterSelector = inputCharacterSelector + 1;
        TurnDetected = false;
      }
      if(!up){
        inputCharacterSelector = inputCharacterSelector - 1;
        TurnDetected = false;
      }
    }
	}
}
}

void sentrySystem(){
	lcd.clear();
	lcd.home();
	lcd.print(" Module Not Install ");
	delay(2000);
	lcd.clear();
	securityComm();
}

void core2Tasks(){
for(;;){
	
}
}

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.createChar(0,fingerFragment1);
  lcd.createChar(1,fingerFragment2);
  lcd.createChar(2,fingerFragment3);
  lcd.createChar(3,fingerFragment4);
  lcd.createChar(4,batteryBase);
  lcd.createChar(5,batteryFilled);
  lcd.createChar(6,batteryDeplete);
  lcd.createChar(7,batteryTop);
  finger.begin(57600);
  Serial.begin(115200);
  Wire.begin();
  rtc.begin();
  ACS.autoMidPoint();
  pinMode(sw,INPUT_PULLUP);
  pinMode(clk,INPUT);
  pinMode(dt,INPUT);
  pinMode(hallsensor,INPUT);
  pinMode(modeselect,INPUT_PULLUP);
  pinMode(buzzer,OUTPUT);
  pinMode(bbconverter,OUTPUT);
  ledcSetup(0,switchingFreq,8);
  ledcAttachPin(4,0);
  attachInterrupt(dt,isr0,RISING);
  attachInterrupt(sw,isr1,FALLING);
  EEPROM.begin(512);
  bootup();
}

void loop(){
  
}
//=========================Setup End Line=======================
