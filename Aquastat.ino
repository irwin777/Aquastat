#include <DHT.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <Time.h>
#include <DS1307RTC.h>
#include <EEPROM.h>

#define DHTPIN A0
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define ONE_WIRE_BUS A1 // port 18b20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

LiquidCrystal_I2C lcd(0x27,16,2); 

byte gradus[8] = {
  0b00110,
  0b01001,
  0b01001,
  0b00110,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

byte heat[8] = {
  0b01000,
  0b01000,
  0b10000,
  0b01000,
  0b01000,
  0b10000,
  0b01000,
  0b01000
};

byte sun[8] = {
  0b00100,
  0b10101,
  0b01110,
  0b10001,
  0b01110,
  0b10101,
  0b00100,
  0b00000
};

#define R1pin A2
#define R2pin A3
#define buzzerPin 5
#define frequency 2400UL
#define duration 500UL
#define dThh 29.00F
#define dThl 22.00F
#define TempReadStep 10000UL

tmElements_t tm;
float h, t, dtC;
byte TEMPLIM, LigtONhour, LigtOFFhour, LigtONmin, LigtOFFmin, menu=0, menuLevel=3, bat[5] {
  8,9,10,11,12};
int t_out, humidity;
boolean oldkey[5]{
  0,0,0,0.0}
, key[5]{
  0,0,0,0,0};

void setup() {

  //RTC.read(tm);
  //setTime(tm.Hour,tm.Minute,tm.Wday,tm.Day,tm.Month,tmYearToCalendar(tm.Year));

  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.createChar(1, gradus);
  lcd.createChar(2, heat);
  lcd.createChar(3, sun);

  pinMode(13, OUTPUT);
  pinMode(R1pin, OUTPUT);
  pinMode(R2pin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  for (byte i=0; i<5; i++) pinMode(bat[i], INPUT_PULLUP);

  digitalWrite(R1pin, HIGH);
  digitalWrite(R2pin, HIGH);
  digitalWrite(13, LOW);

  dht.begin();
  sensors.begin();
  sensors.getAddress(insideThermometer, 0);
  sensors.setResolution(insideThermometer, 12);
  checkSensor1820();
  checkSensorDHT();

  if (EEPROM.read(0)!=1) firstMenu();
  TEMPLIM = EEPROM.read(1);
  //LigtONhour = EEPROM.read(2);
  //LigtOFFhour = EEPROM.read(3);
  //LigtONmin = EEPROM.read(4);
  //LigtOFFmin = EEPROM.read(5);

  LigtONhour = 9;
  LigtOFFhour = 19;
  LigtONmin = 0;
  LigtOFFmin = 30;

  digitalWrite(13, HIGH);
  pic();
  digitalWrite(13, LOW);
}

void loop() {
  RTC.read(tm);
  tempRead();
  heater();
  checkLight();
  key[0] = checkBatton(bat[0]);
  if (key[0]!=oldkey[0]) {
    if (!oldkey[0]){
      menu++;
      if (menu > menuLevel) menu = 0;
      lcd.clear();
    }
    oldkey[0] = key[0];
  }
  key[4] = checkBatton(bat[4]);
  if (key[4]!=oldkey[4]) {
    if (!oldkey[4]){
      digitalWrite(R2pin,!digitalRead(R2pin));
    }
    oldkey[4] = key[4];
  }
  menuOUT();
}

void tempRead() {
  static unsigned long prMillis = 0;
  if (millis()-prMillis >= TempReadStep) {
    digitalWrite(13, HIGH);
    checkSensorDHT();
    checkSensor1820();
    t_out = int(t);
    humidity = int(h);
    if (dtC >= dThh || dtC <= dThl) {
      pic();
    }
    prMillis=millis();
    digitalWrite(13, LOW);
  }
}

void print2digit(int val) {
  if (val < 10) lcd.print("0");
  lcd.print(val);
}

void menuOUT() {
  switch (menu) {
  case 0:
    lcd.home();
    lcd.print(t_out);
    lcd.write(1);
    lcd.print("C");
    lcd.print(" ");
    lcd.print(humidity);
    lcd.print("%");
    lcd.setCursor(11,0);
    print2digit(tm.Hour);
    lcd.print(":");
    print2digit(tm.Minute);
    lcd.setCursor(0,1);
    lcd.print(dtC);
    lcd.write(1);
    lcd.print("C");
    lcd.setCursor(14,1);
    if (digitalRead(R1pin)) lcd.print(" ");
    else lcd.write(2);
    if (digitalRead(R2pin)) lcd.print(" ");
    else lcd.write(3);
    break;
  case 1:
    lcd.home();
    print2digit(tm.Hour);
    lcd.print(":");
    print2digit(tm.Minute);
    lcd.print(":");
    print2digit(tm.Second);
    lcd.setCursor(0,1);
    print2digit(tm.Day);
    lcd.print("/");
    print2digit(tm.Month);
    lcd.print("/");
    lcd.print(tmYearToCalendar(tm.Year));
    break;
  case 2:
    lcd.home();
    lcd.print("Set temperature ");
    lcd.setCursor(0,1);
    print2digit(TEMPLIM);
    key[1] = checkBatton(bat[1]);
    if (key[1]!=oldkey[1]) {
      if (!oldkey[1]){
        TEMPLIM++;
        if (TEMPLIM>34) TEMPLIM=34;
      }
      oldkey[1] = key[1];
    }
    key[2] = checkBatton(bat[2]);
    if (key[2]!=oldkey[2]) {
      if (!oldkey[2]){
        TEMPLIM--;
        if (TEMPLIM<18) TEMPLIM=18;
      }
      oldkey[2] = key[2];
    }
    key[3] = checkBatton(bat[3]);
    if (key[3]!=oldkey[3]) {
      if (!oldkey[3]){
        EEPROM.write(1,TEMPLIM);
        lcd.clear();
        lcd.print("SAVED");
        delay(1000);
        lcd.clear();
      }
      oldkey[3] = key[3];
    }
    break;
  case 3:
    lcd.home();
    lcd.print("Light ON ");
    print2digit(LigtONhour);
    lcd.print(":");
    print2digit(LigtONmin);
    lcd.setCursor(0,1);
    lcd.print("Light OFF ");
    print2digit(LigtOFFhour);
    lcd.print(":");
    print2digit(LigtOFFmin);
    break;
  }
}

void heater() {
  if (int(dtC) < TEMPLIM) if (digitalRead(R1pin)) digitalWrite(R1pin, LOW);
  else if (!digitalRead(R1pin)) digitalWrite(R1pin, HIGH);
}

void checkSensor1820() {
alert:
  sensors.requestTemperatures();
  dtC=sensors.getTempC(insideThermometer);
  if (dtC == DEVICE_DISCONNECTED) {
    lcd.clear();
    lcd.print("Failed read 1820");
    digitalWrite(R1pin, HIGH);
    pic();
    delay(5000);
    lcd.clear();
    goto alert;
  }
}

void checkSensorDHT() {
alert:
  h = dht.readHumidity();
  t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    lcd.clear();
    lcd.print("Failed read DHT ");
    pic();
    delay(5000);
    lcd.clear();
    goto alert;
  }
}

void pic() {
  tone(buzzerPin, frequency, duration);
  delay(500);
  noTone(buzzerPin);
}

boolean checkBatton(byte bat) {
  boolean state;
  if (digitalRead(bat)){
    state = 0;
  }
  else{
    state = 1;
  }
  return state;
} 

void firstMenu() {
  EEPROM.write(0,1);
}

void checkLight() {
  static unsigned long prMillis = 0;
  if (millis()-prMillis >= 30000) {
    if (byte(tm.Hour) == LigtONhour && byte(tm.Minute) == LigtONmin) if (digitalRead(R2pin)) digitalWrite(R2pin, LOW);
    if (byte(tm.Hour) == LigtOFFhour && byte(tm.Minute) == LigtOFFmin) if (!digitalRead(R2pin)) digitalWrite(R2pin, HIGH);
  }
  prMillis=millis();
}





