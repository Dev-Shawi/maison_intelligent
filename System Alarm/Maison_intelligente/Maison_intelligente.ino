#include <LCD_I2C.h>
#include <HCSR04.h>

// Configuration matérielle
#define TRIG_PIN 5
#define ECHO_PIN 6
#define BUZZER_PIN 3
#define LED_RED 8
#define LED_BLUE 7

// Constantes
const String STUDENT_ID = "etd:2305204";
const unsigned long 
  ALARM_TIMEOUT = 3000,
  BLINK_INTERVAL = 200;

const float ALARM_DISTANCE = 15.0;

// États
enum AlarmState {INIT, NORMAL, ALARM_ACTIVE, ALARM_COOLDOWN};

// Variables globales
LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(TRIG_PIN, ECHO_PIN);
AlarmState alarmState = INIT;
unsigned long lastDetectionTime = 0;
float currentDistance = 0.0;
bool ledToggle = false;

void setup() {
  Serial.begin(115200);
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  
  lcd.begin();
  lcd.backlight();
  showSplashScreen();
}

void loop() {
  unsigned long currentMillis = millis();
  
  updateDistance(currentMillis);
  updateAlarmState(currentMillis);
  handleDisplay(currentMillis);
  handleOutputs(currentMillis);
}

// Gestion de la distance
void updateDistance(unsigned long currentTime) {
  static unsigned long lastRead;
  if(currentTime - lastRead >= 50) {
    currentDistance = hc.dist();
    lastRead = currentTime;
  }
}

// Machine à états
void updateAlarmState(unsigned long currentTime) {
  switch(alarmState) {
    case INIT:
      if(currentTime > 2000) {
        alarmState = NORMAL;
        lcd.clear();
      }
      break;
      
    case NORMAL:
      if(currentDistance <= ALARM_DISTANCE) {
        alarmState = ALARM_ACTIVE;
        lastDetectionTime = currentTime;
      }
      break;
      
    case ALARM_ACTIVE:
      if(currentDistance > ALARM_DISTANCE) {
        if(currentTime - lastDetectionTime > ALARM_TIMEOUT) {
          alarmState = NORMAL;
          deactivateAlarm();
        }
      } else {
        lastDetectionTime = currentTime;
      }
      break;
  }
}

// Contrôle des sorties
void handleOutputs(unsigned long currentTime) {
  static unsigned long lastBlink;
  
  if(alarmState == ALARM_ACTIVE) {
    if(currentTime - lastBlink >= BLINK_INTERVAL) {
      ledToggle = !ledToggle;
      digitalWrite(LED_RED, ledToggle);
      digitalWrite(LED_BLUE, !ledToggle);
      lastBlink = currentTime;
    }
    tone(BUZZER_PIN, 1000); 
  } else {
    noTone(BUZZER_PIN);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, LOW);
  }
}

// Affichage LCD
void handleDisplay(unsigned long currentTime) {
  static unsigned long lastUpdate;
  
  if(currentTime - lastUpdate >= 100) {
    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(currentDistance, 1);
    lcd.print(" cm  ");
    
    lcd.setCursor(0, 1);
    lcd.print(alarmState == ALARM_ACTIVE ? "ALARME ACTIVE! " : "Statut: Normal ");
    
    lastUpdate = currentTime;
  }
}

void showSplashScreen() {
  lcd.print("Labo 5");
  lcd.setCursor(0, 1);
  lcd.print(STUDENT_ID);
}

void deactivateAlarm() {
  noTone(BUZZER_PIN);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);
}
