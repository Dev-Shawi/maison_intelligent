
#include <LCD_I2C.h>
#include <AccelStepper.h>
#include <HCSR04.h>

#pragma region Configuration

LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(6, 7);
AccelStepper stepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11);

const float STEPS_PER_DEG = 2048.0 * (64.0 / 360.0);
const int MIN_DEG = 10;
const int MAX_DEG = 170;
const unsigned long MOVE_DURATION = 2000;

// New pin definitions for Lab 05
const int redPin = 2;
const int bluePin = 3;
const int buzzerPin = 4;

enum AppState { INIT,FERME,OUVERT,OUVERTURE,FERMETURE };

AppState appState = INIT;
unsigned long currentTime = 0;

#pragma endregion

#pragma region Tâches

float mesureDistanceTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  const unsigned long rate = 50;
  static float lastDistance = 0.0;

  if (ct - lastTime < rate) return lastDistance;

  lastTime = ct;
  lastDistance = hc.dist();
  return lastDistance;
}

void lcdTask(unsigned long ct, float distance, int degres) {
  static unsigned long lastTime = 0;
  const unsigned long rate = 100;

  if (ct - lastTime < rate) return;

  lastTime = ct;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist : ");
  lcd.print(distance);
  lcd.print(" cm");

  lcd.setCursor(0, 1);
  if (appState == FERME) lcd.print("Port: Fermer");
  else if (appState == OUVERT) lcd.print("Port: Ouverte");
  else {
    lcd.print("Porte: ");
    lcd.print(degres);
    lcd.print(" deg");
  }
}

void serialTask(unsigned long ct, float distance, int degres) {
  static unsigned long lastTime = 0;
  const unsigned long rate = 100;

  if (ct - lastTime < rate) return;

  lastTime = ct;

  Serial.print("etd:2305204,dist:");
  Serial.print(distance);
  Serial.print(",deg:");
  Serial.println(degres);
}

// New task for handling the alarm system
void alarmTask(unsigned long ct, float distance) {
  static bool alarmActive = false;
  static unsigned long lastDetectionTime = 0;
  static unsigned long lastGyroChange = 0;
  static bool gyroRed = false;

  // Update last detection time if object is within 15 cm
  if (distance <= 15.0) {
    lastDetectionTime = ct;
  }

  // Check if alarm should be active (within 3 seconds of last detection)
  bool shouldAlarmBeActive = (ct - lastDetectionTime) < 3000;

  if (shouldAlarmBeActive) {
    if (!alarmActive) {
      // Activate alarm components
      alarmActive = true;
      tone(buzzerPin, 1000); // Start buzzer tone at 1000 Hz
      lastGyroChange = ct; // Initialize gyro timing
      gyroRed = false;
      digitalWrite(bluePin, HIGH);
      digitalWrite(redPin, LOW);
    }

    // Alternate between red and blue every 500 ms
    if (ct - lastGyroChange >= 500) {
      lastGyroChange = ct;
      gyroRed = !gyroRed;
      digitalWrite(redPin, gyroRed ? HIGH : LOW);
      digitalWrite(bluePin, !gyroRed ? HIGH : LOW);
    }
  } else {
    if (alarmActive) {
      // Deactivate alarm components
      alarmActive = false;
      noTone(buzzerPin); // Stop buzzer
      digitalWrite(redPin, LOW);
      digitalWrite(bluePin, LOW);
    }
  }
}

#pragma endregion

#pragma region États

long degToSteps(int deg) {
  return deg * STEPS_PER_DEG;
}

void initState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long startTime = 0;

  if (firstTime) {
    lcd.begin();
    lcd.backlight();
    lcd.print("ETD:2305204");
    lcd.setCursor(0, 1);
    lcd.print("Labo 4A");  //


    stepper.setMaxSpeed((MAX_DEG - MIN_DEG) * STEPS_PER_DEG / (MOVE_DURATION / 1000.0));
    stepper.setAcceleration(1000);
    startTime = ct;
    firstTime = false;
    return;
  }

  if (ct - startTime >= 2000) {
    lcd.clear();
    stepper.moveTo(degToSteps(MIN_DEG));
    stepper.enableOutputs();
    while (stepper.run())
      ;
    //stepper.disableOutputs();
    appState = FERME;
    firstTime = true;
  }
}

void fermeState(unsigned long ct) {
  static bool firstTime = true;

  if (firstTime) {
    stepper.disableOutputs();
    firstTime = false;
    return;
  }

  float distance = mesureDistanceTask(ct);

  if (distance < 30) {
    appState = OUVERTURE;
    firstTime = true;
  }
}

void ouvertState(unsigned long ct) {
  static bool firstTime = true;

  if (firstTime) {
    stepper.disableOutputs();
    firstTime = false;
    return;
  }

  float distance = mesureDistanceTask(ct);

  if (distance > 60) {
    appState = FERMETURE;
    firstTime = true;
  }
}

void ouvertureState(unsigned long ct) {
  static bool firstTime = true;

  if (firstTime) {
    stepper.enableOutputs();
    stepper.moveTo(degToSteps(MAX_DEG));
    firstTime = false;
    return;
  }

  stepper.run();

  if (stepper.distanceToGo() == 0) {
    appState = OUVERT;
    firstTime = true;
  }
}

void fermetureState(unsigned long ct) {
  static bool firstTime = true;

  if (firstTime) {
    stepper.enableOutputs();
    stepper.moveTo(degToSteps(MIN_DEG));
    firstTime = false;
    return;
  }

  stepper.run();

  if (stepper.distanceToGo() == 0) {
    appState = FERME;
    firstTime = true;
  }
}

void stateManager(unsigned long ct) {
  switch (appState) {
    case INIT: initState(ct); break;
    case FERME: fermeState(ct); break;
    case OUVERT: ouvertState(ct); break;
    case OUVERTURE: ouvertureState(ct); break;
    case FERMETURE: fermetureState(ct); break;
  }
}
#pragma endregion

void setup() {
  Serial.begin(9600);

  // Initialize new pins for Lab 05
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(redPin, LOW);
  digitalWrite(bluePin, LOW);
  digitalWrite(buzzerPin, LOW);
}

Analyse bien le projet

Remarque : Dans ce projet, nous avons 4 systèmes :

Le système de lecture de la luminosité;
Le système de lecture de la distance;
Le systeme de moterStepper
Le système d'affichage.

void loop() {
  currentTime = millis();

  stateManager(currentTime);

  float distance = mesureDistanceTask(currentTime);
  int degres = stepper.currentPosition() / STEPS_PER_DEG;
  lcdTask(currentTime, distance, degres);
  serialTask(currentTime, distance, degres);
  alarmTask(currentTime, distance); // Execute alarm task
}
