#include <LCD_I2C.h>
#include <AccelStepper.h>
#include <HCSR04.h>

#pragma region Configuration

// Broches
#define TRIG_PIN 5
#define ECHO_PIN 6
#define BUZZER_PIN 3
#define LED_RED 8
#define LED_BLUE 7
#define MOTOR_IN1 10
#define MOTOR_IN2 11
#define MOTOR_IN3 12
#define MOTOR_IN4 13

// Constantes
const String STUDENT_ID = "etd:2305204";
const float 
  DOOR_OPEN_DIST = 30.0,
  DOOR_CLOSE_DIST = 60.0,
  ALARM_DISTANCE = 15.0;

const unsigned long
  LCD_UPDATE_INTERVAL = 100,
  SERIAL_UPDATE_INTERVAL = 100,
  ALARM_TIMEOUT = 3000,
  BLINK_INTERVAL = 200;

// Moteur
const float STEPS_PER_DEG = 2048.0 * (64.0 / 360.0);
const int 
  MOTOR_MIN_DEG = 10,
  MOTOR_MAX_DEG = 170;

// États
enum SystemState {
  INIT,
  PORTE_FERMEE,
  PORTE_OUVERTE,
  OUVERTURE_EN_COURS,
  FERMETURE_EN_COURS,
  ALARME_ACTIVE
};

#pragma endregion

#pragma region Déclarations

// Composants
LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(TRIG_PIN, ECHO_PIN);
AccelStepper stepper(AccelStepper::FULL4WIRE, MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4);

// Variables globales
SystemState systemState = INIT;
unsigned long currentTime = 0;
float currentDistance = 0.0;
int currentDegrees = MOTOR_MIN_DEG;
bool ledToggle = false;
unsigned long lastDetectionTime = 0;

// Prototypes
void gererEtats(unsigned long ct);
void initialisation(unsigned long ct);
void porteFermee(unsigned long ct);
void porteOuverte(unsigned long ct);
void ouverturePorte(unsigned long ct);
void fermeturePorte(unsigned long ct);
void alarmeActive(unsigned long ct);
void miseAJourLCD(unsigned long ct);
void gestionAlarme(unsigned long ct);
void communicationSerie(unsigned long ct);

#pragma endregion

#pragma region Configuration initiale

void setup() {
  Serial.begin(9600);
  
  // Configuration des broches
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  
  // Initialisation LCD
  lcd.begin();
  lcd.backlight();
  lcd.print("Labo 4A/5");
  lcd.setCursor(0, 1);
  lcd.print(STUDENT_ID);

  // Configuration moteur
  stepper.setMaxSpeed(300);
  stepper.setAcceleration(500);
  stepper.setCurrentPosition(degreesToSteps(MOTOR_MIN_DEG));
}

#pragma endregion

#pragma region Boucle principale

void loop() {
  currentTime = millis();
  
  // Tâche de mesure
  static unsigned long lastMeasure;
  if(currentTime - lastMeasure >= 50) {
    currentDistance = hc.dist();
    lastMeasure = currentTime;
  }

  // Gestion des états
  gererEtats(currentTime);
  
  // Tâches périodiques
  miseAJourLCD(currentTime);
  gestionAlarme(currentTime);
  communicationSerie(currentTime);
}

#pragma endregion

#pragma region Gestion des états

void gererEtats(unsigned long ct) {
  switch(systemState) {
    case INIT: initialisation(ct); break;
    case PORTE_FERMEE: porteFermee(ct); break;
    case PORTE_OUVERTE: porteOuverte(ct); break;
    case OUVERTURE_EN_COURS: ouverturePorte(ct); break;
    case FERMETURE_EN_COURS: fermeturePorte(ct); break;
    case ALARME_ACTIVE: alarmeActive(ct); break;
  }
}

void initialisation(unsigned long ct) {
  static unsigned long startTime = ct;
  
  if(ct - startTime >= 2000) {
    lcd.clear();
    systemState = PORTE_FERMEE;
  }
}

void porteFermee(unsigned long ct) {
  if(currentDistance < DOOR_OPEN_DIST) {
    systemState = OUVERTURE_EN_COURS;
    stepper.enableOutputs();
    stepper.moveTo(degreesToSteps(MOTOR_MAX_DEG));
  }
  
  // Vérification alarme
  if(currentDistance <= ALARM_DISTANCE && systemState != ALARME_ACTIVE) {
    systemState = ALARME_ACTIVE;
    lastDetectionTime = ct;
  }
}

void porteOuverte(unsigned long ct) {
  if(currentDistance > DOOR_CLOSE_DIST) {
    systemState = FERMETURE_EN_COURS;
    stepper.enableOutputs();
    stepper.moveTo(degreesToSteps(MOTOR_MIN_DEG));
  }
  
  // Vérification alarme
  if(currentDistance <= ALARM_DISTANCE && systemState != ALARME_ACTIVE) {
    systemState = ALARME_ACTIVE;
    lastDetectionTime = ct;
  }
}

void ouverturePorte(unsigned long ct) {
  stepper.run();
  currentDegrees = stepsToDegrees(stepper.currentPosition());
  
  if(stepper.distanceToGo() == 0) {
    stepper.disableOutputs();
    systemState = PORTE_OUVERTE;
  }
}

void fermeturePorte(unsigned long ct) {
  stepper.run();
  currentDegrees = stepsToDegrees(stepper.currentPosition());
  
  if(stepper.distanceToGo() == 0) {
    stepper.disableOutputs();
    systemState = PORTE_FERMEE;
  }
}

void alarmeActive(unsigned long ct) {
  if(currentDistance > ALARM_DISTANCE) {
    if(ct - lastDetectionTime > ALARM_TIMEOUT) {
      systemState = (currentDegrees == MOTOR_MIN_DEG) ? PORTE_FERMEE : PORTE_OUVERTE;
      noTone(BUZZER_PIN);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_BLUE, LOW);
    }
  }
  else {
    lastDetectionTime = ct;
  }
}

#pragma endregion

#pragma region Tâches périodiques

void miseAJourLCD(unsigned long ct) {
  static unsigned long lastUpdate;
  if(ct - lastUpdate < LCD_UPDATE_INTERVAL) return;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(currentDistance, 1);
  lcd.print("cm");
  
  lcd.setCursor(0, 1);
  switch(systemState) {
    case ALARME_ACTIVE:
      lcd.print("ALARME! ");
      break;
    case PORTE_FERMEE:
      lcd.print("Porte fermee");
      break;
    case PORTE_OUVERTE:
      lcd.print("Porte ouverte");
      break;
    default:
      lcd.print("Deg: ");
      lcd.print(currentDegrees);
  }
  
  lastUpdate = ct;
}

void gestionAlarme(unsigned long ct) {
  static unsigned long lastBlink;
  
  if(systemState == ALARME_ACTIVE) {
    // Gyrophare
    if(ct - lastBlink >= BLINK_INTERVAL) {
      ledToggle = !ledToggle;
      digitalWrite(LED_RED, ledToggle);
      digitalWrite(LED_BLUE, !ledToggle);
      lastBlink = ct;
    }
    
    // Buzzer pulsé
    tone(BUZZER_PIN, 1000 + (ct % 1000), 100);
  }
}

void communicationSerie(unsigned long ct) {
  static unsigned long lastUpdate;
  if(ct - lastUpdate < SERIAL_UPDATE_INTERVAL) return;
  
  Serial.print("etd:");
  Serial.print(STUDENT_ID);
  Serial.print(",dist:");
  Serial.print(currentDistance);
  Serial.print(",deg:");
  Serial.println(currentDegrees);
  
  lastUpdate = ct;
}

#pragma endregion

#pragma region Utilitaires moteur

long degreesToSteps(int deg) {
  return deg * STEPS_PER_DEG;
}

int stepsToDegrees(long steps) {
  return steps / STEPS_PER_DEG;
}

#pragma endregion