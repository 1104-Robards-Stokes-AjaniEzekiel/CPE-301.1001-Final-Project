#include <DHT.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>

// RTC
RTC_DS3231 rtc;

const int waterLevelPin = A7;
const int redLedPin = 5;
const int greenLedPin = 2;
const int yellowLedPin = 4;
const int blueLedPin = 3;
const int fanEnaPin = 7;
const int fanIn1Pin = 8;
const int fanIn2Pin = 9;
const int stepperStepsPerRevolution = 2048;
Stepper myStepper(stepperStepsPerRevolution, 10, 11, 12, 13);
const int stepperButtonPin = 51;
LiquidCrystal lcd(38, 39, 40, 41, 42, 43);
#define DHTPIN 6
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

enum CoolerState {
  DISABLED,
  IDLE,
  ERROR_STATE,
  RUNNING
};

CoolerState currentState = DISABLED;

bool coolerOn = true;
bool stepperRunning = false;
bool stepperAllowed = true;
const float tempThreshold = 20.0;

const int buttonPin1 = 18;
const int buttonPin2 = 19;
int lastButtonState1 = LOW;
int lastButtonState2 = LOW;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long debounceDelay = 50;

void setup() {
  Serial.begin(9600);
  Serial.println("System Initializing...");

  lcd.begin(16, 2);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  pinMode(fanEnaPin, OUTPUT);
  pinMode(fanIn1Pin, OUTPUT);
  pinMode(fanIn2Pin, OUTPUT);
  pinMode(stepperButtonPin, INPUT_PULLUP);
  myStepper.setSpeed(10);

  stopFanMotor();
  stopStepperMotor();
  setState(DISABLED);
  dht.begin();
Wire.begin();  // Initialize I2C communication for RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void loop() {
  handleSerialInput();
  handleButtonPress();

  if (coolerOn) {
    handleCoolerOperation();
    handleStepperButton();
  }
}

void handleSerialInput() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.equals("off")) {
      coolerOn = false;
      stepperRunning = false;
      lcd.clear();
      setState(DISABLED);
      while (Serial.available() > 0) {
        Serial.read();
      }
      stopFanMotor();
      stopStepperMotor();
    } else {
      coolerOn = true;
      setState(IDLE);
      digitalWrite(yellowLedPin, LOW);
    }
  }
}

void handleButtonPress() {
  int buttonState1 = digitalRead(buttonPin1);
  int buttonState2 = digitalRead(buttonPin2);

  if (buttonState1 == HIGH) {
    if (currentState == ERROR_STATE) {
      setState(IDLE);
    }
  }

  if (buttonState2 == HIGH) {
    if (currentState == ERROR_STATE) {
      setState(DISABLED);
    } else if (currentState == IDLE) {
      setState(DISABLED);
    } else if (currentState == DISABLED) {
      setState(IDLE);
    }
  }
}

void handleCoolerOperation() {
  int waterLevel = analogRead(waterLevelPin);
  float percentage = map(waterLevel, 0, 1023, 0, 100);

  if (percentage < 30) {
    setState(ERROR_STATE);
    return;
  } else {
    float temperature = getTemperature();

    switch (currentState) {
      case IDLE:
        handleIdleState(temperature);
        break;
      case RUNNING:
        handleRunningState(temperature, percentage);
        break;
      case ERROR_STATE:
        handleErrorState(percentage);
        break;
    }
  }
}

void logTimestamp() {
  DateTime now = rtc.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}

void handleIdleState(float temperature) {
  lcd.clear();
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(getHumidity());
  lcd.print("%");

  if (temperature > tempThreshold) {
    setState(RUNNING);
    startFanMotor();
  }
}

void handleRunningState(float temperature, float percentage) {
  lcd.clear();
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(getHumidity());
  lcd.print("%");

  if (temperature <= tempThreshold) {
    setState(IDLE);
    stopFanMotor();
  } else if (percentage < 30) {
    setState(ERROR_STATE);
    stopFanMotor();
  }
}

void handleDISABLEDState() {
  stepperAllowed = true;
  stopFanMotor();
}

void handleErrorState(float percentage) {
  stopFanMotor();

  if (percentage >= 30) {
    setState(IDLE);
  }

  lcd.clear();
  lcd.print("Error: Water is ");
  lcd.setCursor(0, 1);
  lcd.print("too low");
  delay(2000);
  lcd.clear();
  stepperAllowed = false;
}

void handleStepperButton() {
  if (digitalRead(stepperButtonPin) == LOW) {
    if (!stepperRunning && stepperAllowed) {
      Serial.println("Stepper Motor ON");
      stepperRunning = true;
    }

    Serial.println("Stepper Motor Step");
    myStepper.step(10);
    delay(5);
  } else {
    if (stepperRunning) {
      Serial.println("Stepper Motor OFF");
      stepperRunning = false;
    }
  }
}

void startFanMotor() {
  digitalWrite(fanIn1Pin, HIGH);
  digitalWrite(fanIn2Pin, LOW);
  analogWrite(fanEnaPin, 255);
}

void stopFanMotor() {
  digitalWrite(fanIn1Pin, LOW);
  digitalWrite(fanIn2Pin, LOW);
  analogWrite(fanEnaPin, 0);
}

void stopStepperMotor() {
  myStepper.step(0);
}

float getTemperature() {
  return dht.readTemperature();
}

float getHumidity() {
  return dht.readHumidity();
}

void setState(CoolerState newState) {
  logTimestamp();  // Log timestamp for each state transition

  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);
  digitalWrite(yellowLedPin, LOW);
  digitalWrite(blueLedPin, LOW);

  currentState = newState;

  switch (currentState) {
    case DISABLED:
      handleDISABLEDState();
      stopFanMotor();
      digitalWrite(yellowLedPin, HIGH);
      break;
    case IDLE:
      stopFanMotor();
      digitalWrite(greenLedPin, HIGH);
      break;
    case ERROR_STATE:
      stopFanMotor();
      digitalWrite(redLedPin, HIGH);
      lcd.clear();
      lcd.print("Error: Water is ");
      lcd.setCursor(0, 1);
      lcd.print("too low");
      delay(2000);
      lcd.clear();
      stepperAllowed = false;
      break;
    case RUNNING:
      digitalWrite(blueLedPin, HIGH);
      startFanMotor();
      break;
  }
}
