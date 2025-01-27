#include <math.h>

// Pin Definitions
#define SWITCH_PIN      2   // PD2 (Digital Pin 2)
#define PWM_INPUT_PIN   8   // PB0 (Digital Pin 8 - ICP1)
#define PWM_OUTPUT_PIN1 9   // PB1 (Digital Pin 9 - OC1A)
#define PWM_OUTPUT_PIN2 10  // PB2 (Digital Pin 10 - OC1B)
#define LED_PIN         LED_BUILTIN

// Configuration
#define MAX_SPEED       200.0
#define MAX_FREQUENCY   200.0
#define CALIBRATION_STEPS 3
#define DEBOUNCE_DELAY  200

// Global Variables
volatile uint32_t prevCapture = 0;
volatile uint16_t overflowCount = 0;
volatile float measuredFrequency = 0.0;
volatile bool frequencyUpdated = false;

int calibrationStep = 0;
float calibrationFrequencies[CALIBRATION_STEPS] = {0};
const float knownSpeeds[CALIBRATION_STEPS] = {10.0, 100.0, 200.0};
float coeffA = 0, coeffB = 0, coeffC = 0;

void setup() {
  // --- Changed from INPUT_PULLUP to INPUT ---
  pinMode(SWITCH_PIN, INPUT);   
  
  pinMode(PWM_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN1, OUTPUT);
  pinMode(PWM_OUTPUT_PIN2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Configure Timer1 for Input Capture + Overflow Interrupt
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << ICES1) | (1 << CS11);  // Rising edge, prescaler=8
  TIMSK1 |= (1 << ICIE1) | (1 << TOIE1); // Enable interrupts

  // Configure Timer1 for dual PWM (Mode 14: Fast PWM w/ ICR1 as TOP)
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  ICR1 = 19999;         // Initial TOP (~100 Hz)
  OCR1A = ICR1 / 2;     // 50% duty cycle on OC1A
  OCR1B = ICR1 / 2;     // 50% duty cycle on OC1B

  // Do the calibration steps
  while(calibrationStep < CALIBRATION_STEPS) {
    handleCalibration();
  }
  calculateCoefficients();
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if(frequencyUpdated) {
    noInterrupts();
    float currentFreq = measuredFrequency;
    frequencyUpdated = false;
    interrupts();

    float speed = calculateCurrentSpeed(currentFreq);
    updatePWM(speed);
  }
}

// ----------------------------------------------------------------------------
// Interrupt Service Routines
// ----------------------------------------------------------------------------
ISR(TIMER1_CAPT_vect) {
  uint32_t currentCapture = (overflowCount << 16) | ICR1;
  uint32_t periodTicks = currentCapture - prevCapture;
  
  prevCapture = currentCapture;
  overflowCount = 0;
  
  if(periodTicks > 0) {
    measuredFrequency = 16000000.0 / (periodTicks * 8);
    frequencyUpdated = true;
  }
}

ISR(TIMER1_OVF_vect) {
  overflowCount++;
}

// ----------------------------------------------------------------------------
// PWM Update for Dual Outputs
// ----------------------------------------------------------------------------
void updatePWM(float speed) {
  float frequencyHz = constrain(speed, 0, MAX_FREQUENCY);
  
  if(frequencyHz <= 0) {
    OCR1A = 0;
    OCR1B = 0;
    return;
  }

  // Compute the ICR1 (TOP) for the desired frequency
  uint32_t topValue = (16000000UL / (8 * frequencyHz)) - 1;
  topValue = constrain(topValue, 1, 65535);
  uint16_t dutyValue = topValue / 2;  // 50% duty cycle

  noInterrupts();
  ICR1 = topValue;    // Set frequency (affects both PWM channels)
  OCR1A = dutyValue;  // Set OC1A duty cycle
  OCR1B = dutyValue;  // Set OC1B duty cycle
  interrupts();
}

// ----------------------------------------------------------------------------
// Calibration Functions
// ----------------------------------------------------------------------------
void handleCalibration() {
  // For a pull-down circuit, the button is LOW normally, HIGH when pressed
  static uint32_t lastDebounce = 0;
  static bool buttonState = LOW, lastButtonState = LOW;

  bool reading = digitalRead(SWITCH_PIN);

  // Simple debounce check
  if (reading != lastButtonState) {
    lastDebounce = millis();
  }

  if ((millis() - lastDebounce) > DEBOUNCE_DELAY && (reading != buttonState)) {
    buttonState = reading;
    // Now we consider a press as 'reading == HIGH'
    if (buttonState == HIGH) {
      // Store the current measured frequency in the calibration array
      calibrationFrequencies[calibrationStep] = measuredFrequency;

      // Quick LED flash to acknowledge
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);

      calibrationStep++;
    }
  }

  lastButtonState = reading;

  // Blink LED during calibration (faster each step)
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > (500 / (calibrationStep + 1))) {
    lastBlink = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

// Use the three (speed, frequency) pairs to fit a quadratic for speed->frequency
void calculateCoefficients() {
  float x1 = knownSpeeds[0], y1 = calibrationFrequencies[0];
  float x2 = knownSpeeds[1], y2 = calibrationFrequencies[1];
  float x3 = knownSpeeds[2], y3 = calibrationFrequencies[2];

  float denom = (x1 - x2)*(x1 - x3)*(x2 - x3);
  coeffA = (x3*(y2 - y1) + x2*(y1 - y3) + x1*(y3 - y2)) / denom;
  coeffB = (x3*x3*(y1 - y2) + x1*x1*(y2 - y3) + x2*x2*(y3 - y1)) / denom;
  coeffC = (x2*x3*(x2 - x3)*y1 + x3*x1*(x3 - x1)*y2 + x1*x2*(x1 - x2)*y3) / denom;
}

// Solve the quadratic for speed given a measured frequency
float calculateCurrentSpeed(float frequency) {
  float a = coeffA, b = coeffB, c = coeffC - frequency;
  float discriminant = b*b - 4*a*c;
  if (discriminant < 0) return 0;

  float sqrt_d = sqrt(discriminant);
  float speed1 = (-b + sqrt_d) / (2*a);
  float speed2 = (-b - sqrt_d) / (2*a);

  // Only positive (physical) solution, constrained to max speed
  return constrain((speed1 > 0) ? speed1 : speed2, 0.0, MAX_SPEED);
}
