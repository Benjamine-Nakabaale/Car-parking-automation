#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

Servo myservo;   
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Initialize LCD at I2C address 0x27

int pos = 0;
int cm = 0;
int carCount = 0;

// Pin definitions
#define SERVO_PIN PB1      // Servo on pin 9 (OC1A on Arduino Uno)
#define TRIG_PIN PD6       // Trigger on pin 6
#define ECHO_PIN PD7       // Echo on pin 7

// Function to set up pin modes manually using DDR registers
void configurePins() {
  // Set trigger pin as output
  DDRD |= (1 << TRIG_PIN);
  
  // Set echo pin as input
  DDRD &= ~(1 << ECHO_PIN);
  
  // Initialize servo pin as output
  DDRB |= (1 << SERVO_PIN);
}

// Function to set the trigger pin high or low
void setTriggerPin(bool state) {
  if (state) {
    PORTD |= (1 << TRIG_PIN);  // Set high
  } else {
    PORTD &= ~(1 << TRIG_PIN); // Set low
  }
}

// Read distance from ultrasonic sensor
long readUltrasonicDistance() {
  // Trigger pulse
  setTriggerPin(false);
  _delay_us(2);
  setTriggerPin(true);
  _delay_us(10);
  setTriggerPin(false);

  // Wait for echo response and calculate pulse width
  while (!(PIND & (1 << ECHO_PIN))) ;  // Wait for echo high
  unsigned long startTime = micros();
  
  while (PIND & (1 << ECHO_PIN)) ;     // Wait for echo low
  unsigned long endTime = micros();

  return endTime - startTime;
}

// Function to rotate servo to a specific angle
void rotateServo(int angle) {
  // Calculate duty cycle for given angle
  OCR1A = map(angle, 0, 180, 1000, 2000);  // Set PWM width based on angle
  delay(15);  // Small delay for servo to reach position
}

void setup() {
  configurePins();   // Set up the pin configurations
  Serial.begin(9600);
  
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Cars Passed: 0");

  // Configure Timer1 for Servo PWM control
  TCCR1A = (1 << COM1A1) | (1 << WGM11);    // Set up Fast PWM on OC1A (Pin 9)
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Prescaler /8
  ICR1 = 20000;                             // Set TOP for 20ms period (50Hz PWM)
}

void loop() {
  cm = 0.01723 * readUltrasonicDistance();

  if (cm < 30) { // Object within 30 cm triggers gate
    Serial.print(cm);
    Serial.println(" cm");

    // Open gate
    rotateServo(120);
    delay(1000);  // Allow car to pass

    // Update car count and display
    carCount++;
    lcd.setCursor(0, 0);
    lcd.print("Cars Passed: ");
    lcd.setCursor(12, 0);
    lcd.print(carCount);
    lcd.print("   ");  // Clear any leftover characters

    // Close gate
    rotateServo(0);
    delay(2000);  // Wait before next check
  }
}
