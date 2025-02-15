#include <Servo.h>

// Define pin connections
#define posControlSignal 3           // PWM output to servo motor (Orange wire)
#define posFeedbackSignal A3         // Analog input from encoder (White wire)
#define touchPin1 A4                 // Analog input from first touch sensor (purple wire)
#define touchPin2 A5                 // Analog input from second touch sensor (blue wire)

Servo myservo;
int currentServoPosition;       // Start at 90 degrees (straight down)
int threshold = 512; //New

void setup() {
  Serial.begin(9600);                   // Start serial communication at 9600 baud
  pinMode(touchPin1, INPUT);            // Set touchPin1 as an input
  pinMode(touchPin2, INPUT);            // Set touchPin2 as an input
  pinMode(posControlSignal, OUTPUT);    // Set control signal pin as an output
  myservo.attach(posControlSignal);     // Attach the servo object to the control pin
  currentServoPosition = 90;            // Start at 90 degrees (straight down)
  myservo.write(currentServoPosition);  // Move servo to the new 0 degrees position (formerly straight down)
  delay(15);                            // Wait for 1 second to allow servo to reach position
}

void loop() {
    // Read desired new servo position
    int targetPosition = Serial.parseInt();
    if (targetPosition >= 0 && targetPosition <= 90) {
        int direction = (targetPosition - currentServoPosition > 0) ? 1 : -1;
        for (int i = currentServoPosition; i != targetPosition; i += direction)
        {
            myservo.write(i);
            delay(50);
        }
        currentServoPosition = targetPosition;
    }

    // Check if buttons pressed
    bool touch1 = digitalRead(touchPin1) == HIGH;
    bool touch2 = digitalRead(touchPin2) == HIGH;
    
    // Send new servo position and button state to ros2
    Serial.print(currentServoPosition);
    Serial.print(","); 
    Serial.print(touch1 ? "1" : "0"); 
    Serial.print(","); 
    Serial.println(touch2 ? "1" : "0"); 
    
}