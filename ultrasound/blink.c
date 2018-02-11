#include <wiringPi.h>
int main (void) {
  // Initialize wiringPi
  wiringPiSetup();

  // References to the IO pins
  int TRIG = 23;
  int ECHO = 24;

  // Setup the pins
  pinMode(TRIG, OUTPUT);
  pinMond(ECHO, INPUT);

  // Settle the sensor
  printf("Waiting for sensor to settle.");
  digitalOuptut(TRIG, LOW);
  delay(2000);
  
  // Start the ranging program on the sensor
  // By sending a short 10uS pulse on the TRIG pin
  digitalOutput(TRIG, HIGH);
  delayMicroseconds(10);
  digitalOutput(TRIG, LOW);

  // Start and end times will be used to work out distance
  time_t startTime;
  time_t endTime;

  // Start time occurs once the ECHO pin goes high
  printf("Waiting for ultrasound to start.");
  while (digitalInput(ECHO) == 0) {
    startTime = time();
  }

  // Now we've recieved a signal we wait until
  printf("Waiting for ultrasound to end."); 
  while (digitalInput(ECHO) == 1) {
    endTime = time();
  }

  // Now work out the duration
  int duration = stat


  pinMode(0, OUTPUT);
  for (;;) {
    digitalWrite(0, HIGH); delay(500);
    digitalWrite(0, LOW); delay(500);
  }

  return 0;
}
