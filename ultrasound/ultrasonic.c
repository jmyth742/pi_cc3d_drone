#include <wiringPi.h>
#include <stdio.h>
#include <time.h>
int main () {
  // Initialize wiringPi
  printf("Starting wiringPi setup\n");
  wiringPiSetup();

  // References to the IO pins
  int TRIG = 4;
  int ECHO = 5;

  // Setup the pins
  printf("Setting up the pins\n");
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Settle the sensor
  printf("Waiting for sensor to settle.\n");
  digitalWrite(TRIG, LOW);
  delay(2000);

  // Loop forever
  while (1) {
    // Start the ranging program on the sensor
    // By sending a short 10uS pulse on the TRIG pin
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    // Start and end times will be used to work out distance
    clock_t startTime;
    clock_t endTime;

    // Start time occurs once the ECHO pin goes high
    while (digitalRead(ECHO) == 0) {
      startTime = clock();
    }

    // Now we've recieved a signal we wait until
    while (digitalRead(ECHO) == 1) {
      endTime = clock();
    }

    // Now work out the duration in milliseconds and print it
    long durationMillis = ((double) endTime - startTime) / CLOCKS_PER_SEC * 1000;
    printf("Duration: %d ms \n", durationMillis);

    // Now work out the distance in cm and print it
    int distanceCentimeters = (((double) durationMillis / 1000) * 343) * 100;
    printf("Distance: %d cm \n", distanceCentimeters);

    // Now wait a second before looping again
    delay(1000);
  }

  // Return
  return 1;
}
