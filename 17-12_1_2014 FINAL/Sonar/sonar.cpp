#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

#define TRIG 5

void setup() {
	wiringPiSetup();
}

int getCM() {
  	pinMode(TRIG, OUTPUT);
  	digitalWrite(TRIG, LOW);
  	delay(30);
  	digitalWrite(TRIG, HIGH);
  	delay(50);
  	digitalWrite(TRIG, LOW);
	pinMode(TRIG, INPUT);

        //Wait for echo start
        while(digitalRead(TRIG) == LOW);

        //Wait for echo end
        long startTime = micros();
        while(digitalRead(TRIG) == HIGH);
        long travelTime = micros() - startTime;

        //Get distance in cm
        int distance = travelTime / 58;

        return distance;
}

int main(void) {
        setup();

	while (1) {
        	printf("Distance: %dcm\n", getCM());
		delay(1000);
	}
        
	return 0;
}
