#include <assert.h>

# define MOTOR_PIN_RANGE_START 2
# define MOTOR_PIN_RANGE_END 11
# define N_MOTORS 10
# define CONTROL_FREQ 2000

unsigned long timeLastStep = 0;
unsigned int period = 1000000/CONTROL_FREQ;
unsigned int targets[N_MOTORS];
bool commanded = false;


void setup() {
    assert(MOTOR_PIN_RANGE_END - MOTOR_PIN_RANGE_START + 1 == N_MOTORS);
    Serial.begin(115200);
    for (int i = 0; i < N_MOTORS; i++) {
        pinMode(i + MOTOR_PIN_RANGE_START, OUTPUT);
        targets[i] = 0;
    }
}

void loop() {
    if (micros() - timeLastStep < period) {
        delayMicroseconds(10);
        return;
    }
    Serial.println(micros() - timeLastStep);
    timeLastStep = micros();
    controllerStep();
}

void controllerStep() {
    checkInputs();
    if (commanded) {
        driveMotors();
    }
}

void checkInputs() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 's') {
            for (int i = 0; i < N_MOTORS; i++) {
                targets[i] = Serial.parseInt();
            }
            commanded = true;
        }
    }
}

void driveMotors() {
    for (int i = 0; i < N_MOTORS; i++) {
        analogWrite(i + MOTOR_PIN_RANGE_START, targets[i]);
    }
}