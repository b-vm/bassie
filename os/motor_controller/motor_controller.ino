#include <assert.h>

# define MOTOR_PIN_RANGE_START 2
# define MOTOR_PIN_RANGE_END 11
# define ENCODER_PIN_RANGE_START 54
# define ENCODER_PIN_RANGE_END 63
# define MOTOR_DIRECTION_PIN_RANGE_START 22
# define MOTOR_DIRECTION_PIN_RANGE_END 41
# define N_MOTORS 10
# define CONTROL_FREQ 2000

# define VERBOSE true


// TOOD
// - ensure micros() overflow is handled correctly


class PIDController {
    public:
        PIDController() {}

        PIDController(float kp, float ki, float kd, float dt) {
            this->kp = kp;
            this->ki = ki;
            this->kd = kd;
            this->dt = dt;
            this->errorSum = 0;
            this->errorLast = 0;
        }

        float step(float error) {
            this->errorSum += error * dt;
            float errorDiff = (error - errorLast) / dt;
            this->errorLast = error;
            return kp * error + ki * errorSum + kd * errorDiff;
        }

        String getSettings() {
            return String(kp) + ", " + String(ki) + ", " + String(kd);
        }

    private:
        float kp;
        float ki;
        float kd;
        float dt;
        float errorSum;
        float errorLast;
};


unsigned long timeLastStep = 0;
unsigned int period = 1000000/CONTROL_FREQ;
unsigned int targets[N_MOTORS];
unsigned int encoderValues[N_MOTORS];
bool commanded = false;
int stepsSinceCommand = 0;
PIDController pids[N_MOTORS];



void setup() {
    Serial.begin(115200); // for debugging
    Serial1.begin(115200); // comunication with raspberry pi

    assert(MOTOR_PIN_RANGE_END - MOTOR_PIN_RANGE_START + 1 == N_MOTORS);
    assert(ENCODER_PIN_RANGE_END - ENCODER_PIN_RANGE_START + 1 == N_MOTORS);
    assert(MOTOR_DIRECTION_PIN_RANGE_END - MOTOR_DIRECTION_PIN_RANGE_START + 1 == N_MOTORS * 2);
    for (int i = 0; i < N_MOTORS; i++) {
        pinMode(i + MOTOR_PIN_RANGE_START, OUTPUT);
        pinMode(i + MOTOR_DIRECTION_PIN_RANGE_START, OUTPUT);
        pinMode(i + MOTOR_DIRECTION_PIN_RANGE_START + N_MOTORS, OUTPUT);
        targets[i] = 0;
    }

    ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear ADC prescaler bits
    ADCSRA |= (1 << ADPS2) | (0 << ADPS1) | (1 << ADPS0); // 32 prescaler = 4 times faster and less accurate

    wait_for_handshake();
}

void wait_for_handshake() {
    while (true) {
        if (Serial1.available() > 0) {
            if (Serial1.read() == 's') {
                String msg = Serial1.readStringUntil('e');
                Serial.println("Handshake started. Received: " + msg);
                for (int i = 0; i < N_MOTORS; i++) {
                    float kp = parseNextFloat(msg);
                    float ki = parseNextFloat(msg);
                    float kd = parseNextFloat(msg);
                    pids[i] = PIDController(kp, ki, kd, 1.0/CONTROL_FREQ);
                    Serial.println("PID parameters actuator " + String(i) + ":" + pids[i].getSettings() + msg);
                }
                Serial.println("Initialization complete");
                return;
            }
            Serial.println("Initialization failed. Listening for handshake again...");
        }
    }
}

float parseNextFloat(String &str) {
    while (str[0] == ' ') str.remove(0, 1);
    int i = 0;
    while (str[i] != ' ') i++;
    float f = str.substring(0, i).toFloat();
    str.remove(0, i);
    return f;
}



void loop() {
    wait_if_needed();
    timeLastStep = micros();
    controllerStep();
}

void wait_if_needed() {
    int waitTime = period - (micros() - timeLastStep);
    if (waitTime < 0) {
        delayMicroseconds(waitTime);
    }
    if (VERBOSE) {
        Serial.println("wait time: " + String(waitTime) + "us, available time: " + String(period) + "us");
    }
}

void controllerStep() {
    checkTargetInputs();
    registerEncoderReadings();
    if (commanded) {
        driveMotors();
    }
    stepsSinceCommand++;
}

// TODO: convert to angles?
void checkTargetInputs() {
    while (Serial1.available() > 0) {
        if (Serial1.read() == 's') {
            String msg = Serial.readStringUntil('e');
            for (int i = 0; i < N_MOTORS; i++) {
                targets[i] = parseNextFloat(msg);
            }
        }
        if (VERBOSE) {
            Serial.println("steps between commands: " + String(stepsSinceCommand));
        }
        commanded = true;
        stepsSinceCommand = 0;
        return;
    }
}

// TODO convert to angles?
void registerEncoderReadings() {
    Serial1.print('d ');
    Serial1.print(stepsSinceCommand + " ");
    for (int i = 0; i < N_MOTORS; i++) {
        encoderValues[i] = analogRead(i + ENCODER_PIN_RANGE_START);
        Serial1.print(String(encoderValues[i]) + " ");
    }
    Serial1.print('e');
}

void driveMotors() {
    for (int i = 0; i < N_MOTORS; i++) {
        int error = targets[i] - encoderValues[i];
        int correction = pids[i].step(error);
        if (correction > 0) {
            digitalWrite(i + MOTOR_DIRECTION_PIN_RANGE_START, HIGH);
            digitalWrite(i + MOTOR_DIRECTION_PIN_RANGE_START + N_MOTORS, LOW);
        } else {
            digitalWrite(i + MOTOR_DIRECTION_PIN_RANGE_START, LOW);
            digitalWrite(i + MOTOR_DIRECTION_PIN_RANGE_START + N_MOTORS, HIGH);
        }
        // TODO clipping
        analogWrite(i + MOTOR_PIN_RANGE_START, min(abs(correction), 255));

        if (VERBOSE) {
            Serial.println("actuator " + String(i) + " error: " + String(error) + " correction: " + String(correction));
        }
    }
}