import RPi.GPIO as gpio
import time

pins = {
        'pwm': 17,
        'a': 27,
        'b': 22
}
gpio.setmode(gpio.BCM)

for pin in pins.values():
    print(pin)
    gpio.setup(pin, gpio.OUT)

pwm = gpio.PWM(pins['pwm'], 1000)
pwm.start(0)

while(True):
    for duty_cycle in range(0, 101, 10):
        pwm.ChangeDutyCycle(duty_cycle)

        for i, j in zip([0, 1], [1, 0]):
            gpio.output(pins['a'], i)
            gpio.output(pins['b'], j)
            time.sleep(1)




