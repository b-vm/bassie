import time
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)
ser.flush()

# while True:
#     time.sleep(0.025)
#     ser.write(b's 255 0 0 0 0 0 0 0')

for i in range(0, 255):
    ser.write(f"s {i} 0 0 0 0 0 0 0".encode('utf-8'))
    time.sleep(0.025)

    # if ser.in_waiting > 0:
    #     line = ser.readline().decode('utf-8').rstrip()
    #     print(line, ser.in_waiting)
