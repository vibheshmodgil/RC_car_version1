import serial
import time

ser = serial.Serial("/dev/serial0",115200,timeout=1)

time.sleep(2)

cmd='{"cmd":"stop"}\n'

print(cmd)

ser.write(cmd.encode())

while True:
    line=ser.readline().decode(errors="ignore").strip()

    if line:
        print(line)