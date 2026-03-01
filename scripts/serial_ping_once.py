import serial
import time

PORT = "COM4"      # ← 改成你设备管理器里看到的
BAUD = 115200
TIMEOUT = 0.5

ser = serial.Serial(
    port=PORT,
    baudrate=BAUD,
    timeout=TIMEOUT
)

print("串口已打开:", ser.name)

time.sleep(1.0)

ser.write(b"PING\n")
resp = ser.readline()
print("收到:", resp)

ser.close()