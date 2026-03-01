#串口名：/dev/cu.usbmodem6D7F245051571
import serial, time

PORT = "/dev/cu.usbmodem6D7F245051571"   # 改成你的
ser = serial.Serial(PORT, 115200, timeout=0.5)
time.sleep(1)

for cmd in [b"PING\n", b"VER\n", b"ECHO hello world\n", b"FOO\n"]:
    ser.write(cmd)
    print(">>", cmd.strip())
    print("<<", ser.readline().decode(errors="ignore").strip())

ser.close()