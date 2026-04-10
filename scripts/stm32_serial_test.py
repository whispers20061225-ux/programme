import os
import serial
import time

# =========================
# 串口配置
# =========================
PORT = os.environ.get("STM32_PORT") or ("COM4" if os.name == "nt" else "/dev/ttyS4")
BAUD = 115200       # 这是 USB CDC 虚拟串口的波特率
TIMEOUT = 1.0


def send_cmd(ser, cmd: str, wait_s=0.1):
    """
    发送一条文本命令，并读取一行返回
    """
    if not cmd.endswith("\n"):
        cmd += "\n"

    print(f">> {cmd.strip()}")
    ser.write(cmd.encode("utf-8"))

    time.sleep(wait_s)

    resp = ser.readline().decode(errors="ignore").strip()
    print(f"<< {resp}")
    return resp


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    except Exception as e:
        print("串口打开失败：", e)
        return

    print("串口已打开:", ser.name)

    # 给 STM32 一点启动缓冲时间
    time.sleep(1.0)

    # =========================
    # 基础命令测试
    # =========================
    send_cmd(ser, "PING")
    send_cmd(ser, "VER")

    # =========================
    # 触觉传感器测试
    # 先试设备地址 1
    # =========================
    send_cmd(ser, "TRAW 1", wait_s=0.2)

    ser.close()
    print("串口已关闭")


if __name__ == "__main__":
    main()
