import serial  
import json
import time

print("测试")

ser = serial.Serial('COM11', 115200, timeout=1)  # 修改为你的端口

def send_json(cmd, value, name,bbb):
    data = {"x": cmd, "y": value, "z": name,"q": bbb}
    ser.write((json.dumps(data) + '\n').encode())  # 发送 JSON + 换行符

try:
    while True:
        send_json(1, 2, 3,4)  # 开 LED
        print("1")
        time.sleep(1)
        send_json(1, 2, 3,4)  # 开 LED
        print("0")
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    ser.close()