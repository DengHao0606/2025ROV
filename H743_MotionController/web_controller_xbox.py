import json
import sys
import time
import socket
import keyboard
import threading
import os
import portalocker
import serial
import struct
import pygame   #与游戏手柄进行一个交互的pygame模块，下面有个joystick的函数用于获取一些操作轴的相关信息的

#在windows读取的手柄的信息通过socket通道进行传输

#                              _ooOoo_
#                             o8888888o
#                             88" . "88
#                             (| -_- |)
#                              O\ = /O
#                           ____/`---'\____
#                        .   ' \\| |// `.
#                         / \\||| : |||// \
#                        / _||||| -:- |||||- \
#                         | | \\\ - /// | |
#                       | \_| ''\---/'' | |
#                        \ .-\__ `-` ___/-. /
#                    ___`. .' /--.--\ `. . __
#                  ."" '< `.___\_<|>_/___.' >'"".
#                 | | : `- \`.;`\ _ /`;.`/ - ` : | |
#                    \ \ `-. \_ __\ /__ _/ .-` / /
#           ======`-.____`-.___\_____/___.-`____.-'======
#                              `=---='
#
#           .............................................
#                     佛祖保佑             永无BUG

# 使用字典，记录按键操作


#=========================================================================

#         L1   axis(4)                              R1 axis(5)
#          L2   button(4)                          R2  button(5)
#       _=====_                                  _=====_
#      / _____ \                                / _____ \
#    +.-'_____'-.------------------------------.-'_____'-.+
#   /   |     |  '.        X B O X           .'  |  _  |   \
#  / ___| /|\ |___ \                        / ___| /_\ |___ \      (3)
# / |      |      | ;                      ; | _         _ ||
# | | <---   ---> | |                      | ||_|       (_)||  (2)     (1)
# | |___   |   ___| ;                      ; |___       ___||
# |\    | \|/ |    /  _      ____      _   \    | (X) |    /|   (button(0))
# | \   |_____|  .','" "',  (_PS_)  ,'" "', '.  |_____|  .' |
# |  '-.______.-' /       \        /       \  '-._____.-'   |
# |               |  LJ   |--------|  RJ   |                |
# |              /\       /        \       /\               |
# |             /  '.___.'          '.___.'  \              |
# |            /                              \             |
#  \          /  x轴 axis(0)       x轴 axis(2) \           /   
#   \________/   y轴 axis(1)        y轴 axis(3)  \_________/   

#手柄键位分布 以及编号
#备注：   axis(2)方向，控制左右的方向。     axis(0)方向，控制yaw的方向。    axis(1)方向，控制前后的方向。    axis(3)方向，控制上下的方向。

# 电机参数
MOTOR_PARAMS = {
    "m0": {
        'num': 0,
        "np_mid": 2500.0,
        "np_ini": 3000.0,
        "pp_ini": 3000.0,
        "pp_mid": 3500.0,

        "nt_end": -1500.0,
        "nt_mid": -750.0,
        "pt_mid": 750.0,
        "pt_end": 1500.0,
    },
    "m1": {
        'num': 1,
        "np_mid": 2500.0,
        "np_ini": 3000.0,
        "pp_ini": 3000.0,
        "pp_mid": 3500.0,

        "nt_end": -1500.0,
        "nt_mid": -750.0,
        "pt_mid": 750.0,
        "pt_end": 1500.0,
    },
    "m2": {
        'num': 2,
        "np_mid": 2500.0,
        "np_ini": 3000.0,
        "pp_ini": 3000.0,
        "pp_mid": 3500.0,

        "nt_end": -1500.0,
        "nt_mid": -750.0,
        "pt_mid": 750.0,
        "pt_end": 1500.0,
    },
    "m3": {
        'num': 3,
        "np_mid": 2500.0,
        "np_ini": 3000.0,
        "pp_ini": 3000.0,
        "pp_mid": 3500.0,

        "nt_end": -1500.0,
        "nt_mid": -750.0,
        "pt_mid": 750.0,
        "pt_end": 1500.0,
    },
    "m4": {
        'num': 4,
        "np_mid": 2500.0,
        "np_ini": 3000.0,
        "pp_ini": 3000.0,
        "pp_mid": 3500.0,

        "nt_end": -1500.0,
        "nt_mid": -750.0,
        "pt_mid": 750.0,
        "pt_end": 1500.0,
    },
    "m5": {
        'num': 5,
        "np_mid": 2500.0,
        "np_ini": 3000.0,
        "pp_ini": 3000.0,
        "pp_mid": 3500.0,

        "nt_end": -1500.0,
        "nt_mid": -750.0,
        "pt_mid": 750.0,
        "pt_end": 1500.0,
    }
}

# 硬件控制类（通过UDP协议发送数据）
class HardwareController:
    def __init__(self, server_address):
        self.server_address = server_address
        self.curves = type('', (), {})()  # 动态创建曲线数据对象
        
        # # 初始化电机参数（示例值，需根据实际硬件调整）
        # motors = ['m0', 'm1', 'm2', 'm3', 'm4', 'm5']
        # default_values = {
        #     'num': 0, 'np_mid': 1.0, 'np_ini': 0.5, 'pp_ini': 0.5,
        #     'pp_mid': 1.0, 'nt_end': 0.0, 'nt_mid': 0.5,
        #     'pt_mid': 0.5, 'pt_end': 0.0
        # }
        
        # for i, motor in enumerate(motors):
        #     values = default_values.copy()
        #     values['num'] = i  # 电机编号
        #     setattr(self.curves, motor, type('', (), values)())

    # def send_thrust_data(self, motor_name, client_socket):
    #     """发送单个电机的推力参数到网络"""
    #     motor = getattr(self.curves, motor_name)
    #     data = {
    #         "cmd": "thrust_init",
    #         "motor": motor.num,
    #         "np_mid": motor.np_mid,
    #         "np_ini": motor.np_ini,
    #         "pp_ini": motor.pp_ini,
    #         "pp_mid": motor.pp_mid,
    #         "nt_end": motor.nt_end,
    #         "nt_mid": motor.nt_mid,
    #         "pt_mid": motor.pt_mid,
    #         "pt_end": motor.pt_end
    #     }
    #     json_str = json.dumps(data) + "\n"
    #     client_socket.sendto(json_str.encode(), self.server_address)

    def send_thrust_data(self, motor_name, client_socket):
            """发送单个电机的推力参数到网络"""
            if motor_name in MOTOR_PARAMS:
                data = {
                    "cmd": "thrust_init",
                    "motor": MOTOR_PARAMS[motor_name]['num'],
                    "np_mid": MOTOR_PARAMS[motor_name]['np_mid'],
                    "np_ini": MOTOR_PARAMS[motor_name]['np_ini'],
                    "pp_ini": MOTOR_PARAMS[motor_name]['pp_ini'],
                    "pp_mid": MOTOR_PARAMS[motor_name]['pp_mid'],
                    "nt_end": MOTOR_PARAMS[motor_name]['nt_end'],
                    "nt_mid": MOTOR_PARAMS[motor_name]['nt_mid'],
                    "pt_mid": MOTOR_PARAMS[motor_name]['pt_mid'],
                    "pt_end": MOTOR_PARAMS[motor_name]['pt_end']
                }
                json_str = json.dumps(data) + "\n"
                client_socket.sendto(json_str.encode(), self.server_address)
    def hwinit(self, client_socket):
        """初始化所有电机参数"""
        for motor_name in ["m0", "m1", "m2", "m3", "m4", "m5"]:
            self.send_thrust_data(motor_name, client_socket)
            time.sleep(0.05)  # 防止数据堆积


#控制器的值
CONTROLLER_INIT = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    # "roll": 0.0,
    # "pitch": 0.0,
    "yaw": 0.0,
    "servo0": 0.0,
    "servo1": 0.0,
    # "state":0,
}


#手柄的状态
KEYBOARD_STATE_INIT = {
    "w":0,
    "a":0,
    "s":0,
    "d":0,
    "q":0,
    "e":0,
    "i":0,
    "k":0,
    "j":0,
}

#手柄的控制曲线
def controller_curve(input):
    if input < 0 : 
        input = - input
        return - input**3
    return input**3


#通过键盘从而来实现运动控制的进行
class Monitor():
    def __init__(self,mode):
        
        self.controller = CONTROLLER_INIT    #控制器的值
        self.keyboard_state = KEYBOARD_STATE_INIT  #手柄现在的状态赋值到私有变量当中
        self.mode = mode  
        self.stop = False

        keyboard.hook(lambda keyboard_event: self.keyboard_callback(keyboard_event)) #我们利用 keyboard.hook实现回调函数的用法，自动执行这个回调函数的，这里还有了lamba匿名函数的用法
        
    def keyboard_callback(self, keyboard_event):
        if keyboard_event.event_type == "up":      #按键松开还是按下
            self.keyboard_state[keyboard_event.name] = 0
            if keyboard_event.name == "j":
                if self.controller["servo0"] == 0:
                    self.controller["servo0"] = 100.0
                else :
                    self.controller["servo0"] = 0.0    
            elif keyboard_event.name == "p":
                self.mode = 3
                time.sleep(0.04)
                self.stop = True
        elif keyboard_event.event_type == "down":
            self.keyboard_state[keyboard_event.name] = 1
            if keyboard_event.name == "1":
                self.controller["state"] = 0
            elif keyboard_event.name == "2":
                self.controller["state"] = 1
            elif keyboard_event.name == "3":
                self.controller["state"] = 2
            elif keyboard_event.name == "4":
                self.controller["state"] = 3
            
    def controller_callback(self):
        self.controller["y"] = 100.0 * (self.keyboard_state["w"] - self.keyboard_state["s"])
        self.controller["x"] = 100.0 * (self.keyboard_state["a"] - self.keyboard_state["d"])
        self.controller["z"] = 100.0 * (self.keyboard_state["k"] - self.keyboard_state["i"])
        self.controller["yaw"] = 100.0 * (self.keyboard_state["e"] - self.keyboard_state["q"])





def write_shared_json(data):
    """原子化写入JSON文件"""
    try:
        # 1. 先将数据写入临时文件
        with open(r"temp.json", "w", encoding="utf-8") as f:
            portalocker.lock(f, portalocker.LOCK_EX)  # 独占锁
            json.dump(data, f)
            portalocker.unlock(f)
        
        # 2. 原子替换文件（避免读取到半成品）
        os.replace(r"temp.json", r"shared.json")
    
    except Exception as e:
        pass
        # print(f"写入JSON文件失败: {str(e)}")

#将类对象作为一个参数传入了的，这里的pygame模板抽象了手柄的一些相关的驱动
mild_mode = True
def joy_controller_callback(monitor):  
    pygame.init() #功能化初始化调用的一个设置,
    pygame.joystick.init()#初始化交互的函数
    done=False
    servo0=[0.99, 0.2]#舵机最大开启角度，最大关闭及角度
    global mild_mode

    while not done:
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop
        joystick_count = pygame.joystick.get_count()  #临时设置某些组合键为被按下状态
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)#获取操作轴的一些相关信息
            joystick.init()
            if monitor.mode == 0:
                joystick_axis = joystick.get_axis(0),joystick.get_axis(1),joystick.get_axis(2),joystick.get_axis(3)
                
                if abs(joystick_axis[0]) >= 0.05 :
                    monitor.controller["yaw"] = (400 >> mild_mode) * controller_curve(joystick_axis[0])  # 旋转
                else :
                    monitor.controller["yaw"] = 0.0  #monitor.controller["x"] = 0.0  # 左右

                if abs(joystick_axis[1]) >= 0.05 :
                    monitor.controller["y"] = (-6000 >> mild_mode) * controller_curve(joystick_axis[1])  # 前后
                else :
                    monitor.controller["y"] = 0.0  # 前后

                if abs(joystick_axis[2]) >= 0.05 :
                    monitor.controller["x"] = (-3000 >> mild_mode) * controller_curve(joystick_axis[2])  # 左右
                else : 
                    monitor.controller["x"] = 0.0  # 左右 

                if abs(joystick_axis[3]) >= 0.05 :
                    monitor.controller["z"] = (-3000 >> mild_mode) * controller_curve(joystick_axis[3]) # 上下
                else : 
                    monitor.controller["z"] = 0.0 # 上下  

                # if(joystick.get_axis(4) != -1):#(左扳机)
                #     monitor.controller["servo0"] =0.70
                #     #pass

                if(joystick.get_axis(5) != -1):#(右扳机)
                    monitor.controller["servo0"] = (servo0[0]-servo0[1])*(1- (joystick.get_axis(5)+1)/2)+servo0[1] # 舵机
                    joystick.rumble((joystick.get_axis(5)+1)/2,(joystick.get_axis(5)+1)/2,5)#
                    pass

                elif(joystick.get_button(4)):#左肩键
                    monitor.controller["servo0"] = servo0[0]#（开）
                    joystick.rumble(1,1, 5)
                elif(joystick.get_button(5)):#右肩键
                    monitor.controller["servo0"] = servo0[1]#（关）
                    joystick.rumble(1,1, 5)
                
                #if(joystick.get_button(2)):#按下正方形
                    #monitor.controller["servo0"] =0.10

                
                if(joystick.get_button(0)):  #按下按钮A
                    mild_mode = False
                    joystick.rumble(1,1, 5)
                
                if(joystick.get_button(1)):
                    mild_mode = True
                    joystick.rumble(1,1, 5)
                '''if(joystick.get_button(2)):  #按下按钮X
                    monitor.controller["pitch"] = -300
                if(joystick.get_button(3)):
                    monitor.controller["pitch"] = 0 
                '''

                #     # monitor.controller["servo1"] = 0.5 * (joystick.get_axis(4) + 1 ) # 舵机
                #     monitor.controller["servo1"] = 0
                #     pass
                # elif(joystick.get_button(B)):
                #     monitor.controller["servo1"] = 0.86
                # # elif(joystick.get_button(10)):
                # #     monitor.controller["servo1"] = 0.1                    


if __name__ == "__main__":
    joy_data_count = 0  # 初始化计数器
    
    # 串口参数
    
    # serial_port = 'COM15'  # 根据实际情况更改串口号
    # baud_rate = 115200  # 设置波特率
    # ser = serial.Serial(serial_port, baud_rate, timeout=1)  # 打开串口连接

    # 加载参数
    host = "192.168.0.10"
    port = "5000"
    mode = "0"
    '''
    host = input("主机地址?\r\n")
    port = input("主机端口?\r\n")
    mode = input("控制器?\r\n手柄[0]     键盘[1]\r\n")
    '''
    monitor = Monitor(int(mode))

    server_address = (str(host), int(port))
    hw_controller = HardwareController(server_address)


    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    #socket通信，使得局域网或者有路由器交换机的网络结构能够进行通信
    client_socket.bind(('', 1000))      # 绑定本地端口

    localdata_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 

    print("连接到服务器: %s:%s" % server_address)

    # 创建手柄监听线程
    thread_joy_controller = threading.Thread(target=joy_controller_callback,args=(monitor,))
    
    # 启动线程
    thread_joy_controller.start()
    running=True
    # 启动时发送5次推力参数
    for _ in range(10):
        hw_controller.hwinit(client_socket)
        time.sleep(0.05)  # 控制发送频率
    while running:

        if monitor.stop == True:
            mode = input("控制器?\r\n手柄[0]     键盘[1]\r\n")
            monitor.mode = int(mode)
            monitor.stop = False

        time.sleep(0.05)
        # time.sleep(2)
        # 每发送10次摇杆数据，发送一次推力参数
        if joy_data_count >= 100:
            hw_controller.hwinit(client_socket)
            joy_data_count = 0  # 重置计数器
        else:
            joy_data_count += 1  # 增加计数器

        msg = json.dumps(monitor.controller) # 使用json传输数据，这里是将python的字典格式转为json字符串的形式的操作
        print(msg)
        client_socket.sendto((msg + '\n').encode(), server_address)
        # ser.write((msg + '\n').encode())  # 向串口发送 JSON + 换行符
        # # 添加计数器逻辑
        # for _ in range(3):
        #     hw_controller.hwinit(client_socket)
        #     time.sleep(0.05)  # 控制发送频率
        localdata = monitor.controller.copy()
        localdata["mode"] = mild_mode
        
        write_shared_json(localdata)
        
        if monitor.mode == 1:
            monitor.controller_callback()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                sys.exit()


