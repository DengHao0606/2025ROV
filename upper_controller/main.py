import cv2
import subprocess
import numpy as np
import pygame
from pygame.locals import *
from configparser import ConfigParser
import json
import time
import socket
import threading
import os

# 读取摄像头配置
config = ConfigParser()




# 在这改配置文件路径！！！
config.read('config_hailing.ini')




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
RTSP_URL = "rtsp://"+config["camera"].get("username")+":"+config["camera"].get("password")+"@"+config["camera"].get("host")+":554/stream0"
base_width, base_height = config["camera"].getint("width"), config["camera"].getint("height")

# 从 curve.json 加载电机参数
MOTOR_PARAMS = {}
try:
    # 使用绝对路径确保文件位置正确
    json_path = os.path.join(os.path.dirname(__file__), config["curve"].get("location"))
    with open(json_path, 'r') as f:
        MOTOR_PARAMS = json.load(f)
        print(f"成功从 curve.json 加载电机参数")
        
    # 验证参数完整性
    required_keys = ["np_mid", "np_ini", "pp_ini", "pp_mid", "nt_end", "nt_mid", "pt_mid", "pt_end"]
    for motor, params in MOTOR_PARAMS.items():
        if not all(key in params for key in required_keys):
            raise ValueError(f"{motor} 缺少必要参数")
            
except (FileNotFoundError, json.JSONDecodeError, ValueError) as e:
    print(f"加载 curve.json 失败: {e}, 使用默认参数")
    # 保留原始默认参数作为回退
    MOTOR_PARAMS = MOTOR_PARAMS = {
    "m0": {"num": 0, "np_mid": 2717.21, "np_ini": 2921.03, "pp_ini": 3066.62, "pp_mid": 3212.21, "nt_end": -931.92, "nt_mid": -137.17, "pt_mid": 165.37, "pt_end": 1329.89},
    "m1": {"num": 1, "np_mid": 2717.21, "np_ini": 2921.03, "pp_ini": 3066.62, "pp_mid": 3212.21, "nt_end": -931.92, "nt_mid": -137.17, "pt_mid": 165.37, "pt_end": 1329.89},
    "m2": {"num": 2, "np_mid": 2717.21, "np_ini": 2921.03, "pp_ini": 3066.62, "pp_mid": 3212.21, "nt_end": -931.92, "nt_mid": -137.17, "pt_mid": 165.37, "pt_end": 1329.89},
    "m3": {"num": 3, "np_mid": 2717.21, "np_ini": 2921.03, "pp_ini": 3066.62, "pp_mid": 3212.21, "nt_end": -931.92, "nt_mid": -137.17, "pt_mid": 165.37, "pt_end": 1329.89},
    "m4": {"num": 4, "np_mid": 2717.21, "np_ini": 2921.03, "pp_ini": 3066.62, "pp_mid": 3212.21, "nt_end": -931.92, "nt_mid": -137.17, "pt_mid": 165.37, "pt_end": 1329.89},
    "m5": {"num": 5, "np_mid": 2717.21, "np_ini": 2921.03, "pp_ini": 3066.62, "pp_mid": 3212.21, "nt_end": -931.92, "nt_mid": -137.17, "pt_mid": 165.37, "pt_end": 1329.89}
}

# 控制器初始状态
CONTROLLER_INIT = {
    "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, 
    "servo0": config["servo"].getfloat("open"),  # 舵机初始值
}

# 手柄控制曲线
def controller_curve(input):
    return input**3 if input >= 0 else -((-input)**3)

# 硬件控制类
class HardwareController:
    def __init__(self, server_address):
        self.server_address = server_address
        
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
            time.sleep(0.05)

# 控制监控类
class ControllerMonitor:
    def __init__(self):
        self.controller = CONTROLLER_INIT.copy()
        self.depth = 0.0      # 新增深度数据
        self.temperature = 0.0  # 新增温度数据

# 视频处理线程
class VideoThread(threading.Thread):
    def __init__(self, process, screen, base_width, base_height):
        super().__init__()
        self.process = process
        self.screen = screen
        self.base_width = base_width
        self.base_height = base_height
        self.running = True
        self.frame_queue = []
        self.max_queue_size = 2  # 保持队列较小以减少延迟
        
    def run(self):
        frame_size = self.base_width * self.base_height * 3
        while self.running:
            try:
                raw_frame = self.process.stdout.read(frame_size)
                if len(raw_frame) == frame_size:
                    frame = np.frombuffer(raw_frame, np.uint8).reshape((self.base_height, self.base_width, 3))
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    
                    # 保持队列较小
                    if len(self.frame_queue) >= self.max_queue_size:
                        self.frame_queue.pop(0)
                    self.frame_queue.append(frame_rgb)
            except Exception as e:
                print(f"视频读取错误: {e}")
    
    def stop(self):
        self.running = False
        
    def get_latest_frame(self):
        """获取最新的帧，如果队列为空返回None"""
        if self.frame_queue:
            return self.frame_queue[-1]
        return None

# 主函数
def main():
    # 初始化Pygame
    pygame.init()
    screen = pygame.display.set_mode((config["interface"].getint("width"), config["interface"].getint("height")), pygame.RESIZABLE)
    pygame.display.set_caption("ROV control upper computer software")
    try:
        # 获取图标文件路径（假设图标文件名为 icon.ico）
        icon_path = os.path.join(os.path.dirname(__file__), 'EV.jpg')
        icon_surface = pygame.image.load(icon_path)
        pygame.display.set_icon(icon_surface)
        print(f"成功设置窗口图标: {icon_path}")
    except Exception as e:
        print(f"设置窗口图标失败: {e}")
    
    # 初始化字体用于显示控制器数据
    try:
        font = pygame.font.SysFont(config["interface"].get("font"), config["interface"].getint("font_size"))
    except:
        font = pygame.font.Font(None, config["interface"].getint("font_size"))  # 使用默认字体
    
    # 初始化手柄
    pygame.joystick.init()
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"已连接手柄: {joystick.get_name()}")
    
    # 启动FFmpeg进程 - 使用低延迟优化参数
    command = [
        'ffmpeg',
        '-rtsp_transport', 'tcp',         # 强制使用TCP传输
        '-fflags', 'nobuffer',            # 禁用缓冲区
        '-flags', 'low_delay',            # 低延迟标志
        '-i', RTSP_URL,
        '-f', 'image2pipe',
        '-pix_fmt', 'bgr24',
        '-vcodec', 'rawvideo',
        '-an', '-sn',                     # 禁用音频和字幕
        '-probesize', '32',               # 减少探测大小
        '-analyzeduration', '0',          # 立即开始解码
        '-tune', 'zerolatency',           # 零延迟调整
        '-preset', 'ultrafast',           # 使用最快的编码预设
        '-threads', '1',                  # 使用单线程减少上下文切换
        '-'
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, 
                              bufsize=base_width * base_height * 3 * 2)
    
    # 创建视频处理线程
    video_thread = VideoThread(process, screen, base_width, base_height)
    video_thread.start()
    
    # 初始化网络连接
    host = config["serial"].get("host")
    port = config["serial"].getint("remote_port")
    
    monitor = ControllerMonitor()
    server_address = (host, port)
    hw_controller = HardwareController(server_address)
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.setblocking(False)
    client_socket.bind(('', config["serial"].getint("local_port")))  # 绑定本地端口
    
    # 初始化电机参数
    for _ in range(10):
        hw_controller.hwinit(client_socket)
        time.sleep(0.05)
    
    # 主循环变量
    running = True
    joy_data_count = 0
    speed_mode = [{"name": "mild mode", "rate": 0.5, "color": "#00FF00"}, {"name": "wild mode", "rate": 1, "color": "#FF0000"}]  # 速度模式
    lock_mode = [{"name": "unlock", "value": 0, "color": "#00FF00"}, {"name": "lock", "value": 1, "color": "#FF0000"}]  # 锁定模式
    loop_mode = [{"name": "open loop", "value": 0, "color": "#00FF00"}, {"name": "closed loop", "value": 1, "color": "#FF0000"}, {"name": "half closed loop", "value": 2, "color": "#0000FF"}]  # 闭环模式
    speed_mode_ptr = 0  # 当前速度模式指针
    lock_mode_ptr = 0  # 当前锁定模式指针
    loop_mode_ptr = 0  # 当前闭环模式指针
    
    servo0 = [config["servo"].getfloat("open"), config["servo"].getfloat("close"), config["servo"].getfloat("mid")]  # 舵机角度范围
    
    clock = pygame.time.Clock()
    
    # 用于帧率统计
    frame_count = 0
    start_time = time.time()
    
    # 显示参数
    padding = config["interface"].getint("padding")  # 内边距
    
    def draw_text(text, x, y, color=(255, 255, 255)):
                """在指定位置绘制文本"""
                # 渲染黑色轮廓
                offsets = [(-2, -2), (-2, 0), (-2, 2),
                          (0, -2),           (0, 2),
                          (2, -2),  (2, 0),  (2, 2)]
                
                for dx, dy in offsets:
                    outline_surface = font.render(text, True, (0, 0, 0))  # 黑色轮廓
                    screen.blit(outline_surface, (x + dx, y + dy))
                    
                text_surface = font.render(text, True, color)
                screen.blit(text_surface, (x, y))
    
    num_buttons = config["joystick"].getint("buttons")  # 获取手柄按钮数量
    LONG_TIME = config["joystick"].getint("long")  # 长按时间
    DOUBLE_TIME = config["joystick"].getint("double")
    buttons = [{"new": False, "old": False, "edge": False,"down": False, "up": False, "long": False, "short": False, "double": False, "down_time": 0.0, "up_time": 0.0} for i in range(num_buttons)]  # 按钮状态

    while running:
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                running = False  # Flag that we are done so we exit this loop
        joystick_count = pygame.joystick.get_count()  #临时设置某些组合键为被按下状态
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)#获取操作轴的一些相关信息
            joystick.init()
            
            # 更新按钮状态
            for i in range(num_buttons):
                buttons[i]["old"]  = buttons[i]["new"]
                buttons[i]["new"]  = joystick.get_button(i)
                buttons[i]["edge"] = buttons[i]["new"] ^ buttons[i]["old"]
                buttons[i]["down"] = buttons[i]["edge"] & buttons[i]["new"]
                buttons[i]["up"]   = buttons[i]["edge"] & buttons[i]["old"]
                buttons[i]["long"]   = False
                buttons[i]["short"]  = False
                buttons[i]["double"] = False
                
                if buttons[i]["down"]:
                    buttons[i]["down_time"] = 0
                else:
                    buttons[i]["down_time"] += 1
                if buttons[i]["up"]:
                    buttons[i]["up_time"] = 0
                else:
                    buttons[i]["up_time"] += 1
                
                if buttons[i]["down_time"] >= LONG_TIME and buttons[i]["new"]:  # 长按
                    buttons[i]["long"] = True
                    buttons[i]["down_time"] = 0  
                if buttons[i]["down_time"] < LONG_TIME and buttons[i]["up"]:
                    buttons[i]["short"] = True
                    if buttons[i]["long"]:
                        buttons[i]["short"] = False  # 如果是长按则不算短按
                if buttons[i]["up_time"] <= DOUBLE_TIME and buttons[i]["down"]:
                    buttons[i]["double"] = True
            
            if abs(joystick.get_axis(config["yaw"].getint("axis"))) >= config["yaw"].getfloat("deadzone") :
                monitor.controller["yaw"] = (config["yaw"].getfloat("max") * speed_mode[speed_mode_ptr]["rate"]) * controller_curve(joystick.get_axis(config["yaw"].getint("axis")))  # 旋转
            else :
                monitor.controller["yaw"] = 0.0  #monitor.controller["x"] = 0.0  # 左右

            if abs(joystick.get_axis(config["y"].getint("axis"))) >= config["y"].getfloat("deadzone") :
                monitor.controller["y"] = (config["y"].getfloat("max") * speed_mode[speed_mode_ptr]["rate"]) * controller_curve(joystick.get_axis(config["y"].getint("axis")))  # 前后
            else :
                monitor.controller["y"] = 0.0  # 前后

            if abs(joystick.get_axis(config["x"].getint("axis"))) >= config["x"].getfloat("deadzone") :
                monitor.controller["x"] = (config["x"].getfloat("max") * speed_mode[speed_mode_ptr]["rate"]) * controller_curve(joystick.get_axis(config["x"].getint("axis")))  # 左右
            else : 
                monitor.controller["x"] = 0.0  # 左右 

            if abs(joystick.get_axis(config["z"].getint("axis"))) >= config["z"].getfloat("deadzone") :
                monitor.controller["z"] = (config["z"].getfloat("max") * speed_mode[speed_mode_ptr]["rate"]) * controller_curve(joystick.get_axis(config["z"].getint("axis"))) # 上下
            else : 
                monitor.controller["z"] = 0.0 # 上下  

            # if(joystick.get_axis(4) != -1):#(左扳机)
            #     monitor.controller["servo0"] =0.70
            #     #pass
            if lock_mode[lock_mode_ptr]["value"] == 0:  # 未锁定
                if(joystick.get_axis(5) > config["servo"].getfloat("deadzone")):#(右扳机)
                    monitor.controller["servo0"] = (servo0[0]-servo0[1])*(1- (joystick.get_axis(5)+1)/2)+servo0[1] # 舵机
                    joystick.rumble((joystick.get_axis(5)+1)/2,(joystick.get_axis(5)+1)/2,5) 
                    pass

                elif(buttons[config["servo"].getint("open_button")][config["servo"].get("open_trig")]):#左肩键
                    monitor.controller["servo0"] = servo0[0]#（开）
                    joystick.rumble(1, 1, 5)
                elif(buttons[config["servo"].getint("close_button")][config["servo"].get("close_trig")]):#右肩键
                    monitor.controller["servo0"] = servo0[1]#（关）
                    joystick.rumble(1, 1, 5)
                    
                elif(buttons[config["servo"].getint("mid_button")][config["servo"].get("mid_trig")]):  #按下按钮Y
                    monitor.controller["servo0"] = servo0[2]
                    joystick.rumble(1,1, 5)
            
            #if(joystick.get_button(2)):#按下正方形
                #monitor.controller["servo0"] =0.10

            
            if(buttons[config["speed_mode"].getint("button")][config["speed_mode"].get("trig")]):  #按下按钮A
                speed_mode_ptr = (speed_mode_ptr + 1) % len(speed_mode)  # 切换模式
                joystick.rumble(1,1, 5)
                
            if(buttons[config["lock_mode"].getint("button")][config["lock_mode"].get("trig")]):  #按下按钮B
                lock_mode_ptr = (lock_mode_ptr + 1) % len(lock_mode)  # 切换模式
                joystick.rumble(1,1, 5)
            
            if(buttons[config["loop_mode"].getint("button")][config["loop_mode"].get("trig")]):  #按下按钮X
                loop_mode_ptr = (loop_mode_ptr + 1) % len(loop_mode)  # 切换模式
                joystick.rumble(1,1, 5)
                
            
        # 发送控制数据
        if joy_data_count >= 100:
            hw_controller.hwinit(client_socket)
            joy_data_count = 0
        else:
            joy_data_count += 1
            
        # 发送控制器状态
        msg = json.dumps(monitor.controller)
        client_socket.sendto((msg + '\n').encode(), server_address)
        try:
            data, addr = client_socket.recvfrom(1024)
            if data:
                try:
                    sensor_data = json.loads(data.decode())
                    monitor.depth = sensor_data.get("depth", 0.0)
                    monitor.temperature = sensor_data.get("temperature", 0.0)
                except json.JSONDecodeError:
                    # print("接收到的数据格式错误")
                    pass
        except BlockingIOError:
            pass  # 无数据可接收
        # 获取并显示最新的视频帧
        frame_rgb = video_thread.get_latest_frame()
        
        # 准备右上角显示的数据 - 移到条件外部确保始终定义
        right_data_lines = [
            f"Depth: {monitor.depth:.3f} m",
            f"Temp: {monitor.temperature:.2f} °C"
        ]
        
        screen_width, screen_height = screen.get_size()
        if frame_rgb is not None:
            frame_surface = pygame.surfarray.make_surface(frame_rgb.swapaxes(0, 1))
            
            # 缩放并显示
            scaled_surface = pygame.transform.scale(frame_surface, (screen_width, screen_height))
            screen.blit(scaled_surface, (0, 0))
            
            # 准备要显示的数据
            data_lines = [
                f"X: {monitor.controller['x']:.1f}",
                f"Y: {monitor.controller['y']:.1f}",
                f"Z: {monitor.controller['z']:.1f}",
                f"Yaw: {monitor.controller['yaw']:.1f}",
                f"Servo: {monitor.controller['servo0']:.2f}"
            ]
            
            # 渲染并显示带轮廓的文本
            y_offset = padding
            for line in data_lines:
                
                draw_text(line, padding, config["interface"].getint("y_h") + y_offset, color=(255, 255, 255))
                
                y_offset += config["interface"].getint("y_offset")  # 行间距
            # 在左上角显示控制模式
            draw_text(f"{speed_mode[speed_mode_ptr]['name']}", padding, config["interface"].getint("y_h") + y_offset, color=pygame.Color(speed_mode[speed_mode_ptr]["color"]))
            y_offset += config["interface"].getint("y_offset") 
            draw_text(f"{lock_mode[lock_mode_ptr]['name']}", padding, config["interface"].getint("y_h") + y_offset, color=pygame.Color(lock_mode[lock_mode_ptr]["color"]))
            y_offset += config["interface"].getint("y_offset")
            draw_text(f"{loop_mode[loop_mode_ptr]['name']}", padding, config["interface"].getint("y_h") + y_offset, color=pygame.Color(loop_mode[loop_mode_ptr]["color"]))
            
            # 在右上角显示带轮廓的文本 - 确保这段代码始终执行
            y_offset = padding
            for line in right_data_lines:
                # 计算文本宽度以确定右上角位置
                text_width = font.size(line)[0]
                x_pos = screen_width - text_width - padding
                
                draw_text(line, x_pos, config["interface"].getint("y_h") + y_offset, color=(255, 255, 255))
                
                y_offset += config["interface"].getint("y_offset")  # 行间距
        
        pygame.display.flip()
        
        # 帧率统计
        frame_count += 1
        if frame_count % 30 == 0:  # 每30帧输出一次帧率
            elapsed = time.time() - start_time
            fps = frame_count / elapsed
            # print(f"当前帧率: {fps:.2f} FPS")
            frame_count = 0
            start_time = time.time()
        
        # 控制主循环频率
        clock.tick(config["joystick"].getint("tick"))  # 设置更高的循环频率
    
    # 清理资源
    video_thread.stop()
    video_thread.join()
    process.terminate()
    pygame.quit()
    print("程序已退出")

if __name__ == "__main__":
    main()
    
    
    
    
'''__||_____||__
   __||_____||__
   ___\\___//___
   _===========_
   _____|||_____
   _____|||_____
   ______|______
   ___防伪专用___'''