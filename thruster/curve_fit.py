import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as sp
from matplotlib.widgets import Button

# 全局常量
pwm_mid = 3000
pwm_half_p = 400
pwm_half_n = 600

# 定义折线函数
def f(x, np_mid, np_ini, pp_ini, pp_mid, nt_end, nt_mid, pt_mid, pt_end):
    def line(t, start, end):
        denominator = end[0] - start[0]
        if abs(denominator) < 1e-10:
            return (start[1] + end[1]) / 2.0
        return (end[1] - start[1]) / denominator * (t - start[0]) + start[1]
    
    result = np.zeros_like(x)
    
    mask1 = (x < nt_mid)
    mask2 = (x >= nt_mid) & (x < 0)
    mask3 = (x >= 0) & (x < pt_mid)
    mask4 = (x >= pt_mid)
    
    if np.any(mask1):
        result[mask1] = line(x[mask1], [nt_end, pwm_mid-pwm_half_n], [nt_mid, np_mid])
    if np.any(mask2):
        result[mask2] = line(x[mask2], [nt_mid, np_mid], [0, np_ini])
    if np.any(mask3):
        result[mask3] = line(x[mask3], [0, pp_ini], [pt_mid, pp_mid])
    if np.any(mask4):
        result[mask4] = line(x[mask4], [pt_mid, pp_mid], [pt_end, pwm_mid+pwm_half_p])
    
    return result

# 读取数据
df = pd.DataFrame(pd.read_excel(r"E:\document\EV\thruster\thruster.xlsx", sheet_name="Sheet1"))

# 设置初始关键点 (修正：添加括号()并固定部分点)
initial_points = {
    'nt_end': (df["force"].min(), pwm_mid - pwm_half_n),  # y固定
    'nt_mid': (df["force"].min()/2, (df["PWM_P"].min() + pwm_mid)/2),  # 修正：添加括号()
    'np_ini': (0, 2950),  # x固定为0
    'pp_ini': (0, 3050),  # x固定为0
    'pt_mid': (df["force"].max()/2, (df["PWM_P"].max() + pwm_mid)/2),  # 修正：添加括号()
    'pt_end': (df["force"].max(), pwm_mid + pwm_half_p)  # y固定
}

# 创建图形和坐标轴
fig, ax = plt.subplots(figsize=(12, 8))
plt.subplots_adjust(bottom=0.2)

# 绘制原始数据
ax.plot(df["force"], df["PWM_P"], 'o', label='Original data')

# 绘制初始关键点
point_artists = {}
for name, (x, y) in initial_points.items():
    artist = ax.plot(x, y, 'ro', markersize=8, picker=5)[0]
    point_artists[artist] = name
    ax.text(x, y, name, fontsize=9, ha='right', va='bottom')

# 设置坐标轴标签和网格
ax.set_xlabel("force (g)")
ax.set_ylabel("PWM")
ax.axhline(0, color='black', linewidth=0.5)
ax.axvline(0, color='black', linewidth=0.5)
ax.grid(True)
ax.set_title("Drag the key points to adjust the initial position (click the fit button after completion)")

# 添加拟合按钮
ax_button = plt.axes([0.4, 0.05, 0.2, 0.075])
button = Button(ax_button, 'fit curve')

# 当前拖拽的点
current_point = None

# 存储上一次的拟合曲线
last_fit_curve = None

# 拖拽事件处理
def on_pick(event):
    global current_point
    if event.artist in point_artists:
        current_point = event.artist
        fig.canvas.draw()

def on_motion(event):
    if current_point is not None and event.inaxes == ax:
        name = point_artists[current_point]
        x, y = event.xdata, event.ydata
        
        # 对于固定坐标的点，限制移动
        if name == 'np_ini' or name == 'pp_ini':
            # 固定x为0，只允许y移动
            current_point.set_data([0], [y])
        elif name == 'nt_end' or name == 'pt_end':
            # 固定y值，只允许x移动
            if name == 'nt_end':
                fixed_y = pwm_mid - pwm_half_n
            else:  # pt_end
                fixed_y = pwm_mid + pwm_half_p
            current_point.set_data([x], [fixed_y])
        else:
            # 其他点自由移动
            current_point.set_data([x], [y])
        
        # 更新文本位置
        for text in ax.texts:
            if text.get_text() == name:
                if name in ['np_ini', 'pp_ini']:
                    text.set_position((0, y))
                elif name in ['nt_end', 'pt_end']:
                    text.set_position((x, fixed_y))
                else:
                    text.set_position((x, y))
        
        fig.canvas.draw()

def on_release(event):
    global current_point
    current_point = None

# 拟合按钮点击事件
def on_button_clicked(event):
    global last_fit_curve
    
    # 获取当前点位置
    points = {}
    for artist, name in point_artists.items():
        x, y = artist.get_data()
        points[name] = (x[0], y[0])
    
    # 提取参数
    p0 = [
        points['nt_mid'][1],  # np_mid
        points['np_ini'][1],  # np_ini (y值)
        points['pp_ini'][1],  # pp_ini (y值)
        points['pt_mid'][1],  # pp_mid
        points['nt_end'][0],  # nt_end (x值)
        points['nt_mid'][0],  # nt_mid (x值)
        points['pt_mid'][0],  # pt_mid (x值)
        points['pt_end'][0]   # pt_end (x值)
    ]
    
    print(f"使用初始参数: {p0}")
    
    # 执行曲线拟合
    try:
        # 注意：这里需要确保边界约束中的值是数值，而不是方法对象
        # 例如：df["force"].min() 而不是 df["force"].min
        # 但为了安全起见，我们先注释掉边界约束
        # bounds = (
        #     [2000, 2000, pwm_mid, pwm_mid, df["force"].min(), df["force"].min(), 0, 0],
        #     [pwm_mid, pwm_mid, 4000, 4000, 0, 0, df["force"].max(), df["force"].max]
        # )
        
        p, pcov = sp.curve_fit(
            f, 
            df["force"], 
            df["PWM_P"], 
            p0=p0,
            # bounds=bounds,
            maxfev=100000
        )
    except RuntimeError as e:
        print(f"拟合失败: {e}")
        p = p0
    
    np_mid, np_ini, pp_ini, pp_mid, nt_end, nt_mid, pt_mid, pt_end = p
    
    # 删除上一次的拟合曲线
    if last_fit_curve is not None:
        last_fit_curve.remove()
    
    # 绘制新的拟合曲线
    x_fit = np.linspace(df["force"].min(), df["force"].max(), 1000)
    fit_curve, = ax.plot(x_fit, f(x_fit, np_mid, np_ini, pp_ini, pp_mid, nt_end, nt_mid, pt_mid, pt_end), 
            'g-', label='fitted curve', linewidth=2)
    
    # 存储当前拟合曲线的引用
    last_fit_curve = fit_curve
    
    # 更新图例
    # 获取当前所有图例句柄和标签
    handles, labels = ax.get_legend_handles_labels()
    
    # 移除旧的拟合曲线图例（如果存在）
    if 'fitted curve' in labels:
        index = labels.index('fitted curve')
        del handles[index]
        del labels[index]
    
    # 添加新的拟合曲线图例
    handles.append(fit_curve)
    labels.append('fitted curve')
    
    # 重新设置图例
    ax.legend(handles, labels)
    
    # 显示拟合参数
    params_str = f"""
    拟合参数结果:
    np_mid = {np_mid:.2f}
    np_ini = {np_ini:.2f}
    pp_ini = {pp_ini:.2f}
    pp_mid = {pp_mid:.2f}
    nt_end = {nt_end:.2f}
    nt_mid = {nt_mid:.2f}
    pt_mid = {pt_mid:.2f}
    pt_end = {pt_end:.2f}
    """
    print(params_str)
    
    print(f".pwm = {{{np_mid:.2f}f, {np_ini:.2f}f, {pp_ini:.2f}f, {pp_mid:.2f}f}}, .thrust = {{{nt_end:.2f}f, {nt_mid:.2f}f, {pt_mid:.2f}f, {pt_end:.2f}f}},")
    print(f"{{\"cmd\": \"thrust_init\", \"motor\": NUM, \"np_mid\": {np_mid:.2f}, \"np_ini\": {np_ini:.2f}, \"pp_ini\": {pp_ini:.2f}, \"pp_mid\": {pp_mid:.2f}, \"nt_end\": {nt_end:.2f}, \"nt_mid\": {nt_mid:.2f}, \"pt_mid\": {pt_mid:.2f}, \"pt_end\": {pt_end:.2f}}}")
    
    fig.canvas.draw()

# 连接事件
fig.canvas.mpl_connect('pick_event', on_pick)
fig.canvas.mpl_connect('motion_notify_event', on_motion)
fig.canvas.mpl_connect('button_release_event', on_release)
button.on_clicked(on_button_clicked)

plt.show()