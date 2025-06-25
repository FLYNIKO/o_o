

IP = '192.168.100.10' # 机械臂IP地址
PORT = 7003 # 机械臂端口号

"""传感器防撞阈值，若阈值为0则不开启防撞"""
ANTICRASH_UP = 0
ANTICRASH_FRONT = 600 #同时用作喷涂距离
ANTICRASH_LEFT = 0
ANTICRASH_RIGHT = 0

AUTOSPEED = 0.3 # 自动喷涂速度
DEFAULT_VEL = 0.3 # 机械臂末端速度
DEFAULT_ACC = 1.2 # 机械臂末端加速度
KP = 0.005
KI = 0.0
KD = 0.0001
DEADZONE = 30 # PID死区 (mm)

PAINTDEG = 90 # 喷涂角度(圆柱)
PAINTWIDTH = 0.15 # 喷涂宽度(圆柱)

INIT_POS = [0.8, 0.2, 1.2, -1.57, 0.0, -1.57] # 初始位置
SERV_POS = [1.0, -0.2, 0.0, -1.57, 0.0, -1.57] # 维修位置