import time
import rospy
import threading

from DucoCobot import DucoCobot
from s21c_receive_data.msg import STP23
from key_input_pkg.msg import KeyInput
from CylinderPaint_duco import CylinderAutoPaint
from collections import deque
from config import *

class KeyInputStruct:
    def __init__(self, x0=0, x1=0, y0=0, y1=0, z0=0, z1=0,
                 init=0, serv=0, multi=0, start=0,
                 rx0=0, rx1=0, ry0=0, ry1=0, rz0=0, rz1=0):
        self.x0 = x0
        self.x1 = x1
        self.y0 = y0
        self.y1 = y1
        self.z0 = z0
        self.z1 = z1
        self.init = init
        self.serv = serv
        self.multi = multi
        self.start = start
        self.rx0 = rx0
        self.rx1 = rx1
        self.ry0 = ry0
        self.ry1 = ry1
        self.rz0 = rz0
        self.rz1 = rz1

class SimplePID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target, current, dt):
        error = current - target
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class system_control:
    def __init__(self, duco_cobot, app=None):
        self.ip = IP
        self.duco_cobot = duco_cobot
        self.app = app
        self.auto_vel = AUTOSPEED # 自动喷涂速度
        self.vel = DEFAULT_VEL # 机械臂末端速度
        self.acc = DEFAULT_ACC # 机械臂末端加速度
        self.aj_pos = [] # 当前关节角度
        self.tcp_pos = [] # 当前末端位姿
        self.history_pos = [] # 历史位置
        self.sysrun = True
        self.autopaint_flag = True
        self.init_pos = INIT_POS # 初始位置
        self.serv_pos = SERV_POS # 维修位置
        self.safe_pos = SAFE_POS # 安全位置
        self.pid = SimplePID(kp=1, ki=0.0, kd=0.2)
        self.pid_z = SimplePID(kp=KP, ki=KI, kd=KD)
        self.front_sensor_history = deque(maxlen=5) # 滤波队列

        self.theta_deg = PAINTDEG / 2  # 喷涂角度的一半
        self.distance_to_cylinder = ANTICRASH_FRONT  # 末端与圆柱表面距离
        self.painting_width = PAINTWIDTH  # 喷涂宽度

        self.latest_sensor_data = {"up": -1, "front": -1, "left_side": -1, "right_side": -1}
        self.sensor_subscriber = rospy.Subscriber('/STP23', STP23, self._sensor_callback)
        self.latest_keys = [0] * 32
        self.keys_subscriber = rospy.Subscriber('/key_input', KeyInput, self._keys_callback)

        self.emergency_stop_flag = False
        self.emergency_thread = threading.Thread(target=self.emergency_stop_thread, daemon=True)
        self.emergency_thread.start()
    
    # 急停线程
    def emergency_stop_thread(self):
        self.duco_stop = DucoCobot(self.ip, PORT)
        self.duco_stop.open()
        while self.sysrun:
            key_input = self.get_key_input()
            state = self.duco_stop.get_robot_state()
            if key_input.multi:
                print("检测到紧急停止按键，正在执行紧急停止！")
                self.autopaint_flag = False
                self.duco_stop.stop(True)                
                if key_input.start:
                    self.autopaint_flag = False
                    self.sysrun = False
                    self.duco_stop.stop(True)
                    self.duco_stop.disable(True)
                    print("terminate robot")
                    break
                elif state[0] != 6:
                    print("restart robot")
                    if state[0] == 5:
                        self.duco_stop.enable(True)
                    if state[0] == 4:
                        self.duco_stop.power_on(True)
                        self.duco_stop.enable(True)
                    self.duco_stop.switch_mode(1)          
            time.sleep(0.1)

    def get_cylinder_param(self):
        # TODO: 获取圆柱圆心坐标及圆柱半径
        cx, cy, cz = 1.8, 0.3, 1.2   # 圆心坐标
        cy_radius = 0.5               # 圆柱半径
        return cx, cy, cz, cy_radius
    
    # 读取/topic中的按键输入
    def _keys_callback(self, msg):
        self.latest_keys = list(msg.keys)
    # 前16个为按钮
    def get_key_input(self):
        keys = self.latest_keys[:16]
        keys_padded = (keys + [0]*16)[:16]
        return KeyInputStruct(*keys_padded)
    # 16个以后的为需要的信息
    def get_ctrl_msg(self):
        return self.latest_keys[16:]
    
    # 读取/topic中的传感器数据
    def _sensor_callback(self, msg):
        self.latest_sensor_data = {
            "up": msg.Distance_1,
            "front": msg.Distance_2,
            "left": msg.Distance_3,
            "right": msg.Distance_4
        }

    def get_sensor_data(self):
        return self.latest_sensor_data
    
    # 记录当前位置
    def get_replay_pos(self, tcp_pos, history_pos=None):
        history_pos.append(tcp_pos)
        print("new position: %s counter: %d" % (tcp_pos, len(history_pos)))
        time.sleep(0.5)

    # 回放历史位置
    def position_replay(self, tcp_pos, history_pos=None):
        for tcp_pos in history_pos:
            try:
                index = history_pos.index(tcp_pos) + 1
                task_id = self.duco_cobot.movel(tcp_pos, self.vel, self.acc, 0, '', '', '', False)
                print("move to no.%d position: %s" % (index, tcp_pos))
                cur_time = time.time()
                while self.duco_cobot.get_noneblock_taskstate(task_id) != 4:
                    if time.time() - cur_time > 10:
                        print("Timeout.Move to no.%d position failed." % index)
                        break
                    
                    if self.duco_cobot.get_noneblock_taskstate(task_id) == 4:
                        break
                                
            except ValueError:
                print("Position %s not found in history." % tcp_pos)
        # 回到起始位置
        if len(history_pos) > 1:
            self.duco_cobot.movel(history_pos[0], self.vel, self.acc, 0, '', '', '', False)
            print("move to no.1 position: %s" % history_pos[0])
        # 无点位可回放
        elif len(history_pos) == 0:
            print("No position in history.Press LB to record position.")
            time.sleep(0.5)

    # 自动喷涂，边走边喷
    def auto_paint_sync(self):
        self.autopaint_flag = True
        time.sleep(0.1)  # 防止和退出冲突
        self.front_sensor_history.clear()
        v2 = 0.0  # 初始化前后速度
        cur_time = time.time()
        last_time = cur_time

        while self.autopaint_flag:
            sensor_data = self.get_sensor_data()
            tcp_pos = self.duco_cobot.get_tcp_pose()
            now = time.time()
            dt = now - last_time
            last_time = now

            side_count = 0
            side_count_threshold = 5 
            while (ANTICRASH_LEFT != 0 and sensor_data["left"] < ANTICRASH_LEFT) or (ANTICRASH_RIGHT != 0 and sensor_data["right"] < ANTICRASH_RIGHT):
                v2 = self.auto_vel * 1.5
                self.duco_cobot.speedl([0, 0, -v2, 0, 0, 0], self.acc, -1, False)
                time.sleep(0.1)
                sensor_data = self.get_sensor_data()
                side_count += 1
                if side_count > side_count_threshold:
                    print("可能是个梁！")
                    default_pos = self.duco_cobot.get_tcp_pose()
                    self.duco_cobot.servoj_pose(self.safe_pos, self.vel * 1.5, self.acc, '', '', '', True)
                    time.sleep(0.1)
                    sensor_data = self.get_sensor_data()
                    # 检测是否通过梁
                    while ANTICRASH_UP != 0 and sensor_data["up"] < ANTICRASH_UP:
                        sensor_data = self.get_sensor_data()
                        time.sleep(0.05)
                    # 通过梁后，回到下降前的最后位置
                    self.duco_cobot.servoj_pose(default_pos, self.vel, self.acc, '', '', '', True)
                    break
            
            # PID with filter
            v2 = 0.0  # x轴默认速度为0
            if ANTICRASH_FRONT != 0:
                # 1. 数据滤波
                raw_front_dist = sensor_data["front"]
                if raw_front_dist > 0:  # 确保是有效读数
                    self.front_sensor_history.append(raw_front_dist)

                if len(self.front_sensor_history) > 0:
                    filtered_front_dist = sum(self.front_sensor_history) / len(self.front_sensor_history)

                    # 2. 控制死区
                    target_dist = ANTICRASH_FRONT
                    deadband_threshold = DEADZONE  # 单位: mm, 可根据实际情况调整
                    error = filtered_front_dist - target_dist

                    # 3. PID计算 (仅在死区外)
                    if abs(error) > deadband_threshold:
                        v2 = self.pid_z.compute(target_dist, filtered_front_dist, dt)
                        # 限制最大速度
                        v2 = max(min(v2, 0.1), -0.1)  # 建议将最大调整速度也降低一些

            print("v2: %f" % v2)
            self.duco_cobot.speedl([0, 0, v2, 0, 0, 0], self.acc, -1, False)

    # 自动喷涂，车辆不动机械臂动
    def auto_paint_interval(self):
        self.autopaint_flag = True
        time.sleep(0.1)  # 防止和退出冲突
        self.front_sensor_history.clear()
        v0 = self.auto_vel
        v2 = 0.0  # 初始化前后速度
        cur_time = time.time()
        last_time = cur_time

        while self.autopaint_flag:
            sensor_data = self.get_sensor_data()
            tcp_pos = self.duco_cobot.get_tcp_pose()
            now = time.time()
            dt = now - last_time
            last_time = now
            # 防撞保护
            if (ANTICRASH_LEFT != 0 and sensor_data["left"] < ANTICRASH_LEFT) or tcp_pos[1] > 1:
                print("jobs done")
                self.duco_cobot.speed_stop(True)
                break
            # PID with filter
            v2 = 0.0  # x轴默认速度为0
            if ANTICRASH_FRONT != 0:
                # 1. 数据滤波
                raw_front_dist = sensor_data["front"]
                if raw_front_dist > 0:  # 确保是有效读数
                    self.front_sensor_history.append(raw_front_dist)

                if len(self.front_sensor_history) > 0:
                    filtered_front_dist = sum(self.front_sensor_history) / len(self.front_sensor_history)

                    # 2. 控制死区
                    target_dist = ANTICRASH_FRONT
                    deadband_threshold = 10  # 单位: mm, 可根据实际情况调整
                    error = filtered_front_dist - target_dist

                    # 3. PID计算 (仅在死区外)
                    if abs(error) > deadband_threshold:
                        v2 = self.pid_z.compute(target_dist, filtered_front_dist, dt)
                        # 限制最大速度
                        v2 = max(min(v2, 0.1), -0.1)  # 建议将最大调整速度也降低一些

            print("v2: %f" % v2)
            task_id = self.duco_cobot.speedl([-v0, 0, v2, 0, 0, 0], self.acc, -1, False)

            if now - cur_time > 100:
                self.duco_cobot.speed_stop(True)
                print("Timeout.")
                break

    def run(self):
        print("waiting for robot initialization...")
        # self.duco_cobot.movej2(self.init_pos, 2*self.vel, self.acc, 0, True)
        self.duco_cobot.servoj_pose(self.init_pos, self.vel, self.acc, '', '', '', True)
        print("move to initial position: %s" % self.init_pos)
        time.sleep(1)
        
        try:
            while self.sysrun:
                key_input = self.get_key_input()
                sensor_data = self.get_sensor_data()
                self.duco_cobot.switch_mode(1)

                v0 = self.auto_vel                # arm left-/right+
                v1 = self.auto_vel                 # arm up+/down-
                v2 = self.auto_vel                 # arm forward+/backward-
                v3 = -self.auto_vel * 2           # arm head up+/down-
                v4 = self.auto_vel  * 2            # arm head left+/right-
                v5 = self.auto_vel  * 2            # arm head rotate left-/right+
                
                #自动喷涂
                if key_input.start:
                    self.auto_paint_sync()
                    # self.auto_paint_interval()

                #机械臂末端向  前
                elif key_input.x0:
                    if ANTICRASH_FRONT != 0 and sensor_data["front"] < ANTICRASH_FRONT:
                        v2 = 0
                        print("向前移动被防撞保护禁止,front传感器值:", sensor_data.get("front"))
                    self.duco_cobot.speedl([0, 0, v2, 0, 0, 0],self.acc ,-1, False)
                #机械臂末端向  后
                elif key_input.x1:
                    self.duco_cobot.speedl([0, 0, -v2, 0, 0, 0],self.acc ,-1, False)
                #机械臂末端向  右
                elif key_input.y1:
                    if ANTICRASH_RIGHT != 0 and sensor_data["right"] < ANTICRASH_RIGHT:
                        v0 = 0
                        print("向右移动被防撞保护禁止,right传感器值:", sensor_data.get("right"))
                    self.duco_cobot.speedl([v0, 0, 0, 0, 0, 0], self.acc, -1, False)
                #机械臂末端向  左
                elif key_input.y0: 
                    if ANTICRASH_LEFT != 0 and sensor_data["left"] < ANTICRASH_LEFT:
                        v0 = 0
                        print("向左移动被防撞保护禁止,left传感器值:", sensor_data.get("left"))
                    self.duco_cobot.speedl([-v0, 0, 0, 0, 0, 0], self.acc, -1, False)
                #机械臂末端向  上
                elif key_input.z1: 
                    if ANTICRASH_UP != 0 and sensor_data["up"] < ANTICRASH_UP:
                        v1 = 0
                        print("向上移动被防撞保护禁止,up传感器值:", sensor_data.get("up"))
                    self.duco_cobot.speedl([0, v1, 0, 0, 0, 0],self.acc ,-1, False)
                #机械臂末端向  下
                elif key_input.z0:
                    self.duco_cobot.speedl([0, -v1, 0, 0, 0, 0],self.acc ,-1, False)
                #初始化位置
                elif key_input.init:
                    self.duco_cobot.servoj_pose(self.init_pos, self.vel, self.acc, '', '', '', True)
                    print("move to initial position: %s" % self.init_pos)
                #维修位置
                elif key_input.serv:
                    self.duco_cobot.servoj_pose(self.serv_pos, self.vel, self.acc, '', '', '', True)
                    print("move to service position: %s" % self.serv_pos)
                #机械臂末端转  pitch上
                elif key_input.rx0: 
                    self.duco_cobot.speedl([0, 0, 0, 0, 0, v5], self.acc, -1, False)
                    time.sleep(0.05)
                #机械臂末端转  pitch下
                elif key_input.rx1: 
                    self.duco_cobot.speedl([0, 0, 0, 0, 0, -v5], self.acc, -1, False)
                    time.sleep(0.05)
                #机械臂末端转  roll左
                elif key_input.ry0: 
                    self.duco_cobot.speedl([0, 0, 0, v3, 0, 0], self.acc, -1, False)
                #机械臂末端转  roll右
                elif key_input.ry1: 
                    self.duco_cobot.speedl([0, 0, 0, -v3, 0, 0], self.acc, -1, False)
                #机械臂末端转  yaw左
                elif key_input.rz0: 
                    self.duco_cobot.speedl([0, 0, 0, 0, v4, 0], self.acc, -1, False)
                #机械臂末端转  yaw右
                elif key_input.rz1: 
                    self.duco_cobot.speedl([0, 0, 0, 0, -v4, 0], self.acc, -1, False)

                # TODO 圆柱喷涂
                # elif btn_y:
                #     self.duco_cobot.switch_mode(1)
                #     auto_painter = CylinderAutoPaint(self.duco_cobot, self.init_pos, self.theta_deg, self.distance_to_cylinder, self.painting_width, self.vel, self.acc)
                #     auto_painter.auto_paint()

                else:
                    self.duco_cobot.speed_stop(False)
            
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            self.sysrun = False
        finally:
            self.emergency_thread.join()
