import sys
import time
import threading
import rospy

sys.path.append('gen_py')
sys.path.append('lib')
from DucoCobot import DucoCobot
from thrift import Thrift
from ManualControl import system_control
from std_msgs.msg import Float64MultiArray

class DemoApp:
    def __init__(self, ip):
        self.ip = ip
        self.stopheartthread = False
        self.duco_cobot = DucoCobot(ip, 7003)
        self.hearthread = threading.Thread(target=self.hearthread_fun)
        self.thread = threading.Thread(target=self.thread_fun)
        self.tcp_state = []  
        self.tcp_pub = rospy.Publisher('/Duco_state', Float64MultiArray, queue_size=10)

    def robot_connect(self):
        rlt = self.duco_cobot.open()
        print("open:", rlt)
        rlt = self.duco_cobot.power_on(True)
        print("power_on:", rlt)
        rlt = self.duco_cobot.enable(True)
        print("enable:", rlt)
        self.duco_cobot.switch_mode(1)

    def hearthread_fun(self):
        self.duco_heartbeat = DucoCobot(self.ip, 7003)
        self.duco_heartbeat.open()
        while not self.stopheartthread:
            self.duco_heartbeat.rpc_heartbeat()
            time.sleep(0.1)
        self.duco_heartbeat.close()

    def thread_fun(self):
        self.duco_thread = DucoCobot(self.ip, 7003)
        self.duco_thread.open()
        while not self.stopheartthread:
            tcp_state = []
            tcp_state = self.duco_thread.get_robot_state()
            self.tcp_state = tcp_state
            # 发布 ROS topic
            msg = Float64MultiArray()
            msg.data = tcp_state
            self.tcp_pub.publish(msg)
        self.duco_thread.close()

    def run(self):
        self.robot_connect()
        self.hearthread.start()
        self.thread.start()

        try:
            system_control(self.ip, self.duco_cobot, self).run()
        finally:
            self.stopheartthread = True
            time.sleep(1)
            self.hearthread.join()
            self.thread.join()
            rlt = self.duco_cobot.close()
            print("close:", rlt)

    def main(self):
        try:
            self.run()
        except Thrift.TException as tx:
            print('%s' % tx.message)

if __name__ == '__main__':
    # app = DemoApp('192.168.0.18')
    rospy.init_node('Duco_state_publisher', anonymous=True)
    app = DemoApp('192.168.100.10')
    try:
        app.main()
    except KeyboardInterrupt:
        print("主程序收到 KeyboardInterrupt，准备退出。")
        app.stopheartthread = True
        time.sleep(1)
        app.hearthread.join()
        app.thread.join()
        rlt = app.duco_cobot.close()
        print("close:", rlt)
