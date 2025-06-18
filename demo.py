from array import array
import sys
import time
import threading
import pygame


sys.path.append('gen_py')
sys.path.append('lib')
from DucoCobot import DucoCobot
from thrift import Thrift
from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from gen_py.robot.ttypes import StateRobot, StateProgram, OperationMode,TaskState,Op,MoveJogTaskParam,PointOp
from ManualControl import XboxControllerManual
from joystick import Joystick_fun


ip='192.168.100.10'
stopheartthread = False


def hearthread_fun():
    duco_heartbeat = DucoCobot(ip, 7003)
    duco_heartbeat.open()

    while not stopheartthread:
        duco_heartbeat.rpc_heartbeat()
        time.sleep(1)
    duco_heartbeat.close()
    
def thread_fun():
    duco_cobot = DucoCobot(ip, 7003)
    duco_cobot.open()

    while not stopheartthread:
        tcp_pose = []
        tcp_pose = duco_cobot.get_robot_state()
        print("state: ", tcp_pose)
        time.sleep(1)
    duco_cobot.close()

def robot_connect(duco_cobot):
    rlt = duco_cobot.open()
    print("open:", rlt)
    rlt = duco_cobot.power_on(True)
    print("power_on:", rlt)
    rlt = duco_cobot.enable(True)
    print("enable:", rlt)


def main():
    duco_cobot = DucoCobot(ip,7003)
    robot_connect(duco_cobot)
    joystick = Joystick_fun().connect_joystick()
    
    
    thd_A = threading.Thread(target=thread_fun)
    thd_A.start()
    thd_B = threading.Thread(target=hearthread_fun)
    thd_B.start()
    
    try:
        XboxControllerManual(duco_cobot, joystick).run()

    finally:
        global stopheartthread
        stopheartthread = True
        time.sleep(1)
        rlt = duco_cobot.close()
        print("close:", rlt)

if __name__ == '__main__':
    try:
        main()
    except Thrift.TException as tx:
        print('%s' % tx.message)
