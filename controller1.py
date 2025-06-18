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
    # Connect!
    duco_cobot.open()
    while not stopheartthread:
        tcp_pose = []
        tcp_pose = duco_cobot.get_robot_state()
        print("state: ", tcp_pose)
        time.sleep(1)
    duco_cobot.close()

def xbox_control_fun():

    stopheartthread = True

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("no controller found")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    duco_cobot = DucoCobot(ip, 7003)
    rlt = duco_cobot.open()
    print("open:", rlt)
    rlt = duco_cobot.power_on(True)
    print("power_on:", rlt)
    rlt = duco_cobot.enable(True)
    print("enable:", rlt)

    
    while stopheartthread:
        
        pygame.event.pump()
        btn_b = joystick.get_button(1)
        if btn_b == 1:
            stopheartthread = False
            time.sleep(0.5)
        
        while not stopheartthread:
            axis_threshold = 0.1
            speed = 1.2
            
            pygame.event.pump()
            axis_lx = joystick.get_axis(0)  # L/leftright
            axis_ly = joystick.get_axis(1)  # L/updown
            axis_rx = joystick.get_axis(2)  # R/leftright
            axis_ry = joystick.get_axis(3)  # R/updown
            axis_tl = joystick.get_axis(4)  # LT
            axis_tr = joystick.get_axis(5)  # RT
            print(f"0134 {axis_lx:.6f} {axis_ly:.6f} {axis_rx:.6f} {axis_ry:.6f} {axis_tl:.6f} {axis_tr:.6f}")
            btn_a = joystick.get_button(0)
            btn_b = joystick.get_button(1)

            # if abs(axis_lx) > 0.1 or abs(axis_ly) > 0.1 or abs(axis_rx) > 0.1 or abs(axis_ry) > 0.1:
            if any(abs(axis) > axis_threshold for axis in [axis_lx, axis_ly, axis_rx, axis_ry, axis_tl + 1, axis_tr + 1]):
                v0 = axis_lx * speed
                v1 = -axis_ly * speed
                v2 = axis_rx * speed
                v3 = -axis_ry * speed
                v4 = (axis_tl - axis_tr) * speed
                duco_cobot.speedj([v0, v1, v2, v3, v4, 0],5 ,-1, False)
                time.sleep(0.05)
            elif btn_a:
                duco_cobot.movej2([0,0,0,0,0,0],1.0,1.0,0,True)
            elif btn_b:
                stopheartthread = True
                time.sleep(0.5)
                break
            else:
                duco_cobot.speed_stop(False)
        

    rlt = duco_cobot.close()
    print("close:", rlt)
    pygame.quit()

def main():
    thd_A = threading.Thread(target=thread_fun)
    thd_A.start()
    thd_B = threading.Thread(target=hearthread_fun)
    thd_B.start()
    thd_c = threading.Thread(target=xbox_control_fun)
    thd_c.start()

    duco_cobot = DucoCobot(ip,7003)
    
    thd_c.join()
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
