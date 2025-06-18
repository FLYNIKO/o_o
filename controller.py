from array import array
import sys
import time
import pygame
import DucoCobot
from auto import auto_control_fun


def xbox_control_fun(duco_cobot, joystick):

    axis_threshold = 0.1
    speed = 0.2
    aj_pos = [0, 0, 0, 0, 0, 0]
    history_pos = []
    manual_mode = False
    init_pos = [0.00, -0.35, 2.35, -1.91, -1.57, 0.00]

    print("waiting for robot initialization...")
    duco_cobot.movej2(init_pos, 2.0, 1.0, 0, True)
    print("move to initial position: %s" % init_pos)
    print("Press Start to enter manual mode")
    time.sleep(1)

    while not manual_mode:
        try:
            pygame.event.pump()
            btn_start = joystick.get_button(7)
            btn_x = joystick.get_button(2)

            if btn_start == 1:
                manual_mode = True
                print("Enter manual mode")
                time.sleep(0.5)
            elif btn_x == 1:
                auto_control_fun(duco_cobot)
            
            while manual_mode:
                pygame.event.pump()
                axis_values = [joystick.get_axis(i) for i in range(6)]
                btn_values = [joystick.get_button(i) for i in (0, 1, 3, 4, 5, 7)]
                hat_values = joystick.get_hat(0)

                # print(f"0134 {axis_values[0]:.2f} {axis_values[1]:.2f} {axis_values[2]:.2f} {axis_values[3]:.2f} {axis_values[4]:.2f} {axis_values[5]:.2f}")

                axis_lx, axis_ly, axis_rx, axis_ry, axis_tl, axis_tr = axis_values
                btn_a, btn_b, btn_y, btn_lb, btn_rb, btn_start = btn_values

                v0 = axis_lx * speed                # left stick left/right _ arm left/right
                v1 = axis_ly * speed                # left stick up/down _ arm up/down
                v2 = (axis_tl - axis_tr) * speed    # left/right trigger _ arm forward/backward
                v3 = -axis_ry * speed * 2           # right stick up/down _ arm head up/down
                v4 = axis_rx * speed * 2            # right stick left/right _ arm head left/right
                v5 = speed * 5                      # hat left/right _ arm head rotate left/right

                if any(abs(axis) > axis_threshold for axis in [axis_lx, axis_ly, axis_rx, axis_ry, axis_tl + 1, axis_tr + 1]):
                    duco_cobot.speedl([v0, v1, v2, v3, v4, 0],5 ,-1, False)
                    time.sleep(0.05)
                
                elif hat_values == (-1, 0):
                    duco_cobot.speedl([0, 0, 0, 0, 0, -v5], 5, -1, False)
                    time.sleep(0.05)
                
                elif hat_values == (1, 0):
                    duco_cobot.speedl([0, 0, 0, 0, 0, v5], 5, -1, False)
                    time.sleep(0.05)
                
                elif btn_y:
                    duco_cobot.speed_stop(False)
                
                elif btn_rb:
                    for aj_pos in history_pos:
                        try:
                            index = history_pos.index(aj_pos) + 1
                            duco_cobot.movej2(aj_pos, 1.0, 1.0, 0, True)
                            print("move to no.%d position: %s" % (index, aj_pos))

                        except ValueError:
                            print("Position %s not found in history." % aj_pos)

                    if len(history_pos) > 1:
                        duco_cobot.movej2(history_pos[0],1.0,1.0,0,True)
                        print("move to no.1 position: %s" % history_pos[0])

                    elif len(history_pos) == 0:
                        print("No position in history.Press LB to record position.")
                        time.sleep(0.5)

                elif btn_lb:
                    aj_pos = duco_cobot.get_actual_joints_position()
                    position = [aj_pos[0], 
                                aj_pos[1], 
                                aj_pos[2], 
                                aj_pos[3], 
                                aj_pos[4], 
                                aj_pos[5]]
                    history_pos.append(position)
                    print("new position: %s counter: %d" % (position, len(history_pos)))
                    time.sleep(0.5)

                elif btn_a:
                    history_pos.clear()
                    print("clear history position")
                    time.sleep(0.5)

                elif btn_start:
                    manual_mode = False
                    print("Exit manual mode")
                    time.sleep(0.5)
                    break

                else:
                    duco_cobot.speed_stop(False)

        except AttributeError as e:
            print(f"Error accessing joystick buttons: {e}")
            break
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            break
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            manual_mode = True
            break


    pygame.quit()
