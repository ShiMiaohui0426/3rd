from pymycobot import MyCobotSocket
import time
from pymycobot.genre import Angle
from pymycobot.mycobot import MyCobot
import threading

def wait_moving(robot):
    time.sleep(0.05)
    while robot.is_moving():
        time.sleep(0.05)


# mc = MyCobot("/dev/ttyACM0", 115200)

mc = MyCobotSocket("192.168.1.3", 9000)
speed = 20
mc.send_angles([0, 0, 0, 0, 0, 0], speed)
mc.set_encoder(7, 0)
wait_moving(mc)
'''
mc.send_angles([-90, -30, 0, 0, 0, 0], 80)
coords=mc.get_coords()
print(coords)
wait_moving(mc)
time.sleep(1)
def grasp_thread_task():
    tar=[103, -129, 165, -179, -1, 65]
    top=[103, -129, 155, -179, -1, 65-180]
    mc.send_coords(top, speed, 0)
    wait_moving(mc)
    mc.send_coords(top, speed, 0)
    wait_moving(mc)
    mc.set_encoder(7, 0)
    time.sleep(2)
    wait_moving(mc)
    mc.send_coords(top, speed, 0)
    wait_moving(mc)
    mc.send_angles([0, 0, 0, 0, 0, 0], speed)
    wait_moving(mc)
    mc.send_angles([90, -45, 0, 0, 0, 0], speed)
    wait_moving(mc)
    mc.set_encoder(7, 1800)
    time.sleep(2)
    wait_moving(mc)
    mc.send_angles([0, 0, 0, 0, 0, 0], speed)
    wait_moving(mc)

grasp_thread = threading.Thread(target=grasp_thread_task)
grasp_thread.start()
'''
'''
print('start test')
time_start = time.time()  # 记录开始时间
i=0
while i<100:
    i=i+1
    mc.send_angles([-90+i, -30, -45, -15, 0, 0], 50)
    wait_moving(mc)

time_end = time.time()  # 记录结束时间
time_sum = time_end - time_start  # 计算的时间差为程序的执行时间，单位为秒/s
print(time_sum)
print('stop test')
'''

