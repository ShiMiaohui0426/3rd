from pymycobot import MyCobotSocket
import time
from pymycobot.genre import Angle
from pymycobot.mycobot import MyCobot
def wait_moving(robot):
    while robot.is_moving():
        time.sleep(0.05)


mc = MyCobot("/dev/ttyACM0", 115200)

#mc = MyCobotSocket("192.168.1.5", 9000)

mc.send_angles([0, 0, 0, 0, 0, 0], 20)
time.sleep(0.1)
wait_moving(mc)

mc.send_angles([-90, -30, -45, -15, 0, 0], 50)
wait_moving(mc)
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
time.sleep(1)
coords = mc.get_coords()
print(coords)
time.sleep(1)
mc.send_coords([-50, -223.6, 212.1, -177.36, -0.79, -179.68], 80, 1)
wait_moving(mc)
coords = mc.get_coords()
print(coords)
mc.set_encoder(7, 2000)
wait_moving(mc)
