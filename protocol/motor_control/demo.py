import os
import time
import keyboard
from bao import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

PORTNAME            = '/dev/ttyUSB0'  
BAUDRATE            = 115200  

serialport = SerialPort(PORTNAME)

# flag = True  
# def on_key_pressed(event):
#     global flag
#     if event.name == 'enter':
#         print("quit succeeded")
#         flag = False  

# keyboard.on_press(on_key_pressed)

# Open port
if serialport.openPort() == 0:
    print("Failed to open the port")


# Set port baudrate
if serialport.setBaudRate(BAUDRATE) == 0:
    print("Failed to change the baudrate")

for i in range(1,21):
    result, error, message = read(serialport,i,'2')
    if result == None and message == None:
        print('第%d没有值'%i)
        print(error)
        continue
    else:
        print('%d读取到的数据为:%d,错误：%d' %(i,result,error))

#改波特率并保存
''' 
message, error = write(serialport,1,'1',3)
print('发送报文为:%s' %message)

message, error = write(serialport,1,'2d',1)
print('发送报文为:%s' %message)
'''


#回零后给无加减速位置模式输入值,前后差值100 频率500Hz
'''
beacon = 1
message, error = write(serialport,1,'10',1)
print('发送报文为:%s' %message)
time.sleep(0.3)
message, error = write(serialport,1,'32',1)
print('发送报文为:%s' %message)

while beacon:

    result, error, message = read(serialport,1,'1a')
    print('读取到的数据为:%d' %result)
    if result == 0:
        beacon = 0

time.sleep(0.3)

result, error, message = read(serialport,1,'15')
print('读取到的数据为:%d' %result)
wz = result

while flag:
    message, error = write(serialport,1,'81',wz)
    result, error, message = read(serialport,1,'15')
    print('读取到的数据为:%d' %result)

    wz -= 100
    time.sleep(0.002)

time.sleep(0.3)

result, error, message = read(serialport,1,'15')
print('读取到的数据为:%d' %result)

message, error = write(serialport,1,'10',0)
print('发送报文为:%s' %message)
''' 


#回零后以500速度运行,按任意键通知并以位置模式回到原点
''' 
beacon = 1
message, error = write(serialport,1,'10',1)
print('发送报文为:%s' %message)

time.sleep(0.3)

message, error = write(serialport,1,'32',1)
print('发送报文为:%s' %message)

while beacon:

    result, error, message = read(serialport,1,'1a')
    print('读取到的数据为:%d' %result)
    if result == 0:
        beacon = 0

time.sleep(0.3)

message, error = write(serialport,1,'2f',30)
print('发送报文为:%s' %message)

for i in (1,10):
    result, error, message = read(serialport,1,'2f')
    print('读取到的数据为:%d' %result)
getch()

message, error = write(serialport,1,'2f',0)
print('发送报文为:%s' %message)

message, error = write(serialport,1,'33',1)
print('发送报文为:%s' %message)

time.sleep(0.3)

message, error = write(serialport,1,'82',0)
print('发送报文为:%s' %message)
beacon1 = 1
while beacon1:

    result, error, message = read(serialport,1,'1a')
    print('读取到的数据为:%d' %result)
    if result == 0:
        beacon1 = 0

time.sleep(0.3)

message, error = write(serialport,1,'10',0)
print('发送报文为:%s' %message)
'''  

serialport.closePort()