import time
import keyboard
from bao import *

PORTNAME            = '/dev/ttyUSB1'  
BAUDRATE            = 115200  

serialport = SerialPort(PORTNAME)

flag = True  
def on_key_pressed(event):
    global flag
    if event.name == 'enter':
        print("quit succeeded")
        flag = False  

keyboard.on_press(on_key_pressed)

# Open port
if serialport.openPort() == 0:
    print("Failed to open the port")
    print("Press any key to terminate...")


# Set port baudrate
if serialport.setBaudRate(BAUDRATE) == 0:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
'''
message = write(serialport,1,'1',3)
print('发送报文为:%s' %message)

message = write(serialport,1,'2d',1)
print('发送报文为:%s' %message)
'''

message = write(serialport,1,'10',1)
print('发送报文为:%s' %message)
'''

time.sleep(0.3)
wz = 0

while flag:
    message = write(serialport,1,'81',wz)
    result,message = read(serialport,1,'15')
    print('读取到的数据为:%d' %result)

    wz -= 20

time.sleep(0.3)

result,message = read(serialport,1,'15')
print('读取到的数据为:%d' %result)

message = write(serialport,1,'10',0)
print('发送报文为:%s' %message)
'''

'''    
message = write(serialport,1,'32',1)
print('发送报文为:%s' %message)

time.sleep(15)

message = write(serialport,1,'2f',0)
print('发送报文为:%s' %message)

time.sleep(0.3)

message = write(serialport,1,'10',0)
print('发送报文为:%s' %message)
'''
'''
result,message = read(serialport,1,'15')
print('读取到的数据为:%d\n返回报文为:%s' %(result,message))

message = write(serialport,1,'82',0)
print('发送报文为:%s' %message)

time.sleep(0.5)
result,message = read(serialport,1,'15')
print('读取到的数据为:%d\n返回报文为:%s' %(result,message))

message = write(serialport,1,'10',0)
print('发送报文为:%s' %message)
'''

'''
message = write(serialport,1,'10',1)
print('发送报文为:%s' %message)


message = write(serialport,1,'32',1)
print('放送报文为:%s' %message)
'''

'''
result,message = read(serialport,1,'10')
print('读取到的数据为:%d\n返回报文为:%s' %(result,message))

time.sleep(0.5)

message = write(serialport,1,'10',0)
print('放送报文为:%s' %message)
time.sleep(0.5)

result,message = read(serialport,1,'10')
print('读取到的数据为:%d\n返回报文为:%s' %(result,message))
'''



serialport.closePort()