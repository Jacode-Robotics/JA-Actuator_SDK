from bao import *


PORTNAME            = 'COM2'  
BAUDRATE            = 115200  

if serial_port.openPort():
    print("open port suceeded")
else:
    print("无法打开端口，请检查端口名称和权限设置。")

serial_port = SerialPort(PORTNAME, BAUDRATE)


result,list = read(1,'10')
print('读取到的数据为:%d\n返回报文为:%s' %(result,list))

time.sleep(0.5)

write(1,'10',1)

time.sleep(0.5)

result,list = read(1,'10')
print('读取到的数据为:%d\n返回报文为:%s' %(result,list))