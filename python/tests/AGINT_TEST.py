import os
import struct
import time
import keyboard

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

from dynamixel_sdk import *                 # Uses Dynamixel SDK library

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Define the proper baudrate to search DYNAMIXELs. Note that XL320's baudrate is 1 M bps.
BAUDRATE                = 2000000

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 1
NEW_ID                      = 0
dxl_goal_position = [0, 1, 5, 10, 18, 28, 41, 56, 73, 92, 114, 137, 164, 192, 223, 256, 291, 328, 368, 410, 455, 501, 550, 601, 655, 710, 768, 828, 891, 956, 1023, 1092, 1164, 1237, 1313, 1388, 1464, 1539, 1614, 1690, 1765, 1840, 1916, 1991, 2067, 2142, 2217, 2293, 2368, 2443, 2519, 2594, 2670, 2745, 2820, 2896, 2971, 3046, 3122, 3197, 3273, 3348, 3423, 3499, 3574, 3649, 3725, 3800, 3872, 3943, 4011, 4077, 4141, 4202, 4261, 4318, 4372, 4425, 4475, 4522, 4568, 4611, 4652, 4691, 4727, 4761, 4793, 4822, 4850, 4875, 4897, 4918, 4936, 4952, 4966, 4977, 4986, 4993, 4997, 5000, 5000, 4997, 4993, 4986, 4977, 4966, 4952, 4936, 4918, 4897, 4875, 4850, 4822, 4793, 4761, 4727, 4691, 4652, 4611, 4568, 4522, 4475, 4425, 4372, 4318, 4261, 4202, 4141, 4077, 4011, 3943, 3872, 3800, 3725, 3649, 3574, 3499, 3423, 3348, 3273, 3197, 3122, 3046, 2971, 2896, 2820, 2745, 2670, 2594, 2519, 2443, 2368, 2293, 2217, 2142, 2067, 1991, 1916, 1840, 1765, 1690, 1614, 1539, 1464, 1388, 1313, 1237, 1164, 1092, 1023, 956, 891, 828, 768, 710, 655, 601, 550, 501, 455, 410, 368, 328, 291, 256, 223, 192, 164, 137, 114, 92, 73, 56, 41, 28, 18, 10, 5, 1, 0]         # Goal position
# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = 'COM15'

#THE data of calibrate the zero position is 17
ZERO_POSITION               = 17

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncRead = GroupSyncRead(portHandler, packetHandler, 0, 2)

'''def hex_to_decimal(hex_string):
    decimal_value = int(hex_string, 16)
    return decimal_value'''

'''def READ_MODEL_NUMBER(ID):
    data_read = groupSyncRead.getData(ID, 0, 2)
    data_read = struct.unpack('i', struct.pack('I', data_read))[0]
    print(data_read)
    if result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(result))
    elif error != 0:
        print("%s" % packetHandler.getRxPacketError(error))
'''
    
    #NUMBER = hex_to_decimal(data_read)
    #return NUMBER

def MODEL_NUMBER(ID,NUMBER):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, 0, NUMBER)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("ID:[%d],[%d]connected succeeded" % (ID, NUMBER))

def Torque_Enable(ID, DATA):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, 512, DATA)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    if DATA == 0:
        print('disable succeeded')
    else:
        print('Ensable  succeeded')


def opsition_control(ID, DATA):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID, 564, DATA)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("CONTROL Succeeded,%d" %DATA)

def profile_velocity(ID, DATA):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID, 560, DATA)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("velocity modification Succeeded")

def Control_Mode(ID,DATA):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, 11, DATA)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Mode switching Succeeded")

flag = True  
def on_key_pressed(event):
    global flag
    if event.name == 'enter':
        print("quit succeeded")
        flag = False  

keyboard.on_press(on_key_pressed)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

DXL_ID = []
NUMBER = []


for i in range(1,21):
    dxl_model_number,dxl_comm_result, dxl_error = packetHandler.ping(portHandler, i)
    if dxl_comm_result != COMM_SUCCESS:
        continue
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("ID [%d] ping Succeeded" % i)
        print('number:%d' % dxl_model_number)
        DXL_ID.append(i)
        NUMBER.append(dxl_model_number)
        
    time.sleep(0.5)


for i in range(0,len(DXL_ID)):
    MODEL_NUMBER(DXL_ID[i],NUMBER[i])
    time.sleep(0.3)
    Torque_Enable(DXL_ID[i],0)
#    time.sleep(0.2)
#    profile_velocity(DXL_ID[i],500)
    time.sleep(0.2)
    Torque_Enable(DXL_ID[i],1)
  
    #READ_MODEL_NUMBER(i)
    #NUMBER = READ_MODEL_NUMBER(i)
    #MODEL_NUMBER(NUMBER)

#time.sleep(1)
while 1:
    id_number = 0
    for i in range(0,len(DXL_ID)):
        opsition_control(DXL_ID[i],0)
        id_number += 1
    if id_number >= len(DXL_ID):
        break

time.sleep(4)

while flag:
    for j in range(0,len(dxl_goal_position)):

        for i in range(0,len(DXL_ID)):

            opsition_control(DXL_ID[i],dxl_goal_position[j])

    
        time.sleep(0.1)

    print('succeeded')
    print('press Enter to quit!')




for i in range(0,len(DXL_ID)):
    opsition_control(DXL_ID[i],0)
time.sleep(4)

for i in range(0,len(DXL_ID)):
    Torque_Enable(DXL_ID[i],0)


# Close port
portHandler.closePort()






