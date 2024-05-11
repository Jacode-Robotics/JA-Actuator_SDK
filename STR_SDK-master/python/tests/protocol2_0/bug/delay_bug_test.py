import os
import struct
import time
import sys
import timeit

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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL_ID       = [1,2,3,4,5,6]
dxl_present_position  = [0,0,0,0,0,0]

# Control table address
ADDR_GOAL_POSITION          = 564        # goal position address
LEN_GOAL_POSITION           = 4          # Data Byte Length
ADDR_PRESENT_POSITION       = 580
LEN_PRESENT_POSITION        = 4
BAUDRATE                    = 2000000


# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncRead instace for Present Position
groupSyncReadPosition = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)


def Open_port(baudrate):
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(baudrate):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
Open_port(BAUDRATE)

for i in range(0,len(DXL_ID)): 
    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID[i])
    if dxl_comm_result != COMM_SUCCESS:
        print("[ID:%03d] ping failed %s" % (DXL_ID[i], packetHandler.getTxRxResult(dxl_comm_result)))
        quit()
    elif dxl_error != 0:
        print("[ID:%03d] ping failed %s" % (DXL_ID[i], packetHandler.getRxPacketError(dxl_error)))
        quit()
    else:
        print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (DXL_ID[i], dxl_model_number))

for i in range(0,len(DXL_ID)): 
    dxl_addparam_result = groupSyncReadPosition.addParam(DXL_ID[i])
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncReadPosition addparam failed" % DXL_ID[i])
        quit()

def Read_Dynamixel_Position():
    # syncread present position
    dxl_comm_result = groupSyncReadPosition.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    else:
        for i in range(0,len(DXL_ID)):
            dxl_present_position[i] = groupSyncReadPosition.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            dxl_present_position[i] = struct.unpack('i', struct.pack('I', dxl_present_position[i]))[0]
            print("[ID:%03d] PresPos:%03d" % (DXL_ID[i], dxl_present_position[i]))

while 1:
    # 获取当前时间戳
    timestamp_start = timeit.default_timer()
    
    Read_Dynamixel_Position()
    # 获取当前时间戳
    timestamp_end = timeit.default_timer()
    # 打印当前时间戳
    print("time: %lf" % (timestamp_end-timestamp_start))

    time.sleep(1)
    
# Close port
portHandler.closePort()