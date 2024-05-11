import os
import struct
import time

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

# Control table address
ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
ADDR_GOAL_POSITION          = 564
LEN_GOAL_POSITION           = 4          # Data Byte Length
ADDR_PRESENT_POSITION       = 580
LEN_PRESENT_POSITION        = 4          # Data Byte Length
BAUDRATE                    = 2000000
ADDR_DRIVE_MODE             = 10
ADDR_Working_mode           = 11         # Operation mode address

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
Master_ID = [1,2]
Slave_ID = [3,4]

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'


TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
PROFILE_ENABLE              = 0x0              # Value for enable trajectory profile
PROFILE_DISABLE             = 0x02             # Value for disable trajectory profile
CURRENT_CONTROL_MODE        = 0                # Value for current control mode

dxl_goal_position = [0, 0, 0, 0, 0, 0];         # Goal position
dxl_present_position = [0, 0, 0, 0, 0, 0];         # Present position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)


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

def Dynamixel_Homing(ID):
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % ID)

    # Enable trajectory profile
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_DRIVE_MODE, PROFILE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Write start position point
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, 0)
    
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Read present position
        present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, ID, ADDR_PRESENT_POSITION)
        present_position = struct.unpack('i', struct.pack('I', present_position))[0]
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (ID, dxl_goal_position[0], present_position))

        if not abs(dxl_goal_position[0] - present_position) > DXL_MOVING_STATUS_THRESHOLD:
            break    
        print("[present_position:%03d]" % present_position)

    # Disable trajectory profile
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_DRIVE_MODE, PROFILE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))        

for i in range(0,len(Slave_ID)):
    Dynamixel_Homing(Slave_ID[i])
for i in range(0,len(Master_ID)):
    Dynamixel_Homing(Master_ID[i])

for i in range(0,len(Master_ID)):
    # Add parameter storage for Dynamixel present position value
    dxl_addparam_result = groupSyncRead.addParam(Master_ID[i])
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % Master_ID[i])
        quit()

def Disable_Dynamixel(ID):
    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
for i in range(0,len(Master_ID)):
    Disable_Dynamixel(Master_ID[i])
for i in range(0,len(Slave_ID)):
    Disable_Dynamixel(Slave_ID[i])

def Set_Dynamixel_Working_mode(ID,Working_mode):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_Working_mode, Working_mode)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % ID)
# Set the manually operated motor to current mode
for i in range(0, len(Master_ID)):
    Set_Dynamixel_Working_mode(Master_ID[i],CURRENT_CONTROL_MODE)

def Enable_Dynamixel(ID):
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % ID)
for i in range(0, len(Slave_ID)):
    Enable_Dynamixel(Slave_ID[i])
for i in range(0,len(Master_ID)):
    Enable_Dynamixel(Master_ID[i])

while 1:
    # syncread present position
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    else:
        for i in range(0, len(Master_ID)):
            dxl_present_position[i] = groupSyncRead.getData(Master_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            dxl_present_position[i] = struct.unpack('i', struct.pack('I', dxl_present_position[i]))[0]
            if abs(dxl_present_position[i] - dxl_goal_position[i]) < 10000:
                dxl_goal_position[i] = dxl_present_position[i]
            print("dxl_present_position[%03d]:%03d " % (Master_ID[i], dxl_present_position[i]))
            print("dxl_goal_position[%03d]:%03d " % (Master_ID[i], dxl_goal_position[i]))

        for i in range(0, len(Slave_ID)):
            # Allocate goal position value into byte array
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]))]
            # Add Dynamixel goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupSyncWrite.addParam(Slave_ID[i], param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % Slave_ID[i])
                quit()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

    # Wait for movement to goal position
    time.sleep(0.01)


# Clear syncread parameter storage
groupSyncRead.clearParam()

# Close port
portHandler.closePort()













    

