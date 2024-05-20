import os
import struct
import time
import sys
import subprocess

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
Firmware_version            = 'DRV-v7.4.bin'

# Make sure that each DYNAMIXEL ID should have unique ID.
NOW_DXL_ID   = []           # Current ID
DXL_ID       = []           # New ID
DXL_MODEL    = []           # Dynamixel model

NOW_DXL_ID.append(1)
DXL_ID.append(1)
DXL_MODEL.append(5210)

dxl_goal_position     = 5000          # Goal position
dxl_present_position  = []            # Present position

dxl_goal_velocity     = 300             # Goal velocity
dxl_present_velocity  = []            # Present velocity

dxl_goal_current      = 50            # Goal current
dxl_present_current   = []            # Present current

dxl_present_position.append(0)
dxl_present_velocity.append(0)
dxl_present_current.append(0)

# Control table address
ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
ADDR_GOAL_POSITION          = 564        # goal position address
LEN_GOAL_POSITION           = 4          # Data Byte Length
ADDR_PRESENT_POSITION       = 580
LEN_PRESENT_POSITION        = 4
BAUDRATE                    = 2000000
ADDR_DRIVE_MODE             = 10         # Drive mode (whether the lower computer performs motion planning)
ADDR_OPERATING_MODE         = 11         # Operation mode
ADDR_DXL_MODEL              = 0          # DYNAMIXEL model address
ADDR_GOAL_VELOCITY          = 552        # goal velocity address
LEN_GOAL_VELOCITY           = 4
ADDR_PRESENT_VELOCITY       = 576        # present velocity address
LEN_PRESENT_VELOCITY        = 4
ADDR_GOAL_CURRENT          = 550        # goal velocity address
LEN_GOAL_CURRENT           = 4
ADDR_PRESENT_CURRENT       = 574        # present velocity address
LEN_PRESENT_CURRENT        = 4
ADDR_ID                     = 7          # DYNAMIXEL ID address
LEN_ID                      = 1
ADDR_Secret_key             = 14


# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
DXL_MOVING_VELOCITY_THRESHOLD = 1000             # Dynamixel moving status threshold
DXL_MOVING_CURRENT_THRESHOLD = 5             # Dynamixel moving status threshold
PROFILE_ENABLE              = 0x0               # Value for enable trajectory profile
PROFILE_DISABLE             = 0x02              # Value for disable trajectory profile
CURRENT_CONTROL_MODE        = 0                 # Value for current control mode
ELE_ZERO_MODEL              = 17                # Value for Set electrical zero position
VELOCITY_CONTROL_Mode       = 1                 # Value for velocity control mode
POSITION_CONTROL_Mode       = 4                 # Value for position control mode
VALUE_SECRET_KEY            = 0x1234

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance for Position
groupSyncWritePosition = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncReadPosition = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

# Initialize GroupSyncWrite instance for velocity
groupSyncWriteVelocity = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)

# Initialize GroupSyncRead instace for Present velocity
groupSyncReadVelocity = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)

# Initialize GroupSyncWrite instance for current
groupSyncWriteCurrent = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT)

# Initialize GroupSyncRead instace for Present current
groupSyncReadCurrent = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT)

def update_crc(data_blk):
    crc_accum = 0x0000
    crc_table = [
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    ]

    for j in range(0,len(data_blk)-2):
        i = ((crc_accum >> 8) ^ data_blk[j]) & 0xFF
        crc_accum = (crc_accum << 8) ^ crc_table[i]

    return crc_accum

def write_1Byte(ID, ADDR, Data, output):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR, Data)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Write {output} failed %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(f"Write {output} failed %s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"Write {output} successfully")

def write_2Byte(ID, ADDR, Data, output):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR, Data)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Write {output} failed %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(f"Write {output} failed %s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"Write {output} successfully")

def sync_write(dxl_goal, goal_value, output, groupSyncWriteHeader):
    dxl_goal = goal_value

    # Allocate goal value into byte array
    param_goal = [DXL_LOBYTE(DXL_LOWORD(dxl_goal)), DXL_HIBYTE(DXL_LOWORD(dxl_goal)), DXL_LOBYTE(DXL_HIWORD(dxl_goal)), DXL_HIBYTE(DXL_HIWORD(dxl_goal))]
    for i in range(0,len(DXL_ID)):
        # Add Dynamixel goal value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteHeader.addParam(DXL_ID[i], param_goal)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[i])
            quit()

    # Syncwrite goal
    dxl_comm_result = groupSyncWriteHeader.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    else:
        print(f"Write Dynamixel {output} successfully")

    # Clear syncwrite parameter storage
    groupSyncWriteHeader.clearParam()

def sync_read(dxl_present_value, dxl_goal_value, ADDR_PRESENT_value, LEN_PRESENT_value, groupSyncHeader, output):
    # syncread present position
    dxl_comm_result = groupSyncHeader.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    else:
        for i in range(0,len(DXL_ID)):
            dxl_present_value[i] = groupSyncHeader.getData(DXL_ID[i], ADDR_PRESENT_value, LEN_PRESENT_value)
            dxl_present_value[i] = struct.unpack('i', struct.pack('I', dxl_present_value[i]))[0]
            print(f"[ID:%03d] Goal{output}:%03d  Pres{output}:%03d" % (DXL_ID[i], dxl_goal_value, dxl_present_value[i]))

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

def ping_the_Dynamixel(ID):
    # Try to ping the Dynamixel and get Dynamixel model number
    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, ID)
    if dxl_comm_result != COMM_SUCCESS:
        print("[ID:%03d] ping failed %s" % (ID, packetHandler.getTxRxResult(dxl_comm_result)))
        quit()
    elif dxl_error != 0:
        print("[ID:%03d] ping failed %s" % (ID, packetHandler.getRxPacketError(dxl_error)))
        quit()
    else:
        print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (ID, dxl_model_number))

def Write_Secret_key(ID,Secret_key):
    write_2Byte(ID, ADDR_Secret_key, Secret_key, "Secret_key")

def Write_Dynamixel_Model(ID,model_num):
    write_2Byte(ID, ADDR_DXL_MODEL, model_num, "Dynamixel model")

def Control_Table_Backup(ID):
    crc = 0x0000
    # 要发送的数据
    data = [0xFF,0xFF,0xFD,0x00,ID,0x08,0x00,0x20,0x01,0x43,0x54,0x52,0x4C,0X00,0X00]
    crc = update_crc(data)
    data[len(data)-2] = crc & 0xff
    data[len(data)-1] = (crc >> 8) & 0xff

    # 将数据转换为字节串
    byte_data = bytes(data)

    # 发送数据
    written_bytes = portHandler.writePort(byte_data)
    # 检查写入的字节数
    if written_bytes != len(byte_data):
        print("Failed to write all bytes")
    else:
        print("Control_Table_Backup success")

def Dynamixel_reboot(ID):
    # Try reboot
    # Dynamixel LED will flicker while it reboots
    dxl_comm_result, dxl_error = packetHandler.reboot(portHandler, ID)
    if dxl_comm_result != COMM_SUCCESS:
        print("reboot Failed %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("reboot Failed %s" % packetHandler.getRxPacketError(dxl_error))
    print("[ID:%03d] reboot Succeeded\n" % ID)

# Try Factory Reset
# Usually, the next step requires a restart
def Dynamixel_Factory_Reset(ID):
    crc = 0x0000
    data = [0xFF, 0xFF, 0xFD, 0x00, ID, 0x04, 0x00, 0x06, 0x01, 0x00, 0x00]      # 要发送的数据
    crc = update_crc(data)
    data[len(data)-2] = crc & 0xff
    data[len(data)-1] = (crc >> 8) & 0xff

    # 将数据转换为字节串
    byte_data = bytes(data)

    # 发送数据
    written_bytes = portHandler.writePort(byte_data)
    # 检查写入的字节数
    if written_bytes != len(byte_data):
        print("Factory Reset Failed")
    else:
        print("Factory Reset successfully")

def Set_Dynamixel_Operating_mode(ID,Operating_mode):
    Set_Dynamixel_Torque(DXL_ID[i],TORQUE_DISABLE)
    time.sleep(0.1)
    write_1Byte(ID, ADDR_OPERATING_MODE, Operating_mode, "dynamixel operating mode")
    time.sleep(0.1)
    Set_Dynamixel_Torque(DXL_ID[i],TORQUE_ENABLE)


def Send_Dynamixel_Velocity(Velocity):
    print("Testing velocity mode!")
    sync_write(dxl_goal_velocity, Velocity, "Velocity", groupSyncWriteVelocity)

def Read_Dynamixel_Velocity():
    sync_read(dxl_present_velocity, dxl_goal_velocity, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY, groupSyncReadVelocity, "vel")
    
    if abs(dxl_present_velocity[i] - dxl_goal_velocity) <= DXL_MOVING_VELOCITY_THRESHOLD:
        print("dxl_goal_velocity:%03d  dxl_present_velocity[%03d]:%03d velocity normal"\
            % (dxl_goal_velocity, DXL_ID[i], dxl_present_velocity[0]))
    else:
        print("dxl_goal_velocity:%03d  dxl_present_velocity[%03d]:%03d velocity abnormal"\
            % (dxl_goal_velocity, DXL_ID[i], dxl_present_velocity[0]))
        quit()

def Send_Dynamixel_Position(Position):
    sync_write(dxl_goal_position, Position, "Position", groupSyncWritePosition)

def Read_Dynamixel_Position():
    sync_read(dxl_present_position, dxl_goal_position, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION, groupSyncReadPosition, "Pos")

def turn_the_Dynamixel(position):
    Send_Dynamixel_Position(position)

    # while 1:
    #     Read_Dynamixel_Position()
    #     arrived_joint_num = 0
    #     for i in range(0,len(DXL_ID)):
    #         if abs(dxl_present_position[i]-dxl_goal_position) <= DXL_MOVING_STATUS_THRESHOLD:
    #             arrived_joint_num += 1
    #     if arrived_joint_num == len(DXL_ID):
    #         break
    time.sleep(5)


def Set_Dynamixel_Torque(ID,TORQUE_en_dis):
    write_1Byte(ID, ADDR_TORQUE_ENABLE, TORQUE_en_dis, "Dynamixel Torque")

def Set_trajectory_profile(ID,PROFILE_en_dis):
    write_1Byte(ID, ADDR_DRIVE_MODE, PROFILE_en_dis, "trajectory profile")

def Dynamixel_Homing():
    print("Dynamixel start Homing!")
    
    for i in range(0,len(DXL_ID)):
        Set_Dynamixel_Torque(DXL_ID[i],TORQUE_ENABLE)
        Set_trajectory_profile(DXL_ID[i],PROFILE_ENABLE)

    turn_the_Dynamixel(0)

def Send_Dynamixel_Current(Current):
    print("Testing current mode!")
    sync_write(dxl_goal_current, Current, "Current", groupSyncWriteCurrent)

def Read_Dynamixel_Current():
    sync_read(dxl_present_current, dxl_goal_current, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT, groupSyncReadCurrent, "cur")

    for i in range(0,len(DXL_ID)):
        if abs(dxl_present_current[i] - dxl_goal_current) <= DXL_MOVING_CURRENT_THRESHOLD:
            print("dxl_goal_current:%03d  dxl_present_current[%03d]:%03d current normal"\
                % (dxl_goal_current, DXL_ID[i], dxl_present_current[i]))
        else:
            print("dxl_goal_current:%03d  dxl_present_current[%03d]:%03d current abnormal"\
                % (dxl_goal_current, DXL_ID[i], dxl_present_current[i]))
            quit()

print(f"\nDefault serial port name: {DEVICENAME}")
print(f"Default firmware version: {Firmware_version}")
print("Default not to download the firmware")
print(f"Default NOW_DXL_ID: {NOW_DXL_ID[0]}")
print(f"Default DXL_ID: {DXL_ID[0]}")
print(f"Default Dynamixel number: {DXL_MODEL[0]}")
print(f"Default Goal position: {dxl_goal_position}")
print(f"Default Goal velocity: {dxl_goal_velocity}")
print(f"Default Goal current: {dxl_goal_current}")
print("You can use '-help' to get more effective help.\n")

# 获取命令行参数
args = sys.argv
num = 0
if len(args) > 1:
    for i in range(0, len(args)):
        # Obtain the serial port name of the input
        if args[i] == '-p':
            DEVICENAME = args[i+1]
            print(f"The input serial port name is: {DEVICENAME}")

        # Obtain the firmware version of the input   
        elif args[i] == '-fv':
            Firmware_version = args[i+1]
            print(f"The input firmware version is: {Firmware_version}")
        
        # download the firmware or not
        elif args[i] == '-d':
            print("Please ensure that the motor is currently in a power-off state!")
            print("Press any key to continue...")
            getch()
            print("Power on in about 2 seconds!")

            # 调用终端命令
            command = f"sudo ./download {DEVICENAME} ./{Firmware_version}"  # 要执行的终端命令

            process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)

            time.sleep(20)
            process.terminate()

        # Obtain Dynamixel number of the input
        elif args[i] == '-n':
            num = int(args[i+1])
            dxl_present_position.clear()  # 清除列表内的所有元素
            dxl_present_velocity.clear()  # 清除列表内的所有元素
            dxl_present_current.clear()  # 清除列表内的所有元素
            for j in range(0, num):
                dxl_present_position.append(0)
                dxl_present_velocity.append(0)
                dxl_present_current.append(0)

        # Obtain NOW_DXL_ID of the input
        elif args[i] == '-CID':
            NOW_DXL_ID.clear()  # 清除列表内的所有元素
            for j in range(0, num):
                NOW_DXL_ID.append(int(args[i + 1 + j]))
            # print("Current ID is %d" % NOW_DXL_ID)

        # Obtain DXL_ID of the input
        elif args[i] == '-NID':
            DXL_ID.clear()  # 清除列表内的所有元素
            for j in range(0, num):
                DXL_ID.append(int(args[i + 1 + j]))
            # print("New ID is %d" % DXL_ID)

        # Obtain Dynamixel model of the input
        elif args[i] == '-DM':
            DXL_MODEL.clear()  # 清除列表内的所有元素
            for j in range(0, num):
                DXL_MODEL.append(int(args[i + 1 + j]))
            # print("Dynamixel model is %d" % DXL_ID)

        # Obtain Goal position of the input
        elif args[i] == '-gp':
            dxl_goal_position = int(int(args[i+1]))
            print("Goal position is %d" % dxl_goal_position)

        # Obtain Goal velocity of the input
        elif args[i] == '-gv':
            dxl_goal_velocity = int(int(args[i+1]))
            print("Goal velocity is %d" % dxl_goal_velocity)

        # Obtain Goal current of the input
        elif args[i] == '-gc':
            dxl_goal_current = int(int(args[i+1]))
            print("Goal current is %d" % dxl_goal_current)

        # Help Information
        elif args[i] == '-help':
            print("-p:   input serial port name")
            print("-fv:  input firmware version")
            print("-d:   If you want to download firmware, you can add '- d' to the command")
            print("-n:   input Dynamixel number")
            print("-CID: input Current ID")
            print("-NID: input New ID")
            print("-DM:  input Dynamixel model")
            print("-gp:  input Goal position")
            print("-gv:  input Goal velocity")
            print("-gp:  input Goal current")
            print("If you need to input the Current ID , New ID or model, please be sure to input the number of motors(-n)")
            print("This is an example:\nsudo python3 Multiple_factory_debug.py -d -p /dev/ttyUSB0 -fv ./DRV-v6.bin -n 2 -CID 1 2 -NID 1 2 -DM 5210 5210 -gp 5000 -gv 5 -gc 50")
            quit()


# Open port
Open_port(BAUDRATE)

for i in range(0,len(NOW_DXL_ID)):
    # Ping ID
    ping_the_Dynamixel(NOW_DXL_ID[i])
    # write secret key
    Write_Secret_key(NOW_DXL_ID[i],VALUE_SECRET_KEY)
    # Write new ID
    write_1Byte(NOW_DXL_ID[i], ADDR_ID, DXL_ID[i], "ID")
    # Ping ID
    ping_the_Dynamixel(DXL_ID[i])
    # Write dynamixel model
    Write_Dynamixel_Model(DXL_ID[i],DXL_MODEL[i])

# Control table backup
for i in range(0,len(DXL_ID)):
    Control_Table_Backup(DXL_ID[i])
time.sleep(2)

# Dynamixel reboot
for i in range(0,len(DXL_ID)):
    Dynamixel_reboot(DXL_ID[i])
time.sleep(1)

# Dynamixel factory reset
for i in range(0,len(DXL_ID)):
    Dynamixel_Factory_Reset(DXL_ID[i])
time.sleep(1)

# Dynamixel reboot
for i in range(0,len(DXL_ID)):
    Dynamixel_reboot(DXL_ID[i])
time.sleep(1)

# Set dynamixel operating mode to ELE_ZERO_MODEL
for i in range(0,len(DXL_ID)):
    Set_Dynamixel_Operating_mode(DXL_ID[i],ELE_ZERO_MODEL)
time.sleep(1)

# Add parameter storage for Dynamixel present value
for i in range(0,len(DXL_ID)):
    dxl_addparam_result = groupSyncReadPosition.addParam(DXL_ID[i])
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncReadPosition addparam failed" % DXL_ID[i])
        quit()
    dxl_addparam_result = groupSyncReadVelocity.addParam(DXL_ID[i])
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncReadVelocity addparam failed" % DXL_ID[i])
        quit()
    dxl_addparam_result = groupSyncReadCurrent.addParam(DXL_ID[i])
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncReadCurrent addparam failed" % DXL_ID[i])
        quit()

# Dynamixel homing
Dynamixel_Homing()
time.sleep(1)

# velocity position mode test
turn_the_Dynamixel(dxl_goal_position)
time.sleep(1)
turn_the_Dynamixel(0)
time.sleep(1)

# Set dynamixel operating mode to VELOCITY_CONTROL_Mode
for i in range(0,len(DXL_ID)):
    Set_Dynamixel_Operating_mode(DXL_ID[i], VELOCITY_CONTROL_Mode)

# velocity mode test
Send_Dynamixel_Velocity(dxl_goal_velocity)
time.sleep(2)
# Read_Dynamixel_Velocity()
# time.sleep(0.001)
Send_Dynamixel_Velocity(0)
time.sleep(0.5)

# Set dynamixel operating mode to CURRENT_CONTROL_MODE
for i in range(0,len(DXL_ID)):
    Set_Dynamixel_Operating_mode(DXL_ID[i], CURRENT_CONTROL_MODE)

# current mode test
Send_Dynamixel_Current(dxl_goal_current)
time.sleep(2)
# Read_Dynamixel_Current()
Send_Dynamixel_Current(0)
time.sleep(0.5)

# Set dynamixel operating mode to POSITION_CONTROL_Mode
for i in range(0,len(DXL_ID)):
    Set_Dynamixel_Operating_mode(DXL_ID[i], POSITION_CONTROL_Mode)

# dynamixel homing
Dynamixel_Homing()
time.sleep(1)

# disable torque
for i in range(0,len(DXL_ID)):
    Set_Dynamixel_Torque(DXL_ID[i],TORQUE_DISABLE)
time.sleep(0.1)

# Control table backup
for i in range(0,len(DXL_ID)):
    Control_Table_Backup(DXL_ID[i])

# Close port
portHandler.closePort()
