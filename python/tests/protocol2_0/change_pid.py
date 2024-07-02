import os
import struct
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

from dynamixel_sdk import * # Uses Dynamixel SDK library

# Control table address
ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
ADDR_POSITION_P_GAIN        = 532
ADDR_POSITION_I_GAIN        = 530
ADDR_POSITION_D_GAIN        = 528
BAUDRATE                    = 2000000

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0


# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

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

def Control_Table_Backup(ID):
    crc = 0x0000
 
    data = [0xFF,0xFF,0xFD,0x00,ID,0x08,0x00,0x20,0x01,0x43,0x54,0x52,0x4C,0X00,0X00]
    crc = update_crc(data)
    data[len(data)-2] = crc & 0xff
    data[len(data)-1] = (crc >> 8) & 0xff

    byte_data = bytes(data)

    written_bytes = portHandler.writePort(byte_data)

    if written_bytes != len(byte_data):
        print("Failed to write all bytes")
    else:
        print("Control_Table_Backup succeeded")

flag = True  
def on_key_pressed(event):
    global flag
    if event.name == 'esc':
        print("quit succeeded")
        flag = False  

keyboard.on_press(on_key_pressed)
# Open port
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

ID = []
for i in range(1,20):
    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, i)
    if dxl_comm_result != COMM_SUCCESS:
        continue
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%3d] ping Succeeded. Dynamixel model number : %d" % (i, dxl_model_number))
        ID.append(i)

print()

for i in range(0,len(ID)):
    dxl_position_p_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, ID[i], ADDR_POSITION_P_GAIN)
    dxl_position_p_gain = struct.unpack('i', struct.pack('I', dxl_position_p_gain))[0]
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d]  position_p_gain:%03d" % (ID[i],  dxl_position_p_gain))

    dxl_position_i_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, ID[i], ADDR_POSITION_I_GAIN)
    dxl_position_i_gain = struct.unpack('i', struct.pack('I', dxl_position_i_gain))[0]
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d]  position_i_gain:%03d" % (ID[i],  dxl_position_i_gain))

    dxl_position_d_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, ID[i], ADDR_POSITION_D_GAIN)
    dxl_position_d_gain = struct.unpack('i', struct.pack('I', dxl_position_d_gain))[0]
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d]  position_d_gain:%03d" % (ID[i],  dxl_position_d_gain))
    print()

while flag:
    alter_id = int(input("Need to change the PID's ID:"))
    dxl_position_p_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, alter_id, ADDR_POSITION_P_GAIN)
    dxl_position_p_gain = struct.unpack('i', struct.pack('I', dxl_position_p_gain))[0]
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d]  position_p_gain:%03d" % (alter_id,  dxl_position_p_gain))

    dxl_position_i_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, alter_id, ADDR_POSITION_I_GAIN)
    dxl_position_i_gain = struct.unpack('i', struct.pack('I', dxl_position_i_gain))[0]
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d]  position_i_gain:%03d" % (alter_id,  dxl_position_i_gain))

    dxl_position_d_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, alter_id, ADDR_POSITION_D_GAIN)
    dxl_position_d_gain = struct.unpack('i', struct.pack('I', dxl_position_d_gain))[0]
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d]  position_d_gain:%03d" % (alter_id,  dxl_position_d_gain))

    new_p = int(input("P:"))
    new_i = int(input("I:"))
    new_d = int(input("D:"))
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, alter_id, ADDR_POSITION_P_GAIN, new_p)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("change P:%d" %new_p)
    time.sleep(0.3)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, alter_id, ADDR_POSITION_I_GAIN, new_i)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("change I:%d" %new_i)
    time.sleep(0.3)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, alter_id, ADDR_POSITION_D_GAIN, new_d)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("change D:%d" %new_d)
    time.sleep(0.2)
    Control_Table_Backup(alter_id)
    print("Press ESC to exit")
    print("Press any key to continue")
    getch()
    
print('\n')
print('power-off save')
print('\n')

# Close port
portHandler.closePort()
