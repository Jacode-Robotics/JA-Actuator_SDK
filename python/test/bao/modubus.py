import time
import serial

from .port_pack import *


ID                  = 0
FUNCTION            = 1
ADDRESS             = 2
DATA_CONTENT        = 4
DATA_LENGTH         = 4
CRC                 = 8
READ                = 3
WRITE               = 6


PORTNAME            = 'COM2'  
BAUDRATE            = 115200  

serial_port = SerialPort(PORTNAME, BAUDRATE)

def crc16(buffer):

    temp = 0xFFFF  # 初始值
    for byte in buffer:  # 处理字节序列中的每一个字节
        temp ^= byte
        for _ in range(8):  # 单字节位移数
            if (temp & 0x01) == 0:
                temp >>= 1
            else:
                temp >>= 1
                temp ^= 0xA001
    return temp

def format_hex_last(num):
    # 将整数转换为8位十六进制字符串，不足8位的部分用0填充
    hex_str = f"{num:08x}"
    # 将8位十六进制字符串分割成4个2位的十六进制数
    parts = [hex_str[i:i+2] for i in range(0, 8, 2)]
    # 将分割后的十六进制数重新排列，使最低位在最后
    parts = parts[:-1] + [parts[-1]]
    # 将重新排列后的十六进制数格式化为列表形式
    formatted_parts = [f"0x{part}" for part in parts]
    return formatted_parts

def write(id,address,data):
    pack = [0] * 10

    pack[ID]                = id
    pack[FUNCTION]          = WRITE

    location = address.zfill(4)
    location_int = int(location, 16)
    address_1 = (location_int >> 8) & 0xff
    address_2 = location_int & 0xff

    pack[ADDRESS]           = address_1
    pack[ADDRESS + 1]       = address_2

    if data >= 0:
        hex_data = format_hex_last(data)
        for i in range(0,len(hex_data)):
            hex_data_int = int(hex_data[i],16)

            pack[DATA_CONTENT + i] = hex_data_int

    elif data < 0:  
        max_uint32 = (1 << 32) - 1  
        # 计算补码
        complement = max_uint32 + 1 + data
        # 转换为十六进制
        hex_data = hex(complement)[2:]
        data_32bit_int = int(hex_data,16)
        processing_data =[
        (data_32bit_int >> 24) & 0xff,
        (data_32bit_int >> 16) & 0xff,
        (data_32bit_int >> 8) & 0xff,
        data_32bit_int & 0xff
        ]
        final_data = [f"0x{byte:02x}" for byte in processing_data]
        for i in range(0,len(final_data)):
            final_data_int = int(final_data[i],16)

            pack[DATA_CONTENT + i] = final_data_int

    crc_list = [0] * 8
    crc_list = pack[:-2]
    crc_int = crc16(crc_list)
    crc1 = (crc_int >> 8) & 0xff
    crc2 = crc_int & 0xff

    pack[CRC]               = crc1
    pack[CRC + 1]           = crc2

    
    serial_port.writePort(pack)
    response = serial_port.readPort(10)
    if response is not None:
        response_list = [f"0x{response.hex()[i:i+2]}" for i in range(0, len(response.hex()), 2)]
        print(response_list)
    else:
        print("没有接收到响应。")

    return pack

def read(id,address):
    pack = [0] * 10

    pack[ID]                = id
    pack[FUNCTION]          = READ

    location = address.zfill(4)
    location_int = int(location, 16)
    address_1 = (location_int >> 8) & 0xff
    address_2 = location_int & 0xff

    pack[ADDRESS]               = address_1
    pack[ADDRESS + 1]           = address_2

    pack[DATA_LENGTH]           = 0X00
    pack[DATA_LENGTH + 1]       = 0X00
    pack[DATA_LENGTH + 2]       = 0X00
    pack[DATA_LENGTH + 3]       = 0X02

    crc_list = [0] * 8
    crc_list = pack[:-2]
    crc_int = crc16(crc_list)
    crc1 = (crc_int >> 8) & 0xff
    crc2 = crc_int & 0xff

    pack[CRC]               = crc1
    pack[CRC + 1]           = crc2
    try:
        # 假设pack列表已经正确填充
        crc_list = pack[:-2]  # 除去CRC部分
        crc_int = crc16(crc_list)
        pack[CRC] = crc_int >> 8
        pack[CRC + 1] = crc_int & 0xff
    except Exception as e:
        print(f"CRC计算失败: {e}")
        return None
    
    serial_port.writePort(pack)
    response = serial_port.readPort(10)
    if response is not None:
        response_list = [f"0x{response.hex()[i:i+2]}" for i in range(0, len(response.hex()), 2)]
    else:
        print("没有接收到响应。")

    read_data1 = response_list[DATA_CONTENT]
    read_data2 = response_list[DATA_CONTENT + 1]
    read_data3 = response_list[DATA_CONTENT + 2]
    read_data4 = response_list[DATA_CONTENT + 3]

    read_data1 = read_data1[2:]
    read_data2 = read_data2[2:]
    read_data3 = read_data3[2:]
    read_data4 = read_data4[2:]

    hex_read_data = '0x' + read_data1 + read_data2 + read_data3 + read_data4
    read_data = int(hex_read_data,16)

    return read_data,response_list

def send_receive_modbus_rtu_request(ser, request):
    try:
        if not ser.is_open:
            ser.open()
        ser.write(bytearray(request)) 
        time.sleep(0.1) 
        response = ser.read(ser.in_waiting)
        if response:
            return response
        else:
            print("没有接收到响应。")
            return None
    except serial.SerialException as e:
        print(f"串行通信错误: {e}")
        return None
    except Exception as e:
        print(f"发送或接收数据时发生未知错误: {e}")
        return None

