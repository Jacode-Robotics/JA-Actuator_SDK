from .port_pack import *

ID                  = 0
FUNCTION            = 1
ADDRESS             = 2
DATA_CONTENT        = 4
DATA_LENGTH         = 4
CRC                 = 8
READ                = 3
WRITE               = 6

COMMUNICATION_ABNORMALITY       = 1
VALUE_ERROR                     = 2
INDEX_ERROR                     = 3
TIME_OUT                        = 4

#CRC计算
def CRC16(buffer):

    temp = 0xFFFF 
    for byte in buffer:  
        temp ^= byte
        for _ in range(8):  
            if (temp & 0x01) == 0:
                temp >>= 1
            else:
                temp >>= 1
                temp ^= 0xA001
    return temp

def Error_handling(port,frequency,timeout):
    try:
        response = port.readPort(10, timeout)
        if response is not None:
            response_array = [f"0x{response.hex()[i:i+2]}" for i in range(0, len(response.hex()), 2)]
            return response_array

    except serial.SerialException as e:
        # 捕获串口通信异常，并打印错误信息
        # print(f"串口通信异常: {e}")
        return COMMUNICATION_ABNORMALITY
    except ValueError as e:
        # 捕获值错误，并打印错误信息
        # print(f"值错误: {e}")
        return VALUE_ERROR
    except IndexError as e:
        # 捕获索引错误，并打印错误信息
        # print(f"索引错误: {e}")
        return INDEX_ERROR
    except Exception as e:
        # 捕获其他所有异常，并打印错误信息
        # print(f"未知错误: {e}")
        return TIME_OUT

#将整数转换成元素为四个二位十六进制数的列表
def Hexadecimal_conversion(num):

    hex_str = f"{num:08x}"
    parts = [hex_str[i:i+2] for i in range(0, 8, 2)]
    parts = parts[:-1] + [parts[-1]]
    formatted_parts = [f"0x{part}" for part in parts]
    return formatted_parts

#写命令
def write(port,id,address,data):
    pack = [0] * 10
    error = 0

    pack[ID]                = id
    pack[FUNCTION]          = WRITE

    location = address.zfill(4)
    location_int = int(location, 16)
    address_1 = (location_int >> 8) & 0xff
    address_2 = location_int & 0xff

    pack[ADDRESS]           = address_1
    pack[ADDRESS + 1]       = address_2

    if data >= 0:
        hex_data = Hexadecimal_conversion(data)
        for i in range(0,len(hex_data)):
            hex_data_int = int(hex_data[i],16)

            pack[DATA_CONTENT + i] = hex_data_int

    elif data < 0:  
        max_uint32 = (1 << 32) - 1  
        complement = max_uint32 + 1 + data
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

    crc_array = [0] * 8
    crc_array = pack[:-2]
    crc_int = CRC16(crc_array)
    crc1 = (crc_int >> 8) & 0xff
    crc2 = crc_int & 0xff

    pack[CRC]               = crc1
    pack[CRC + 1]           = crc2
    try:
        crc_array = pack[:-2]
        crc_int = CRC16(crc_array)
        pack[CRC] = crc_int >> 8
        pack[CRC + 1] = crc_int & 0xff
    except Exception as e:
        print(f"CRC计算失败: {e}")
        return None
    
    port.writePort(pack)
    response = port.readPort(10)
    if response is not None and isinstance(response, bytes):
        response_array = [f"0x{response.hex()[i:i+2]}" for i in range(0, len(response.hex()), 2)]
    else:
        response_array  = None
        error           = response
    if error != 0 and error != 4:
        error = Error_handling(port,3,1.5)
        pass

    return response_array,error

#读命令
def read(port,id,address):
    pack = [0] * 10
    error = 0

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

    crc_array = [0] * 8
    crc_array = pack[:-2]
    crc_int = CRC16(crc_array)
    crc1 = (crc_int >> 8) & 0xff
    crc2 = crc_int & 0xff

    pack[CRC]               = crc1
    pack[CRC + 1]           = crc2
    try:
        crc_array = pack[:-2]
        crc_int = CRC16(crc_array)
        pack[CRC] = crc_int >> 8
        pack[CRC + 1] = crc_int & 0xff
    except Exception as e:
        print(f"CRC计算失败: {e}")
        return None
    
    port.writePort(pack)
    response = port.readPort(10)
    if response is not None and isinstance(response, bytes):
        response_array = [f"0x{response.hex()[i:i+2]}" for i in range(0, len(response.hex()), 2)]
        read_data1 = response_array[DATA_CONTENT]
        read_data2 = response_array[DATA_CONTENT + 1]
        read_data3 = response_array[DATA_CONTENT + 2]
        read_data4 = response_array[DATA_CONTENT + 3]

        read_data1 = read_data1[2:]
        read_data2 = read_data2[2:]
        read_data3 = read_data3[2:]
        read_data4 = read_data4[2:]

        hex_read_data = '0x' + read_data1 + read_data2 + read_data3 + read_data4
        read_data = int(hex_read_data,16)
    else:
        read_data       = None
        response_array  = None
        error           = response
    if error != 0 and error != 4:
        error = Error_handling(port,3,1.5)
        pass
    # else:
    #     read_data = None
    #     error = response
    #     error = Error_handling(port,3,1.5)

    return read_data, error, response_array
