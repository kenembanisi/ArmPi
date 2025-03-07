#!/usr/bin/env python3
# encoding: utf-8
import time
import serial
import pigpio
import ctypes
#幻尔科技总线舵机通信#

LOBOT_SERVO_FRAME_HEADER         = 0x55
LOBOT_SERVO_MOVE_TIME_WRITE      = 1
LOBOT_SERVO_MOVE_TIME_READ       = 2
LOBOT_SERVO_MOVE_TIME_WAIT_WRITE = 7
LOBOT_SERVO_MOVE_TIME_WAIT_READ  = 8
LOBOT_SERVO_MOVE_START           = 11
LOBOT_SERVO_MOVE_STOP            = 12
LOBOT_SERVO_ID_WRITE             = 13
LOBOT_SERVO_ID_READ              = 14
LOBOT_SERVO_ANGLE_OFFSET_ADJUST  = 17
LOBOT_SERVO_ANGLE_OFFSET_WRITE   = 18
LOBOT_SERVO_ANGLE_OFFSET_READ    = 19
LOBOT_SERVO_ANGLE_LIMIT_WRITE    = 20
LOBOT_SERVO_ANGLE_LIMIT_READ     = 21
LOBOT_SERVO_VIN_LIMIT_WRITE      = 22
LOBOT_SERVO_VIN_LIMIT_READ       = 23
LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE = 24
LOBOT_SERVO_TEMP_MAX_LIMIT_READ  = 25
LOBOT_SERVO_TEMP_READ            = 26
LOBOT_SERVO_VIN_READ             = 27
LOBOT_SERVO_POS_READ             = 28
LOBOT_SERVO_OR_MOTOR_MODE_WRITE  = 29
LOBOT_SERVO_OR_MOTOR_MODE_READ   = 30
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = 31
LOBOT_SERVO_LOAD_OR_UNLOAD_READ  = 32
LOBOT_SERVO_LED_CTRL_WRITE       = 33
LOBOT_SERVO_LED_CTRL_READ        = 34
LOBOT_SERVO_LED_ERROR_WRITE      = 35
LOBOT_SERVO_LED_ERROR_READ       = 36

pi = pigpio.pi()  # 初始化 pigpio库
# serialHandle = serial.Serial("/dev/ttyAMA0", 115200)  # 初始化串口， 波特率为115200
serialHandle = serial.Serial("/dev/ttyS0", 115200, timeout=1)  # 初始化串口， 波特率为115200

rx_pin = 4
tx_pin = 27

def portInit():  # 配置用到的IO口
    pi.set_mode(rx_pin, pigpio.OUTPUT)  # 配置RX_CON 即 GPIO17 为输出
    pi.write(rx_pin, 0)
    pi.set_mode(tx_pin, pigpio.OUTPUT)  # 配置TX_CON 即 GPIO27 为输出
    pi.write(tx_pin, 1)

portInit()

def portWrite():  # 配置单线串口为输出
    pi.write(tx_pin, 1)  # 拉高TX_CON 即 GPIO27
    pi.write(rx_pin, 0)  # 拉低RX_CON 即 GPIO17

def portRead():  # 配置单线串口为输入
    pi.write(rx_pin, 1)  # 拉高RX_CON 即 GPIO17
    pi.write(tx_pin, 0)  # 拉低TX_CON 即 GPIO27

def portRest():
    time.sleep(0.1)
    serialHandle.close()
    pi.write(rx_pin, 1)
    pi.write(tx_pin, 1)
    serialHandle.open()
    time.sleep(0.1)

def checksum(buf):
    # 计算校验和
    sum = 0x00
    for b in buf:  # 求和
        sum += b
    sum = sum - 0x55 - 0x55  # 去掉命令开头的两个 0x55
    sum = ~sum  # 取反
    return sum & 0xff

def serial_serro_wirte_cmd(id=None, w_cmd=None, dat1=None, dat2=None):
    '''
    写指令
    :param id:
    :param w_cmd:
    :param dat1:
    :param dat2:
    :return:
    '''
    portWrite()
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(id)
    # 指令长度
    if dat1 is None and dat2 is None:
        buf.append(3)
    elif dat1 is not None and dat2 is None:
        buf.append(4)
    elif dat1 is not None and dat2 is not None:
        buf.append(7)

    buf.append(w_cmd)  # 指令
    # 写数据
    if dat1 is None and dat2 is None:
        pass
    elif dat1 is not None and dat2 is None:
        buf.append(dat1 & 0xff)  # 偏差
    elif dat1 is not None and dat2 is not None:
        buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])  # 分低8位 高8位 放入缓存
        buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])  # 分低8位 高8位 放入缓存
    # 校验和
    buf.append(checksum(buf))
    # for i in buf:
    #     print('%x' %i)
    serialHandle.write(buf)  # 发送

def serial_servo_read_cmd(id=None, r_cmd=None):
    '''
    发送读取命令
    :param id:
    :param r_cmd:
    :param dat:
    :return:
    '''
    portWrite()
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(id)
    buf.append(3)  # 指令长度
    buf.append(r_cmd)  # 指令
    buf.append(checksum(buf))  # 校验和
    serialHandle.write(buf)  # 发送
    time.sleep(0.00034)

def serial_servo_get_rmsg(cmd):
    serialHandle.flushInput()  # Clear input buffer
    portRead()  # Configure serial for reading
    time.sleep(0.005)  # Allow time for data to arrive
    
    count = serialHandle.inWaiting()  # Check available bytes
    # print(f"[DEBUG] Bytes waiting in buffer: {count}")  # Debug print
    
    if count != 0:
        recv_data = serialHandle.read(count)  # Read available bytes
        # print(f"[DEBUG] Raw received data: {recv_data.hex()}")  # Print raw data in hex
        
        try:
            if recv_data[0] == 0x55 and recv_data[1] == 0x55 and recv_data[4] == cmd:
                dat_len = recv_data[3]
                serialHandle.flushInput()  # Clear input buffer
                
                if dat_len == 4:
                    return recv_data[5]
                elif dat_len == 5:
                    pos = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    return ctypes.c_int16(pos).value
                elif dat_len == 7:
                    pos1 = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    pos2 = 0xffff & (recv_data[7] | (0xff00 & (recv_data[8] << 8)))
                    return ctypes.c_int16(pos1).value, ctypes.c_int16(pos2).value
            else:
                # print("[DEBUG] Data format incorrect")
                reset_serial()
                return None

        except Exception as e:
            print(f"Error parsing response: {e}")
            return None

    else:
        # print("[DEBUG] No data received")
        reset_serial()
        serialHandle.flushInput()
        return None

def reset_serial():
    # print("Resetting serial connection...")
    serialHandle.close()
    time.sleep(0.01)
    serialHandle.open()
    time.sleep(0.01)
