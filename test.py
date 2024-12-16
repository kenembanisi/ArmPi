import smbus2 # install pip install smbus2
import time
import struct

# Define constants
MOTOR_TYPE_JGB37_520_12V_110RPM = 3 # the magnetic ring generates 44 pulses per revolution, combined with a gear reduction ratio of 90 Default

I2C_PORT = 1
ENCODER_MOTOR_MODULE_ADDR = 0x34
MOTOR_TYPE_ADDR = 20 #0x20
MOTOR_ENCODER_POLARITY_ADDR = 21 #0x21
ADC_BAT_ADDR = 0x00
MOTOR_ENCODER_TOTAL_ADDR = 60 #0x60
MOTOR_FIXED_PWM_ADDR = 31 #0x31 
MOTOR_FIXED_SPEED_ADDR = 51 #0x51 


#电机类型及编码方向极性
MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM
MotorEncoderPolarity = 0

bus = smbus2.SMBus(I2C_PORT)
speed1 = [15]*4
speed2 = [-15]*4
speed3 = [0,0,0,0]

pwm1 = [50,50,50,50]
pwm2 = [-100,-100,-100,-100]
pwm3 = [0,0,0,0]

def Motor_Init(): #电机初始化
    bus.write_byte_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_TYPE_ADDR, MotorType)  #设置电机类型 (Set motor type)
    time.sleep(0.5)
    bus.write_byte_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_ENCODER_POLARITY_ADDR, MotorEncoderPolarity)  #设置编码极性 (Set encoding polarity)

    print('Encoder motor driver module has been initialized!! \n')

def main():

    try:
        while True:
            # read battery data
            battery = bus.read_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, ADC_BAT_ADDR, 2)
            print("V = {0}mV".format(battery[0]+(battery[1]<<8)))
            
            # read encoder data
            Encode = struct.unpack('iiii',bytes(bus.read_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_ENCODER_TOTAL_ADDR,16)))
            print("Encode1 = {0}  Encode2 = {1}  Encode3 = {2}  Encode4 = {3}".format(Encode[0],Encode[1],Encode[2],Encode[3]))
            

            # PWM control (note: PWM control is a continuous control process, if there is a delay, it will interrupt the operation of the motor)
            #   bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_FIXED_PWM_ADDR,pwm2)
            
            
            #固定转速控制 (fixed speed control)
            # write motor speed data
            bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_FIXED_SPEED_ADDR,speed1)
            time.sleep(3)
            bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_FIXED_SPEED_ADDR,speed2)
            time.sleep(3)
    except KeyboardInterrupt:
        print('Stopping motors')
        bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_FIXED_SPEED_ADDR,speed3)
        


if __name__ == "__main__":
    Motor_Init()
    main()


# RESOURCES:
# 1. https://smbus2.readthedocs.io/en/latest/ 
# 2. https://www.abelectronics.co.uk/kb/article/1094/i2c-part-4-programming-i2c-with-python
# 3. https://drive.google.com/file/d/19BFlVI4pYvOb_Tf3MHPpHsck3rE9Yzpv/view