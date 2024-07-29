from smbus3 import SMBus
# |  0  |       1     |      2     |     3      |    4, 5, 6, 7  |          8         |     9    |    10   |    11   |     12      |
# | mod |  position-p | position-i | position-d | position-point | position-max-speed |  speed-p | speed-i | speed-d | speed-point |


NORMAL_MODE            =0x00
POSITION_MODE          =0x01
SPEED_MODE             =0x02
IAP_UPDATE_MODE        =0x03

class M5Module4EncoderMotorController:
    def __init__(self, module_addr = 0x24) -> None:
        self.MODULE_4ENCODERMOTOR_ADDR                    = module_addr
        self.MODULE_4ENCODERMOTOR_SERVO_ANGLE_ADDR        =0x00
        self.MODULE_4ENCODERMOTOR_SERVO_PULSE_ADDR        =0x10
        self.MODULE_4ENCODERMOTOR_PWM_DUTY_ADDR           =0x20
        self.MODULE_4ENCODERMOTOR_ENCODER_ADDR            =0x30
        self.MODULE_4ENCODERMOTOR_SPEED_ADDR              =0x40
        self.MODULE_4ENCODERMOTOR_ADC_8BIT_REG            =0xA0
        self.MODULE_4ENCODERMOTOR_ADC_12BIT_REG           =0xB0
        self.JUMP_TO_BOOTLOADER_REG                       =0xFD
        self.UPGRADE_BOOTLOADER_REG                       =0xE0
        self.MODULE_4ENCODERMOTOR_FIRMWARE_VERSION_ADDR   =0xFE
        self.MODULE_4ENCODERMOTOR_BOOTLOADER_VERSION_ADDR =0xFC
        self.MODULE_4ENCODERMOTOR_I2C_ADDRESS_ADDR        =0xFF
        self.MODULE_4ENCODERMOTOR_CONFIG_ADDR             =0x50
        self.MODULE_4ENCODERMOTOR_CURRENT_ADDR            =0x90
        
    def checkIndex(self, index):
        return min(max(0,index),3)
    
    def setMode(self, index, mode):
        with SMBus(1) as bus:
            bus.write_byte_data(self.MODULE_4ENCODERMOTOR_ADDR, self.MODULE_4ENCODERMOTOR_CONFIG_ADDR + (0x10 * index),mode)

    def getEncoderValue(self, index): 
        index = self.checkIndex(index)
        addr  = self.MODULE_4ENCODERMOTOR_ENCODER_ADDR + 4 * index
        with SMBus(1) as bus:
            read_buf = bus.read_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,4)

        return (read_buf[0] << 24) | (read_buf[1] << 16) | (read_buf[2] << 8) | read_buf[3]
    
    def getEncoderValues(self): 
        addr  = self.MODULE_4ENCODERMOTOR_ENCODER_ADDR
        values=[]
        with SMBus(1) as bus:
            read_buf = bus.read_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,16)
        for i in range(0, 16, 4):
            value = (read_buf[i] << 24) | (read_buf[i+1] << 16) | (read_buf[i+2] << 8) | read_buf[i+3]
            values.append(value)
        return values
    
    def setEncoderValue(self, index, encoder):
        index        = self.checkIndex(index)
        addr         = self.MODULE_4ENCODERMOTOR_ENCODER_ADDR + 4 * index
        write_buf = [0,0,0,0]
        write_buf[0] = encoder >> 24
        write_buf[1] = encoder >> 16
        write_buf[2] = encoder >> 8
        write_buf[3] = encoder & 0xff
        with SMBus(1) as bus:
           bus.write_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,write_buf)

    def setEncoderValues(self, encoder):
        addr         = self.MODULE_4ENCODERMOTOR_ENCODER_ADDR
        write_buf = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        for i in range(4):
            write_buf[i*4 + 0] = encoder[i] >> 24
            write_buf[i*4 + 1] = encoder[i] >> 16
            write_buf[i*4 + 2] = encoder[i] >> 8
            write_buf[i*4 + 3] = encoder[i] & 0xff
        with SMBus(1) as bus:
           bus.write_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,write_buf)

    def setMotorSpeed(self, index, duty):
        index = self.checkIndex(index)
        addr  = self.MODULE_4ENCODERMOTOR_PWM_DUTY_ADDR + index
        with SMBus(1) as bus:
            bus.write_byte_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,duty)

    def setMotorSpeeds(self,duty_array):
        addr  = self.MODULE_4ENCODERMOTOR_PWM_DUTY_ADDR 
        with SMBus(1) as bus:
            bus.write_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,duty_array)

    def getMotorSpeed(self, index):
        
        index = self.checkIndex(index)
        addr  = self.MODULE_4ENCODERMOTOR_PWM_DUTY_ADDR + index
        with SMBus(1) as bus:
            read_buf = bus.read_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,1)

        return read_buf[0]
    
    def getMotorSpeed20MS(self, index):
        
        index = self.checkIndex(index)
        addr  = self.MODULE_4ENCODERMOTOR_SPEED_ADDR + index
        with SMBus(1) as bus:
            read_buf = bus.read_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,1)
        return read_buf[0]

    def setPositionPID(self, index, kp, ki, kd):
        write_buf = [0, 0, 0]
        index = self.checkIndex(index)
        addr         = self.MODULE_4ENCODERMOTOR_CONFIG_ADDR + index * 0x10 + 0x01
        write_buf[0] = kp
        write_buf[1] = ki
        write_buf[2] = kd
        with SMBus(1) as bus:
           bus.write_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,write_buf)
        
    def setPositionPoint(self, index, position_point):
        write_buf = [0, 0, 0, 0]

        index        = self.checkIndex(index)
        addr         = self.MODULE_4ENCODERMOTOR_CONFIG_ADDR + index * 0x10 + 0x04
        write_buf[0] = position_point & 0xff
        write_buf[1] = position_point >> 8
        write_buf[2] = position_point >> 16
        write_buf[3] = position_point >> 24

        with SMBus(1) as bus:
           bus.write_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,write_buf)
        
    def setPostionPIDMaxSpeed(self, index, max_pwm):
        
        index = self.checkIndex(index)
        addr  = self.MODULE_4ENCODERMOTOR_CONFIG_ADDR + index * 0x10 + 0x08
        with SMBus(1) as bus:
           bus.write_byte_data(self.MODULE_4ENCODERMOTOR_ADDR,addr, max_pwm)
       
    def setSpeedPID(self, index, kp, ki, kd):
        write_buf = [0, 0, 0]
        
        index = self.checkIndex(index)
        addr         = self.MODULE_4ENCODERMOTOR_CONFIG_ADDR + index * 0x10 + 0x09
        write_buf[0] = kp
        write_buf[1] = ki
        write_buf[2] = kd
        with SMBus(1) as bus:
           bus.write_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,write_buf)
    
    def setSpeedPoint(self, index, speed_point):
        index = self.checkIndex(index)
        addr  = self.MODULE_4ENCODERMOTOR_CONFIG_ADDR + index * 0x10 + 0x0c
        with SMBus(1) as bus:
           bus.write_byte_data(self.MODULE_4ENCODERMOTOR_ADDR,addr, speed_point)

    def setI2CAddress(self, addr):
        if (addr > 127): addr = 127

        reg       = self.MODULE_4ENCODERMOTOR_I2C_ADDRESS_ADDR
        write_buf = addr
        with SMBus(1) as bus:
           bus.write_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,write_buf)
        
    def getI2CAddress(self):
        addr = self.MODULE_4ENCODERMOTOR_I2C_ADDRESS_ADDR
        with SMBus(1) as bus:
            read_buf = bus.read_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,addr,1)
       
        return read_buf[0]
 

    def getMotorCurrent(self):
        with SMBus(1) as bus:
            read_buf = bus.read_i2c_block_data(self.MODULE_4ENCODERMOTOR_ADDR,self.MODULE_4ENCODERMOTOR_CURRENT_ADDR,4)
        
        c = int.from_bytes(read_buf, byteorder='little')

        return c