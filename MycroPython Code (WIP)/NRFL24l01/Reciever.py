import ustruct as struct
import utime
from machine import Pin, SPI, I2C
from nrf24l01 import NRF24L01
from micropython import const
from Kalman import KalmanAngle
import smbus
import time
import math

# delay between receiving a message and waiting for the next message
POLL_DELAY = const(15)
# Delay between receiving a message and sending the response
SEND_DELAY = const(10)
# Pico pins
myPins = {"spi": 0, "miso": 4, "mosi": 7, "sck": 6, "csn": 5, "ce": 12}
# Addresses
pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")
csn = Pin(myPins["csn"], mode=Pin.OUT, value=1)
ce = Pin(myPins["ce"], mode=Pin.OUT, value=0)
nrf = NRF24L01(SPI(myPins["spi"]), csn, ce, payload_size=8)
nrf.open_tx_pipe(pipes[1])
nrf.open_rx_pipe(1, pipes[0])
nrf.start_listening()
print("nRF24L01 receiver; waiting for the first post...")

kalmanX = KalmanAngle()
kalmanY = KalmanAngle()

RestrictPitch = True    #Comment out to restrict roll to ±90deg
radToDeg = 57.2957786
kalAngleX = 0
kalAngleY = 0
#MPU6050 Registers + Addresses
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


#Read the gyro and acceleromater values from MPU6050
def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(DeviceAddress, SMPLRT_DIV, 7)

    #Write to power management register
    bus.write_byte_data(DeviceAddress, PWR_MGMT_1, 1)

    #Write to Configuration register
    #Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise due to vibration.) https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
    bus.write_byte_data(DeviceAddress, CONFIG, int('0000110',2))

    #Write to Gyro configuration register
    bus.write_byte_data(DeviceAddress, GYRO_CONFIG, 24)

    #Write to interrupt enable register
    bus.write_byte_data(DeviceAddress, INT_ENABLE, 1)


def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(DeviceAddress, addr)
        low = bus.read_byte_data(DeviceAddress, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
DeviceAddress = 0x68   # MPU6050 device address

MPU_Init()

time.sleep(1)
#Read Accelerometer raw value
accX = read_raw_data(ACCEL_XOUT_H)
accY = read_raw_data(ACCEL_YOUT_H)
accZ = read_raw_data(ACCEL_ZOUT_H)

#print(accX,accY,accZ)
#print(math.sqrt((accY**2)+(accZ**2)))
if (RestrictPitch):
    roll = math.atan2(accY,accZ) * radToDeg
    pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
else:
    roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
    pitch = math.atan2(-accX,accZ) * radToDeg
print(roll)
kalmanX.setAngle(roll)
kalmanY.setAngle(pitch)
gyroXAngle = roll;
gyroYAngle = pitch;
compAngleX = roll;
compAngleY = pitch;

timer = time.time()
flag = 0

while True:
    if(flag >100): #Problem with the connection
        print("There is a problem with the connection")
        flag=0
        continue
    try:
        #Read Accelerometer raw value
	    accX = read_raw_data(ACCEL_XOUT_H)
	    accY = read_raw_data(ACCEL_YOUT_H)
	    accZ = read_raw_data(ACCEL_ZOUT_H)

	    #Read Gyroscope raw value
	    gyroX = read_raw_data(GYRO_XOUT_H)
	    gyroY = read_raw_data(GYRO_YOUT_H)
	    gyroZ = read_raw_data(GYRO_ZOUT_H)

	    dt = time.time() - timer
	    timer = time.time()

	    if (RestrictPitch):
	        roll = math.atan2(accY,accZ) * radToDeg
	        pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
	    else:
	        roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
	        pitch = math.atan2(-accX,accZ) * radToDeg

	    gyroXRate = gyroX/131
	    gyroYRate = gyroY/131

	    if (RestrictPitch):

	        if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
	            kalmanX.setAngle(roll)
	            complAngleX = roll
	            kalAngleX   = roll
	            gyroXAngle  = roll
	        else:
	            kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

	        if(abs(kalAngleX)>90):
	            gyroYRate  = -gyroYRate
	            kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)
	    else:

	        if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
	            kalmanY.setAngle(pitch)
	            complAngleY = pitch
	            kalAngleY   = pitch
	            gyroYAngle  = pitch
	        else:
	            kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)

	        if(abs(kalAngleY)>90):
	            gyroXRate  = -gyroXRate
	            kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

        #angle = (rate of change of angle) * change in time
	    gyroXAngle = gyroXRate * dt
	    gyroYAngle = gyroYAngle * dt

        #compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
	    compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
	    compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

	    if ((gyroXAngle < -180) or (gyroXAngle > 180)):
	        gyroXAngle = kalAngleX
	    if ((gyroYAngle < -180) or (gyroYAngle > 180)):
	        gyroYAngle = kalAngleY

        print("Angle X: " + str(kalAngleX)+"   " +"Angle Y: " + str(kalAngleY))
	    #print(str(roll)+"  "+str(gyroXAngle)+"  "+str(compAngleX)+"  "+str(kalAngleX)+"  "+str(pitch)+"  "+str(gyroYAngle)+"  "+str(compAngleY)+"  "+str(kalAngleY))
        time.sleep(0.005)

    except Exception as exc:
		flag += 1
    
    if nrf.any(): # we received something
        while nrf.any():
            buf = nrf.recv()
            counter = struct.unpack("i", buf)
            print("message received:", counter[0])
            utime.sleep_ms(POLL_DELAY) # delay before next listening
            
        response = counter[0]%2 # preparing the response
        #The % (modulo) operator yields the remainder from the division of the first argument by the second. The numeric arguments are first converted to a common type. A zero right argument raises the ZeroDivisionError exception. The arguments may be floating point numbers, e.g., 3.14%0.7 equals 0.34 (since 3.14 equals 4*0.7 + 0.34.) The modulo operator always yields a result with the same sign as its second operand (or zero); the absolute value of the result is strictly smaller than the absolute value of the second operand [2].
        utime.sleep_ms(SEND_DELAY) # Give the other Pico a brief time to listen
        nrf.stop_listening()
        try:
            nrf.send(struct.pack("i", response))
        except OSError:
            pass
        print("reply sent:", response)
        nrf.start_listening()