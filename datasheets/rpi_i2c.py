#RPi Pinouts

#I2C Pins 
#GPIO2 -> SDA
#GPIO3 -> SCL

#Import the Library Requreid 
import smbus2
import time

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus2.SMBus(1)

# This is the address we setup in the Arduino Program
#Slave Address 1
address = 0x05

#Slave Address 2
size=5
data_list = [2, 3, 4, 5, 0]

def readBytes():
	number = bus.read_i2c_block_data(address, 0, 7)
	return number



#Read from the Slaves 
data_list=readBytes()


print(data_list)

print("the enwqd")
#End of the Script

