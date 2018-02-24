import numpy as np
import serial
import time

# Ignite, Start, Steer, Throttle
class ArduinoMap:

    def __init__(self, COM, baud):
        self.steeringBuffer = np.array([70,70,70,70])
        self.throttleBuffer = np.array([0,0,0,0])
        try:
            self.ser = serial.Serial(timeout=0)
            self.ser.port = COM
            self.ser.baudrate = baud
            self.ser.open()
        except:
            print("No Connection found on Serial COM")
            quit()
        # # self.gearlookup = [267, # P
        #                    312, # R
        #                    367, # N
        #                    417, # D
        #                    457, # 3
        #                    508, # 2
        #                    540] # L

    def readSerial(self):
        return self.ser.readline()

    def updateSteering(self, newSteer):
        """updates steering angle ( between -900 and 900 )"""

        self.steeringBuffer = np.append(self.steeringBuffer,np.clip(newSteer, 0, 140))
        self.steeringBuffer = np.delete(self.steeringBuffer,0)

    #def updateBrake(self, newBrake):
    #    """ updates braking percentage"""
    #    self.brake = np.clip(newBrake, 0.0, 1.0)

    def updateThrottle(self, newThrottle):
        """updates throttle as a percentage"""
        self.throttleBuffer = np.append(self.throttleBuffer,np.clip(newThrottle, 0, 90))
        self.throttleBuffer = np.delete(self.throttleBuffer,0)
    # def updateGear(self, newGear):
    #     """udates gear as a 0 indexed integer """
    #     self.gear = np.clip(newGear, 0, 5)
    #
    def updateStart(self, newStart):
         """Changes modes from autonomous to RC car """
         self.start = np.clip(newStart, 0, 1)

    def updateIgnition(self, newIgnition):
        """Using ignition relays to start car and starter motor """
        self.ignition = np.clip(newIgnition, 0, 1)
    #
    # def updateKill(self, newKill):
    #     """Changes mode from alive to dead """
    #     self.kill = np.clip(newKill, 0, 1)

    def update(self, newSteer, newThrottle, newStart, newIgnition):
        """Updates all with defaults """
        self.updateIgnition(newIgnition)
        self.updateStart(newStart)
        self.updateThrottle(newThrottle)
        self.updateSteering(newSteer)

    def convertAll(self):
        self.steeringArdu = int(round(np.average(self.steeringBuffer)))
        self.throttleArdu = int(round(np.average(self.throttleBuffer)))
        # self.gearArdu = self.gearlookup[self.gear]
        self.checksum = self.steeringArdu + self.throttleArdu

    def sendCommands(self):
        string = ""
        string += str(self.ignition)
        string += str(self.start)
        string += str(self.steeringArdu).zfill(4)
        # string += str(self.brakeArdu).zfill(4)
        string += str(self.throttleArdu).zfill(4)
        # string += str(self.gearArdu).zfill(4)
        # string += str(self.auto)
        string += str(self.checksum).zfill(4)
        string += "\n"
        print(string)
        try:
            self.ser.write(string.encode())
        except:
            print('SERIAL ERROR: Not able to send message')
            pass

    def arduinoSerial(self, newSteer, newThrottle, newStart, newIgnition):
        self.update( newSteer, newThrottle, newStart, newIgnition)
        self.convertAll()
        self.sendCommands()

    def Default(self):
        self.update(70, 0, 0, 0)
        self.convertAll()
        self.sendCommands()
