from copy import deepcopy
import can
import rclpy
from rclpy.node import Node
from motor_comm.msg import SpeedReturn
from motor_comm.msg import MotorRequest
from enum import Enum

can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'vcan0'
can.rc['bitrate'] = 500000

bus = can.Bus()

""" This entire node and package exists to communicate with the CAN bus motors
this is because the firmware being used only has a python library. The latency won't be great in terms of requests to action times."""
class motor(Enum):
    DRIVE_FRONT_LEFT = 0 
    DRIVE_FRONT_RIGHT = 1 
    DRIVE_BACK_LEFT = 2 
    DRIVE_BACK_RIGHT = 3

BASE_CURRENT = 10
SPEED_KP = 750
SPEED_KI = 42
SPEED_SUMCAP = 380
POS_KP = 0.05
MAX_SPEED_SUM = 0
MAX_MOTOR_CURRENT = 16384
RECIEVE_TIMEOUT = 5.0 #timeout to recieve a response (in seconds)

LOOP_TIMER_LENGTH = 0.005 #in seconds

class CAN_motor(Node):
    can_id = 0x200 #first 4 controllers use 0x1FF if using the last 4
    can_dlc = 8
    can_conversion_factor = 256
    set_currents = [0, 0, 0, 0] 
    prevAngles = [0, 0, 0, 0]
    errors = [0, 0, 0, 0]
    prevErrors = [0, 0, 0, 0]
    requested_speeds = [0, 0, 0, 0]
    curr_speeds = [0, 0, 0, 0]
    displacements = [0, 0, 0, 0]
    sums = [0, 0, 0, 0]


    def __init__(self):
        super().__init__('can_motor_communicator')
        self.speedPub = self.create_publisher(SpeedReturn, '/mooncake/motor_speed')
        self.requestSub = self.create_subscription(MotorRequest, '/mooncake/can_motor_requests')
        

    def motor_req_callback(self):
        None #TODO
    
    def timer_callback(self):
        None #TODO
    

    
    def doPID(self):
        newCurrent = []
        for x in motor:
            self.errors[x] = self.requested_speeds[x] - self.curr_speeds[x]
            self.sums[x] = min(MAX_SPEED_SUM, max(-MAX_SPEED_SUM, self.sums[x] + self.errors[x])) #basically constrains the value between the MAX and MIN currents set above,
            if self.requested_speeds[x] == 0:
                newCurrent.append(0)
                self.sums[x] = 0
            elif self.requested_speeds > 0:
                newCurrent.append(BASE_CURRENT + SPEED_KP * self.errors[x] + SPEED_KI * self.sums[x])
            else:
                newCurrent.append(-BASE_CURRENT + SPEED_KP * self.errors[x] + SPEED_KI * self.sums[x])
        self.prevErrors = deepcopy(self.errors)
        self.set_currents(newCurrent)
    
    def getSpeeds(self): #get all speeds and return
        listOSpeeds = [] #TODO
        #the ENUM is the ID for the motor
        #reading is 0x200 + ID (ID being the motor controller ID {setting ID in the datasheet}, ONLY for IDs 1-4)
        #0x1FF + ID for IDs 5-8
        #frequency is 1kHz by default
        #mechanical angle value is 0-8191
        #coressponding angle is 0-360
        for x in motor: #for each motor
            try:
                msgFromController : can.Message = bus.recv(timeout=RECIEVE_TIMEOUT)
                self.curr_speeds[msgFromController.arbitration_id - self.can_id] = ((msgFromController.data[2] & 0x0F) << 8) | msgFromController.data[3] #DO WE NEED TO USE BITWISE & 0x0F HERE?
                listOSpeeds.append(self.curr_speeds[msgFromController.arbitration_id - self.can_id])
            except can.exceptions.CanOperationError or can.exceptions.CanTimeoutError:
                #we should never get here but if we do
                return self.getSpeeds() #call the function again          
        return listOSpeeds
    
    def getSpeed(self, motor : motor):# get speed of one specific motor
        speed = 0 #TODO
        
    #sets the motor currents, changing the requested one and leaving the rest the same
    #we are also having to send out all the currents because the CAN controllers will take their section of data from the list and use that
    def setCurrent(self, current: int, Givenmotor: motor):
        self.set_currents[Givenmotor] = current
        msg = can.Message(
            arbitration_id=self.can_id, 
            dlc=self.can_dlc, 
            data=[
                bytes(self.set_currents[motor.DRIVE_FRONT_LEFT] / self.can_conversion_factor),
                bytes(self.set_currents[motor.DRIVE_FRONT_LEFT] % self.can_conversion_factor),
                bytes(self.set_currents[motor.DRIVE_FRONT_RIGHT] / self.can_conversion_factor),
                bytes(self.set_currents[motor.DRIVE_FRONT_RIGHT] % self.can_conversion_factor),
                bytes(self.set_currents[motor.DRIVE_BACK_RIGHT] / self.can_conversion_factor),
                bytes(self.set_currents[motor.DRIVE_BACK_RIGHT] % self.can_conversion_factor),
                bytes(self.set_currents[motor.DRIVE_BACK_LEFT] / self.can_conversion_factor),
                bytes(self.set_currents[motor.DRIVE_BACK_LEFT] % self.can_conversion_factor)
            ])
        bus.send(msg=msg) #send the message
        
    
    def setCurrents(self, currents : list):
        if len(currents) <= 0: #check to ensure all currents are there
            return
        else:
            self.set_currents = deepcopy(currents)
            bus.send(can.Message( #create and send this msg
                arbitration_id=self.can_id,
                dlc=self.can_dlc,
                data=[
                    bytes(self.set_currents[motor.DRIVE_FRONT_LEFT] / self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_FRONT_LEFT] % self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_FRONT_RIGHT] / self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_FRONT_RIGHT] % self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_BACK_RIGHT] / self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_BACK_RIGHT] % self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_BACK_LEFT] / self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_BACK_LEFT] % self.can_conversion_factor)
                ]
                )
            )

        
    

def main(args=None):
    rclpy.init(args=args)
    motor_con = CAN_motor()
    rclpy.spin(motor_con)
    motor_con.destroy_node()
    rclpy.shutdown


if __name__ == "__main__":
    main()