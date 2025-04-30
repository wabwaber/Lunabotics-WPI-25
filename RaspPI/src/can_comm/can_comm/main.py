from copy import deepcopy
import can
import rclpy
import time
import numpy as np
from rclpy.node import Node
from motor_comm.msg import SpeedReturn
from motor_comm.msg import MotorRequest
from enum import Enum
from drivetrain_controller.msg import JetsonDrivetrainCommand

can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'can0'
can.rc['bitrate'] = 500000

#https://python-can.readthedocs.io/en/stable/

bus = can.Bus()

""" This entire node and package exists to communicate with the CAN bus motors
this is because the firmware being used only has a python library. The latency won't be great in terms of requests to action times."""
class motor(Enum):
    DRIVE_FRONT_LEFT = 0 
    DRIVE_BACK_LEFT = 1 
    DRIVE_BACK_RIGHT = 2
    DRIVE_FRONT_RIGHT = 3 

BASE_CURRENT = 10
SPEED_KP = 750
SPEED_KI = 42
SPEED_SUMCAP = 380
POS_KP = 0.05
MAX_SPEED_SUM = 380
MAX_MOTOR_CURRENT = 16384
RECIEVE_TIMEOUT = 5.0 #timeout to recieve a response (in seconds)
MAX_ALLOWABLE_CURRENT_SLOPE = 0 #TODO

LOOP_TIMER_LENGTH = 0.005 #in seconds

class CAN_motor(Node):
    can_id = 0x200 #first 4 controllers, use 0x1FF if using the last 4
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
    loopCount = 0
    loopUntilLog = 5
    motor_temps = [0, 0, 0, 0]
    is_recovering = False
    curr_currents = np.array([0, 0, 0, 0])
    curr_timestamp = np.array([0, 0, 0, 0])
    prev_timestamp = np.array([0, 0, 0, 0])
    prev_currents = np.array([0, 0, 0, 0])
    num_of_failed_reads = 0
    max_failed_reads = 4

    def __init__(self):
        super().__init__('can_motor_communicator')
        self.speedPub = self.create_publisher(SpeedReturn, '/mooncake/motor_speed', 10)
        self.requestSub = self.create_subscription(MotorRequest, '/mooncake/can_motor_requests', self.motor_req_callback, 10)
        self.loopTimer = self.create_timer(0.05, self.timer_callback) #will run 20 times per second
        self.recoveryPub = self.create_publisher(JetsonDrivetrainCommand, '/mooncake/driveCommand', 10)

    def motor_req_callback(self, msg : MotorRequest):
        if msg.has_drive:
            self.requested_speeds = [msg.fl_drive, msg.bl_drive, msg.br_drive, msg.fr_drive]

    def timer_callback(self):
        if self.is_recovering: #if we are recovering IE this is our second time through this timer
            return #exit as the drivetrain controller will tell us what to do.
            
        else:
            #if we either don't have any readings yet or we only have the first
            if((self.prev_currents[0:] == 0 and self.curr_currents[0:] != 0) or (self.prev_currents[0:] == 0 and self.curr_currents[0:] != 0)):
                return #continue
            else:
                #otherwise we need to compare the two and see the slope
                #also I am vectorizing the arrays using numpy as it is far faster than looping through it. (at least as fast as python can be :p)
                dy = self.curr_currents[0:] - self.prev_currents[0:]
                dx = self.curr_timestamp[0:] - self.prev_timestamp[0:]
                slopes = np.array(dy[0:] / dx[0:])
                if slopes[0:] >= MAX_ALLOWABLE_CURRENT_SLOPE: #if any slopes are above the allowable amount
                    self.is_recovering = True #set the flag to true
                    self.set_currents([0,0,0,0]) #zero currents
                    msg = JetsonDrivetrainCommand() 
                    msg.to_state = "RECOVERY"
                    self.recoveryPub.publish(msg) #inform the drivetrain that we need to run the recovery process
                    return #end of function if we need to recover
            self.is_recovering = False #we we reach here then we should be all good and wont need to recover, so make sure the recovery flag is set to False

        self.doPID() #do the PID (happens 20 times a second)

        if self.loopCount >= self.loopUntilLog: #will do the following every quarter of a second 4 times per second
            self.loopCount = 0 #reset loop count
            self.get_logger().info("Speeds are " + self.curr_speeds)
            msg = SpeedReturn()
            msg.fl_drive = self.curr_speeds[0]
            msg.bl_drive = self.curr_speeds[1]
            msg.br_drive = self.curr_speeds[2]
            msg.fr_drive = self.curr_speeds[3]
            msg.location = "drive_motors"
            msg.fl_temp = self.motor_temps[0]
            msg.bl_temp = self.motor_temps[1]
            msg.br_temp = self.motor_temps[2]
            msg.fr_temp = self.motor_temps[3]
            self.speedPub.publish(msg=msg)

    #does the PID loop things, if you don't know what that is it essentialy changes the motors 'effort' relative to a goal. PID stands for Proportional Intergal Derivative as well. if you are a pure CS student reading this I would reccomend asking one of the RBE majors.
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
        listOSpeeds = [None, None, None, None]
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
                self.motor_temps[msgFromController.arbitration_id - self.can_id] = msgFromController[6]
                #4&5 are torque current H&L byte
                self.prev_currents = deepcopy(self.curr_currents)
                self.prev_timestamp = deepcopy(self.curr_timestamp) #copy over the timestamps and currents 
                self.curr_currents[x] = deepcopy(((msgFromController.data[4] & 0x0F) << 8) | msgFromController.data[5])
                self.curr_timestamp[x] = time.time() * 1000 #get the current timestamp in miliseconds
                self.num_of_failed_reads = 0 #reset failed reads counter in case this is a nested call
            except can.exceptions.CanOperationError or can.exceptions.CanTimeoutError:
                #we should never get here but if we do
                if self.num_of_failed_reads >= self.max_failed_reads: #if we have tried the max number of times
                    return can.exceptions.CanTimeoutError #throw an error
                return self.getSpeeds() # otherwise call the function again
        return listOSpeeds
    
    #as a side note to anyone looking at the code, this funciton is not great as it has the chance to block for 5 seconds. Idealy it would be handled as the information came in but the library doesnt seem to handle that kind of funcitonality :(
    def getSpeed(self, motor : motor):# get speed of one specific motor
        bus.set_filters([{"can_id": self.can_id + motor, "can_mask": 0x1FFFFF00}]) #get only messages from the motor we are interested in
        msg = bus.recv(timeout=RECIEVE_TIMEOUT) #wait for the message
        speed = ((msg.data[2] & 0x0F) << 8) | msg.data[3] #get the number
        bus.set_filters(filters=None) #reset filters
        return speed #and return

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
                bytes(self.set_currents[motor.DRIVE_BACK_LEFT] / self.can_conversion_factor),
                bytes(self.set_currents[motor.DRIVE_BACK_LEFT] % self.can_conversion_factor),
                bytes(self.set_currents[motor.DRIVE_BACK_RIGHT] / self.can_conversion_factor),
                bytes(self.set_currents[motor.DRIVE_BACK_RIGHT] % self.can_conversion_factor),
                bytes(self.set_currents[motor.DRIVE_FRONT_RIGHT] / self.can_conversion_factor),
                bytes(self.set_currents[motor.DRIVE_FRONT_RIGHT] % self.can_conversion_factor)
            ])
        bus.send(msg=msg) #send the message
        
    def setCurrents(self, currents : list):
        if len(currents) < 4: #check to ensure all currents are there
            return
        else:
            self.set_currents = deepcopy(currents)
            bus.send(can.Message( #create and send this msg
                arbitration_id=self.can_id,
                dlc=self.can_dlc,
                data=(
                [
                    bytes(self.set_currents[motor.DRIVE_FRONT_LEFT] / self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_FRONT_LEFT] % self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_BACK_LEFT] / self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_BACK_LEFT] % self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_BACK_RIGHT] / self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_BACK_RIGHT] % self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_FRONT_RIGHT] / self.can_conversion_factor),
                    bytes(self.set_currents[motor.DRIVE_FRONT_RIGHT] % self.can_conversion_factor)
                ])
                
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