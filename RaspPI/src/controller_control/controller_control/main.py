from inputs import get_gamepad
import threading
import time
from enum import Enum
import rclpy
from rclpy.node import Node
#from drivetrain_controller.msg import JetsonDrivetrainCommand
from motor_comm.msg import MotorRequest

class motor(Enum):
	DRIVE_FRONT_LEFT = 0 
	DRIVE_BACK_LEFT = 1 
	DRIVE_BACK_RIGHT = 2
	DRIVE_FRONT_RIGHT = 3 

class drive_mode(Enum):
	STRAIGHT = 0
	POINT_TURN = 1
	ICC = 2
	STOP = 3
	

class Logitech_DUEL_ACTION_CONTROLLER():
	MAX_JOY_VAL = 255
	Y_JOY_CENTER = 127
	X_JOY_CENTER = 128


	def __init__(self):
		self.A = 0
		self.B = 0
		self.X = 0
		self.Y = 0
		self.start = 0
		self.back = 0
		self.L_JOY_X = self.X_JOY_CENTER / self.MAX_JOY_VAL
		self.L_JOY_Y = self.Y_JOY_CENTER / self.MAX_JOY_VAL
		self.R_JOY_X = self.X_JOY_CENTER / self.MAX_JOY_VAL
		self.R_JOY_Y = self.Y_JOY_CENTER / self.MAX_JOY_VAL
		self.LR_DPAD = 0
		self.UD_DPAD = 0
		self.L_DPAD = 0
		self.R_DPAD = 0
		self.U_DPAD = 0
		self.D_DPAD = 0
		self.L_BUMP = 0
		self.R_BUMP = 0
		self.L_TRIG = 0
		self.R_TRIG = 0

		self.monitor_thread = threading.Thread(target=self.monitor, args=())
		self.monitor_thread.daemon = True
		self.monitor_thread.start()

	def monitor(self):
		while True:
			events = get_gamepad()
			for event in events:
				if event.code == 'BTN_THUMB':
					self.A = event.state
				elif event.code == 'BTN_THUMB2':
					self.B = event.state
				elif event.code == 'BTN_TRIGGER':
					self.X = event.state
				elif event.code == 'BTN_TOP':
					self.Y = event.state
				elif event.code == 'BTN_BASE4':
					self.start = event.state
				elif event.code == 'BTN_BASE3':
					self.back = event.state
				elif event.code == 'ABS_X':
					self.L_JOY_X = event.state / self.MAX_JOY_VAL
				elif event.code == 'ABS_Y':
					self.L_JOY_Y = event.state / self.MAX_JOY_VAL
				elif event.code == 'ABS_Z':
					self.R_JOY_X = event.state / self.MAX_JOY_VAL
				elif event.code == 'ABS_RZ':
					self.R_JOY_Y = event.state / self.MAX_JOY_VAL
				elif event.code == 'ABS_HAT0X':
					self.LR_DPAD = event.state
					self.L_DPAD = 1 if event.state == -1 else 0
					self.R_DPAD = 1 if event.state == 1 else 0
				elif event.code == 'ABS_HAT0Y':
					self.UD_DPAD = event.state
					self.U_DPAD = 1 if event.state == -1 else 0
					self.D_DPAD = 1 if event.state == 1 else 0
				elif event.code == 'BTN_TOP2':
					self.L_BUMP = event.state
				elif event.code == 'BTN_PINKIE':
					self.R_BUMP = event.state
				elif event.code == 'BTN_BASE':
					self.L_TRIG = event.state
				elif event.code == 'BTN_BASE2':
					self.R_TRIG = event.state
				
	def readAll(self):
		return [self.A, self.B, self.Y, self.X, self.start, self.back, self.UD_DPAD, self.U_DPAD, self.D_DPAD, self.LR_DPAD, self.R_DPAD, self.L_DPAD, self.L_BUMP, self.R_BUMP, self.L_TRIG, self.R_TRIG, self.L_JOY_X, self.L_JOY_Y, self.R_JOY_X, self.R_JOY_Y]
	def readLetters(self):
		return [self.A, self.B, self.Y, self.X]
	def readJoy(self):
		return [self.L_JOY_X, self.L_JOY_Y, self.R_JOY_X, self.R_JOY_Y]
	def readDPAD(self):
		return [self.U_DPAD, self.D_DPAD, self.UD_DPAD, self.L_DPAD, self.R_DPAD, self.LR_DPAD]
	def readSB(self):
		return [self.start, self.back]
	def readBT(self):
		return [self.L_BUMP, self.R_BUMP, self.L_TRIG, self.R_TRIG]
	def readImportantStuff(self):
		return [self.L_JOY_Y, self.R_JOY_X, self.L_TRIG, self.A]
	def ky(self):
		self.monitor_thread._stop()

MAX_MOTOR_CURRENT = 16864

class control_controller_class(Node):
	def __init__(self):
		super().__init__('manual_controller_node')
		self.canPub = self.create_publisher(MotorRequest, '/mooncake/motor_request')
		timer_period = 0.1 #10 times per second
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.controller = Logitech_DUEL_ACTION_CONTROLLER()
		self.currState = drive_mode.STRAIGHT
		self.previousVals = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] #first 4 are joysticks 
	
	def change_mode(self, input : list):
		if input[0] > 0: #A
			self.currState = drive_mode.STRAIGHT
		elif input[1] > 0: #B
			self.currState = drive_mode.POINT_TURN
		elif input[2] > 0: #Y
			self.currState = drive_mode.ICC
		elif input[3] > 0: #X
			self.currState = drive_mode.STOP

	def timer_callback(self):
		#pull controller data
		#create motor Request message
		#make sure to take the multiplier of MAX_MOTOR_CURRENT for the can bus motors
		#send it
		#indexes between 0 and 19 , 20 vals total
		currentVals = self.controller.readAll()
		msgToSend = MotorRequest()
		if currentVals != self.previousVals: #if we have new values
			LeftForwardMovement = currentVals[17]
			LeftTurn = currentVals[16]
			RightForwardMovement = currentVals[19]
			RightTurn = currentVals[18]
			self.change_mode(currentVals) #change the current drive mode
			match self.currState:
				case drive_mode.STRAIGHT:
					#thing
					msgToSend.has_drive = True
					msgToSend.fl_drive = MAX_MOTOR_CURRENT * LeftForwardMovement
				case drive_mode.POINT_TURN:
					msgToSend.has_drive = True
					#thing
					msgToSend.fl_drive = MAX_MOTOR_CURRENT * LeftForwardMovement
					msgToSend.bl_drive = MAX_MOTOR_CURRENT * LeftForwardMovement
					msgToSend.br_drive = MAX_MOTOR_CURRENT * RightForwardMovement
					msgToSend.fr_drive = MAX_MOTOR_CURRENT * RightForwardMovement
					msgToSend.has_turn = True
					msgToSend.left_wheel_pod_turn = 

			
		


#main funciton here
if __name__ == '__main__':
	#stuff thats called once at boot
	previous_Joy_Vals = [0,0,0,0]
	previous_output = []
	controller = Logitech_DUEL_ACTION_CONTROLLER()

	#loop over and over
	while True: #this code is scruffed AF but it will work for now, at least to get a video of the robot moving
		joy =  controller.readImportantStuff()
		#print(joy) #DEBUG
		if(joy != previous_Joy_Vals): #if the current joy values are different from last time
			ForwardMovement =  joy[0] #get the current Y percentage of the left joystick
			SideMovement = joy[1]
			doWeirdTurn = joy[2]
			speed = 0
			if(ForwardMovement > 0.55):
				speed = 0-(ForwardMovement - 0.5)
			elif(ForwardMovement < 0.45):
				speed =(0.5 - ForwardMovement)
			else:
				speed = 0
			turnFactor = SideMovement - 0.5 if(SideMovement > 0.55 and SideMovement < 0.45) else 0
			if(joy[3] == 1): #if A is pressed
				packet = [0, 0, 0, 0, 0, 0] #ALL STOP
			elif(doWeirdTurn == 0): #strafe
				if(SideMovement > 0.55): #If we are going to the right
					packet = [-speed, speed, -speed, speed, SideMovement, -SideMovement]
				elif(SideMovement < 0.45): #If we are going left
					packet = [speed, -speed, speed, -speed, -SideMovement, SideMovement]
				else:
					packet = [speed, speed, speed, speed, 0, 0] #going forward
			else: #If we are doing a point turn
				packet = [-speed, -speed, speed, speed, SideMovement, -SideMovement]
		previous_Joy_Vals = joy #set the previous joy vals to the curr ones
		#sendData(packet)
		print(packet)
		#send the packet (will be the same if the joy values are the same)

	#NEXT LOOP