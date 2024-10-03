from inputs import devices
from inputs import get_gamepad
import math
import threading
import serial



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

#main funciton here
if __name__ == '__main__':
	#stuff thats called once at boot
	previous_Joy_Vals = []
	previous_output = []
	controller = Logitech_DUEL_ACTION_CONTROLLER()
	ser = serial.Serial() #open serial port given with a baud rate the same as the arduinos and give 5 seconds to connect.
	port = input("Give COM port as 'COMX' where X is the COM port number: ")
	ser.port(port)
	ser.baudrate(115200)
	ser.timeout(5) #5 second time out
	ser.open()
	if(not ser.is_open()): #if the serial did not open
		exit #exit
	packet = [] #packet to send to serial, formatted like LF LB RB RF LT RT, will be the same as it was previously each loop
	#loop over and over
	while True: #this code is scruffed AF but it will work for now, at least to get a video of the robot moving
		joy =  controller.readImportantStuff()
		if(joy != previous_Joy_Vals): #if the current joy values are different from last time
			ForwardMovement =  joy.atIndex(0) #get the current Y percentage of the left joystick
			SideMovement = joy.atIndex(1)
			doWeirdTurn = joy.atIndex(2)
			speed = ForwardMovement - 0.5 if(ForwardMovement > 0.55 and ForwardMovement < 0.45) else 0 #motor speed is this as if we are in the middle, +- 0.05 on both sides for a dead zone
			turnFactor = SideMovement - 0.5 if(SideMovement > 0.55 and SideMovement < 0.45) else 0
			if(joy.atIndex(3) == 1): #if A is pressed
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
			serial.write(packet) #write the packet to serial
			serial.flush() #wait until packet is sent
			continue #next loop
		else: #the values are the same
			serial.write(packet) #write the same packet
			serial.flush() #wait for packet to be written