# main program


import json
import time
import math
from vex import *
from vex import Motor 


def clamp(n, min, max):
    if n < min:
       return min
    elif n > max:
        return max
    else:
        return n

    # quick fix to declare controller first cause its actually defined after th drive train class
controller:Controller = None


r = 2
def AngleToDistance(angle):
    return r*math.pi*2 * (angle/360)

def DistanceToAngle(dist):
    return (360*dist)/(2*math.pi*r)



    #basiclaly trying to replicat the default DRive train class can use more than 2/4 wheels
    # and other features
class DriveTrainCool():

    #constructor function
    def __init__(self, motors:list):
        self.motors = motors    
        self.motor_velocities = []
        self.motor_count = len(self.motors)

        self.input_cache = []
        self.instruction_cache = []

        self.recorded_inputs = []
        self.last_input = None
        self.recording_inputs = False
        self.record_timer = Timer()

        self.playback_timer = Timer()
        self.playback_enabled = False

        for motorIndex in range(len(self.motors)):
            self.motor_velocities.append(0)
            #self.motor_velocities[i] = 0


    def get_motor_count(self):
        return self.motor_count
        
    def get_motors(self):
        return self.motors
        
    def get_motor_velocity(self,index):
        return self.motor_velocities[index]
        
    def get_motor_velocities(self):
        return self.motor_velocities

    def set_drive_velocity(self,vel):
            # ok so basically for turning were gonna loop through the motors and set half of them to plus vel
            # and half to minus vel.
            # this is assuming that the motors were configured in the order of all left motors first
            # then right motors.

        half = int(self.motor_count/2)
        
        for motor_velocity_index in range(len(self.motor_velocities)):
            motor_velocity = self.motor_velocities[motor_velocity_index]

            if motor_velocity_index > half-1:
                self.motor_velocities[motor_velocity_index] -= vel
            else:
                self.motor_velocities[motor_velocity_index] += vel


    def set_turn_velocity(self,vel):
        for i in range(len(self.motor_velocities)):
           self.motor_velocities[i] += vel

        #print(self.motor_velocities)
            
        
    def update_velocities(self):
        motor: Motor #declare empty variable for type annotations

        for motorIndex in range(len(self.get_motors())):
            motor = self.get_motors()[motorIndex]      

            # clamp the velocity to a range of -100 to 100 to make sure the motor doesnt overheat
            self.get_motor_velocities()[motorIndex] = clamp(self.get_motor_velocity(motorIndex),-100,100)
            
            
            vel = self.get_motor_velocity(motorIndex)
            motor.set_velocity(vel,VelocityUnits.PERCENT)
            
            
            if abs(vel) > 0:
                motor.spin(DirectionType.FORWARD)
            else:
                # if the velocity is 0, then stop the motor to make movement snappy and responsive
                motor.set_stopping(BrakeType.HOLD)
            

            

    # this is probably an extremely overcomplicated way to do this but basically 
    # it inserts a dictionary into the list of instructions to process containing info such as 
    # velocity, start time, etc
    # the process instructions method then loops through the instructions and does stuff 
    # removing expired instructions as needed
    def drive_for(self,time_length,velocity):
        time_length = float(time_length)
        self.instruction_cache.append({"end_time":time.time() + time_length,"type":"drive","vel":velocity})

    def turn_for(self,time_length,velocity):
        time_length = float(time_length)
        self.instruction_cache.append({"end_time":time.time() + time_length,"type":"turn","vel":velocity})


    def process_instructions(self):
        for instruction_index in range(len(self.instruction_cache)):
            if instruction_index > len(self.instruction_cache)-1:
                continue 
            
            instruction_info = self.instruction_cache[instruction_index]
            # if the current time is greater than the end time of the instruction, continue to the next instruction
            # and remove the current from the list
            if time.time() >= instruction_info["end_time"]:
                self.instruction_cache.pop(instruction_index)
            
            #carry out the action associated with the instruction packet
            if instruction_info["type"] == "drive":
                self.set_drive_velocity(instruction_info["vel"])

            if instruction_info["type"] == "turn":
                self.set_turn_velocity(instruction_info["vel"])

    def stop(self):
        motor: Motor #define empty for type annotations

        for motorIndex in range(len(self.get_motors())):
            motor = self.get_motors()[motorIndex]

            motor.set_stopping(HOLD)


    def record_input_update(self,input):
        # if recording and the current input is different than the previous recorded input, add the new input into the list
            

            print(input,self.record_timer.system_high_res())

            if input != self.last_input:
                # define a variable called "instruction_packet" that contains the input and time since recording start
                instruction_packet = [input[0],input[1],self.record_timer.system_high_res()]
                self.recorded_inputs.append(instruction_packet)
    

    def process_controller_inputs(self):

        forward_input = controller.axis3.position()
        turn_input = controller.axis1.position()        

        input = [forward_input,turn_input]

        if self.recording_inputs:
            self.record_input_update(input)
        
        self.last_input = input

        self.set_drive_velocity(forward_input)
        self.set_turn_velocity(turn_input)


        
    def toggle_recording_inputs(self, toggle:bool):
        self.recording_inputs = toggle

        if toggle:
            self.recorded_inputs = []
            self.record_timer.reset()
        else:
            pass
    
    def get_recorded_inputs(self):
        return self.recorded_inputs
    
    def get_recorded_inputs_json(self):
        # takes a python object and returns a json encoded string
        return json.dumps(self.get_recorded_inputs())

    def playback_json_recording(self,json_str:str):
        # takes a json encoded string and returns a python object
        inputs = json.loads(json_str)
        self.playback_recording(inputs)


    
    def playback_recording(self,inputs:list):

        self.curr_playback_index = 0
        self.playback_timer.reset()

        self.playback_enabled = True


        self.input_packet = inputs[self.curr_playback_index]
        while self.curr_playback_index <= len(inputs)-1:
            self.input_packet = inputs[self.curr_playback_index]

            print(self.input_packet, self.playback_timer.system_high_res())

            if self.playback_timer.system_high_res() > self.input_packet[2]:
                self.curr_playback_index += 1

        self.playback_enabled = False

    
    def playback_update(self):
        if self.playback_enabled:
            self.set_drive_velocity(self.input_packet[0])
            self.set_turn_velocity(self.input_packet[1])
                

    
    def Update(self):
        for i in range(len(self.motor_velocities)):
            self.motor_velocities[i] = 0
            
        self.process_controller_inputs()
        self.process_instructions()
        
        if self.playback_enabled:
            self.playback_update()

        if controller.buttonR1.pressing():
            SpinIntake(True,100)
        if controller.buttonR2.pressing():
            SpinIntake(False,100)

        #print(self.get_motor_velocities())

        #self.update_velocities()

    def bind_to_update(self,func:function):

        pass

brain=Brain()


# left motors first then right
# back to front
motorPorts = [10,20,1,11]
motors = []

intake_motor_ports = [18,19]
intake_motors = []
for motor_index in range(len(intake_motor_ports)):
    motor_port = intake_motor_ports[motor_index]
    intake_motor = Motor(motor_port,motor_index%2!=0)
    intake_motors.append(intake_motor)

# creating a list of motor objects to pass into our custom drive train class
for i in motorPorts:
    motors.append(Motor(i-1))

controller = Controller(ControllerType.PRIMARY)
drivetrainCool = DriveTrainCool(motors)


def SpinIntake(inward,velocity):
    motor:Motor = None
    for motor in intake_motors:
        if inward:
            motor.spin(DirectionType.FORWARD,velocity,PERCENT)
        else:
            motor.spin(DirectionType.REVERSE,velocity,PERCENT)


def Autonomous():
    global drivetrainCool


    def AutonUpdateLoop():
        while comp.is_autonomous():
            #drivetrainCool.process_instructions()
            drivetrainCool.Update()
        
    #auton_thread = threading.Thread(target=AutonUpdateLoop,args=())
    #auton_thread.start()

    Thread(AutonUpdateLoop)
    #sys.run_in_thread(AutonUpdateLoop)
    drivetrainCool.drive_for(time_length=1,velocity=100)
    
    #   drivetrain.drive_for(DirectionType.FORWARD,10,DistanceUnits.IN)


# stuff for driver control
def DriverControl():
    global comp
    

    while comp.is_driver_control():
        drivetrainCool.Update()
        pass



def BPressed():
    if drivetrainCool.recording_inputs:
        drivetrainCool.toggle_recording_inputs(False)
        controller.rumble("..")
        
    else:
        controller.rumble(".")
        time.sleep(0.1)
        drivetrainCool.toggle_recording_inputs(True)

def APressed():
    if drivetrainCool.recording_inputs:
        return
    
    controller.rumble("-")
    recording = drivetrainCool.get_recorded_inputs()
    print(recording)
    # recording_json = drivetrainCool.get_recorded_inputs_json()
    # print(recording_json)
    # drivetrainCool.playback_json_recording(recording_json)

    drivetrainCool.playback_recording(recording)

controller.buttonB.pressed(BPressed)
controller.buttonA.pressed(APressed)


def Test():
    
    pass


comp = Competition(DriverControl, Autonomous)
if comp.is_enabled():
    pass
else:
    Test()
    pass
