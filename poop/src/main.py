    # awesome program / not so awsome program -asiah
    #allah

import json
import time
import math
# imports all yo stuffz
from vex import *
from vex import Motor 
#sys.run_in_thread(lambda: print("test")) # idk WHY THE FREAK this not working
 
    # import threading

def clamp(n, min, max):
    if n < min:
       return min
    elif n > max:
        return max
    else:
        return n

    # quick fix to define controller first cause its actually defined after th drive train class
controller:Controller = None

    # im creating the drive train class here because vex is horrible and cant load modules

r = 2
def AngleToDistance(angle):
    return r*math.pi*2 * (angle/360)

def DistanceToAngle(dist):
    return (360*dist)/(2*math.pi*r)



    #basiclaly trying to replicat the default DRive train class but worse and can use more than 2 wheels
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
        self.playback_inputs = []

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
        motor: Motor #define empty for type annotations

        for motorIndex in range(len(self.get_motors())):
            motor = self.get_motors()[motorIndex]

            #print(motor.position())
                
            self.get_motor_velocities()[motorIndex] = clamp(self.get_motor_velocity(motorIndex),-100,100)
            
            
            vel = self.get_motor_velocity(motorIndex)
            motor.set_velocity(vel,VelocityUnits.PERCENT)
            
            if abs(vel) > 0:
                
                motor.spin(DirectionType.FORWARD)
            else:
                motor.set_stopping(BrakeType.HOLD)
                #self.stop()
            

            

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
                #self.instruction_cache.remove(instruction_info)
                #del self.instruction_cache[instruction_index]
                #continue
            
            if instruction_info["type"] == "drive":
                self.set_drive_velocity(instruction_info["vel"])

            if instruction_info["type"] == "turn":
                self.set_turn_velocity(instruction_info["vel"])

    def stop(self):
        motor: Motor #define empty for type annotations

        for motorIndex in range(len(self.get_motors())):
            motor = self.get_motors()[motorIndex]

            motor.set_stopping(HOLD)

    
    def process_controller_inputs(self):

        forward_input = controller.axis3.position()
        turn_input = controller.axis1.position()

        self.set_drive_velocity(forward_input)
        self.set_turn_velocity(turn_input)

        if abs(forward_input) <= 0 and abs(turn_input) <= 0:
            pass
        else:
            pass
        
        input = [forward_input,turn_input]
        
        if self.recording_inputs:
            if input != self.last_input:
                #print(self.record_start)
                self.recorded_inputs.append([forward_input,turn_input,self.record_timer.time()])
        
        self.last_input = input

        
    def toggle_recording_inputs(self, toggle:bool):
        self.recording_inputs = toggle

        if toggle:
            self.record_timer.reset()
        else:
            pass

    def get_recorded_inputs(self):
        return self.recorded_inputs
    
    def get_recorded_inputs_json(self):
        return json.dumps(self.get_recorded_inputs())

    def playback_json_recording(self,json_str:str):
        inputs = json.loads(json_str)
        self.playback_recording(inputs)

    def playback_recording(self,inputs:list):
        self.playback_timer.reset()
        self.playback_inputs = inputs
        self.playback_enabled = True

        self.curr_playback_index = 0

        last_input = inputs[len(inputs)-1]
        while self.playback_timer.time() < last_input[2]:
            pass
            
            
        self.playback_enabled = False

        pass

    def Update(self):
        for i in range(len(self.motor_velocities)):
            self.motor_velocities[i] = 0
            
        self.process_controller_inputs()
        self.process_instructions()

        if self.playback_enabled:
            if self.curr_playback_index <= len(self.playback_inputs)-1:
                curr_input = self.playback_inputs[self.curr_playback_index]
                
                if self.playback_timer.time() > curr_input[2]:
                    self.curr_playback_index += 1

                self.set_drive_velocity(curr_input[0])
                self.set_turn_velocity(curr_input[1])
            

            

        self.update_velocities()



brain=Brain()


# left motors first then right
motorPorts = [20,15,10,5]
motors = []

for i in motorPorts:
    motors.append(Motor(i-1))


controller = Controller(ControllerType.PRIMARY)
drivetrain = DriveTrain(motors[0],motors[1])
drivetrainCool = DriveTrainCool(motors)


    # maybe create a custom drivetrain class actually because we can calculate all the stuff with math
    # wheels are 4 inches in diameter, 2 inches radius


    # were not going to be using a drive train for the driver control because it is limited in its control 
    # of each motor so it cant like turn while moving forward.
    # so were gonna have a list of motors to update their velocities in a loop so we can individually control 
    # each motor

    # were gonna drive train for auton though because of its precise measurement thingies

    # autonomous stuff
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




def Update():

    #drivetrainCool.process_controller_inputs()
    #drivetrainCool.process_instructions()
    #drivetrainCool.update_velocities()
    drivetrainCool.Update()
    pass


# stuff for driver control
def DriverControl():
    global comp
    

    while comp.is_driver_control():
        drivetrainCool.Update()
        pass




comp = Competition(DriverControl, Autonomous)

def Test():
    #drivetrainCool.turn_for(time_length=1,velocity=5)
    #drivetrainCool.drive_for(time_length=1,velocity=20)
    drivetrainCool.toggle_recording_inputs(True)
    time.sleep(2)
    drivetrainCool.toggle_recording_inputs(False)

    recording_json = drivetrainCool.get_recorded_inputs_json()
    brain.screen.print(drivetrainCool.get_recorded_inputs())
    print("test")
    print(drivetrainCool.get_recorded_inputs())
     
     
    
    #drivetrainCool.drive_for(time_length=1.0,velocity=40)
    time.sleep(2)
    drivetrainCool.playback_json_recording(recording_json)
    #drivetrainCool.turn_for(time_length=0.7,velocity=-20)


Test()
if comp.is_enabled():
    pass
else:
    #Test()
    #Thread(Test)
    while True:
        #Update()
        pass
