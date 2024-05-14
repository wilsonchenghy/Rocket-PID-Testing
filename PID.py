import turtle
import time



### Initialise Global Parameters
SIM_DURATION = 100
TIME_STEP = 0.1

SETPOINT = 0

INITIAL_HEIGHT = -50

# PID Values that can be tuned (can be tuned by using the stabilization time printed in the terminal)
KP = 0.88
KI = 0.21
KD = 0.62

# Properties of the rocket
MASS = 1
MAX_THRUST = 15

# Properties of environment
GRAVITY = -9.81



### Simulation Class
class Simulation(object):
    def __init__(self):
        self.simDuration = SIM_DURATION
        self.count = 1

        self.screen = turtle.Screen()
        self.screen.setup(600,700)
        self.screen.title("PID Control")
        self.marker = turtle.Turtle()
        self.marker.color('red')
        self.marker.penup()
        self.marker.left(180)
        self.marker.goto(20,SETPOINT)

        self.rocket = Rocket()

        self.pid_controller = PID(KP, KI, KD, SETPOINT)

    def run(self):
        presentTime = time.time()
        prevTime = presentTime
        currentTime = 0
        counterForStabilization = 0
        isSteady = False

        # Simulation run for simDuration second
        while ((time.time() - presentTime) < self.simDuration):
            
            while ((time.time() - prevTime) >= TIME_STEP):

                # CurrentTime depends on TIME_STEP
                currentTime += 1
                # print(f'Current Time: {currentTime}')
                # print(" ")


                rocketHeight = self.rocket.get_Y()

                thrust = self.pid_controller.computeThrust(rocketHeight)
                self.rocket.enablePhysics(thrust)

                # Print important data (print every 1 TIME_STEP)
                if (currentTime % 1 == 0): # can change the numbers to print out data slower
                    # print(f'Thrust: {thrust}')
                    # self.rocket.printData()
                    # print(" ")

                    # Print stabilization time
                    if abs(rocketHeight - SETPOINT) < 2:
                        counterForStabilization += 1
                        if (counterForStabilization > 20 and (not isSteady)):
                            print('Reach stable state')
                            print(f'Time Taken: {currentTime}')
                            isSteady = True
                    else:
                        counterForStabilization = 0


                prevTime = time.time()

        print("Simulation Complete")
        turtle.done()



### Rocket Class
class Rocket(object):
    def __init__(self):
        self.rocket = turtle.Turtle()
        self.rocket.shape('square')
        self.rocket.color('black')
        self.rocket.pendown() # pendown to track the overshoot of the pid contoller

        self.y = INITIAL_HEIGHT
        self.dy = 0
        self.ddy = 0

        self.rocket.goto(0, self.y)

    def get_Y(self):
        y_coordinate = self.y
        return y_coordinate

    def set_Y(self, height):
        self.y = height
        self.rocket.goto(0, height)

    def enablePhysics(self, thrust):
        self.ddy = GRAVITY + thrust / MASS

        self.dy += self.ddy * TIME_STEP

        self.y += self.dy * TIME_STEP

        self.set_Y(self.y)

    def printData(self):
        print(f'Y_Coord: {self.y}')
        print(f'Velocity: {self.dy}')
        print(f'Acceleration: {self.ddy}')



### PID Control
class PID(object):
    def __init__(self, Kp, Ki, Kd, setPoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setPoint = setPoint
        self.error = 0
        self.prevError = 0
        self.derivativeError = 0
        self.integralError = 0
        self.output = 0
    
    def computeThrust(self, currentPoint):
        self.error = self.setPoint - currentPoint

        self.derivativeError = (self.error - self.prevError) / TIME_STEP
        # print(f'error: {self.error}')
        # print(f'prevError: {self.prevError}')
        # print(f'derivativeError: {self.derivativeError}')
        # print(" ")
        self.prevError = self.error

        # print(f'1: {self.error * self.Kp}')
        # print(f'2: {self.derivativeError * self.Kd}')
        # print(f'3: {self.error * self.Ki}')
        self.output = self.error * self.Kp + self.derivativeError * self.Kd + self.integralError * self.Ki

        # Prevent integral windup
        if (abs(self.output) >= MAX_THRUST) and ((self.error >= 0 and self.integralError >= 0) or (self.error < 0 and self.integralError < 0)):
            self.integralError = self.integralError
        else:
            self.integralError += self.error * TIME_STEP
            # print(f'integralError: {self.integralError}')


        if (self.output > MAX_THRUST):
            self.output = MAX_THRUST
        if (self.output < 0):
            self.output = 0

        return self.output



### main() function
def main():
    sim = Simulation()
    sim.run()



### Run main()
main()