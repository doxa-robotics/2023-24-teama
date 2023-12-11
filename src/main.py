from vex import *
import math


def convert_damped_controller(val):
    value = math.pow(0.1*val, 2)
    if val < 0:
        return -value
    else:
        return value


class PneumaticsGroup:
    members: list[Pneumatics]
    value: bool

    def __init__(self, *args: Pneumatics):
        self.members = list(args)
        self.value = False

    def open(self):
        self.value = True
        for member in self.members:
            member.open()

    def close(self):
        self.value = False
        for member in self.members:
            member.close()


brain = Brain()

controller = Controller()

piston = Pneumatics(brain.three_wire_port.a)


fleft = Motor(Ports.PORT6)
mleft = Motor(Ports.PORT9, True)
bleft = Motor(Ports.PORT8, True)

fright = Motor(Ports.PORT1, True)
mright = Motor(Ports.PORT2)
bright = Motor(Ports.PORT5)

lever = Motor(Ports.PORT4)

flywheel = Motor(Ports.PORT20)

left = MotorGroup(fleft, mleft, bleft)
right = MotorGroup(fright, mright, bright)

gyro = Gyro(brain.three_wire_port.a)

drive_train = SmartDrive(left, right, gyro, 255, 393.7)


def driver_control():
    while True:
        # drivetrain
        left.spin(
            DirectionType.FORWARD,
            convert_damped_controller(controller.axis3.position(
            )) + convert_damped_controller(controller.axis1.position()),
            VelocityUnits.PERCENT)

        right.spin(
            DirectionType.FORWARD,
            convert_damped_controller(controller.axis3.position(
            )) - convert_damped_controller(controller.axis1.position()),
            VelocityUnits.PERCENT)

        wait(20)

        # lever
        if controller.buttonUp.pressing():
            lever.spin(DirectionType.FORWARD, 90, RPM)

        elif controller.buttonDown.pressing():
            lever.spin(DirectionType.REVERSE, 90, RPM)

        else:
            lever.stop(BRAKE)

        # fly wheel (TOGGLE IT. BUTTONS. ADD INTAKE VERSION)
        if controller.buttonA.pressing():
            flywheel.spin(DirectionType.FORWARD, 100, PERCENT)

        elif controller.buttonX.pressing():
            flywheel.spin(DirectionType.REVERSE, 50, PERCENT)
        elif controller.buttonB.pressing():
            flywheel.spin(DirectionType.FORWARD, 20, PERCENT)
        else:
            flywheel.stop()


all = DriveTrain(left, right, 259)
distance = 1000


def move(direction: DirectionType.DirectionType, distance: int):
    all.drive_for(direction, distance, MM, velocity=50)


def autooo_defense():
    move(FORWARD, 1000)
    all.turn(LEFT, 90)
    move(FORWARD, 500)
    all.turn(RIGHT, 90)
    move(FORWARD, 400)
    all.turn(RIGHT, 90)
    piston.open()
    wait(100)
    move(FORWARD, 900)
    move(REVERSE, 500)
    piston.close()


def autoo_offense():
    move(FORWARD, 1600)
    all.turn(LEFT, 90)
    piston.open()
    wait(100)
    move(FORWARD, 630)
    move(REVERSE, 500)


autooo_defense()
driver_control()
