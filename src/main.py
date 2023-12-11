from vex import *
import math


def convert_damped_controller(val):
    value = math.pow(0.1*val, 2)
    if val < 0:
        return -value
    else:
        return value


brain = Brain()

controller = Controller()

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
        if controller.buttonR1.pressing():
            lever.spin(DirectionType.FORWARD, 90, RPM)

        elif controller.buttonL1.pressing():
            lever.spin(DirectionType.REVERSE, 90, RPM)

        else:
            lever.stop(BRAKE)

        # fly wheel (TOGGLE IT. BUTTONS. ADD INTAKE VERSION)
        if controller.buttonR2.pressing():
            flywheel.spin(DirectionType.FORWARD, 100, PERCENT)

        elif controller.buttonL2.pressing():
            flywheel.spin(DirectionType.REVERSE, 50, PERCENT)
        else:
            flywheel.stop()


def auton_defence():
    pass


driver_control()
