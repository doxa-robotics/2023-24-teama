from vex import *
import math

DEBUG = True
# How aggressive the PID should be when adjusting the driving.
# This is multiplied by how far off the gyro is to get the speed adjustment.
# TODO: Fine tweak this until it works well.
PID_AGGRESSION_MODIFIER = 5


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

gyro = Gyro(brain.three_wire_port.b)

drive_train = SmartDrive(left, right, gyro, 255, 393.7)

wait(200)
# TODO: we should calibrate the gyro here instead


def driver_control():
    # The heading when we started driving straight.
    initial_heading: vexnumber | None = None
    # The last value of axis1.
    last_controller_turn_pos: vexnumber | None = None
    while True:
        # drivetrain
        axis1 = controller.axis1.position()  # Turning modifier
        axis3 = controller.axis3.position()  # Speed
        if axis3 == 0:
            # Special PID straight driving
            if last_controller_turn_pos != 0 or initial_heading == None:
                # If the last position of the controller wasn't centered:
                # Reset the current heading and try to maintain it.
                initial_heading = gyro.heading()
            current_heading = gyro.heading()
            heading_difference = current_heading - initial_heading
            desired_speed = convert_damped_controller(axis3)
            # If heading_difference is positive, we're drifting right.
            # If heading_difference is negative, we're drifitng left.
            # If we're going right, we need to slow down the left motors, and
            # vice versa.
            left.spin(
                DirectionType.FORWARD,
                desired_speed - heading_difference*PID_AGGRESSION_MODIFIER,
                VelocityUnits.PERCENT
            )
            right.spin(
                DirectionType.FORWARD,
                desired_speed + heading_difference*PID_AGGRESSION_MODIFIER,
                VelocityUnits.PERCENT
            )
        else:
            # Normal driving
            left.spin(
                DirectionType.FORWARD,
                convert_damped_controller(
                    axis3) + convert_damped_controller(axis1),
                VelocityUnits.PERCENT)

            right.spin(
                DirectionType.FORWARD,
                convert_damped_controller(
                    axis3) - convert_damped_controller(axis1),
                VelocityUnits.PERCENT)
        last_controller_turn_pos = axis1

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

        # Pneumatics
        if controller.buttonR1.pressing():
            piston.open()
        elif controller.buttonR2.pressing():
            piston.close()


def move(direction: DirectionType.DirectionType, distance: int):
    drive_train.drive_for(direction, distance, MM, velocity=50)


def autooo_d():
    move(FORWARD, 500)
    drive_train.turn_for(LEFT, 90)
    move(FORWARD, 270)
    drive_train.turn_for(RIGHT, 90)
    move(FORWARD, 300)
    drive_train.turn_for(RIGHT, 90)
    piston.open()
    wait(100)
    move(FORWARD, 450)
    move(REVERSE, 250)
    piston.close()


def autoo_o():
    move(FORWARD, 1600)
    drive_train.turn_for(LEFT, 90)
    piston.open()
    wait(100)
    move(FORWARD, 630)
    move(REVERSE, 500)


if DEBUG:
    autooo_d()
    driver_control()
else:
    Competition(driver_control, driver_control)
