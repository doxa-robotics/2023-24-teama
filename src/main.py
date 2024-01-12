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

# Pistons (DONE)
wing_piston = Pneumatics(brain.three_wire_port.a)
balance_piston = Pneumatics(brain.three_wire_port.h)

# Motors (Done)
fleft = Motor(Ports.PORT19)
mleft = Motor(Ports.PORT18, True)
bleft = Motor(Ports.PORT20, True)
left = MotorGroup(fleft, mleft, bleft)

fright = Motor(Ports.PORT7)
mright = Motor(Ports.PORT6, True)
bright = Motor(Ports.PORT9)
right = MotorGroup(fright, mright, bright)

gyro = Gyro(brain.three_wire_port.b)

drive_train = SmartDrive(left, right, gyro, 460, 393.7)

# DONE
lever = Motor(Ports.PORT8)

# DONE
flywheel = Motor(Ports.PORT12)


wait(200)
# TODO: we should calibrate the gyro here instead


def driver_control():
    last_r2_pressing = False
    flywheel_spin_forward = False
    last_a_pressing = False
    last_b_pressing = False
    # The heading when we started driving straight.
    initial_heading: vexnumber | None = None
    # The last value of axis1.
    last_controller_turn_pos: vexnumber | None = None
    while True:
        # drivetrain
        axis1 = controller.axis1.position()  # Turning
        axis3 = controller.axis3.position()  # Speed
        if axis1 == 0:
            # Special PID straight driving
            heading = gyro.heading()
            if last_controller_turn_pos != 0 or initial_heading == None:
                # If the last position of the controller wasn't centered:
                # Reset the current heading and try to maintain it.
                initial_heading = heading
            current_heading = heading
            heading_difference = initial_heading - current_heading
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
        if controller.buttonR1.pressing():
            lever.spin(DirectionType.FORWARD, 90, RPM)

        elif controller.buttonL1.pressing():
            lever.spin(DirectionType.REVERSE, 90, RPM)

        else:
            lever.stop(BRAKE)

        # fly wheel (TOGGLE IT. BUTTONS. ADD INTAKE VERSION)

        if controller.buttonR2.pressing() and not last_r2_pressing:
            flywheel_spin_forward = not flywheel_spin_forward
        last_r2_pressing = controller.buttonR2.pressing()

        if flywheel_spin_forward:
            flywheel.spin(DirectionType.FORWARD, 100, PERCENT)
        elif controller.buttonL2.pressing():
            flywheel.spin(DirectionType.REVERSE, 50, PERCENT)
        else:
            flywheel.stop()

        # Pneumatics
        if controller.buttonA.pressing() and not last_a_pressing:
            if wing_piston.value():
                wing_piston.close()
            else:
                wing_piston.open()
        last_a_pressing = controller.buttonA.pressing()
        if controller.buttonB.pressing() and not last_b_pressing:
            if balance_piston.value():
                balance_piston.close()
            else:
                balance_piston.open()
        last_b_pressing = controller.buttonB.pressing()


def move(direction: DirectionType.DirectionType, distance: int):
    drive_train.drive_for(direction, distance, MM, velocity=50)


def autooo_d():
    move(FORWARD, 500)
    drive_train.turn_for(LEFT, 90)
    move(FORWARD, 270)
    drive_train.turn_for(RIGHT, 90)
    move(FORWARD, 300)
    drive_train.turn_for(RIGHT, 90)
    wing_piston.open()
    wait(100)
    move(FORWARD, 450)
    move(REVERSE, 250)
    wing_piston.close()


def autooo_d2():
    move(FORWARD, 360)
    drive_train.turn_for(LEFT, 90)
    move(FORWARD, 300)
    drive_train.turn_for(RIGHT, 90)
    wing_piston.open()
    move(FORWARD, 300)
    drive_train.turn_for(RIGHT, 90, velocity=50)
    wait(100)
    move(FORWARD, 500)
    move(REVERSE, 250)
    wing_piston.close()


def autoo_o():
    move(FORWARD, 1600)
    drive_train.turn_for(LEFT, 90)
    wing_piston.open()
    wait(100)
    move(FORWARD, 630)
    move(REVERSE, 500)


def autoo_o2():
    move(FORWARD, )
    drive_train.turn_for(RIGHT, )
    wing_piston.open()
    wait(100)
    move(FORWARD, )
    move(REVERSE, )


if DEBUG:
    driver_control()
else:
    Competition(driver_control, driver_control)
