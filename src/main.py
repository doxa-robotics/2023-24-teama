import math

from vex import *

DEBUG = False
#     d1: Defense, push into goal
#     d2: None
#     o1:  match load offense, touch bar
#     o2: don't touch bar
# skills: 60s *auton* skills
#   none: no-op, so do nothing during auton period
AUTON_ROUTINE = "skills"

# Distance between wheel centers, mm
TRACK_WIDTH = 305
# Distance robot moves in one motor turn, mm
TRACK_DISTANCE = 460


def convert_damped_controller(val):
    value = math.pow(0.1*val, 2)
    if val < 0:
        return -value
    else:
        return value


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

drive_train = SmartDrive(left, right, gyro, TRACK_DISTANCE)
drive_train.set_timeout(4000)
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
    while True:
        # drivetrain
        axis1 = controller.axis1.position()  # Turning
        axis3 = controller.axis3.position()  # Speed
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
    drive_train.drive_for(direction, distance, MM, velocity=80)


def arced_turn(direction: DirectionType.DirectionType, turn_direction: TurnType.TurnType, inner_radius: int, angle: int):
    right_distance = (math.pi * inner_radius * angle) / 180
    left_distance = right_distance + ((math.pi * TRACK_WIDTH * angle) / 180)
    velocity = 50
    ratio = (left_distance / right_distance) if right_distance > 0 else 0
    if turn_direction == RIGHT:
        right.spin_for(direction, right_distance / TRACK_DISTANCE,
                       TURNS, velocity * ratio, PERCENT, wait=False)
        left.spin_for(direction, left_distance / TRACK_DISTANCE,
                      TURNS, velocity, PERCENT)
    else:
        left.spin_for(direction, right_distance / TRACK_DISTANCE, TURNS,
                      velocity * ratio, PERCENT, wait=False)
        right.spin_for(direction, left_distance /
                       TRACK_DISTANCE, TURNS, velocity, PERCENT)


# start: along the side      (rebecca)
def autoo_o1():
    move(FORWARD, 680)
    drive_train.turn_for(LEFT, 90)
    wing_piston.open()
    move(FORWARD, 340)
    drive_train.turn_for(RIGHT, 90)
    move(FORWARD, 880)
    drive_train.turn_for(RIGHT, 90)
    move(FORWARD, 400)
    lever.spin(DirectionType.REVERSE, 90, RPM)
    move(FORWARD, 200)
    move(REVERSE, 200)
    wing_piston.close()


def autoo_o2():
    move(FORWARD, 680)
    drive_train.turn_for(LEFT, 90)
    move(FORWARD, 340)
    drive_train.turn_for(RIGHT, 90)
    move(FORWARD, 880)
    drive_train.turn_for(RIGHT, 90)
    move(FORWARD, 800)
    drive_train.turn_for(RIGHT, 90)
    wing_piston.open()
    move(FORWARD, 550)
    lever.spin(DirectionType.REVERSE, 90, RPM)
    move(FORWARD, 100)
    move(REVERSE, 200)
    drive_train.turn_for(RIGHT, 90)
    lever.spin(DirectionType.FORWARD, 20, RPM)
    move(FORWARD, 1700)
    drive_train.turn_for(RIGHT, 90)
    move(FORWARD, 720)


# start:
def autoo_d():
    move(FORWARD, 300)
    balance_piston.open()
    drive_train.turn_for(RIGHT, 45)
    move(FORWARD, 200)
    drive_train.turn_for(RIGHT, 45)
    move(FORWARD, 100)
    drive_train.turn_for(RIGHT, 90)
    move(FORWARD, 600)
    move(REVERSE, 150)
    balance_piston.close()
    drive_train.turn_for(LEFT, 20)
    move(FORWARD, 200)
    drive_train.turn_for(RIGHT, 20)
    move(FORWARD, 100)
    move(REVERSE, 350)
    drive_train.turn_for(RIGHT, 90)
    move(FORWARD, 250)
    drive_train.turn_for(LEFT, 90)
    move(FORWARD, 250)
    # lever.spin_to_position(DirectionType.REVERSE, 90, RPM)


def auton_skills():
    """ The auton routine to run during skills.

    Starts on defense: start (e.g., bottom right looking at alliance goal)
    """
    initial_heading = drive_train.heading()
    # towards the alliance goal
    north = initial_heading + 45
    flywheel.spin(DirectionType.FORWARD, 100, PERCENT)
    # TODO: change this to 40000 after testing
    # 40 seconds wait for preloading
    wait(0)
    flywheel.stop()
    move(FORWARD, 1000)
    wing_piston.open()
    drive_train.turn_to_heading(north - 90)  # west
    move(FORWARD, 1000)
    arced_turn(FORWARD, RIGHT, 0, 90)
    # should already be facing north, but to check
    drive_train.turn_to_heading(45)
    move(REVERSE, 400)
    # Crossing the middle
    drive_train.drive_for(FORWARD, 1400, MM, velocity=100, units_v=PERCENT)
    wait(2000)
    drive_train.drive_for(FORWARD, 600, MM, velocity=100, units_v=PERCENT)
    arced_turn(FORWARD, RIGHT, 400, 45)
    move(FORWARD, 1000)
    drive_train.turn_to_heading(north)
    wing_piston.close()
    # moving out of the goal
    move(REVERSE, 1100)
    drive_train.turn_to_heading(north+70)  # north-east
    wing_piston.open()
    move(FORWARD, 400)
    # could use arced turn
    drive_train.turn_to_heading(north)
    move(FORWARD, 1200)
    wing_piston.close()
    move(REVERSE, 600)


def auton():
    """ Main auton code. Put calls to functions here. """
    if AUTON_ROUTINE == "o1":
        autoo_o1()
    elif AUTON_ROUTINE == "2":
        autoo_o2()
    elif AUTON_ROUTINE == "d2":
        autoo_d()
    elif AUTON_ROUTINE == "skills":
        auton_skills()
    elif AUTON_ROUTINE == "test":
        arced_turn(FORWARD, RIGHT, 10, 45)


if DEBUG:
    driver_control()
else:
    Competition(driver_control, auton)
