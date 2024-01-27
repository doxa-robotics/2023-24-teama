import math

from vex import *

DEBUG = False
#     d1: Defense, push into goal
#     d2: None
#     o1:  match load offense, touch bar
#     o2: don't touch bar
# skills: 60s *auton* skills
#         IMPORTANT NOTE: Driver skills needs AUTON_ROUTINE = "skills" too
#         Both start on the right
#  4253r: completely mess up opponent's auton
# 4253r2: completely mess up opponent's auton by swatting center triball away
#   none: no-op, so do nothing during auton period
AUTON_ROUTINE = "4253r2"

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


def driver_control(flywheel_on=False, flywheel_speed=100):
    last_r2_pressing = False
    flywheel_spin_forward = flywheel_on
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
            flywheel.spin(DirectionType.FORWARD, flywheel_speed, PERCENT)
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


def move(direction: DirectionType.DirectionType, distance: int, velocity=70):
    drive_train.drive_for(direction, distance, MM, velocity, RPM)


def arced_turn(direction: DirectionType.DirectionType, turn_direction: TurnType.TurnType, inner_radius: int, angle: int, velocity=50):
    right_distance = (math.pi * inner_radius * angle) / 180
    left_distance = right_distance + ((math.pi * TRACK_WIDTH * angle) / 180)
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


# start: along the side with triball under lever   Coder:(Rebecca)
def autoo_o1():
    lever.stop()
    move(FORWARD, 680)
    drive_train.turn_for(LEFT, 90)
    move(FORWARD, 410, velocity=50)
    drive_train.turn_for(RIGHT, 90)
    wing_piston.open()
    move(FORWARD, 200, velocity=80)
    move(FORWARD, 450, velocity=30)
    drive_train.turn_for(RIGHT, 45)
    wait(200)
    lever.spin(DirectionType.FORWARD, 10, PERCENT)
    flywheel.spin(DirectionType.REVERSE, 100, PERCENT)
    drive_train.turn_for(RIGHT, 45)
    move(FORWARD, 200)
    flywheel.stop()
    move(FORWARD, 400)
    move(REVERSE, 200)
    wing_piston.close()
    flywheel.stop()
    lever.stop()

# start: along the side with triball under lever  *elevation bar  Coder:(Rebecca)


def autoo_o2():
    lever.stop()
    move(FORWARD, 620, velocity=100)
    drive_train.turn_for(LEFT, 90)
    move(FORWARD, 380, velocity=100)
    drive_train.turn_for(RIGHT, 90)
    wing_piston.open()
    move(FORWARD, 200, velocity=100)
    move(FORWARD, 450, velocity=50)
    drive_train.turn_for(RIGHT, 95)
    lever.spin(DirectionType.FORWARD, 100, PERCENT)
    flywheel.spin(DirectionType.REVERSE, 100, PERCENT)
    move(FORWARD, 200)
    flywheel.stop()
    lever.spin(DirectionType.FORWARD, 5, PERCENT)
    move(FORWARD, 400)
    move(REVERSE, 260)
    wing_piston.close()
    flywheel.stop()
    drive_train.turn_for(LEFT, 118)
    move(REVERSE, 1250, velocity=100)
    drive_train.turn_for(RIGHT, 90)
    lever.stop()
    move(REVERSE, 640, velocity=100)


# start:
def autoo_d1():
    balance_piston.open()
    move(FORWARD, 400)
    right.spin_for(REVERSE, 500/TRACK_DISTANCE, TURNS, 25, PERCENT)
    left.spin_for(FORWARD, 100/TRACK_DISTANCE, TURNS, 12.5, PERCENT)


def autoo_d2():
    balance_piston.open()
    move(FORWARD, 400)
    right.spin_for(REVERSE, 500/TRACK_DISTANCE, TURNS, 25, PERCENT)
    left.spin_for(FORWARD, 100/TRACK_DISTANCE, TURNS, 12.5, PERCENT)


def position_skills():
    flywheel.spin(DirectionType.FORWARD, 70, PERCENT)
    left.spin_for(REVERSE, 140 / TRACK_DISTANCE, TURNS)
    right.spin_for(REVERSE, 10 / TRACK_DISTANCE, TURNS)


def auton_skills():
    """ The auton routine to run during skills.

    Starts on defense: start (e.g., bottom right looking at alliance goal)
    """
    lever.stop(BRAKE)
    initial_heading = drive_train.heading()
    # towards the alliance goal
    position_skills()
    # TODO: change this to 40000 after testing
    # 33 seconds wait for preloading
    wait(33 * 1000)
    flywheel.stop()
    left.spin_for(REVERSE, 140 / TRACK_DISTANCE, TURNS)
    initial_heading = drive_train.heading()
    north = initial_heading + 45
    move(FORWARD, 1000, velocity=90)
    wing_piston.open()
    drive_train.turn_to_heading(north - 90)  # west
    flywheel.spin(DirectionType.REVERSE, 100, PERCENT)
    move(FORWARD, 1000, velocity=90)
    arced_turn(FORWARD, RIGHT, 0, 90)
    # should already be facing north, but to check
    drive_train.turn_to_heading(0)
    lever.spin(REVERSE, 80, RPM)
    move(FORWARD, 400, velocity=20)
    lever.stop()
    move(REVERSE, 700)
    # Crossing the middle
    drive_train.drive_for(FORWARD, 1600, MM, velocity=100, units_v=PERCENT)
    wait(500)
    drive_train.drive_for(FORWARD, 400, MM, velocity=70, units_v=PERCENT)
    flywheel.stop()
    wait(500)
    # To reset angles/pos
    wing_piston.close()
    move(REVERSE, 1200, velocity=80)
    drive_train.turn_to_heading(north+90)  # east
    move(FORWARD, 500, velocity=80)
    # could use arced turn
    drive_train.turn_to_heading(north)
    wing_piston.open()
    wait(500)
    move(FORWARD, 600)
    wing_piston.close()
    move(REVERSE, 800, velocity=100)
    wing_piston.open()
    move(FORWARD, 800)
    wing_piston.close()
    move(REVERSE, 900, velocity=100)


def driver_skills():
    # fix since driver skills runs during auton skills because vex is dumb
    # now need to press [left] at the beginning
    while not controller.buttonLeft.pressing():
        wait(10)
    position_skills()
    driver_control(flywheel_on=True, flywheel_speed=70)


def auton_4253r():
    """ Starting at defense and messing up the triballs for the other team """

    move(FORWARD, 1050, velocity=150)
    drive_train.turn_for(LEFT, 50)
    # careful not to cross the line
    move(FORWARD, 470, velocity=150)
    wing_piston.open()
    wait(1000)
    move(REVERSE, 30, velocity=150)
    wait(500)
    drive_train.turn_for(LEFT, 90)
    wing_piston.close()
    lever.spin(DirectionType.FORWARD, 10, PERCENT)
    flywheel.spin(DirectionType.REVERSE, 100, PERCENT)
    move(FORWARD, 600, velocity=110)
    flywheel.stop()
    move(REVERSE, 100, velocity=110)
    drive_train.turn_for(RIGHT, 90)
    lever.spin(DirectionType.REVERSE, 90, PERCENT)
    lever.stop()


def auton_4253r_2_towards_barrier():
    """ Starting at defense and messing up the center triball for the other team """

    move(FORWARD, 1050, velocity=150)
    drive_train.turn_for(LEFT, 50)
    # careful not to cross the line
    move(FORWARD, 450, velocity=150)
    wing_piston.open()
    wait(500)
    drive_train.turn_for(RIGHT, 90, velocity=130)
    wing_piston.close()
    wait(500)
    drive_train.turn_for(LEFT, 180)
    lever.spin(DirectionType.FORWARD, 10, PERCENT)
    flywheel.spin(DirectionType.REVERSE, 100, PERCENT)
    move(FORWARD, 600, velocity=110)
    flywheel.stop()
    move(REVERSE, 100, velocity=110)
    drive_train.turn_for(RIGHT, 90)
    lever.spin(DirectionType.REVERSE, 90, PERCENT)
    lever.stop()


def auton():
    """ Main auton code. Put calls to functions here. """
    if AUTON_ROUTINE == "o1":
        autoo_o1()
    elif AUTON_ROUTINE == "o2":
        autoo_o2()
    elif AUTON_ROUTINE == "d1":
        autoo_d1()
    elif AUTON_ROUTINE == "d2":
        autoo_d2()
    elif AUTON_ROUTINE == "skills":
        auton_skills()
    elif AUTON_ROUTINE == "4253r":
        auton_4253r()
    elif AUTON_ROUTINE == "4253r2":
        auton_4253r_2_towards_barrier()
    elif AUTON_ROUTINE == "test":
        arced_turn(FORWARD, RIGHT, 10, 45)


if DEBUG:
    driver_control()
else:
    Competition(driver_skills if AUTON_ROUTINE ==
                "skills" else driver_control, auton)
9
