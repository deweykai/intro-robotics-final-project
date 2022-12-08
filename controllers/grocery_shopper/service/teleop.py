from robot import keyboard, MAX_SPEED
import bus
import logging

logger = logging.getLogger(__name__)
auto_cooldown = 0
autonomous = False
grip_open = False
precision_mode = False

left_wheel_pub = bus.Publisher('/bot/wheel/cmd_vel/left', float)
right_wheel_pub = bus.Publisher('/bot/wheel/cmd_vel/right', float)
autonomous_pub = bus.Publisher('/bot/cmd_auto', bool)
gripper_pub = bus.Publisher('/bot/cmd_gripper', bool)
mapper_pub = bus.Publisher('/bot/cmd_map', str)
mapper_pub.publish('load')
arm_pub = bus.Publisher('/bot/cmd_arm', str)
autonomous_pub.publish(autonomous)
gripper_pub.publish(True)
arm_pub.publish('standby')


@bus.subscribe('/bot/cmd_auto', bool)
def update_auto_state(new_state):
    global autonomous
    autonomous = new_state
    logger.info(f'auto state set to {autonomous}')


@bus.subscribe('/bot/cmd_gripper', bool)
def update_gripper_state(new_state):
    global grip_open
    grip_open = new_state


def update(delta):
    global auto_cooldown, precision_mode
    if auto_cooldown > 0:
        auto_cooldown = max(0, auto_cooldown - delta)

    key = keyboard.getKey()
    if not autonomous:
        vL, vR = 0, 0
        if key == keyboard.LEFT:
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        else:  # slow down
            vL *= 0.75
            vR *= 0.75

        if precision_mode:
            vL *= 0.1
            vR *= 0.1

        left_wheel_pub.publish(vL)
        right_wheel_pub.publish(vR)

    if key == ord('S'):
        mapper_pub.publish('save')
    elif key == ord('L'):
        mapper_pub.publish('load')
    elif key == ord('A'):
        if auto_cooldown > 0:
            logger.debug('autonomous switching cooldown not finished')
        else:
            auto_cooldown = 100
            autonomous_pub.publish(not autonomous)

    elif key == ord('1'):
        arm_pub.publish('upper')
    elif key == ord('2'):
        arm_pub.publish('lower')
    elif key == ord('3'):
        arm_pub.publish('pre-basket')
    elif key == ord('4'):
        arm_pub.publish('basket')
    elif key == ord('5'):
        arm_pub.publish('post-basket')
    elif key == ord('R'):
        if auto_cooldown > 0:
            logger.debug('autonomous switching cooldown not finished')
        else:
            auto_cooldown = 100
            gripper_pub.publish(not grip_open)

    elif key == ord('P'):
        if auto_cooldown > 0:
            logger.debug('autonomous switching cooldown not finished')
        else:
            auto_cooldown = 100
            precision_mode = not precision_mode
            logger.info(f'Precission mode set to {precision_mode}')


bus.Subscriber('/bot/cmd_tick', int, update)
