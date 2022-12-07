from robot import keyboard, MAX_SPEED
import bus
import logging

logger = logging.getLogger(__name__)
auto_cooldown = 0
autonomous = False

left_wheel_pub = bus.Publisher('/bot/wheel/cmd_vel/left', float)
right_wheel_pub = bus.Publisher('/bot/wheel/cmd_vel/right', float)
autonomous_pub = bus.Publisher('/bot/cmd_auto', bool)


def update(_):
    global auto_cooldown, autonomous
    if auto_cooldown > 0:
        auto_cooldown -= 1

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

        left_wheel_pub.publish(vL)
        right_wheel_pub.publish(vR)

    if key == ord('S'):
        pass
        # mapping.mapper.save()
    elif key == ord('L'):
        pass
        # mapping.mapper.load()
    elif key == ord('A'):
        if auto_cooldown > 0:
            logger.debug('autonomous switching cooldown not finished')
        else:
            auto_cooldown = 100
            autonomous = not autonomous
            logger.info(f'auto state set to {autonomous}')
            autonomous_pub.publish(autonomous)

    elif key == ord('1'):
        pass
        # upper_height()
    elif key == ord('2'):
        pass
        # lower_height()
    elif key == ord('3'):
        pass
        # above_basket()
    elif key == ord('R'):
        global gripper_status
        gripper_status = 'closed'


bus.Subscriber('/bot/cmd_tick', int, update)
