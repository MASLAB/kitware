#!/usr/bin/env python

import pygame
import rospy
import kitware.msg
import sys

# Rate to check for keypresses and send commands
RATE = 10

# Some constants for the GUI
PADDING = 32
BOX_SIZE = 64
SCREEN_SIZE = BOX_SIZE * 3 + PADDING * 2
KEY_SPEEDS = {
    pygame.K_w: (0.60, 0.60),
    pygame.K_a: (-0.40, 0.40),
    pygame.K_s: (-0.60, -0.60),
    pygame.K_d: (0.40, -0.40)
}
BOX_OFFSETS = {
    pygame.K_w: (0, -1),
    pygame.K_a: (-1, 0),
    pygame.K_s: (0, 1),
    pygame.K_d: (1, 0)
}
BG_COLOR = (255, 255, 255)
BOX_COLOR = (0, 0, 0)


def kbd_driver():
    # Setup the GUI
    pygame.init()
    pygame.display.set_caption('KitBot Keyboard Driver')
    screen = pygame.display.set_mode((SCREEN_SIZE, SCREEN_SIZE), 0, 32)
    surface = pygame.Surface(screen.get_size()).convert()

    # Setup ROS and a publisher for the drive commands
    rospy.init_node('kbd_driver', anonymous=True)
    pub = rospy.Publisher('drive_cmd', kitware.msg.DriveCMD, queue_size=10)
    rate = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        # Calculate new drive speeds and GUI coordinates from keypresses
        drive_speed = 0, 0
        box_pos = PADDING + BOX_SIZE, PADDING + BOX_SIZE
        pressed = pygame.key.get_pressed()
        show_dir = False
        for keycode in [pygame.K_w, pygame.K_a, pygame.K_s, pygame.K_d]:
            if pressed[keycode]:
                drive_speed = (drive_speed[0] + KEY_SPEEDS[keycode][0],
                               drive_speed[1] + KEY_SPEEDS[keycode][1])
                box_pos = (box_pos[0] + BOX_OFFSETS[keycode][0] * BOX_SIZE,
                           box_pos[1] + BOX_OFFSETS[keycode][1] * BOX_SIZE)
                show_dir = True

        # Update the GUI
        surface.fill(BG_COLOR)
        if show_dir:
            r = pygame.Rect(box_pos, (BOX_SIZE, BOX_SIZE))
            pygame.draw.rect(surface, BOX_COLOR, r)
        screen.blit(surface, (0, 0))
        pygame.display.flip()
        pygame.display.update()

        # Publish a drive command
        drive_cmd_msg = kitware.msg.DriveCMD()
        drive_cmd_msg.l_speed, drive_cmd_msg.r_speed = drive_speed
        pub.publish(drive_cmd_msg)

        # Process event queue and sleep between loops
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        rate.sleep()


if __name__ == '__main__':
    try:
        kbd_driver()
    except rospy.ROSInterruptException:
        pass
