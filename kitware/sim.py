#!/usr/bin/env python3

from matplotlib.ft2font import VERTICAL
import rclpy
from rclpy.node import Node
from kitware.msg import DriveCmd
import pygame
import numpy as np

# Rate of simulation time [Hz]
RATE = 10

# Robot parameters
L = 0.1 # wheel base diameter [m]
R = 0.03 # wheel radius [m]

# Some constants for the GUI
PADDING = 32
BOX_SIZE = 64
SCREEN_SIZE = BOX_SIZE * 3 + PADDING * 2

BG_COLOR = (255, 255, 255)
BOT_COLOR = (50, 50, 50)
BOT_RADIUS = 10

def pygame_setup():
    # Setup the GUI
    pygame.init()
    pygame.display.set_caption('KitBot Simulator')
    screen = pygame.display.set_mode((SCREEN_SIZE, SCREEN_SIZE), 0, 32)
    surface = pygame.Surface(screen.get_size()).convert()
    return screen, surface

def simulate(screen, surface, speeds):
    surface.fill(BG_COLOR)

    l_speed, r_speed = speeds

    # forward kinematics
    # convert wheel speeds to translation/rotation velocities
    velT = R/2*(l_speed+r_speed)
    velR = 


    # integrate the speed to get position
    position = (SCREEN_SIZE/2, SCREEN_SIZE/2)
    pygame.draw.circle(surface, BOT_COLOR, position, BOT_RADIUS)
    screen.blit(surface, (0, 0))
    pygame.display.flip()
    pygame.display.update()

class KitSimNode(Node):
    """ROS2 Node that simulates the KitBot and returns sensor data"""
    def __init__(self):
        super().__init__('kitsim')
        self.screen, self.surface = pygame_setup()
        # Create a subscriber to listen for drive motor commands
        self.drive_sub = self.create_subscription(
            DriveCmd,
            'drive_cmd',
            self.sim_callback,
            10)
        self.drive_sub  # prevent unused variable warning
    
    def sim_callback(self, msg):
        simulate(self.screen, self.surface, (msg.l_speed, msg.r_speed))


if __name__ == '__main__':
    rclpy.init()

    kb = KitSimNode()
    rclpy.spin(kb)

    kb.destroy_node()  # Destroys the ROS node
    rclpy.shutdown()
