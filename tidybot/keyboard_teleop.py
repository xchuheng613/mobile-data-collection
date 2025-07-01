import os
import signal
import time
import numpy as np
import pygame
from base_controller import Vehicle

pygame.init()
screen = pygame.display.set_mode((300, 100))  # required to receive key events
pygame.display.set_caption("Keyboard Teleop")

def key_velocity(keys):
    vx = vy = vth = 0.0
    if keys[pygame.K_w]:
        vx += 1
    if keys[pygame.K_s]:
        vx -= 1
    if keys[pygame.K_d]:
        vy += 1
    if keys[pygame.K_a]:
        vy -= 1
    if keys[pygame.K_q]:
        vth -= 1
    if keys[pygame.K_e]:
        vth += 1
    return np.array([vx, vy, vth], dtype=np.float32)

class KeyboardTeleop:
    def __init__(self):
        self.vehicle = Vehicle(max_vel=(1.0, 1.0, 3.14), max_accel=(0.5, 0.5, 2.36))
        self.enabled = False

    def run(self):
        print("Press SPACE to enable control, ESC to exit.")
        while True:
            pygame.event.pump()
            keys = pygame.key.get_pressed()

            if keys[pygame.K_ESCAPE]:
                break

            if keys[pygame.K_SPACE] and not self.enabled:
                print("Control started")
                self.vehicle.start_control()
                self.enabled = True

            if self.enabled:
                v = key_velocity(keys)
                if np.linalg.norm(v) > 0:
                    scaled_v = v * self.vehicle.max_vel
                    print("[teleop] enqueue", scaled_v)  
                    self.vehicle.set_target_velocity(scaled_v, frame="local")
                else:
                    # If no key pressed, stop
                    self.vehicle.set_target_velocity(np.zeros(3), frame="local")

            time.sleep(0.01)

        if self.enabled:
            print("Control stopped")
            self.vehicle.stop_control()

# Clean exit on SIGTERM
def handler(signum, frame):
    os.kill(os.getpid(), signal.SIGINT)
signal.signal(signal.SIGTERM, handler)

if __name__ == '__main__':
    teleop = KeyboardTeleop()
    try:
        teleop.run()
    finally:
        if teleop.enabled:
            teleop.vehicle.stop_control()
