import serial
import cv2
import numpy as np
import time
import gym
import pygame
pygame.init()
screen = pygame.display.set_mode((800, 400)) #x and y are height and width
observations = ['x_val', 'y_val', 'old_x_val', 'old_y_val']

def make_obs_space():
    lower_obs_bound = {
        'x_val': 0,
        'y_val': 0,
        'old_x_val': 0,
        'old_y_val': 0,
    }
    higher_obs_bound = {
        'x_val': 640,
        'y_val': 360,
        'old_x_val': 640,
        'old_y_val': 360,
    }

    low = np.array([lower_obs_bound[o] for o in observations])
    high = np.array([higher_obs_bound[o] for o in observations])
    shape = (len(observations),)
    return gym.spaces.Box(low,high,shape)
def forward(serial_connection):
    serial_connection.write(b'F')
    time.sleep(0.1)
def rest(serial_connection):
    serial_connection.write(b'N')
    time.sleep(0.1)
def backwards(serial_connection):
    serial_connection.write(b'B')
    time.sleep(0.1)

actions = [forward, rest, backwards]
class Pendulum(gym.Env):
    def __init__(self, max_steps=1000):
        self.last = None
        self.actions = actions
        self.observations = observations
        self.action_space =gym.spaces.Discrete(len(actions))
        self.observation_space = make_obs_space()
        self.log = ''
        self.max_steps = max_steps
        self.rest_position = 350

        self.ser = serial.Serial('COM9', 9800, timeout=1)
        self.cap = cv2.VideoCapture(1)


    def reset(self, **kwargs):

        self.ser.write(b'N')
        time.sleep(10)

        self.steps_left = self.max_steps
        val = None
        while val is None:
            _, frame = self.cap.read()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            blur = cv2.GaussianBlur(hsv, (5, 5),
                                    cv2.BORDER_DEFAULT)
            lower_red = np.array([22, 130, 80])
            upper_red = np.array([45, 255, 255])
            mask = cv2.inRange(blur, lower_red, upper_red)
            th, threshed = cv2.threshold(mask, 100, 255,
                                         cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
            cnts = cv2.findContours(threshed, cv2.RETR_LIST,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
            s1 = 60
            s2 = 10000
            for cnt in cnts:
                if s1 < cv2.contourArea(cnt) < s2:

                    M = cv2.moments(cnt)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])

                        val = [cx, cy,cx, cy]
                        self.last = [cx, cy]
                        self.rest_position = cx
        return np.array(val)
    def step(self, action):
        self.actions[action](self.ser)

        val = None
        while val is None:
            _, frame = self.cap.read()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            blur = cv2.GaussianBlur(hsv, (5, 5),
                                    cv2.BORDER_DEFAULT)
            lower_red = np.array([22, 130, 80])
            upper_red = np.array([45, 255, 255])
            mask = cv2.inRange(blur, lower_red, upper_red)
            res = cv2.bitwise_and(frame, frame, mask=mask)
            th, threshed = cv2.threshold(mask, 100, 255,
                                         cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
            cnts = cv2.findContours(threshed, cv2.RETR_LIST,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
            s1 = 60
            s2 = 10000
            self.ser.write(b'N')
            for cnt in cnts:
                if s1 < cv2.contourArea(cnt) < s2:

                    M = cv2.moments(cnt)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])

                        val = [cx, cy, self.last[0], self.last[1]]
                        self.last = [cx, cy]

        self.steps_left -= 1
        done = (self.steps_left <= 0)
        reward = (val[0] - self.rest_position)/100
        if reward < 0:
            reward = -reward
        return np.array(val), reward, done, ""

    def close(self):
        pass

    def render(self, mode=None):
        pygame.event.get()
        screen.fill((0, 0, 0))
        pygame.draw.line(screen, (255, 255, 255), (318, 0), (self.last[0], self.last[1]), 2)
        pygame.draw.circle(screen, (255, 255, 255), (self.last[0], self.last[1]), 10)
        pygame.display.flip()
