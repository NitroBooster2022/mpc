import cv2
import numpy as np
from pykalman import KalmanFilter  # pip install pykalman

A = np.array(
    [[1, 0, 1, 0],
     [0, 1, 0, 1],
     [0, 0, 1, 0],
     [0, 0, 0, 1]])
H = np.array(
    [[1, 0, 0, 0],
     [0, 1, 0, 0]])
Q = np.eye(4) * 0.05

kf = KalmanFilter(
    transition_matrices=A,
    observation_matrices=H,
    transition_covariance=Q)

last_mean = np.zeros(shape=(4,))
last_covariances = np.eye(4)

lm_x, lm_y = 0, 0
lp_x, lp_y = 0, 0


def mouse_move_callback(event, m_x, m_y, flags, param):
    global kf
    global lm_x, lm_y, lp_x, lp_y
    global last_mean, last_covariances

    last_mean, last_covariances = kf.filter_update(last_mean,
                                                   last_covariances,
                                                   np.array([m_x, m_y]))

    px, py = int(last_mean[0]), int(last_mean[1])

    cv2.line(WINDOW_BG, (lm_x, lm_y), (m_x, m_y), (0, 255, 0), 2)
    cv2.line(WINDOW_BG, (lp_x, lp_y), (px, py), (0, 0, 255), 2)

    lp_x, lp_y = px, py
    lm_x, lm_y = m_x, m_y


WINDOW_NAME = 'Kalman Filter Tracker Demo'
WINDOW_BG = np.ones((768, 1024, 3), np.uint8) * 255
cv2.namedWindow(WINDOW_NAME)
cv2.setMouseCallback(WINDOW_NAME, mouse_move_callback)
while True:
    cv2.imshow(WINDOW_NAME, WINDOW_BG)
    if (cv2.waitKey(30) & 0xff) == 27:
        break