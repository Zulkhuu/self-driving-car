import cv2
import os
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import glob

class Line():
    def __init__(self):
        # was the line detected in the last iteration?
        self.detected = False
        self.buffer_size = 5
        # x values of the last n fits of the line
        self.recent_xfitted = []
        #average x values of the fitted line over the last n iterations
        self.bestx = None
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = None
        #polynomial coefficients for the most recent fit
        self.current_fit = [np.array([False])]
        #radius of curvature of the line in some units
        self.radius_of_curvature = None
        #distance in meters of vehicle center from the line
        self.line_base_pos = None
        #difference in fit coefficients between last and new fits
        self.diffs = np.array([0,0,0], dtype='float')
        #x values for detected line pixels
        self.allx = None
        #y values for detected line pixels
        self.ally = None

    def update(self, fit):
        self.detected = True
        self.current_fit = fit
        if len(self.recent_xfitted) == self.buffer_size:
            self.recent_xfitted = self.recent_xfitted[1:]
        self.recent_xfitted.append(fit)
        self.best_fit = np.average(np.array(self.recent_xfitted), axis=0)

    def print(self):
        print(f"Recent {len(self.recent_xfitted)}:{self.recent_xfitted}")
        print(f"Best:{self.best_fit}")
