import cv2
import os
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import glob

from camera_calibration import read_calibration, undistort_image
from color_and_gradient import pipeline
from warp_perspective import read_warp, warp_image

from line import Line
from lane import fit_poly, find_lane_pixels, search_around_poly

class LaneFinder():
    def __init__(self):
        self.rlane = Line()
        self.llane = Line()
        self.int_params = read_calibration("intrinsics.p")
        self.warp_params = read_warp("perspective.p")
        self.unwarp_params = np.linalg.inv(self.warp_params)
        self.img_size = (1280, 720)
        self.img_thresh = np.zeros((1280, 720))
        self.img_warped = np.zeros((1280, 720, 3))
        self.img_warped_bin = np.zeros((1280, 720))

    def detected(self):
        return self.rlane.detected and self.llane.detected

    def undistort_image(self, img):
        return undistort_image(img, self.int_params)

    def threshold_image(self, img):
        return pipeline(img)

    def warp_image(self, img):
        return warp_image(img, self.warp_params)

    def process(self, img):
        img_rect = self.undistort_image(img)
        self.img_thresh = self.threshold_image(img_rect)
        self.img_warped_bin = self.warp_image(self.img_thresh)

        if self.detected() is False:
            #print("fitting")
            self.fit_polynomial()
        else:
            #print("searching")
            self.search_around_poly()

        return self.draw_lane(img_rect)

    def fit_polynomial(self, ret_img=False):
        binary_warped = self.img_warped_bin
        # Find our lane pixels first
        leftx, lefty, rightx, righty, out_img = find_lane_pixels(binary_warped)
        left_fit, right_fit = fit_poly(leftx, lefty, rightx, righty)

        # Fit a second order polynomial to each using `np.polyfit`
        #left_fit = np.polyfit(lefty, leftx, 2)
        #right_fit = np.polyfit(righty, rightx, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        try:
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
            self.llane.update(left_fit, leftx, lefty)
            self.rlane.update(right_fit, rightx, righty)
        except TypeError:
            # Avoids an error if `left` and `right_fit` are still none or incorrect
            left_fitx = 1*ploty**2 + 1*ploty
            right_fitx = 1*ploty**2 + 1*ploty
            self.rlane.detected = False
            self.rlane.detected = False

        if ret_img is True:
            out_img[lefty, leftx] = [255, 0, 0]
            out_img[righty, rightx] = [0, 0, 255]
            return out_img

    def search_around_poly(self):
        # Fit new polynomials
        leftx, lefty, rightx, righty, self.img_warped = search_around_poly(self.img_warped_bin, self.llane.current_fit, self.rlane.current_fit)
        left_fit, right_fit = fit_poly(leftx, lefty, rightx, righty)

        try:
            ploty = np.linspace(0, self.img_warped_bin.shape[0]-1, self.img_warped_bin.shape[0] )
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
            if left_fit[2] > self.img_warped_bin.shape[1]//2 or right_fit[2] < self.img_warped_bin.shape[1]//2:
                self.rlane.detected = False
                self.rlane.detected = False
            else:
                self.llane.update(left_fit, leftx, lefty)
                self.rlane.update(right_fit, rightx, righty)
        except TypeError:
            # Avoids an error if `left` and `right_fit` are still none or incorrect
            left_fitx = 1*ploty**2 + 1*ploty
            right_fitx = 1*ploty**2 + 1*ploty
            self.rlane.detected = False
            self.rlane.detected = False

    def draw_lane(self, img):
        Minv = self.unwarp_params
        left_fit = self.llane.best_fit
        right_fit = self.rlane.best_fit
        left_fit = self.llane.current_fit
        right_fit = self.rlane.current_fit

        # add thresholded and warped image previews
        h, w = img.shape[:2]
        h, w = h//4, w//4
        thresh_resized = cv2.resize(self.img_thresh, dsize=(w, h))
        thresh_resized = np.dstack([thresh_resized, thresh_resized, thresh_resized]) * 255
        img[50:h+50, 50:50+w, :] = thresh_resized

        warped_resized = cv2.resize(self.img_warped, dsize=(w, h))
        img[50:h+50, 100+w:100+2*w, :] = warped_resized

        ploty = np.array(np.linspace(0, img.shape[0]-1, img.shape[0]))
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        # Create an image to draw the lines on
        color_warp = np.zeros_like(img).astype(np.uint8)

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int32(pts), (0,255, 0))

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, Minv, (img.shape[1], img.shape[0]))
        # Combine the result with the original image
        result = cv2.addWeighted(img, 1, newwarp, 0.3, 0)

        np.set_printoptions(precision=2)
        #result = cv2.putText(result, f"Left:{left_fit}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (255, 0, 0), 2, cv2.LINE_AA)
        #result = cv2.putText(result, f"Right:{right_fit}", (50, 150), cv2.FONT_HERSHEY_SIMPLEX , 1, (255, 0, 0), 2, cv2.LINE_AA)
        #plt.imshow(result)
        return result
