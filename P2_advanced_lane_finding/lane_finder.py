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

class LaneFinder():
    def __init__(self):
        self.rlane = Line()
        self.llane = Line()
        self.int_params = read_calibration("intrinsics.p")
        self.warp_params = read_warp("perspective.p")
        self.unwarp_params = np.linalg.inv(self.warp_params)
        self.img_size = (1280, 720)
        self.img_thresh = np.zeros((1280, 720))
        self.img_warped_bin = np.zeros((1280, 720))
        self.img_warped = np.zeros((1280, 720, 3))

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

    def find_lane_pixels(self,binary_warped):
        # Take a histogram of the bottom half of the image
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        # Create an output image to draw on and visualize the result
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # HYPERPARAMETERS
        # Choose the number of sliding windows
        nwindows = 9
        # Set the width of the windows +/- margin
        margin = 100
        # Set minimum number of pixels found to recenter window
        minpix = 50

        # Set height of windows - based on nwindows above and image shape
        window_height = np.int(binary_warped.shape[0]//nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated later for each window in nwindows
        leftx_current = leftx_base
        rightx_current = rightx_base

        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            # Draw the windows on the visualization image
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),
            (win_xleft_high,win_y_high),(0,255,0), 2)
            cv2.rectangle(out_img,(win_xright_low,win_y_low),
            (win_xright_high,win_y_high),(0,255,0), 2)

            # Identify the nonzero pixels in x and y within the window #
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        #try:
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        #except ValueError:
        #    # Avoids an error if the above is not implemented fully
        #    pass

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        return leftx, lefty, rightx, righty, out_img

    def fit_polynomial(self, ret_img=False):
        binary_warped = self.img_warped_bin
        # Find our lane pixels first
        leftx, lefty, rightx, righty, out_img = self.find_lane_pixels(binary_warped)

        # Fit a second order polynomial to each using `np.polyfit`
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        try:
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
            self.llane.update(left_fit)
            self.rlane.update(right_fit)
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

    def fit_poly(self, leftx, lefty, rightx, righty):
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        return left_fit, right_fit

    def search_around_poly(self):
        binary_warped = self.img_warped_bin
        # HYPERPARAMETER
        left_fit = self.llane.current_fit
        right_fit = self.rlane.current_fit
        margin = 100

        # Grab activated pixels
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy +
                        left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) +
                        left_fit[1]*nonzeroy + left_fit[2] + margin)))
        right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy +
                        right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) +
                        right_fit[1]*nonzeroy + right_fit[2] + margin)))

        # Again, extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        self.img_warped = np.dstack([self.img_warped_bin, self.img_warped_bin, self.img_warped_bin]) * 255
        self.img_warped[lefty, leftx] = [255, 0, 0]
        self.img_warped[righty, rightx] = [0, 0, 255]

        # Fit new polynomials
        left_fit, right_fit = self.fit_poly(leftx, lefty, rightx, righty)

        try:
            ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
            self.llane.update(left_fit)
            self.rlane.update(right_fit)
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
        result = cv2.putText(result, f"Left:{left_fit}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (255, 0, 0), 2, cv2.LINE_AA)
        result = cv2.putText(result, f"Right:{right_fit}", (50, 150), cv2.FONT_HERSHEY_SIMPLEX , 1, (255, 0, 0), 2, cv2.LINE_AA)
        #plt.imshow(result)
        return result
