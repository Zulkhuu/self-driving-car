import numpy as np
import cv2

def pipeline(img, s_thresh=(170, 255), sx_thresh=(20, 100)):
    """Apply Color and Gradient thresholding pipeline
    inputs:
        img: RGB image as a 3 channel numpy array
        s_thresh: Threshold Ranges for Saturation channel
        sx_thresh: Threshold Ranges for Sobel X magnitude
    outputs:
        ret: one channel binary image
    """

    # Convert to HLS color space and separate the S channel
    gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    s_channel = hls[:,:,2]
    
    # Apply Sobel filter in horizontal direction
    sobelx = cv2.Sobel(gray_img, cv2.CV_64F, 1, 0) # Take the derivative in x
    abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))

    # Threshold x gradient magnitude
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1

    # Threshold Saturation channel
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
    
    # Combine S channel and gradient thresholds
    ret = np.zeros_like(s_channel)
    ret[(s_binary == 1) | (sxbinary == 1)] = 1
    
    return ret
