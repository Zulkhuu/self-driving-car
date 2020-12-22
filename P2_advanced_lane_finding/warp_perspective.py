import cv2
import numpy as np
import matplotlib.pyplot as plt
import pickle

'''
straight_images = glob.glob("test_images/straight_lines*.jpg")

images = []
m = read_calibration("intrinsics.p")
for image in straight_images:
    img = mpimg.imread(image)
    img_rect = undistort_image(img, m)
    images.append(img)

h, w = images[0].shape[:2]
print(w,h)
src = np.int32([[w/6 - 10, h], [w*5/6 + 50, h],
                [w/2 + 56, h/2 + 100], [w/2 - 55, h/2 + 100]])
dst = np.int32([[w/5, h], [w*4/5, h],
                [w*4/5, 0], [w/5, 0]])
'''

def calibrate_warp(out_file = "perspective.p", src=None, dst=None):
    if src is None or dst is None:
        # Hardcoded values
        src = [[203, 720], [1116, 720], [696, 460], [585, 460]]
        dst = [[256, 720], [1024, 720], [1024, 0], [256, 0]]

    M = cv2.getPerspectiveTransform(np.array(src, dtype=np.float32),np.array(dst, dtype=np.float32))

    with open(out_file, "wb") as f:
        pickle.dump(M, f, protocol=pickle.HIGHEST_PROTOCOL)

    return M

def read_warp(filename):
    with open(filename, 'rb') as handle:
        parsed = pickle.load(handle)
    return parsed
