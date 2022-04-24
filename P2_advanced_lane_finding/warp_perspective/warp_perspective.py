import cv2
import numpy as np
import matplotlib.pyplot as plt
import pickle

def calibrate_warp(out_file = "perspective.p", src=None, dst=None):
    if src is None or dst is None:
        # Hardcoded values
        src = [[248, 720], [1089, 720], [706, 460], [592, 460]]
        dst = [[256, 720], [1024, 720], [1024, 0], [256, 0]]

    M = cv2.getPerspectiveTransform(np.array(src, dtype=np.float32),np.array(dst, dtype=np.float32))

    with open(out_file, "wb") as f:
        pickle.dump(M, f, protocol=pickle.HIGHEST_PROTOCOL)

    return M

def read_warp(filename):
    with open(filename, 'rb') as handle:
        parsed = pickle.load(handle)
    return parsed

def warp_image(img, M):
    img_size = (img.shape[1], img.shape[0])
    return cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)
