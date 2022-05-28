import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob
import time
import pickle

def read_calibration(filename):
    """Read camera intrinsics from file
    inputs:
        filename: filename holding calibration result
    outputs:
        dict: dictionary holding camera internal parameters
    """
    with open(filename, 'rb') as handle:
        parsed = pickle.load(handle)
    return parsed

def undistort_image(img, k):
    """Undistort lens distortion in image
    inputs:
        img: input image array
        k: dictionary holding camera internal parameters
    outputs:
        dst: undistorted image array
    """
    # undistort
    dst = cv2.undistort(img, k['mtx'], k['dist'], None, k['nmtx'])
    # crop the image
    x, y, w, h = k['roi']
    dst = dst[y:y+h, x:x+w]
    dst = cv2.resize(dst, (k['image_width'], k['image_height']))
    return dst

def calibrate_camera(img_folder="images", out_file = "camera_intrinsics.pkl", cb_size=(9,6), debug=False):
    """Perform Intrinsic Camera Calibration
    inputs:
        img_folder: folder path for calibraiton images
        out_file: filename for saving calibration result
        cb_size: checkerboard grid size
        debug: whether to print debug message
    outputs:
        dst: camera parameters as a Python dictionary
    """
    tile = 1 # size = 1m?
    bw = cb_size[0]
    bh = cb_size[1]

    # termination criteria
    #criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((bh*bw,3), np.float32)
    objp[:,:2] = tile*np.mgrid[0:bw,0:bh].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob(f"{img_folder}/*.jpg")

    if debug is True:
        print("\nCheckerboard parameters")
        print(f"  Tile size: {tile}m")
        print(f"  Width:     {bw}")
        print(f"  Height:    {bh}")
        print(f"\nImages:{len(images)}")

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        h,  w = gray.shape[:2]

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (bw,bh),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            if debug is True:
                img = cv2.drawChessboardCorners(img, (bw,bh), corners2,ret)
                cv2.imshow('img',img)
                cv2.waitKey(500)


    if debug is True:
        print('objpoints:')
        print(objpoints)
        print('imgpoints:')
        print(imgpoints)

    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    tot_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        tot_error += error

    d = {}
    d['mtx'] = mtx
    d['dist'] = dist
    #d['rvecs'] = rvecs
    #d['tvecs'] = tvecs
    d['roi'] = roi
    d['nmtx'] = newcameramtx
    d['image_width'] = w
    d['image_height'] = h
    d['proj_error'] = tot_error/len(objpoints)

    with open(out_file, "wb") as f:
        pickle.dump(d, f, protocol = 4)

    if debug is True:
        print("\nCalibration result:")
        for item in d:
            print(item,':',d[item])
        print(f"\nResult saved to: {out_file}")

    return d
