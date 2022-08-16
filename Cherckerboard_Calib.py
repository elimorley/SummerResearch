import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
row_num =9
col_num =6
objp = np.zeros((row_num * col_num,3), np.float32)
objp[:,:2] = np.mgrid[0:col_num,0:row_num].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('to_use/*.jpg')

count = 0

for fname in images:
    print(fname)
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (col_num,row_num), None)
    print(ret)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
       
        

        # Draw and display the corners
        cv.drawChessboardCorners(img, (col_num,row_num), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        print("mtx ", mtx)
        print("dist ", dist)
        print("rvecs ",rvecs)
        print("tvecs ", tvecs)

        h,  w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
            
        # undistort method 1
        # dst = cv.undistort(img, mtx, dist, None, newcameramtx)
        
        # # crop the image
        # x, y, w, h = roi
        # dst = dst[y:y+h, x:x+w]
        # cv.imwrite("calibresult"+fname+'.png', dst)
        # count =  count + 1  

        # undistort method 2
        mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
        dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv.imwrite('calibresult.png', dst)
     
    
    
 
   
#cv.destroyAllWindows()


