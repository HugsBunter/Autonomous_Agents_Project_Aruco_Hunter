#!/usr/bin/env python3

import cv2 
import numpy as np
import cv2.aruco as aruco
import rospy
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()

cameraMatrix = np.array([[528.433756558705, 0.0, 320.5],
                        [0.0, 528.433756558705, 240.5],
                        [0.0, 0.0, 1.0]])

distortionCoefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

bridge = CvBridge()

TwistMsg = Twist

twist = TwistMsg()

twist.linear.x = 0
twist.linear.y = 0
twist.linear.z = 0
twist.angular.x = 0
twist.angular.y = 0
twist.angular.z = 0

def callback(data):

  global pub
  global start_time 
  global timeout  
  global timeout_ang
  global start_time_ang
  global timeout_ns      #ns = no signal
  global start_time_ns
  global timeout_mf
  global start_time_mf
  global timeout_final
  global start_time_final
    
  current_frame = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

  mID_flag = False

  (corners, ids, rejected) = cv2.aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParams, cameraMatrix= cameraMatrix, distCoeff=distortionCoefficients)

  if len(corners) > 0:
    # flatten the ArUco IDs list
    ids = ids.flatten()
    # loop over the detected ArUCo corners
    for (markerCorner, markerID) in zip(corners, ids):
      # extract the marker corners (which are always returned in
      # top-left, top-right, bottom-right, and bottom-left order)
      corners2 = markerCorner.reshape((4, 2))
      (topLeft, topRight, bottomRight, bottomLeft) = corners2
      # convert each of the (x, y)-coordinate pairs to integers
      topRight = (int(topRight[0]), int(topRight[1]))
      bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
      bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
      topLeft = (int(topLeft[0]), int(topLeft[1]))

      # draw the bounding box of the ArUCo detection
      cv2.line(current_frame, topLeft, topRight, (0, 255, 0), 2)
      cv2.line(current_frame, topRight, bottomRight, (0, 255, 0), 2)
      cv2.line(current_frame, bottomRight, bottomLeft, (0, 255, 0), 2)
      cv2.line(current_frame, bottomLeft, topLeft, (0, 255, 0), 2)
      cv2.circle(current_frame, topLeft, 4, (0, 0, 255), -1)
      cv2.circle(current_frame, topRight, 4, (0, 0, 255), -1)
      cv2.circle(current_frame, bottomLeft, 4, (0, 0, 255), -1)
      cv2.circle(current_frame, bottomRight, 4, (0, 0, 255), -1)

      # compute and draw the center (x, y)-coordinates of the ArUco marker
      cX = int((topLeft[0] + bottomRight[0]) / 2.0)
      cY = int((topLeft[1] + bottomRight[1]) / 2.0)
      cv2.circle(current_frame, (cX, cY), 4, (0, 0, 255), -1)

      if np.all(ids is not None):  # If there are markers found by detector
            for i in range(0, len(ids)):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 1.0, cameraMatrix,
                                                                          distortionCoefficients)
              
                x_cam = tvec[0][0][0]
                y_cam = tvec[0][0][1]
                z_cam = tvec[0][0][2]

                
                
                (rvec - tvec).any()  # get rid of the numpy value array error
                aruco.drawDetectedMarkers(current_frame, corners)  # Draw a square around the markers
                aruco.drawAxis(current_frame, cameraMatrix, distortionCoefficients, rvec, tvec, 0.01)  # Draw Axis

      # draw the ArUco marker ID on the image
      cv2.putText(current_frame, str(markerID),
        (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (0, 255, 0), 2)

      if markerID == 5:
        mID_flag = True
      else:
         mID_flag = False

      print(f"Marker ID {markerID} and {mID_flag}")
  
  # The case when a marker id detected. The robot should either correct its angle
  # towards the aruco target or go ahead until the distance between them is zero.
  if mID_flag:
    timeout_final = -1.0
    timeout_ns = -1.0
    timeout_mf = -1.0
    thita = math.atan(x_cam/z_cam)
    if thita < -0.0872664626:
      twist.angular.z = -0.5
      timeout_ang = rospy.Duration(thita / twist.angular.z).to_sec()
      start_time_ang = rospy.Time.now()
      pub.publish(twist)
    elif thita > 0.0872664626:
      twist.angular.z = 0.5
      timeout_ang = rospy.Duration(thita / twist.angular.z).to_sec()
      start_time_ang = rospy.Time.now()
      pub.publish(twist)
    else :
      twist.angular.z = 0
      twist.linear.x = 2
      distance = math.sqrt(x_cam ** 2 + z_cam ** 2)
      print("Distance: ", distance)
      timeout = rospy.Duration(distance / twist.linear.x).to_sec()
      print("Timeout: ", timeout)
      timeout_ang = -1.0
      start_time = rospy.Time.now()
      pub.publish(twist)
  
  # The case when a marker id is not detected. The robot should start searching for an aruco 
  # target. It has only 60 seconds till it stuck in eternity (or an aruco target get in front of it).
  else:
     if (timeout_ang == -1.0) and (timeout == -1.0) and (timeout_ns == -1.0) and (timeout_mf == -1.0) and (timeout_final != -2.0):
        twist.angular.z = 0.5
        timeout_ns = rospy.Duration(6.2831853072 / twist.angular.z).to_sec()
        start_time_ns = rospy.Time.now()
        pub.publish(twist)
        print("360 deg searching started.")

     if (timeout_final == -1.0):
        timeout_final = rospy.Duration(120).to_sec()
        start_time_final = rospy.Time.now()
        print("Final countdown started.")
        
  
  # Stop angular move when no signal (with timeout).
  if (timeout_ang != -1.0) and (start_time_ang is not None) and ((rospy.Time.now() - start_time_ang).to_sec() >= timeout_ang):
    twist.angular.z = 0
    twist.linear.x = 2
    pub.publish(twist)
    timeout_ang = -1.0
    start_time_ang = None
    print("Angular move stopped. Going a little bit ahead.")

  # Stop when target is reached (with timeout).
  elif (timeout != -1.0) and (start_time is not None) and ((rospy.Time.now() - start_time).to_sec() >= timeout):
    twist.angular.z = 0
    twist.linear.x = 0
    pub.publish(twist)
    timeout = -1.0
    start_time = None
    print("You have reached your destination.")

  # Never found signal in the 360 deg search (with timeout). Start the circular move.
  elif (timeout_ns != -1.0) and (start_time_ns is not None) and ((rospy.Time.now() - start_time_ns).to_sec() >= timeout_ns):
    twist.angular.z = 0.5
    twist.linear.x = 2
    pub.publish(twist)
    timeout_ns = -1.0
    start_time_ns = None
    timeout_mf = rospy.Duration(5).to_sec()
    start_time_mf = rospy.Time.now()
    print("360 deg search stopped. Circular move started.")

  # Stop the circular move (with timeout).
  elif (timeout_mf != -1.0) and (start_time_mf is not None) and ((rospy.Time.now() - start_time_mf).to_sec() >= timeout_mf):
    twist.angular.z = 0
    twist.linear.x = 0
    pub.publish(twist)
    timeout_mf = -1.0
    start_time_mf = None
    print("Circular move stopped.")

  # Stop forever (with timeout).
  if (timeout_final != -1.0) and (start_time_final is not None) and ((rospy.Time.now() - start_time_final).to_sec() >= timeout_final):
    twist.angular.z = 0
    twist.linear.x = 0
    pub.publish(twist)
    timeout_final = -2.0
    start_time_final = None
    print("Stuck in eternity. Waiting to be rescued by an aruco marker!")

    rospy.logwarn("timer")
       
  # show the output image
  cv2.imshow("Image", current_frame)
  cv2.waitKey(1)

def listener():

  global pub 
  global start_time 
  global timeout 
  global timeout_ang
  global start_time_ang
  global timeout_ns      #ns = no signal
  global start_time_ns
  global timeout_mf
  global start_time_mf
  global timeout_final
  global start_time_final

  rospy.init_node('video_sub', anonymous=True)

  start_time_ang = None
  start_time = None
  start_time_ns = None
  start_time_final = None
  start_time_mf = None

  timeout = -1.0
  timeout_ang = -1.0
  timeout_ns = -1.0
  timeout_final = -1.0
  timeout_mf = -1.0

  cb = callback

  pub = rospy.Publisher('cmd_vel', TwistMsg, queue_size = 1)

  sub = rospy.Subscriber("/image_raw", Image, cb)

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

if __name__ == '__main__':
    listener()