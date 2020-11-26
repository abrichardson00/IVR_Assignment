#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class joint_angles:

  def __init__(self):
    rospy.init_node('joint_angles_node', anonymous = True)
    self.img1_subscriber = message_filters.Subscriber("/image_topic1", Image)
    self.img2_subscriber = message_filters.Subscriber("/image_topic2", Image)
        
    time_sync = message_filters.TimeSynchronizer([self.img1_subscriber, self.img2_subscriber], 1)
    time_sync.registerCallback(self.callback)
    
    self.bridge = CvBridge()
    
    self.joint_angles_publisher = rospy.Publisher("joint_angles", Float64MultiArray, queue_size=1)
    #self.base_joint_location_pub = rospy.Publisher("/base_joint_location", Float64MultiArray, queue_size=1)
    self.end_effector_location_pub = rospy.Publisher("/end_effector_location", Float64MultiArray, queue_size=1)
    self.target_location_pub = rospy.Publisher("/target_location", Float64MultiArray, queue_size=1)
    # initialise variables to keep track of target position on image
    self.target_xz_centroid = np.array([0,0])
    self.target_yz_centroid = np.array([0,0])
    

  def callback(self, yz_image_msg, xz_image_msg):
    try:
      self.yz_image = self.bridge.imgmsg_to_cv2(yz_image_msg, "bgr8")
      self.xz_image = self.bridge.imgmsg_to_cv2(xz_image_msg, "bgr8")
      self.image_height,self.image_width = self.yz_image.shape[:2] # <- assume yz and xz images are the same
      #print("height, width: " + str(self.yz_image.shape))
      yz_image_normalized = self.normalizeRGB(self.yz_image)
      xz_image_normalized = self.normalizeRGB(self.xz_image)
      
      #cv2.imshow('yz_view', yz_image_normalized)
      #cv2.imshow('xz_view', xz_image_normalized)
      #cv2.waitKey(1)
      
      red_centroid_yz = self.get_joint_center(yz_image_normalized, [(0, 0, 100), (7, 7, 255)], 'red_yz', erosion=0, dilation=2)
      green_centroid_yz = self.get_joint_center(yz_image_normalized, [(0, 100, 0), (7, 255, 7)], 'green_yz', erosion=0, dilation=2)
      blue_centroid_yz = self.get_joint_center(yz_image_normalized, [(100, 0, 0), (255, 7, 7)], 'blue_yz', erosion=0, dilation=2)
      yellow_centroid_yz = self.get_joint_center(yz_image_normalized, [(0, 100, 100), (7, 255, 255)], 'yellow_yz', erosion=0, dilation=2)
      
      red_centroid_xz = self.get_joint_center(xz_image_normalized, [(0, 0, 100), (7, 7, 255)], 'red_xz', erosion=0, dilation=2)
      green_centroid_xz = self.get_joint_center(xz_image_normalized, [(0, 100, 0), (7, 255, 7)], 'green_xz', erosion=0, dilation=2)
      blue_centroid_xz = self.get_joint_center(xz_image_normalized, [(100, 0, 0), (255, 7, 7)], 'blue_xz', erosion=0, dilation=2)
      yellow_centroid_xz = self.get_joint_center(xz_image_normalized, [(0, 100, 100), (7, 255, 255)], 'yellow_xz', erosion=0, dilation=2)
      
      yz_centroids = np.array([yellow_centroid_yz, blue_centroid_yz, green_centroid_yz, red_centroid_yz])
      xz_centroids = np.array([yellow_centroid_xz, blue_centroid_xz, green_centroid_xz, red_centroid_xz])
      
      centered_yz_centroids = self.center_image_coordinates_around_first_joint(yz_centroids[0],yz_centroids[1:])
      centered_xz_centroids = self.center_image_coordinates_around_first_joint(xz_centroids[0],xz_centroids[1:])
      
      coordinates_3d = self.merge_plane_coordinates(centered_yz_centroids, centered_xz_centroids)

      

      self.compute_joint_angles(coordinates_3d)
      
      joint_angles = self.compute_joint_angles(coordinates_3d)
      # publish joint angles
      joint_angles_payload = Float64MultiArray()
      joint_angles_payload.data = joint_angles
      self.joint_angles_publisher.publish(joint_angles_payload)
      
      # find distance between yellow and blue spheres, and we know the real metre distance is 2.5m
      yellow_to_blue_dist = np.linalg.norm(coordinates_3d[1])
      metres_per_pixel_ratio = 2.5/yellow_to_blue_dist

      #cv2.circle(self.xz_image,(int(red_centroid_xz[0]),int(red_centroid_xz[1])),5,(255,0,0),2)
      #cv2.circle(self.yz_image,(int(red_centroid_yz[0]),int(red_centroid_yz[1])),5,(255,0,0),2)

      # get and publish end effector position (red sphere position)
      red_sphere_position = coordinates_3d[3]*metres_per_pixel_ratio
      end_effector_payload = Float64MultiArray()
      end_effector_payload.data = red_sphere_position
      self.end_effector_location_pub.publish(end_effector_payload)

      # get orange sphere image coords
      self.update_target_location(yz_image_normalized,xz_image_normalized)
      #cv2.circle(self.xz_image,(self.target_xz_centroid[0],self.target_xz_centroid[1]),5,(0,0,255),2) # display dot on our 'target location'
      #cv2.circle(self.yz_image,(self.target_yz_centroid[0],self.target_yz_centroid[1]),5,(0,0,255),2)
      centred_target_yz_centroid = self.center_image_coordinates_around_first_joint(yellow_centroid_yz,np.array([self.target_yz_centroid]))[1]
      centred_target_xz_centroid = self.center_image_coordinates_around_first_joint(yellow_centroid_xz,np.array([self.target_xz_centroid]))[1]
      target_location = self.merge_plane_coordinates([centred_target_yz_centroid],[centred_target_xz_centroid])[0]
      target_location = target_location*metres_per_pixel_ratio
      #print(target_location)
      
      # publish target location:
      target_location_payload = Float64MultiArray()
      target_location_payload.data = target_location
      self.target_location_pub.publish(target_location_payload)

      # uncomment if one needs to display images
      #im1=cv2.imshow('window1', self.xz_image)
      #im1=cv2.imshow('window2', self.yz_image)
      #cv2.waitKey(1)
      
    except CvBridgeError as e:
      print(e)
   

  def update_target_location(self,yz_image_normalized,xz_image_normalized):
    # get orange shapes
    # try get image location of sphere, if one cant - use previously found position
    yz_image_coord = self.get_target_image_coord(yz_image_normalized)
    if yz_image_coord is not None:
      self.target_yz_centroid = yz_image_coord
    xz_image_coord = self.get_target_image_coord(xz_image_normalized)
    if xz_image_coord is not None:
      self.target_xz_centroid = xz_image_coord
    

  def get_target_image_coord(self,image):
    ### process image
    orange_thresholds = [(0, 64, 70), (20, 100, 255)]
    orange_image = self.threshold_and_dilate(image,orange_thresholds,2)

    ### get all orange objects in image
    contours,hierarchy = cv2.findContours(orange_image,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#cv2.findContours(orange_yz, 1, 2)
    area_ratios = []
    centres = []
    for contour in contours:
      contour = cv2.approxPolyDP(contour, 0.5, True)
      #cv2.drawContours(self.xz_image, [contour], -1, (255, 0, 0), 1)
      #print("contour: " + str(contour.shape))
      M = cv2.moments(contour)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      centres.append(np.array([cx,cy]))
      (x,y,w,h) = cv2.boundingRect(contour)
      
      #cv2.rectangle(image, (x,y),(x+w,y+h),(0,255,0),1)
      # get area of bounding box
      bbox_area = w*h
      # get ratio of bounding box area filled by orange shape
      shape_area = np.sum(orange_image[y:y+h,x:x+w] * (1.0/255.0))
      area_ratios.append(shape_area/bbox_area)

      
    ### handle each relevant case where we detect a different number of orange objects:
    if len(contours) == 1:
      return centres[0]
    if len(contours) == 2: # check if one shape is definitely a circle or rectangle (if we know one we can infer the other)
      # check if definitely a rectangle 
      if contours[0].shape[0] == 4 or area_ratios[0] > 0.95:
        return centres[1] # then other shape is sphere
      if contours[1].shape[0] == 4 or area_ratios[1] > 0.95:
        return centres[0]
      # if we cant find definite rectangle, actually check for a circle
      for i in range(2):
        (x,y),r = cv2.minEnclosingCircle(contours[i])
        min_circle_area = 3.14*r*r
        circleness = cv2.contourArea(contours[i])/min_circle_area
        if circleness > 0.9:
          # definentely a circle
          return centres[i]
    
    if len(contours) == 3: # one shape is definitely a sphere or rectangle - other contours come from split shape
      is_rectangle = [False,False,False]
      for i in range(3):
        # check if defenently a rectangle
        if contours[i].shape[0] == 4 or area_ratios[i] > 0.95:
          is_rectangle[i] = True
      if np.sum(np.array(is_rectangle))==1: # we have exactly 1 rectangle, other 2 come from obscured sphere
        s_ind = [n for n in range(3) if not is_rectangle[n]] # <- other indices for shapes that are not rectangle
        return ((centres[s_ind[0]][0] + centres[s_ind[1]][0])//2,(centres[s_ind[0]][1] + centres[s_ind[1]][1])//2) # return middle of 2 split sphere shapes
      # otherwise actually check for single circle:
      for i in range(3):
        (x,y),r = cv2.minEnclosingCircle(contours[i])
        min_circle_area = 3.14*r*r
        circleness = cv2.contourArea(contours[i])/min_circle_area
        if circleness > 0.9:
          # definentely a circle
          return centres[i]
    return None
    

  def threshold_and_dilate(self,image,thresholds,iteration_num):
    binary_image = cv2.inRange(image, thresholds[0], thresholds[1])
    kernel = np.ones((2, 2), np.uint8)
    dilated_image = cv2.dilate(binary_image, kernel, iterations=2)
    return dilated_image

 
  def compute_joint_angles(self, joint_coordinates):
    blue_coordinates = joint_coordinates[1]
    green_coordinates = joint_coordinates[2]
    
    blue_green_vector = green_coordinates - blue_coordinates
    joint_2_angle = -np.arctan2(blue_green_vector[1],blue_green_vector[2])
    
    joint_coordinates_rotated_around_x = self.rotate_around_x_axis(-joint_2_angle, joint_coordinates)
    
    blue_coordinates_rotated_around_x = joint_coordinates_rotated_around_x[1]
    green_coordinates_rotated_around_x = joint_coordinates_rotated_around_x[2]
    
    blue_green_vector_rotated_around_x = green_coordinates_rotated_around_x - blue_coordinates_rotated_around_x
    joint_3_angle = np.arctan2(blue_green_vector_rotated_around_x[0], blue_green_vector_rotated_around_x[2])
    
    joint_coordinates_rotated_around_x_and_y = self.rotate_around_y_axis(joint_3_angle, joint_coordinates_rotated_around_x)
    
    #print(joint_coordinates_rotated_around_x_and_y)
    
    green_coordinates_rotated_around_x_and_y = joint_coordinates_rotated_around_x_and_y[2]
    red_coordinates_rotated_around_x_and_y = joint_coordinates_rotated_around_x_and_y[3]
    
    green_red_vector_rotated_around_x_and_y = red_coordinates_rotated_around_x_and_y - green_coordinates_rotated_around_x_and_y
    joint_4_angle = -np.arctan2(green_red_vector_rotated_around_x_and_y[1], green_red_vector_rotated_around_x_and_y[2])
    
    
    joint_angles = np.array([joint_2_angle, joint_3_angle, joint_4_angle])
    
    #print(joint_angles)
    
    return joint_angles
  
  
  def rotate_around_y_axis(self, angle, coordinates):
    new_coordinates = np.zeros((len(coordinates),3), dtype=np.int64)
    rotation_matrix = np.array([[np.cos(angle), 0, -np.sin(angle)],[0, 1, 0],[np.sin(angle), 0, np.cos(angle)]])
    
    for i in range(len(coordinates)):
      new_coordinates[i] = rotation_matrix.dot(coordinates[i])
      
    return new_coordinates
  
  def rotate_around_x_axis(self, angle, coordinates):
    
    new_coordinates = np.zeros((len(coordinates),3), dtype=np.int64)
    rotation_matrix = np.array([[1, 0, 0],[0, np.cos(angle), -np.sin(angle)],[0, np.sin(angle), np.cos(angle)]])
    
    for i in range(len(coordinates)):
      new_coordinates[i] = rotation_matrix.dot(coordinates[i])
      
    return new_coordinates
    
  
  def merge_plane_coordinates(self, yz_coordinates, xz_coordinates):
    coordinates_3d = np.zeros((len(yz_coordinates),3))
    for i in range(len(yz_coordinates)):
      xz = xz_coordinates[i]
      yz = yz_coordinates[i]
      
      if(xz[0] == None): 
        xz = self.infer_coordinates(i, xz_coordinates, yz_coordinates)
      
      elif(yz[0] == None):
        yz = self.infer_coordinates(i, yz_coordinates, xz_coordinates)
        
      coordinates_3d[i] = np.array([xz[0], yz[0], yz[1]])
    
    return coordinates_3d
    
      
  def infer_coordinates(self, i, loss_coordinates, coordinates_for_inference):
    the_same_point_in_different_plane = coordinates_for_inference[i]
    
    closest_point_index_on_z_axis = None
    
    for j in range(len(coordinates_for_inference)):
      if(j == i):
        continue
      
      if(closest_point_index_on_z_axis == None):
        closest_point_index_on_z_axis = j
        continue
      elif(abs(the_same_point_in_different_plane[1]-coordinates_for_inference[j,1]) < abs(the_same_point_in_different_plane[1]-coordinates_for_inference[closest_point_index_on_z_axis,1])):
        closest_point_index_on_z_axis = j
        continue
    
    inferred_coordinates = np.array([loss_coordinates[closest_point_index_on_z_axis, 0],coordinates_for_inference[i,1]])
    
    return inferred_coordinates
        
  
  def center_image_coordinates_around_first_joint(self, first_joint, other_joints):
    
    new_coordinates = np.zeros((len(other_joints)+1,2))
    new_coordinates[0] = np.array([0,0])
    
    for i in range(len(other_joints)):
      if(other_joints[i,0] == None):
        new_coordinates[i+1] = other_joints[i]
        continue
      
      new_coordinates[i+1] = other_joints[i]-first_joint
      new_coordinates[i+1,1] = new_coordinates[i+1,1]*(-1)
    
    return new_coordinates
      
    
  
  def get_joint_center(self, image, thresholds, color_name, erosion=0, dilation=0):
    binary_image = cv2.inRange(image, thresholds[0], thresholds[1])
    
    kernel = np.ones((2, 2), np.uint8)
    eroded_image = cv2.erode(binary_image, kernel, iterations=erosion)
    dilated_image = cv2.dilate(eroded_image, kernel, iterations=dilation)
    final_image = dilated_image
    
    #cv2.imshow(color_name, final_image)
    #cv2.waitKey(1)
    
    #contours,hierarchy = cv2.findContours(final_image, 1, 2)
    
    #if(len(contours) == 0):
      #return np.array([None,None])
    
    #contour = contours[0]
    #M = cv2.moments(contour)
    
    M = cv2.moments(final_image)
    
    #compactness = (2*np.sqrt(M['m00']*np.pi))/cv2.arcLength(contour,True)
    
    if(M['m00'] == 0):
      return np.array([None, None])
    
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    centroid = np.array([cx, cy])
    
    #print(color_name, 'centroid', centroid)
    # return final_image, centroid
    return centroid
  
  def normalizeRGB(self, img):
  
    height, width = img.shape[:2]
  
    normalized_rgb = np.zeros((height,width,3),np.float64)
    
    img_float = img.astype(np.float64)
    
    b = img_float[:,:,0]
    g = img_float[:,:,1]
    r = img_float[:,:,2]
    
    rgb_sum = b + g + r
    rgb_sum[rgb_sum == 0] = 1
    
    normalized_rgb[:,:,0]=(b/rgb_sum)*255.0
    normalized_rgb[:,:,1]=(g/rgb_sum)*255.0
    normalized_rgb[:,:,2]=(r/rgb_sum)*255.0
    
    #print(normalized_rgb)
    
    normalized_rgb = normalized_rgb.astype(np.uint8)
    
    #normalized_rgb=cv2.convertScaleAbs(normalized_rgb)
    return normalized_rgb
    
    
  ### DO NOT USE
  def normalizeRGB_slow(self, img):
    for row in img:
      for pixel in row:
        rgb_sum = int(pixel[0])+int(pixel[1])+int(pixel[2])
        if(rgb_sum==0):
          continue
        pixel[0] = np.uint8(round((int(pixel[0])*255)/rgb_sum))
        pixel[1] = np.uint8(round((int(pixel[1])*255)/rgb_sum))
        pixel[2] = np.uint8(round((int(pixel[2])*255)/rgb_sum))
        
        


if __name__ == '__main__':
  joint_angles()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting Down")
