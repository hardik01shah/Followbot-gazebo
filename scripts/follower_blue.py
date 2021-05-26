#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                                       Twist, queue_size=1)
    self.twist = Twist()
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_blue = numpy.array([110,50,50])
    upper_blue = numpy.array([130,255,255])
    lower_yellow = numpy.array([ 20,  100,  100])
    upper_yellow = numpy.array([30, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask_blue[0:search_top, 0:w] = 0
    mask_blue[search_bot:h, 0:w] = 0
    mask_yellow[0:search_top, 0:w] = 0
    mask_yellow[search_bot:h, 0:w] = 0
    MB = cv2.moments(mask_blue)
    MY = cv2.moments(mask_yellow)
    if MB['m00'] > 0:
      cx = int(MB['m10']/MB['m00'])
      cy = int(MB['m01']/MB['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL
    else:
      try:
        cx = int(MY['m10']/MY['m00'])
        cy = int(MY['m01']/MY['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        # BEGIN CONTROL
        err = cx - w/2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twist)
        # END CONTROL
      except:
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
      
    win1 = cv2.resize(image, (380, 210))
    cv2.imshow("window", win1)
    win2 = cv2.resize(mask_yellow, (380, 210))
    cv2.imshow("yellow_hsv", win2)
    win3 = cv2.resize(mask_blue, (380, 210))
    cv2.imshow("blue_hsv", win3)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
