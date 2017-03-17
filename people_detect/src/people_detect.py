#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
import cv2
import numpy

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError


class people_detect:
  
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
    self.img = []
  
  def inside(self, r, q):
    rx, ry, rw, rh = r
    qx, qy, qw, qh = q
    return rx > qx and ry > qy and rx + rw < qx + qw and ry + rh < qy + qh


  def draw_detections(self, img, rects, thickness = 1):
    for x, y, w, h in rects:
        # the HOG detector returns slightly larger rectangles than the real objects.
        # so we slightly shrink the rectangles to get a nicer output.
        pad_w, pad_h = int(0.15*w), int(0.05*h)
        cv2.rectangle(img, (x+pad_w, y+pad_h), (x+w-pad_w, y+h-pad_h), (0, 255, 0), thickness)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    self.img=cv_image

    hog = cv2.HOGDescriptor()
    hog.setSVMDetector( cv2.HOGDescriptor_getDefaultPeopleDetector() )
        
    found, w = hog.detectMultiScale(self.img, winStride=(4,4), padding=(16,16), scale=1.05)
    found_filtered = []
    for ri, r in enumerate(found):
        for qi, q in enumerate(found):
            if ri != qi and self.inside(r, q):
               break
        else:
            found_filtered.append(r)
    self.draw_detections(self.img, found)
    self.draw_detections(self.img, found_filtered, 3)
    rospy.loginfo('%d (%d) found' % (len(found_filtered), len(found)))

    cv2.imshow("Image window", self.img)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
    except CvBridgeError as e:
      print(e)




def main(args):
  people = people_detect()
  rospy.init_node('people_detect', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

