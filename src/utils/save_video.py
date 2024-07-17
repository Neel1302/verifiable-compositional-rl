# Neel P. Bhatt

import rclpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy
import cv2
import csv
import os
import getpass
import time

# Save directory
root_dir = '/home/{}/dev_ws/'.format(getpass.getuser())+"/Videos/"

if not os.path.exists(root_dir):
    os.makedirs(root_dir)

bridge = CvBridge()

init = True
display_image = False
out = None

def callback_Image(ros_image):
    global init, out

    image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8") # Convert ROS Image to CV2 Mat

    if init:
        print('Saving video ... ')
        init = False
        # Get image size
        image_y, image_x, image_channels = image.shape
        image_fps = 5.0

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(root_dir+'output.mp4', fourcc, image_fps, (image_x,image_y))


    out.write(image) # Save incoming frame
    
    if display_image:
        # Display image - comment out if you do not want to see the image
        cv2.namedWindow("Image")
        cv2.imshow("Image", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("User pressed ctrl+c")
            raise SystemExit
    
def video_saver(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('video_saver')

    node.create_subscription(Image, "/adk_node/SimpleFlight/front_center_custom/Scene", callback_Image, 1)

    # spin() simply keeps python from exiting until this node is stopped
    try:
        rclpy.spin(node)
    except:
        exit_cleanly()

def exit_cleanly():
    out.release() # Release frame buffer
    cv2.destroyAllWindows()
    print("Shutting down the ros node ...")

if __name__ == '__main__':
    video_saver()