import rospy
import sys
import threading
import cv2
from sensor_msgs.msg import Image
from vimba import *
from alvium_driver import get_camera, setup_camera
from cv_bridge import CvBridge, CvBridgeError


class Handler4ros:
    def __init__(self, publisher):
        self.shutdown_event = threading.Event()
        self.publisher = publisher
        self.bridge = CvBridge()

    def __call__(self, cam: Camera, frame: Frame):
        ENTER_KEY_CODE = 13

        key = cv2.waitKey(1)
        if key == ENTER_KEY_CODE:
            # self.shutdown_event.set()
            # return
            self.close_properly()

        elif frame.get_status() == FrameStatus.Complete:
            # print('{} acquired {}'.format(cam, frame), flush=True)
            try:
                # ros_img = self.bridge.cv2_to_imgmsg(frame.as_opencv_image(), encoding="passthrough")
                self.publisher.publish(self.bridge.cv2_to_imgmsg(frame.as_opencv_image(), encoding="passthrough"))

                # you can compare the ros img and the cv img here - they r the same!!! :- )
                # msg = 'Stream from \'{}\'. Press <Enter> to stop stream.'
                # cv2.imshow(msg.format(cam.get_name()), frame.as_opencv_image())
            except CvBridgeError as e:
                print(e)



        cam.queue_frame(frame)

    def close_properly(self):
        self.shutdown_event.set()
        rospy.loginfo("Alvium camera driver closed properly.")
        return


def main(args):

    pub = rospy.Publisher('pp/rgb_raw', Image, queue_size=1)
    rospy.init_node('pp_alvium_python_driver', anonymous=False)
    rospy.loginfo("Alvium driver initialised")

    #TODO make that changeable
    rate = rospy.Rate(10)

    # while not rospy.is_shutdown():
    #     cv_img = "hello world %s" % rospy.get_time()
    #
    #     pub.publish(cv_img)
    #     rate.sleep()

    cam_id = 0
    frequency = 30

    rospy.loginfo("Opening Vimba ...")
    with Vimba.get_instance():
        with get_camera(cam_id) as cam:

            # Start Streaming, wait for five seconds, stop streaming
            setup_camera(cam)
            handler = Handler4ros(pub)
            rospy.loginfo("Alvium camera of id={} opened with intended fps={}".format(cam_id, frequency))

            try:
                # Start Streaming with a custom a buffer of 10 Frames (defaults to 5)
                cam.start_streaming(handler=handler, buffer_count=frequency)
                handler.shutdown_event.wait()

            finally:
                cam.stop_streaming()
                # handler.close_properly()
                rospy.on_shutdown(handler.close_properly())

    rospy.on_shutdown(handler.close_properly())

if __name__ == '__main__':
    main(sys.argv)