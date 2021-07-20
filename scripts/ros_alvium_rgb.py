import rospy
import sys
import threading
import yaml
# import cv2
from signal import signal, SIGINT
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from vimba import *
# this must be like this to properly install Python package in ROS
# see http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
from pp_alvium_driver.alvium_driver import get_camera, setup_camera
from cv_bridge import CvBridge, CvBridgeError
from pathlib import Path


class Handler4ros:
    def __init__(self, img_publisher, cam_info_publisher, yaml_fname):
        self.shutdown_event = threading.Event()
        self.img_publisher = img_publisher
        self.cam_info_publisher = cam_info_publisher
        self.bridge = CvBridge()
        if yaml_fname is not None:
            rospy.loginfo("Reading parameters from the calibration file: " + yaml_fname)
            self.cam_info_params_msg = Handler4ros.read_calibration_configuration(yaml_fname)
            if self.cam_info_params_msg is None:
                rospy.logerr("Could not read from the calibration file!")
        # self.f = open("./210517_time_diffs.txt", "a")

    def __call__(self, cam: Camera, frame: Frame):
        # ENTER_KEY_CODE = 13
        #
        # key = cv2.waitKey(1)
        # if key == ENTER_KEY_CODE:
        #     # self.shutdown_event.set()
        #     # return
        #     self.close_properly()

        # elif frame.get_status() == FrameStatus.Complete:
        if frame.get_status() == FrameStatus.Complete:
            # print('{} acquired {}'.format(cam, frame), flush=True)
            try:
                # ros_img = self.bridge.cv2_to_imgmsg(frame.as_opencv_image(), encoding="passthrough")
                # encodings rgb8 odwraca kolory, see here: http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
                # and bgr8 seems to work well
                time_stamp_before = rospy.Time.now()
                ros_img_msg = self.bridge.cv2_to_imgmsg(frame.as_opencv_image(), encoding="bgr8")
                raw_time_stamp = rospy.Time.now()
                # diff_secs = time_stamp_before.secs - raw_time_stamp.secs
                duration_diff = raw_time_stamp - time_stamp_before
                # print(str(diff_secs))
                # diff_nanosecs = time_stamp_before.nsecs - raw_time_stamp.nsecs
                # print(str(diff_nanosecs))
                # print(str(duration_diff))
                # print(str(raw_time_stamp))
                # print(str(raw_time_stamp-duration_diff))
                # print(str(raw_time_stamp - (duration_diff/2)))
                time_stamp = raw_time_stamp-(duration_diff/2)
                # self.f.write(str(diff_nanosecs)+"\n")
                # ros_time_stamp = frame.get_timestamp()
                # denominator = 1000000000 # for nano seconds which i 1e-9
                # secs = int(ros_time_stamp //denominator)
                # print(time_stamp.secs - 1621000800) # 1621000800 is unix time of 2021-05-15 at 15:00
                # print(secs)
                # print("diff is: " + str(time_stamp.secs - 1621000800 - secs))
                # print("diff: " + str(time_stamp.secs-secs))
                # nanosecs = (ros_time_stamp % denominator)
                # print("diff is: " + str(time_stamp.nsecs - nanosecs))
                # ros_time_stamp = rospy.Time() # secs=nanosecs=0
                # ros_time_stamp.secs = secs
                # ros_time_stamp.nsecs = nanosecs
                # ros_img_msg.header.stamp = ros_time_stamp

                ros_img_msg.header.stamp = time_stamp
                self.cam_info_params_msg.header.stamp = time_stamp
                # self.cam_info_params_msg.header.stamp = ros_time_stamp
                ros_img_msg.header.frame_id = self.cam_info_params_msg.header.frame_id
                self.img_publisher.publish(ros_img_msg)
                self.cam_info_publisher.publish(self.cam_info_params_msg)

                # you can compare the ros img and the cv img here - they r the same!!! :- )
                # msg = 'Stream from \'{}\'. Press <Enter> to stop stream.'
                # cv2.imshow(msg.format(cam.get_name()), frame.as_opencv_image())
            except CvBridgeError as e:
                print(e)

        cam.queue_frame(frame)

    def handler_f(self, signal_received, frame):
        # Handle any cleanup here
        self.close_properly()
        # self.f.close()
        # print('SIGINT or CTRL-C detected. Exiting gracefully')
        exit(0)

    def close_properly(self):
        # self.f.close()
        self.shutdown_event.set()
        rospy.loginfo("Alvium camera's thread closing ...")
        return

    @staticmethod
    def read_calibration_configuration(yaml_fname):

        path2file = Path(yaml_fname)
        if path2file.is_file():
            with open(yaml_fname, "r") as file_handle:
                calib_data = yaml.load(file_handle)
            # Parse
            camera_info_msg = CameraInfo()
            camera_info_msg.width = calib_data["image_width"]
            camera_info_msg.height = calib_data["image_height"]
            camera_info_msg.K = calib_data["camera_matrix"]["data"]
            camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
            #camera_info_msg.R = calib_data["rectification_matrix"]["data"]
            #camera_info_msg.P = calib_data["projection_matrix"]["data"]
            #camera_info_msg.distortion_model = calib_data["camera_model"]
            camera_info_msg.header.frame_id = calib_data["camera_name"]
            return camera_info_msg
        else:
            return None

def main(args):

    # try:
    #     cam_id = str(rospy.get_param("/ros_alvium_driver/rbg_cam_id"))
    #     frequency = int(rospy.get_param("/ros_alvium_driver/rgb_frequency"))
    #     cam_info = bool(rospy.get_param("/ros_alvium_driver/rbg_camera_info"))
    #     if cam_info:
    #         yaml_fname = str(rospy.get_param("/ros_alvium_driver/calibration_yaml"))
    #     else:
    #         yaml_fname = None
    # except:
    #     rospy.logerr("Alvium camera driver couldn't load my config parameters")

    pub_img = rospy.Publisher('pp/rgb_raw', Image, queue_size=1)
    pub_cam_info = rospy.Publisher('pp/rgb_cam_info', CameraInfo, queue_size=1)
    rospy.init_node('pp_alvium_python_driver', anonymous=True)
    rospy.loginfo("Alvium driver initialised")

    ##########################################
    ### RGB SETTINGS - REPLACING PARAMS.YAML
    ##########################################
    cam_id = "DEV_1AB22C00A470"
    frequency = 45
    cam_info = True
    #yaml_fname = "/home/maciej/ros1_wss/pp_collector/src/pp_alvium_driver/calib/210408_no_opt_constraints.yml"
    # yaml_fname = "/home/maciej/ros/py3_ws/src/pp_alvium_driver/calib/210408_no_opt_constraints.yml"
    yaml_fname = "../calib/rgb1.yml"
    ##########################################


    # cam_info works properly
    # if cam_info:
    #     rospy.loginfo("Got True")
    # else:
    #     rospy.loginfo("Got False")


    rospy.loginfo("Opening Vimba ...")
    with Vimba.get_instance():
        with get_camera(cam_id) as cam:

            # Start Streaming, wait for five seconds, stop streaming
            setup_camera(cam)
            handler = Handler4ros(pub_img, pub_cam_info, yaml_fname)
            # this handles CTRL+C to close the node properly
            signal(SIGINT, handler.handler_f)
            rospy.loginfo("Alvium camera of id={} opened with intended fps={}".format(cam_id, frequency))

            try:
                # Start Streaming with a custom a buffer of 10 Frames (defaults to 5)
                cam.start_streaming(handler=handler, buffer_count=frequency)
                handler.shutdown_event.wait()

            finally:
                cam.stop_streaming()
                rospy.loginfo("Alvium camera driver closed properly.")
                # handler.close_properly()
                # rospy.on_shutdown(handler.close_properly())

    # that does not seem to work so uncommented
    # rospy.on_shutdown(close_cam(cam))

if __name__ == '__main__':
    main(sys.argv)
