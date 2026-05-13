#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

def main():
    rospy.init_node("csi0_pub")
    width  = rospy.get_param("~width", 1280)
    height = rospy.get_param("~height", 720)
    fps    = rospy.get_param("~fps", 30)
    frame_id = rospy.get_param("~frame_id", "csi0_optical_frame")
    camera_name = rospy.get_param("~camera_name", "csi0")

    pub_img  = rospy.Publisher("image_raw", Image, queue_size=1)
    pub_info = rospy.Publisher("camera_info", CameraInfo, queue_size=1)

    cim = CameraInfoManager(cname=camera_name)
    cim.loadCameraInfo()
    bridge = CvBridge()

    Gst.init(None)
    pipeline_str = (
        "nvarguscamerasrc sensor-id=0 ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=sink sync=false max-buffers=1 drop=true"
    )
    pipeline = Gst.parse_launch(pipeline_str)
    sink = pipeline.get_by_name("sink")
    pipeline.set_state(Gst.State.PLAYING)

    rate = rospy.Rate(fps)
    while not rospy.is_shutdown():
        sample = sink.emit("pull-sample")
        if sample is None:
            rospy.logwarn_throttle(2.0, "No sample yet...")
            rate.sleep()
            continue

        buf = sample.get_buffer()
        caps = sample.get_caps()
        s = caps.get_structure(0)
        w = int(s.get_value("width"))
        h = int(s.get_value("height"))

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            rospy.logwarn_throttle(2.0, "Failed to map buffer")
            rate.sleep()
            continue

        img = np.frombuffer(mapinfo.data, dtype=np.uint8).reshape((h, w, 3))
        buf.unmap(mapinfo)

        msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        pub_img.publish(msg)

        info = cim.getCameraInfo()
        info.header = msg.header
        pub_info.publish(info)

        rate.sleep()

    pipeline.set_state(Gst.State.NULL)

if __name__ == "__main__":
    main()
