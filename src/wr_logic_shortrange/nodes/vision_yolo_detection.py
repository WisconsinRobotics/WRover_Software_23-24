import subprocess

import cv2 as cv
import numpy as np
import rospy
import torch
from ultralytics import YOLO

from wr_logic_shortrange.msg import VisionTarget

## Width of the camera frame, in pixels
# CAMERA_WIDTH = 640
CAMERA_WIDTH = 1280
## Height of the camera frame, in pixels
# CAMERA_HEIGHT = 480
CAMERA_HEIGHT = 720
## Frames per second
CAMERA_FPS = 5

FOCAL_LENGTH_MM = 1360.17

OBJECT_NAME_TO_ID = {
    "hammer": VisionTarget.OBJ_MALLET,
    "bottle": VisionTarget.OBJ_BOTTLE,
}


def draw_bounding_box(frame: np.ndarray, box: torch.Tensor, label: str) -> np.ndarray:
    x1, y1, x2, y2 = box
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

    frame = cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    return cv.putText(
        frame,
        text=label,
        org=(x1, y1),
        fontFace=cv.FONT_HERSHEY_PLAIN,
        fontScale=1.5,
        color=(0, 0, 255),
    )


def generate_vision_msg(box: torch.Tensor, label: str):
    x1, y1, x2, y2 = box
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

    # Find long edge and short edge
    x_edge = abs(x2 - x1)
    y_edge = abs(y2 - y1)
    long_edge = max(x_edge, y_edge)
    short_edge = min(x_edge, y_edge)

    # x offset is returned as -1 to 1
    x_offset = (x1 + x2 - CAMERA_WIDTH) / CAMERA_WIDTH

    distance = 0
    target_id = VisionTarget.ANY
    if label in OBJECT_NAME_TO_ID:
        target_id = OBJECT_NAME_TO_ID[label]
        if target_id == VisionTarget.OBJ_MALLET:
            # length of mallet handle in mm
            long_edge_distance = 305 * FOCAL_LENGTH_MM / (long_edge * 1000)
            # length of mallet head in mm
            short_edge_distance = 100 * FOCAL_LENGTH_MM / (short_edge * 1000)
            distance = max(long_edge_distance, short_edge_distance)
        elif target_id == VisionTarget.OBJ_BOTTLE:
            # height of water bottle in mm
            long_edge_distance = 215 * FOCAL_LENGTH_MM / (long_edge * 1000)
            # diameter of water bottle in mm
            short_edge_distance = 90 * FOCAL_LENGTH_MM / (short_edge * 1000)
            distance = max(long_edge_distance, short_edge_distance)

    # assume we are close to object
    return VisionTarget(id=target_id, x_offset=x_offset, distance=distance)


def main():
    rospy.init_node("vision_aruco_detection")

    rate = rospy.Rate(10)

    # Set up publisher
    vision_topic = rospy.get_param("~vision_topic")
    pub = rospy.Publisher(vision_topic, VisionTarget, queue_size=10)

    # Retrieve video stream from parameter server
    # If no video capture is specified, try to use /dev/video0
    stream_path = rospy.get_param("~video_stream")
    if stream_path is not None and stream_path != "":
        cap = cv.VideoCapture(stream_path)
    else:
        cap = cv.VideoCapture(0)

    cap.set(cv.CAP_PROP_FPS, CAMERA_FPS)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    # Load model
    # TODO add scripts/documentation for TensorRT
    model_path = rospy.get_param("~model_path", "yolov8n.engine")
    model = YOLO(model_path)

    # Stream video if debugging
    stream = None
    debug = rospy.get_param("~debug", False)
    stream_port = rospy.get_param("~debug_port", 8000)
    if debug:
        ffmpeg_command = [
            "ffmpeg",
            # Input format
            "-f",
            "rawvideo",
            # OpenCV stores images as BGR
            "-pixel_format",
            "bgr24",
            # Video resolution
            "-s",
            f"{CAMERA_WIDTH}x{CAMERA_HEIGHT}",
            # Framerate
            "-r",
            f"{CAMERA_FPS}",
            # Pipe video to ffmpeg
            "-i",
            "-",
            # Encode video with h264
            "-vcodec",
            "libx264",
            "-preset",
            "ultrafast",
            "-tune",
            "zerolatency",
            "-b:v",
            "8M",
            # Stream using MPEG-TS over UDP
            "-f",
            "mpegts",
            f"udp://192.168.1.44:{stream_port}",
        ]
        stream = subprocess.Popen(ffmpeg_command, stdin=subprocess.PIPE)

        # Command for viewing stream
        # ffplay -fflags nobuffer -flags low_delay -probesize 32 -analyzeduration 1 -strict experimental -framedrop -f mpegts -vf setpts=0 udp://192.168.1.134:8000

    if not cap.isOpened():
        rospy.logerr("Failed to open camera")
        exit()

    while not rospy.is_shutdown():
        # Read frame and publish detected targets
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to read frame")
        else:
            results = model(frame, stream=True)
            if results is not None:
                for r in results:
                    for box in r.boxes:
                        box_label = model.names[int(box.cls[0])]
                        if debug:
                            frame = draw_bounding_box(frame, box.xyxy[0], box_label)

            if debug:
                stream.stdin.write(frame.tobytes())

        rate.sleep()

    if debug:
        stream.stdin.close()
        stream.wait()
    cap.release()


if __name__ == "__main__":
    main()
