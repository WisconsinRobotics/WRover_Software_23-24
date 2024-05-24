import subprocess

import cv2 as cv
import rospy
from ultralytics import YOLO

from wr_logic_shortrange.msg import VisionTarget

## Width of the camera frame, in pixels
CAMERA_WIDTH = 640
## Height of the camera frame, in pixels
CAMERA_HEIGHT = 480
## Frames per second
CAMERA_FPS = 30

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
    model_path = rospy.get_param("~model_path", "yolov8n.pt")
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
                    x1, y1, x2, y2 = r.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    cv.rectangle(frame, (x1, y1), (x2, y2))

                    cv.putText(
                        frame,
                        text=model.names[r.cls[0]],
                        org=(x1, y1),
                        fontFace=cv.FONT_HERSHEY_PLAIN,
                        fontScale=1.5,
                        color=(0, 0, 255),
                    )

            if debug:
                stream.stdin.write(frame.tobytes())

        rate.sleep()

    if debug:
        stream.stdin.close()
        stream.wait()
    cap.release()




if __name__ == "__main__":
    main()
