import base64
import cv2
import zmq
import socket
import time
import argparse
import subprocess
import ipaddress

# BROADCAST_IP = subprocess.check_output('hostname -I', shell=True).decode().split(" ")[0]
# SUBNET_MASK = subprocess.check_output('ip addr show', shell=True).decode().split(f"inet {BROADCAST_IP}/")[-1].split(" ")[0]
# DISCOVERY_IP = ipaddress.IPv4Network(f"{BROADCAST_IP}/{SUBNET_MASK}", strict=False).broadcast_address.exploded

# def broadcast_ip(discovery_port, discovery_timeout):
#     broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

#     connection_countdown = discovery_timeout  # seconds
#     while connection_countdown > 0:
#         broadcast_socket.sendto(f'IP_BROADCASTER:tcp://{BROADCAST_IP}:{discovery_port}'.encode('utf8'), (DISCOVERY_IP, discovery_port))
#         time.sleep(1)
#         connection_countdown -= 1

def broadcast_camera_data(port, camera_id):
    context = zmq.Context()
    footage_socket = context.socket(zmq.PUB)
    footage_socket.bind(f'tcp://*:{port}') # 172.20.10.3

    footage_socket2 = context.socket(zmq.PUB)
    footage_socket2.bind(f'tcp://*:{5556}') # 172.20.10.3

    footage_socket3 = context.socket(zmq.PUB)
    footage_socket3.bind(f'tcp://*:{5557}') # 172.20.10.3

    footage_socket4 = context.socket(zmq.PUB)
    footage_socket4.bind(f'tcp://*:{5558}') # 172.20.10.3

    camera = cv2.VideoCapture(camera_id)  # init the camera
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

    while True:
        try:
            grabbed, frame = camera.read()  # grab the current frame
            encoded, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer)
            footage_socket.send(jpg_as_text)
            footage_socket2.send(jpg_as_text)
            footage_socket3.send(jpg_as_text)
            footage_socket4.send(jpg_as_text)

        except KeyboardInterrupt:
            camera.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    # Necessary arguments:
    # - auto-ip-discovery
    # - discovery-port
    # - discovery-timeout
    # - broadcast-port
    # - camera-id
    parser = argparse.ArgumentParser(prog='opencv_streamer', description='Streams camera data using opencv2')
    parser.add_argument('--auto-ip-discovery', default="off")
    parser.add_argument('--discovery-port', type=int, default=5556)
    parser.add_argument('--discovery-timeout', type=int, default=15)
    parser.add_argument('--broadcast-port', type=int, default=5555)
    parser.add_argument('--camera-id', type=int, default=0)

    args = parser.parse_args()
    auto_ip_discovery = args.auto_ip_discovery == "on"
    discovery_port = args.discovery_port
    discovery_timeout = args.discovery_timeout
    broadcast_port = args.broadcast_port
    camera_id = args.camera_id

    # if auto_ip_discovery:
    #     print(f"Broadcasting IP on port {discovery_port} for {discovery_timeout} seconds...")
    #     broadcast_ip(discovery_port, discovery_timeout)
    print(f"Starting camera stream on port {broadcast_port}...")
    broadcast_camera_data(broadcast_port, camera_id)
    