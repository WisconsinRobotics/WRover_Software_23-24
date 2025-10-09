import asyncio
import threading
import time
from streamReceiver import receive_stream
 
stop_event = threading.Event()

ports = [(8081, 5555), (8082, 5556), (8083, 5557), (8084, 5558)]  # List of ports to use for the WebSocket server

def run_receive_stream(port, streamPort):
    asyncio.run(receive_stream(port, stop_event, streamPort))

threads = []
for port in ports:
    thread = threading.Thread(target=run_receive_stream, args=(port[0], port[1]), daemon=True)
    print(f"Starting WebSocket server on port {port}...")
    threads.append(thread)
    thread.start()
    
# run_receive_stream(8081)


try:
    while True:
        time.sleep(100)
except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping all threads...")
    stop_event.set()