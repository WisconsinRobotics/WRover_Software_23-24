import asyncio
import websockets
import zmq
import base64



async def receive_stream(port, stop_event, streamPort):

	asyncEvent = asyncio.Event()

	async def stream(websocket):
		context = zmq.Context()
		socket = context.socket(zmq.SUB)
		socket.connect(f"tcp://127.0.0.1:{streamPort}")  # Replace with your broadcaster IP if needed
		socket.setsockopt_string(zmq.SUBSCRIBE, '')

		while True:
			try:
				frame = socket.recv_string()
				await websocket.send(frame)
			except Exception as e:
				print("Error:", e)
				break

	async with websockets.serve(stream, "127.0.0.1", port):
		print(f"WebSocket server running on ws://localhost:{port}")
		await asyncEvent.wait()

	stop_event.wait()
	asyncEvent.set()

if __name__ == "__main__":
	asyncio.run(receive_stream(8081, asyncio.Event(), 5555))
