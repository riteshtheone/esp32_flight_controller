import websocket
ws = websocket.WebSocket()
ws.connect("ws://192.168.4.1/ws")
while True:
    result = ws.recv()
    print(result)