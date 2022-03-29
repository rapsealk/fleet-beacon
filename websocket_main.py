import asyncio
import os

import redis
from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse
from starlette.websockets import WebSocketDisconnect
from websockets.exceptions import ConnectionClosedError


def create_app() -> FastAPI:
    app = FastAPI()

    html = """
    <!DOCTYPE html>
    <html>
        <head>
            <title>Chat</title>
        </head>
        <body>
            <h1>WebSocket Chat</h1>
            <form action="" onsubmit="sendMessage(event)">
                <input type="text" id="messageText" autocomplete="off"/>
                <button>Send</button>
            </form>
            <ul id='messages'>
            </ul>
            <script>
                var ws = new WebSocket("ws://localhost:8000/ws");
                ws.onmessage = function(event) {
                    var messages = document.getElementById('messages')
                    var message = document.createElement('li')
                    var content = document.createTextNode(event.data)
                    message.appendChild(content)
                    messages.appendChild(message)
                };
                function sendMessage(event) {
                    var input = document.getElementById("messageText")
                    ws.send(input.value)
                    input.value = ''
                    event.preventDefault()
                }
            </script>
        </body>
    </html>
    """

    @app.get("/")
    async def get():
        return HTMLResponse(html)

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        print(f"[WebSocket::{os.getpid()}] Accepted: {websocket}")
        # Get Redis Pub/Sub
        pubsub = redis.Redis(host="localhost", port=6379, db=0).pubsub()
        pubsub.subscribe("global_position")

        while True:
            try:
                if message := pubsub.get_message(timeout=5):
                    message_type = message.get("type", None)
                    if message_type == "subscribe":
                        pass
                    elif message_type == "message":
                        message_data = message.get("data", None)
                        print(f"[Redis::{os.getpid()}] Message: {message_data}")
                        await websocket.send_text(f"{{\"message\": {message}}}")    # ConnectionClosedError
                else:
                    print(f"[Redis::{os.getpid()}] Message is None! ({message})")
                await asyncio.sleep(0.001)
            except WebSocketDisconnect as e:
                print(f"[WebSocket::{os.getpid()}] Disconnected: {e}")
                break
            except ConnectionClosedError as e:
                print(f"[WebSocket::{os.getpid()}] ConnectionClosed: {e}")
                break

    return app


# uvicorn websocket_main:app --host 0.0.0.0 --port 8000 --workers 4
app = create_app()
