import asyncio
import os
from typing import Iterable

import redis
from fastapi import FastAPI, WebSocket
from starlette.websockets import WebSocketDisconnect
from websockets.exceptions import ConnectionClosedError


def add_websocket_redis_bridge(
    app: FastAPI,
    *,
    topics: Iterable[str],
    redis_host: str = "localhost",
    redis_port: int = 6379,
    redis_db: int = 0
) -> FastAPI:
    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        print(f"[WebSocket::{os.getpid()}] Accepted: {websocket}")

        # Get Redis Pub/Sub
        pubsub = redis.Redis(host="localhost", port=6379, db=0).pubsub()
        for topic in topics:
            pubsub.subscribe(topic)

        while True:
            try:
                if message := pubsub.get_message(timeout=5):
                    message_type = message.get("type", None)
                    if message_type == "subscribe":
                        pass
                    elif message_type == "message":
                        message_data = message.get("data", None)
                        print(f"[Redis::{os.getpid()}] Message: {message_data}")
                        if message_data:
                            await websocket.send_text(message_data.decode("utf-8"))     # ConnectionClosedError
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
