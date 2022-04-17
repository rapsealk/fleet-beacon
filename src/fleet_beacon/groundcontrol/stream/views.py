import struct
import zlib

import cv2
import numpy as np
from fastapi import APIRouter
from fastapi.responses import StreamingResponse

from fleet_beacon.common.subscriber import RedisSubscriber

METADATA_BYTES = 4

router = APIRouter()


@router.get("/{channel}")
async def _(channel: str):
    async def bypass():
        subscriber = RedisSubscriber(channels=[f"stream/{channel}"])
        await subscriber.initialize()
        async for message in subscriber.subscribe_sync():
            buffer = zlib.decompress(message)
            shape = []
            for _ in range(3):
                shape.append(struct.unpack(">I", buffer[:METADATA_BYTES])[0])
                buffer = buffer[METADATA_BYTES:]
            buffer = np.frombuffer(buffer, dtype=np.uint8).reshape(shape)
            ret, buffer = cv2.imencode(".jpg", buffer)
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + buffer.tobytes()
                + b"\r\n"
            )

    return StreamingResponse(bypass(), media_type="multipart/x-mixed-replace; boundary=frame")
