import json
import uuid
from typing import Any, Optional

import aioredis
from fastapi.requests import Request
from starlette.middleware.base import BaseHTTPMiddleware, RequestResponseEndpoint
from starlette.responses import Response


class SessionMiddleware(BaseHTTPMiddleware):
    def __init__(self, *args, **kwargs):
        super(SessionMiddleware, self).__init__(*args, **kwargs)
        self._in_memory_storage = RedisSessionStorage()

    async def dispatch(self, request: Request, call_next: RequestResponseEndpoint) -> Response:
        session_id = request.cookies.get("session") or str(uuid.uuid4())
        request.state.session = await self._in_memory_storage.get(session_id) or {}
        response = await call_next(request)
        await self._in_memory_storage.put(session_id, request.state.session)
        response.set_cookie(key="session", value=session_id)
        return response


class BaseSessionStorage:
    def __init__(self, *args, **kwargs):
        pass

    def get(self, key: str) -> Any:
        raise NotImplementedError()

    def put(self, key: str, value: Any):
        raise NotImplementedError()


class InMemorySessionStorage(BaseSessionStorage):
    def __init__(self, *args, **kwargs):
        super(InMemorySessionStorage, self).__init__(*args, **kwargs)
        self._storage = {}

    async def get(self, key: str) -> Optional[Any]:
        return self._storage.get(key)

    async def put(self, key: str, value: Any):
        self._storage[key] = value


class RedisSessionStorage(BaseSessionStorage):
    def __init__(self, *args, **kwargs):
        super(RedisSessionStorage, self).__init__(*args, **kwargs)
        self._storage = aioredis.from_url("redis://localhost")

    async def get(self, key: str) -> Optional[Any]:
        if value := await self._storage.get(key):
            return json.loads(value.decode("utf-8"))
        return None

    async def put(self, key: str, value: Any):
        value = json.dumps(value)
        await self._storage.set(key, value.encode("utf-8"))
