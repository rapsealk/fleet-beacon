import uuid

from fastapi.requests import Request
from starlette.middleware.base import BaseHTTPMiddleware, RequestResponseEndpoint
from starlette.responses import Response


class SessionMiddleware(BaseHTTPMiddleware):
    def __init__(self, *args, **kwargs):
        super(SessionMiddleware, self).__init__(*args, **kwargs)
        self._in_memory_storage = {}

    async def dispatch(self, request: Request, call_next: RequestResponseEndpoint) -> Response:
        session_id = request.cookies.get("session") or str(uuid.uuid4())
        request.state.session = self._in_memory_storage.get(session_id, {})
        response = await call_next(request)
        self._in_memory_storage[session_id] = request.state.session
        response.set_cookie(key="session", value=session_id)
        return response
