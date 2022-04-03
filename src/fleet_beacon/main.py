import os
import uuid
from typing import Callable, Optional, Final
from contextvars import ContextVar

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from sqlalchemy.orm import sessionmaker, scoped_session
from starlette.requests import Request

from fleet_beacon import config
from fleet_beacon.api import router as api_router
from fleet_beacon.view import router as view_router
from fleet_beacon.database import Database
from fleet_beacon.websocket import add_websocket_redis_bridge
from fleet_beacon.middlewares import SessionMiddleware

REQUEST_ID_CTX_KEY: Final[str] = "request_id"
_request_id_ctx_var: ContextVar[Optional[str]] = ContextVar(REQUEST_ID_CTX_KEY, default=None)

base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def get_request_id() -> Optional[str]:
    return _request_id_ctx_var.get()


def create_app() -> FastAPI:
    app = FastAPI(BASE_DIR=base_dir)

    database = Database(config.SQLALCHEMY_DATABASE_URI)
    database.init_database()

    @app.middleware("http")
    async def db_session_middleware(request: Request, call_next: Callable):
        request_id = str(uuid.uuid1())

        # we create a per-request id such that we can ensure that our session is scoped for a particular request.
        # see: https://github.com/tiangolo/fastapi/issues/726
        ctx_token = _request_id_ctx_var.set(request_id)

        try:
            session = scoped_session(sessionmaker(bind=database.engine), scopefunc=get_request_id)
            request.state.db = session()
            response = await call_next(request)
        except Exception as e:
            raise e from None
        finally:
            request.state.db.close()

        _request_id_ctx_var.reset(ctx_token)

        return response

    app.include_router(api_router)
    app.include_router(view_router)

    # Middlewares
    app.add_middleware(SessionMiddleware)

    app.mount("/static", StaticFiles(directory="static"), name="static")

    app = add_websocket_redis_bridge(app, topics=["global_position"])

    return app


app = create_app()


if __name__ == "__main__":
    pass
