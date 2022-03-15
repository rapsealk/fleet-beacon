import os

from fastapi import FastAPI

from src.fleet_beacon.database import Database
from src.fleet_beacon import config

base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def create_app() -> FastAPI:
    app = FastAPI(BASE_DIR=base_dir)

    database = Database(config.SQLALCHEMY_DATABASE_URI)
    database.init_database()

    return app


app = create_app()


if __name__ == "__main__":
    pass
