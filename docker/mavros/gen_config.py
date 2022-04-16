import argparse
import json
import os
import uuid

from sqlalchemy.orm import sessionmaker, scoped_session

from fleet_beacon import config as _config
from fleet_beacon.database import Database
from fleet_beacon.warehouse.models import Warehouse


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--warehouse", type=int)
    return parser.parse_args()


def main():
    args = parse_args()

    database = Database(_config.SQLALCHEMY_DATABASE_URI)
    database.init_database()
    session = scoped_session(sessionmaker(bind=database.engine))

    with session() as sess:
        if not (warehouse := sess.query(Warehouse).filter(Warehouse.id == args.warehouse).first()):
            raise Exception()

    config = {
        "uuid": str(uuid.uuid4()),
        "warehouse": args.warehouse,
        "global_position": {
            "latitude": warehouse.latitude,
            "longitude": warehouse.longitude,
            "altitude": 0
        }
    }
    with open(os.path.join(os.path.dirname(__file__), config["uuid"] + ".json"), "w") as f:
        json.dump(config, f)


if __name__ == "__main__":
    main()
