import json
import os
import uuid


if __name__ == "__main__":
    config = {
        "base_url": "http://host.docker.internal:8000",
        "uuid": str(uuid.uuid4()),
        "warehouse": 1,
        "global_position": {
            "latitude": 0,
            "longitude": 0,
            "altitude": 0
        }
    }
    with open(os.path.join(os.path.dirname(__file__), config["uuid"] + ".json"), "w") as f:
        json.dump(config, f)
