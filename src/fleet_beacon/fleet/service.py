from typing import Optional

from src.fleet_beacon.fleet.models import Fleet


def get(*, db_session, fleet_id: int) -> Optional[Fleet]:
    pass
