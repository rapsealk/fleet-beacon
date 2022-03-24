from typing import List

from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey
from src.fleet_beacon.waypoint.schemas import WaypointCreate


class MissionBase(FleetBeaconBase):
    id: PrimaryKey


class MissionCreate(MissionBase):
    waypoints: List[WaypointCreate]


class MissionUpdate(MissionBase):
    status: str


class MissionDelete(MissionBase):
    pass
