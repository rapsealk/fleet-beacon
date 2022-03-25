
from typing import List

from sqlalchemy import Column, Integer, String
from sqlalchemy.orm import relationship

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin
from src.fleet_beacon.mission.enums import MissionStatus
from src.fleet_beacon.waypoint.models import WaypointCreate, WaypointRead


class Mission(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    status = Column(String(8), default=MissionStatus.assigned)
    waypoints = relationship("Waypoint")


# Pydantic models...
class MissionBase(FleetBeaconBase):
    pass


class MissionCreate(MissionBase):
    waypoints: List[WaypointCreate]


class MissionRead(MissionBase):
    id: PrimaryKey
    waypoints: List[WaypointRead]


class MissionList(FleetBeaconBase):
    total: int
    items: List[MissionRead] = []


class MissionUpdate(MissionBase):
    status: str


class MissionDelete(MissionBase):
    pass
