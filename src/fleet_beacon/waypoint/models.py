from typing import Optional

from sqlalchemy import Column, Integer, ForeignKey
from sqlalchemy.orm import relationship
from sqlalchemy.dialects.mysql import FLOAT

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin


class Waypoint(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    latitude = Column(FLOAT(precision=32), nullable=False)
    longitude = Column(FLOAT(precision=32), nullable=False)
    altitude = Column(FLOAT(precision=32), nullable=False)
    mission_id = Column(Integer, ForeignKey("mission.id", ondelete="CASCADE"))
    mission = relationship("Mission", back_populates="waypoints")


# Pydantic models...
class WaypointBase(FleetBeaconBase):
    latitude: float
    longitude: float
    altitude: float = 0


class WaypointCreate(WaypointBase):
    mission_id: Optional[PrimaryKey]


class WaypointRead(WaypointBase):
    id: PrimaryKey


class WaypointUpdate(WaypointBase):
    mission_id: Optional[PrimaryKey]
