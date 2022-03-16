from sqlalchemy import Column, Float, Integer, ForeignKey

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin
from src.fleet_beacon.task.models import Task


class Waypoint(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    latitude = Column(Float)
    longitude = Column(Float)
    altitude = Column(Float)
    task = Column(Integer, ForeignKey(Task.id), nullable=False)


# Pydantic models...
class WaypointBase(FleetBeaconBase):
    id: PrimaryKey


class WaypointCreate(WaypointBase):
    latitude: float
    longitude: float
    altitude: float
    task: PrimaryKey


class WaypointUpdate(WaypointBase):
    latitude: float
    longitude: float
    altitude: float
    task: PrimaryKey


class WaypointDelete(WaypointBase):
    pass
