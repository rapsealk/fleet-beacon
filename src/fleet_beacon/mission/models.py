from sqlalchemy import Column, Integer, String

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin
from src.fleet_beacon.mission.enums import MissionStatus


class Mission(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    status = Column(String(8), default=MissionStatus.assigned)


# Pydantic models...
class MissionBase(FleetBeaconBase):
    id: PrimaryKey


class MissionCreate(MissionBase):
    pass


class MissionUpdate(MissionBase):
    status: str


class MissionDelete(MissionBase):
    pass
