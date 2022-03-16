from sqlalchemy import Column, Integer

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin


class Fleet(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)


# Pydantic models...
class FleetBase(FleetBeaconBase):
    id: PrimaryKey


class RobotCreate(FleetBase):
    pass


class RobotUpdate(FleetBase):
    pass


class FleetDelete(FleetBase):
    pass
