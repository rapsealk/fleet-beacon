from sqlalchemy import Column, Integer, ForeignKey

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin
from src.fleet_beacon.mission.models import Mission
from src.fleet_beacon.warehouse.models import Warehouse


class Fleet(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    warehouse = Column(Integer, ForeignKey(Warehouse.id, ondelete="CASCADE"), nullable=False)
    mission = Column(Integer, ForeignKey(Mission.id, ondelete="SET NULL"), nullable=True)


# Pydantic models...
class FleetBase(FleetBeaconBase):
    warehouse: PrimaryKey


class FleetCreate(FleetBase):
    pass


class FleetRead(FleetBase):
    id: PrimaryKey
    mission: PrimaryKey


class FleetUpdate(FleetBase):
    mission: PrimaryKey
