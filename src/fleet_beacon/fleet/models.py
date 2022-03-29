from typing import List, Optional

from sqlalchemy import Column, Integer, ForeignKey
from sqlalchemy.orm import relationship

from fleet_beacon.database import Base
from fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin
from fleet_beacon.unit.models import UnitRead


class Fleet(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    warehouse_id = Column(Integer, ForeignKey("warehouse.id", ondelete="CASCADE"), nullable=False)
    mission_id = Column(Integer, ForeignKey("mission.id", ondelete="SET NULL"), nullable=True)
    units = relationship("Unit")


# Pydantic models...
class FleetBase(FleetBeaconBase):
    pass


class FleetCreate(FleetBase):
    warehouse_id: PrimaryKey
    unit_ids: List[int]


class FleetRead(FleetBase):
    id: PrimaryKey
    warehouse_id: PrimaryKey
    mission_id: Optional[PrimaryKey] = None
    units: List[UnitRead] = []


class FleetList(FleetBase):
    total: int = 0
    items: List[FleetRead] = []


class FleetUpdate(FleetBase):
    warehouse_id: Optional[PrimaryKey] = None
    mission_id: Optional[PrimaryKey] = None
