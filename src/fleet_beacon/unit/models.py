from datetime import datetime
from typing import List, Optional

from sqlalchemy import Column, Boolean, DateTime, Float, Integer, String, ForeignKey
from sqlalchemy.orm import relationship

from fleet_beacon.database import Base
from fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin
from fleet_beacon.unit.enums import PX4Mode
from fleet_beacon.warehouse.models import WarehouseRead


class Unit(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    uuid = Column(String(36), nullable=False, unique=True)  # uuid.uuid1(): Produced on separate machines
    connected = Column(Boolean, default=False)
    armed = Column(Boolean, default=False)
    guided = Column(Boolean, default=False)
    mode = Column(String(12), default=str(PX4Mode.offboard))
    system_status = Column(Integer, default=0)
    latitude = Column(Float, default=0.0)
    longitude = Column(Float, default=0.0)
    altitude = Column(Float, default=0.0)
    yaw = Column(Float, default=0.0)
    heartbeat = Column(DateTime, nullable=True)
    fleet_id = Column(Integer, ForeignKey("fleet.id"), nullable=True)
    warehouse_id = Column(Integer, ForeignKey("warehouse.id"), nullable=False)
    fleet = relationship("Fleet", back_populates="units")
    warehouse = relationship("Warehouse", back_populates="units")


# Pydantic models...
class UnitBase(FleetBeaconBase):
    pass


class UnitCreate(UnitBase):
    uuid: str
    warehouse_id: int


class UnitRead(UnitBase):
    id: PrimaryKey
    uuid: str
    connected: bool = False
    armed: bool = False
    guided: bool = False
    mode: str = str(PX4Mode.offboard)
    system_status: int = 0
    latitude: float
    longitude: float
    altitude: float
    yaw: float
    heartbeat: Optional[datetime] = None
    fleet_id: Optional[int] = None
    warehouse_id: int
    created_at: datetime
    updated_at: datetime


class UnitList(FleetBeaconBase):
    total: int
    items: List[UnitRead] = []


class UnitWarehouseDetail(WarehouseRead):
    units: UnitList


class UnitUpdate(UnitBase):
    connected: bool = False
    armed: bool = False
    guided: bool = False
    mode: str = str(PX4Mode.offboard)
    system_status: int = 0
    latitude: float = 0
    longitude: float = 0
    altitude: float = 0
    yaw: float = 0
    heartbeat: Optional[datetime] = None
    fleet_id: Optional[int] = None
    warehouse_id: Optional[int] = None
