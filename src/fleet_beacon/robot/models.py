from datetime import datetime
from typing import List, Optional

from sqlalchemy import Column, Boolean, Float, Integer, String, ForeignKey

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import FleetBeaconBase, PrimaryKey, TimeStampMixin
from src.fleet_beacon.robot.enums import PX4RobotMode
from src.fleet_beacon.fleet.models import Fleet
from src.fleet_beacon.warehouse.models import Warehouse


class Robot(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    uuid = Column(String(36), nullable=False, unique=True)  # uuid.uuid1(): Produced on separate machines
    connected = Column(Boolean, default=False)
    armed = Column(Boolean, default=False)
    guided = Column(Boolean, default=False)
    mode = Column(String(12), default=str(PX4RobotMode.offboard))
    system_status = Column(Integer)
    latitude = Column(Float)
    longitude = Column(Float)
    altitude = Column(Float)
    yaw = Column(Float)
    fleet = Column(Integer, ForeignKey(Fleet.id), nullable=True)
    warehouse = Column(Integer, ForeignKey(Warehouse.id), nullable=False)


# Pydantic models...
class RobotBase(FleetBeaconBase):
    uuid: str
    connected: bool = False
    armed: bool = False
    guided: bool = False
    mode: str = str(PX4RobotMode.offboard)
    system_status: int = 0
    latitude: float
    longitude: float
    altitude: float
    yaw: float
    fleet: Optional[int] = None
    warehouse: int


class RobotCreate(RobotBase):
    pass


class RobotRead(RobotBase):
    id: PrimaryKey
    created_at: datetime
    updated_at: datetime


class RobotList(FleetBeaconBase):
    total: int
    items: List[RobotRead] = []


class RobotUpdate(RobotBase):
    pass
