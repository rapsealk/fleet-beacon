from sqlalchemy import Column, Integer, ForeignKey
from sqlalchemy.dialects.mysql import FLOAT

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import TimeStampMixin
from src.fleet_beacon.mission.models import Mission


class Waypoint(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
    latitude = Column(FLOAT(precision=32), nullable=False)
    longitude = Column(FLOAT(precision=32), nullable=False)
    altitude = Column(FLOAT(precision=32), nullable=False)
    mission = Column(Integer, ForeignKey(Mission.id, ondelete="CASCADE"), nullable=False)
