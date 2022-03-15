from pydantic import Field
from sqlalchemy import Column, Float, Integer, String, ForeignKey

from src.fleet_beacon.database import Base
from src.fleet_beacon.models import TimeStampMixin


class Fleet(Base, TimeStampMixin):
    id = Column(Integer, primary_key=True)
