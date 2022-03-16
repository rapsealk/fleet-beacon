from enum import Enum


class FleetBeaconEnum(str, Enum):
    def __str__(self) -> str:
        return str.__str__(self)
