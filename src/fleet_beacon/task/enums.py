from src.fleet_beacon.enums import FleetBeaconEnum


class TaskTypes(FleetBeaconEnum):
    move = "Move"
    takeoff = "Takeoff"
    land = "Land"


class TaskStatus(FleetBeaconEnum):
    assigned = "Assigned"
    executed = "Executed"
    accomplished = "Accomplished"
