from src.fleet_beacon.enums import FleetBeaconEnum


class MissionStatus(FleetBeaconEnum):
    allocated = "Allocated"
    executed = "Executed"
    closed = "Closed"
