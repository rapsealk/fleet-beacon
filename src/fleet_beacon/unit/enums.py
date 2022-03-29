from fleet_beacon.enums import FleetBeaconEnum


class PX4Mode(FleetBeaconEnum):
    manual = "MANUAL"
    acro = "ACRO"
    altitude = "ALTCTL"
    position = "POSCTL"
    offboard = "OFFBOARD"
    stabilized = "STABILIZED"
    rattitude = "RATTITUDE"
    mission = "AUTO.MISSION"
    loiter = "AUTO.LOITER"
    rtl = "AUTO.RTL"
    land = "AUTO.LAND"
    rtgs = "AUTO.RTGS"
    ready = "AUTO.READY"
    takeoff = "AUTO.TAKEOFF"
