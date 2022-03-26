import math
from collections import namedtuple

LatLng = namedtuple("LatLng",
                    ("latitude", "longitude"))


def _haversine(theta: float) -> float:
    return math.sin(theta / 2.0)


def get_haversine_distance(src: LatLng, dst: LatLng, radius: float = 6371) -> float:
    # phi(φ): latitude, lambda(λ): longitude
    (rad_lat_src, rad_lng_src) = (math.radians(src.latitude), math.radians(src.longitude))
    (rad_lat_dst, rad_lng_dst) = (math.radians(dst.latitude), math.radians(dst.longitude))

    delta_lat = rad_lat_dst - rad_lat_src
    delta_lng = rad_lng_dst - rad_lng_src

    sin_delta_lat = math.pow(math.sin(delta_lat / 2.0), 2.0)
    sin_delta_lng = math.pow(math.sin(delta_lng / 2.0), 2.0)

    return 2 * radius * math.asin(math.sqrt(sin_delta_lat + math.cos(rad_lat_src) * math.cos(rad_lat_dst) * sin_delta_lng))
