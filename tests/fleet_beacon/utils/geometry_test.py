import os
import sys
import unittest

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))
from src.fleet_beacon.utils.geometry import LatLng, get_haversine_distance


class GeometryUtilsTestCase(unittest.TestCase):

    def test_haversien_metric(self):
        광화문 = LatLng(37.57802983406804, 126.97664492476694)
        국립중앙박물관 = LatLng(37.525830487890126, 126.98045394616595)
        서울역 = LatLng(37.55481936677628, 126.97072196848406)
        한국항공우주연구원 = LatLng(36.376324425460936, 127.35456248412363)

        distance = get_haversine_distance(광화문, 국립중앙박물관)
        self.assertTrue(abs(distance - 5.75) < 1e-1)    # 100m 단위 오차 허용

        distance = get_haversine_distance(서울역, 한국항공우주연구원)
        self.assertTrue(abs(distance - 135.47) < 1e-1)


if __name__ == "__main__":
    unittest.main()
