import os

from starlette.config import Config

config = Config(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), '.env'))

SQLALCHEMY_DATABASE_URI = config('SQLALCHEMY_DATABASE_URI',
                                 default='mysql+pymysql://root:0000@localhost:3306/fleet_beacon')

KAKAO_MAP_APP_KEY = config('KAKAO_MAP_APP_KEY')
