from typing import get_args
from opendbc.car.mock.values import CAR as MOCK
from opendbc.car.toyota.values import CAR as TOYOTA

from opendbc.car.tesla.values import CAR as TESLA

Platform = MOCK | TOYOTA | TESLA
BRANDS = get_args(Platform)

PLATFORMS: dict[str, Platform] = {str(platform): platform for brand in BRANDS for platform in brand}
