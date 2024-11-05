# python imports

# 3rd party imports
from fastapi import APIRouter

# local imports
from delivery_bridge.webapp.apps.ros2_app.routers import (
    battery_routers,
    set_pose_routers,
    # operation_mode_routers,
)

router = APIRouter()

router.include_router(battery_routers.router)
router.include_router(set_pose_routers.router)
# router.include_router(operation_mode_routers.router)