# python imports

# 3rd party imports
from fastapi import APIRouter

# local imports
from delivery_bridge.webapp.apps.users.routers import user_routers


router = APIRouter()

router.include_router(user_routers.router, prefix="/user")
