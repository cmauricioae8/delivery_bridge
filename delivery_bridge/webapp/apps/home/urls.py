# python imports

# 3rd party imports
from fastapi import APIRouter

# local imports
from delivery_bridge.webapp.apps.home.routers import home_router
from delivery_bridge.webapp.apps.home.routers import login_router

router = APIRouter()

router.include_router(home_router.router, tags=["Inicio"], include_in_schema=False)
router.include_router(login_router.router, tags=["Inicio de sesión"], include_in_schema=False)
