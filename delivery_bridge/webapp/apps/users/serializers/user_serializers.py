# python imports
from datetime import datetime

# 3rd party imports
from pydantic import BaseModel

# local imports
from delivery_bridge.webapp.apps.base.serializers import DataResponse, DataWithPagination
from delivery_bridge.webapp.apps.users.models import User


class UserSerializer(BaseModel):
    id: int
    username: str
    # is_active: bool
    is_admin: bool
    created_at: datetime
    updated_at: datetime
    # created_by: int
    # updated_by: int


class UserResponseSerializer(DataResponse):
    data: UserSerializer = None

    def set_data(self, user: User):
        self.data = UserSerializer(
            id=user.id,
            username=user.username,
            # is_active=user.is_active,
            is_admin=user.is_admin,
            created_at=user.created_at,
            updated_at=user.updated_at,
            # created_by=user.created_by,
            # updated_by=user.updated_by,
        )


class UserListSerializer(DataWithPagination):
    records: list[UserSerializer]


class UserListResponseSerializer(DataResponse):
    data: UserListSerializer
