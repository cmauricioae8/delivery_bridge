# python imports
import logging
from typing import Union

# 3rd party imports
from fastapi import APIRouter
from fastapi.requests import Request

# local imports
from delivery_bridge.webapp.apps.base.serializers import ErrorResponse
from delivery_bridge.webapp.apps.users.cruds.user_cruds import (
    get_users,
    get_user,
    user_crud,
)
from delivery_bridge.webapp.apps.users.forms.user_forms import (
    UserCreateForm,
    UserUpdateForm,
)
from delivery_bridge.webapp.apps.users.serializers.user_serializers import (
    UserResponseSerializer,
    UserListResponseSerializer,
)


logger = logging.getLogger(__name__)


router = APIRouter()


@router.get("", response_model=UserListResponseSerializer)
def user_list(
    request: Request,
    username: str = None,
    is_active: bool = None,
    is_admin: bool = None,
    page: int = 1,
    page_limit: int = 10,
):
    """
    Lista de usuarios

    Lista de usuarios con filtros opcionales y paginación.
    """
    return UserListResponseSerializer(
        status="OK",
        message="",
        data=get_users(
            username=username,
            is_active=is_active,
            is_admin=is_admin,
            page=page,
            page_limit=page_limit,
        ),
    )


@router.get("/{user_id}", response_model=Union[UserResponseSerializer, ErrorResponse])
def user_detail(request: Request, user_id: int):
    """
    Detalle de usuario

    Detalle de usuario por ID.
    """
    user = user_crud.get(user_id)
    if user is None:
        return ErrorResponse(
            status="FAIL",
            message="Usuario no encontrado",
            error="Usuario no encontrado",
        )
    # return user detail
    response = UserResponseSerializer(
        status="OK",
        message="Usuario encontrado",
    )
    response.set_data(user)
    return response


@router.post("", response_model=Union[UserResponseSerializer, ErrorResponse])
def user_create(request: Request, form: UserCreateForm):
    """
    Nuevo usuario

    Crear un nuevo usuario.
    """
    # check if username already exists
    user = user_crud.get_by_field("username", form.username)
    if user is not None:
        return ErrorResponse(
            status="FAIL",
            message="El nombre de usuario ya existe",
            error="El nombre de usuario ya existe",
        )

    # check if password and confirm password match
    if form.password == "" or form.password != form.password_confirm:
        return ErrorResponse(
            status="FAIL",
            message="Las contraseñas no coinciden",
            error="Las contraseñas no coinciden",
        )
    # create user
    user = form.save()

    # return user detail
    response = UserResponseSerializer(
        status="OK",
        message="Usuario creado",
    )
    response.set_data(user)
    return response


@router.put("/{user_id}", response_model=Union[UserResponseSerializer, ErrorResponse])
def user_update(request: Request, user_id: int, form: UserUpdateForm):
    """
    Update user
    """
    user = get_user(user_id)
    if user is None:
        return ErrorResponse(
            status="FAIL",
            message="Usuario no encontrado",
            error="Usuario no encontrado",
        )

    # check if password and confirm password match
    if form.password is not None and form.password != form.password_confirm:
        return ErrorResponse(
            status="FAIL",
            message="Las contraseñas no coinciden",
            error="Las contraseñas no coinciden",
        )

    # update user
    user = form.update(user)

    # return user detail
    response = UserResponseSerializer(
        status="OK",
        message="Usuario actualizado",
    )
    response.set_data(user)
    return response


@router.delete(
    "/{user_id}", response_model=Union[UserResponseSerializer, ErrorResponse]
)
def user_delete(request: Request, user_id: int):
    """
    Delete user
    """
    user = user_crud.get(user_id)
    if user is None:
        return ErrorResponse(
            status="FAIL",
            message="Usuario no encontrado",
            error="Usuario no encontrado",
        )

    # delete user
    user_crud.delete(user_id)

    # return user detail
    response = UserResponseSerializer(
        status="OK",
        message="Usuario eliminado",
    )
    response.set_data(user)
    return response
