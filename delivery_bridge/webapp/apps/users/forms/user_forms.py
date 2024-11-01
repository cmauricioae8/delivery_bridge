from datetime import datetime

from pydantic import BaseModel, Field

from delivery_bridge.webapp.apps.users.models import User


class UserCreateForm(BaseModel):
    username: str = Field(
        title="Nombre de usuario",
        description="Nombre de usuario",
        max_length=100,
        examples=["admin"],
    )
    password: str = Field(
        title="Contraseña",
        description="Contraseña",
        max_length=100,
        examples=["admin"],
    )
    password_confirm: str = Field(
        title="Confirmar contraseña",
        description="Confirmar contraseña",
        max_length=100,
        examples=["admin"],
    )
    is_active: bool = Field(
        title="Activo?",
        description="Activo?",
        default=True,
        examples=[True],
    )
    is_admin: bool = Field(
        title="Administrador?",
        description="Administrador?",
        default=False,
        examples=[False],
    )

    def save(self) -> User:
        user = User(
            username=self.username, is_admin=self.is_admin
        )
        user.set_password(self.password)
        user.created_at = datetime.now()
        user.save()
        return user


class UserUpdateForm(BaseModel):
    username: str | None = Field(
        title="Nombre de usuario",
        description="Nombre de usuario",
        max_length=100,
        examples=["admin"],
        default=None,
    )
    password: str | None = Field(
        title="Contraseña",
        description="Contraseña",
        max_length=100,
        examples=["admin"],
        default=None,
    )
    password_confirm: str | None = Field(
        title="Confirmar contraseña",
        description="Confirmar contraseña",
        max_length=100,
        examples=["admin"],
        default=None,
    )
    is_active: bool | None = Field(
        title="Activo?",
        description="Activo?",
        examples=[True],
        default=None,
    )
    is_admin: bool | None = Field(
        title="Administrador?",
        description="Administrador?",
        examples=[False],
        default=None,
    )

    def update(self, user: User) -> User:
        if self.username is not None:
            user.username = self.username
        if self.password is not None:
            user.set_password(self.password)
        if self.is_active is not None:
            user.is_active = self.is_active
        if self.is_admin is not None:
            user.is_admin = self.is_admin
        user.updated_at = datetime.now()
        user.save()
        return user
