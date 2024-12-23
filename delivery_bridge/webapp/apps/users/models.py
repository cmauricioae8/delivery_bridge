# python imports

# 3rd party imports
from sqlmodel import Field
from passlib.context import CryptContext

from delivery_bridge.webapp.apps.base.models import BaseModel

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


class User(BaseModel, table=True):
    """Model definition for User"""

    username: str = Field(max_length=100)
    password: str = Field(max_length=100)
    is_active: bool = Field(title="Activo?", default=True)
    is_admin: bool = Field(title="Administrador?", default=False)

    def __str__(self):
        """Unicode representation of User."""
        return self.username

    def set_password(self, plain_password: str) -> None:
        """Set the password."""
        self.password = pwd_context.hash(plain_password)

    def verify_password(self, plain_password: str) -> bool:
        """Verify the password."""
        return pwd_context.verify(plain_password, self.password)
