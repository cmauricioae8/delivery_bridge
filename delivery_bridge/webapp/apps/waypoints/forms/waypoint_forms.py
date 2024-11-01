from pydantic import BaseModel, Field


class WaypointCreateForm(BaseModel):
    name: str = Field(
        description="Nombre del waypoint",
        max_length=50,
        unique=True,
        examples=["waypoint 1", "waypoint 2"],
    )
    # type: int = Field(
    #     description="Tipo de waypoint (fijo, auxiliar)",
    #     examples=[1, 2],
    # )
    is_mandatory: bool | None = Field(
        default=True,
        description="Waypoint obligatorio",
        examples=[True, False],
    )
    description: str | None = Field(
        default="",
        description="Descripción del waypoint",
        examples=["waypoint1", "waypoint2"],
    )
    # map: int = Field(
    #     description="Mapa al que pertenece el waypoint",
    #     examples=[1, 2],
    # )
    # icon: str | None = Field(
    #     default="default",
    #     description="Icono del waypoint",
    #     examples=["icon1", "icon2"],
    # )
    # color: str | None = Field(
    #     default="#000000",
    #     description="Color del waypoint",
    #     examples=["#000000", "#ffffff"],
    # )


class WaypointUpdateForm(BaseModel):
    name: str | None = Field(
        default=None,
        description="Nombre del waypoint",
        max_length=50,
        unique=True,
        examples=["waypoint 1", "waypoint 2"],
    )
    # type: int | None = Field(
    #     default=None,
    #     description="Tipo de waypoint",
    #     examples=[1, 2],
    # )
    is_mandatory: bool | None = Field(
        default=True,
        description="Waypoint obligatorio",
        examples=[True, False],
    )
    description: str | None = Field(
        default=None,
        description="Descripción del waypoint",
        examples=["waypoint1", "waypoint2"],
    )
    # map: int | None = Field(
    #     default=None,
    #     description="Mapa al que pertenece el waypoint",
    #     examples=[1, 2],
    # )
    position_x: float | None = Field(
        default=None,
        description="Posición en el eje X (en metros)",
        examples=[0.1, -3.78],
    )
    position_y: float | None = Field(
        default=None,
        description="Posición en el eje Y (en metros)",
        examples=[0.1, -3.78],
    )
    orientation_z: float | None = Field(
        gt=-3.1416,
        lt=3.1416,
        default=None,
        description="Orientación con respecto al eje X (en radianes)",
        examples=[0.1, -3.78],
    )
    # icon: str | None = Field(
    #     default=None,
    #     description="Icono del waypoint",
    #     examples=["icon1", "icon2"],
    # )
    # color: str | None = Field(
    #     default=None,
    #     description="Color del waypoint",
    #     examples=["#000000", "#ffffff"],
    # )