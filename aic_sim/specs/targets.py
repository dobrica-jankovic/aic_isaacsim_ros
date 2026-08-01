"""Target and port asset contracts used by insertion tasks."""

from __future__ import annotations

from dataclasses import dataclass

from .base import AssetIdentity, AssetSpec, UsdAssetInterface, Vector3, asset_path


@dataclass(frozen=True)
class TargetPortSpec:
    """A port/frame contract inside a target USD asset."""

    name: str
    index: int
    link_path: str
    seat_frame_path: str
    entrance_frame_path: str | None
    insertion_axis_local: Vector3


@dataclass(frozen=True)
class TargetAssetSpec(AssetSpec):
    """Reusable target USD asset with optional insertion ports."""

    ports: tuple[TargetPortSpec, ...] = ()
    default_port: str | None = None

    def port(self, name: str) -> TargetPortSpec:
        """Return a named port contract."""

        for port in self.ports:
            if port.name == name:
                return port
        raise KeyError(f"Unknown port '{name}' for target asset '{self.name}'.")

    def default_port_spec(self) -> TargetPortSpec:
        """Return the default port contract."""

        if self.default_port is None:
            raise KeyError(f"Target asset '{self.name}' does not define a default port.")
        return self.port(self.default_port)


NIC_SFP_PORT_0 = TargetPortSpec(
    name="sfp_port_0",
    index=0,
    link_path="/sfp_port_0_link",
    seat_frame_path="/sfp_port_0_link",
    entrance_frame_path="/sfp_port_0_link/sfp_port_0_link_entrance",
    insertion_axis_local=(0.0, 1.0, 0.0),
)

NIC_CARD_ASSET = TargetAssetSpec(
    identity=AssetIdentity(name="nic_card", role="target"),
    usd=UsdAssetInterface(kind="rigid_object", root_prim="nic_card_link"),
    usd_path=asset_path("targets", "nic_card", "nic_card.usd"),
    ports=(NIC_SFP_PORT_0,),
    default_port=NIC_SFP_PORT_0.name,
)

SC_PORT_ASSET = TargetAssetSpec(
    identity=AssetIdentity(name="sc_port", role="target"),
    usd=UsdAssetInterface(kind="rigid_object", root_prim="sc_port_visual"),
    usd_path=asset_path("targets", "sc_port", "sc_port.usd"),
)
