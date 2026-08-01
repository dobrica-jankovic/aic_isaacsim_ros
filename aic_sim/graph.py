"""Thin OmniGraph authoring helpers shared by the bridge builders.

The one thing worth knowing here is :meth:`GraphBuilder.set_input`: it writes
both the runtime (Fabric) value *and* the USD attribute. ``og.Controller.set``
only does the former, which leaves the stage with no opinion, so a
saved-and-reopened graph silently reverts every input to its OGN default --
``topicName`` becomes ``/rgb``, script paths become empty. The bridge then comes
up *wrong* rather than failing, which is much harder to notice. Go through this
class and that cannot happen.
"""

from __future__ import annotations

from typing import Any, Iterable

import omni.graph.core as og
from pxr import Sdf, Usd


_OUTPUT_PORT = og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT


class GraphBuilder:
    """Builds one action graph, recreating it from scratch if it already exists."""

    def __init__(self, stage: Usd.Stage, graph_path: str) -> None:
        self.stage = stage
        self.graph_path = graph_path
        if stage.GetPrimAtPath(graph_path):
            stage.RemovePrim(graph_path)
        self.graph = og.Controller.create_graph(
            {"graph_path": graph_path, "evaluator_name": "execution"}
        )

    def node(self, name: str, node_type: str):
        """Create a node and return its handle."""

        og.Controller.edit(
            self.graph, {og.Controller.Keys.CREATE_NODES: [(name, node_type)]}
        )
        return og.Controller.node(name, self.graph)

    def attribute(self, name: str, attribute: str):
        return og.Controller.attribute(attribute, og.Controller.node(name, self.graph))

    def connect(self, src: str, src_attr: str, dst: str, dst_attr: str) -> None:
        og.Controller.connect(
            self.attribute(src, src_attr), self.attribute(dst, dst_attr)
        )

    def set_input(self, name: str, attribute: str, value: Any) -> None:
        """Set an input at runtime *and* author it into USD. See module docs."""

        og.Controller.set(self.attribute(name, attribute), value)
        usd_attribute = self.prim(name).GetAttribute(attribute)
        if not usd_attribute:
            raise RuntimeError(f"No USD attribute '{attribute}' on node '{name}'.")
        usd_attribute.Set(value)

    def set_relationship(self, name: str, relationship: str, targets: Iterable[str]):
        """Set a relationship. Relationships are plain USD, so this is enough."""

        self.prim(name).CreateRelationship(relationship).SetTargets(
            [Sdf.Path(target) for target in targets]
        )

    def add_outputs(self, name: str, outputs: Iterable[tuple[str, str]]) -> None:
        """Add dynamic output attributes, e.g. to a ScriptNode."""

        node = og.Controller.node(name, self.graph)
        for attribute, type_name in outputs:
            og.Controller.create_attribute(node, attribute, type_name, _OUTPUT_PORT)

    def prim(self, name: str) -> Usd.Prim:
        return self.stage.GetPrimAtPath(f"{self.graph_path}/{name}")

    def compute_errors(self) -> list[str]:
        """Per-node compute errors, for a build report."""

        errors = []
        for node in self.graph.get_nodes():
            messages = list(node.get_compute_messages(og.Severity.ERROR))
            if messages:
                name = node.get_prim_path().split("/")[-1]
                errors.append(f"{name}: {messages[0][:160]}")
        return errors

    def node_count(self) -> int:
        return len(list(self.graph.get_nodes()))


__all__ = ["GraphBuilder"]
