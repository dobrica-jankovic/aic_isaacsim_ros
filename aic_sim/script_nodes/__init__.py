"""Bodies for the OmniGraph ScriptNodes in the ROS 2 bridge.

Each module here is a standalone Python file defining ``compute(db)``, read as
*text* and inlined into the node's ``inputs:script``. Inlining (rather than
pointing ``inputs:scriptPath`` at these files) is what lets the saved stage open
on a machine that does not have this repo checked out at the same path.

Every robot- or scene-specific value arrives through ``CONFIG``, which
:func:`script_body` substitutes at build time. The bodies themselves name no
joints, prims or frames, so a second industrial robot is a spec change.

**Everything a body needs lives inside ``compute``.** ``OgnScriptNode`` execs
each script and then merges the names it defined into ``compute.__globals__``,
which is the *``OgnScriptNode`` module's* globals -- one dict shared by every
ScriptNode in the process. Two bodies that both define ``CONFIG`` or ``_init``
at module level silently overwrite each other, whichever initialised last, and
the symptom is a nonsense ``KeyError`` in an unrelated node. Locals and nested
functions are not captured, so they cannot collide.
"""

from __future__ import annotations

from pathlib import Path


_BODY_DIR = Path(__file__).resolve().parent
_CONFIG_PLACEHOLDER = "CONFIG = {}"


def allow_script_execution() -> None:
    """Let ScriptNodes run without the opt-in prompt.

    Executing a ScriptNode is opt-in, and the prompt is a UI dialog that never
    resolves headless -- the nodes then sit there computing nothing. Call this
    before *any* graph containing ScriptNodes runs, which means when opening a
    saved stage as well as when building one.
    """

    import carb.settings

    carb.settings.get_settings().set("/app/omni.graph.scriptnode/enable_opt_in", False)


def script_body(name: str, **config: object) -> str:
    """Return the body of ``<name>.py`` with its ``CONFIG`` filled in."""

    source = (_BODY_DIR / f"{name}.py").read_text()
    if _CONFIG_PLACEHOLDER not in source:
        raise ValueError(f"Script body '{name}' has no '{_CONFIG_PLACEHOLDER}' line.")
    return source.replace(_CONFIG_PLACEHOLDER, f"CONFIG = {config!r}", 1)


__all__ = ["allow_script_execution", "script_body"]
