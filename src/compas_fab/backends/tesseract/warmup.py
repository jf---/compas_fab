"""Fail-loud Task Composer pipeline warmup."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Union

import yaml
from tesseract_robotics.planning import TaskComposer

from .errors import InvalidTesseractRuntimeConfigurationError
from .errors import TesseractPipelineWarmupError

WarmupSelection = Union[bool, list[str]]


def warmup_composer(
    composer: TaskComposer,
    selection: WarmupSelection,
) -> tuple[str, ...]:
    """Load exactly the selected pipelines and reject every missing plugin."""
    requested = _requested_pipelines(composer, selection)
    if not requested:
        return ()
    loaded = tuple(composer.warmup(list(requested)))
    missing = [name for name in requested if name not in loaded]
    if missing:
        raise TesseractPipelineWarmupError("Task Composer failed to warm requested pipelines: {}.".format(", ".join(missing)))
    return loaded


def _requested_pipelines(
    composer: TaskComposer,
    selection: WarmupSelection,
) -> tuple[str, ...]:
    if isinstance(selection, bool):
        return _configured_pipelines(composer) if selection else ()
    if not isinstance(selection, list):
        raise InvalidTesseractRuntimeConfigurationError("warmup must be bool or list[str], got {}.".format(type(selection).__name__))
    if any(not isinstance(name, str) or not name.strip() for name in selection):
        raise InvalidTesseractRuntimeConfigurationError("warmup pipeline names must be non-empty strings.")
    requested = tuple(name.strip() for name in selection)
    if len(requested) != len(set(requested)):
        raise InvalidTesseractRuntimeConfigurationError("warmup pipeline names must be unique.")
    return requested


def _configured_pipelines(composer: TaskComposer) -> tuple[str, ...]:
    config_path = getattr(composer, "_config_path", None)
    if not isinstance(config_path, Path):
        raise InvalidTesseractRuntimeConfigurationError("warmup_all requires a TaskComposer retaining its exact config path.")
    try:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        plugins = config["task_composer_plugins"]["tasks"]["plugins"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise InvalidTesseractRuntimeConfigurationError("Task Composer config {!s} cannot enumerate warmup pipelines: {}.".format(config_path, error)) from error
    if not isinstance(plugins, Mapping):
        raise InvalidTesseractRuntimeConfigurationError("Task Composer config {!s} has no task plugin mapping.".format(config_path))
    names = tuple(sorted(plugins))
    if not names or any(not isinstance(name, str) or not name for name in names):
        raise InvalidTesseractRuntimeConfigurationError("Task Composer config {!s} has no valid pipeline names.".format(config_path))
    return names
