from typing import cast

import pytest
from tesseract_robotics.planning import TaskComposer

from compas_fab.backends.tesseract.errors import InvalidTesseractRuntimeConfigurationError
from compas_fab.backends.tesseract.errors import TesseractPipelineWarmupError
from compas_fab.backends.tesseract.warmup import warmup_composer


class ConfiguredComposer:
    def __init__(self, config_path, loaded):
        self._config_path = config_path
        self.loaded = loaded
        self.requested = []

    def warmup(self, pipelines):
        self.requested = list(pipelines)
        return list(self.loaded)


def _config(tmp_path):
    path = tmp_path / "task_composer_plugins.yaml"
    path.write_text(
        """task_composer_plugins:
  tasks:
    plugins:
      PipelineB: {}
      PipelineA: {}
""",
        encoding="utf-8",
    )
    return path


def test_warmup_all_enumerates_every_configured_pipeline(tmp_path):
    composer = ConfiguredComposer(_config(tmp_path), ["PipelineA", "PipelineB"])

    loaded = warmup_composer(cast(TaskComposer, composer), True)

    assert composer.requested == ["PipelineA", "PipelineB"]
    assert loaded == ("PipelineA", "PipelineB")


def test_requested_warmup_pipeline_failure_is_loud(tmp_path):
    composer = ConfiguredComposer(_config(tmp_path), ["PipelineA"])

    with pytest.raises(TesseractPipelineWarmupError, match="PipelineB"):
        warmup_composer(cast(TaskComposer, composer), ["PipelineA", "PipelineB"])


def test_warmup_rejects_duplicate_pipeline_names(tmp_path):
    composer = ConfiguredComposer(_config(tmp_path), [])

    with pytest.raises(InvalidTesseractRuntimeConfigurationError, match="unique"):
        warmup_composer(cast(TaskComposer, composer), ["PipelineA", "PipelineA"])
