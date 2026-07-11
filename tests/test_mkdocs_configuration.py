from pathlib import Path

from mkdocs.config import load_config

MKDOCS_CONFIG = Path(__file__).parents[1] / "mkdocs.yml"


def test_print_site_runs_after_dynamically_registered_plugins() -> None:
    config = load_config(str(MKDOCS_CONFIG))

    config.plugins.run_event("config", config)

    plugin_names = tuple(config.plugins)
    assert plugin_names[-2:] == ("autorefs", "print-site")
    assert len(plugin_names) == len(set(plugin_names))
