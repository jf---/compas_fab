from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import logging
import os
import re
import sys
import traceback
from contextlib import contextmanager
from pathlib import Path

from tesseract.tesseract_common import FilesystemPath
from tesseract.tesseract_environment import Environment
from tesseract.tesseract_scene_graph import SimpleResourceLocatorFn, SimpleResourceLocator


# __all__ = [
#     'LOG',
#     'redirect_stdout',
# ]


@contextmanager
def redirect_stdout(to=os.devnull, enabled=True):
    """Context manager to capture and redirect console output.
    https://stackoverflow.com/a/17954769
    Parameters
    ----------
    to : Location to redirect output to.  Defaults to ``os.devnull``.
    enabled : (:obj:`bool`) Flag to enable or disable redirection.

    Examples
    --------
    >>> import os                                                        # doctest: +SKIP
    >>> with redirect_stdout(to='filename'):                             # doctest: +SKIP
    ...     print("from Python")                                         # doctest: +SKIP
    ...     os.system("echo non-Python applications are also supported") # doctest: +SKIP
    """

    def _redirect_stdout(to_):
        sys.stdout.close()  # + implicit flush()
        os.dup2(to_.fileno(), fd)
        sys.stdout = os.fdopen(fd, 'w')

    # Pytest interferes with file descriptor capture.
    called_from_test = 'pytest' in sys.modules
    enabled = False if called_from_test else enabled

    if not enabled:
        yield
    else:
        fd = sys.stdout.fileno()
        with os.fdopen(os.dup(fd), 'w') as old_stdout:
            with open(to, 'w') as file:
                _redirect_stdout(to_=file)
            try:
                yield
            finally:
                _redirect_stdout(to_=old_stdout)  # restore stdout. buffering and flags such as CLOEXEC may be different


def get_logger(name):
    logger = logging.getLogger(name)

    try:
        from colorlog import ColoredFormatter
        formatter = ColoredFormatter("%(log_color)s%(levelname)-8s%(reset)s %(white)s%(message)s",
                                     datefmt=None,
                                     reset=True,
                                     log_colors={'DEBUG': 'cyan', 'INFO': 'green',
                                                 'WARNING': 'yellow',
                                                 'ERROR': 'red', 'CRITICAL': 'red',
                                                 }
                                     )
    except ImportError:
        formatter = logging.Formatter('[%(levelname)s] %(message)s')

    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

    return logger


def path_to_filesystempath(pth: Path) -> FilesystemPath:
    return FilesystemPath(str(pth))


def load_env(path_urdf: Path, path_srdf: Path) -> Environment:
    locate_resource_fn = SimpleResourceLocatorFn(_locate_resource)
    locator = SimpleResourceLocator(locate_resource_fn)
    env = Environment()
    assert path_urdf.is_file(), f"{path_urdf} invalid path"

    assert path_srdf.is_file(), f"{path_srdf} invalid path"

    assert env.init(path_to_filesystempath(path_urdf), path_to_filesystempath(path_srdf), locator)
    return env


LOG = get_logger(__name__)


def _locate_resource(url):
    try:
        url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$", url)
        if (url_match is None):
            return ""
        if not "TESSERACT_SUPPORT_DIR" in os.environ:
            return ""
        tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
        return os.path.join(tesseract_support, os.path.normpath(url_match.group(1)))
    except:
        traceback.print_exc()
