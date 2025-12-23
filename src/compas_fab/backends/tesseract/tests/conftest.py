"""
pytest configuration for compas_fab_tesseract tests.
"""

import pytest
import os

# Check if tesseract is available
TESSERACT_AVAILABLE = False
try:
    import tesseract_robotics
    TESSERACT_AVAILABLE = True
except ImportError:
    pass


def pytest_configure(config):
    """Add custom markers."""
    config.addinivalue_line(
        "markers", "tesseract: mark test as requiring tesseract installation"
    )


@pytest.fixture(scope="session")
def tesseract_available():
    """Check if tesseract is available."""
    return TESSERACT_AVAILABLE


@pytest.fixture
def skip_without_tesseract(tesseract_available):
    """Skip test if tesseract is not available."""
    if not tesseract_available:
        pytest.skip("tesseract_robotics not installed")


@pytest.fixture
def urdf_path():
    """Path to test URDF file."""
    # This would point to a test URDF in a fixtures directory
    test_dir = os.path.dirname(__file__)
    urdf_file = os.path.join(test_dir, "fixtures", "ur5.urdf")
    if os.path.exists(urdf_file):
        return urdf_file
    return None


@pytest.fixture
def srdf_path():
    """Path to test SRDF file."""
    test_dir = os.path.dirname(__file__)
    srdf_file = os.path.join(test_dir, "fixtures", "ur5.srdf")
    if os.path.exists(srdf_file):
        return srdf_file
    return None
