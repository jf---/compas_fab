"""Named failures for immutable Grasshopper tree contracts."""


class TreeContractError(ValueError):
    """Base class for invalid tree values."""


class InvalidTreeRootIdError(TreeContractError):
    """Raised when a tree routing identifier is invalid."""


class InvalidItemIndexError(TreeContractError):
    """Raised when an item coordinate index is invalid."""


class InvalidGhPathError(TreeContractError):
    """Raised when a Grasshopper path is empty or has an invalid segment."""


class InvalidTreeCoordinateError(TreeContractError):
    """Raised when a branch or item coordinate contains invalid values."""


class InvalidTreeItemError(TreeContractError):
    """Raised when null and value item state is inconsistent."""


class InvalidTreeBranchError(TreeContractError):
    """Raised when a branch contains an invalid path or item tuple."""


class NonCanonicalTreeOrderError(TreeContractError):
    """Raised when branches are not in canonical host path order."""


class DuplicateTreePathError(TreeContractError):
    """Raised when multiple branches use the same exact path."""


class InvalidShapeTagError(TreeContractError):
    """Raised when domain shape metadata is invalid."""


class InvalidItemShapeError(TreeContractError):
    """Raised when item-shape kind, tag, and length disagree."""


class InvalidFixedVectorShapeError(TreeContractError):
    """Raised when a fixed vector receives a non-vector shape."""


class FixedVectorLengthError(TreeContractError):
    """Raised when fixed-vector values do not match their declared length."""


class InvalidPortSemanticsError(TreeContractError):
    """Raised when a port declaration contains invalid semantic axes."""


class TreeScalarBranchCardinalityError(TreeContractError):
    """Raised when scalar conversion receives other than one branch."""


class TreeScalarItemCardinalityError(TreeContractError):
    """Raised when scalar conversion receives other than one item."""


class NullTreeScalarError(TreeContractError):
    """Raised when scalar conversion receives a null item slot."""


class TreeAtomicBranchCardinalityError(TreeContractError):
    """Raised when atomic conversion receives other than one branch."""


class TreeAtomicItemCardinalityError(TreeContractError):
    """Raised when atomic conversion receives other than one item."""


class NullTreeAtomicError(TreeContractError):
    """Raised when atomic conversion receives a null item slot."""


class InvalidAtomicTreeItemTypeError(TreeContractError):
    """Raised when an atomic tree item has the wrong domain type."""
