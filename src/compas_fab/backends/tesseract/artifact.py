"""Exact immutable robot descriptions consumed by Tesseract."""

from __future__ import annotations

import math
from enum import Enum
from typing import Mapping
from typing import Optional
from xml.etree import ElementTree

import yaml
from attrs import define

from .errors import CollisionMeshPolicyConflictError
from .errors import ContactManagerPluginConflictError
from .errors import InvalidKinematicsConfigError
from .errors import InvalidRobotResourceError
from .errors import InvalidSrdfError
from .errors import InvalidUrdfError
from .errors import KinematicsPluginConflictError
from .errors import RobotArtifactIdentityMismatchError
from .identity import BuildIdentity
from .resource_url import PackageResourceUrl

TESSERACT_XML_NAMESPACE = "https://github.com/tesseract-robotics/tesseract"
TESSERACT_MAKE_CONVEX_ATTRIBUTE = "{{{}}}make_convex".format(TESSERACT_XML_NAMESPACE)
KINEMATICS_PLUGIN_URL = "package://compas_fab/tesseract/kinematics.yaml"
CONTACT_MANAGERS_PLUGIN_URL = "package://compas_fab/tesseract/contact_managers.yaml"


class CollisionMeshPolicy(Enum):
    """Explicit global treatment of URDF collision meshes."""

    PRESERVE = "false"
    CONVEX_HULL = "true"


class KdlInverseKinematics(Enum):
    """Native KDL inverse-kinematics plugin selected for a group."""

    LMA = "KDLInvKinChainLMA"
    NEWTON_RAPHSON = "KDLInvKinChainNR"


class DiscreteContactManager(Enum):
    """Native discrete collision-manager plugin."""

    BULLET_BVH = "BulletDiscreteBVHManager"
    BULLET_SIMPLE = "BulletDiscreteSimpleManager"
    FCL_BVH = "FCLDiscreteBVHManager"


class ContinuousContactManager(Enum):
    """Native continuous collision-manager plugin."""

    BULLET_CAST_BVH = "BulletCastBVHManager"
    BULLET_CAST_SIMPLE = "BulletCastSimpleManager"


class CoupledTopology(Enum):
    """Which body an external axis carries in a coordinated kinematic group.

    The value is the native factory that resolves the coupled inverse kinematics.
    """

    #: Track/rail carries the robot: the manipulator base rides the positioner
    #: (moving base). Positioner forward kinematics place the robot base, then the
    #: manipulator solves to the target.
    ROBOT_ON_POSITIONER = "ROPInvKinFactory"
    #: Positioner carries the workpiece: the work object rides the positioner and
    #: the TCP tracks it. The robot base is fixed; the positioner reorients the part.
    ROBOT_WITH_EXTERNAL_POSITIONER = "REPInvKinFactory"


def native_coupled_ik_solver_name(topology: CoupledTopology) -> str:
    """Return the native coupled inverse-kinematics solver name for one topology.

    The single source of the coordinated solver name shared by the plugin-YAML
    emitter (which registers it as the group default) and any consumer that must
    name the same solver. The enum value is the native factory class, so the
    solver name is that class without its ``Factory`` suffix
    (``ROPInvKinFactory`` -> ``ROPInvKin``).

    Args:
        topology: Which body the external axis carries.

    Returns:
        The native inverse-kinematics solver name, e.g. ``"ROPInvKin"``.
    """
    return topology.value.removesuffix("Factory")


# The native coordinated (coupled) inverse-kinematics solver names, for classifying a
# planning group as coordinated. Derived from the topology enum so the set cannot drift.
_COUPLED_SOLVER_NAMES = frozenset(native_coupled_ik_solver_name(topology) for topology in CoupledTopology)
_REP_SOLVER_NAME = native_coupled_ik_solver_name(CoupledTopology.ROBOT_WITH_EXTERNAL_POSITIONER)


@define(frozen=True, slots=True)
class KdlKinematics:
    """Explicit KDL plugin configuration for one semantic group."""

    group: str
    base_link: str
    tip_link: str
    inverse: KdlInverseKinematics

    @classmethod
    def build(
        cls,
        group: str,
        base_link: str,
        tip_link: str,
        inverse: KdlInverseKinematics,
    ) -> KdlKinematics:
        """Validate one explicit KDL plugin selection.

        Args:
            group: Exact SRDF planning-group name.
            base_link: Exact base-link name.
            tip_link: Exact tip-link name.
            inverse: Native KDL inverse solver.

        Returns:
            Validated immutable KDL configuration.

        Raises:
            InvalidKinematicsConfigError: Any name is empty.
        """
        fields = {"group": group, "base_link": base_link, "tip_link": tip_link}
        empty = [name for name, value in fields.items() if not value]
        if empty:
            raise InvalidKinematicsConfigError("KDL kinematics fields are empty: {}.".format(", ".join(empty)))
        return cls(group, base_link, tip_link, inverse)


@define(frozen=True, slots=True)
class OpwParameters:
    """Closed-form OPW solver parameters for a spherical-wrist 6-DOF manipulator.

    These are the ABB/OPW kinematic constants (`a1, a2, b, c1..c4`) plus the
    per-joint `offsets` (radians) and `sign_corrections` (+/-1) that map the OPW
    convention onto the URDF joints.
    """

    a1: float
    a2: float
    b: float
    c1: float
    c2: float
    c3: float
    c4: float
    offsets: tuple[float, float, float, float, float, float]
    sign_corrections: tuple[int, int, int, int, int, int]

    @classmethod
    def build(
        cls,
        a1: float,
        a2: float,
        b: float,
        c1: float,
        c2: float,
        c3: float,
        c4: float,
        offsets: "tuple[float, ...] | list[float]",
        sign_corrections: "tuple[int, ...] | list[int]",
    ) -> OpwParameters:
        """Validate one set of OPW manipulator parameters.

        Args:
            a1, a2, b, c1, c2, c3, c4: OPW link constants (metres).
            offsets: Six joint offsets in radians.
            sign_corrections: Six values, each +1 or -1.

        Returns:
            Validated immutable OPW parameters.

        Raises:
            InvalidKinematicsConfigError: A constant is not finite, a sequence is
                not length six, or a sign correction is not +/-1.
        """
        constants = {"a1": a1, "a2": a2, "b": b, "c1": c1, "c2": c2, "c3": c3, "c4": c4}
        nonfinite = sorted(name for name, value in constants.items() if not math.isfinite(value))
        if nonfinite:
            raise InvalidKinematicsConfigError("OPW constants are not finite: {}.".format(", ".join(nonfinite)))
        offsets_tuple = tuple(float(value) for value in offsets)
        signs_tuple = tuple(int(value) for value in sign_corrections)
        if len(offsets_tuple) != 6:
            raise InvalidKinematicsConfigError("OPW offsets must have six values, got {}.".format(len(offsets_tuple)))
        if any(not math.isfinite(value) for value in offsets_tuple):
            raise InvalidKinematicsConfigError("OPW offsets must be finite.")
        if len(signs_tuple) != 6:
            raise InvalidKinematicsConfigError("OPW sign corrections must have six values, got {}.".format(len(signs_tuple)))
        if any(value not in (-1, 1) for value in signs_tuple):
            raise InvalidKinematicsConfigError("OPW sign corrections must each be +1 or -1.")
        return cls(a1, a2, b, c1, c2, c3, c4, offsets_tuple, signs_tuple)


@define(frozen=True, slots=True)
class CoupledKinematics:
    """Coordinated robot + external-axis (ROP/REP) kinematics for one group.

    A coupled group spans the manipulator and its external axes as one chain. The
    native solver named by `topology` composes a positioner forward-kinematics
    chain (`positioner_base_link` -> `positioner_tip_link`) with the manipulator
    inverse-kinematics chain (`manipulator_base_link` -> `manipulator_tip_link`),
    sampling each external joint at `positioner_sample_resolution`.
    """

    group: str
    topology: CoupledTopology
    positioner_base_link: str
    positioner_tip_link: str
    manipulator_base_link: str
    manipulator_tip_link: str
    manipulator: OpwParameters
    manipulator_reach: float
    positioner_sample_resolution: tuple[tuple[str, float], ...]

    @classmethod
    def build(
        cls,
        group: str,
        topology: CoupledTopology,
        positioner_base_link: str,
        positioner_tip_link: str,
        manipulator_base_link: str,
        manipulator_tip_link: str,
        manipulator: OpwParameters,
        manipulator_reach: float,
        positioner_sample_resolution: "tuple[tuple[str, float], ...] | list[tuple[str, float]]",
    ) -> CoupledKinematics:
        """Validate one coordinated coupled-kinematics configuration.

        Args:
            group: Exact SRDF group spanning the manipulator and its external axes.
            topology: Which body the external axis carries (ROP/REP).
            positioner_base_link: Base link of the positioner forward chain.
            positioner_tip_link: Tip link of the positioner forward chain.
            manipulator_base_link: Base link of the manipulator inverse chain.
            manipulator_tip_link: Tip link of the manipulator inverse chain.
            manipulator: OPW parameters for the manipulator inverse solver.
            manipulator_reach: Positive manipulator reach (metres).
            positioner_sample_resolution: One `(joint_name, resolution)` per external
                joint; resolution is the positive sampling step (metres or radians).

        Returns:
            Validated immutable coupled configuration.

        Raises:
            InvalidKinematicsConfigError: A name is empty, a type is wrong, the reach
                is not positive-finite, or the sample resolution is empty/invalid.
        """
        names = {
            "group": group,
            "positioner_base_link": positioner_base_link,
            "positioner_tip_link": positioner_tip_link,
            "manipulator_base_link": manipulator_base_link,
            "manipulator_tip_link": manipulator_tip_link,
        }
        empty = sorted(name for name, value in names.items() if not value)
        if empty:
            raise InvalidKinematicsConfigError("Coupled kinematics fields are empty: {}.".format(", ".join(empty)))
        if not math.isfinite(manipulator_reach) or manipulator_reach <= 0.0:
            raise InvalidKinematicsConfigError("Coupled manipulator reach must be positive, got {!r}.".format(manipulator_reach))
        samples = tuple((str(name), float(value)) for name, value in positioner_sample_resolution)
        if not samples:
            raise InvalidKinematicsConfigError("Coupled kinematics require at least one positioner sample resolution.")
        for name, value in samples:
            if not name:
                raise InvalidKinematicsConfigError("Positioner sample resolution joint name is empty.")
            if not math.isfinite(value) or value <= 0.0:
                raise InvalidKinematicsConfigError("Positioner sample resolution for {!r} must be positive, got {!r}.".format(name, value))
        sample_names = [name for name, _ in samples]
        if len(sample_names) != len(set(sample_names)):
            raise InvalidKinematicsConfigError("Positioner sample resolution joints must be unique.")
        return cls(
            group,
            topology,
            positioner_base_link,
            positioner_tip_link,
            manipulator_base_link,
            manipulator_tip_link,
            manipulator,
            manipulator_reach,
            samples,
        )


@define(frozen=True, slots=True)
class RobotResource:
    """One exact resource referenced by a robot description."""

    url: str
    content: bytes

    def __attrs_post_init__(self) -> None:
        _validate_resource(self.url, self.content)

    @classmethod
    def build(cls, url: str, content: bytes) -> RobotResource:
        """Validate and copy one robot resource.

        Args:
            url: Exact URL used in URDF/SRDF.
            content: Complete resource bytes.

        Returns:
            Immutable resource.

        Raises:
            InvalidRobotResourceError: The URL is empty or content is not bytes.
        """
        validated_url, validated_content = _validate_resource(url, content)
        return cls(validated_url, validated_content)


@define(frozen=True, slots=True)
class RobotArtifact:
    """Exact URDF, SRDF, and resources for a reproducible environment."""

    urdf: str
    srdf: str
    resources: tuple[RobotResource, ...]
    identity: BuildIdentity

    def __attrs_post_init__(self) -> None:
        resources = _validate_artifact_resources(self.resources)
        if not isinstance(self.identity, BuildIdentity):
            raise RobotArtifactIdentityMismatchError("Robot artifact identity must be BuildIdentity.")
        expected_identity = BuildIdentity.build(
            self.urdf,
            self.srdf,
            {resource.url: resource.content for resource in resources},
        )
        if self.identity != expected_identity:
            raise RobotArtifactIdentityMismatchError(
                "Robot artifact identity {} does not match recomputed identity {}.".format(
                    self.identity.digest,
                    expected_identity.digest,
                )
            )

    @classmethod
    def build(
        cls,
        urdf: str,
        srdf: str,
        resources: Mapping[str, bytes],
    ) -> RobotArtifact:
        """Build an immutable artifact without regenerating source XML.

        Args:
            urdf: Exact source URDF.
            srdf: Exact source SRDF.
            resources: Exact resource payloads keyed by description URL.

        Returns:
            Immutable, content-addressed artifact.

        Raises:
            InvalidRobotResourceError: A resource URL or payload is invalid.
        """
        artifact_resources = tuple(
            sorted(
                (RobotResource.build(url, content) for url, content in resources.items()),
                key=lambda resource: resource.url,
            )
        )
        identity = BuildIdentity.build(
            urdf,
            srdf,
            {resource.url: resource.content for resource in artifact_resources},
        )
        return cls(urdf, srdf, artifact_resources, identity)

    @classmethod
    def from_compas_urdf(
        cls,
        urdf: str,
        srdf: str,
        resources: Mapping[str, bytes],
        collision_mesh_policy: CollisionMeshPolicy,
    ) -> RobotArtifact:
        """Compile a standard URDF with an explicit Tesseract mesh policy.

        Args:
            urdf: Exact source URDF from COMPAS or another standard loader.
            srdf: Exact source SRDF.
            resources: Exact resource payloads keyed by description URL.
            collision_mesh_policy: Required global collision-mesh treatment.

        Returns:
            Content-addressed artifact containing Tesseract-compatible URDF.

        Raises:
            InvalidUrdfError: XML is malformed or its root is not `robot`.
            CollisionMeshPolicyConflictError: Existing policy conflicts.
        """
        try:
            root = ElementTree.fromstring(urdf)
        except ElementTree.ParseError as error:
            raise InvalidUrdfError("URDF XML cannot be parsed: {}".format(error)) from error
        if root.tag.rsplit("}", 1)[-1] != "robot":
            raise InvalidUrdfError("URDF root element must be 'robot', got {!r}.".format(root.tag))

        existing_policy = root.attrib.get(TESSERACT_MAKE_CONVEX_ATTRIBUTE)
        if existing_policy is not None:
            if existing_policy != collision_mesh_policy.value:
                raise CollisionMeshPolicyConflictError("URDF collision mesh policy is {!r}, requested {!r}.".format(existing_policy, collision_mesh_policy.value))
            return cls.build(urdf, srdf, resources)

        ElementTree.register_namespace("tesseract", TESSERACT_XML_NAMESPACE)
        root.set(TESSERACT_MAKE_CONVEX_ATTRIBUTE, collision_mesh_policy.value)
        compiled_urdf = ElementTree.tostring(root, encoding="unicode")
        return cls.build(compiled_urdf, srdf, resources)

    def with_kdl_kinematics(
        self,
        configurations: list[KdlKinematics],
    ) -> RobotArtifact:
        """Return an artifact with an explicit KDL plugin resource.

        Args:
            configurations: One typed KDL configuration per planned group.

        Returns:
            New content-addressed artifact with derived SRDF and plugin YAML.

        Raises:
            InvalidSrdfError: SRDF XML is malformed or its root is not `robot`.
            InvalidKinematicsConfigError: Configurations are empty, duplicate, or unknown.
            KinematicsPluginConflictError: Exact SRDF/resource already selects plugins.
        """
        if not configurations:
            raise InvalidKinematicsConfigError("At least one KDL configuration is required.")
        try:
            root = ElementTree.fromstring(self.srdf)
        except ElementTree.ParseError as error:
            raise InvalidSrdfError("SRDF XML cannot be parsed: {}".format(error)) from error
        if root.tag.rsplit("}", 1)[-1] != "robot":
            raise InvalidSrdfError("SRDF root element must be 'robot', got {!r}.".format(root.tag))
        if root.find("kinematics_plugin_config") is not None:
            raise KinematicsPluginConflictError("Exact SRDF already contains kinematics_plugin_config.")
        if any(resource.url == KINEMATICS_PLUGIN_URL for resource in self.resources):
            raise KinematicsPluginConflictError("Artifact already contains {!r}.".format(KINEMATICS_PLUGIN_URL))

        group_names = {element.attrib["name"] for element in root.findall("group")}
        configured_names = [configuration.group for configuration in configurations]
        if len(configured_names) != len(set(configured_names)):
            raise InvalidKinematicsConfigError("KDL group configurations must be unique.")
        unknown = sorted(set(configured_names) - group_names)
        if unknown:
            raise InvalidKinematicsConfigError("KDL configurations reference unknown SRDF groups: {}.".format(", ".join(unknown)))

        plugin_element = ElementTree.Element("kinematics_plugin_config", {"filename": KINEMATICS_PLUGIN_URL})
        root.append(plugin_element)
        compiled_srdf = ElementTree.tostring(root, encoding="unicode")
        plugin_yaml = _kdl_plugin_yaml(configurations).encode("utf-8")
        resources = {resource.url: resource.content for resource in self.resources}
        resources[KINEMATICS_PLUGIN_URL] = plugin_yaml
        return RobotArtifact.build(self.urdf, compiled_srdf, resources)

    def with_coupled_kinematics(
        self,
        configuration: CoupledKinematics,
    ) -> RobotArtifact:
        """Return an artifact with a coordinated ROP/REP kinematics plugin.

        A coupled group resolves the manipulator and its external axes together;
        the emitted plugin names the native `ROPInvKin`/`REPInvKin` solver for the
        group. A cell selects either KDL or coupled kinematics, never both.

        Args:
            configuration: Typed coupled configuration for one combined group.

        Returns:
            New content-addressed artifact with derived SRDF and plugin YAML.

        Raises:
            InvalidSrdfError: SRDF XML is malformed or its root is not `robot`.
            InvalidKinematicsConfigError: The group is not defined in the SRDF.
            KinematicsPluginConflictError: Exact SRDF/resource already selects plugins.
        """
        try:
            root = ElementTree.fromstring(self.srdf)
        except ElementTree.ParseError as error:
            raise InvalidSrdfError("SRDF XML cannot be parsed: {}".format(error)) from error
        if root.tag.rsplit("}", 1)[-1] != "robot":
            raise InvalidSrdfError("SRDF root element must be 'robot', got {!r}.".format(root.tag))
        if root.find("kinematics_plugin_config") is not None:
            raise KinematicsPluginConflictError("Exact SRDF already contains kinematics_plugin_config.")
        if any(resource.url == KINEMATICS_PLUGIN_URL for resource in self.resources):
            raise KinematicsPluginConflictError("Artifact already contains {!r}.".format(KINEMATICS_PLUGIN_URL))

        group_names = {element.attrib["name"] for element in root.findall("group")}
        if configuration.group not in group_names:
            raise InvalidKinematicsConfigError("Coupled configuration references unknown SRDF group: {}.".format(configuration.group))

        plugin_element = ElementTree.Element("kinematics_plugin_config", {"filename": KINEMATICS_PLUGIN_URL})
        root.append(plugin_element)
        compiled_srdf = ElementTree.tostring(root, encoding="unicode")
        plugin_yaml = _coupled_plugin_yaml(configuration).encode("utf-8")
        resources = {resource.url: resource.content for resource in self.resources}
        resources[KINEMATICS_PLUGIN_URL] = plugin_yaml
        return RobotArtifact.build(self.urdf, compiled_srdf, resources)

    def with_contact_managers(
        self,
        discrete: DiscreteContactManager,
        continuous: ContinuousContactManager,
    ) -> RobotArtifact:
        """Return an artifact with explicit native collision managers.

        Args:
            discrete: Exact discrete contact-manager plugin.
            continuous: Exact continuous contact-manager plugin.

        Returns:
            New content-addressed artifact with derived SRDF and plugin YAML.

        Raises:
            InvalidSrdfError: SRDF XML is malformed or its root is not `robot`.
            ContactManagerPluginConflictError: Exact SRDF/resource already selects plugins.
        """
        try:
            root = ElementTree.fromstring(self.srdf)
        except ElementTree.ParseError as error:
            raise InvalidSrdfError("SRDF XML cannot be parsed: {}".format(error)) from error
        if root.tag.rsplit("}", 1)[-1] != "robot":
            raise InvalidSrdfError("SRDF root element must be 'robot', got {!r}.".format(root.tag))
        if root.find("contact_managers_plugin_config") is not None:
            raise ContactManagerPluginConflictError("Exact SRDF already contains contact_managers_plugin_config.")
        if any(resource.url == CONTACT_MANAGERS_PLUGIN_URL for resource in self.resources):
            raise ContactManagerPluginConflictError("Artifact already contains {!r}.".format(CONTACT_MANAGERS_PLUGIN_URL))

        plugin_element = ElementTree.Element("contact_managers_plugin_config", {"filename": CONTACT_MANAGERS_PLUGIN_URL})
        root.append(plugin_element)
        compiled_srdf = ElementTree.tostring(root, encoding="unicode")
        plugin_yaml = _contact_manager_plugin_yaml(discrete, continuous).encode("utf-8")
        resources = {resource.url: resource.content for resource in self.resources}
        resources[CONTACT_MANAGERS_PLUGIN_URL] = plugin_yaml
        return RobotArtifact.build(self.urdf, compiled_srdf, resources)

    def resource(self, url: str) -> RobotResource:
        """Return a resource by its exact source URL.

        Args:
            url: Exact URL from URDF/SRDF.

        Returns:
            Matching immutable resource.

        Raises:
            InvalidRobotResourceError: The artifact does not contain the URL.
        """
        for resource in self.resources:
            if resource.url == url:
                return resource
        raise InvalidRobotResourceError("Robot artifact {} does not contain resource {!r}.".format(self.identity.digest, url))

    def default_inv_kin_solver(self, group: str) -> Optional[str]:
        """Return the group's configured default inverse-kinematics solver name.

        Reads the emitted kinematics plugin config (the exact bytes the native
        environment loads) rather than a discarded build-time configuration, so
        the name cannot drift from the solver that actually runs. For a coupled
        group this is the coordinated ROP/REP solver named by
        `native_coupled_ik_solver_name`.

        Args:
            group: Exact SRDF planning-group name.

        Returns:
            The default solver name, or None when the artifact declares no
            kinematics plugin config or the group has no inverse-kinematics
            plugin entry (an uncoordinated group whose planner uses the library
            default solver).
        """
        if not any(resource.url == KINEMATICS_PLUGIN_URL for resource in self.resources):
            return None
        document = yaml.safe_load(self.resource(KINEMATICS_PLUGIN_URL).content.decode("utf-8"))
        inv_kin_plugins = document.get("kinematic_plugins", {}).get("inv_kin_plugins", {})
        group_plugins = inv_kin_plugins.get(group)
        if not group_plugins:
            return None
        default_solver = group_plugins.get("default")
        return default_solver if isinstance(default_solver, str) else None

    def is_coupled_group(self, group: str) -> bool:
        """Whether a group's default solver is a coordinated ROP/REP solver.

        Args:
            group: Exact SRDF planning-group name.

        Returns:
            True when the emitted kinematics plugin makes the group's default
            inverse-kinematics solver the coordinated ``ROPInvKin``/``REPInvKin``.
        """
        return self.default_inv_kin_solver(group) in _COUPLED_SOLVER_NAMES

    def coupled_group_frames(self, group: str) -> Optional[tuple[str, str]]:
        """Return a coordinated group's ``(working_frame, tcp_frame)`` link names.

        For a coordinated group the TCP is the manipulator tip and the working frame is
        the positioner link the coordinated target references: the positioner **tip** for
        a robot-with-external-positioner cell (the workpiece rides it and targets are
        authored relative to it -- ``REPInvKin`` accepts only this frame) and the
        positioner **base** for a robot-on-positioner cell (a fixed reference the moving
        robot base is measured against). The links are read from the emitted
        kinematics-plugin YAML -- the same authoritative bytes the native solver loads --
        so they cannot drift. COMPAS's joint-group accessors cannot report these for a
        cross-branch coupled group.

        Args:
            group: Exact SRDF planning-group name.

        Returns:
            ``(working_frame, tcp_frame)`` for a coordinated group, else ``None``.

        Raises:
            InvalidKinematicsConfigError: The group is coordinated but its plugin config
                lacks the positioner/manipulator link names.
        """
        solver = self.default_inv_kin_solver(group)
        if solver not in _COUPLED_SOLVER_NAMES:
            return None
        document = yaml.safe_load(self.resource(KINEMATICS_PLUGIN_URL).content.decode("utf-8"))
        group_plugins = document.get("kinematic_plugins", {}).get("inv_kin_plugins", {}).get(group, {})
        plugin_config = group_plugins.get("plugins", {}).get(solver, {}).get("config", {})
        positioner = plugin_config.get("positioner", {}).get("config", {})
        manipulator = plugin_config.get("manipulator", {}).get("config", {})
        tcp_frame = manipulator.get("tip_link")
        working_frame = positioner.get("tip_link") if solver == _REP_SOLVER_NAME else positioner.get("base_link")
        if not isinstance(working_frame, str) or not isinstance(tcp_frame, str):
            raise InvalidKinematicsConfigError("Coupled group {!r} plugin config lacks positioner/manipulator link names.".format(group))
        return working_frame, tcp_frame


def _validate_resource(url: object, content: object) -> tuple[str, bytes]:
    normalized_url = PackageResourceUrl.build(url)
    if not isinstance(content, bytes):
        raise InvalidRobotResourceError("Robot resource {!r} must contain bytes, got {}.".format(url, type(content).__name__))
    return normalized_url.value, bytes(content)


def _validate_artifact_resources(resources: object) -> tuple[RobotResource, ...]:
    if not isinstance(resources, tuple):
        raise InvalidRobotResourceError("Robot artifact resources must be an immutable tuple.")
    for resource in resources:
        if not isinstance(resource, RobotResource):
            raise InvalidRobotResourceError("Robot artifact resources must contain RobotResource values, got {}.".format(type(resource).__name__))
        _validate_resource(resource.url, resource.content)
    urls = tuple(resource.url for resource in resources)
    if len(urls) != len(set(urls)):
        raise InvalidRobotResourceError("Robot artifact resource URLs must be unique.")
    if urls != tuple(sorted(urls)):
        raise InvalidRobotResourceError("Robot artifact resources must use canonical URL order.")
    return resources


def _kdl_plugin_yaml(configurations: list[KdlKinematics]) -> str:
    fwd_plugins = {}
    inv_plugins = {}
    for configuration in configurations:
        link_config = {
            "base_link": configuration.base_link,
            "tip_link": configuration.tip_link,
        }
        fwd_plugins[configuration.group] = {
            "default": "KDLFwdKinChain",
            "plugins": {
                "KDLFwdKinChain": {
                    "class": "KDLFwdKinChainFactory",
                    "config": link_config,
                }
            },
        }
        inv_plugins[configuration.group] = {
            "default": configuration.inverse.value,
            "plugins": {
                configuration.inverse.value: {
                    "class": "{}Factory".format(configuration.inverse.value),
                    "config": link_config,
                }
            },
        }
    document = {
        "kinematic_plugins": {
            "search_libraries": ["tesseract_kinematics_kdl_factories"],
            "fwd_kin_plugins": fwd_plugins,
            "inv_kin_plugins": inv_plugins,
        }
    }
    return yaml.safe_dump(document, sort_keys=False)


def _coupled_plugin_yaml(configuration: CoupledKinematics) -> str:
    opw = configuration.manipulator
    solver_name = native_coupled_ik_solver_name(configuration.topology)
    plugin_config = {
        "manipulator_reach": configuration.manipulator_reach,
        "positioner_sample_resolution": [{"name": name, "value": value} for name, value in configuration.positioner_sample_resolution],
        "positioner": {
            "class": "KDLFwdKinChainFactory",
            "config": {
                "base_link": configuration.positioner_base_link,
                "tip_link": configuration.positioner_tip_link,
            },
        },
        "manipulator": {
            "class": "OPWInvKinFactory",
            "config": {
                "base_link": configuration.manipulator_base_link,
                "tip_link": configuration.manipulator_tip_link,
                "params": {
                    "a1": opw.a1,
                    "a2": opw.a2,
                    "b": opw.b,
                    "c1": opw.c1,
                    "c2": opw.c2,
                    "c3": opw.c3,
                    "c4": opw.c4,
                    "offsets": list(opw.offsets),
                    "sign_corrections": list(opw.sign_corrections),
                },
            },
        },
    }
    document = {
        "kinematic_plugins": {
            "search_libraries": [
                "tesseract_kinematics_kdl_factories",
                "tesseract_kinematics_opw_factories",
            ],
            "inv_kin_plugins": {
                configuration.group: {
                    "default": solver_name,
                    "plugins": {solver_name: {"class": configuration.topology.value, "config": plugin_config}},
                }
            },
        }
    }
    return yaml.safe_dump(document, sort_keys=False)


def _contact_manager_plugin_yaml(
    discrete: DiscreteContactManager,
    continuous: ContinuousContactManager,
) -> str:
    discrete_library = "tesseract_collision_fcl_factories" if discrete is DiscreteContactManager.FCL_BVH else "tesseract_collision_bullet_factories"
    search_libraries = [discrete_library]
    if "tesseract_collision_bullet_factories" not in search_libraries:
        search_libraries.append("tesseract_collision_bullet_factories")
    document = {
        "contact_manager_plugins": {
            "search_libraries": search_libraries,
            "discrete_plugins": {
                "default": discrete.value,
                "plugins": {discrete.value: {"class": "{}Factory".format(discrete.value)}},
            },
            "continuous_plugins": {
                "default": continuous.value,
                "plugins": {continuous.value: {"class": "{}Factory".format(continuous.value)}},
            },
        }
    }
    return yaml.safe_dump(document, sort_keys=False)
