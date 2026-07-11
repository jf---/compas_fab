"""Exact immutable robot descriptions consumed by Tesseract."""

from __future__ import annotations

from enum import Enum
from typing import Mapping
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
