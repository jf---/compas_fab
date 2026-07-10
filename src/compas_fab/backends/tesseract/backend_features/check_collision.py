"""Native Tesseract discrete collision query and COMPAS failure projection."""

from __future__ import annotations

from typing import TYPE_CHECKING
from typing import Optional

from tesseract_robotics.tesseract_collision import ContactRequest
from tesseract_robotics.tesseract_collision import ContactResultMap
from tesseract_robotics.tesseract_collision import ContactTestType_ALL

from compas_fab.backends.interfaces import CheckCollision
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState

from ..cell_state_validation import require_matching_cell_state
from ..client import TesseractClient
from ..collision import TesseractCollisionResult
from ..collision import is_collision_distance
from ..errors import MissingContactManagerPluginError
from ..errors import MissingTesseractStartStateError
from ..errors import TesseractCollisionError
from ..errors import TesseractContactQueryError
from ..errors import UnknownTesseractOptionError
from ..errors import UnsupportedTesseractCellStateError
from ..joint_state import apply_active_joint_state


class TesseractCheckCollision(CheckCollision):
    """Query the exact native contact manager on an isolated environment."""

    if TYPE_CHECKING:
        client: TesseractClient

    def check_collision_native(
        self,
        robot_cell_state: RobotCellState,
        request: ContactRequest,
    ) -> TesseractCollisionResult:
        """Return the complete native contact result for an exact request."""
        if not isinstance(request, ContactRequest):
            raise TesseractContactQueryError("Native collision request must be ContactRequest, got {}.".format(type(request).__name__))
        client: TesseractClient = self.client
        robot_cell: RobotCell = client.robot_cell
        if robot_cell is None:
            raise MissingTesseractStartStateError("Call set_robot_cell before collision checking.")
        require_matching_cell_state(robot_cell, robot_cell_state, "collision checking")
        if robot_cell_state.tool_states or robot_cell_state.rigid_body_states:
            raise UnsupportedTesseractCellStateError("Native collision checking requires tools and rigid bodies to be applied to the environment first.")
        configuration = robot_cell_state.robot_configuration
        if configuration is None:
            raise MissingTesseractStartStateError("robot_cell_state.robot_configuration is required for collision checking.")

        robot = client.environment.clone_robot()
        try:
            apply_active_joint_state(
                robot,
                configuration,
                "complete collision-check configuration",
            )
            native_environment = robot.env
        except (KeyError, RuntimeError, ValueError) as error:
            raise TesseractContactQueryError("Tesseract collision state application failed: {}.".format(error)) from error
        try:
            manager = native_environment.getDiscreteContactManager()
        except RuntimeError as error:
            raise MissingContactManagerPluginError("Tesseract collision checking cannot load its discrete contact manager: {}.".format(error)) from error
        if manager is None:
            raise MissingContactManagerPluginError("Tesseract collision checking requires a configured discrete contact manager.")
        try:
            manager.setActiveCollisionObjects(native_environment.getActiveLinkNames())
            manager.setCollisionObjectsTransform(native_environment.getState().link_transforms)
            result_map = ContactResultMap()
            manager.contactTest(result_map, request)
            native_results = result_map.flattenCopyResults()
        except (KeyError, RuntimeError, ValueError) as error:
            raise TesseractContactQueryError("Tesseract discrete contact query failed: {}.".format(error)) from error
        return TesseractCollisionResult(request, result_map, native_results)

    def check_collision(
        self,
        robot_cell_state: RobotCellState,
        options: Optional[dict[str, object]] = None,
    ) -> None:
        """Raise with the retained native result when actual contact is found."""
        values = dict(options or {})
        unknown = sorted(set(values) - {"contact_request"})
        if unknown:
            raise UnknownTesseractOptionError("Unknown Tesseract collision options: {}.".format(", ".join(unknown)))
        request_value = values.get("contact_request")
        if request_value is None:
            request = ContactRequest(ContactTestType_ALL)
        elif isinstance(request_value, ContactRequest):
            request = request_value
        else:
            raise TesseractContactQueryError("contact_request must be ContactRequest, got {}.".format(type(request_value).__name__))

        result = self.check_collision_native(robot_cell_state, request)
        colliding = [result.native_results[index] for index in range(len(result.native_results)) if is_collision_distance(float(result.native_results[index].distance))]
        self.client._store_robot_cell_state(robot_cell_state)
        if not colliding:
            return None
        link_pairs = tuple((contact.link_names[0], contact.link_names[1]) for contact in colliding)
        distances = tuple(float(contact.distance) for contact in colliding)
        message = "Tesseract found {} colliding contact(s): {}.".format(
            len(colliding),
            ", ".join("{} / {} ({:.9g} m)".format(pair[0], pair[1], distance) for pair, distance in zip(link_pairs, distances)),
        )
        raise TesseractCollisionError(message, result, link_pairs, distances)
