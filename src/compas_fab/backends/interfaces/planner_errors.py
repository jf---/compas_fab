from compas_fab.backends.exceptions import BackendError


class PlannerContractError(BackendError):
    def __init__(self, message: str) -> None:
        Exception.__init__(self, message)
        self.message = message


class InvalidPlannerCapabilitiesError(PlannerContractError):
    pass


class PlannerCapabilityError(PlannerContractError):
    pass


class InvalidPlannerOptionsError(PlannerContractError):
    pass


class ConflictingPlannerOptionsError(PlannerContractError):
    pass
