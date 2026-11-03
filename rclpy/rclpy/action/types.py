"""Structural contracts of the classes produced by the action IDL generator."""

from typing import Generic, Protocol, TypeVar, TypedDict

from action_msgs.msg import GoalInfo, GoalStatusArray
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Time
from unique_identifier_msgs.msg import UUID

from rclpy.type_support import ServiceType
from rclpy import native_types

GoalT = TypeVar('GoalT', default=object)
ResultT = TypeVar('ResultT', default=object)
FeedbackT = TypeVar('FeedbackT', default=object)


class GoalRequest(Protocol[GoalT]):
    @property
    def goal_id(self) -> UUID: ...

    @goal_id.setter
    def goal_id(self, value: UUID) -> None: ...
    @property
    def goal(self) -> GoalT: ...

    @goal.setter
    def goal(self, value: GoalT) -> None: ...


class GoalResponse(Protocol):
    @property
    def accepted(self) -> bool: ...

    @accepted.setter
    def accepted(self, value: bool) -> None: ...
    @property
    def stamp(self) -> Time: ...

    @stamp.setter
    def stamp(self, value: Time) -> None: ...


class ResultRequest(Protocol):
    @property
    def goal_id(self) -> UUID: ...

    @goal_id.setter
    def goal_id(self, value: UUID) -> None: ...


class ResultResponse(Protocol[ResultT]):
    @property
    def status(self) -> int: ...

    @status.setter
    def status(self, value: int) -> None: ...
    @property
    def result(self) -> ResultT: ...

    @result.setter
    def result(self, value: ResultT) -> None: ...


class ActionFeedback(Protocol[FeedbackT]):
    @property
    def goal_id(self) -> UUID: ...

    @goal_id.setter
    def goal_id(self, value: UUID) -> None: ...
    @property
    def feedback(self) -> FeedbackT: ...

    @feedback.setter
    def feedback(self, value: FeedbackT) -> None: ...


class ActionImplementation(Protocol[GoalT, ResultT, FeedbackT]):
    @property
    def SendGoalService(self) -> ServiceType[GoalRequest[GoalT], GoalResponse]: ...

    @property
    def GetResultService(self) -> ServiceType[ResultRequest, ResultResponse[ResultT]]: ...

    @property
    def CancelGoalService(self) -> ServiceType[CancelGoal.Request, CancelGoal.Response]: ...

    @property
    def FeedbackMessage(self) -> type[ActionFeedback[FeedbackT]]: ...

    @property
    def GoalStatusMessage(self) -> type[GoalStatusArray]: ...


class ActionType(Protocol[GoalT, ResultT, FeedbackT]):
    @property
    def Goal(self) -> type[GoalT]: ...

    @property
    def Result(self) -> type[ResultT]: ...

    @property
    def Feedback(self) -> type[FeedbackT]: ...

    @property
    def Impl(self) -> ActionImplementation[GoalT, ResultT, FeedbackT]: ...


class ClientTakenData(TypedDict, Generic[ResultT, FeedbackT], total=False):
    goal: tuple[int, GoalResponse]
    cancel: tuple[int, CancelGoal.Response]
    result: tuple[int, ResultResponse[ResultT]]
    feedback: ActionFeedback[FeedbackT]
    status: GoalStatusArray


class GoalHandleIdentity(Protocol):
    @property
    def goal_id(self) -> UUID: ...


class ServerTakenData(TypedDict, Generic[GoalT], total=False):
    goal: tuple[native_types.rmw_request_id_t, GoalRequest[GoalT]]
    cancel: tuple[native_types.rmw_request_id_t, CancelGoal.Request]
    result: tuple[native_types.rmw_request_id_t, ResultRequest]
    expired: tuple[GoalInfo, ...]
