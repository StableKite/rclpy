"""Lazily expose the actual native classes for precise runtime type annotations.

Importing this module does not load the ROS extension. Attribute access returns
the original class object, retaining native identity and constructor behavior.
"""

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
    ActionClient = _rclpy.ActionClient
    ActionGoalHandle = _rclpy.ActionGoalHandle
    ActionServer = _rclpy.ActionServer
    Client = _rclpy.Client
    Clock = _rclpy.Clock
    ClockChange = _rclpy.ClockChange
    ClockEvent = _rclpy.ClockEvent
    ClockType = _rclpy.ClockType
    Context = _rclpy.Context
    Destroyable = _rclpy.Destroyable
    EventHandle = _rclpy.EventHandle
    EventsExecutor = _rclpy.EventsExecutor
    GoalEvent = _rclpy.GoalEvent
    GuardCondition = _rclpy.GuardCondition
    InvalidHandle = _rclpy.InvalidHandle
    LifecycleStateMachine = _rclpy.LifecycleStateMachine
    Node = _rclpy.Node
    NodeNameNonExistentError = _rclpy.NodeNameNonExistentError
    NotImplementedError = _rclpy.NotImplementedError
    Publisher = _rclpy.Publisher
    QoSCheckCompatibleResult = _rclpy.QoSCheckCompatibleResult
    QoSCompatibility = _rclpy.QoSCompatibility
    RCLError = _rclpy.RCLError
    RCLInvalidROSArgsError = _rclpy.RCLInvalidROSArgsError
    RCUTILS_LOG_SEVERITY = _rclpy.RCUTILS_LOG_SEVERITY
    RCUtilsError = _rclpy.RCUtilsError
    RMWError = _rclpy.RMWError
    Service = _rclpy.Service
    SignalHandlerOptions = _rclpy.SignalHandlerOptions
    Subscription = _rclpy.Subscription
    Timer = _rclpy.Timer
    TimerCancelledError = _rclpy.TimerCancelledError
    TransitionCallbackReturnType = _rclpy.TransitionCallbackReturnType
    TypeDescriptionService = _rclpy.TypeDescriptionService
    UnknownROSArgsError = _rclpy.UnknownROSArgsError
    UnsupportedEventTypeError = _rclpy.UnsupportedEventTypeError
    WaitSet = _rclpy.WaitSet
    rcl_duration_t = _rclpy.rcl_duration_t
    rcl_publisher_event_type_t = _rclpy.rcl_publisher_event_type_t
    rcl_subscription_event_type_t = _rclpy.rcl_subscription_event_type_t
    rcl_time_point_t = _rclpy.rcl_time_point_t
    rmw_incompatible_type_status_t = _rclpy.rmw_incompatible_type_status_t
    rmw_liveliness_changed_status_t = _rclpy.rmw_liveliness_changed_status_t
    rmw_liveliness_lost_status_t = _rclpy.rmw_liveliness_lost_status_t
    rmw_matched_status_t = _rclpy.rmw_matched_status_t
    rmw_message_lost_status_t = _rclpy.rmw_message_lost_status_t
    rmw_offered_deadline_missed_status_t = _rclpy.rmw_offered_deadline_missed_status_t
    rmw_qos_policy_kind_t = _rclpy.rmw_qos_policy_kind_t
    rmw_qos_profile_t = _rclpy.rmw_qos_profile_t
    rmw_request_id_t = _rclpy.rmw_request_id_t
    rmw_requested_deadline_missed_status_t = _rclpy.rmw_requested_deadline_missed_status_t
    rmw_requested_qos_incompatible_event_status_t = _rclpy.rmw_requested_qos_incompatible_event_status_t
    rmw_service_info_t = _rclpy.rmw_service_info_t

__all__ = ('ActionClient', 'ActionGoalHandle', 'ActionServer', 'Client', 'Clock', 'ClockChange', 'ClockEvent', 'ClockType', 'Context', 'Destroyable', 'EventHandle', 'EventsExecutor', 'GoalEvent', 'GuardCondition', 'InvalidHandle', 'LifecycleStateMachine', 'Node', 'NodeNameNonExistentError', 'NotImplementedError', 'Publisher', 'QoSCheckCompatibleResult', 'QoSCompatibility', 'RCLError', 'RCLInvalidROSArgsError', 'RCUTILS_LOG_SEVERITY', 'RCUtilsError', 'RMWError', 'Service', 'SignalHandlerOptions', 'Subscription', 'Timer', 'TimerCancelledError', 'TransitionCallbackReturnType', 'TypeDescriptionService', 'UnknownROSArgsError', 'UnsupportedEventTypeError', 'WaitSet', 'rcl_duration_t', 'rcl_publisher_event_type_t', 'rcl_subscription_event_type_t', 'rcl_time_point_t', 'rmw_incompatible_type_status_t', 'rmw_liveliness_changed_status_t', 'rmw_liveliness_lost_status_t', 'rmw_matched_status_t', 'rmw_message_lost_status_t', 'rmw_offered_deadline_missed_status_t', 'rmw_qos_policy_kind_t', 'rmw_qos_profile_t', 'rmw_request_id_t', 'rmw_requested_deadline_missed_status_t', 'rmw_requested_qos_incompatible_event_status_t', 'rmw_service_info_t')


def __getattr__(name: str) -> type[object]:
    if name not in __all__:
        raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
    from rclpy.impl.implementation_singleton import rclpy_implementation
    value: object = getattr(rclpy_implementation, name)
    if not isinstance(value, type):
        raise TypeError(f'native attribute {name!r} is not a class')
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
