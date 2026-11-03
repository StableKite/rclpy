"""Preserve the native service-introspection enum through an importable typed alias."""
from rclpy.impl.implementation_singleton import rclpy_implementation

ServiceIntrospectionState = rclpy_implementation.service_introspection.ServiceIntrospectionState
