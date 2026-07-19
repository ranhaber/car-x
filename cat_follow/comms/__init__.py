"""In-process comms layer for the contract-driven runtime.

Milestone 2 ships an in-process, no-network implementation: producers call
``CommsManager.submit_tracking`` / ``submit_command`` directly.  The wire
format and validation rules are honored so a UDP transport in Milestone 3
can plug in without touching the rest of the control stack.
"""

from cat_follow.comms.comms_manager import CommsManager
from cat_follow.comms.messages import (
    AckMessage,
    CommandMessage,
    SchemaVersionError,
    TrackingCar,
    TrackingCat,
    TrackingMessage,
)
from cat_follow.comms.udp_receiver import UdpReceiver
from cat_follow.comms.udp_sender import UdpSender

__all__ = [
    "AckMessage",
    "CommandMessage",
    "CommsManager",
    "SchemaVersionError",
    "TrackingCar",
    "TrackingCat",
    "TrackingMessage",
    "UdpReceiver",
    "UdpSender",
]
