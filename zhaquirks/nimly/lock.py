"""Device handler for Nimly Smart Locks."""

from zigpy.quirks.v2 import QuirkBuilder
from zigpy.zdo.types import NodeDescriptor

from zhaquirks.nimly import NIMLY

# clears the mains powered mac capability flag
NIMLY_LOCK_NODE_DESCRIPTOR = NodeDescriptor(
    logical_type=2,
    complex_descriptor_available=0,
    user_descriptor_available=0,
    reserved=0,
    aps_flags=0,
    frequency_band=8,
    manufacturer_code=4660,
    maximum_buffer_size=108,
    maximum_incoming_transfer_size=127,
    server_mask=11264,
    maximum_outgoing_transfer_size=127,
    descriptor_capability_field=0,
    mac_capability_flags=NodeDescriptor.MACCapabilityFlags.AllocateAddress
    | NodeDescriptor.MACCapabilityFlags.RxOnWhenIdle,
)


(
    QuirkBuilder(NIMLY, "NimlyPRO")
    .also_applies_to(NIMLY, "NimlyCode")
    .also_applies_to(NIMLY, "NimlyTouch")
    .also_applies_to(NIMLY, "NimlyIn")
    .also_applies_to(NIMLY, "EasyFingerTouch")
    .also_applies_to(NIMLY, "EasyCodeTouch")
    .node_descriptor(NIMLY_LOCK_NODE_DESCRIPTOR)
    .add_to_registry()
)
