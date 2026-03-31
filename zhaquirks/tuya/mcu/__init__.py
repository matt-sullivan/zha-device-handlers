"""Tuya MCU communications."""

from __future__ import annotations

from collections.abc import Callable
import dataclasses
import datetime
import logging
from typing import Any, Final, Optional, Protocol, Union

from zigpy.quirks import CustomCluster
import zigpy.types as t
from zigpy.typing import UNDEFINED, UndefinedType
from zigpy.zcl import foundation
from zigpy.zcl.clusters.closures import WindowCovering
from zigpy.zcl.clusters.general import LevelControl, OnOff
from zigpy.zcl.foundation import ZCLAttributeDef

from zhaquirks import Bus, DoublingPowerConfigurationCluster

# add EnchantedDevice import for custom quirks backwards compatibility
from zhaquirks.tuya import (
    ATTR_COVER_DIRECTION,
    ATTR_COVER_DIRECTION_NAME,
    ATTR_COVER_INVERTED,
    ATTR_COVER_INVERTED_NAME,
    ATTR_COVER_MOTOR_STATUS,
    ATTR_COVER_MOTOR_STATUS_NAME,
    ATTR_COVER_POSITION_NAME,
    TUYA_DP_ID_CONTROL,
    TUYA_DP_ID_LIMIT_SETTINGS,
    TUYA_DP_ID_PERCENT_CONTROL,
    TUYA_DP_ID_SMALL_STEP,
    TUYA_MCU_COMMAND,
    TUYA_MCU_SET_DATAPOINTS,
    TUYA_MCU_VERSION_RSP,
    TUYA_SET_DATA,
    TUYA_SET_TIME,
    WINDOW_COVER_COMMAND_CLEAR_BOTH_LIMITS,
    WINDOW_COVER_COMMAND_CLEAR_BOTH_LIMITS_NAME,
    WINDOW_COVER_COMMAND_CLEAR_CLOSE_LIMIT,
    WINDOW_COVER_COMMAND_CLEAR_CLOSE_LIMIT_NAME,
    WINDOW_COVER_COMMAND_CLEAR_OPEN_LIMIT,
    WINDOW_COVER_COMMAND_CLEAR_OPEN_LIMIT_NAME,
    WINDOW_COVER_COMMAND_DOWNCLOSE,
    WINDOW_COVER_COMMAND_LIFTPERCENT,
    WINDOW_COVER_COMMAND_SET_CLOSE_LIMIT,
    WINDOW_COVER_COMMAND_SET_CLOSE_LIMIT_NAME,
    WINDOW_COVER_COMMAND_SET_OPEN_LIMIT,
    WINDOW_COVER_COMMAND_SET_OPEN_LIMIT_NAME,
    WINDOW_COVER_COMMAND_SMALL_STEP_CLOSE,
    WINDOW_COVER_COMMAND_SMALL_STEP_CLOSE_NAME,
    WINDOW_COVER_COMMAND_SMALL_STEP_OPEN,
    WINDOW_COVER_COMMAND_SMALL_STEP_OPEN_NAME,
    WINDOW_COVER_COMMAND_STOP,
    WINDOW_COVER_COMMAND_UPOPEN,
    DPToAttributeMapping as DpToAttributeMappingBase,
    NoManufacturerCluster,
    PowerOnState,
    TuyaCommand,
    TuyaData,
    TuyaDatapointData,
    TuyaLocalCluster,
    TuyaNewManufCluster,
    TuyaTimePayload,
)

_LOGGER = logging.getLogger(__name__)

# New manufacturer attributes
ATTR_MCU_VERSION = 0xEF00

# manufacturer commands
TUYA_MCU_CONNECTION_STATUS = 0x25

_LOGGER = logging.getLogger(__name__)


class DPToAttributeMapping(DpToAttributeMappingBase):
    """Container for datapoint to cluster attribute update mapping."""

    def __init__(
        self,
        ep_attribute: str,
        attribute_name: str | tuple[str, ...],
        converter: Callable[[Any], Any] | None = None,
        dp_converter: Callable[[Any], Any] | None = None,
        endpoint_id: int | None = None,
    ):
        """Init method for compatibility with previous quirks using positional arguments."""
        super().__init__(ep_attribute, attribute_name, converter, endpoint_id)
        self.dp_converter = dp_converter
        if dp_converter:
            _LOGGER.debug(
                "DPToAttributeMapping with dp_converter is deprecated, use TuyaQuirkBuilder "
                "(or TuyaMCUCluster.attributes_to_dp_converters) instead. attribute_name: %s",
                attribute_name,
            )


class CommandToDPValueResolver(Protocol):
    """Protocol describing CommandToDPValueMapping callbacks."""

    def __call__(
        self,
        command_id: Union[foundation.GeneralCommand, int, t.uint8_t],
        *args,
        **kwargs: Any,
    ) -> TuyaData:
        """Call back with self, command id, ordered and named variable args."""


@dataclasses.dataclass
class CommandToDPValueMapping:
    """Container for zigbee command id to datapoint id & value mappings.

    TuyaCommandCluster keeps a dictionary of how command ids map to a data point id and data point
    value as specified in this class.
    """

    dp: t.uint8_t
    value_resolver: CommandToDPValueResolver


class TuyaClusterData(t.Struct):
    """Tuya cluster data."""

    endpoint_id: int
    cluster_name: str
    cluster_attr: str
    attr_value: int  # Maybe also others types?
    expect_reply: bool
    manufacturer: int | UndefinedType | None


class MoesBacklight(t.enum8):
    """MOES switch backlight mode enum."""

    off = 0x00
    light_when_on = 0x01
    light_when_off = 0x02
    freeze = 0x03


class CoverCommandStepDirection(t.enum8):
    """Window cover step command direction enum."""

    Open = 0
    Close = 1


class CoverMotorCommand(t.enum8):
    """Window cover motor command states enum."""

    Open = 0
    Stop = 1
    Close = 2


class CoverMotorStatus(t.enum8):
    """Window cover motor states enum.

    Uses the same Tuya data point to send a command and receive the status, so needs the same
    values as CoverMotorCommand.
    """

    Opening = 0
    Stopped = 1
    Closing = 2


class CoverSettingMotorDirection(t.enum8):
    """Window cover motor direction configuration enum."""

    Forward = 0
    Backward = 1


class CoverSettingLimitOperation(t.enum8):
    """Window cover limits item to set / clear."""

    SetOpen = 0
    SetClose = 1
    ClearOpen = 2
    ClearClose = 3
    ClearBoth = 4


class TuyaPowerConfigurationCluster(
    TuyaLocalCluster, DoublingPowerConfigurationCluster
):
    """PowerConfiguration cluster for battery-operated tuya devices reporting percentage."""


class TuyaAttributesCluster(TuyaLocalCluster):
    """Manufacturer specific cluster for Tuya converting attributes <-> commands."""

    async def read_attributes(
        self,
        attributes: list[int | str | foundation.ZCLAttributeDef],
        **kwargs,
    ) -> Any:
        """Ignore remote reads as the "get_data" command doesn't seem to do anything."""

        self.debug("read_attributes --> attrs: %s", attributes)
        # Pop from kwargs to avoid duplicate keyword argument errors
        kwargs.pop("allow_cache", None)
        kwargs.pop("only_cache", None)
        return await super().read_attributes(
            attributes, allow_cache=True, only_cache=True, **kwargs
        )

    async def write_attributes(
        self,
        attributes: dict[str | int | foundation.ZCLAttributeDef, Any],
        manufacturer: int | UndefinedType | None = UNDEFINED,  # XXX: default in quirks
        **kwargs,
    ) -> list[list[foundation.WriteAttributesStatusRecord]]:
        """Defer attributes writing to the set_data tuya command."""

        await super().write_attributes(attributes, manufacturer=manufacturer, **kwargs)

        records = self._write_attr_records(attributes)

        for record in records:
            self.debug("write_attributes --> record: %s", record)

            cluster_data = TuyaClusterData(
                endpoint_id=self.endpoint.endpoint_id,
                cluster_name=self.ep_attribute,
                cluster_attr=self.attributes[record.attrid].name,
                attr_value=record.value.value,
                expect_reply=False,
                manufacturer=manufacturer,
            )
            self.endpoint.device.command_bus.listener_event(
                TUYA_MCU_COMMAND,
                cluster_data,
            )

        return [[foundation.WriteAttributesStatusRecord(foundation.Status.SUCCESS)]]


class TuyaCommandCluster(CustomCluster):
    """A tuya-based cluster that accepts zigbee commands and maps them to data point updates.

    Derived classed only need define a map and value converter to enable processing of commands
    into data point updates, sent to the tuya mcu cluster to send a set data command to the device.
    """

    command_to_dp: dict[
        Union[foundation.GeneralCommand, int, t.uint8_t], CommandToDPValueMapping
    ] = {}

    async def command(
        self,
        command_id: Union[foundation.GeneralCommand, int, t.uint8_t],
        *args,
        manufacturer: Optional[Union[int, t.uint16_t]] = None,
        expect_reply: bool = True,
        _tsn: Optional[Union[int, t.uint8_t]] = None,
        **kwargs: Any,
    ):
        """Process any commands that are mapped to data points."""
        _LOGGER.debug(
            "Processing command to dp mappings for Cluster Command. Command is %x, args=%s, kwargs=%s",
            command_id,
            args,
            kwargs,
        )

        # if there's a map for this command to a data point, call the map value function and send
        # the new value to the MCU cluster to send to the device
        command_map = self.command_to_dp.get(command_id, None)
        if command_map is not None:
            value = command_map.value_resolver(self, command_id, *args, **kwargs)

            self.send_tuya_set_datapoints_command(
                command_map.dp,
                value,
                expect_reply=expect_reply,
                manufacturer=manufacturer,
            )
            return self.default_response(command_id)

        _LOGGER.warning("Unsupported command_id: %s", command_id)
        return self.unsupported_response(command_id)

    def send_tuya_set_datapoints_command(
        self,
        dp: t.uint8_t,
        data: TuyaData,
        manufacturer: Optional[Union[int, t.uint16_t]] = None,
        expect_reply: bool = True,
    ):
        """Send a set_data for a Tuya data point value (via the mcu cluster)."""

        datapoints = [TuyaDatapointData(dp, data)]
        self.debug("Sending TUYA_MCU_SET_DATAPOINTS: %s", datapoints)

        self.endpoint.device.command_bus.listener_event(
            TUYA_MCU_SET_DATAPOINTS, datapoints, manufacturer, expect_reply
        )

    def default_response(
        self, command_id: Union[foundation.GeneralCommand, int, t.uint8_t]
    ):
        """Return a default success response for a given command."""

        return foundation.GENERAL_COMMANDS[
            foundation.GeneralCommand.Default_Response
        ].schema(command_id=command_id, status=foundation.Status.SUCCESS)

    def unsupported_response(
        self, command_id: Union[foundation.GeneralCommand, int, t.uint8_t]
    ):
        """Return an 'unsupported' response for a given command."""

        return foundation.GENERAL_COMMANDS[
            foundation.GeneralCommand.Default_Response
        ].schema(command_id=command_id, status=foundation.Status.UNSUP_CLUSTER_COMMAND)


class MCUVersion(t.Struct):
    """Tuya MCU version response Zcl payload."""

    status: t.uint8_t
    tsn: t.uint8_t
    version_raw: t.uint8_t

    @property
    def version(self) -> str:
        """Format the raw version to X.Y.Z."""

        if self.version_raw:
            # MCU version is 1 byte length
            # is converted from HEX -> BIN -> XX.XX.XXXX -> DEC (x.y.z)
            # example: 0x98 -> 10011000 -> 10.01.1000 -> 2.1.8
            # https://developer.tuya.com/en/docs/iot-device-dev/firmware-version-description?id=K9zzuc5n2gff8#title-1-Zigbee%20firmware%20versions
            major = self.version_raw >> 6
            minor = (self.version_raw & 63) >> 4
            release = self.version_raw & 15

            return f"{major}.{minor}.{release}"

        return None


class TuyaConnectionStatus(t.Struct):
    """Tuya connection status data."""

    tsn: t.uint8_t
    status: t.LVBytes


class TuyaMCUCluster(TuyaAttributesCluster, TuyaNewManufCluster):
    """Manufacturer specific cluster for sending Tuya MCU commands."""

    attributes_to_dp_converters: dict[int, Callable[[Any], Any]] = {}
    set_time_offset = datetime.datetime(1970, 1, 1, tzinfo=datetime.UTC)
    set_time_local_offset = datetime.datetime(1970, 1, 1)

    # TODO: Backwards compatibility, remove
    MCUVersion = MCUVersion
    TuyaConnectionStatus = TuyaConnectionStatus

    class AttributeDefs(TuyaNewManufCluster.AttributeDefs):
        """Attribute Definitions."""

        mcu_version = foundation.ZCLAttributeDef(
            id=ATTR_MCU_VERSION,
            type=t.uint48_t,
            access=foundation.ZCLAttributeAccess.Read,
            is_manufacturer_specific=True,
        )

    class ClientCommandDefs(TuyaNewManufCluster.ClientCommandDefs):
        """Client command definitions."""

        mcu_version_response = foundation.ZCLCommandDef(
            id=TUYA_MCU_VERSION_RSP,
            schema={"version": MCUVersion},
            is_manufacturer_specific=True,
        )
        mcu_connection_status = foundation.ZCLCommandDef(
            id=TUYA_MCU_CONNECTION_STATUS,
            schema={"payload": TuyaConnectionStatus},
            is_manufacturer_specific=True,
        )

    class ServerCommandDefs(TuyaNewManufCluster.ServerCommandDefs):
        """Server command definitions."""

        mcu_connection_status_rsp = foundation.ZCLCommandDef(
            id=TUYA_MCU_CONNECTION_STATUS,
            schema={"payload": TuyaConnectionStatus},
            is_manufacturer_specific=True,
        )

    def __init__(self, *args, **kwargs):
        """Init."""
        super().__init__(*args, **kwargs)

        self._attributes_to_dp_converters: dict[int, Callable[[Any], Any]]
        if self.attributes_to_dp_converters:
            self._attributes_to_dp_converters = self.attributes_to_dp_converters
        else:
            # convert from legacy DP2AttributeMapping with attribute_name tuple to new
            # DP2AttributeMapping with single attribute_name
            self._attributes_to_dp_converters = {}
            for dp, mappings in self.dp_to_attribute.items():
                if not isinstance(mappings, list):
                    mappings = [mappings]
                for dp_mapping in mappings:
                    # DPToAttributeMapping from the base Tuya module doesn't have `dp_converter`
                    # only the MCU DPToAttributeMapping has dp_converter, so check hasattr before
                    if hasattr(dp_mapping, "dp_converter") and dp_mapping.dp_converter:
                        self._attributes_to_dp_converters[dp] = dp_mapping.dp_converter

        # Cluster for endpoint: 1 (listen MCU commands)
        self.endpoint.device.command_bus = Bus()
        self.endpoint.device.command_bus.add_listener(self)

    def from_cluster_data(self, data: TuyaClusterData) -> list[TuyaCommand]:
        """Convert from cluster data to a tuya data payload."""

        dp_mapping = self.get_dp_mapping(data.endpoint_id, data.cluster_attr)
        self.debug("from_cluster_data: %s", dp_mapping)
        if len(dp_mapping) == 0:
            self.warning(
                "No cluster_dp found for %s, %s",
                data.endpoint_id,
                data.cluster_attr,
            )
            return []

        tuya_commands: list[TuyaCommand] = []
        for dp in dp_mapping:
            val = data.attr_value

            if attr_to_dp_converter := self._attributes_to_dp_converters.get(dp):
                args = []
                for dp_attr in self._dp_to_attributes[dp]:
                    if dp_attr.attribute_name == data.cluster_attr:
                        args.append(val)
                        continue
                    endpoint = self.endpoint
                    if dp_attr.endpoint_id:
                        endpoint = endpoint.device.endpoints[dp_attr.endpoint_id]
                    cluster = getattr(endpoint, dp_attr.ep_attribute)
                    args.append(cluster.get(dp_attr.attribute_name))
                val = attr_to_dp_converter(*args)
            self.debug("value: %s", val)

            dpd = TuyaDatapointData(dp, val)
            self.debug("raw: %s", dpd.data.raw)

            tuya_commands.append(
                TuyaCommand(
                    status=0,
                    tsn=self.endpoint.device.application.get_sequence(),
                    datapoints=[dpd],
                )
            )

        return tuya_commands

    def tuya_mcu_command(self, cluster_data: TuyaClusterData):
        """Tuya MCU command listener to send/set tuya data points from cluster attributes.

        Only manufacturer endpoint must listen to MCU commands.
        """

        self.debug(
            "tuya_mcu_command: cluster_data=%s",
            cluster_data,
        )

        tuya_commands = self.from_cluster_data(cluster_data)
        self.debug("tuya_commands: %s", tuya_commands)
        if len(tuya_commands) == 0:
            self.warning(
                "no MCU command for data %s",
                cluster_data,
            )
            return

        for tuya_command in tuya_commands:
            self.create_catching_task(
                self.command(
                    self.mcu_write_command,
                    tuya_command,
                    expect_reply=cluster_data.expect_reply,
                    manufacturer=cluster_data.manufacturer,
                )
            )

        endpoint = self.endpoint.device.endpoints[cluster_data.endpoint_id]
        cluster = getattr(endpoint, cluster_data.cluster_name)
        cluster.update_attribute(cluster_data.cluster_attr, cluster_data.attr_value)

    def tuya_mcu_set_datapoints(
        self,
        datapoints: list[TuyaDatapointData],
        manufacturer: Optional[Union[int, t.uint16_t]] = None,
        expect_reply: bool = True,
    ):
        """Tuya MCU listener to send/set tuya datapoint values.

        (Using DP values explicitly provided, usually from command handlers, rather than
        translating from cluster attributes as tuya_mcu_command does.)
        """

        self.debug("tuya_mcu_set_datapoints: datapoints=%s", datapoints)

        if len(datapoints) == 0:
            self.warning("no datapoints for tuya_mcu_set_datapoints")
            return

        cmd_payload = TuyaCommand()
        cmd_payload.status = 0
        cmd_payload.tsn = self.endpoint.device.application.get_sequence()
        cmd_payload.datapoints = datapoints

        self.create_catching_task(
            self.command(
                TUYA_SET_DATA,
                cmd_payload,
                manufacturer=manufacturer,
                expect_reply=expect_reply,
            )
        )

    def get_dp_mapping(
        self, endpoint_id: int, attribute_name: str
    ) -> dict[int, DPToAttributeMapping]:
        """Search for the DP in _dp_to_attributes."""

        result: dict[int, DPToAttributeMapping] = {}
        for dp, dp_mapping in self._dp_to_attributes.items():
            for mapped_attr in dp_mapping:
                if attribute_name != mapped_attr.attribute_name:
                    continue
                if not (
                    (
                        mapped_attr.endpoint_id is None
                        and endpoint_id == self.endpoint.endpoint_id
                    )
                    or (endpoint_id == mapped_attr.endpoint_id)
                ):
                    continue
                self.debug("get_dp_mapping --> found DP: %s", dp)
                result[dp] = mapped_attr
                break

        return result

    def handle_mcu_version_response(self, payload: MCUVersion) -> foundation.Status:  # type:ignore[valid-type]
        """Handle MCU version response."""

        self.debug("MCU version: %s", payload.version)
        self.update_attribute("mcu_version", payload.version)
        return foundation.Status.SUCCESS

    def handle_set_time_request(self, payload: t.uint16_t) -> foundation.Status:
        """Handle set_time requests (0x24)."""

        self.debug("handle_set_time_request payload: %s", payload)
        payload_rsp = TuyaTimePayload()

        utc_timestamp = int(
            (datetime.datetime.now(datetime.UTC) - self.set_time_offset).total_seconds()
        )
        local_timestamp = int(
            (datetime.datetime.now() - self.set_time_local_offset).total_seconds()
        )

        payload_rsp.extend(utc_timestamp.to_bytes(4, "big", signed=False))
        payload_rsp.extend(local_timestamp.to_bytes(4, "big", signed=False))

        self.debug("handle_set_time_request response: %s", payload_rsp)
        self.create_catching_task(
            super().command(TUYA_SET_TIME, payload_rsp, expect_reply=False)
        )

        return foundation.Status.SUCCESS

    def handle_mcu_connection_status(
        self,
        payload: TuyaConnectionStatus,  # type:ignore[valid-type]
    ) -> foundation.Status:
        """Handle gateway connection status requests (0x25)."""

        payload_rsp = TuyaMCUCluster.TuyaConnectionStatus()
        payload_rsp.tsn = payload.tsn
        payload_rsp.status = b"\x01"  # 0x00 not connected to internet | 0x01 connected to internet | 0x02 time out

        self.create_catching_task(
            super().command(TUYA_MCU_CONNECTION_STATUS, payload_rsp, expect_reply=False)
        )

        return foundation.Status.SUCCESS


class TuyaOnOff(OnOff, TuyaLocalCluster):
    """Tuya MCU OnOff cluster."""

    class AttributeDefs(OnOff.AttributeDefs):
        """Cluster attributes."""

    class ServerCommandDefs(OnOff.ServerCommandDefs):
        """Server command definitions."""

    async def command(
        self,
        command_id: foundation.GeneralCommand | int | t.uint8_t,
        *args,
        manufacturer: int | t.uint16_t | None = None,
        expect_reply: bool = True,
        tsn: int | t.uint8_t | None = None,
        **kwargs: Any,
    ):
        """Override the default Cluster command."""

        self.debug(
            "Sending Tuya Cluster Command... Cluster Command is %x, Arguments are %s",
            command_id,
            args,
        )

        # (off, on)
        if command_id in (0x0000, 0x0001):
            cluster_data = TuyaClusterData(
                endpoint_id=self.endpoint.endpoint_id,
                cluster_name=self.ep_attribute,
                cluster_attr="on_off",
                attr_value=bool(command_id),
                expect_reply=expect_reply,
                manufacturer=manufacturer,
            )
            self.endpoint.device.command_bus.listener_event(
                TUYA_MCU_COMMAND,
                cluster_data,
            )
            return foundation.GENERAL_COMMANDS[
                foundation.GeneralCommand.Default_Response
            ].schema(command_id=command_id, status=foundation.Status.SUCCESS)

        self.warning("Unsupported command_id: %s", command_id)
        return foundation.GENERAL_COMMANDS[
            foundation.GeneralCommand.Default_Response
        ].schema(command_id=command_id, status=foundation.Status.UNSUP_CLUSTER_COMMAND)


class TuyaOnOffNM(NoManufacturerCluster, TuyaOnOff):
    """Tuya OnOff cluster with NoManufacturerID."""


class TuyaCoverControl(t.enum8):
    """Tuya cover control command values."""

    Open = 0x00
    Stop = 0x01
    Close = 0x02


class TuyaWindowCovering(WindowCovering, TuyaLocalCluster):
    """Tuya MCU WindowCovering cluster."""

    class AttributeDefs(WindowCovering.AttributeDefs):
        """Attribute definitions."""

        tuya_cover_command: Final = ZCLAttributeDef(
            id=0xEF01, type=TuyaCoverControl, is_manufacturer_specific=True
        )

    async def command(
        self,
        command_id: foundation.GeneralCommand | int | t.uint8_t,
        *args,
        manufacturer: int | t.uint16_t | None = None,
        expect_reply: bool = True,
        tsn: int | t.uint8_t | None = None,
        **kwargs: Any,
    ):
        """Override the default Cluster command."""

        self.debug(
            "Sending Tuya Cluster Command... Cluster Command is %x, Arguments are %s",
            command_id,
            args,
        )

        # up_open
        if command_id == WindowCovering.ServerCommandDefs.up_open.id:
            cluster_data = TuyaClusterData(
                endpoint_id=self.endpoint.endpoint_id,
                cluster_name=self.ep_attribute,
                cluster_attr=self.AttributeDefs.tuya_cover_command.name,
                attr_value=TuyaCoverControl.Open,
                expect_reply=expect_reply,
                manufacturer=manufacturer,
            )
            self.endpoint.device.command_bus.listener_event(
                TUYA_MCU_COMMAND,
                cluster_data,
            )
            return foundation.GENERAL_COMMANDS[
                foundation.GeneralCommand.Default_Response
            ].schema(command_id=command_id, status=foundation.Status.SUCCESS)

        # down_close
        if command_id == WindowCovering.ServerCommandDefs.down_close.id:
            cluster_data = TuyaClusterData(
                endpoint_id=self.endpoint.endpoint_id,
                cluster_name=self.ep_attribute,
                cluster_attr=self.AttributeDefs.tuya_cover_command.name,
                attr_value=TuyaCoverControl.Close,
                expect_reply=expect_reply,
                manufacturer=manufacturer,
            )
            self.endpoint.device.command_bus.listener_event(
                TUYA_MCU_COMMAND,
                cluster_data,
            )
            return foundation.GENERAL_COMMANDS[
                foundation.GeneralCommand.Default_Response
            ].schema(command_id=command_id, status=foundation.Status.SUCCESS)

        # stop
        if command_id == WindowCovering.ServerCommandDefs.stop.id:
            cluster_data = TuyaClusterData(
                endpoint_id=self.endpoint.endpoint_id,
                cluster_name=self.ep_attribute,
                cluster_attr=self.AttributeDefs.tuya_cover_command.name,
                attr_value=TuyaCoverControl.Stop,
                expect_reply=expect_reply,
                manufacturer=manufacturer,
            )
            self.endpoint.device.command_bus.listener_event(
                TUYA_MCU_COMMAND,
                cluster_data,
            )
            return foundation.GENERAL_COMMANDS[
                foundation.GeneralCommand.Default_Response
            ].schema(command_id=command_id, status=foundation.Status.SUCCESS)

        # go_to_lift_percentage
        if command_id == WindowCovering.ServerCommandDefs.go_to_lift_percentage.id:
            cluster_data = TuyaClusterData(
                endpoint_id=self.endpoint.endpoint_id,
                cluster_name=self.ep_attribute,
                cluster_attr=WindowCovering.AttributeDefs.current_position_lift_percentage.name,
                attr_value=args[0],
                expect_reply=expect_reply,
                manufacturer=manufacturer,
            )
            self.endpoint.device.command_bus.listener_event(
                TUYA_MCU_COMMAND,
                cluster_data,
            )
            return foundation.GENERAL_COMMANDS[
                foundation.GeneralCommand.Default_Response
            ].schema(command_id=command_id, status=foundation.Status.SUCCESS)

        self.warning("Unsupported command_id: %s", command_id)
        return foundation.GENERAL_COMMANDS[
            foundation.GeneralCommand.Default_Response
        ].schema(command_id=command_id, status=foundation.Status.UNSUP_CLUSTER_COMMAND)


class TuyaOnOffManufCluster(TuyaMCUCluster):
    """Tuya with On/Off data points."""

    dp_to_attribute: dict[int, DPToAttributeMapping] = {
        1: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
        ),
        2: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=2,
        ),
        3: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=3,
        ),
        4: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=4,
        ),
        5: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=5,
        ),
        6: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=6,
        ),
        0x65: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=7,
        ),
        0x66: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=8,
        ),
        0x67: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=9,
        ),
        0x68: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=10,
        ),
        0x69: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=11,
        ),
        0x6A: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=12,
        ),
        0x6B: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=13,
        ),
        0x6C: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=14,
        ),
        0x6D: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=15,
        ),
        0x6E: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=16,
        ),
    }

    data_point_handlers = {
        1: "_dp_2_attr_update",
        2: "_dp_2_attr_update",
        3: "_dp_2_attr_update",
        4: "_dp_2_attr_update",
        5: "_dp_2_attr_update",
        6: "_dp_2_attr_update",
        0x65: "_dp_2_attr_update",
        0x66: "_dp_2_attr_update",
        0x67: "_dp_2_attr_update",
        0x68: "_dp_2_attr_update",
        0x69: "_dp_2_attr_update",
        0x6A: "_dp_2_attr_update",
        0x6B: "_dp_2_attr_update",
        0x6C: "_dp_2_attr_update",
        0x6D: "_dp_2_attr_update",
        0x6E: "_dp_2_attr_update",
    }


class MoesSwitchManufCluster(TuyaOnOffManufCluster):
    """On/Off Tuya cluster with extra device attributes."""

    class AttributeDefs(TuyaOnOffManufCluster.AttributeDefs):
        """Attribute definitions."""

        backlight_mode: Final = ZCLAttributeDef(id=0x8001, type=MoesBacklight)
        power_on_state: Final = ZCLAttributeDef(id=0x8002, type=PowerOnState)

    dp_to_attribute: dict[int, DPToAttributeMapping] = (
        TuyaOnOffManufCluster.dp_to_attribute.copy()
    )
    dp_to_attribute.update(
        {
            14: DPToAttributeMapping(
                TuyaMCUCluster.ep_attribute,
                "power_on_state",
                converter=lambda x: PowerOnState(x),
            )
        }
    )
    dp_to_attribute.update(
        {
            15: DPToAttributeMapping(
                TuyaMCUCluster.ep_attribute,
                "backlight_mode",
                converter=lambda x: MoesBacklight(x),
            ),
        }
    )

    data_point_handlers = TuyaOnOffManufCluster.data_point_handlers.copy()
    data_point_handlers.update({14: "_dp_2_attr_update"})
    data_point_handlers.update({15: "_dp_2_attr_update"})


class TuyaWindowCoverControlV2(
    TuyaAttributesCluster, TuyaCommandCluster, WindowCovering
):
    """Tuya Window Cover Cluster v2.

    Replacement for TuyaWindowCoverControl, supporting extra attributes & commands and compatible
    with TuyaMCUCluster & TuyaWindowCoverManufClusterV2 which can handle multiple dp updates
    in one zigby frame.

    Derived from TuyaAttributesCluster & TuyaCommandCluster to support mapping from zigbee
    attributes & commands to tuya data points respectively.
    """

    attributes = WindowCovering.attributes.copy()
    attributes.update(
        {
            # main motor status attribute is logically write-only, only used by commands and not
            # very useful to HA, but it's returned in the set_data and set_data_response packets
            # so I've mapped it to an attribute.
            ATTR_COVER_MOTOR_STATUS: (ATTR_COVER_MOTOR_STATUS_NAME, t.enum8),
            ATTR_COVER_INVERTED: (
                ATTR_COVER_INVERTED_NAME,
                t.Bool,
            ),
            ATTR_COVER_DIRECTION: (
                ATTR_COVER_DIRECTION_NAME,
                CoverSettingMotorDirection,
            ),
        }
    )

    server_commands = WindowCovering.server_commands.copy()
    server_commands.update(
        {
            WINDOW_COVER_COMMAND_SMALL_STEP_OPEN: foundation.ZCLCommandDef(
                WINDOW_COVER_COMMAND_SMALL_STEP_OPEN,
                {},
                is_manufacturer_specific=True,
                name=WINDOW_COVER_COMMAND_SMALL_STEP_OPEN_NAME,
            ),
            WINDOW_COVER_COMMAND_SMALL_STEP_CLOSE: foundation.ZCLCommandDef(
                WINDOW_COVER_COMMAND_SMALL_STEP_CLOSE,
                {},
                is_manufacturer_specific=True,
                name=WINDOW_COVER_COMMAND_SMALL_STEP_CLOSE_NAME,
            ),
            WINDOW_COVER_COMMAND_SET_OPEN_LIMIT: foundation.ZCLCommandDef(
                WINDOW_COVER_COMMAND_SET_OPEN_LIMIT,
                {},
                is_manufacturer_specific=True,
                name=WINDOW_COVER_COMMAND_SET_OPEN_LIMIT_NAME,
            ),
            WINDOW_COVER_COMMAND_SET_CLOSE_LIMIT: foundation.ZCLCommandDef(
                WINDOW_COVER_COMMAND_SET_CLOSE_LIMIT,
                {},
                is_manufacturer_specific=True,
                name=WINDOW_COVER_COMMAND_SET_CLOSE_LIMIT_NAME,
            ),
            WINDOW_COVER_COMMAND_CLEAR_OPEN_LIMIT: foundation.ZCLCommandDef(
                WINDOW_COVER_COMMAND_CLEAR_OPEN_LIMIT,
                {},
                is_manufacturer_specific=True,
                name=WINDOW_COVER_COMMAND_CLEAR_OPEN_LIMIT_NAME,
            ),
            WINDOW_COVER_COMMAND_CLEAR_CLOSE_LIMIT: foundation.ZCLCommandDef(
                WINDOW_COVER_COMMAND_CLEAR_CLOSE_LIMIT,
                {},
                is_manufacturer_specific=True,
                name=WINDOW_COVER_COMMAND_CLEAR_CLOSE_LIMIT_NAME,
            ),
            WINDOW_COVER_COMMAND_CLEAR_BOTH_LIMITS: foundation.ZCLCommandDef(
                WINDOW_COVER_COMMAND_CLEAR_BOTH_LIMITS,
                {},
                is_manufacturer_specific=True,
                name=WINDOW_COVER_COMMAND_CLEAR_BOTH_LIMITS_NAME,
            ),
        }
    )

    # Translate from zigbee move command ids to tuya dp values.
    # For most tuya devices Up/Open = 0, Stop = 1, Down/Close = 2
    tuya_cover_command_to_dp_values = {
        WINDOW_COVER_COMMAND_UPOPEN: 0x0000,
        WINDOW_COVER_COMMAND_DOWNCLOSE: 0x0002,
        WINDOW_COVER_COMMAND_STOP: 0x0001,
    }

    def update_lift_percent(self, raw_value: int):
        """Update lift percent attribute when it's data point data is received.

        This can't be done as a builder converter/dp_converter lambda because it needs access
        to self which those callbacks don't have, but methods like this, registered as builder
        dp_handlers do.
        """

        new_attribute_value = self._compute_lift_percent(raw_value)
        self.update_attribute(ATTR_COVER_POSITION_NAME, new_attribute_value)

    def _compute_lift_percent(self, input_value: int):
        """Convert/invert lift percent when needed.

        HA shows % open. The zigbee cluster value is called 'lift_percent' but seems to need to
        be % closed. This logic follows the convention of other Tuya covers, inverting the value
        by default, unless the cluster invert attribute is set. (This seems strange to me, but it's
        better to be consistent.)

        It's safe to use the same calculation converting motor position to zigbee attribute value
        and attribute value to motor position command.
        """

        invert = self._attr_cache.get(ATTR_COVER_INVERTED) == 1
        return input_value if invert else 100 - input_value

    command_to_dp: dict[
        Union[foundation.GeneralCommand, int, t.uint8_t], CommandToDPValueMapping
    ] = {
        WINDOW_COVER_COMMAND_UPOPEN: CommandToDPValueMapping(
            TUYA_DP_ID_CONTROL,
            lambda self, command: CoverMotorCommand(
                self.tuya_cover_command_to_dp_values[command]
            ),
        ),
        WINDOW_COVER_COMMAND_DOWNCLOSE: CommandToDPValueMapping(
            TUYA_DP_ID_CONTROL,
            lambda self, command: CoverMotorCommand(
                self.tuya_cover_command_to_dp_values[command]
            ),
        ),
        WINDOW_COVER_COMMAND_STOP: CommandToDPValueMapping(
            TUYA_DP_ID_CONTROL,
            lambda self, command: CoverMotorCommand(
                self.tuya_cover_command_to_dp_values[command]
            ),
        ),
        WINDOW_COVER_COMMAND_LIFTPERCENT: CommandToDPValueMapping(
            TUYA_DP_ID_PERCENT_CONTROL,
            lambda self, command, *args: self._compute_lift_percent(args[0]),
        ),
        WINDOW_COVER_COMMAND_SMALL_STEP_OPEN: CommandToDPValueMapping(
            TUYA_DP_ID_SMALL_STEP,
            lambda self, command: CoverCommandStepDirection.Open,
        ),
        WINDOW_COVER_COMMAND_SMALL_STEP_CLOSE: CommandToDPValueMapping(
            TUYA_DP_ID_SMALL_STEP,
            lambda self, command: CoverCommandStepDirection.Close,
        ),
        WINDOW_COVER_COMMAND_SET_OPEN_LIMIT: CommandToDPValueMapping(
            TUYA_DP_ID_LIMIT_SETTINGS,
            lambda self, command: CoverSettingLimitOperation.SetOpen,
        ),
        WINDOW_COVER_COMMAND_SET_CLOSE_LIMIT: CommandToDPValueMapping(
            TUYA_DP_ID_LIMIT_SETTINGS,
            lambda self, command: CoverSettingLimitOperation.SetClose,
        ),
        WINDOW_COVER_COMMAND_CLEAR_OPEN_LIMIT: CommandToDPValueMapping(
            TUYA_DP_ID_LIMIT_SETTINGS,
            lambda self, command: CoverSettingLimitOperation.ClearOpen,
        ),
        WINDOW_COVER_COMMAND_CLEAR_CLOSE_LIMIT: CommandToDPValueMapping(
            TUYA_DP_ID_LIMIT_SETTINGS,
            lambda self, command: CoverSettingLimitOperation.ClearClose,
        ),
        WINDOW_COVER_COMMAND_CLEAR_BOTH_LIMITS: CommandToDPValueMapping(
            TUYA_DP_ID_LIMIT_SETTINGS,
            lambda self, command: CoverSettingLimitOperation.ClearBoth,
        ),
    }


class TuyaWindowCoverManufClusterV2(TuyaMCUCluster):
    """Manufacturer Specific Cluster for cover device v2.

    Uses newer TuyaMCUCluster to handle multiple dp updates from one zigby frame.
    """

    def update_lift_percent(self, datapoint: TuyaDatapointData):
        """Update lift percent attribute when it's data point data is received.

        This can't be done as a builder converter/dp_converter lambda because it needs access
        to self which those callbacks don't have, but methods like this, registered as builder
        dp_handlers do.
        """
        cluster = self.endpoint.window_covering
        cluster.update_lift_percent(datapoint.data.payload)

    def ignore_update(self, _datapoint: TuyaDatapointData) -> None:
        """Process (and ignore) some data point updates."""
        return None


class TuyaLevelControl(LevelControl, TuyaLocalCluster):
    """Tuya MCU Level cluster for dimmable device."""

    class AttributeDefs(LevelControl.AttributeDefs):
        """Cluster attributes."""

    async def command(
        self,
        command_id: foundation.GeneralCommand | int | t.uint8_t,
        *args,
        manufacturer: int | t.uint16_t | None = None,
        expect_reply: bool = True,
        tsn: int | t.uint8_t | None = None,
        **kwargs: Any,
    ):
        """Override the default Cluster command."""
        self.debug(
            "Sending Tuya Cluster Command. Cluster Command is %x, Arguments are %s, %s",
            command_id,
            args,
            kwargs,
        )

        # getting the level value
        if kwargs and "level" in kwargs:
            level = kwargs["level"]
        elif args:
            level = args[0]
        else:
            level = 0

        on_off = bool(level)  # maybe must be compared against `minimum_level` attribute

        # (move_to_level_with_on_off --> send the on_off command first, but only if needed)
        if command_id == 0x0004 and self.endpoint.on_off.get("on_off") != on_off:
            cluster_data = TuyaClusterData(
                endpoint_id=self.endpoint.endpoint_id,
                cluster_name="on_off",
                cluster_attr="on_off",
                attr_value=on_off,
                expect_reply=expect_reply,
                manufacturer=manufacturer,
            )
            self.endpoint.device.command_bus.listener_event(
                TUYA_MCU_COMMAND,
                cluster_data,
            )

        # level 0 --> switched off
        if command_id == 0x0004 and not on_off:
            return foundation.GENERAL_COMMANDS[
                foundation.GeneralCommand.Default_Response
            ].schema(command_id=command_id, status=foundation.Status.SUCCESS)

        # (move_to_level, move, move_to_level_with_on_off)
        if command_id in (0x0000, 0x0001, 0x0004):
            cluster_data = TuyaClusterData(
                endpoint_id=self.endpoint.endpoint_id,
                cluster_name=self.ep_attribute,
                cluster_attr="current_level",
                attr_value=level,
                expect_reply=expect_reply,
                manufacturer=manufacturer,
            )
            self.endpoint.device.command_bus.listener_event(
                TUYA_MCU_COMMAND,
                cluster_data,
            )
            return foundation.GENERAL_COMMANDS[
                foundation.GeneralCommand.Default_Response
            ].schema(command_id=command_id, status=foundation.Status.SUCCESS)

        self.warning("Unsupported command_id: %s", command_id)
        return foundation.GENERAL_COMMANDS[
            foundation.GeneralCommand.Default_Response
        ].schema(command_id=command_id, status=foundation.Status.UNSUP_CLUSTER_COMMAND)


class TuyaInWallLevelControl(TuyaAttributesCluster, TuyaLevelControl):
    """Tuya Level cluster for inwall dimmable device."""

    class AttributeDefs(TuyaLevelControl.AttributeDefs):
        """Attribute definitions."""

        minimum_level: Final = ZCLAttributeDef(
            id=0xEF01, type=t.uint32_t, is_manufacturer_specific=True
        )
        bulb_type: Final = ZCLAttributeDef(
            id=0xEF02, type=t.enum8, is_manufacturer_specific=True
        )


class TuyaLevelControlManufCluster(TuyaMCUCluster):
    """Tuya with Level Control data points."""

    dp_to_attribute: dict[int, DPToAttributeMapping] = {
        1: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
        ),
        2: DPToAttributeMapping(
            TuyaLevelControl.ep_attribute,
            "current_level",
            converter=lambda x: (x * 255) // 1000,
            dp_converter=lambda x: (x * 1000) // 255,
        ),
        3: DPToAttributeMapping(
            TuyaLevelControl.ep_attribute,
            "minimum_level",
            converter=lambda x: (x * 255) // 1000,
            dp_converter=lambda x: (x * 1000) // 255,
        ),
        4: DPToAttributeMapping(
            TuyaLevelControl.ep_attribute,
            "bulb_type",
        ),
        7: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=2,
        ),
        8: DPToAttributeMapping(
            TuyaLevelControl.ep_attribute,
            "current_level",
            converter=lambda x: (x * 255) // 1000,
            dp_converter=lambda x: (x * 1000) // 255,
            endpoint_id=2,
        ),
        9: DPToAttributeMapping(
            TuyaLevelControl.ep_attribute,
            "minimum_level",
            converter=lambda x: (x * 255) // 1000,
            dp_converter=lambda x: (x * 1000) // 255,
            endpoint_id=2,
        ),
        10: DPToAttributeMapping(
            TuyaLevelControl.ep_attribute,
            "bulb_type",
            endpoint_id=2,
        ),
        15: DPToAttributeMapping(
            TuyaOnOff.ep_attribute,
            "on_off",
            endpoint_id=3,
        ),
        16: DPToAttributeMapping(
            TuyaLevelControl.ep_attribute,
            "current_level",
            converter=lambda x: (x * 255) // 1000,
            dp_converter=lambda x: (x * 1000) // 255,
            endpoint_id=3,
        ),
        17: DPToAttributeMapping(
            TuyaLevelControl.ep_attribute,
            "minimum_level",
            converter=lambda x: (x * 255) // 1000,
            dp_converter=lambda x: (x * 1000) // 255,
            endpoint_id=3,
        ),
        18: DPToAttributeMapping(
            TuyaLevelControl.ep_attribute,
            "bulb_type",
            endpoint_id=3,
        ),
    }

    data_point_handlers = {
        1: "_dp_2_attr_update",
        2: "_dp_2_attr_update",
        3: "_dp_2_attr_update",
        4: "_dp_2_attr_update",
        7: "_dp_2_attr_update",
        8: "_dp_2_attr_update",
        9: "_dp_2_attr_update",
        10: "_dp_2_attr_update",
        15: "_dp_2_attr_update",
        16: "_dp_2_attr_update",
        17: "_dp_2_attr_update",
        18: "_dp_2_attr_update",
    }
