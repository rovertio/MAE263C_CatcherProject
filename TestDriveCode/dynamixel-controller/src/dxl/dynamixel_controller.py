import dataclasses
import json
import math
import warnings
from enum import IntEnum, Enum
from pathlib import Path

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
    DXL_LOBYTE,
    DXL_LOWORD,
    DXL_HIBYTE,
    DXL_HIWORD,
)


# Author: Hunter Halloran (Jyumpp)


class DynamixelModel(Enum):
    AX12 = "AX12"
    MX12 = "MX12"
    MX28 = "MX28"
    MX64 = "MX64"
    MX106 = "MX106"
    XM5400W270 = "XM5400W270"


class DynamixelMode(IntEnum):
    Position = 3
    Velocity = 1
    PWM = 16


class DynamixelCommunicationWarning(RuntimeWarning):
    pass


class DynamixelIO:
    """Creates communication handler for Dynamixe motors"""

    def __init__(self, device_name: str = "/dev/ttyUSB0", baud_rate: int = 57_600):
        if device_name is None:
            raise ValueError("`device_name` parameter cannot be None")

        self.port_handler = PortHandler(port_name=device_name)
        self.packet_handler = PacketHandler(protocol_version=2)

        if not self.port_handler.setBaudRate(baud_rate):
            raise RuntimeError(f"Failed to set baud rate to {baud_rate}")

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port on device named `{device_name}`")

    def check_error(self, dxl_comm_result, dxl_error):
        """Prints the error message when not successful"""
        if dxl_comm_result != COMM_SUCCESS:
            print(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"{self.packet_handler.getRxPacketError(dxl_error)}")


@dataclasses.dataclass(frozen=True)
class DynamixelModelInfo:
    control_table: dict[str, list[int]]
    max_angle_deg: float
    model: DynamixelModel

@dataclasses.dataclass
class DynamixelInfo:
    min_position: int
    max_position: int
    velocity_limit: int
    pwm_limit: int


def map_range(
    input_value: int | float,
    input_min: int | float,
    input_max: int | float,
    output_min: int | float,
    output_max: int | float,
) -> int | float:
    return (input_value - input_min) / (input_max - input_min) * (
        output_max - output_min
    ) + output_min


class DynamixelMotorGroup:
    def __init__(
        self,
        dxl_io: DynamixelIO,
        dynamixel_model_info: DynamixelModelInfo,
        *dynamixel_ids: tuple[int, ...],
    ):
        self.dxl_io = dxl_io
        self.model_info = dynamixel_model_info
        self.dynamixel_ids = tuple(*sorted(dynamixel_ids))

        self.motor_info: dict[int, DynamixelInfo] = {}

        min_positions = self._group_sync_read(
            self.model_info.control_table["Min_Position_Limit"][0],
            self.model_info.control_table["Min_Position_Limit"][1],
        )
        max_positions = self._group_sync_read(
            self.model_info.control_table["Max_Position_Limit"][0],
            self.model_info.control_table["Max_Position_Limit"][1],
        )
        velocity_limits = self._group_sync_read(
            self.model_info.control_table["Velocity_Limit"][0],
            self.model_info.control_table["Velocity_Limit"][1],
        )
        pwm_limits = self._group_sync_read(
            self.model_info.control_table["PWM_Limit"][0],
            self.model_info.control_table["PWM_Limit"][1],
        )

        for dynamixel_id in self.dynamixel_ids:
            self.motor_info[dynamixel_id] = DynamixelInfo(
                min_positions[dynamixel_id],
                max_positions[dynamixel_id],
                velocity_limits[dynamixel_id],
                pwm_limits[dynamixel_id],
            )

    def _group_sync_read(
        self, control_table_address: int, data_length: int
    ) -> dict[int, int]:

        group_sync_read = GroupSyncRead(
            port=self.dxl_io.port_handler,
            ph=self.dxl_io.packet_handler,
            start_address=control_table_address,
            data_length=data_length,
        )

        for dynamixel_id in self.dynamixel_ids:
            group_sync_read.addParam(dynamixel_id)

        comm_result = group_sync_read.txRxPacket()

        results = {}
        if comm_result != COMM_SUCCESS:
            warnings.warn(
                f"{self.dxl_io.packet_handler.getTxRxResult(comm_result)}",
                category=DynamixelCommunicationWarning,
            )
            return results

        for dynamixel_id in self.dynamixel_ids:
            if group_sync_read.isAvailable(
                dynamixel_id, control_table_address, data_length
            ):
                results[dynamixel_id] = group_sync_read.getData(
                    dynamixel_id, control_table_address, data_length
                )

        return results

    def _group_sync_write(
        self,
        data_for_dynamixel_id: dict[int, int],
        control_table_address: int,
        data_length: int,
    ):

        group_sync_write = GroupSyncWrite(
            port=self.dxl_io.port_handler,
            ph=self.dxl_io.packet_handler,
            start_address=control_table_address,
            data_length=data_length,
        )

        for dxl_id, datum in data_for_dynamixel_id.items():
            if data_length == 1:
                datum = [DXL_LOBYTE(DXL_LOWORD(data_for_dynamixel_id[dxl_id]))]
            elif data_length == 2:
                datum = [
                    DXL_LOBYTE(DXL_LOWORD(data_for_dynamixel_id[dxl_id])),
                    DXL_HIBYTE(DXL_LOWORD(data_for_dynamixel_id[dxl_id])),
                ]
            else:
                datum = [
                    DXL_LOBYTE(DXL_LOWORD(data_for_dynamixel_id[dxl_id])),
                    DXL_HIBYTE(DXL_LOWORD(data_for_dynamixel_id[dxl_id])),
                    DXL_LOBYTE(DXL_HIWORD(data_for_dynamixel_id[dxl_id])),
                    DXL_HIBYTE(DXL_HIWORD(data_for_dynamixel_id[dxl_id])),
                ]
            group_sync_write.addParam(dxl_id, datum)

        comm_result = group_sync_write.txPacket()

        if comm_result != COMM_SUCCESS:
            warnings.warn(
                f"{self.dxl_io.packet_handler.getTxRxResult(comm_result)}",
                category=DynamixelCommunicationWarning,
            )

    @property
    def position(self) -> dict[int, int]:
        return self._group_sync_read(
            control_table_address=self.model_info.control_table["Present_Position"][0],
            data_length=self.model_info.control_table["Present_Position"][1],
        )

    @position.setter
    def position(self, position_for_dynamixel_id: dict[int, int]):
        self._group_sync_write(
            data_for_dynamixel_id=position_for_dynamixel_id,
            control_table_address=self.model_info.control_table["Goal_Position"][0],
            data_length=self.model_info.control_table["Goal_Position"][1],
        )

    @property
    def angle_deg(self) -> dict[int, float]:
        return {
            dynamixel_id: float(
                map_range(
                    position_ticks,
                    input_min=self.motor_info[dynamixel_id].min_position,
                    input_max=self.motor_info[dynamixel_id].max_position + 1,
                    output_min=0.0,
                    output_max=self.model_info.max_angle_deg,
                )
            )
            for dynamixel_id, position_ticks in self.position.items()
        }

    @angle_deg.setter
    def angle_deg(self, angle_deg_for_dynamixel_id: dict[int, float]):
        position_for_dynamixel_id = {
            dynamixel_id: int(
                map_range(
                    angle_deg,
                    input_min=0.0,
                    input_max=self.model_info.max_angle_deg,
                    output_min=self.motor_info[dynamixel_id].min_position,
                    output_max=self.motor_info[dynamixel_id].max_position + 1,
                )
            )
            for dynamixel_id, angle_deg in angle_deg_for_dynamixel_id.items()
        }

        self.position = position_for_dynamixel_id

    @property
    def angle_rad(self) -> dict[int, float]:
        return {
            dynamixel_id: math.radians(float(angle_deg))
            for dynamixel_id, angle_deg in self.angle_deg.items()
        }

    @angle_rad.setter
    def angle_rad(self, angle_rad_for_dynamixel_id: dict[int, float]):
        position_for_dynamixel_id = {
            dynamixel_id: int(
                map_range(
                    math.degrees(angle_rad),
                    input_min=0.0,
                    input_max=self.model_info.max_angle_deg,
                    output_min=self.motor_info[dynamixel_id].min_position,
                    output_max=self.motor_info[dynamixel_id].max_position + 1,
                )
            )
            for dynamixel_id, angle_rad in angle_rad_for_dynamixel_id.items()
        }

        self.position = position_for_dynamixel_id

    @property
    def velocity(self) -> dict[int, int]:
        return self._group_sync_read(
            control_table_address=self.model_info.control_table["Present_Velocity"][0],
            data_length=self.model_info.control_table["Present_Velocity"][1],
        )

    @velocity.setter
    def velocity(self, velocity_for_dynamixel_id: dict[int, int]):
        self._group_sync_write(
            data_for_dynamixel_id=velocity_for_dynamixel_id,
            control_table_address=self.model_info.control_table["Goal_Velocity"][0],
            data_length=self.model_info.control_table["Goal_Velocity"][1],
        )

    def velocity_rpm_per_tick_factor(self) -> float:
        rev_per_min_per_tick = 0.0
        if self.model_info.model is DynamixelModel.AX12:
            rev_per_min_per_tick = 0.111
        elif self.model_info.model is DynamixelModel.MX12:
            rev_per_min_per_tick = 0.916
        elif self.model_info.model is DynamixelModel.MX28:
            rev_per_min_per_tick = 0.229
        elif self.model_info.model is DynamixelModel.MX64:
            rev_per_min_per_tick = 0.229
        elif self.model_info.model is DynamixelModel.MX106:
            rev_per_min_per_tick = 0.229

        return rev_per_min_per_tick

    @property
    def velocity_deg_per_s(self) -> dict[int, float]:
        factor = self.velocity_rpm_per_tick_factor()
        factor *= 360.0 / 60.0
        return {
            dynamixel_id: float(
                map_range(
                    velocity_ticks,
                    input_min=-self.motor_info[dynamixel_id].velocity_limit,
                    input_max=self.motor_info[dynamixel_id].velocity_limit,
                    output_min=-factor * self.motor_info[dynamixel_id].velocity_limit,
                    output_max=factor * self.motor_info[dynamixel_id].velocity_limit,
                )
            )
            for dynamixel_id, velocity_ticks in self.velocity.items()
        }

    @velocity_deg_per_s.setter
    def velocity_deg_per_s(self, velocity_deg_per_s_for_dynamixel_id: dict[int, float]):
        factor = self.velocity_rpm_per_tick_factor()
        factor *= 360.0 / 60.0
        velocity_for_dynamixel_id = {
            dynamixel_id: int(
                map_range(
                    velocity_deg_per_s,
                    input_min=-factor * self.motor_info[dynamixel_id].velocity_limit,
                    input_max=factor * self.motor_info[dynamixel_id].velocity_limit,
                    output_min=-self.motor_info[dynamixel_id].velocity_limit,
                    output_max=self.motor_info[dynamixel_id].velocity_limit,
                )
            )
            for dynamixel_id, velocity_deg_per_s 
            in velocity_deg_per_s_for_dynamixel_id.items()
        }

        self.velocity = velocity_for_dynamixel_id

    @property
    def pwm(self) -> dict[int, int]:
        return self._group_sync_read(
            control_table_address=self.model_info.control_table["Present_PWM"][0],
            data_length=self.model_info.control_table["Present_PWM"][1],
        )

    @pwm.setter
    def pwm(self, pwm_for_dynamixel_id: dict[int, int]):
        self._group_sync_write(
            data_for_dynamixel_id=pwm_for_dynamixel_id,
            control_table_address=self.model_info.control_table["Goal_PWM"][0],
            data_length=self.model_info.control_table["Goal_PWM"][1],
        )

    @property
    def pwm_percentage(self) -> dict[int, float]:
        return {
            dynamixel_id: float(
                map_range(
                    pwm,
                    input_min=-self.motor_info[dynamixel_id].pwm_limit,
                    input_max=self.motor_info[dynamixel_id].pwm_limit,
                    output_min=-100.0,
                    output_max=100.0,
                )
            )
            for dynamixel_id, pwm in self.pwm.items()
        }

    @pwm_percentage.setter
    def pwm_percentage(self, pwm_percentage_for_dynamixel_id: dict[int, float]):
        pwm_for_dynamixel_id = {
            dynamixel_id: int(
                map_range(
                    max(min(pwm_percentage, 100.0), -100.0),
                    input_min=-100.0,
                    input_max=100.0,
                    output_min=-self.motor_info[dynamixel_id].pwm_limit,
                    output_max=self.motor_info[dynamixel_id].pwm_limit,
                )
            )
            for dynamixel_id, pwm_percentage in pwm_percentage_for_dynamixel_id.items()
        }

        self.pwm = pwm_for_dynamixel_id

    def enable_torque(self):
        self._group_sync_write(
            {dynamixel_id: 1 for dynamixel_id in self.dynamixel_ids},
            self.model_info.control_table["Torque_Enable"][0],
            self.model_info.control_table["Torque_Enable"][1],
        )

    def disable_torque(self):
        self._group_sync_write(
            {dynamixel_id: 0 for dynamixel_id in self.dynamixel_ids},
            self.model_info.control_table["Torque_Enable"][0],
            self.model_info.control_table["Torque_Enable"][1],
        )

    def set_mode(self, mode: DynamixelMode):
        self._group_sync_write(
            {dynamixel_id: mode.value for dynamixel_id in self.dynamixel_ids},
            self.model_info.control_table["Operating_Mode"][0],
            self.model_info.control_table["Operating_Mode"][1],
        )


class DynamixelMotor:
    def __init__(
        self,
        dxl_io: DynamixelIO,
        dynamixel_model_info: DynamixelModelInfo,
        dynamixel_id: int,
    ):
        self.dxl_io = dxl_io
        self.model_info = dynamixel_model_info
        self.dynamixel_id = dynamixel_id

        min_position = self._read(
            self.model_info.control_table["Min_Position_Limit"][0],
            self.model_info.control_table["Min_Position_Limit"][1],
        )
        max_position = self._read(
            self.model_info.control_table["Max_Position_Limit"][0],
            self.model_info.control_table["Max_Position_Limit"][1],
        )
        velocity_limit = self._read(
            self.model_info.control_table["Velocity_Limit"][0],
            self.model_info.control_table["Velocity_Limit"][1],
        )
        pwm_limit = self._read(
            self.model_info.control_table["PWM_Limit"][0],
            self.model_info.control_table["PWM_Limit"][1],
        )

        self.motor_info = DynamixelInfo(
            min_position, max_position, velocity_limit, pwm_limit
        )

    def _read(self, control_table_address: int, data_length: int) -> int:
        """Returns the held value from a given address in the control table"""
        ret_val = 0
        dxl_comm_result = 0
        dxl_error = 0

        # the following has to be done inelegantly due to the dynamixel sdk having separate functions per packet size.
        # future versions of this library may replace usage of the dynamixel sdk to increase efficiency and remove this
        # bulky situation.
        if data_length == 1:
            ret_val, dxl_comm_result, dxl_error = (
                self.dxl_io.packet_handler.read1ByteTxRx(
                    self.dxl_io.port_handler, self.dynamixel_id, control_table_address
                )
            )
        elif data_length == 2:
            ret_val, dxl_comm_result, dxl_error = (
                self.dxl_io.packet_handler.read2ByteTxRx(
                    self.dxl_io.port_handler, self.dynamixel_id, control_table_address
                )
            )
        elif data_length == 4:
            ret_val, dxl_comm_result, dxl_error = (
                self.dxl_io.packet_handler.read4ByteTxRx(
                    self.dxl_io.port_handler, self.dynamixel_id, control_table_address
                )
            )

        self.dxl_io.check_error(dxl_comm_result, dxl_error)

        return ret_val

    def _write(self, data: int, control_table_address: int, data_length: int):
        """Writes a specified value to a given address in the control table"""
        dxl_comm_result = 0
        dxl_error = 0

        if data_length == 1:
            dxl_comm_result, dxl_error = self.dxl_io.packet_handler.write1ByteTxRx(
                self.dxl_io.port_handler, self.dynamixel_id, control_table_address, data
            )
        elif data_length == 2:
            dxl_comm_result, dxl_error = self.dxl_io.packet_handler.write2ByteTxRx(
                self.dxl_io.port_handler, self.dynamixel_id, control_table_address, data
            )
        elif data_length == 4:
            dxl_comm_result, dxl_error = self.dxl_io.packet_handler.write4ByteTxRx(
                self.dxl_io.port_handler, self.dynamixel_id, control_table_address, data
            )

        self.dxl_io.check_error(dxl_comm_result, dxl_error)

    @property
    def position(self) -> int:
        return self._read(
            control_table_address=self.model_info.control_table["Present_Position"][0],
            data_length=self.model_info.control_table["Present_Position"][1],
        )

    @position.setter
    def position(self, position: int):
        self._write(
            data=position,
            control_table_address=self.model_info.control_table["Goal_Position"][0],
            data_length=self.model_info.control_table["Goal_Position"][1],
        )

    @property
    def angle_deg(self) -> float:
        return float(
            map_range(
                self.position,
                input_min=self.motor_info.min_position,
                input_max=self.motor_info.max_position + 1,
                output_min=0.0,
                output_max=self.model_info.max_angle_deg,
            )
        )

    @angle_deg.setter
    def angle_deg(self, angle_deg: float):
        position = int(
            map_range(
                angle_deg,
                input_min=0.0,
                input_max=self.model_info.max_angle_deg,
                output_min=self.motor_info.min_position,
                output_max=self.motor_info.max_position + 1,
            )
        )

        self.position = position

    @property
    def angle_rad(self) -> float:
        return math.radians(self.angle_deg)

    @angle_rad.setter
    def angle_rad(self, angle_rad: float):
        position = int(
            map_range(
                math.degrees(angle_rad),
                input_min=0.0,
                input_max=self.model_info.max_angle_deg,
                output_min=self.motor_info.min_position,
                output_max=self.motor_info.max_position + 1,
            )
        )

        self.position = position

    @property
    def velocity(self) -> int:
        return self._read(
            control_table_address=self.model_info.control_table["Present_Velocity"][0],
            data_length=self.model_info.control_table["Present_Velocity"][1],
        )

    @velocity.setter
    def velocity(self, velocity: int):
        self._write(
            data=velocity,
            control_table_address=self.model_info.control_table["Goal_Velocity"][0],
            data_length=self.model_info.control_table["Goal_Velocity"][1],
        )

    def velocity_rpm_per_tick_factor(self) -> float:
        rev_per_min_per_tick = 0.0
        if self.model_info.model is DynamixelModel.AX12:
            rev_per_min_per_tick = 0.111
        elif self.model_info.model is DynamixelModel.MX12:
            rev_per_min_per_tick = 0.916
        elif self.model_info.model is DynamixelModel.MX28:
            rev_per_min_per_tick = 0.229
        elif self.model_info.model is DynamixelModel.MX64:
            rev_per_min_per_tick = 0.229
        elif self.model_info.model is DynamixelModel.MX106:
            rev_per_min_per_tick = 0.229

        return rev_per_min_per_tick

    @property
    def velocity_deg_per_s(self) -> float:
        factor = self.velocity_rpm_per_tick_factor()
        factor *= 360.0 / 60.0
        return float(
            map_range(
                self.velocity,
                input_min=-self.motor_info.velocity_limit,
                input_max=self.motor_info.velocity_limit,
                output_min=-factor * self.motor_info.velocity_limit,
                output_max=factor * self.motor_info.velocity_limit,
            )
        )

    @velocity_deg_per_s.setter
    def velocity_deg_per_s(self, velocity_deg_per_s: dict[int, float]):
        factor = self.velocity_rpm_per_tick_factor()
        factor *= 360.0 / 60.0
        velocity = int(
            map_range(
                velocity_deg_per_s,
                input_min=-factor * self.motor_info.velocity_limit,
                input_max=factor * self.motor_info.velocity_limit,
                output_min=-self.motor_info.velocity_limit,
                output_max=self.motor_info.velocity_limit,
            )
        )

        self.velocity = velocity


    @property
    def pwm(self) -> int:
        return self._read(
            control_table_address=self.model_info.control_table["Present_PWM"][0],
            data_length=self.model_info.control_table["Present_PWM"][1],
        )

    @pwm.setter
    def pwm(self, pwm: int):
        self._write(
            data=pwm,
            control_table_address=self.model_info.control_table["Goal_PWM"][0],
            data_length=self.model_info.control_table["Goal_PWM"][1],
        )

    @property
    def pwm_percentage(self) -> float:
        return float(
            map_range(
                self.pwm,
                input_min=-self.motor_info.pwm_limit,
                input_max=self.motor_info.pwm_limit,
                output_min=-100.0,
                output_max=100.0,
            )
        )

    @pwm_percentage.setter
    def pwm_percentage(self, pwm_percentage: float):
        pwm = int(
            map_range(
                max(min(pwm_percentage, 100.0), -100.0),
                input_min=-100.0,
                input_max=100.0,
                output_min=-self.motor_info.pwm_limit,
                output_max=self.motor_info.pwm_limit,
            )
        )

        self.pwm = pwm

    def enable_torque(self):
        self._write(
            1,
            self.model_info.control_table["Torque_Enable"][0],
            self.model_info.control_table["Torque_Enable"][1],
        )

    def disable_torque(self):
        self._write(
            0,
            self.model_info.control_table["Torque_Enable"][0],
            self.model_info.control_table["Torque_Enable"][1],
        )

    def set_mode(self, mode: DynamixelMode):
        self._write(
            mode.value,
            self.model_info.control_table["Operating_Mode"][0],
            self.model_info.control_table["Operating_Mode"][1],
        )


class DynamixelMotorFactory:
    def __init__(
        self,
        dxl_io: DynamixelIO,
        dynamixel_model: DynamixelModel,
    ):
        self.dxl_io = dxl_io
        self.dynamixel_model = dynamixel_model

        with open(
            Path(__file__).parent
            / "DynamixelJSON"
            / f"{self.dynamixel_model.value}.json",
            mode="r",
        ) as control_table_file:
            motor_info: dict[str, dict[str, list[int]] | dict[str, int]] = (
                json.load(control_table_file)
            )["Protocol_2"]

            control_table: dict[str, list[int]] = motor_info["Control_Table"]
            motor_constants: dict[str, int] = motor_info["Values"]
            max_angle_deg: float = float(motor_constants["Max_Angle"])

            self.model_info = DynamixelModelInfo(
                control_table, max_angle_deg, self.dynamixel_model
            )

    def create(self, *dynamixel_ids: int):
        if len(dynamixel_ids) == 1:
            return DynamixelMotor(
                dxl_io=self.dxl_io,
                dynamixel_model_info=self.model_info,
                dynamixel_id=dynamixel_ids[0],
            )
        else:
            return DynamixelMotorGroup(
                dxl_io=self.dxl_io, dynamixel_model_info=self.model_info, *dynamixel_ids
            )
