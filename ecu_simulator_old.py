import can
import cantools
import time
from pprint import pprint
from typing import Literal

def create_eng_13c_message(
    db: cantools.database.Database,
    alive_counter: int,
    pedal_pos_percent: float = 0.0,
    driver_torque_nm: float = 0.0,
    engine_torque_nm: float = 0.0,
    in_reverse: bool = False
) -> can.Message:
    """
    Encodes the ENG_13C CAN message with the provided signal values.

    Args:
        db: The loaded cantools database object.
        alive_counter: The value for the message's alive counter (0-3).
        pedal_pos_percent: Accelerator pedal position (0.0 to 100.0).
        driver_torque_nm: The driver's requested torque in Nm.
        engine_torque_nm: The actual engine torque in Nm.
        in_reverse: Set to True if the car is in reverse.

    Returns:
        A can.Message object ready to be sent.
    """
    # Get the message definition from the database
    message_def = db.get_message_by_name('ENG_13C')

    # --- Build the data dictionary ---
    # This dictionary must contain ALL signals for the message.
    data_to_encode = {
        # --- Dynamic Signals (from function arguments) ---
        'ENG_SMART_ACCELE_PEDAL_POS_13C': pedal_pos_percent,
        'ENG_DRIVER_REQ_TRQ_13C': driver_torque_nm,
        'ENG_ENG_TRQ_13C': engine_torque_nm,
        'ENG_ALIVE_COUNTER_13C': alive_counter,
        'ENG_SHFT_IN_REVERSE_13C': 1 if in_reverse else 0,

        # --- Unimportant Signals (set to default values) ---
        'ENG_VSA_ACK1_13C': 0,
        'ENG_VSA_PERMIT_PLUS_TRQ_13C': 0,
        'ENG_VSA_INHIBIT_VSA_CONTROL_13C': 0,
        'ENG_VSA_REFUSED_VSA_CONTROL_13C': 0,
        'ENG_CLUTCH_SW_CC_13C': 0,
        'ENG_RECEIVE_ERROR_1AA': 0,
        
        # The checksum is often calculated by the hardware or a lower-level driver.
        # Cantools can also be configured to calculate it if the DBC defines the logic.
        # Here, we provide a default of 0.
        'ENG_CHECKSUM_13C': 0,
    }

    # Encode the data into a payload (byte array)
    encoded_payload = message_def.encode(data_to_encode)

    # Create the final CAN message object with the correct ID and data
    message_to_send = can.Message(
        arbitration_id=message_def.frame_id,
        data=encoded_payload
    )

    return message_to_send

def create_vsa_1d0_message(
    db: cantools.db.Database,
    fl_speed_kph: float,
    fr_speed_kph: float,
    rl_speed_kph: float,
    rr_speed_kph: float,
) -> can.Message:
    """
    Encodes the VSA_1D0 CAN message with individual wheel speeds.

    Args:
        db: The loaded cantools database object.
        fl_speed_kph: Front-left wheel speed in km/h.
        fr_speed_kph: Front-right wheel speed in km/h.
        rl_speed_kph: Rear-left wheel speed in km/h.
        rr_speed_kph: Rear-right wheel speed in km/h.

    Returns:
        A can.Message object ready to be sent.
    """
    # Get the message definition from the database
    message_def = db.get_message_by_name('VSA_1D0')

    # --- Build the data dictionary ---
    # This dictionary must contain ALL signals for the message.
    data_to_encode = {
        # --- Dynamic Signals (from function arguments) ---
        'VSA_ABS_FL_WHEEL_SPEED': fl_speed_kph,
        'VSA_ABS_FR_WHEEL_SPEED': fr_speed_kph,
        'VSA_ABS_RL_WHEEL_SPEED': rl_speed_kph,
        'VSA_ABS_RR_WHEEL_SPEED': rr_speed_kph,
        
        # --- Unimportant Signal (set to default value) ---
        # The checksum is often calculated by the hardware. We'll default to 0.
        'VSA_ABS_CHECKSUM_1D0': 0,
    }

    # Encode the data into a payload (byte array)
    encoded_payload = message_def.encode(data_to_encode)

    # Create the final CAN message object with the correct ID and data
    message_to_send = can.Message(
        arbitration_id=message_def.frame_id,
        data=encoded_payload
    )

    return message_to_send

GearSelection = Literal['P', 'R', 'N', 'D', 'S', 'L']

def create_cvt_191_message(
    db: cantools.db.Database,
    gear: GearSelection,
    alive_counter: int,
    current_ratio: float = 0.0,
    target_ratio: float = 0.0
) -> can.Message:
    """
    Encodes the CVT_191 CAN message with the selected gear and other values.

    Args:
        db: The loaded cantools database object.
        gear: The selected gear ('P', 'R', 'N', 'D', 'S', or 'L').
        alive_counter: The value for the message's alive counter (0-3).
        current_ratio: The current CVT gear ratio.
        target_ratio: The target CVT gear ratio.

    Returns:
        A can.Message object ready to be sent.
    """
    # Get the message definition from the database
    message_def = db.get_message_by_name('CVT_191')

    # --- Set all gear flags to 0 initially ---
    gear_flags = {
        'CVT_SHFT_IN_PARKING': 0,
        'CVT_SHFT_IN_REVERSE': 0,
        'CVT_SHFT_IN_NEUTRAL': 0,
        'CVT_SHFT_IN_D': 0,
        'CVT_SHFT_IN_S': 0,
        'CVT_SHFT_IN_L': 0,
    }
    
    # --- Set the correct gear flag to 1 based on input ---
    if gear == 'P':
        gear_flags['CVT_SHFT_IN_PARKING'] = 1
    elif gear == 'R':
        gear_flags['CVT_SHFT_IN_REVERSE'] = 1
    elif gear == 'N':
        gear_flags['CVT_SHFT_IN_NEUTRAL'] = 1
    elif gear == 'D':
        gear_flags['CVT_SHFT_IN_D'] = 1
    elif gear == 'S':
        gear_flags['CVT_SHFT_IN_S'] = 1
    elif gear == 'L':
        gear_flags['CVT_SHFT_IN_L'] = 1

    # --- Build the data dictionary ---
    data_to_encode = {
        # --- Dynamic Signals ---
        **gear_flags,  # Unpack the gear flags dictionary
        'CVT_ALIVE_COUNTER_191': alive_counter,
        'CVT_RATIO_INFO_191': current_ratio,
        'CVT_RATIO_TARGET': target_ratio,

        # --- Unimportant Signals (set to default values) ---
        # CVT_GEAR_POSITION_IND_CVT could be dynamic, but often mirrors the gear flags.
        # We will set a simple default.
        'CVT_GEAR_POSITION_IND_CVT': 0,
        'CVT_REFUSE_VSA_CONTROL': 0,
        'CVT_CHECKSUM_191': 0,
        'CVT_CAS_ON': 1, # Assume system is on
    }

    # Encode the data into a payload (byte array)
    encoded_payload = message_def.encode(data_to_encode)

    # Create the final CAN message object
    message_to_send = can.Message(
        arbitration_id=message_def.frame_id,
        data=encoded_payload
    )

    return message_to_send

def create_eng_17c_message(
    db: cantools.db.Database,
    engine_speed_rpm: float,
    alive_counter: int,
    is_brake_pedal_pressed: bool = False,
) -> can.Message:
    """
    Encodes the ENG_17C CAN message with engine speed and brake status.

    Args:
        db: The loaded cantools database object.
        engine_speed_rpm: The current engine speed in RPM.
        alive_counter: The value for the message's alive counter (0-3).
        is_brake_pedal_pressed: True if the brake pedal is pressed, False otherwise.

    Returns:
        A can.Message object ready to be sent.
    """
    # Get the message definition from the database
    message_def = db.get_message_by_name('ENG_17C')

    # --- Build the data dictionary ---
    # This dictionary must contain ALL signals for the message.
    data_to_encode = {
        # --- Dynamic Signals (from function arguments) ---
        'ENG_ENG_SPEED': engine_speed_rpm,
        'ENG_SW_STATUS_BRAKE_NO': 1 if is_brake_pedal_pressed else 0,
        'ENG_ALIVE_COUNTER_17C': alive_counter,

        # --- Unimportant Signals (set to default values) ---
        'ENG_CRUISE_STATUS_CRUISE_LMP': 0,
        'ENG_IS_PRE_PROGRESS': 0,
        'ENG_IS_PRE_RESTART': 0,
        'ENG_IS_PROGRESS': 0,
        'ENG_CHECKSUM_17C': 0,
    }

    # Encode the data into a payload (byte array)
    encoded_payload = message_def.encode(data_to_encode)

    # Create the final CAN message object
    message_to_send = can.Message(
        arbitration_id=message_def.frame_id,
        data=encoded_payload
    )

    return message_to_send

def create_vsa_091_message(
    db: cantools.db.Database,
    alive_counter: int,
    yaw_rate: float = 0.0,
    steering_angle: float = 0.0,
    lateral_g: float = 0.0,
    longitudinal_g: float = 0.0
) -> can.Message:
    """
    Encodes the VSA_091 CAN message with vehicle dynamics data.

    Args:
        db: The loaded cantools database object.
        alive_counter: The value for the message's alive counter (0-3).
        yaw_rate: Vehicle's yaw rate.
        steering_angle: Steering wheel angle.
        lateral_g: Lateral G-force.
        longitudinal_g: Longitudinal G-force.

    Returns:
        A can.Message object ready to be sent.
    """
    # Get the message definition from the database
    message_def = db.get_message_by_name('VSA_091')

    # --- Build the data dictionary ---
    data_to_encode = {
        # --- Dynamic Signals (from function arguments) ---
        'VSA_YAW_1': yaw_rate,
        'VSA_DEGC': steering_angle,
        'VSA_LAT_G': lateral_g,
        'VSA_LON_G': longitudinal_g,
        'VSA_ALIVE_COUNTER_091': alive_counter,

        # --- Unimportant Signals (set to default "OK" or "inactive" values) ---
        'VSA_T_ERR_LON_G': 0,
        'VSA_T_ERR_LAT_G': 0,
        'VSA_T_ERR_DEGC': 0,
        'VSA_T_ERR_YAW1': 0,
        'VSA_SENSOR_ERR_MC': 0,
        'VSA_SENSOR_STATE_IG': 0,
        'VSA_P_ERR_LON_G': 0,
        'VSA_P_ERR_LAT_G': 0,
        'VSA_P_ERR_DEGC': 0,
        'VSA_P_ERR_YAW1': 0,
        'VSA_SENSOR_START_UP': 0, # Assuming sensor has started up
        'VSA_CHECKSUM_091': 0,
    }

    # Encode the data into a payload (byte array)
    encoded_payload = message_def.encode(data_to_encode)

    # Create the final CAN message object
    message_to_send = can.Message(
        arbitration_id=message_def.frame_id,
        data=encoded_payload
    )

    return message_to_send