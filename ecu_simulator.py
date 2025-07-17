# ecu_simulator.py

import can
import cantools
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
    """
    message_def = db.get_message_by_name('ENG_13C')
    data_to_encode = {
        'ENG_SMART_ACCELE_PEDAL_POS_13C': pedal_pos_percent,
        'ENG_DRIVER_REQ_TRQ_13C': driver_torque_nm,
        'ENG_ENG_TRQ_13C': engine_torque_nm,
        'ENG_ALIVE_COUNTER_13C': alive_counter,
        'ENG_SHFT_IN_REVERSE_13C': 1 if in_reverse else 0,
        'ENG_VSA_ACK1_13C': 0,
        'ENG_VSA_PERMIT_PLUS_TRQ_13C': 0,
        'ENG_VSA_INHIBIT_VSA_CONTROL_13C': 0,
        'ENG_VSA_REFUSED_VSA_CONTROL_13C': 0,
        'ENG_CLUTCH_SW_CC_13C': 0,
        'ENG_RECEIVE_ERROR_1AA': 0,
        'ENG_CHECKSUM_13C': 0,
    }
    encoded_payload = message_def.encode(data_to_encode)
    return can.Message(arbitration_id=message_def.frame_id, data=encoded_payload)

def create_vsa_1d0_message(
    db: cantools.db.Database,
    fl_speed_kph: float,
    fr_speed_kph: float = None,
    rl_speed_kph: float = None,
    rr_speed_kph: float = None,
) -> can.Message:
    """
    Encodes the VSA_1D0 CAN message with individual wheel speeds.
    """
    if fr_speed_kph is None: fr_speed_kph = fl_speed_kph
    if rl_speed_kph is None: rl_speed_kph = fl_speed_kph
    if rr_speed_kph is None: rr_speed_kph = fl_speed_kph

    message_def = db.get_message_by_name('VSA_1D0')
    data_to_encode = {
        'VSA_ABS_FL_WHEEL_SPEED': fl_speed_kph,
        'VSA_ABS_FR_WHEEL_SPEED': fr_speed_kph,
        'VSA_ABS_RL_WHEEL_SPEED': rl_speed_kph,
        'VSA_ABS_RR_WHEEL_SPEED': rr_speed_kph,
        'VSA_ABS_CHECKSUM_1D0': 0,
    }
    encoded_payload = message_def.encode(data_to_encode)
    return can.Message(arbitration_id=message_def.frame_id, data=encoded_payload)

GearSelection = Literal['P', 'R', 'N', 'D', 'S', 'L']

def create_cvt_191_message(
    db: cantools.db.Database,
    alive_counter: int,
    cvt_gear_position_ind_cvt: int = 0,
    gear: GearSelection = 'D',
    current_ratio: float = 0.0,
    target_ratio: float = 0.0
) -> can.Message:
    """
    Encodes the CVT_191 CAN message with the selected gear and other values.
    """
    message_def = db.get_message_by_name('CVT_191')

    gear_flags = {
        'CVT_SHFT_IN_PARKING': 0, 'CVT_SHFT_IN_REVERSE': 0, 'CVT_SHFT_IN_NEUTRAL': 0,
        'CVT_SHFT_IN_D': 0, 'CVT_SHFT_IN_S': 0, 'CVT_SHFT_IN_L': 0,
    }
    if gear == 'P': gear_flags['CVT_SHFT_IN_PARKING'] = 1
    elif gear == 'R': gear_flags['CVT_SHFT_IN_REVERSE'] = 1
    elif gear == 'N': gear_flags['CVT_SHFT_IN_NEUTRAL'] = 1
    elif gear == 'D': gear_flags['CVT_SHFT_IN_D'] = 1
    elif gear == 'S': gear_flags['CVT_SHFT_IN_S'] = 1
    elif gear == 'L': gear_flags['CVT_SHFT_IN_L'] = 1

    data_to_encode = {
        **gear_flags,
        'CVT_ALIVE_COUNTER_191': alive_counter,
        'CVT_RATIO_INFO_191': current_ratio,
        'CVT_RATIO_TARGET': target_ratio,
        'CVT_GEAR_POSITION_IND_CVT': cvt_gear_position_ind_cvt,
        'CVT_REFUSE_VSA_CONTROL': 0,
        'CVT_CHECKSUM_191': 0,
        'CVT_CAS_ON': 1,
    }
    encoded_payload = message_def.encode(data_to_encode)
    return can.Message(arbitration_id=message_def.frame_id, data=encoded_payload)

def create_eng_17c_message(
    db: cantools.db.Database,
    engine_speed_rpm: float,
    alive_counter: int,
    is_brake_pedal_pressed: bool = False,
    is_progress: bool = False
) -> can.Message:
    """
    Encodes the ENG_17C CAN message with engine speed, brake status, and IS progress.
    """
    message_def = db.get_message_by_name('ENG_17C')
    data_to_encode = {
        'ENG_ENG_SPEED': engine_speed_rpm,
        'ENG_SW_STATUS_BRAKE_NO': 1 if is_brake_pedal_pressed else 0,
        'ENG_ALIVE_COUNTER_17C': alive_counter,
        'ENG_CRUISE_STATUS_CRUISE_LMP': 0,
        'ENG_IS_PRE_PROGRESS': 0,
        'ENG_IS_PRE_RESTART': 0,
        'ENG_IS_PROGRESS': 1 if is_progress else 0,
        'ENG_CHECKSUM_17C': 0,
    }
    encoded_payload = message_def.encode(data_to_encode)
    return can.Message(arbitration_id=message_def.frame_id, data=encoded_payload)

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
    """
    message_def = db.get_message_by_name('VSA_091')
    data_to_encode = {
        'VSA_YAW_1': yaw_rate,
        'VSA_DEGC': steering_angle,
        'VSA_LAT_G': lateral_g,
        'VSA_LON_G': longitudinal_g,
        'VSA_ALIVE_COUNTER_091': alive_counter,
        'VSA_T_ERR_LON_G': 0, 'VSA_T_ERR_LAT_G': 0, 'VSA_T_ERR_DEGC': 0, 'VSA_T_ERR_YAW1': 0,
        'VSA_SENSOR_ERR_MC': 0, 'VSA_SENSOR_STATE_IG': 0,
        'VSA_P_ERR_LON_G': 0, 'VSA_P_ERR_LAT_G': 0, 'VSA_P_ERR_DEGC': 0, 'VSA_P_ERR_YAW1': 0,
        'VSA_SENSOR_START_UP': 0,
        'VSA_CHECKSUM_091': 0,
    }
    encoded_payload = message_def.encode(data_to_encode)
    return can.Message(arbitration_id=message_def.frame_id, data=encoded_payload)

# --- MODIFIED: create_vsa_255_message (removed TCS/ABS flags) ---
def create_vsa_255_message(
    db: cantools.db.Database,
    alive_counter: int,
    fl_speed_kph_255: float = 0.0,
    fr_speed_kph_255: float = 0.0,
    rl_speed_kph_255: float = 0.0,
    rr_speed_kph_255: float = 0.0,
    abs_speed_fixed_number1: float = 0.0,
    abs_speed_fixed_number2: float = 0.0,
) -> can.Message:
    """
    Encodes the VSA_255 CAN message with wheel speed signals and fixed numbers.
    VSA_VSA_TCS_ACT and VSA_ABS_EBD_ACT are NOT in this message as per new info.
    """
    message_def = db.get_message_by_name('VSA_255')
    data_to_encode = {
        'VSA_ALIVE_COUNTER_255': alive_counter,
        'VSA_ABS_FL_WHEEL_SPEED_255': fl_speed_kph_255,
        'VSA_ABS_FR_WHEEL_SPEED_255': fr_speed_kph_255,
        'VSA_ABS_RL_WHEEL_SPEED_255': rl_speed_kph_255,
        'VSA_ABS_RR_WHEEL_SPEED_255': rr_speed_kph_255,
        'VSA_ABS_SPEED_FIXED_NUMBER1': abs_speed_fixed_number1,
        'VSA_ABS_SPEED_FIXED_NUMBER2': abs_speed_fixed_number2,
        'VSA_ABS_CHECKSUM_255': 0,
        # Placeholder signals from previous version, confirm if they are in VSA_255 DBC
        'VSA_ABS_STATUS_1': 0,
        'VSA_VSA_TCS_STATUS': 0,
        'VSA_VSA_MODE': 0,
        'VSA_BRAKE_FORCE': 0,
    }
    encoded_payload = message_def.encode(data_to_encode)
    return can.Message(arbitration_id=message_def.frame_id, data=encoded_payload)

def create_vsa_1a4_message(
    db: cantools.db.Database,
    alive_counter: int,
    vsa_tcs_act: bool = False,
    abs_ebd_act: bool = False,
    vsa_fail_mc_pressure_sensor: bool = False,
    vsa_inhibit_mc_pressure_sensor: bool = False,
    vsa_ess_act2: bool = False,
    vsa_ess_act1: bool = False,
    vsa_master_cylinder_pressure: float = 0.0,
    vsa_vsa_tcs_mil: bool = False,
    vsa_abs_ebd_mil: bool = False,
    vsa_casen: bool = False,
    vsa_cas_act: bool = False,
    vsa_ba_act: bool = False,
    vsa_acc_brake_act: bool = False,
    vsa_answer_dws_init_req: bool = False,
    vsa_warn_status_puncture: bool = False,
    vsa_warn_status_dws: bool = False,
    vsa_mid_request_vsa: bool = False,
    vsa_lamp_status_vsa_off: bool = False,
    vsa_warn_status_vsa: bool = False,
    vsa_warn_status_abs: bool = False,
    vsa_warn_status_brake: bool = False,
    vsa_l_mode_abs_ebd: bool = False,
    vsa_rewrite_start: bool = False,
    vsa_buzzer_status: bool = False,
    vsa_warn_status_hsa: bool = False,
    vsa_hsa_status_act: bool = False,
    vsa_ess_act4: bool = False,
    vsa_ess_act3: bool = False,
    vsa_abs_ctrl: bool = False,
    vsa_vsa_ctrl: bool = False
) -> can.Message:
    """
    Encodes the VSA_1A4 CAN message with various VSA/ABS status and activation flags.
    Includes VSA_VSA_TCS_ACT and VSA_ABS_EBD_ACT.
    """
    message_def = db.get_message_by_name('VSA_1A4')
    data_to_encode = {
        'VSA_FAIL_MC_PRESSURE_SENSOR': 1 if vsa_fail_mc_pressure_sensor else 0,
        'VSA_INHIBIT_MC_PRESSURE_SENSOR': 1 if vsa_inhibit_mc_pressure_sensor else 0,
        'VSA_ESS_ACT2': 1 if vsa_ess_act2 else 0,
        'VSA_ESS_ACT1': 1 if vsa_ess_act1 else 0,
        'VSA_MASTER_CYLINDER_PRESSURE': vsa_master_cylinder_pressure,
        'VSA_VSA_TCS_ACT': 1 if vsa_tcs_act else 0,
        'VSA_ABS_EBD_ACT': 1 if abs_ebd_act else 0,
        'VSA_VSA_TCS_MIL': 1 if vsa_vsa_tcs_mil else 0,
        'VSA_ABS_EBD_MIL': 1 if vsa_abs_ebd_mil else 0,
        'VSA_CASEN': 1 if vsa_casen else 0,
        'VSA_CAS_ACT': 1 if vsa_cas_act else 0,
        'VSA_BA_ACT': 1 if vsa_ba_act else 0,
        'VSA_ACC_BRAKE_ACT': 1 if vsa_acc_brake_act else 0,
        'VSA_ANSWER_DWS_INIT_REQ': 1 if vsa_answer_dws_init_req else 0,
        'VSA_WARN_STATUS_PUNCTURE': 1 if vsa_warn_status_puncture else 0,
        'VSA_WARN_STATUS_DWS': 1 if vsa_warn_status_dws else 0,
        'VSA_MID_REQUEST_VSA': 1 if vsa_mid_request_vsa else 0,
        'VSA_LAMP_STATUS_VSA_OFF': 1 if vsa_lamp_status_vsa_off else 0,
        'VSA_WARN_STATUS_VSA': 1 if vsa_warn_status_vsa else 0,
        'VSA_WARN_STATUS_ABS': 1 if vsa_warn_status_abs else 0,
        'VSA_WARN_STATUS_BRAKE': 1 if vsa_warn_status_brake else 0,
        'VSA_L_MODE_ABS_EBD': 1 if vsa_l_mode_abs_ebd else 0,
        'VSA_REWRITE_START': 1 if vsa_rewrite_start else 0,
        'VSA_BUZZER_STATUS': 1 if vsa_buzzer_status else 0,
        'VSA_WARN_STATUS_HSA': 1 if vsa_warn_status_hsa else 0,
        'VSA_HSA_STATUS_ACT': 1 if vsa_hsa_status_act else 0,
        'VSA_ESS_ACT4': 1 if vsa_ess_act4 else 0,
        'VSA_ESS_ACT3': 1 if vsa_ess_act3 else 0,
        'VSA_CHECKSU M_1A4': 0, # Assuming this is VSA_CHECKSUM_1A4
        'VSA_ALIVE_COUNTER_1A4': alive_counter,
        'VSA_ABS_CTRL': 1 if vsa_abs_ctrl else 0,
        'VSA_VSA_CTRL': 1 if vsa_vsa_ctrl else 0,
    }
    encoded_payload = message_def.encode(data_to_encode)
    return can.Message(arbitration_id=message_def.frame_id, data=encoded_payload)