import can
import cantools
import time
from pprint import pprint
from ecu_simulator import create_eng_13c_message, create_vsa_1d0_message, create_cvt_191_message, GearSelection, create_eng_17c_message, create_vsa_091_message

db = cantools.database.load_file('BOSCH_CAN.dbc')
bus = can.interface.Bus(channel='vcan0', interface='socketcan')

VSA_255 = db.get_message_by_name('VSA_255')
METER_294 = db.get_message_by_name('METER_294')


def send_speed():
  pass

  

def main():
  # try:
  #   while True:

  # except KeyboardInterrupt:
  #   print("Exiting...")
  #   bus.shutdown()
  #   return

  # data_to_encode = {'METER_ALIVE_COUNTER_294': 1, 'METER_ODO_DATA': 123456, 'METER_CHECKSUM_294': 0}
  # METER_294_data = METER_294.encode(data_to_encode)
  # message_to_send = can.Message(
  #   arbitration_id=METER_294.frame_id, 
  #   data=METER_294_data,
  # )
  # message_to_send = create_eng_13c_message(db, 1, 50.0, 100.0, 200.0, False)
  # message_to_send = create_vsa_1d0_message(db, 1, 50, 50, 50)
  # message_to_send = create_cvt_191_message(
  #   db=db,
  #   gear='D',
  #   alive_counter=1,
  #   current_ratio=0.5,
  #   target_ratio=0.7
  # )
  # message_to_send = create_eng_17c_message(
  #   db=db,
  #   engine_speed_rpm=3000,
  #   alive_counter=1,
  #   is_brake_pedal_pressed=False,
  # )
  message_to_send = create_vsa_091_message(
    db=db,
    yaw_rate=0.1,
    steering_angle=0.5,
    lateral_g=0.2,
    longitudinal_g=0.3,
    alive_counter=1
  )
  bus.send(message_to_send)
  bus.shutdown()

  

if __name__ == '__main__':
    main()