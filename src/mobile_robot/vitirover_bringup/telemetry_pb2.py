# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: telemetry.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='telemetry.proto',
  package='telemetry',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x0ftelemetry.proto\x12\ttelemetry\"(\n\x0cGnssPosition\x12\x0b\n\x03lat\x18\x01 \x01(\x02\x12\x0b\n\x03lng\x18\x02 \x01(\x02\"\x92\x06\n\x12VitiroverTelemetry\x12\x10\n\x08robot_id\x18\x01 \x01(\r\x12)\n\x08position\x18\x02 \x01(\x0b\x32\x17.telemetry.GnssPosition\x12\x15\n\rerror_gnss_cm\x18\x03 \x01(\r\x12\x0f\n\x07heading\x18\x04 \x01(\r\x12\x17\n\x0f\x62\x61ttery_voltage\x18\x05 \x01(\x02\x12.\n\x10\x66ront_left_wheel\x18\x06 \x01(\x0b\x32\x14.telemetry.MotorData\x12/\n\x11\x66ront_right_wheel\x18\x07 \x01(\x0b\x32\x14.telemetry.MotorData\x12.\n\x10\x62\x61\x63k_right_wheel\x18\x08 \x01(\x0b\x32\x14.telemetry.MotorData\x12-\n\x0f\x62\x61\x63k_left_wheel\x18\t \x01(\x0b\x32\x14.telemetry.MotorData\x12(\n\nleft_mower\x18\n \x01(\x0b\x32\x14.telemetry.MotorData\x12)\n\x0bright_mower\x18\x0b \x01(\x0b\x32\x14.telemetry.MotorData\x12\x17\n\x0f\x62\x61\x63k_axle_angle\x18\x0c \x01(\x02\x12\x12\n\nsun_charge\x18\r \x01(\x02\x12\x12\n\nwifi_is_up\x18\x0e \x01(\x08\x12\x0c\n\x04roll\x18\x0f \x01(\x02\x12\r\n\x05pitch\x18\x10 \x01(\x02\x12\x19\n\x11timestamp_seconds\x18\x11 \x01(\r\x12\x1e\n\x16timestamp_milliseconds\x18\x12 \x01(\r\x12\x13\n\x0bgyroscope_x\x18\x13 \x01(\x02\x12\x13\n\x0bgyroscope_y\x18\x14 \x01(\x02\x12\x13\n\x0bgyroscope_z\x18\x15 \x01(\x02\x12\x17\n\x0f\x61\x63\x63\x65lerometer_x\x18\x16 \x01(\x02\x12\x17\n\x0f\x61\x63\x63\x65lerometer_y\x18\x17 \x01(\x02\x12\x17\n\x0f\x61\x63\x63\x65lerometer_z\x18\x18 \x01(\x02\x12\x16\n\x0emagnetometer_x\x18\x19 \x01(\x02\x12\x16\n\x0emagnetometer_y\x18\x1a \x01(\x02\x12\x16\n\x0emagnetometer_z\x18\x1b \x01(\x02\"P\n\tMotorData\x12\x12\n\nmilli_amps\x18\x01 \x01(\x05\x12\r\n\x05power\x18\x02 \x01(\x05\x12 \n\x18\x62\x61\x63k_electromotive_force\x18\x03 \x01(\x05\"\x9c\x01\n\x0eVitiroverOrder\x12>\n\x10high_level_order\x18\x01 \x01(\x0b\x32\".telemetry.VitiroverHighLevelOrderH\x00\x12<\n\x0flow_level_order\x18\x02 \x01(\x0b\x32!.telemetry.VitiroverLowLevelOrderH\x00\x42\x0c\n\norder_type\"\x9d\x01\n\x17VitiroverHighLevelOrder\x12,\n\x0c\x63ontrol_mode\x18\x01 \x01(\x0e\x32\x16.telemetry.ControlMode\x12\r\n\x05speed\x18\x02 \x01(\x05\x12\x17\n\x0f\x62\x61\x63k_axle_angle\x18\x03 \x01(\x02\x12,\n\x0cturning_mode\x18\x04 \x01(\x0e\x32\x16.telemetry.TurningMode\"\xae\x01\n\x16VitiroverLowLevelOrder\x12,\n\x0c\x63ontrol_mode\x18\x01 \x01(\x0e\x32\x16.telemetry.ControlMode\x12\x18\n\x10\x66ront_left_speed\x18\x02 \x01(\x05\x12\x19\n\x11\x66ront_right_speed\x18\x03 \x01(\x05\x12\x18\n\x10\x62\x61\x63k_right_speed\x18\x04 \x01(\x05\x12\x17\n\x0f\x62\x61\x63k_left_speed\x18\x05 \x01(\x05*\x1f\n\x0b\x43ontrolMode\x12\x07\n\x03PID\x10\x00\x12\x07\n\x03PWM\x10\x01*6\n\x0bTurningMode\x12\x11\n\rMAX_BACK_AXLE\x10\x00\x12\x08\n\x04STOP\x10\x03\x12\n\n\x06MANUAL\x10\x04\x62\x06proto3')
)

_CONTROLMODE = _descriptor.EnumDescriptor(
  name='ControlMode',
  full_name='telemetry.ControlMode',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='PID', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PWM', index=1, number=1,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1439,
  serialized_end=1470,
)
_sym_db.RegisterEnumDescriptor(_CONTROLMODE)

ControlMode = enum_type_wrapper.EnumTypeWrapper(_CONTROLMODE)
_TURNINGMODE = _descriptor.EnumDescriptor(
  name='TurningMode',
  full_name='telemetry.TurningMode',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='MAX_BACK_AXLE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STOP', index=1, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MANUAL', index=2, number=4,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1472,
  serialized_end=1526,
)
_sym_db.RegisterEnumDescriptor(_TURNINGMODE)

TurningMode = enum_type_wrapper.EnumTypeWrapper(_TURNINGMODE)
PID = 0
PWM = 1
MAX_BACK_AXLE = 0
STOP = 3
MANUAL = 4



_GNSSPOSITION = _descriptor.Descriptor(
  name='GnssPosition',
  full_name='telemetry.GnssPosition',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lat', full_name='telemetry.GnssPosition.lat', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='lng', full_name='telemetry.GnssPosition.lng', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=30,
  serialized_end=70,
)


_VITIROVERTELEMETRY = _descriptor.Descriptor(
  name='VitiroverTelemetry',
  full_name='telemetry.VitiroverTelemetry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='robot_id', full_name='telemetry.VitiroverTelemetry.robot_id', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='position', full_name='telemetry.VitiroverTelemetry.position', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='error_gnss_cm', full_name='telemetry.VitiroverTelemetry.error_gnss_cm', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='heading', full_name='telemetry.VitiroverTelemetry.heading', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='battery_voltage', full_name='telemetry.VitiroverTelemetry.battery_voltage', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='front_left_wheel', full_name='telemetry.VitiroverTelemetry.front_left_wheel', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='front_right_wheel', full_name='telemetry.VitiroverTelemetry.front_right_wheel', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='back_right_wheel', full_name='telemetry.VitiroverTelemetry.back_right_wheel', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='back_left_wheel', full_name='telemetry.VitiroverTelemetry.back_left_wheel', index=8,
      number=9, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='left_mower', full_name='telemetry.VitiroverTelemetry.left_mower', index=9,
      number=10, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='right_mower', full_name='telemetry.VitiroverTelemetry.right_mower', index=10,
      number=11, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='back_axle_angle', full_name='telemetry.VitiroverTelemetry.back_axle_angle', index=11,
      number=12, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sun_charge', full_name='telemetry.VitiroverTelemetry.sun_charge', index=12,
      number=13, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='wifi_is_up', full_name='telemetry.VitiroverTelemetry.wifi_is_up', index=13,
      number=14, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='roll', full_name='telemetry.VitiroverTelemetry.roll', index=14,
      number=15, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pitch', full_name='telemetry.VitiroverTelemetry.pitch', index=15,
      number=16, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timestamp_seconds', full_name='telemetry.VitiroverTelemetry.timestamp_seconds', index=16,
      number=17, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timestamp_milliseconds', full_name='telemetry.VitiroverTelemetry.timestamp_milliseconds', index=17,
      number=18, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gyroscope_x', full_name='telemetry.VitiroverTelemetry.gyroscope_x', index=18,
      number=19, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gyroscope_y', full_name='telemetry.VitiroverTelemetry.gyroscope_y', index=19,
      number=20, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gyroscope_z', full_name='telemetry.VitiroverTelemetry.gyroscope_z', index=20,
      number=21, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accelerometer_x', full_name='telemetry.VitiroverTelemetry.accelerometer_x', index=21,
      number=22, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accelerometer_y', full_name='telemetry.VitiroverTelemetry.accelerometer_y', index=22,
      number=23, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accelerometer_z', full_name='telemetry.VitiroverTelemetry.accelerometer_z', index=23,
      number=24, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='magnetometer_x', full_name='telemetry.VitiroverTelemetry.magnetometer_x', index=24,
      number=25, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='magnetometer_y', full_name='telemetry.VitiroverTelemetry.magnetometer_y', index=25,
      number=26, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='magnetometer_z', full_name='telemetry.VitiroverTelemetry.magnetometer_z', index=26,
      number=27, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=73,
  serialized_end=859,
)


_MOTORDATA = _descriptor.Descriptor(
  name='MotorData',
  full_name='telemetry.MotorData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='milli_amps', full_name='telemetry.MotorData.milli_amps', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='power', full_name='telemetry.MotorData.power', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='back_electromotive_force', full_name='telemetry.MotorData.back_electromotive_force', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=861,
  serialized_end=941,
)


_VITIROVERORDER = _descriptor.Descriptor(
  name='VitiroverOrder',
  full_name='telemetry.VitiroverOrder',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='high_level_order', full_name='telemetry.VitiroverOrder.high_level_order', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='low_level_order', full_name='telemetry.VitiroverOrder.low_level_order', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='order_type', full_name='telemetry.VitiroverOrder.order_type',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=944,
  serialized_end=1100,
)


_VITIROVERHIGHLEVELORDER = _descriptor.Descriptor(
  name='VitiroverHighLevelOrder',
  full_name='telemetry.VitiroverHighLevelOrder',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='control_mode', full_name='telemetry.VitiroverHighLevelOrder.control_mode', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='speed', full_name='telemetry.VitiroverHighLevelOrder.speed', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='back_axle_angle', full_name='telemetry.VitiroverHighLevelOrder.back_axle_angle', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='turning_mode', full_name='telemetry.VitiroverHighLevelOrder.turning_mode', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1103,
  serialized_end=1260,
)


_VITIROVERLOWLEVELORDER = _descriptor.Descriptor(
  name='VitiroverLowLevelOrder',
  full_name='telemetry.VitiroverLowLevelOrder',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='control_mode', full_name='telemetry.VitiroverLowLevelOrder.control_mode', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='front_left_speed', full_name='telemetry.VitiroverLowLevelOrder.front_left_speed', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='front_right_speed', full_name='telemetry.VitiroverLowLevelOrder.front_right_speed', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='back_right_speed', full_name='telemetry.VitiroverLowLevelOrder.back_right_speed', index=3,
      number=4, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='back_left_speed', full_name='telemetry.VitiroverLowLevelOrder.back_left_speed', index=4,
      number=5, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1263,
  serialized_end=1437,
)

_VITIROVERTELEMETRY.fields_by_name['position'].message_type = _GNSSPOSITION
_VITIROVERTELEMETRY.fields_by_name['front_left_wheel'].message_type = _MOTORDATA
_VITIROVERTELEMETRY.fields_by_name['front_right_wheel'].message_type = _MOTORDATA
_VITIROVERTELEMETRY.fields_by_name['back_right_wheel'].message_type = _MOTORDATA
_VITIROVERTELEMETRY.fields_by_name['back_left_wheel'].message_type = _MOTORDATA
_VITIROVERTELEMETRY.fields_by_name['left_mower'].message_type = _MOTORDATA
_VITIROVERTELEMETRY.fields_by_name['right_mower'].message_type = _MOTORDATA
_VITIROVERORDER.fields_by_name['high_level_order'].message_type = _VITIROVERHIGHLEVELORDER
_VITIROVERORDER.fields_by_name['low_level_order'].message_type = _VITIROVERLOWLEVELORDER
_VITIROVERORDER.oneofs_by_name['order_type'].fields.append(
  _VITIROVERORDER.fields_by_name['high_level_order'])
_VITIROVERORDER.fields_by_name['high_level_order'].containing_oneof = _VITIROVERORDER.oneofs_by_name['order_type']
_VITIROVERORDER.oneofs_by_name['order_type'].fields.append(
  _VITIROVERORDER.fields_by_name['low_level_order'])
_VITIROVERORDER.fields_by_name['low_level_order'].containing_oneof = _VITIROVERORDER.oneofs_by_name['order_type']
_VITIROVERHIGHLEVELORDER.fields_by_name['control_mode'].enum_type = _CONTROLMODE
_VITIROVERHIGHLEVELORDER.fields_by_name['turning_mode'].enum_type = _TURNINGMODE
_VITIROVERLOWLEVELORDER.fields_by_name['control_mode'].enum_type = _CONTROLMODE
DESCRIPTOR.message_types_by_name['GnssPosition'] = _GNSSPOSITION
DESCRIPTOR.message_types_by_name['VitiroverTelemetry'] = _VITIROVERTELEMETRY
DESCRIPTOR.message_types_by_name['MotorData'] = _MOTORDATA
DESCRIPTOR.message_types_by_name['VitiroverOrder'] = _VITIROVERORDER
DESCRIPTOR.message_types_by_name['VitiroverHighLevelOrder'] = _VITIROVERHIGHLEVELORDER
DESCRIPTOR.message_types_by_name['VitiroverLowLevelOrder'] = _VITIROVERLOWLEVELORDER
DESCRIPTOR.enum_types_by_name['ControlMode'] = _CONTROLMODE
DESCRIPTOR.enum_types_by_name['TurningMode'] = _TURNINGMODE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

GnssPosition = _reflection.GeneratedProtocolMessageType('GnssPosition', (_message.Message,), dict(
  DESCRIPTOR = _GNSSPOSITION,
  __module__ = 'telemetry_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.GnssPosition)
  ))
_sym_db.RegisterMessage(GnssPosition)

VitiroverTelemetry = _reflection.GeneratedProtocolMessageType('VitiroverTelemetry', (_message.Message,), dict(
  DESCRIPTOR = _VITIROVERTELEMETRY,
  __module__ = 'telemetry_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.VitiroverTelemetry)
  ))
_sym_db.RegisterMessage(VitiroverTelemetry)

MotorData = _reflection.GeneratedProtocolMessageType('MotorData', (_message.Message,), dict(
  DESCRIPTOR = _MOTORDATA,
  __module__ = 'telemetry_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.MotorData)
  ))
_sym_db.RegisterMessage(MotorData)

VitiroverOrder = _reflection.GeneratedProtocolMessageType('VitiroverOrder', (_message.Message,), dict(
  DESCRIPTOR = _VITIROVERORDER,
  __module__ = 'telemetry_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.VitiroverOrder)
  ))
_sym_db.RegisterMessage(VitiroverOrder)

VitiroverHighLevelOrder = _reflection.GeneratedProtocolMessageType('VitiroverHighLevelOrder', (_message.Message,), dict(
  DESCRIPTOR = _VITIROVERHIGHLEVELORDER,
  __module__ = 'telemetry_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.VitiroverHighLevelOrder)
  ))
_sym_db.RegisterMessage(VitiroverHighLevelOrder)

VitiroverLowLevelOrder = _reflection.GeneratedProtocolMessageType('VitiroverLowLevelOrder', (_message.Message,), dict(
  DESCRIPTOR = _VITIROVERLOWLEVELORDER,
  __module__ = 'telemetry_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.VitiroverLowLevelOrder)
  ))
_sym_db.RegisterMessage(VitiroverLowLevelOrder)


# @@protoc_insertion_point(module_scope)