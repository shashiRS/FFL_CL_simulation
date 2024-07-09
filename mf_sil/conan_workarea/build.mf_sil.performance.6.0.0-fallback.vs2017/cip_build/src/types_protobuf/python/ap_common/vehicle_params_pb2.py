# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_common/vehicle_params.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_common/vehicle_params.proto',
  package='pb.ap_common.vehicle_params',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1e\x61p_common/vehicle_params.proto\x12\x1bpb.ap_common.vehicle_params\x1a\x17\x65\x63o/signal_header.proto\"\xd6\x0c\n\x0eVehicle_Params\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x1c\n\x13\x41P_V_STEER_RATIO_NU\x18\xa4\x1b \x01(\x02\x12%\n\x1c\x41P_V_STEER_LOOKUP_ST_WHL_RAD\x18\xee\n \x03(\x02\x12%\n\x1c\x41P_V_STEER_LOOKUP_IN_WHL_RAD\x18\xdf\x12 \x03(\x02\x12&\n\x1d\x41P_V_STEER_LOOKUP_OUT_WHL_RAD\x18\x9d\x1d \x03(\x02\x12&\n\x1d\x41P_V_STEER_LOOKUP_CTR_WHL_RAD\x18\x99\x11 \x03(\x02\x12\x19\n\x10\x41P_V_WHEELBASE_M\x18\xeb\n \x01(\x02\x12\x1d\n\x14\x41P_V_OVERHANG_REAR_M\x18\x92\x06 \x01(\x02\x12\x16\n\rAP_V_LENGTH_M\x18\xc9\x1b \x01(\x02\x12\x15\n\x0c\x41P_V_WIDTH_M\x18\xb7\x0b \x01(\x02\x12&\n\x1d\x41P_V_WHEEL_NUMBER_OF_TEETH_NU\x18\xb8\x03 \x01(\r\x12(\n\x1f\x41P_V_TYRE_CIRCUMFERENCE_FRONT_M\x18\xb2\x0c \x01(\x02\x12\'\n\x1e\x41P_V_TYRE_CIRCUMFERENCE_REAR_M\x18\xf2\x1b \x01(\x02\x12$\n\x1b\x41P_V_NUM_STANDARD_SHAPE_PTS\x18\x95\x1b \x01(\r\x12 \n\x17\x41P_V_STANDARD_SHAPE_X_M\x18\xe2\x0f \x03(\x02\x12 \n\x17\x41P_V_STANDARD_SHAPE_Y_M\x18\xd2\x01 \x03(\x02\x12\x1e\n\x15\x41P_V_NUM_BOUNDING_PTS\x18\xd4\x14 \x01(\r\x12\x1d\n\x14\x41P_V_BOUNDINGBOX_X_M\x18\xaf\x08 \x03(\x02\x12\x1d\n\x14\x41P_V_BOUNDINGBOX_Y_M\x18\x9f\x06 \x03(\x02\x12\x1b\n\x12\x41P_V_TRACK_FRONT_M\x18\xe5\x12 \x01(\x02\x12\x1a\n\x11\x41P_V_TRACK_REAR_M\x18\xab\x02 \x01(\x02\x12\"\n\x19\x41P_V_MIRROR_SHAPE_SIZE_NU\x18\xda\x1b \x01(\r\x12!\n\x18\x41P_V_HITCH_SHAPE_SIZE_NU\x18\x8c\x1c \x01(\r\x12!\n\x18\x41P_V_WHEEL_SHAPE_SIZE_NU\x18\xab\x03 \x01(\r\x12#\n\x1a\x41P_V_LEFT_MIRROR_SHAPE_X_M\x18\xe0\x1d \x03(\x02\x12#\n\x1a\x41P_V_LEFT_MIRROR_SHAPE_Y_M\x18\xd0\x13 \x03(\x02\x12$\n\x1b\x41P_V_RIGHT_MIRROR_SHAPE_X_M\x18\x99\r \x03(\x02\x12$\n\x1b\x41P_V_RIGHT_MIRROR_SHAPE_Y_M\x18\xa9\x03 \x03(\x02\x12\x1d\n\x14\x41P_V_HITCH_SHAPE_X_M\x18\xf4\x0e \x03(\x02\x12\x1c\n\x14\x41P_V_HITCH_SHAPE_Y_M\x18\x44 \x03(\x02\x12 \n\x17\x41P_V_FL_WHEEL_SHAPE_X_M\x18\xeb\x0e \x03(\x02\x12\x1f\n\x17\x41P_V_FL_WHEEL_SHAPE_Y_M\x18[ \x03(\x02\x12 \n\x17\x41P_V_RL_WHEEL_SHAPE_X_M\x18\xb3\n \x03(\x02\x12 \n\x17\x41P_V_RL_WHEEL_SHAPE_Y_M\x18\x83\x04 \x03(\x02\x12 \n\x17\x41P_V_RR_WHEEL_SHAPE_X_M\x18\xee\x1c \x03(\x02\x12 \n\x17\x41P_V_RR_WHEEL_SHAPE_Y_M\x18\xde\x12 \x03(\x02\x12 \n\x17\x41P_V_FR_WHEEL_SHAPE_X_M\x18\xb6\x18 \x03(\x02\x12 \n\x17\x41P_V_FR_WHEEL_SHAPE_Y_M\x18\x86\x16 \x03(\x02\x12%\n\x1c\x41P_V_MAX_STEER_ANG_VEL_RADPS\x18\xfb\x11 \x01(\x02\x12&\n\x1d\x41P_V_COMF_STEER_ANG_VEL_RADPS\x18\x94\x17 \x01(\x02\x12%\n\x1c\x41P_V_STEER_ANG_TO_YAW_ANG_NU\x18\xd4\r \x01(\x02\x12$\n\x1b\x41P_V_STEER_POLY_OUT_WHL_RAD\x18\xc6\x04 \x03(\x02\x12$\n\x1b\x41P_V_STEER_POLY_CTR_WHL_RAD\x18\xc2\x08 \x03(\x02\x12#\n\x1a\x41P_V_STEER_POLY_IN_WHL_RAD\x18\xd4\x13 \x03(\x02\x12\x1d\n\x14\x41P_V_MAX_INFL_DIST_M\x18\x9a\x13 \x01(\x02\"W\n\x19Vehicle_Params_array_port\x12:\n\x04\x64\x61ta\x18\xcb\x02 \x03(\x0b\x32+.pb.ap_common.vehicle_params.Vehicle_Params'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_VEHICLE_PARAMS = _descriptor.Descriptor(
  name='Vehicle_Params',
  full_name='pb.ap_common.vehicle_params.Vehicle_Params',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_common.vehicle_params.Vehicle_Params.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_common.vehicle_params.Vehicle_Params.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_STEER_RATIO_NU', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_STEER_RATIO_NU', index=2,
      number=3492, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_STEER_LOOKUP_ST_WHL_RAD', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_STEER_LOOKUP_ST_WHL_RAD', index=3,
      number=1390, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_STEER_LOOKUP_IN_WHL_RAD', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_STEER_LOOKUP_IN_WHL_RAD', index=4,
      number=2399, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_STEER_LOOKUP_OUT_WHL_RAD', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_STEER_LOOKUP_OUT_WHL_RAD', index=5,
      number=3741, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_STEER_LOOKUP_CTR_WHL_RAD', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_STEER_LOOKUP_CTR_WHL_RAD', index=6,
      number=2201, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_WHEELBASE_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_WHEELBASE_M', index=7,
      number=1387, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_OVERHANG_REAR_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_OVERHANG_REAR_M', index=8,
      number=786, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_LENGTH_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_LENGTH_M', index=9,
      number=3529, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_WIDTH_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_WIDTH_M', index=10,
      number=1463, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_WHEEL_NUMBER_OF_TEETH_NU', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_WHEEL_NUMBER_OF_TEETH_NU', index=11,
      number=440, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_TYRE_CIRCUMFERENCE_FRONT_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_TYRE_CIRCUMFERENCE_FRONT_M', index=12,
      number=1586, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_TYRE_CIRCUMFERENCE_REAR_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_TYRE_CIRCUMFERENCE_REAR_M', index=13,
      number=3570, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_NUM_STANDARD_SHAPE_PTS', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_NUM_STANDARD_SHAPE_PTS', index=14,
      number=3477, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_STANDARD_SHAPE_X_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_STANDARD_SHAPE_X_M', index=15,
      number=2018, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_STANDARD_SHAPE_Y_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_STANDARD_SHAPE_Y_M', index=16,
      number=210, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_NUM_BOUNDING_PTS', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_NUM_BOUNDING_PTS', index=17,
      number=2644, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_BOUNDINGBOX_X_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_BOUNDINGBOX_X_M', index=18,
      number=1071, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_BOUNDINGBOX_Y_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_BOUNDINGBOX_Y_M', index=19,
      number=799, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_TRACK_FRONT_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_TRACK_FRONT_M', index=20,
      number=2405, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_TRACK_REAR_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_TRACK_REAR_M', index=21,
      number=299, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_MIRROR_SHAPE_SIZE_NU', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_MIRROR_SHAPE_SIZE_NU', index=22,
      number=3546, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_HITCH_SHAPE_SIZE_NU', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_HITCH_SHAPE_SIZE_NU', index=23,
      number=3596, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_WHEEL_SHAPE_SIZE_NU', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_WHEEL_SHAPE_SIZE_NU', index=24,
      number=427, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_LEFT_MIRROR_SHAPE_X_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_LEFT_MIRROR_SHAPE_X_M', index=25,
      number=3808, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_LEFT_MIRROR_SHAPE_Y_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_LEFT_MIRROR_SHAPE_Y_M', index=26,
      number=2512, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_RIGHT_MIRROR_SHAPE_X_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_RIGHT_MIRROR_SHAPE_X_M', index=27,
      number=1689, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_RIGHT_MIRROR_SHAPE_Y_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_RIGHT_MIRROR_SHAPE_Y_M', index=28,
      number=425, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_HITCH_SHAPE_X_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_HITCH_SHAPE_X_M', index=29,
      number=1908, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_HITCH_SHAPE_Y_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_HITCH_SHAPE_Y_M', index=30,
      number=68, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_FL_WHEEL_SHAPE_X_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_FL_WHEEL_SHAPE_X_M', index=31,
      number=1899, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_FL_WHEEL_SHAPE_Y_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_FL_WHEEL_SHAPE_Y_M', index=32,
      number=91, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_RL_WHEEL_SHAPE_X_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_RL_WHEEL_SHAPE_X_M', index=33,
      number=1331, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_RL_WHEEL_SHAPE_Y_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_RL_WHEEL_SHAPE_Y_M', index=34,
      number=515, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_RR_WHEEL_SHAPE_X_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_RR_WHEEL_SHAPE_X_M', index=35,
      number=3694, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_RR_WHEEL_SHAPE_Y_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_RR_WHEEL_SHAPE_Y_M', index=36,
      number=2398, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_FR_WHEEL_SHAPE_X_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_FR_WHEEL_SHAPE_X_M', index=37,
      number=3126, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_FR_WHEEL_SHAPE_Y_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_FR_WHEEL_SHAPE_Y_M', index=38,
      number=2822, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_MAX_STEER_ANG_VEL_RADPS', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_MAX_STEER_ANG_VEL_RADPS', index=39,
      number=2299, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_COMF_STEER_ANG_VEL_RADPS', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_COMF_STEER_ANG_VEL_RADPS', index=40,
      number=2964, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_STEER_ANG_TO_YAW_ANG_NU', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_STEER_ANG_TO_YAW_ANG_NU', index=41,
      number=1748, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_STEER_POLY_OUT_WHL_RAD', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_STEER_POLY_OUT_WHL_RAD', index=42,
      number=582, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_STEER_POLY_CTR_WHL_RAD', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_STEER_POLY_CTR_WHL_RAD', index=43,
      number=1090, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_STEER_POLY_IN_WHL_RAD', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_STEER_POLY_IN_WHL_RAD', index=44,
      number=2516, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_MAX_INFL_DIST_M', full_name='pb.ap_common.vehicle_params.Vehicle_Params.AP_V_MAX_INFL_DIST_M', index=45,
      number=2458, type=2, cpp_type=6, label=1,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=89,
  serialized_end=1711,
)


_VEHICLE_PARAMS_ARRAY_PORT = _descriptor.Descriptor(
  name='Vehicle_Params_array_port',
  full_name='pb.ap_common.vehicle_params.Vehicle_Params_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_common.vehicle_params.Vehicle_Params_array_port.data', index=0,
      number=331, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1713,
  serialized_end=1800,
)

_VEHICLE_PARAMS.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_VEHICLE_PARAMS_ARRAY_PORT.fields_by_name['data'].message_type = _VEHICLE_PARAMS
DESCRIPTOR.message_types_by_name['Vehicle_Params'] = _VEHICLE_PARAMS
DESCRIPTOR.message_types_by_name['Vehicle_Params_array_port'] = _VEHICLE_PARAMS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Vehicle_Params = _reflection.GeneratedProtocolMessageType('Vehicle_Params', (_message.Message,), {
  'DESCRIPTOR' : _VEHICLE_PARAMS,
  '__module__' : 'ap_common.vehicle_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_common.vehicle_params.Vehicle_Params)
  })
_sym_db.RegisterMessage(Vehicle_Params)

Vehicle_Params_array_port = _reflection.GeneratedProtocolMessageType('Vehicle_Params_array_port', (_message.Message,), {
  'DESCRIPTOR' : _VEHICLE_PARAMS_ARRAY_PORT,
  '__module__' : 'ap_common.vehicle_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_common.vehicle_params.Vehicle_Params_array_port)
  })
_sym_db.RegisterMessage(Vehicle_Params_array_port)


# @@protoc_insertion_point(module_scope)