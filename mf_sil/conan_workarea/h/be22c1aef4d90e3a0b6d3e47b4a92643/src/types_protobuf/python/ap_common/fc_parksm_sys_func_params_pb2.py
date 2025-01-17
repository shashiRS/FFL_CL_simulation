# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_common/fc_parksm_sys_func_params.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_common/fc_parksm_sys_func_params.proto',
  package='pb.ap_common.fc_parksm_sys_func_params',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n)ap_common/fc_parksm_sys_func_params.proto\x12&pb.ap_common.fc_parksm_sys_func_params\x1a\x17\x65\x63o/signal_header.proto\"\xe9\x05\n\x19\x46\x43_PARKSM_Sys_Func_Params\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x1f\n\x16\x41P_G_ESC_TIME_THRESH_S\x18\x8a\x18 \x01(\x02\x12\x1c\n\x13\x41P_G_MAX_REM_DIST_M\x18\xaa\x06 \x01(\x02\x12\x1d\n\x14\x41P_G_MAX_IDLE_TIME_S\x18\xfc\x03 \x01(\x02\x12%\n\x1c\x41P_G_ROLLED_DIST_IN_THRESH_M\x18\xaa\x1e \x01(\x02\x12&\n\x1d\x41P_G_ROLLED_DIST_OUT_THRESH_M\x18\xeb\x15 \x01(\x02\x12#\n\x1a\x41P_G_V_SCANNING_THRESH_MPS\x18\x8e\t \x01(\x02\x12$\n\x1b\x41P_G_V_START_AVG_THRESH_MPS\x18\x9d\t \x01(\x02\x12\x1f\n\x16\x41P_G_EBA_TIME_THRESH_S\x18\xfd\x15 \x01(\x02\x12 \n\x17\x41P_G_RCTA_TIME_THRESH_S\x18\xd4\x18 \x01(\x02\x12\x1f\n\x16\x41P_G_TCS_TIME_THRESH_S\x18\xe9\x1e \x01(\x02\x12\x1f\n\x16\x41P_G_ABS_TIME_THRESH_S\x18\xa6\x14 \x01(\x02\x12\x1f\n\x16\x41P_G_EBD_TIME_THRESH_S\x18\xb3\x1f \x01(\x02\x12\"\n\x19\x41P_G_THROTTLE_THRESH_PERC\x18\xfd\x01 \x01(\x02\x12!\n\x18\x41P_G_MAX_HANDOVER_TIME_S\x18\xee\x17 \x01(\x02\x12\'\n\x1e\x41P_G_STEERING_TORQUE_THRESH_NM\x18\xc8\x1b \x01(\x02\x12!\n\x18\x41P_G_MAX_RM_DRIVE_DIST_M\x18\xb8\r \x01(\x02\x12!\n\x18\x41P_G_PERC_HMI_DIST_BAR_M\x18\xb6\r \x01(\x02\x12%\n\x1c\x41P_G_ROLLED_DIST_RA_THRESH_M\x18\xd2\r \x01(\x02\"x\n$FC_PARKSM_Sys_Func_Params_array_port\x12P\n\x04\x64\x61ta\x18\xbc\x02 \x03(\x0b\x32\x41.pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_FC_PARKSM_SYS_FUNC_PARAMS = _descriptor.Descriptor(
  name='FC_PARKSM_Sys_Func_Params',
  full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_ESC_TIME_THRESH_S', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_ESC_TIME_THRESH_S', index=2,
      number=3082, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_MAX_REM_DIST_M', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_MAX_REM_DIST_M', index=3,
      number=810, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_MAX_IDLE_TIME_S', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_MAX_IDLE_TIME_S', index=4,
      number=508, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_ROLLED_DIST_IN_THRESH_M', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_ROLLED_DIST_IN_THRESH_M', index=5,
      number=3882, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_ROLLED_DIST_OUT_THRESH_M', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_ROLLED_DIST_OUT_THRESH_M', index=6,
      number=2795, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_V_SCANNING_THRESH_MPS', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_V_SCANNING_THRESH_MPS', index=7,
      number=1166, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_V_START_AVG_THRESH_MPS', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_V_START_AVG_THRESH_MPS', index=8,
      number=1181, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_EBA_TIME_THRESH_S', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_EBA_TIME_THRESH_S', index=9,
      number=2813, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_RCTA_TIME_THRESH_S', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_RCTA_TIME_THRESH_S', index=10,
      number=3156, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_TCS_TIME_THRESH_S', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_TCS_TIME_THRESH_S', index=11,
      number=3945, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_ABS_TIME_THRESH_S', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_ABS_TIME_THRESH_S', index=12,
      number=2598, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_EBD_TIME_THRESH_S', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_EBD_TIME_THRESH_S', index=13,
      number=4019, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_THROTTLE_THRESH_PERC', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_THROTTLE_THRESH_PERC', index=14,
      number=253, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_MAX_HANDOVER_TIME_S', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_MAX_HANDOVER_TIME_S', index=15,
      number=3054, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_STEERING_TORQUE_THRESH_NM', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_STEERING_TORQUE_THRESH_NM', index=16,
      number=3528, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_MAX_RM_DRIVE_DIST_M', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_MAX_RM_DRIVE_DIST_M', index=17,
      number=1720, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_PERC_HMI_DIST_BAR_M', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_PERC_HMI_DIST_BAR_M', index=18,
      number=1718, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_ROLLED_DIST_RA_THRESH_M', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params.AP_G_ROLLED_DIST_RA_THRESH_M', index=19,
      number=1746, type=2, cpp_type=6, label=1,
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
  serialized_start=111,
  serialized_end=856,
)


_FC_PARKSM_SYS_FUNC_PARAMS_ARRAY_PORT = _descriptor.Descriptor(
  name='FC_PARKSM_Sys_Func_Params_array_port',
  full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params_array_port.data', index=0,
      number=316, type=11, cpp_type=10, label=3,
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
  serialized_start=858,
  serialized_end=978,
)

_FC_PARKSM_SYS_FUNC_PARAMS.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_FC_PARKSM_SYS_FUNC_PARAMS_ARRAY_PORT.fields_by_name['data'].message_type = _FC_PARKSM_SYS_FUNC_PARAMS
DESCRIPTOR.message_types_by_name['FC_PARKSM_Sys_Func_Params'] = _FC_PARKSM_SYS_FUNC_PARAMS
DESCRIPTOR.message_types_by_name['FC_PARKSM_Sys_Func_Params_array_port'] = _FC_PARKSM_SYS_FUNC_PARAMS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FC_PARKSM_Sys_Func_Params = _reflection.GeneratedProtocolMessageType('FC_PARKSM_Sys_Func_Params', (_message.Message,), {
  'DESCRIPTOR' : _FC_PARKSM_SYS_FUNC_PARAMS,
  '__module__' : 'ap_common.fc_parksm_sys_func_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params)
  })
_sym_db.RegisterMessage(FC_PARKSM_Sys_Func_Params)

FC_PARKSM_Sys_Func_Params_array_port = _reflection.GeneratedProtocolMessageType('FC_PARKSM_Sys_Func_Params_array_port', (_message.Message,), {
  'DESCRIPTOR' : _FC_PARKSM_SYS_FUNC_PARAMS_ARRAY_PORT,
  '__module__' : 'ap_common.fc_parksm_sys_func_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_common.fc_parksm_sys_func_params.FC_PARKSM_Sys_Func_Params_array_port)
  })
_sym_db.RegisterMessage(FC_PARKSM_Sys_Func_Params_array_port)


# @@protoc_insertion_point(module_scope)
