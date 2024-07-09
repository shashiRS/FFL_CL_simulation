# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_common/sys_func_params.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_common/sys_func_params.proto',
  package='pb.ap_common.sys_func_params',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1f\x61p_common/sys_func_params.proto\x12\x1cpb.ap_common.sys_func_params\x1a\x17\x65\x63o/signal_header.proto\"\xe0\x02\n\x0fSys_Func_Params\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x1b\n\x12\x41P_G_MAX_AVG_V_MPS\x18\xf6\x01 \x01(\x02\x12!\n\x18\x41P_G_MAX_WAIT_TIME_REM_S\x18\xef\x1c \x01(\x02\x12\x1b\n\x12\x41P_G_REM_MAX_V_MPS\x18\xd9\t \x01(\x02\x12\'\n\x1e\x41P_G_VARIANT_SEMI_AP_ACTIVE_NU\x18\xd1\x06 \x01(\x08\x12(\n\x1f\x41P_G_SEMI_AP_MAX_SPEED_WARN_MPS\x18\xd4\x04 \x01(\x02\x12%\n\x1c\x41P_G_STANDSTILL_V_THRESH_MPS\x18\xa4\x03 \x01(\x02\x12#\n\x1a\x41P_G_WS_OVERSHOOT_LENGTH_M\x18\xbe\x11 \x01(\x02\"Z\n\x1aSys_Func_Params_array_port\x12<\n\x04\x64\x61ta\x18\xbd\t \x03(\x0b\x32-.pb.ap_common.sys_func_params.Sys_Func_Params'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_SYS_FUNC_PARAMS = _descriptor.Descriptor(
  name='Sys_Func_Params',
  full_name='pb.ap_common.sys_func_params.Sys_Func_Params',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_common.sys_func_params.Sys_Func_Params.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_common.sys_func_params.Sys_Func_Params.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_MAX_AVG_V_MPS', full_name='pb.ap_common.sys_func_params.Sys_Func_Params.AP_G_MAX_AVG_V_MPS', index=2,
      number=246, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_MAX_WAIT_TIME_REM_S', full_name='pb.ap_common.sys_func_params.Sys_Func_Params.AP_G_MAX_WAIT_TIME_REM_S', index=3,
      number=3695, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_REM_MAX_V_MPS', full_name='pb.ap_common.sys_func_params.Sys_Func_Params.AP_G_REM_MAX_V_MPS', index=4,
      number=1241, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_VARIANT_SEMI_AP_ACTIVE_NU', full_name='pb.ap_common.sys_func_params.Sys_Func_Params.AP_G_VARIANT_SEMI_AP_ACTIVE_NU', index=5,
      number=849, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_SEMI_AP_MAX_SPEED_WARN_MPS', full_name='pb.ap_common.sys_func_params.Sys_Func_Params.AP_G_SEMI_AP_MAX_SPEED_WARN_MPS', index=6,
      number=596, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_STANDSTILL_V_THRESH_MPS', full_name='pb.ap_common.sys_func_params.Sys_Func_Params.AP_G_STANDSTILL_V_THRESH_MPS', index=7,
      number=420, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_G_WS_OVERSHOOT_LENGTH_M', full_name='pb.ap_common.sys_func_params.Sys_Func_Params.AP_G_WS_OVERSHOOT_LENGTH_M', index=8,
      number=2238, type=2, cpp_type=6, label=1,
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
  serialized_start=91,
  serialized_end=443,
)


_SYS_FUNC_PARAMS_ARRAY_PORT = _descriptor.Descriptor(
  name='Sys_Func_Params_array_port',
  full_name='pb.ap_common.sys_func_params.Sys_Func_Params_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_common.sys_func_params.Sys_Func_Params_array_port.data', index=0,
      number=1213, type=11, cpp_type=10, label=3,
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
  serialized_start=445,
  serialized_end=535,
)

_SYS_FUNC_PARAMS.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_SYS_FUNC_PARAMS_ARRAY_PORT.fields_by_name['data'].message_type = _SYS_FUNC_PARAMS
DESCRIPTOR.message_types_by_name['Sys_Func_Params'] = _SYS_FUNC_PARAMS
DESCRIPTOR.message_types_by_name['Sys_Func_Params_array_port'] = _SYS_FUNC_PARAMS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Sys_Func_Params = _reflection.GeneratedProtocolMessageType('Sys_Func_Params', (_message.Message,), {
  'DESCRIPTOR' : _SYS_FUNC_PARAMS,
  '__module__' : 'ap_common.sys_func_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_common.sys_func_params.Sys_Func_Params)
  })
_sym_db.RegisterMessage(Sys_Func_Params)

Sys_Func_Params_array_port = _reflection.GeneratedProtocolMessageType('Sys_Func_Params_array_port', (_message.Message,), {
  'DESCRIPTOR' : _SYS_FUNC_PARAMS_ARRAY_PORT,
  '__module__' : 'ap_common.sys_func_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_common.sys_func_params.Sys_Func_Params_array_port)
  })
_sym_db.RegisterMessage(Sys_Func_Params_array_port)


# @@protoc_insertion_point(module_scope)