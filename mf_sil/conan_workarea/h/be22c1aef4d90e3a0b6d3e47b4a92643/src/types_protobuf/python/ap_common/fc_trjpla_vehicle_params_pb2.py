# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_common/fc_trjpla_vehicle_params.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_common/fc_trjpla_vehicle_params.proto',
  package='pb.ap_common.fc_trjpla_vehicle_params',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n(ap_common/fc_trjpla_vehicle_params.proto\x12%pb.ap_common.fc_trjpla_vehicle_params\x1a\x17\x65\x63o/signal_header.proto\"\xaf\x01\n\x18\x46\x43_TRJPLA_Vehicle_Params\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x1f\n\x16\x41P_V_MIN_SAFETY_DIST_M\x18\xb2\x16 \x01(\x02\x12\x1f\n\x16\x41P_V_MAX_ROBUST_DIST_M\x18\xbd\x0b \x01(\x02\"u\n#FC_TRJPLA_Vehicle_Params_array_port\x12N\n\x04\x64\x61ta\x18\xfd\x1b \x03(\x0b\x32?.pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_FC_TRJPLA_VEHICLE_PARAMS = _descriptor.Descriptor(
  name='FC_TRJPLA_Vehicle_Params',
  full_name='pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_MIN_SAFETY_DIST_M', full_name='pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.AP_V_MIN_SAFETY_DIST_M', index=2,
      number=2866, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='AP_V_MAX_ROBUST_DIST_M', full_name='pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params.AP_V_MAX_ROBUST_DIST_M', index=3,
      number=1469, type=2, cpp_type=6, label=1,
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
  serialized_start=109,
  serialized_end=284,
)


_FC_TRJPLA_VEHICLE_PARAMS_ARRAY_PORT = _descriptor.Descriptor(
  name='FC_TRJPLA_Vehicle_Params_array_port',
  full_name='pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params_array_port.data', index=0,
      number=3581, type=11, cpp_type=10, label=3,
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
  serialized_start=286,
  serialized_end=403,
)

_FC_TRJPLA_VEHICLE_PARAMS.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_FC_TRJPLA_VEHICLE_PARAMS_ARRAY_PORT.fields_by_name['data'].message_type = _FC_TRJPLA_VEHICLE_PARAMS
DESCRIPTOR.message_types_by_name['FC_TRJPLA_Vehicle_Params'] = _FC_TRJPLA_VEHICLE_PARAMS
DESCRIPTOR.message_types_by_name['FC_TRJPLA_Vehicle_Params_array_port'] = _FC_TRJPLA_VEHICLE_PARAMS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FC_TRJPLA_Vehicle_Params = _reflection.GeneratedProtocolMessageType('FC_TRJPLA_Vehicle_Params', (_message.Message,), {
  'DESCRIPTOR' : _FC_TRJPLA_VEHICLE_PARAMS,
  '__module__' : 'ap_common.fc_trjpla_vehicle_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params)
  })
_sym_db.RegisterMessage(FC_TRJPLA_Vehicle_Params)

FC_TRJPLA_Vehicle_Params_array_port = _reflection.GeneratedProtocolMessageType('FC_TRJPLA_Vehicle_Params_array_port', (_message.Message,), {
  'DESCRIPTOR' : _FC_TRJPLA_VEHICLE_PARAMS_ARRAY_PORT,
  '__module__' : 'ap_common.fc_trjpla_vehicle_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_common.fc_trjpla_vehicle_params.FC_TRJPLA_Vehicle_Params_array_port)
  })
_sym_db.RegisterMessage(FC_TRJPLA_Vehicle_Params_array_port)


# @@protoc_insertion_point(module_scope)
