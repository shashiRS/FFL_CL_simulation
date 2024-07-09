# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: cfg_mgr/ecu_coding_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cfg_mgr import ecu_coding_state_pb2 as cfg__mgr_dot_ecu__coding__state__pb2
from cfg_mgr import us_sensor_degradation_type_pb2 as cfg__mgr_dot_us__sensor__degradation__type__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='cfg_mgr/ecu_coding_port.proto',
  package='pb.cfg_mgr.ecu_coding_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1d\x63\x66g_mgr/ecu_coding_port.proto\x12\x1apb.cfg_mgr.ecu_coding_port\x1a\x1e\x63\x66g_mgr/ecu_coding_state.proto\x1a(cfg_mgr/us_sensor_degradation_type.proto\"\xb6\x05\n\rEcuCodingPort\x12\x19\n\x10timestamp_us_u64\x18\xc2\x1c \x01(\x04\x12\x43\n\rcodingState_e\x18\x91\r \x01(\x0e\x32+.pb.cfg_mgr.ecu_coding_state.EcuCodingState\x12\x16\n\risUsSupported\x18\x97\x18 \x01(\x08\x12\x17\n\x0enumOfUsSensors\x18\xbf\x0b \x01(\r\x12\\\n\x13UsDegradationMode_e\x18\x9d\x1e \x01(\x0e\x32>.pb.cfg_mgr.us_sensor_degradation_type.UsSensorDegradationType\x12 \n\x17isPdwFrontOnlySupported\x18\x8e\x1b \x01(\x08\x12\x1f\n\x16isPdwRearOnlySupported\x18\xfd\r \x01(\x08\x12\x1a\n\x11isPdw360Supported\x18\xc4\x03 \x01(\x08\x12$\n\x1bisBrakingFrontOnlySupported\x18\xd7\x06 \x01(\x08\x12\"\n\x1aisBrakingRearOnlySupported\x18< \x01(\x08\x12\x1e\n\x15isBraking360Supported\x18\x8f\x11 \x01(\x08\x12#\n\x1aisSteeringSuggestSupported\x18\xbf\x03 \x01(\x08\x12#\n\x1aisSteeringProtectSupported\x18\xa2\x0c \x01(\x08\x12\x1e\n\x15isWhlProtectSupported\x18\xd0\x17 \x01(\x08\x12\x1e\n\x15isSemiAuParkSupported\x18\xcc\x1f \x01(\x08\x12\x1e\n\x15isFullAuParkSupported\x18\xfa\x06 \x01(\x08\x12\x1e\n\x15isRemoteParkSupported\x18\xcf\x02 \x01(\x08\x12#\n\x1aisBasicGarageParkSupported\x18\xd8\x1d \x01(\x08\"T\n\x18\x45\x63uCodingPort_array_port\x12\x38\n\x04\x64\x61ta\x18\xb2\r \x03(\x0b\x32).pb.cfg_mgr.ecu_coding_port.EcuCodingPort'
  ,
  dependencies=[cfg__mgr_dot_ecu__coding__state__pb2.DESCRIPTOR,cfg__mgr_dot_us__sensor__degradation__type__pb2.DESCRIPTOR,])




_ECUCODINGPORT = _descriptor.Descriptor(
  name='EcuCodingPort',
  full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp_us_u64', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.timestamp_us_u64', index=0,
      number=3650, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='codingState_e', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.codingState_e', index=1,
      number=1681, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isUsSupported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isUsSupported', index=2,
      number=3095, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numOfUsSensors', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.numOfUsSensors', index=3,
      number=1471, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='UsDegradationMode_e', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.UsDegradationMode_e', index=4,
      number=3869, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isPdwFrontOnlySupported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isPdwFrontOnlySupported', index=5,
      number=3470, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isPdwRearOnlySupported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isPdwRearOnlySupported', index=6,
      number=1789, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isPdw360Supported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isPdw360Supported', index=7,
      number=452, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isBrakingFrontOnlySupported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isBrakingFrontOnlySupported', index=8,
      number=855, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isBrakingRearOnlySupported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isBrakingRearOnlySupported', index=9,
      number=60, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isBraking360Supported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isBraking360Supported', index=10,
      number=2191, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isSteeringSuggestSupported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isSteeringSuggestSupported', index=11,
      number=447, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isSteeringProtectSupported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isSteeringProtectSupported', index=12,
      number=1570, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isWhlProtectSupported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isWhlProtectSupported', index=13,
      number=3024, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isSemiAuParkSupported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isSemiAuParkSupported', index=14,
      number=4044, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isFullAuParkSupported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isFullAuParkSupported', index=15,
      number=890, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isRemoteParkSupported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isRemoteParkSupported', index=16,
      number=335, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isBasicGarageParkSupported', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort.isBasicGarageParkSupported', index=17,
      number=3800, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_start=136,
  serialized_end=830,
)


_ECUCODINGPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='EcuCodingPort_array_port',
  full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.cfg_mgr.ecu_coding_port.EcuCodingPort_array_port.data', index=0,
      number=1714, type=11, cpp_type=10, label=3,
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
  serialized_start=832,
  serialized_end=916,
)

_ECUCODINGPORT.fields_by_name['codingState_e'].enum_type = cfg__mgr_dot_ecu__coding__state__pb2._ECUCODINGSTATE
_ECUCODINGPORT.fields_by_name['UsDegradationMode_e'].enum_type = cfg__mgr_dot_us__sensor__degradation__type__pb2._USSENSORDEGRADATIONTYPE
_ECUCODINGPORT_ARRAY_PORT.fields_by_name['data'].message_type = _ECUCODINGPORT
DESCRIPTOR.message_types_by_name['EcuCodingPort'] = _ECUCODINGPORT
DESCRIPTOR.message_types_by_name['EcuCodingPort_array_port'] = _ECUCODINGPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

EcuCodingPort = _reflection.GeneratedProtocolMessageType('EcuCodingPort', (_message.Message,), {
  'DESCRIPTOR' : _ECUCODINGPORT,
  '__module__' : 'cfg_mgr.ecu_coding_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.cfg_mgr.ecu_coding_port.EcuCodingPort)
  })
_sym_db.RegisterMessage(EcuCodingPort)

EcuCodingPort_array_port = _reflection.GeneratedProtocolMessageType('EcuCodingPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _ECUCODINGPORT_ARRAY_PORT,
  '__module__' : 'cfg_mgr.ecu_coding_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.cfg_mgr.ecu_coding_port.EcuCodingPort_array_port)
  })
_sym_db.RegisterMessage(EcuCodingPort_array_port)


# @@protoc_insertion_point(module_scope)
