# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_psm/reset_origin_request_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from lsm_geoml import pose_pod_pb2 as lsm__geoml_dot_pose__pod__pb2
from ap_psm import reset_origin_type_pb2 as ap__psm_dot_reset__origin__type__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_psm/reset_origin_request_port.proto',
  package='pb.ap_psm.reset_origin_request_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n&ap_psm/reset_origin_request_port.proto\x12#pb.ap_psm.reset_origin_request_port\x1a\x18lsm_geoml/pose_pod.proto\x1a\x1e\x61p_psm/reset_origin_type.proto\"\xb3\x01\n\x16ResetOriginRequestPort\x12\x38\n\x0etransformation\x18\xa4\x08 \x01(\x0b\x32\x1f.pb.lsm_geoml.pose_pod.Pose_POD\x12\x45\n\x0eresetOrigin_nu\x18\xdb\x12 \x01(\x0e\x32,.pb.ap_psm.reset_origin_type.ResetOriginType\x12\x18\n\x0fresetCounter_nu\x18\xe8\x11 \x01(\r\"o\n!ResetOriginRequestPort_array_port\x12J\n\x04\x64\x61ta\x18\xd1\x03 \x03(\x0b\x32;.pb.ap_psm.reset_origin_request_port.ResetOriginRequestPort'
  ,
  dependencies=[lsm__geoml_dot_pose__pod__pb2.DESCRIPTOR,ap__psm_dot_reset__origin__type__pb2.DESCRIPTOR,])




_RESETORIGINREQUESTPORT = _descriptor.Descriptor(
  name='ResetOriginRequestPort',
  full_name='pb.ap_psm.reset_origin_request_port.ResetOriginRequestPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='transformation', full_name='pb.ap_psm.reset_origin_request_port.ResetOriginRequestPort.transformation', index=0,
      number=1060, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='resetOrigin_nu', full_name='pb.ap_psm.reset_origin_request_port.ResetOriginRequestPort.resetOrigin_nu', index=1,
      number=2395, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='resetCounter_nu', full_name='pb.ap_psm.reset_origin_request_port.ResetOriginRequestPort.resetCounter_nu', index=2,
      number=2280, type=13, cpp_type=3, label=1,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=138,
  serialized_end=317,
)


_RESETORIGINREQUESTPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='ResetOriginRequestPort_array_port',
  full_name='pb.ap_psm.reset_origin_request_port.ResetOriginRequestPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_psm.reset_origin_request_port.ResetOriginRequestPort_array_port.data', index=0,
      number=465, type=11, cpp_type=10, label=3,
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
  serialized_start=319,
  serialized_end=430,
)

_RESETORIGINREQUESTPORT.fields_by_name['transformation'].message_type = lsm__geoml_dot_pose__pod__pb2._POSE_POD
_RESETORIGINREQUESTPORT.fields_by_name['resetOrigin_nu'].enum_type = ap__psm_dot_reset__origin__type__pb2._RESETORIGINTYPE
_RESETORIGINREQUESTPORT_ARRAY_PORT.fields_by_name['data'].message_type = _RESETORIGINREQUESTPORT
DESCRIPTOR.message_types_by_name['ResetOriginRequestPort'] = _RESETORIGINREQUESTPORT
DESCRIPTOR.message_types_by_name['ResetOriginRequestPort_array_port'] = _RESETORIGINREQUESTPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ResetOriginRequestPort = _reflection.GeneratedProtocolMessageType('ResetOriginRequestPort', (_message.Message,), {
  'DESCRIPTOR' : _RESETORIGINREQUESTPORT,
  '__module__' : 'ap_psm.reset_origin_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm.reset_origin_request_port.ResetOriginRequestPort)
  })
_sym_db.RegisterMessage(ResetOriginRequestPort)

ResetOriginRequestPort_array_port = _reflection.GeneratedProtocolMessageType('ResetOriginRequestPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _RESETORIGINREQUESTPORT_ARRAY_PORT,
  '__module__' : 'ap_psm.reset_origin_request_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_psm.reset_origin_request_port.ResetOriginRequestPort_array_port)
  })
_sym_db.RegisterMessage(ResetOriginRequestPort_array_port)


# @@protoc_insertion_point(module_scope)
