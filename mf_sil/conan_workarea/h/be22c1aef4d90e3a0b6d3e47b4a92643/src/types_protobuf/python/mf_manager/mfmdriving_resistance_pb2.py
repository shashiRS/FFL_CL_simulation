# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_manager/mfmdriving_resistance.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from mf_manager import mfmdriving_resistance_type_pb2 as mf__manager_dot_mfmdriving__resistance__type__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_manager/mfmdriving_resistance.proto',
  package='pb.mf_manager.mfmdriving_resistance',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n&mf_manager/mfmdriving_resistance.proto\x12#pb.mf_manager.mfmdriving_resistance\x1a+mf_manager/mfmdriving_resistance_type.proto\"\x81\x01\n\x14MFMDrivingResistance\x12\x13\n\ndistance_m\x18\x96\n \x01(\x02\x12T\n\x07type_nu\x18\x8f\x0c \x01(\x0e\x32\x42.pb.mf_manager.mfmdriving_resistance_type.MFMDrivingResistanceType\"k\n\x1fMFMDrivingResistance_array_port\x12H\n\x04\x64\x61ta\x18\xbe\x01 \x03(\x0b\x32\x39.pb.mf_manager.mfmdriving_resistance.MFMDrivingResistance'
  ,
  dependencies=[mf__manager_dot_mfmdriving__resistance__type__pb2.DESCRIPTOR,])




_MFMDRIVINGRESISTANCE = _descriptor.Descriptor(
  name='MFMDrivingResistance',
  full_name='pb.mf_manager.mfmdriving_resistance.MFMDrivingResistance',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='distance_m', full_name='pb.mf_manager.mfmdriving_resistance.MFMDrivingResistance.distance_m', index=0,
      number=1302, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='type_nu', full_name='pb.mf_manager.mfmdriving_resistance.MFMDrivingResistance.type_nu', index=1,
      number=1551, type=14, cpp_type=8, label=1,
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
  serialized_start=125,
  serialized_end=254,
)


_MFMDRIVINGRESISTANCE_ARRAY_PORT = _descriptor.Descriptor(
  name='MFMDrivingResistance_array_port',
  full_name='pb.mf_manager.mfmdriving_resistance.MFMDrivingResistance_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_manager.mfmdriving_resistance.MFMDrivingResistance_array_port.data', index=0,
      number=190, type=11, cpp_type=10, label=3,
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
  serialized_start=256,
  serialized_end=363,
)

_MFMDRIVINGRESISTANCE.fields_by_name['type_nu'].enum_type = mf__manager_dot_mfmdriving__resistance__type__pb2._MFMDRIVINGRESISTANCETYPE
_MFMDRIVINGRESISTANCE_ARRAY_PORT.fields_by_name['data'].message_type = _MFMDRIVINGRESISTANCE
DESCRIPTOR.message_types_by_name['MFMDrivingResistance'] = _MFMDRIVINGRESISTANCE
DESCRIPTOR.message_types_by_name['MFMDrivingResistance_array_port'] = _MFMDRIVINGRESISTANCE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MFMDrivingResistance = _reflection.GeneratedProtocolMessageType('MFMDrivingResistance', (_message.Message,), {
  'DESCRIPTOR' : _MFMDRIVINGRESISTANCE,
  '__module__' : 'mf_manager.mfmdriving_resistance_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.mfmdriving_resistance.MFMDrivingResistance)
  })
_sym_db.RegisterMessage(MFMDrivingResistance)

MFMDrivingResistance_array_port = _reflection.GeneratedProtocolMessageType('MFMDrivingResistance_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MFMDRIVINGRESISTANCE_ARRAY_PORT,
  '__module__' : 'mf_manager.mfmdriving_resistance_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_manager.mfmdriving_resistance.MFMDrivingResistance_array_port)
  })
_sym_db.RegisterMessage(MFMDrivingResistance_array_port)


# @@protoc_insertion_point(module_scope)
