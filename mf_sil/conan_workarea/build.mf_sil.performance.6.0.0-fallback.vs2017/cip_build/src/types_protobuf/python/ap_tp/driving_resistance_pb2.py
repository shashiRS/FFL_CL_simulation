# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/driving_resistance.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ap_tp import driving_resistance_type_pb2 as ap__tp_dot_driving__resistance__type__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/driving_resistance.proto',
  package='pb.ap_tp.driving_resistance',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1e\x61p_tp/driving_resistance.proto\x12\x1bpb.ap_tp.driving_resistance\x1a#ap_tp/driving_resistance_type.proto\"s\n\x11\x44rivingResistance\x12\x13\n\ndistance_m\x18\x96\n \x01(\x02\x12I\n\x07type_nu\x18\xd6\x05 \x01(\x0e\x32\x37.pb.ap_tp.driving_resistance_type.DrivingResistanceType\"]\n\x1c\x44rivingResistance_array_port\x12=\n\x04\x64\x61ta\x18\xdd\x01 \x03(\x0b\x32..pb.ap_tp.driving_resistance.DrivingResistance'
  ,
  dependencies=[ap__tp_dot_driving__resistance__type__pb2.DESCRIPTOR,])




_DRIVINGRESISTANCE = _descriptor.Descriptor(
  name='DrivingResistance',
  full_name='pb.ap_tp.driving_resistance.DrivingResistance',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='distance_m', full_name='pb.ap_tp.driving_resistance.DrivingResistance.distance_m', index=0,
      number=1302, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='type_nu', full_name='pb.ap_tp.driving_resistance.DrivingResistance.type_nu', index=1,
      number=726, type=14, cpp_type=8, label=1,
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
  serialized_start=100,
  serialized_end=215,
)


_DRIVINGRESISTANCE_ARRAY_PORT = _descriptor.Descriptor(
  name='DrivingResistance_array_port',
  full_name='pb.ap_tp.driving_resistance.DrivingResistance_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_tp.driving_resistance.DrivingResistance_array_port.data', index=0,
      number=221, type=11, cpp_type=10, label=3,
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
  serialized_start=217,
  serialized_end=310,
)

_DRIVINGRESISTANCE.fields_by_name['type_nu'].enum_type = ap__tp_dot_driving__resistance__type__pb2._DRIVINGRESISTANCETYPE
_DRIVINGRESISTANCE_ARRAY_PORT.fields_by_name['data'].message_type = _DRIVINGRESISTANCE
DESCRIPTOR.message_types_by_name['DrivingResistance'] = _DRIVINGRESISTANCE
DESCRIPTOR.message_types_by_name['DrivingResistance_array_port'] = _DRIVINGRESISTANCE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

DrivingResistance = _reflection.GeneratedProtocolMessageType('DrivingResistance', (_message.Message,), {
  'DESCRIPTOR' : _DRIVINGRESISTANCE,
  '__module__' : 'ap_tp.driving_resistance_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.driving_resistance.DrivingResistance)
  })
_sym_db.RegisterMessage(DrivingResistance)

DrivingResistance_array_port = _reflection.GeneratedProtocolMessageType('DrivingResistance_array_port', (_message.Message,), {
  'DESCRIPTOR' : _DRIVINGRESISTANCE_ARRAY_PORT,
  '__module__' : 'ap_tp.driving_resistance_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.driving_resistance.DrivingResistance_array_port)
  })
_sym_db.RegisterMessage(DrivingResistance_array_port)


# @@protoc_insertion_point(module_scope)
