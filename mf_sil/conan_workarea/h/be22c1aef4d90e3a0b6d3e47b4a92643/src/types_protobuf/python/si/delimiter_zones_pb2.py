# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/delimiter_zones.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from si import quadrilateral_serializable_pb2 as si_dot_quadrilateral__serializable__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/delimiter_zones.proto',
  package='pb.si.delimiter_zones',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x18si/delimiter_zones.proto\x12\x15pb.si.delimiter_zones\x1a#si/quadrilateral_serializable.proto\"\x82\x04\n\x0e\x44\x65limiterZones\x12N\n\x08\x63urbZone\x18\xc5\x1b \x01(\x0b\x32;.pb.si.quadrilateral_serializable.QuadrilateralSerializable\x12N\n\x08roadZone\x18\xf6\x14 \x01(\x0b\x32;.pb.si.quadrilateral_serializable.QuadrilateralSerializable\x12N\n\x08leftZone\x18\x86\r \x01(\x0b\x32;.pb.si.quadrilateral_serializable.QuadrilateralSerializable\x12O\n\trightZone\x18\x98\x03 \x01(\x0b\x32;.pb.si.quadrilateral_serializable.QuadrilateralSerializable\x12P\n\ninsideZone\x18\xe7\x15 \x01(\x0b\x32;.pb.si.quadrilateral_serializable.QuadrilateralSerializable\x12I\n\x03\x61ll\x18\xbf\x16 \x01(\x0b\x32;.pb.si.quadrilateral_serializable.QuadrilateralSerializable\x12\x12\n\tslotId_nu\x18\xdb\x1b \x01(\r\"Q\n\x19\x44\x65limiterZones_array_port\x12\x34\n\x04\x64\x61ta\x18\xee\x1e \x03(\x0b\x32%.pb.si.delimiter_zones.DelimiterZones'
  ,
  dependencies=[si_dot_quadrilateral__serializable__pb2.DESCRIPTOR,])




_DELIMITERZONES = _descriptor.Descriptor(
  name='DelimiterZones',
  full_name='pb.si.delimiter_zones.DelimiterZones',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='curbZone', full_name='pb.si.delimiter_zones.DelimiterZones.curbZone', index=0,
      number=3525, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='roadZone', full_name='pb.si.delimiter_zones.DelimiterZones.roadZone', index=1,
      number=2678, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='leftZone', full_name='pb.si.delimiter_zones.DelimiterZones.leftZone', index=2,
      number=1670, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rightZone', full_name='pb.si.delimiter_zones.DelimiterZones.rightZone', index=3,
      number=408, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='insideZone', full_name='pb.si.delimiter_zones.DelimiterZones.insideZone', index=4,
      number=2791, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='all', full_name='pb.si.delimiter_zones.DelimiterZones.all', index=5,
      number=2879, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='slotId_nu', full_name='pb.si.delimiter_zones.DelimiterZones.slotId_nu', index=6,
      number=3547, type=13, cpp_type=3, label=1,
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
  serialized_start=89,
  serialized_end=603,
)


_DELIMITERZONES_ARRAY_PORT = _descriptor.Descriptor(
  name='DelimiterZones_array_port',
  full_name='pb.si.delimiter_zones.DelimiterZones_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.delimiter_zones.DelimiterZones_array_port.data', index=0,
      number=3950, type=11, cpp_type=10, label=3,
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
  serialized_start=605,
  serialized_end=686,
)

_DELIMITERZONES.fields_by_name['curbZone'].message_type = si_dot_quadrilateral__serializable__pb2._QUADRILATERALSERIALIZABLE
_DELIMITERZONES.fields_by_name['roadZone'].message_type = si_dot_quadrilateral__serializable__pb2._QUADRILATERALSERIALIZABLE
_DELIMITERZONES.fields_by_name['leftZone'].message_type = si_dot_quadrilateral__serializable__pb2._QUADRILATERALSERIALIZABLE
_DELIMITERZONES.fields_by_name['rightZone'].message_type = si_dot_quadrilateral__serializable__pb2._QUADRILATERALSERIALIZABLE
_DELIMITERZONES.fields_by_name['insideZone'].message_type = si_dot_quadrilateral__serializable__pb2._QUADRILATERALSERIALIZABLE
_DELIMITERZONES.fields_by_name['all'].message_type = si_dot_quadrilateral__serializable__pb2._QUADRILATERALSERIALIZABLE
_DELIMITERZONES_ARRAY_PORT.fields_by_name['data'].message_type = _DELIMITERZONES
DESCRIPTOR.message_types_by_name['DelimiterZones'] = _DELIMITERZONES
DESCRIPTOR.message_types_by_name['DelimiterZones_array_port'] = _DELIMITERZONES_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

DelimiterZones = _reflection.GeneratedProtocolMessageType('DelimiterZones', (_message.Message,), {
  'DESCRIPTOR' : _DELIMITERZONES,
  '__module__' : 'si.delimiter_zones_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.delimiter_zones.DelimiterZones)
  })
_sym_db.RegisterMessage(DelimiterZones)

DelimiterZones_array_port = _reflection.GeneratedProtocolMessageType('DelimiterZones_array_port', (_message.Message,), {
  'DESCRIPTOR' : _DELIMITERZONES_ARRAY_PORT,
  '__module__' : 'si.delimiter_zones_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.delimiter_zones.DelimiterZones_array_port)
  })
_sym_db.RegisterMessage(DelimiterZones_array_port)


# @@protoc_insertion_point(module_scope)
