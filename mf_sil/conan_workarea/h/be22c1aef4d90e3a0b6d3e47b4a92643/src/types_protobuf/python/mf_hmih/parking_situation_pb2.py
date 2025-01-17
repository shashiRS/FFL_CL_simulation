# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/parking_situation.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from mf_hmih import parking_situation_sides_pb2 as mf__hmih_dot_parking__situation__sides__pb2
from mf_hmih import parking_situation_front_rear_pb2 as mf__hmih_dot_parking__situation__front__rear__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/parking_situation.proto',
  package='pb.mf_hmih.parking_situation',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1fmf_hmih/parking_situation.proto\x12\x1cpb.mf_hmih.parking_situation\x1a%mf_hmih/parking_situation_sides.proto\x1a*mf_hmih/parking_situation_front_rear.proto\"\xce\x02\n\x10ParkingSituation\x12H\n\x04left\x18\xdd\x06 \x01(\x0b\x32\x39.pb.mf_hmih.parking_situation_sides.ParkingSituationSides\x12I\n\x05right\x18\xe3\x1b \x01(\x0b\x32\x39.pb.mf_hmih.parking_situation_sides.ParkingSituationSides\x12R\n\x05\x66ront\x18\xd6\x07 \x01(\x0b\x32\x42.pb.mf_hmih.parking_situation_front_rear.ParkingSituationFrontRear\x12Q\n\x04rear\x18\x96\t \x01(\x0b\x32\x42.pb.mf_hmih.parking_situation_front_rear.ParkingSituationFrontRear\"\\\n\x1bParkingSituation_array_port\x12=\n\x04\x64\x61ta\x18\xf9\x1b \x03(\x0b\x32..pb.mf_hmih.parking_situation.ParkingSituation'
  ,
  dependencies=[mf__hmih_dot_parking__situation__sides__pb2.DESCRIPTOR,mf__hmih_dot_parking__situation__front__rear__pb2.DESCRIPTOR,])




_PARKINGSITUATION = _descriptor.Descriptor(
  name='ParkingSituation',
  full_name='pb.mf_hmih.parking_situation.ParkingSituation',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='left', full_name='pb.mf_hmih.parking_situation.ParkingSituation.left', index=0,
      number=861, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='right', full_name='pb.mf_hmih.parking_situation.ParkingSituation.right', index=1,
      number=3555, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='front', full_name='pb.mf_hmih.parking_situation.ParkingSituation.front', index=2,
      number=982, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rear', full_name='pb.mf_hmih.parking_situation.ParkingSituation.rear', index=3,
      number=1174, type=11, cpp_type=10, label=1,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=149,
  serialized_end=483,
)


_PARKINGSITUATION_ARRAY_PORT = _descriptor.Descriptor(
  name='ParkingSituation_array_port',
  full_name='pb.mf_hmih.parking_situation.ParkingSituation_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.parking_situation.ParkingSituation_array_port.data', index=0,
      number=3577, type=11, cpp_type=10, label=3,
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
  serialized_start=485,
  serialized_end=577,
)

_PARKINGSITUATION.fields_by_name['left'].message_type = mf__hmih_dot_parking__situation__sides__pb2._PARKINGSITUATIONSIDES
_PARKINGSITUATION.fields_by_name['right'].message_type = mf__hmih_dot_parking__situation__sides__pb2._PARKINGSITUATIONSIDES
_PARKINGSITUATION.fields_by_name['front'].message_type = mf__hmih_dot_parking__situation__front__rear__pb2._PARKINGSITUATIONFRONTREAR
_PARKINGSITUATION.fields_by_name['rear'].message_type = mf__hmih_dot_parking__situation__front__rear__pb2._PARKINGSITUATIONFRONTREAR
_PARKINGSITUATION_ARRAY_PORT.fields_by_name['data'].message_type = _PARKINGSITUATION
DESCRIPTOR.message_types_by_name['ParkingSituation'] = _PARKINGSITUATION
DESCRIPTOR.message_types_by_name['ParkingSituation_array_port'] = _PARKINGSITUATION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ParkingSituation = _reflection.GeneratedProtocolMessageType('ParkingSituation', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGSITUATION,
  '__module__' : 'mf_hmih.parking_situation_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.parking_situation.ParkingSituation)
  })
_sym_db.RegisterMessage(ParkingSituation)

ParkingSituation_array_port = _reflection.GeneratedProtocolMessageType('ParkingSituation_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGSITUATION_ARRAY_PORT,
  '__module__' : 'mf_hmih.parking_situation_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.parking_situation.ParkingSituation_array_port)
  })
_sym_db.RegisterMessage(ParkingSituation_array_port)


# @@protoc_insertion_point(module_scope)
