# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/parking_space.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from mf_hmih import poss_orientation_pb2 as mf__hmih_dot_poss__orientation__pb2
from mf_hmih import selected_orientation_pb2 as mf__hmih_dot_selected__orientation__pb2
from mf_hmih import possible_direction_pb2 as mf__hmih_dot_possible__direction__pb2
from mf_hmih import selected_direction_pb2 as mf__hmih_dot_selected__direction__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/parking_space.proto',
  package='pb.mf_hmih.parking_space',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1bmf_hmih/parking_space.proto\x12\x18pb.mf_hmih.parking_space\x1a\x1emf_hmih/poss_orientation.proto\x1a\"mf_hmih/selected_orientation.proto\x1a mf_hmih/possible_direction.proto\x1a mf_hmih/selected_direction.proto\"\xbe\x03\n\x0cParkingSpace\x12\x13\n\nscanned_nu\x18\xd7\x04 \x03(\x08\x12\x10\n\x07\x66ree_nu\x18\xf1\x0c \x03(\x08\x12\x14\n\x0bselected_nu\x18\xe8\x1f \x03(\x08\x12I\n\x12possOrientation_nu\x18\xdd\x05 \x03(\x0e\x32,.pb.mf_hmih.poss_orientation.PossOrientation\x12U\n\x16selectedOrientation_nu\x18\xa7\x1f \x03(\x0e\x32\x34.pb.mf_hmih.selected_orientation.SelectedOrientation\x12K\n\x10possDirection_nu\x18\x92\x14 \x03(\x0e\x32\x30.pb.mf_hmih.possible_direction.PossibleDirection\x12O\n\x14selectedDirection_nu\x18\xca\x07 \x03(\x0e\x32\x30.pb.mf_hmih.selected_direction.SelectedDirection\x12\x12\n\tposeID_nu\x18\xbc\x03 \x03(\r\x12\x1d\n\x14memorizedPoseYaw_rad\x18\xf0\x0e \x01(\x02\"P\n\x17ParkingSpace_array_port\x12\x35\n\x04\x64\x61ta\x18\xf0\x14 \x03(\x0b\x32&.pb.mf_hmih.parking_space.ParkingSpace'
  ,
  dependencies=[mf__hmih_dot_poss__orientation__pb2.DESCRIPTOR,mf__hmih_dot_selected__orientation__pb2.DESCRIPTOR,mf__hmih_dot_possible__direction__pb2.DESCRIPTOR,mf__hmih_dot_selected__direction__pb2.DESCRIPTOR,])




_PARKINGSPACE = _descriptor.Descriptor(
  name='ParkingSpace',
  full_name='pb.mf_hmih.parking_space.ParkingSpace',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='scanned_nu', full_name='pb.mf_hmih.parking_space.ParkingSpace.scanned_nu', index=0,
      number=599, type=8, cpp_type=7, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='free_nu', full_name='pb.mf_hmih.parking_space.ParkingSpace.free_nu', index=1,
      number=1649, type=8, cpp_type=7, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='selected_nu', full_name='pb.mf_hmih.parking_space.ParkingSpace.selected_nu', index=2,
      number=4072, type=8, cpp_type=7, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='possOrientation_nu', full_name='pb.mf_hmih.parking_space.ParkingSpace.possOrientation_nu', index=3,
      number=733, type=14, cpp_type=8, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='selectedOrientation_nu', full_name='pb.mf_hmih.parking_space.ParkingSpace.selectedOrientation_nu', index=4,
      number=4007, type=14, cpp_type=8, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='possDirection_nu', full_name='pb.mf_hmih.parking_space.ParkingSpace.possDirection_nu', index=5,
      number=2578, type=14, cpp_type=8, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='selectedDirection_nu', full_name='pb.mf_hmih.parking_space.ParkingSpace.selectedDirection_nu', index=6,
      number=970, type=14, cpp_type=8, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='poseID_nu', full_name='pb.mf_hmih.parking_space.ParkingSpace.poseID_nu', index=7,
      number=444, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='memorizedPoseYaw_rad', full_name='pb.mf_hmih.parking_space.ParkingSpace.memorizedPoseYaw_rad', index=8,
      number=1904, type=2, cpp_type=6, label=1,
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
  serialized_start=194,
  serialized_end=640,
)


_PARKINGSPACE_ARRAY_PORT = _descriptor.Descriptor(
  name='ParkingSpace_array_port',
  full_name='pb.mf_hmih.parking_space.ParkingSpace_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.parking_space.ParkingSpace_array_port.data', index=0,
      number=2672, type=11, cpp_type=10, label=3,
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
  serialized_start=642,
  serialized_end=722,
)

_PARKINGSPACE.fields_by_name['possOrientation_nu'].enum_type = mf__hmih_dot_poss__orientation__pb2._POSSORIENTATION
_PARKINGSPACE.fields_by_name['selectedOrientation_nu'].enum_type = mf__hmih_dot_selected__orientation__pb2._SELECTEDORIENTATION
_PARKINGSPACE.fields_by_name['possDirection_nu'].enum_type = mf__hmih_dot_possible__direction__pb2._POSSIBLEDIRECTION
_PARKINGSPACE.fields_by_name['selectedDirection_nu'].enum_type = mf__hmih_dot_selected__direction__pb2._SELECTEDDIRECTION
_PARKINGSPACE_ARRAY_PORT.fields_by_name['data'].message_type = _PARKINGSPACE
DESCRIPTOR.message_types_by_name['ParkingSpace'] = _PARKINGSPACE
DESCRIPTOR.message_types_by_name['ParkingSpace_array_port'] = _PARKINGSPACE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ParkingSpace = _reflection.GeneratedProtocolMessageType('ParkingSpace', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGSPACE,
  '__module__' : 'mf_hmih.parking_space_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.parking_space.ParkingSpace)
  })
_sym_db.RegisterMessage(ParkingSpace)

ParkingSpace_array_port = _reflection.GeneratedProtocolMessageType('ParkingSpace_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGSPACE_ARRAY_PORT,
  '__module__' : 'mf_hmih.parking_space_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.parking_space.ParkingSpace_array_port)
  })
_sym_db.RegisterMessage(ParkingSpace_array_port)


# @@protoc_insertion_point(module_scope)
