# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/general_warnings.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from mf_hmih import day_time_pb2 as mf__hmih_dot_day__time__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/general_warnings.proto',
  package='pb.mf_hmih.general_warnings',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1emf_hmih/general_warnings.proto\x12\x1bpb.mf_hmih.general_warnings\x1a\x16mf_hmih/day_time.proto\"\xf9\x01\n\x0fGeneralWarnings\x12\x41\n\x1avisualDaytimeEstimation_nu\x18\xbb\x01 \x01(\x0e\x32\x1c.pb.mf_hmih.day_time.DayTime\x12%\n\x1cnarrowParkingSpaceWarning_nu\x18\x99\x1c \x01(\x08\x12*\n!parkingSpaceObstaclesAreMoving_nu\x18\xdb\x0c \x01(\x08\x12%\n\x1cvisualSensorSystemWarning_nu\x18\xc3\x14 \x01(\x08\x12)\n ultrasoundSensorSystemWarning_nu\x18\x80\x01 \x01(\x08\"Y\n\x1aGeneralWarnings_array_port\x12;\n\x04\x64\x61ta\x18\x97\x1a \x03(\x0b\x32,.pb.mf_hmih.general_warnings.GeneralWarnings'
  ,
  dependencies=[mf__hmih_dot_day__time__pb2.DESCRIPTOR,])




_GENERALWARNINGS = _descriptor.Descriptor(
  name='GeneralWarnings',
  full_name='pb.mf_hmih.general_warnings.GeneralWarnings',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='visualDaytimeEstimation_nu', full_name='pb.mf_hmih.general_warnings.GeneralWarnings.visualDaytimeEstimation_nu', index=0,
      number=187, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='narrowParkingSpaceWarning_nu', full_name='pb.mf_hmih.general_warnings.GeneralWarnings.narrowParkingSpaceWarning_nu', index=1,
      number=3609, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='parkingSpaceObstaclesAreMoving_nu', full_name='pb.mf_hmih.general_warnings.GeneralWarnings.parkingSpaceObstaclesAreMoving_nu', index=2,
      number=1627, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='visualSensorSystemWarning_nu', full_name='pb.mf_hmih.general_warnings.GeneralWarnings.visualSensorSystemWarning_nu', index=3,
      number=2627, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ultrasoundSensorSystemWarning_nu', full_name='pb.mf_hmih.general_warnings.GeneralWarnings.ultrasoundSensorSystemWarning_nu', index=4,
      number=128, type=8, cpp_type=7, label=1,
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
  serialized_start=88,
  serialized_end=337,
)


_GENERALWARNINGS_ARRAY_PORT = _descriptor.Descriptor(
  name='GeneralWarnings_array_port',
  full_name='pb.mf_hmih.general_warnings.GeneralWarnings_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.general_warnings.GeneralWarnings_array_port.data', index=0,
      number=3351, type=11, cpp_type=10, label=3,
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
  serialized_start=339,
  serialized_end=428,
)

_GENERALWARNINGS.fields_by_name['visualDaytimeEstimation_nu'].enum_type = mf__hmih_dot_day__time__pb2._DAYTIME
_GENERALWARNINGS_ARRAY_PORT.fields_by_name['data'].message_type = _GENERALWARNINGS
DESCRIPTOR.message_types_by_name['GeneralWarnings'] = _GENERALWARNINGS
DESCRIPTOR.message_types_by_name['GeneralWarnings_array_port'] = _GENERALWARNINGS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

GeneralWarnings = _reflection.GeneratedProtocolMessageType('GeneralWarnings', (_message.Message,), {
  'DESCRIPTOR' : _GENERALWARNINGS,
  '__module__' : 'mf_hmih.general_warnings_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.general_warnings.GeneralWarnings)
  })
_sym_db.RegisterMessage(GeneralWarnings)

GeneralWarnings_array_port = _reflection.GeneratedProtocolMessageType('GeneralWarnings_array_port', (_message.Message,), {
  'DESCRIPTOR' : _GENERALWARNINGS_ARRAY_PORT,
  '__module__' : 'mf_hmih.general_warnings_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.general_warnings.GeneralWarnings_array_port)
  })
_sym_db.RegisterMessage(GeneralWarnings_array_port)


# @@protoc_insertion_point(module_scope)