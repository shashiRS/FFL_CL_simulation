# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/trained_path.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from mf_mempark import localization_status_pb2 as mf__mempark_dot_localization__status__pb2
from mf_mempark import gpssilent_offering_enabled_pb2 as mf__mempark_dot_gpssilent__offering__enabled__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/trained_path.proto',
  package='pb.mf_mempark.trained_path',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1dmf_mempark/trained_path.proto\x12\x1apb.mf_mempark.trained_path\x1a$mf_mempark/localization_status.proto\x1a+mf_mempark/gpssilent_offering_enabled.proto\"\xd7\x01\n\x0bTrainedPath\x12\x0b\n\x02ID\x18\x9a\x1a \x01(\r\x12T\n\x14relocalizationStatus\x18\xfd\x15 \x01(\x0e\x32\x35.pb.mf_mempark.localization_status.LocalizationStatus\x12\x65\n\x18gpsSilentOfferingEnabled\x18\xdb\x0e \x01(\x0e\x32\x42.pb.mf_mempark.gpssilent_offering_enabled.GPSSilentOfferingEnabled\"P\n\x16TrainedPath_array_port\x12\x36\n\x04\x64\x61ta\x18\xfa\x0f \x03(\x0b\x32\'.pb.mf_mempark.trained_path.TrainedPath'
  ,
  dependencies=[mf__mempark_dot_localization__status__pb2.DESCRIPTOR,mf__mempark_dot_gpssilent__offering__enabled__pb2.DESCRIPTOR,])




_TRAINEDPATH = _descriptor.Descriptor(
  name='TrainedPath',
  full_name='pb.mf_mempark.trained_path.TrainedPath',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ID', full_name='pb.mf_mempark.trained_path.TrainedPath.ID', index=0,
      number=3354, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='relocalizationStatus', full_name='pb.mf_mempark.trained_path.TrainedPath.relocalizationStatus', index=1,
      number=2813, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsSilentOfferingEnabled', full_name='pb.mf_mempark.trained_path.TrainedPath.gpsSilentOfferingEnabled', index=2,
      number=1883, type=14, cpp_type=8, label=1,
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
  serialized_start=145,
  serialized_end=360,
)


_TRAINEDPATH_ARRAY_PORT = _descriptor.Descriptor(
  name='TrainedPath_array_port',
  full_name='pb.mf_mempark.trained_path.TrainedPath_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.trained_path.TrainedPath_array_port.data', index=0,
      number=2042, type=11, cpp_type=10, label=3,
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
  serialized_start=362,
  serialized_end=442,
)

_TRAINEDPATH.fields_by_name['relocalizationStatus'].enum_type = mf__mempark_dot_localization__status__pb2._LOCALIZATIONSTATUS
_TRAINEDPATH.fields_by_name['gpsSilentOfferingEnabled'].enum_type = mf__mempark_dot_gpssilent__offering__enabled__pb2._GPSSILENTOFFERINGENABLED
_TRAINEDPATH_ARRAY_PORT.fields_by_name['data'].message_type = _TRAINEDPATH
DESCRIPTOR.message_types_by_name['TrainedPath'] = _TRAINEDPATH
DESCRIPTOR.message_types_by_name['TrainedPath_array_port'] = _TRAINEDPATH_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TrainedPath = _reflection.GeneratedProtocolMessageType('TrainedPath', (_message.Message,), {
  'DESCRIPTOR' : _TRAINEDPATH,
  '__module__' : 'mf_mempark.trained_path_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.trained_path.TrainedPath)
  })
_sym_db.RegisterMessage(TrainedPath)

TrainedPath_array_port = _reflection.GeneratedProtocolMessageType('TrainedPath_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TRAINEDPATH_ARRAY_PORT,
  '__module__' : 'mf_mempark.trained_path_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.trained_path.TrainedPath_array_port)
  })
_sym_db.RegisterMessage(TrainedPath_array_port)


# @@protoc_insertion_point(module_scope)
