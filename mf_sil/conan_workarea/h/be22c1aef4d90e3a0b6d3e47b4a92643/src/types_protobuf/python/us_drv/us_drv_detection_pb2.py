# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_detection.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from us_drv import us_drv_detection_type_pb2 as us__drv_dot_us__drv__detection__type__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_detection.proto',
  package='pb.us_drv.us_drv_detection',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1dus_drv/us_drv_detection.proto\x12\x1apb.us_drv.us_drv_detection\x1a\"us_drv/us_drv_detection_type.proto\"\xed\x01\n\x0eUsDrvDetection\x12\x11\n\x08sensorId\x18\xf9\x0b \x01(\r\x12K\n\rdetectionType\x18\xb0\x11 \x01(\x0e\x32\x33.pb.us_drv.us_drv_detection_type.UsDrvDetectionType\x12\x12\n\tamplitude\x18\xab\x14 \x01(\r\x12\x18\n\x0fphaseDerivative\x18\x88\x1c \x01(\x11\x12\x13\n\nsyncCnt_ms\x18\x93\t \x01(\r\x12\x1b\n\x12sensorTimestamp_us\x18\xc6\x1d \x01(\r\x12\x1b\n\x12relEcuTimestamp_us\x18\xb4\x14 \x01(\x11\"V\n\x19UsDrvDetection_array_port\x12\x39\n\x04\x64\x61ta\x18\xed\x1b \x03(\x0b\x32*.pb.us_drv.us_drv_detection.UsDrvDetection'
  ,
  dependencies=[us__drv_dot_us__drv__detection__type__pb2.DESCRIPTOR,])




_USDRVDETECTION = _descriptor.Descriptor(
  name='UsDrvDetection',
  full_name='pb.us_drv.us_drv_detection.UsDrvDetection',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='sensorId', full_name='pb.us_drv.us_drv_detection.UsDrvDetection.sensorId', index=0,
      number=1529, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='detectionType', full_name='pb.us_drv.us_drv_detection.UsDrvDetection.detectionType', index=1,
      number=2224, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=15,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='amplitude', full_name='pb.us_drv.us_drv_detection.UsDrvDetection.amplitude', index=2,
      number=2603, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='phaseDerivative', full_name='pb.us_drv.us_drv_detection.UsDrvDetection.phaseDerivative', index=3,
      number=3592, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='syncCnt_ms', full_name='pb.us_drv.us_drv_detection.UsDrvDetection.syncCnt_ms', index=4,
      number=1171, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sensorTimestamp_us', full_name='pb.us_drv.us_drv_detection.UsDrvDetection.sensorTimestamp_us', index=5,
      number=3782, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='relEcuTimestamp_us', full_name='pb.us_drv.us_drv_detection.UsDrvDetection.relEcuTimestamp_us', index=6,
      number=2612, type=17, cpp_type=1, label=1,
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
  serialized_start=98,
  serialized_end=335,
)


_USDRVDETECTION_ARRAY_PORT = _descriptor.Descriptor(
  name='UsDrvDetection_array_port',
  full_name='pb.us_drv.us_drv_detection.UsDrvDetection_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_drv.us_drv_detection.UsDrvDetection_array_port.data', index=0,
      number=3565, type=11, cpp_type=10, label=3,
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
  serialized_start=337,
  serialized_end=423,
)

_USDRVDETECTION.fields_by_name['detectionType'].enum_type = us__drv_dot_us__drv__detection__type__pb2._USDRVDETECTIONTYPE
_USDRVDETECTION_ARRAY_PORT.fields_by_name['data'].message_type = _USDRVDETECTION
DESCRIPTOR.message_types_by_name['UsDrvDetection'] = _USDRVDETECTION
DESCRIPTOR.message_types_by_name['UsDrvDetection_array_port'] = _USDRVDETECTION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsDrvDetection = _reflection.GeneratedProtocolMessageType('UsDrvDetection', (_message.Message,), {
  'DESCRIPTOR' : _USDRVDETECTION,
  '__module__' : 'us_drv.us_drv_detection_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_detection.UsDrvDetection)
  })
_sym_db.RegisterMessage(UsDrvDetection)

UsDrvDetection_array_port = _reflection.GeneratedProtocolMessageType('UsDrvDetection_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USDRVDETECTION_ARRAY_PORT,
  '__module__' : 'us_drv.us_drv_detection_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_detection.UsDrvDetection_array_port)
  })
_sym_db.RegisterMessage(UsDrvDetection_array_port)


# @@protoc_insertion_point(module_scope)
