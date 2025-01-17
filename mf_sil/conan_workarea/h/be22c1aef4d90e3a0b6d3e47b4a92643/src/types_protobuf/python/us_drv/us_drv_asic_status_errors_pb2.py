# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_asic_status_errors.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from us_drv import us_drv_error_status_pb2 as us__drv_dot_us__drv__error__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_asic_status_errors.proto',
  package='pb.us_drv.us_drv_asic_status_errors',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n&us_drv/us_drv_asic_status_errors.proto\x12#pb.us_drv.us_drv_asic_status_errors\x1a us_drv/us_drv_error_status.proto\"\xc1\x04\n\x15UsDrvAsicStatusErrors\x12I\n\x0f\x61sicInitFailure\x18\xcd\x0f \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12\x1f\n\x16\x61sicInitFailureDetails\x18\xa4\n \x01(\r\x12I\n\x0f\x61sicInitTimeout\x18\xc3\x10 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12I\n\x0f\x61sicCommTimeout\x18\xe9\x1a \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12G\n\rramBistFailed\x18\xd9\x0e \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12I\n\x0foverTemperature\x18\x97\x04 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12I\n\x0fvccUnderVoltage\x18\xe6\x1e \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12G\n\rclkrefFailure\x18\x8a\x01 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\"m\n UsDrvAsicStatusErrors_array_port\x12I\n\x04\x64\x61ta\x18\xf6\n \x03(\x0b\x32:.pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors'
  ,
  dependencies=[us__drv_dot_us__drv__error__status__pb2.DESCRIPTOR,])




_USDRVASICSTATUSERRORS = _descriptor.Descriptor(
  name='UsDrvAsicStatusErrors',
  full_name='pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='asicInitFailure', full_name='pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors.asicInitFailure', index=0,
      number=1997, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='asicInitFailureDetails', full_name='pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors.asicInitFailureDetails', index=1,
      number=1316, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='asicInitTimeout', full_name='pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors.asicInitTimeout', index=2,
      number=2115, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='asicCommTimeout', full_name='pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors.asicCommTimeout', index=3,
      number=3433, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ramBistFailed', full_name='pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors.ramBistFailed', index=4,
      number=1881, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='overTemperature', full_name='pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors.overTemperature', index=5,
      number=535, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='vccUnderVoltage', full_name='pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors.vccUnderVoltage', index=6,
      number=3942, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='clkrefFailure', full_name='pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors.clkrefFailure', index=7,
      number=138, type=14, cpp_type=8, label=1,
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
  serialized_start=114,
  serialized_end=691,
)


_USDRVASICSTATUSERRORS_ARRAY_PORT = _descriptor.Descriptor(
  name='UsDrvAsicStatusErrors_array_port',
  full_name='pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors_array_port.data', index=0,
      number=1398, type=11, cpp_type=10, label=3,
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
  serialized_start=693,
  serialized_end=802,
)

_USDRVASICSTATUSERRORS.fields_by_name['asicInitFailure'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVASICSTATUSERRORS.fields_by_name['asicInitTimeout'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVASICSTATUSERRORS.fields_by_name['asicCommTimeout'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVASICSTATUSERRORS.fields_by_name['ramBistFailed'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVASICSTATUSERRORS.fields_by_name['overTemperature'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVASICSTATUSERRORS.fields_by_name['vccUnderVoltage'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVASICSTATUSERRORS.fields_by_name['clkrefFailure'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVASICSTATUSERRORS_ARRAY_PORT.fields_by_name['data'].message_type = _USDRVASICSTATUSERRORS
DESCRIPTOR.message_types_by_name['UsDrvAsicStatusErrors'] = _USDRVASICSTATUSERRORS
DESCRIPTOR.message_types_by_name['UsDrvAsicStatusErrors_array_port'] = _USDRVASICSTATUSERRORS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsDrvAsicStatusErrors = _reflection.GeneratedProtocolMessageType('UsDrvAsicStatusErrors', (_message.Message,), {
  'DESCRIPTOR' : _USDRVASICSTATUSERRORS,
  '__module__' : 'us_drv.us_drv_asic_status_errors_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors)
  })
_sym_db.RegisterMessage(UsDrvAsicStatusErrors)

UsDrvAsicStatusErrors_array_port = _reflection.GeneratedProtocolMessageType('UsDrvAsicStatusErrors_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USDRVASICSTATUSERRORS_ARRAY_PORT,
  '__module__' : 'us_drv.us_drv_asic_status_errors_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_asic_status_errors.UsDrvAsicStatusErrors_array_port)
  })
_sym_db.RegisterMessage(UsDrvAsicStatusErrors_array_port)


# @@protoc_insertion_point(module_scope)
