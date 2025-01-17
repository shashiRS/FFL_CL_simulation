# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_spi_comm_errors.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from us_drv import us_drv_error_status_pb2 as us__drv_dot_us__drv__error__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_spi_comm_errors.proto',
  package='pb.us_drv.us_drv_spi_comm_errors',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n#us_drv/us_drv_spi_comm_errors.proto\x12 pb.us_drv.us_drv_spi_comm_errors\x1a us_drv/us_drv_error_status.proto\"\xc1\x02\n\x12UsDrvSpiCommErrors\x12K\n\x11prevCmdIncomplete\x18\xfd\x11 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12J\n\x10\x63rcErrorDetected\x18\xce\x1c \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12\x46\n\x0cundefinedCmd\x18\xac\x07 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12J\n\x10spiCrcBistFailed\x18\xa5\x10 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\"d\n\x1dUsDrvSpiCommErrors_array_port\x12\x43\n\x04\x64\x61ta\x18\xa7\x05 \x03(\x0b\x32\x34.pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors'
  ,
  dependencies=[us__drv_dot_us__drv__error__status__pb2.DESCRIPTOR,])




_USDRVSPICOMMERRORS = _descriptor.Descriptor(
  name='UsDrvSpiCommErrors',
  full_name='pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='prevCmdIncomplete', full_name='pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors.prevCmdIncomplete', index=0,
      number=2301, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='crcErrorDetected', full_name='pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors.crcErrorDetected', index=1,
      number=3662, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='undefinedCmd', full_name='pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors.undefinedCmd', index=2,
      number=940, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='spiCrcBistFailed', full_name='pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors.spiCrcBistFailed', index=3,
      number=2085, type=14, cpp_type=8, label=1,
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
  serialized_start=108,
  serialized_end=429,
)


_USDRVSPICOMMERRORS_ARRAY_PORT = _descriptor.Descriptor(
  name='UsDrvSpiCommErrors_array_port',
  full_name='pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors_array_port.data', index=0,
      number=679, type=11, cpp_type=10, label=3,
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
  serialized_start=431,
  serialized_end=531,
)

_USDRVSPICOMMERRORS.fields_by_name['prevCmdIncomplete'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVSPICOMMERRORS.fields_by_name['crcErrorDetected'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVSPICOMMERRORS.fields_by_name['undefinedCmd'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVSPICOMMERRORS.fields_by_name['spiCrcBistFailed'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVSPICOMMERRORS_ARRAY_PORT.fields_by_name['data'].message_type = _USDRVSPICOMMERRORS
DESCRIPTOR.message_types_by_name['UsDrvSpiCommErrors'] = _USDRVSPICOMMERRORS
DESCRIPTOR.message_types_by_name['UsDrvSpiCommErrors_array_port'] = _USDRVSPICOMMERRORS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsDrvSpiCommErrors = _reflection.GeneratedProtocolMessageType('UsDrvSpiCommErrors', (_message.Message,), {
  'DESCRIPTOR' : _USDRVSPICOMMERRORS,
  '__module__' : 'us_drv.us_drv_spi_comm_errors_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors)
  })
_sym_db.RegisterMessage(UsDrvSpiCommErrors)

UsDrvSpiCommErrors_array_port = _reflection.GeneratedProtocolMessageType('UsDrvSpiCommErrors_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USDRVSPICOMMERRORS_ARRAY_PORT,
  '__module__' : 'us_drv.us_drv_spi_comm_errors_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors_array_port)
  })
_sym_db.RegisterMessage(UsDrvSpiCommErrors_array_port)


# @@protoc_insertion_point(module_scope)
