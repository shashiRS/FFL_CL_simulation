# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_crm_mode_errors.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from us_drv import us_drv_error_status_pb2 as us__drv_dot_us__drv__error__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_crm_mode_errors.proto',
  package='pb.us_drv.us_drv_crm_mode_errors',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n#us_drv/us_drv_crm_mode_errors.proto\x12 pb.us_drv.us_drv_crm_mode_errors\x1a us_drv/us_drv_error_status.proto\"\xd2\x05\n\x12UsDrvCrmModeErrors\x12G\n\rcrmCmdFailure\x18\xbf\x04 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12\x1d\n\x14\x63rmCmdFailureDetails\x18\xb9\n \x01(\r\x12H\n\x0e\x64\x61taEarlyError\x18\xee\x18 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12G\n\rdataLateError\x18\xa0\x02 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12J\n\x10symbolCountError\x18\xf3\x13 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12\x42\n\x08\x63rcError\x18\xa3\x02 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12O\n\x15spiCmdSequenceFailure\x18\xea\x02 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12K\n\x11symbolCodingError\x18\xf4\x11 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12L\n\x12\x64siPinUndervoltage\x18\xfc\x05 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\x12\x45\n\x0b\x63lkRefError\x18\xd3\x17 \x01(\x0e\x32/.pb.us_drv.us_drv_error_status.UsDrvErrorStatus\"d\n\x1dUsDrvCrmModeErrors_array_port\x12\x43\n\x04\x64\x61ta\x18\xd5\x0b \x03(\x0b\x32\x34.pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors'
  ,
  dependencies=[us__drv_dot_us__drv__error__status__pb2.DESCRIPTOR,])




_USDRVCRMMODEERRORS = _descriptor.Descriptor(
  name='UsDrvCrmModeErrors',
  full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='crmCmdFailure', full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors.crmCmdFailure', index=0,
      number=575, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='crmCmdFailureDetails', full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors.crmCmdFailureDetails', index=1,
      number=1337, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='dataEarlyError', full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors.dataEarlyError', index=2,
      number=3182, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='dataLateError', full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors.dataLateError', index=3,
      number=288, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='symbolCountError', full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors.symbolCountError', index=4,
      number=2547, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='crcError', full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors.crcError', index=5,
      number=291, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='spiCmdSequenceFailure', full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors.spiCmdSequenceFailure', index=6,
      number=362, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='symbolCodingError', full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors.symbolCodingError', index=7,
      number=2292, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='dsiPinUndervoltage', full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors.dsiPinUndervoltage', index=8,
      number=764, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='clkRefError', full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors.clkRefError', index=9,
      number=3027, type=14, cpp_type=8, label=1,
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
  serialized_end=830,
)


_USDRVCRMMODEERRORS_ARRAY_PORT = _descriptor.Descriptor(
  name='UsDrvCrmModeErrors_array_port',
  full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors_array_port.data', index=0,
      number=1493, type=11, cpp_type=10, label=3,
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
  serialized_start=832,
  serialized_end=932,
)

_USDRVCRMMODEERRORS.fields_by_name['crmCmdFailure'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVCRMMODEERRORS.fields_by_name['dataEarlyError'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVCRMMODEERRORS.fields_by_name['dataLateError'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVCRMMODEERRORS.fields_by_name['symbolCountError'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVCRMMODEERRORS.fields_by_name['crcError'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVCRMMODEERRORS.fields_by_name['spiCmdSequenceFailure'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVCRMMODEERRORS.fields_by_name['symbolCodingError'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVCRMMODEERRORS.fields_by_name['dsiPinUndervoltage'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVCRMMODEERRORS.fields_by_name['clkRefError'].enum_type = us__drv_dot_us__drv__error__status__pb2._USDRVERRORSTATUS
_USDRVCRMMODEERRORS_ARRAY_PORT.fields_by_name['data'].message_type = _USDRVCRMMODEERRORS
DESCRIPTOR.message_types_by_name['UsDrvCrmModeErrors'] = _USDRVCRMMODEERRORS
DESCRIPTOR.message_types_by_name['UsDrvCrmModeErrors_array_port'] = _USDRVCRMMODEERRORS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsDrvCrmModeErrors = _reflection.GeneratedProtocolMessageType('UsDrvCrmModeErrors', (_message.Message,), {
  'DESCRIPTOR' : _USDRVCRMMODEERRORS,
  '__module__' : 'us_drv.us_drv_crm_mode_errors_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors)
  })
_sym_db.RegisterMessage(UsDrvCrmModeErrors)

UsDrvCrmModeErrors_array_port = _reflection.GeneratedProtocolMessageType('UsDrvCrmModeErrors_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USDRVCRMMODEERRORS_ARRAY_PORT,
  '__module__' : 'us_drv.us_drv_crm_mode_errors_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_crm_mode_errors.UsDrvCrmModeErrors_array_port)
  })
_sym_db.RegisterMessage(UsDrvCrmModeErrors_array_port)


# @@protoc_insertion_point(module_scope)
