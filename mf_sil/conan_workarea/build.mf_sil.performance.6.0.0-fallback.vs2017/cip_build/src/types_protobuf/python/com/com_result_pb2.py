# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: com/com_result.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='com/com_result.proto',
  package='pb.com.com_result',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x14\x63om/com_result.proto\x12\x11pb.com.com_result*b\n\tComResult\x12\r\n\tCOMRES_OK\x10\x00\x12\x12\n\x0e\x43OMRES_WARNING\x10\x01\x12\x1c\n\x18\x43OMRES_WARN_WRONG_CONFIG\x10\x01\x12\x10\n\x0c\x43OMRES_ERROR\x10\x02\x1a\x02\x10\x01'
)

_COMRESULT = _descriptor.EnumDescriptor(
  name='ComResult',
  full_name='pb.com.com_result.ComResult',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='COMRES_OK', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COMRES_WARNING', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COMRES_WARN_WRONG_CONFIG', index=2, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COMRES_ERROR', index=3, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=b'\020\001',
  serialized_start=43,
  serialized_end=141,
)
_sym_db.RegisterEnumDescriptor(_COMRESULT)

ComResult = enum_type_wrapper.EnumTypeWrapper(_COMRESULT)
COMRES_OK = 0
COMRES_WARNING = 1
COMRES_WARN_WRONG_CONFIG = 1
COMRES_ERROR = 2


DESCRIPTOR.enum_types_by_name['ComResult'] = _COMRESULT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


_COMRESULT._options = None
# @@protoc_insertion_point(module_scope)
