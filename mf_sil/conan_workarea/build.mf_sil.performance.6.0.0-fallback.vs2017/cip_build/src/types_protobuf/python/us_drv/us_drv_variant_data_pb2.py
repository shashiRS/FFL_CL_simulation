# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_variant_data.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_variant_data.proto',
  package='pb.us_drv.us_drv_variant_data',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n us_drv/us_drv_variant_data.proto\x12\x1dpb.us_drv.us_drv_variant_data\"@\n\x10UsDrvVariantData\x12\x14\n\x0busedAsicNum\x18\xe9\x01 \x01(\r\x12\x16\n\rusedSensorNum\x18\xbe\x0e \x01(\r\"]\n\x1bUsDrvVariantData_array_port\x12>\n\x04\x64\x61ta\x18\xe6\x16 \x03(\x0b\x32/.pb.us_drv.us_drv_variant_data.UsDrvVariantData'
)




_USDRVVARIANTDATA = _descriptor.Descriptor(
  name='UsDrvVariantData',
  full_name='pb.us_drv.us_drv_variant_data.UsDrvVariantData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='usedAsicNum', full_name='pb.us_drv.us_drv_variant_data.UsDrvVariantData.usedAsicNum', index=0,
      number=233, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='usedSensorNum', full_name='pb.us_drv.us_drv_variant_data.UsDrvVariantData.usedSensorNum', index=1,
      number=1854, type=13, cpp_type=3, label=1,
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
  serialized_start=67,
  serialized_end=131,
)


_USDRVVARIANTDATA_ARRAY_PORT = _descriptor.Descriptor(
  name='UsDrvVariantData_array_port',
  full_name='pb.us_drv.us_drv_variant_data.UsDrvVariantData_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_drv.us_drv_variant_data.UsDrvVariantData_array_port.data', index=0,
      number=2918, type=11, cpp_type=10, label=3,
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
  serialized_start=133,
  serialized_end=226,
)

_USDRVVARIANTDATA_ARRAY_PORT.fields_by_name['data'].message_type = _USDRVVARIANTDATA
DESCRIPTOR.message_types_by_name['UsDrvVariantData'] = _USDRVVARIANTDATA
DESCRIPTOR.message_types_by_name['UsDrvVariantData_array_port'] = _USDRVVARIANTDATA_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsDrvVariantData = _reflection.GeneratedProtocolMessageType('UsDrvVariantData', (_message.Message,), {
  'DESCRIPTOR' : _USDRVVARIANTDATA,
  '__module__' : 'us_drv.us_drv_variant_data_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_variant_data.UsDrvVariantData)
  })
_sym_db.RegisterMessage(UsDrvVariantData)

UsDrvVariantData_array_port = _reflection.GeneratedProtocolMessageType('UsDrvVariantData_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USDRVVARIANTDATA_ARRAY_PORT,
  '__module__' : 'us_drv.us_drv_variant_data_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_variant_data.UsDrvVariantData_array_port)
  })
_sym_db.RegisterMessage(UsDrvVariantData_array_port)


# @@protoc_insertion_point(module_scope)