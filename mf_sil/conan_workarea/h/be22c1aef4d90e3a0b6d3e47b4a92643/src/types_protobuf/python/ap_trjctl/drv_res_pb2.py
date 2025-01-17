# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_trjctl/drv_res.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ap_trjctl import drv_res_type_pb2 as ap__trjctl_dot_drv__res__type__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_trjctl/drv_res.proto',
  package='pb.ap_trjctl.drv_res',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x17\x61p_trjctl/drv_res.proto\x12\x14pb.ap_trjctl.drv_res\x1a\x1c\x61p_trjctl/drv_res_type.proto\"V\n\x06\x44rvRes\x12\x13\n\ndistance_m\x18\x96\n \x01(\x02\x12\x37\n\x07type_nu\x18\x9f\x16 \x01(\x0e\x32%.pb.ap_trjctl.drv_res_type.DrvResType\"@\n\x11\x44rvRes_array_port\x12+\n\x04\x64\x61ta\x18\xe2\x0e \x03(\x0b\x32\x1c.pb.ap_trjctl.drv_res.DrvRes'
  ,
  dependencies=[ap__trjctl_dot_drv__res__type__pb2.DESCRIPTOR,])




_DRVRES = _descriptor.Descriptor(
  name='DrvRes',
  full_name='pb.ap_trjctl.drv_res.DrvRes',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='distance_m', full_name='pb.ap_trjctl.drv_res.DrvRes.distance_m', index=0,
      number=1302, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='type_nu', full_name='pb.ap_trjctl.drv_res.DrvRes.type_nu', index=1,
      number=2847, type=14, cpp_type=8, label=1,
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
  serialized_start=79,
  serialized_end=165,
)


_DRVRES_ARRAY_PORT = _descriptor.Descriptor(
  name='DrvRes_array_port',
  full_name='pb.ap_trjctl.drv_res.DrvRes_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_trjctl.drv_res.DrvRes_array_port.data', index=0,
      number=1890, type=11, cpp_type=10, label=3,
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
  serialized_start=167,
  serialized_end=231,
)

_DRVRES.fields_by_name['type_nu'].enum_type = ap__trjctl_dot_drv__res__type__pb2._DRVRESTYPE
_DRVRES_ARRAY_PORT.fields_by_name['data'].message_type = _DRVRES
DESCRIPTOR.message_types_by_name['DrvRes'] = _DRVRES
DESCRIPTOR.message_types_by_name['DrvRes_array_port'] = _DRVRES_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

DrvRes = _reflection.GeneratedProtocolMessageType('DrvRes', (_message.Message,), {
  'DESCRIPTOR' : _DRVRES,
  '__module__' : 'ap_trjctl.drv_res_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_trjctl.drv_res.DrvRes)
  })
_sym_db.RegisterMessage(DrvRes)

DrvRes_array_port = _reflection.GeneratedProtocolMessageType('DrvRes_array_port', (_message.Message,), {
  'DESCRIPTOR' : _DRVRES_ARRAY_PORT,
  '__module__' : 'ap_trjctl.drv_res_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_trjctl.drv_res.DrvRes_array_port)
  })
_sym_db.RegisterMessage(DrvRes_array_port)


# @@protoc_insertion_point(module_scope)
