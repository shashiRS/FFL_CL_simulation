# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/pdcsectors.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from mf_hmih import pdcsector_info_pb2 as mf__hmih_dot_pdcsector__info__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/pdcsectors.proto',
  package='pb.mf_hmih.pdcsectors',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x18mf_hmih/pdcsectors.proto\x12\x15pb.mf_hmih.pdcsectors\x1a\x1cmf_hmih/pdcsector_info.proto\"\x91\x03\n\nPDCSectors\x12\x36\n\x04left\x18r \x03(\x0b\x32(.pb.mf_hmih.pdcsector_info.PDCSectorInfo\x12\x38\n\x05right\x18\x88\x06 \x03(\x0b\x32(.pb.mf_hmih.pdcsector_info.PDCSectorInfo\x12\x38\n\x05\x66ront\x18\xeb\x11 \x03(\x0b\x32(.pb.mf_hmih.pdcsector_info.PDCSectorInfo\x12\x37\n\x04rear\x18\xde\x01 \x03(\x0b\x32(.pb.mf_hmih.pdcsector_info.PDCSectorInfo\x12&\n\x1dPDC_P_SECTOR_INNER_COORDS_X_M\x18\xb2\x01 \x03(\x02\x12&\n\x1dPDC_P_SECTOR_INNER_COORDS_Y_M\x18\x82\x0f \x03(\x02\x12&\n\x1dPDC_P_SECTOR_OUTER_COORDS_X_M\x18\xbd\x03 \x03(\x02\x12&\n\x1dPDC_P_SECTOR_OUTER_COORDS_Y_M\x18\x8d\r \x03(\x02\"I\n\x15PDCSectors_array_port\x12\x30\n\x04\x64\x61ta\x18\xa1\x18 \x03(\x0b\x32!.pb.mf_hmih.pdcsectors.PDCSectors'
  ,
  dependencies=[mf__hmih_dot_pdcsector__info__pb2.DESCRIPTOR,])




_PDCSECTORS = _descriptor.Descriptor(
  name='PDCSectors',
  full_name='pb.mf_hmih.pdcsectors.PDCSectors',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='left', full_name='pb.mf_hmih.pdcsectors.PDCSectors.left', index=0,
      number=114, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='right', full_name='pb.mf_hmih.pdcsectors.PDCSectors.right', index=1,
      number=776, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='front', full_name='pb.mf_hmih.pdcsectors.PDCSectors.front', index=2,
      number=2283, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rear', full_name='pb.mf_hmih.pdcsectors.PDCSectors.rear', index=3,
      number=222, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDC_P_SECTOR_INNER_COORDS_X_M', full_name='pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_INNER_COORDS_X_M', index=4,
      number=178, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDC_P_SECTOR_INNER_COORDS_Y_M', full_name='pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_INNER_COORDS_Y_M', index=5,
      number=1922, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDC_P_SECTOR_OUTER_COORDS_X_M', full_name='pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_OUTER_COORDS_X_M', index=6,
      number=445, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDC_P_SECTOR_OUTER_COORDS_Y_M', full_name='pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_OUTER_COORDS_Y_M', index=7,
      number=1677, type=2, cpp_type=6, label=3,
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
  serialized_start=82,
  serialized_end=483,
)


_PDCSECTORS_ARRAY_PORT = _descriptor.Descriptor(
  name='PDCSectors_array_port',
  full_name='pb.mf_hmih.pdcsectors.PDCSectors_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.pdcsectors.PDCSectors_array_port.data', index=0,
      number=3105, type=11, cpp_type=10, label=3,
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
  serialized_start=485,
  serialized_end=558,
)

_PDCSECTORS.fields_by_name['left'].message_type = mf__hmih_dot_pdcsector__info__pb2._PDCSECTORINFO
_PDCSECTORS.fields_by_name['right'].message_type = mf__hmih_dot_pdcsector__info__pb2._PDCSECTORINFO
_PDCSECTORS.fields_by_name['front'].message_type = mf__hmih_dot_pdcsector__info__pb2._PDCSECTORINFO
_PDCSECTORS.fields_by_name['rear'].message_type = mf__hmih_dot_pdcsector__info__pb2._PDCSECTORINFO
_PDCSECTORS_ARRAY_PORT.fields_by_name['data'].message_type = _PDCSECTORS
DESCRIPTOR.message_types_by_name['PDCSectors'] = _PDCSECTORS
DESCRIPTOR.message_types_by_name['PDCSectors_array_port'] = _PDCSECTORS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PDCSectors = _reflection.GeneratedProtocolMessageType('PDCSectors', (_message.Message,), {
  'DESCRIPTOR' : _PDCSECTORS,
  '__module__' : 'mf_hmih.pdcsectors_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.pdcsectors.PDCSectors)
  })
_sym_db.RegisterMessage(PDCSectors)

PDCSectors_array_port = _reflection.GeneratedProtocolMessageType('PDCSectors_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PDCSECTORS_ARRAY_PORT,
  '__module__' : 'mf_hmih.pdcsectors_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.pdcsectors.PDCSectors_array_port)
  })
_sym_db.RegisterMessage(PDCSectors_array_port)


# @@protoc_insertion_point(module_scope)