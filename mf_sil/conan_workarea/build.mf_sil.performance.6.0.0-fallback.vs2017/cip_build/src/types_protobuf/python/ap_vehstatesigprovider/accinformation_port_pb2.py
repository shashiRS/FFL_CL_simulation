# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_vehstatesigprovider/accinformation_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_vehstatesigprovider import accstatus_pb2 as ap__vehstatesigprovider_dot_accstatus__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_vehstatesigprovider/accinformation_port.proto',
  package='pb.ap_vehstatesigprovider.accinformation_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n0ap_vehstatesigprovider/accinformation_port.proto\x12-pb.ap_vehstatesigprovider.accinformation_port\x1a\x17\x65\x63o/signal_header.proto\x1a&ap_vehstatesigprovider/accstatus.proto\"\xae\x01\n\x12\x41\x43\x43InformationPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x45\n\x0c\x61\x63\x63Status_nu\x18\xe1\r \x01(\x0e\x32..pb.ap_vehstatesigprovider.accstatus.ACCStatus\"q\n\x1d\x41\x43\x43InformationPort_array_port\x12P\n\x04\x64\x61ta\x18\xb5\n \x03(\x0b\x32\x41.pb.ap_vehstatesigprovider.accinformation_port.ACCInformationPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__vehstatesigprovider_dot_accstatus__pb2.DESCRIPTOR,])




_ACCINFORMATIONPORT = _descriptor.Descriptor(
  name='ACCInformationPort',
  full_name='pb.ap_vehstatesigprovider.accinformation_port.ACCInformationPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_vehstatesigprovider.accinformation_port.ACCInformationPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_vehstatesigprovider.accinformation_port.ACCInformationPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accStatus_nu', full_name='pb.ap_vehstatesigprovider.accinformation_port.ACCInformationPort.accStatus_nu', index=2,
      number=1761, type=14, cpp_type=8, label=1,
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
  serialized_start=165,
  serialized_end=339,
)


_ACCINFORMATIONPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='ACCInformationPort_array_port',
  full_name='pb.ap_vehstatesigprovider.accinformation_port.ACCInformationPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_vehstatesigprovider.accinformation_port.ACCInformationPort_array_port.data', index=0,
      number=1333, type=11, cpp_type=10, label=3,
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
  serialized_start=341,
  serialized_end=454,
)

_ACCINFORMATIONPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_ACCINFORMATIONPORT.fields_by_name['accStatus_nu'].enum_type = ap__vehstatesigprovider_dot_accstatus__pb2._ACCSTATUS
_ACCINFORMATIONPORT_ARRAY_PORT.fields_by_name['data'].message_type = _ACCINFORMATIONPORT
DESCRIPTOR.message_types_by_name['ACCInformationPort'] = _ACCINFORMATIONPORT
DESCRIPTOR.message_types_by_name['ACCInformationPort_array_port'] = _ACCINFORMATIONPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ACCInformationPort = _reflection.GeneratedProtocolMessageType('ACCInformationPort', (_message.Message,), {
  'DESCRIPTOR' : _ACCINFORMATIONPORT,
  '__module__' : 'ap_vehstatesigprovider.accinformation_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.accinformation_port.ACCInformationPort)
  })
_sym_db.RegisterMessage(ACCInformationPort)

ACCInformationPort_array_port = _reflection.GeneratedProtocolMessageType('ACCInformationPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _ACCINFORMATIONPORT_ARRAY_PORT,
  '__module__' : 'ap_vehstatesigprovider.accinformation_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.accinformation_port.ACCInformationPort_array_port)
  })
_sym_db.RegisterMessage(ACCInformationPort_array_port)


# @@protoc_insertion_point(module_scope)
