# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/high_plot_data_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/high_plot_data_interface_version.proto',
  package='pb.si.high_plot_data_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n)si/high_plot_data_interface_version.proto\x12&pb.si.high_plot_data_interface_version\">\n\x1dHighPlotData_InterfaceVersion\x12\x1d\n\x14HighPlotData_VERSION\x18\xc4\x17 \x01(\r\"\x80\x01\n(HighPlotData_InterfaceVersion_array_port\x12T\n\x04\x64\x61ta\x18\xc1\x11 \x03(\x0b\x32\x45.pb.si.high_plot_data_interface_version.HighPlotData_InterfaceVersion'
)




_HIGHPLOTDATA_INTERFACEVERSION = _descriptor.Descriptor(
  name='HighPlotData_InterfaceVersion',
  full_name='pb.si.high_plot_data_interface_version.HighPlotData_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='HighPlotData_VERSION', full_name='pb.si.high_plot_data_interface_version.HighPlotData_InterfaceVersion.HighPlotData_VERSION', index=0,
      number=3012, type=13, cpp_type=3, label=1,
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
  serialized_start=85,
  serialized_end=147,
)


_HIGHPLOTDATA_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='HighPlotData_InterfaceVersion_array_port',
  full_name='pb.si.high_plot_data_interface_version.HighPlotData_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.high_plot_data_interface_version.HighPlotData_InterfaceVersion_array_port.data', index=0,
      number=2241, type=11, cpp_type=10, label=3,
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
  serialized_start=150,
  serialized_end=278,
)

_HIGHPLOTDATA_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _HIGHPLOTDATA_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['HighPlotData_InterfaceVersion'] = _HIGHPLOTDATA_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['HighPlotData_InterfaceVersion_array_port'] = _HIGHPLOTDATA_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

HighPlotData_InterfaceVersion = _reflection.GeneratedProtocolMessageType('HighPlotData_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _HIGHPLOTDATA_INTERFACEVERSION,
  '__module__' : 'si.high_plot_data_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.high_plot_data_interface_version.HighPlotData_InterfaceVersion)
  })
_sym_db.RegisterMessage(HighPlotData_InterfaceVersion)

HighPlotData_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('HighPlotData_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _HIGHPLOTDATA_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'si.high_plot_data_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.high_plot_data_interface_version.HighPlotData_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(HighPlotData_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)