# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/low_plot_data_interface_version.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/low_plot_data_interface_version.proto',
  package='pb.si.low_plot_data_interface_version',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n(si/low_plot_data_interface_version.proto\x12%pb.si.low_plot_data_interface_version\"<\n\x1cLowPlotData_InterfaceVersion\x12\x1c\n\x13LowPlotData_VERSION\x18\xd9\x11 \x01(\r\"}\n\'LowPlotData_InterfaceVersion_array_port\x12R\n\x04\x64\x61ta\x18\xbd\x08 \x03(\x0b\x32\x43.pb.si.low_plot_data_interface_version.LowPlotData_InterfaceVersion'
)




_LOWPLOTDATA_INTERFACEVERSION = _descriptor.Descriptor(
  name='LowPlotData_InterfaceVersion',
  full_name='pb.si.low_plot_data_interface_version.LowPlotData_InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='LowPlotData_VERSION', full_name='pb.si.low_plot_data_interface_version.LowPlotData_InterfaceVersion.LowPlotData_VERSION', index=0,
      number=2265, type=13, cpp_type=3, label=1,
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
  serialized_start=83,
  serialized_end=143,
)


_LOWPLOTDATA_INTERFACEVERSION_ARRAY_PORT = _descriptor.Descriptor(
  name='LowPlotData_InterfaceVersion_array_port',
  full_name='pb.si.low_plot_data_interface_version.LowPlotData_InterfaceVersion_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.low_plot_data_interface_version.LowPlotData_InterfaceVersion_array_port.data', index=0,
      number=1085, type=11, cpp_type=10, label=3,
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
  serialized_start=145,
  serialized_end=270,
)

_LOWPLOTDATA_INTERFACEVERSION_ARRAY_PORT.fields_by_name['data'].message_type = _LOWPLOTDATA_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['LowPlotData_InterfaceVersion'] = _LOWPLOTDATA_INTERFACEVERSION
DESCRIPTOR.message_types_by_name['LowPlotData_InterfaceVersion_array_port'] = _LOWPLOTDATA_INTERFACEVERSION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LowPlotData_InterfaceVersion = _reflection.GeneratedProtocolMessageType('LowPlotData_InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _LOWPLOTDATA_INTERFACEVERSION,
  '__module__' : 'si.low_plot_data_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.low_plot_data_interface_version.LowPlotData_InterfaceVersion)
  })
_sym_db.RegisterMessage(LowPlotData_InterfaceVersion)

LowPlotData_InterfaceVersion_array_port = _reflection.GeneratedProtocolMessageType('LowPlotData_InterfaceVersion_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LOWPLOTDATA_INTERFACEVERSION_ARRAY_PORT,
  '__module__' : 'si.low_plot_data_interface_version_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.low_plot_data_interface_version.LowPlotData_InterfaceVersion_array_port)
  })
_sym_db.RegisterMessage(LowPlotData_InterfaceVersion_array_port)


# @@protoc_insertion_point(module_scope)