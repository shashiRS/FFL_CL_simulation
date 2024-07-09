# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_lvmd/lvmdlead_vehicle_status.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_lvmd/lvmdlead_vehicle_status.proto',
  package='pb.mf_lvmd.lvmdlead_vehicle_status',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n%mf_lvmd/lvmdlead_vehicle_status.proto\x12\"pb.mf_lvmd.lvmdlead_vehicle_status\"\xc4\x01\n\x15LVMDLeadVehicleStatus\x12#\n\x1aleadvehicle_standstilltime\x18\xd9\x11 \x01(\x02\x12%\n\x1cleadvehicle_forward_distance\x18\xa2\x13 \x01(\x02\x12\x1a\n\x11leadvehicle_valid\x18\xba\t \x01(\x08\x12#\n\x1bleadvehicle_driven_distance\x18\x1e \x01(\x02\x12\x1e\n\x15leadvehicle_object_id\x18\xa1\x07 \x01(\r\"l\n LVMDLeadVehicleStatus_array_port\x12H\n\x04\x64\x61ta\x18\xee\x0e \x03(\x0b\x32\x39.pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus'
)




_LVMDLEADVEHICLESTATUS = _descriptor.Descriptor(
  name='LVMDLeadVehicleStatus',
  full_name='pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='leadvehicle_standstilltime', full_name='pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus.leadvehicle_standstilltime', index=0,
      number=2265, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='leadvehicle_forward_distance', full_name='pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus.leadvehicle_forward_distance', index=1,
      number=2466, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='leadvehicle_valid', full_name='pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus.leadvehicle_valid', index=2,
      number=1210, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='leadvehicle_driven_distance', full_name='pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus.leadvehicle_driven_distance', index=3,
      number=30, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='leadvehicle_object_id', full_name='pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus.leadvehicle_object_id', index=4,
      number=929, type=13, cpp_type=3, label=1,
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
  serialized_start=78,
  serialized_end=274,
)


_LVMDLEADVEHICLESTATUS_ARRAY_PORT = _descriptor.Descriptor(
  name='LVMDLeadVehicleStatus_array_port',
  full_name='pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus_array_port.data', index=0,
      number=1902, type=11, cpp_type=10, label=3,
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
  serialized_start=276,
  serialized_end=384,
)

_LVMDLEADVEHICLESTATUS_ARRAY_PORT.fields_by_name['data'].message_type = _LVMDLEADVEHICLESTATUS
DESCRIPTOR.message_types_by_name['LVMDLeadVehicleStatus'] = _LVMDLEADVEHICLESTATUS
DESCRIPTOR.message_types_by_name['LVMDLeadVehicleStatus_array_port'] = _LVMDLEADVEHICLESTATUS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LVMDLeadVehicleStatus = _reflection.GeneratedProtocolMessageType('LVMDLeadVehicleStatus', (_message.Message,), {
  'DESCRIPTOR' : _LVMDLEADVEHICLESTATUS,
  '__module__' : 'mf_lvmd.lvmdlead_vehicle_status_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus)
  })
_sym_db.RegisterMessage(LVMDLeadVehicleStatus)

LVMDLeadVehicleStatus_array_port = _reflection.GeneratedProtocolMessageType('LVMDLeadVehicleStatus_array_port', (_message.Message,), {
  'DESCRIPTOR' : _LVMDLEADVEHICLESTATUS_ARRAY_PORT,
  '__module__' : 'mf_lvmd.lvmdlead_vehicle_status_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus_array_port)
  })
_sym_db.RegisterMessage(LVMDLeadVehicleStatus_array_port)


# @@protoc_insertion_point(module_scope)
