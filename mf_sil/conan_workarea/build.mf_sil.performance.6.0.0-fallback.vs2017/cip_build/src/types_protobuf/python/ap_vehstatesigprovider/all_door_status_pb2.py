# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_vehstatesigprovider/all_door_status.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ap_vehstatesigprovider import door_status_pb2 as ap__vehstatesigprovider_dot_door__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_vehstatesigprovider/all_door_status.proto',
  package='pb.ap_vehstatesigprovider.all_door_status',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n,ap_vehstatesigprovider/all_door_status.proto\x12)pb.ap_vehstatesigprovider.all_door_status\x1a(ap_vehstatesigprovider/door_status.proto\"\xb3\x02\n\rAllDoorStatus\x12H\n\x0c\x66rontPsgr_nu\x18\xbe\x14 \x01(\x0e\x32\x31.pb.ap_vehstatesigprovider.door_status.DoorStatus\x12\x45\n\tdriver_nu\x18\xe7\x0f \x01(\x0e\x32\x31.pb.ap_vehstatesigprovider.door_status.DoorStatus\x12H\n\x0crearRight_nu\x18\x87\t \x01(\x0e\x32\x31.pb.ap_vehstatesigprovider.door_status.DoorStatus\x12G\n\x0brearLeft_nu\x18\x95\x0e \x01(\x0e\x32\x31.pb.ap_vehstatesigprovider.door_status.DoorStatus\"c\n\x18\x41llDoorStatus_array_port\x12G\n\x04\x64\x61ta\x18\xc6\x15 \x03(\x0b\x32\x38.pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus'
  ,
  dependencies=[ap__vehstatesigprovider_dot_door__status__pb2.DESCRIPTOR,])




_ALLDOORSTATUS = _descriptor.Descriptor(
  name='AllDoorStatus',
  full_name='pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='frontPsgr_nu', full_name='pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus.frontPsgr_nu', index=0,
      number=2622, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='driver_nu', full_name='pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus.driver_nu', index=1,
      number=2023, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearRight_nu', full_name='pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus.rearRight_nu', index=2,
      number=1159, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearLeft_nu', full_name='pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus.rearLeft_nu', index=3,
      number=1813, type=14, cpp_type=8, label=1,
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
  serialized_start=134,
  serialized_end=441,
)


_ALLDOORSTATUS_ARRAY_PORT = _descriptor.Descriptor(
  name='AllDoorStatus_array_port',
  full_name='pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port.data', index=0,
      number=2758, type=11, cpp_type=10, label=3,
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
  serialized_start=443,
  serialized_end=542,
)

_ALLDOORSTATUS.fields_by_name['frontPsgr_nu'].enum_type = ap__vehstatesigprovider_dot_door__status__pb2._DOORSTATUS
_ALLDOORSTATUS.fields_by_name['driver_nu'].enum_type = ap__vehstatesigprovider_dot_door__status__pb2._DOORSTATUS
_ALLDOORSTATUS.fields_by_name['rearRight_nu'].enum_type = ap__vehstatesigprovider_dot_door__status__pb2._DOORSTATUS
_ALLDOORSTATUS.fields_by_name['rearLeft_nu'].enum_type = ap__vehstatesigprovider_dot_door__status__pb2._DOORSTATUS
_ALLDOORSTATUS_ARRAY_PORT.fields_by_name['data'].message_type = _ALLDOORSTATUS
DESCRIPTOR.message_types_by_name['AllDoorStatus'] = _ALLDOORSTATUS
DESCRIPTOR.message_types_by_name['AllDoorStatus_array_port'] = _ALLDOORSTATUS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

AllDoorStatus = _reflection.GeneratedProtocolMessageType('AllDoorStatus', (_message.Message,), {
  'DESCRIPTOR' : _ALLDOORSTATUS,
  '__module__' : 'ap_vehstatesigprovider.all_door_status_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
  })
_sym_db.RegisterMessage(AllDoorStatus)

AllDoorStatus_array_port = _reflection.GeneratedProtocolMessageType('AllDoorStatus_array_port', (_message.Message,), {
  'DESCRIPTOR' : _ALLDOORSTATUS_ARRAY_PORT,
  '__module__' : 'ap_vehstatesigprovider.all_door_status_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
  })
_sym_db.RegisterMessage(AllDoorStatus_array_port)


# @@protoc_insertion_point(module_scope)