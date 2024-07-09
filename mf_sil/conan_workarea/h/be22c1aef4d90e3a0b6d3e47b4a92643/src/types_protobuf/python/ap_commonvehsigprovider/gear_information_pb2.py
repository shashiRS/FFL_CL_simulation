# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_commonvehsigprovider/gear_information.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ap_commonvehsigprovider import gear_box_ctrl_system_state_pb2 as ap__commonvehsigprovider_dot_gear__box__ctrl__system__state__pb2
from ap_commonvehsigprovider import gear_pb2 as ap__commonvehsigprovider_dot_gear__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_commonvehsigprovider/gear_information.proto',
  package='pb.ap_commonvehsigprovider.gear_information',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n.ap_commonvehsigprovider/gear_information.proto\x12+pb.ap_commonvehsigprovider.gear_information\x1a\x38\x61p_commonvehsigprovider/gear_box_ctrl_system_state.proto\x1a\"ap_commonvehsigprovider/gear.proto\"\xc0\x01\n\x0fGearInformation\x12q\n\x19gearboxCtrlSystemState_nu\x18\xba\x1d \x01(\x0e\x32M.pb.ap_commonvehsigprovider.gear_box_ctrl_system_state.GearBoxCtrlSystemState\x12:\n\ngearCur_nu\x18\xc2\x02 \x01(\x0e\x32%.pb.ap_commonvehsigprovider.gear.Gear\"i\n\x1aGearInformation_array_port\x12K\n\x04\x64\x61ta\x18\xe6\x11 \x03(\x0b\x32<.pb.ap_commonvehsigprovider.gear_information.GearInformation'
  ,
  dependencies=[ap__commonvehsigprovider_dot_gear__box__ctrl__system__state__pb2.DESCRIPTOR,ap__commonvehsigprovider_dot_gear__pb2.DESCRIPTOR,])




_GEARINFORMATION = _descriptor.Descriptor(
  name='GearInformation',
  full_name='pb.ap_commonvehsigprovider.gear_information.GearInformation',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='gearboxCtrlSystemState_nu', full_name='pb.ap_commonvehsigprovider.gear_information.GearInformation.gearboxCtrlSystemState_nu', index=0,
      number=3770, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gearCur_nu', full_name='pb.ap_commonvehsigprovider.gear_information.GearInformation.gearCur_nu', index=1,
      number=322, type=14, cpp_type=8, label=1,
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
  serialized_start=190,
  serialized_end=382,
)


_GEARINFORMATION_ARRAY_PORT = _descriptor.Descriptor(
  name='GearInformation_array_port',
  full_name='pb.ap_commonvehsigprovider.gear_information.GearInformation_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_commonvehsigprovider.gear_information.GearInformation_array_port.data', index=0,
      number=2278, type=11, cpp_type=10, label=3,
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
  serialized_start=384,
  serialized_end=489,
)

_GEARINFORMATION.fields_by_name['gearboxCtrlSystemState_nu'].enum_type = ap__commonvehsigprovider_dot_gear__box__ctrl__system__state__pb2._GEARBOXCTRLSYSTEMSTATE
_GEARINFORMATION.fields_by_name['gearCur_nu'].enum_type = ap__commonvehsigprovider_dot_gear__pb2._GEAR
_GEARINFORMATION_ARRAY_PORT.fields_by_name['data'].message_type = _GEARINFORMATION
DESCRIPTOR.message_types_by_name['GearInformation'] = _GEARINFORMATION
DESCRIPTOR.message_types_by_name['GearInformation_array_port'] = _GEARINFORMATION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

GearInformation = _reflection.GeneratedProtocolMessageType('GearInformation', (_message.Message,), {
  'DESCRIPTOR' : _GEARINFORMATION,
  '__module__' : 'ap_commonvehsigprovider.gear_information_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.gear_information.GearInformation)
  })
_sym_db.RegisterMessage(GearInformation)

GearInformation_array_port = _reflection.GeneratedProtocolMessageType('GearInformation_array_port', (_message.Message,), {
  'DESCRIPTOR' : _GEARINFORMATION_ARRAY_PORT,
  '__module__' : 'ap_commonvehsigprovider.gear_information_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.gear_information.GearInformation_array_port)
  })
_sym_db.RegisterMessage(GearInformation_array_port)


# @@protoc_insertion_point(module_scope)