# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_trjctl/la_dmcctrl_request_interface_type.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_trjctl/la_dmcctrl_request_interface_type.proto',
  package='pb.ap_trjctl.la_dmcctrl_request_interface_type',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n1ap_trjctl/la_dmcctrl_request_interface_type.proto\x12.pb.ap_trjctl.la_dmcctrl_request_interface_type*\xd8\x02\n\x1dLaDMCCtrlRequestInterfaceType\x12$\n LADMC_REQ_TYPE_STEER_WHEEL_ANGLE\x10\x00\x12\x1e\n\x1aLADMC_REQ_TYPE_ANGLE_FRONT\x10\x01\x12\x1d\n\x19LADMC_REQ_TYPE_ANGLE_REAR\x10\x02\x12#\n\x1fLADMC_REQ_TYPE_ANGLE_FRONT_REAR\x10\x03\x12\x1f\n\x1bLADMC_REQ_TYPE_TORQUE_FRONT\x10\x04\x12\x1e\n\x1aLADMC_REQ_TYPE_TORQUE_REAR\x10\x05\x12$\n LADMC_REQ_TYPE_TORQUE_FRONT_REAR\x10\x06\x12\x1c\n\x18LADMC_REQ_TYPE_CURVATURE\x10\x07\x12(\n$MAX_NUM_LADMC_REQUEST_INTERFACE_TYPE\x10\x08'
)

_LADMCCTRLREQUESTINTERFACETYPE = _descriptor.EnumDescriptor(
  name='LaDMCCtrlRequestInterfaceType',
  full_name='pb.ap_trjctl.la_dmcctrl_request_interface_type.LaDMCCtrlRequestInterfaceType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='LADMC_REQ_TYPE_STEER_WHEEL_ANGLE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LADMC_REQ_TYPE_ANGLE_FRONT', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LADMC_REQ_TYPE_ANGLE_REAR', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LADMC_REQ_TYPE_ANGLE_FRONT_REAR', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LADMC_REQ_TYPE_TORQUE_FRONT', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LADMC_REQ_TYPE_TORQUE_REAR', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LADMC_REQ_TYPE_TORQUE_FRONT_REAR', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LADMC_REQ_TYPE_CURVATURE', index=7, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MAX_NUM_LADMC_REQUEST_INTERFACE_TYPE', index=8, number=8,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=102,
  serialized_end=446,
)
_sym_db.RegisterEnumDescriptor(_LADMCCTRLREQUESTINTERFACETYPE)

LaDMCCtrlRequestInterfaceType = enum_type_wrapper.EnumTypeWrapper(_LADMCCTRLREQUESTINTERFACETYPE)
LADMC_REQ_TYPE_STEER_WHEEL_ANGLE = 0
LADMC_REQ_TYPE_ANGLE_FRONT = 1
LADMC_REQ_TYPE_ANGLE_REAR = 2
LADMC_REQ_TYPE_ANGLE_FRONT_REAR = 3
LADMC_REQ_TYPE_TORQUE_FRONT = 4
LADMC_REQ_TYPE_TORQUE_REAR = 5
LADMC_REQ_TYPE_TORQUE_FRONT_REAR = 6
LADMC_REQ_TYPE_CURVATURE = 7
MAX_NUM_LADMC_REQUEST_INTERFACE_TYPE = 8


DESCRIPTOR.enum_types_by_name['LaDMCCtrlRequestInterfaceType'] = _LADMCCTRLREQUESTINTERFACETYPE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
