# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/traj_plan_debug_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/traj_plan_debug_port.proto',
  package='pb.ap_tp.traj_plan_debug_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n ap_tp/traj_plan_debug_port.proto\x12\x1dpb.ap_tp.traj_plan_debug_port\x1a\x17\x65\x63o/signal_header.proto\"\xfb\x01\n\x11TrajPlanDebugPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x11\n\x08\x64\x65\x62ugInt\x18\xce\t \x03(\x11\x12\x13\n\ndebugFloat\x18\xf9\x0f \x03(\x02\x12\x1a\n\x11mNumOfReplanCalls\x18\xb5\r \x01(\r\x12\x17\n\x0emTrajPlanState\x18\xc5\x18 \x01(\r\x12\x1d\n\x14mReplanSuccessful_nu\x18\xbb\x05 \x01(\x08\x12\x17\n\x0emStateEntry_nu\x18\xf3\x13 \x01(\x08\"_\n\x1cTrajPlanDebugPort_array_port\x12?\n\x04\x64\x61ta\x18\x89\x15 \x03(\x0b\x32\x30.pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_TRAJPLANDEBUGPORT = _descriptor.Descriptor(
  name='TrajPlanDebugPort',
  full_name='pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='debugInt', full_name='pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.debugInt', index=2,
      number=1230, type=17, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='debugFloat', full_name='pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.debugFloat', index=3,
      number=2041, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='mNumOfReplanCalls', full_name='pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.mNumOfReplanCalls', index=4,
      number=1717, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='mTrajPlanState', full_name='pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.mTrajPlanState', index=5,
      number=3141, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='mReplanSuccessful_nu', full_name='pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.mReplanSuccessful_nu', index=6,
      number=699, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='mStateEntry_nu', full_name='pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.mStateEntry_nu', index=7,
      number=2547, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_start=93,
  serialized_end=344,
)


_TRAJPLANDEBUGPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='TrajPlanDebugPort_array_port',
  full_name='pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort_array_port.data', index=0,
      number=2697, type=11, cpp_type=10, label=3,
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
  serialized_start=346,
  serialized_end=441,
)

_TRAJPLANDEBUGPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_TRAJPLANDEBUGPORT_ARRAY_PORT.fields_by_name['data'].message_type = _TRAJPLANDEBUGPORT
DESCRIPTOR.message_types_by_name['TrajPlanDebugPort'] = _TRAJPLANDEBUGPORT
DESCRIPTOR.message_types_by_name['TrajPlanDebugPort_array_port'] = _TRAJPLANDEBUGPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TrajPlanDebugPort = _reflection.GeneratedProtocolMessageType('TrajPlanDebugPort', (_message.Message,), {
  'DESCRIPTOR' : _TRAJPLANDEBUGPORT,
  '__module__' : 'ap_tp.traj_plan_debug_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort)
  })
_sym_db.RegisterMessage(TrajPlanDebugPort)

TrajPlanDebugPort_array_port = _reflection.GeneratedProtocolMessageType('TrajPlanDebugPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TRAJPLANDEBUGPORT_ARRAY_PORT,
  '__module__' : 'ap_tp.traj_plan_debug_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort_array_port)
  })
_sym_db.RegisterMessage(TrajPlanDebugPort_array_port)


# @@protoc_insertion_point(module_scope)