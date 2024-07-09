# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/taposddebug_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_tp import parking_box_debug_info_pb2 as ap__tp_dot_parking__box__debug__info__pb2
from ap_tp import pose_box_dataset_pb2 as ap__tp_dot_pose__box__dataset__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/taposddebug_port.proto',
  package='pb.ap_tp.taposddebug_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1c\x61p_tp/taposddebug_port.proto\x12\x19pb.ap_tp.taposddebug_port\x1a\x17\x65\x63o/signal_header.proto\x1a\"ap_tp/parking_box_debug_info.proto\x1a\x1c\x61p_tp/pose_box_dataset.proto\"\xc8\x03\n\x0fTAPOSDDebugPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12O\n\x10pbDebugBackwards\x18\xca\x08 \x03(\x0b\x32\x34.pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo\x12N\n\x0fpbDebugForwards\x18\xbb\x13 \x03(\x0b\x32\x34.pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo\x12\x42\n\x0eposeBoxDataset\x18\x83\x1b \x01(\x0b\x32).pb.ap_tp.pose_box_dataset.PoseBoxDataset\x12\x1a\n\x11latDistToTarget_m\x18\x87\x16 \x01(\x02\x12\x1b\n\x12longDistToTarget_m\x18\xcb\x0e \x01(\x02\x12\x1c\n\x13yawDiffToTarget_rad\x18\xbf\x1e \x01(\x02\x12\x11\n\x08\x64\x65\x62ugInt\x18\xce\t \x03(\x11\x12\x13\n\ndebugFloat\x18\xf9\x0f \x03(\x02\"W\n\x1aTAPOSDDebugPort_array_port\x12\x39\n\x04\x64\x61ta\x18\xec\x1f \x03(\x0b\x32*.pb.ap_tp.taposddebug_port.TAPOSDDebugPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__tp_dot_parking__box__debug__info__pb2.DESCRIPTOR,ap__tp_dot_pose__box__dataset__pb2.DESCRIPTOR,])




_TAPOSDDEBUGPORT = _descriptor.Descriptor(
  name='TAPOSDDebugPort',
  full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pbDebugBackwards', full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort.pbDebugBackwards', index=2,
      number=1098, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pbDebugForwards', full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort.pbDebugForwards', index=3,
      number=2491, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='poseBoxDataset', full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort.poseBoxDataset', index=4,
      number=3459, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='latDistToTarget_m', full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort.latDistToTarget_m', index=5,
      number=2823, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='longDistToTarget_m', full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort.longDistToTarget_m', index=6,
      number=1867, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='yawDiffToTarget_rad', full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort.yawDiffToTarget_rad', index=7,
      number=3903, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='debugInt', full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort.debugInt', index=8,
      number=1230, type=17, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='debugFloat', full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort.debugFloat', index=9,
      number=2041, type=2, cpp_type=6, label=3,
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
  serialized_start=151,
  serialized_end=607,
)


_TAPOSDDEBUGPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='TAPOSDDebugPort_array_port',
  full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_tp.taposddebug_port.TAPOSDDebugPort_array_port.data', index=0,
      number=4076, type=11, cpp_type=10, label=3,
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
  serialized_start=609,
  serialized_end=696,
)

_TAPOSDDEBUGPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_TAPOSDDEBUGPORT.fields_by_name['pbDebugBackwards'].message_type = ap__tp_dot_parking__box__debug__info__pb2._PARKINGBOXDEBUGINFO
_TAPOSDDEBUGPORT.fields_by_name['pbDebugForwards'].message_type = ap__tp_dot_parking__box__debug__info__pb2._PARKINGBOXDEBUGINFO
_TAPOSDDEBUGPORT.fields_by_name['poseBoxDataset'].message_type = ap__tp_dot_pose__box__dataset__pb2._POSEBOXDATASET
_TAPOSDDEBUGPORT_ARRAY_PORT.fields_by_name['data'].message_type = _TAPOSDDEBUGPORT
DESCRIPTOR.message_types_by_name['TAPOSDDebugPort'] = _TAPOSDDEBUGPORT
DESCRIPTOR.message_types_by_name['TAPOSDDebugPort_array_port'] = _TAPOSDDEBUGPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TAPOSDDebugPort = _reflection.GeneratedProtocolMessageType('TAPOSDDebugPort', (_message.Message,), {
  'DESCRIPTOR' : _TAPOSDDEBUGPORT,
  '__module__' : 'ap_tp.taposddebug_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.taposddebug_port.TAPOSDDebugPort)
  })
_sym_db.RegisterMessage(TAPOSDDebugPort)

TAPOSDDebugPort_array_port = _reflection.GeneratedProtocolMessageType('TAPOSDDebugPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _TAPOSDDEBUGPORT_ARRAY_PORT,
  '__module__' : 'ap_tp.taposddebug_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.taposddebug_port.TAPOSDDebugPort_array_port)
  })
_sym_db.RegisterMessage(TAPOSDDebugPort_array_port)


# @@protoc_insertion_point(module_scope)
