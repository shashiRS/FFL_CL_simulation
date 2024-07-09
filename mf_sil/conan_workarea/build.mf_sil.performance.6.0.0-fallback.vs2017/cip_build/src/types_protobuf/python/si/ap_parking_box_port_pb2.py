# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/ap_parking_box_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from si import parking_box_serializable_pb2 as si_dot_parking__box__serializable__pb2
from si import external_pose_data_pb2 as si_dot_external__pose__data__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/ap_parking_box_port.proto',
  package='pb.si.ap_parking_box_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1csi/ap_parking_box_port.proto\x12\x19pb.si.ap_parking_box_port\x1a\x17\x65\x63o/signal_header.proto\x1a!si/parking_box_serializable.proto\x1a\x1bsi/external_pose_data.proto\"\xb8\x02\n\x10\x41pParkingBoxPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12 \n\x17numValidParkingBoxes_nu\x18\x95\x08 \x01(\r\x12\x1e\n\x15numValidExternalPoses\x18\xa6\x15 \x01(\r\x12M\n\x0cparkingBoxes\x18\xe1\x0b \x03(\x0b\x32\x36.pb.si.parking_box_serializable.ParkingBoxSerializable\x12@\n\x0b\x65xtPoseData\x18\x9e\x1a \x03(\x0b\x32*.pb.si.external_pose_data.ExternalPoseData\"Y\n\x1b\x41pParkingBoxPort_array_port\x12:\n\x04\x64\x61ta\x18\xde\x1c \x03(\x0b\x32+.pb.si.ap_parking_box_port.ApParkingBoxPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,si_dot_parking__box__serializable__pb2.DESCRIPTOR,si_dot_external__pose__data__pb2.DESCRIPTOR,])




_APPARKINGBOXPORT = _descriptor.Descriptor(
  name='ApParkingBoxPort',
  full_name='pb.si.ap_parking_box_port.ApParkingBoxPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.si.ap_parking_box_port.ApParkingBoxPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.si.ap_parking_box_port.ApParkingBoxPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numValidParkingBoxes_nu', full_name='pb.si.ap_parking_box_port.ApParkingBoxPort.numValidParkingBoxes_nu', index=2,
      number=1045, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numValidExternalPoses', full_name='pb.si.ap_parking_box_port.ApParkingBoxPort.numValidExternalPoses', index=3,
      number=2726, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='parkingBoxes', full_name='pb.si.ap_parking_box_port.ApParkingBoxPort.parkingBoxes', index=4,
      number=1505, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='extPoseData', full_name='pb.si.ap_parking_box_port.ApParkingBoxPort.extPoseData', index=5,
      number=3358, type=11, cpp_type=10, label=3,
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
  serialized_start=149,
  serialized_end=461,
)


_APPARKINGBOXPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='ApParkingBoxPort_array_port',
  full_name='pb.si.ap_parking_box_port.ApParkingBoxPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.ap_parking_box_port.ApParkingBoxPort_array_port.data', index=0,
      number=3678, type=11, cpp_type=10, label=3,
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
  serialized_start=463,
  serialized_end=552,
)

_APPARKINGBOXPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_APPARKINGBOXPORT.fields_by_name['parkingBoxes'].message_type = si_dot_parking__box__serializable__pb2._PARKINGBOXSERIALIZABLE
_APPARKINGBOXPORT.fields_by_name['extPoseData'].message_type = si_dot_external__pose__data__pb2._EXTERNALPOSEDATA
_APPARKINGBOXPORT_ARRAY_PORT.fields_by_name['data'].message_type = _APPARKINGBOXPORT
DESCRIPTOR.message_types_by_name['ApParkingBoxPort'] = _APPARKINGBOXPORT
DESCRIPTOR.message_types_by_name['ApParkingBoxPort_array_port'] = _APPARKINGBOXPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ApParkingBoxPort = _reflection.GeneratedProtocolMessageType('ApParkingBoxPort', (_message.Message,), {
  'DESCRIPTOR' : _APPARKINGBOXPORT,
  '__module__' : 'si.ap_parking_box_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.ap_parking_box_port.ApParkingBoxPort)
  })
_sym_db.RegisterMessage(ApParkingBoxPort)

ApParkingBoxPort_array_port = _reflection.GeneratedProtocolMessageType('ApParkingBoxPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _APPARKINGBOXPORT_ARRAY_PORT,
  '__module__' : 'si.ap_parking_box_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
  })
_sym_db.RegisterMessage(ApParkingBoxPort_array_port)


# @@protoc_insertion_point(module_scope)
