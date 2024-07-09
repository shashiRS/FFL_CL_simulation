# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/parking_slot.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_commonvehsigprovider import gpsdata_pb2 as ap__commonvehsigprovider_dot_gpsdata__pb2
from lsm_geoml import pose_pod_pb2 as lsm__geoml_dot_pose__pod__pb2
from mf_mempark import system_defined_pose_side_pb2 as mf__mempark_dot_system__defined__pose__side__pb2
from mf_mempark import system_defined_pose_type_pb2 as mf__mempark_dot_system__defined__pose__type__pb2
from mf_mempark import slot_meta_data_pb2 as mf__mempark_dot_slot__meta__data__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/parking_slot.proto',
  package='pb.mf_mempark.parking_slot',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1dmf_mempark/parking_slot.proto\x12\x1apb.mf_mempark.parking_slot\x1a\x17\x65\x63o/signal_header.proto\x1a%ap_commonvehsigprovider/gpsdata.proto\x1a\x18lsm_geoml/pose_pod.proto\x1a)mf_mempark/system_defined_pose_side.proto\x1a)mf_mempark/system_defined_pose_type.proto\x1a\x1fmf_mempark/slot_meta_data.proto\"\x9a\x04\n\x0bParkingSlot\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x0f\n\x06slotID\x18\xd0\x01 \x01(\r\x12>\n\x08slotGNSS\x18\xa2\x0b \x01(\x0b\x32+.pb.ap_commonvehsigprovider.gpsdata.GPSData\x12\x32\n\x08slotPose\x18\xff\x05 \x01(\x0b\x32\x1f.pb.lsm_geoml.pose_pod.Pose_POD\x12\x33\n\tstartPose\x18\xce\x19 \x01(\x0b\x32\x1f.pb.lsm_geoml.pose_pod.Pose_POD\x12P\n\x08slotSide\x18\xfa\x0f \x01(\x0e\x32=.pb.mf_mempark.system_defined_pose_side.SystemDefinedPoseSide\x12P\n\x08slotType\x18\x88\x1c \x01(\x0e\x32=.pb.mf_mempark.system_defined_pose_type.SystemDefinedPoseType\x12\x1b\n\x12\x63orrespondingMapId\x18\x88\x1d \x01(\r\x12=\n\x08metaData\x18\xd4\x0e \x01(\x0b\x32*.pb.mf_mempark.slot_meta_data.SlotMetaData\"P\n\x16ParkingSlot_array_port\x12\x36\n\x04\x64\x61ta\x18\x9e\x1e \x03(\x0b\x32\'.pb.mf_mempark.parking_slot.ParkingSlot'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__commonvehsigprovider_dot_gpsdata__pb2.DESCRIPTOR,lsm__geoml_dot_pose__pod__pb2.DESCRIPTOR,mf__mempark_dot_system__defined__pose__side__pb2.DESCRIPTOR,mf__mempark_dot_system__defined__pose__type__pb2.DESCRIPTOR,mf__mempark_dot_slot__meta__data__pb2.DESCRIPTOR,])




_PARKINGSLOT = _descriptor.Descriptor(
  name='ParkingSlot',
  full_name='pb.mf_mempark.parking_slot.ParkingSlot',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_mempark.parking_slot.ParkingSlot.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_mempark.parking_slot.ParkingSlot.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='slotID', full_name='pb.mf_mempark.parking_slot.ParkingSlot.slotID', index=2,
      number=208, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='slotGNSS', full_name='pb.mf_mempark.parking_slot.ParkingSlot.slotGNSS', index=3,
      number=1442, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='slotPose', full_name='pb.mf_mempark.parking_slot.ParkingSlot.slotPose', index=4,
      number=767, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='startPose', full_name='pb.mf_mempark.parking_slot.ParkingSlot.startPose', index=5,
      number=3278, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='slotSide', full_name='pb.mf_mempark.parking_slot.ParkingSlot.slotSide', index=6,
      number=2042, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='slotType', full_name='pb.mf_mempark.parking_slot.ParkingSlot.slotType', index=7,
      number=3592, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='correspondingMapId', full_name='pb.mf_mempark.parking_slot.ParkingSlot.correspondingMapId', index=8,
      number=3720, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='metaData', full_name='pb.mf_mempark.parking_slot.ParkingSlot.metaData', index=9,
      number=1876, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=271,
  serialized_end=809,
)


_PARKINGSLOT_ARRAY_PORT = _descriptor.Descriptor(
  name='ParkingSlot_array_port',
  full_name='pb.mf_mempark.parking_slot.ParkingSlot_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.parking_slot.ParkingSlot_array_port.data', index=0,
      number=3870, type=11, cpp_type=10, label=3,
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
  serialized_start=811,
  serialized_end=891,
)

_PARKINGSLOT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_PARKINGSLOT.fields_by_name['slotGNSS'].message_type = ap__commonvehsigprovider_dot_gpsdata__pb2._GPSDATA
_PARKINGSLOT.fields_by_name['slotPose'].message_type = lsm__geoml_dot_pose__pod__pb2._POSE_POD
_PARKINGSLOT.fields_by_name['startPose'].message_type = lsm__geoml_dot_pose__pod__pb2._POSE_POD
_PARKINGSLOT.fields_by_name['slotSide'].enum_type = mf__mempark_dot_system__defined__pose__side__pb2._SYSTEMDEFINEDPOSESIDE
_PARKINGSLOT.fields_by_name['slotType'].enum_type = mf__mempark_dot_system__defined__pose__type__pb2._SYSTEMDEFINEDPOSETYPE
_PARKINGSLOT.fields_by_name['metaData'].message_type = mf__mempark_dot_slot__meta__data__pb2._SLOTMETADATA
_PARKINGSLOT_ARRAY_PORT.fields_by_name['data'].message_type = _PARKINGSLOT
DESCRIPTOR.message_types_by_name['ParkingSlot'] = _PARKINGSLOT
DESCRIPTOR.message_types_by_name['ParkingSlot_array_port'] = _PARKINGSLOT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ParkingSlot = _reflection.GeneratedProtocolMessageType('ParkingSlot', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGSLOT,
  '__module__' : 'mf_mempark.parking_slot_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.parking_slot.ParkingSlot)
  })
_sym_db.RegisterMessage(ParkingSlot)

ParkingSlot_array_port = _reflection.GeneratedProtocolMessageType('ParkingSlot_array_port', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGSLOT_ARRAY_PORT,
  '__module__' : 'mf_mempark.parking_slot_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.parking_slot.ParkingSlot_array_port)
  })
_sym_db.RegisterMessage(ParkingSlot_array_port)


# @@protoc_insertion_point(module_scope)