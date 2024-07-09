# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/memorized_slot.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from mf_mempark import localization_status_pb2 as mf__mempark_dot_localization__status__pb2
from mf_mempark import gpssilent_offering_enabled_pb2 as mf__mempark_dot_gpssilent__offering__enabled__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/memorized_slot.proto',
  package='pb.mf_mempark.memorized_slot',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1fmf_mempark/memorized_slot.proto\x12\x1cpb.mf_mempark.memorized_slot\x1a$mf_mempark/localization_status.proto\x1a+mf_mempark/gpssilent_offering_enabled.proto\"\xed\x01\n\rMemorizedSlot\x12\x0b\n\x02ID\x18\x9a\x1a \x01(\r\x12T\n\x14relocalizationStatus\x18\xfd\x15 \x01(\x0e\x32\x35.pb.mf_mempark.localization_status.LocalizationStatus\x12\x65\n\x18gpsSilentOfferingEnabled\x18\xdb\x0e \x01(\x0e\x32\x42.pb.mf_mempark.gpssilent_offering_enabled.GPSSilentOfferingEnabled\x12\x12\n\troadWidth\x18\x85\x1d \x01(\r\"V\n\x18MemorizedSlot_array_port\x12:\n\x04\x64\x61ta\x18\xb8\x1e \x03(\x0b\x32+.pb.mf_mempark.memorized_slot.MemorizedSlot'
  ,
  dependencies=[mf__mempark_dot_localization__status__pb2.DESCRIPTOR,mf__mempark_dot_gpssilent__offering__enabled__pb2.DESCRIPTOR,])




_MEMORIZEDSLOT = _descriptor.Descriptor(
  name='MemorizedSlot',
  full_name='pb.mf_mempark.memorized_slot.MemorizedSlot',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ID', full_name='pb.mf_mempark.memorized_slot.MemorizedSlot.ID', index=0,
      number=3354, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='relocalizationStatus', full_name='pb.mf_mempark.memorized_slot.MemorizedSlot.relocalizationStatus', index=1,
      number=2813, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsSilentOfferingEnabled', full_name='pb.mf_mempark.memorized_slot.MemorizedSlot.gpsSilentOfferingEnabled', index=2,
      number=1883, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='roadWidth', full_name='pb.mf_mempark.memorized_slot.MemorizedSlot.roadWidth', index=3,
      number=3717, type=13, cpp_type=3, label=1,
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
  serialized_start=149,
  serialized_end=386,
)


_MEMORIZEDSLOT_ARRAY_PORT = _descriptor.Descriptor(
  name='MemorizedSlot_array_port',
  full_name='pb.mf_mempark.memorized_slot.MemorizedSlot_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.memorized_slot.MemorizedSlot_array_port.data', index=0,
      number=3896, type=11, cpp_type=10, label=3,
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
  serialized_start=388,
  serialized_end=474,
)

_MEMORIZEDSLOT.fields_by_name['relocalizationStatus'].enum_type = mf__mempark_dot_localization__status__pb2._LOCALIZATIONSTATUS
_MEMORIZEDSLOT.fields_by_name['gpsSilentOfferingEnabled'].enum_type = mf__mempark_dot_gpssilent__offering__enabled__pb2._GPSSILENTOFFERINGENABLED
_MEMORIZEDSLOT_ARRAY_PORT.fields_by_name['data'].message_type = _MEMORIZEDSLOT
DESCRIPTOR.message_types_by_name['MemorizedSlot'] = _MEMORIZEDSLOT
DESCRIPTOR.message_types_by_name['MemorizedSlot_array_port'] = _MEMORIZEDSLOT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MemorizedSlot = _reflection.GeneratedProtocolMessageType('MemorizedSlot', (_message.Message,), {
  'DESCRIPTOR' : _MEMORIZEDSLOT,
  '__module__' : 'mf_mempark.memorized_slot_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.memorized_slot.MemorizedSlot)
  })
_sym_db.RegisterMessage(MemorizedSlot)

MemorizedSlot_array_port = _reflection.GeneratedProtocolMessageType('MemorizedSlot_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MEMORIZEDSLOT_ARRAY_PORT,
  '__module__' : 'mf_mempark.memorized_slot_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.memorized_slot.MemorizedSlot_array_port)
  })
_sym_db.RegisterMessage(MemorizedSlot_array_port)


# @@protoc_insertion_point(module_scope)
