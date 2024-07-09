# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/slot_cost_fun_data_per_slot_serializable.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from si import slot_cost_fun_data_per_slot_pb2 as si_dot_slot__cost__fun__data__per__slot__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/slot_cost_fun_data_per_slot_serializable.proto',
  package='pb.si.slot_cost_fun_data_per_slot_serializable',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n1si/slot_cost_fun_data_per_slot_serializable.proto\x12.pb.si.slot_cost_fun_data_per_slot_serializable\x1a$si/slot_cost_fun_data_per_slot.proto\"\x84\x01\n\"SlotCostFunDataPerSlotSerializable\x12\x13\n\nactualSize\x18\xc1\x17 \x01(\r\x12I\n\x05\x61rray\x18\xa4\x15 \x03(\x0b\x32\x39.pb.si.slot_cost_fun_data_per_slot.SlotCostFunDataPerSlot\"\x92\x01\n-SlotCostFunDataPerSlotSerializable_array_port\x12\x61\n\x04\x64\x61ta\x18\x95\x01 \x03(\x0b\x32R.pb.si.slot_cost_fun_data_per_slot_serializable.SlotCostFunDataPerSlotSerializable'
  ,
  dependencies=[si_dot_slot__cost__fun__data__per__slot__pb2.DESCRIPTOR,])




_SLOTCOSTFUNDATAPERSLOTSERIALIZABLE = _descriptor.Descriptor(
  name='SlotCostFunDataPerSlotSerializable',
  full_name='pb.si.slot_cost_fun_data_per_slot_serializable.SlotCostFunDataPerSlotSerializable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='actualSize', full_name='pb.si.slot_cost_fun_data_per_slot_serializable.SlotCostFunDataPerSlotSerializable.actualSize', index=0,
      number=3009, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='array', full_name='pb.si.slot_cost_fun_data_per_slot_serializable.SlotCostFunDataPerSlotSerializable.array', index=1,
      number=2724, type=11, cpp_type=10, label=3,
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
  serialized_start=140,
  serialized_end=272,
)


_SLOTCOSTFUNDATAPERSLOTSERIALIZABLE_ARRAY_PORT = _descriptor.Descriptor(
  name='SlotCostFunDataPerSlotSerializable_array_port',
  full_name='pb.si.slot_cost_fun_data_per_slot_serializable.SlotCostFunDataPerSlotSerializable_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.slot_cost_fun_data_per_slot_serializable.SlotCostFunDataPerSlotSerializable_array_port.data', index=0,
      number=149, type=11, cpp_type=10, label=3,
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
  serialized_start=275,
  serialized_end=421,
)

_SLOTCOSTFUNDATAPERSLOTSERIALIZABLE.fields_by_name['array'].message_type = si_dot_slot__cost__fun__data__per__slot__pb2._SLOTCOSTFUNDATAPERSLOT
_SLOTCOSTFUNDATAPERSLOTSERIALIZABLE_ARRAY_PORT.fields_by_name['data'].message_type = _SLOTCOSTFUNDATAPERSLOTSERIALIZABLE
DESCRIPTOR.message_types_by_name['SlotCostFunDataPerSlotSerializable'] = _SLOTCOSTFUNDATAPERSLOTSERIALIZABLE
DESCRIPTOR.message_types_by_name['SlotCostFunDataPerSlotSerializable_array_port'] = _SLOTCOSTFUNDATAPERSLOTSERIALIZABLE_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SlotCostFunDataPerSlotSerializable = _reflection.GeneratedProtocolMessageType('SlotCostFunDataPerSlotSerializable', (_message.Message,), {
  'DESCRIPTOR' : _SLOTCOSTFUNDATAPERSLOTSERIALIZABLE,
  '__module__' : 'si.slot_cost_fun_data_per_slot_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.slot_cost_fun_data_per_slot_serializable.SlotCostFunDataPerSlotSerializable)
  })
_sym_db.RegisterMessage(SlotCostFunDataPerSlotSerializable)

SlotCostFunDataPerSlotSerializable_array_port = _reflection.GeneratedProtocolMessageType('SlotCostFunDataPerSlotSerializable_array_port', (_message.Message,), {
  'DESCRIPTOR' : _SLOTCOSTFUNDATAPERSLOTSERIALIZABLE_ARRAY_PORT,
  '__module__' : 'si.slot_cost_fun_data_per_slot_serializable_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.slot_cost_fun_data_per_slot_serializable.SlotCostFunDataPerSlotSerializable_array_port)
  })
_sym_db.RegisterMessage(SlotCostFunDataPerSlotSerializable_array_port)


# @@protoc_insertion_point(module_scope)