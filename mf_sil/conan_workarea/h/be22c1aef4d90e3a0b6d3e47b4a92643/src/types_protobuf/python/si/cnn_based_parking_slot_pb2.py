# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: si/cnn_based_parking_slot.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from si import quadrilateral_serializable_pb2 as si_dot_quadrilateral__serializable__pb2
from si import parking_scenario_confidence_pb2 as si_dot_parking__scenario__confidence__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='si/cnn_based_parking_slot.proto',
  package='pb.si.cnn_based_parking_slot',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1fsi/cnn_based_parking_slot.proto\x12\x1cpb.si.cnn_based_parking_slot\x1a#si/quadrilateral_serializable.proto\x1a$si/parking_scenario_confidence.proto\"\x80\x02\n\x13\x43nnBasedParkingSlot\x12\x12\n\tslotId_nu\x18\xdb\x1b \x01(\r\x12Q\n\x0bslotShape_m\x18\x97\x17 \x01(\x0b\x32;.pb.si.quadrilateral_serializable.QuadrilateralSerializable\x12\x1b\n\x12\x65xistenceProb_perc\x18\xf8\x12 \x01(\r\x12\x65\n\x1eparkingScenarioConfidence_perc\x18\xb4\x04 \x01(\x0b\x32<.pb.si.parking_scenario_confidence.ParkingScenarioConfidence\"b\n\x1e\x43nnBasedParkingSlot_array_port\x12@\n\x04\x64\x61ta\x18\xdc\x1b \x03(\x0b\x32\x31.pb.si.cnn_based_parking_slot.CnnBasedParkingSlot'
  ,
  dependencies=[si_dot_quadrilateral__serializable__pb2.DESCRIPTOR,si_dot_parking__scenario__confidence__pb2.DESCRIPTOR,])




_CNNBASEDPARKINGSLOT = _descriptor.Descriptor(
  name='CnnBasedParkingSlot',
  full_name='pb.si.cnn_based_parking_slot.CnnBasedParkingSlot',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='slotId_nu', full_name='pb.si.cnn_based_parking_slot.CnnBasedParkingSlot.slotId_nu', index=0,
      number=3547, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='slotShape_m', full_name='pb.si.cnn_based_parking_slot.CnnBasedParkingSlot.slotShape_m', index=1,
      number=2967, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='existenceProb_perc', full_name='pb.si.cnn_based_parking_slot.CnnBasedParkingSlot.existenceProb_perc', index=2,
      number=2424, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='parkingScenarioConfidence_perc', full_name='pb.si.cnn_based_parking_slot.CnnBasedParkingSlot.parkingScenarioConfidence_perc', index=3,
      number=564, type=11, cpp_type=10, label=1,
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
  serialized_start=141,
  serialized_end=397,
)


_CNNBASEDPARKINGSLOT_ARRAY_PORT = _descriptor.Descriptor(
  name='CnnBasedParkingSlot_array_port',
  full_name='pb.si.cnn_based_parking_slot.CnnBasedParkingSlot_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.si.cnn_based_parking_slot.CnnBasedParkingSlot_array_port.data', index=0,
      number=3548, type=11, cpp_type=10, label=3,
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
  serialized_start=399,
  serialized_end=497,
)

_CNNBASEDPARKINGSLOT.fields_by_name['slotShape_m'].message_type = si_dot_quadrilateral__serializable__pb2._QUADRILATERALSERIALIZABLE
_CNNBASEDPARKINGSLOT.fields_by_name['parkingScenarioConfidence_perc'].message_type = si_dot_parking__scenario__confidence__pb2._PARKINGSCENARIOCONFIDENCE
_CNNBASEDPARKINGSLOT_ARRAY_PORT.fields_by_name['data'].message_type = _CNNBASEDPARKINGSLOT
DESCRIPTOR.message_types_by_name['CnnBasedParkingSlot'] = _CNNBASEDPARKINGSLOT
DESCRIPTOR.message_types_by_name['CnnBasedParkingSlot_array_port'] = _CNNBASEDPARKINGSLOT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

CnnBasedParkingSlot = _reflection.GeneratedProtocolMessageType('CnnBasedParkingSlot', (_message.Message,), {
  'DESCRIPTOR' : _CNNBASEDPARKINGSLOT,
  '__module__' : 'si.cnn_based_parking_slot_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.cnn_based_parking_slot.CnnBasedParkingSlot)
  })
_sym_db.RegisterMessage(CnnBasedParkingSlot)

CnnBasedParkingSlot_array_port = _reflection.GeneratedProtocolMessageType('CnnBasedParkingSlot_array_port', (_message.Message,), {
  'DESCRIPTOR' : _CNNBASEDPARKINGSLOT_ARRAY_PORT,
  '__module__' : 'si.cnn_based_parking_slot_pb2'
  # @@protoc_insertion_point(class_scope:pb.si.cnn_based_parking_slot.CnnBasedParkingSlot_array_port)
  })
_sym_db.RegisterMessage(CnnBasedParkingSlot_array_port)


# @@protoc_insertion_point(module_scope)
