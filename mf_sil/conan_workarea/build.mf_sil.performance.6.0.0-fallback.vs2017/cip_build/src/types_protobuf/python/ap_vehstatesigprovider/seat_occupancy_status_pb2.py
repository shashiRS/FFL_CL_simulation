# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_vehstatesigprovider/seat_occupancy_status.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ap_vehstatesigprovider import seat_occupancy_pb2 as ap__vehstatesigprovider_dot_seat__occupancy__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_vehstatesigprovider/seat_occupancy_status.proto',
  package='pb.ap_vehstatesigprovider.seat_occupancy_status',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n2ap_vehstatesigprovider/seat_occupancy_status.proto\x12/pb.ap_vehstatesigprovider.seat_occupancy_status\x1a+ap_vehstatesigprovider/seat_occupancy.proto\"\xd7\x02\n\x13SeatOccupancyStatus\x12N\n\x0c\x66rontPsgr_nu\x18\xe4\x02 \x01(\x0e\x32\x37.pb.ap_vehstatesigprovider.seat_occupancy.SeatOccupancy\x12K\n\tdriver_nu\x18\xed\x0c \x01(\x0e\x32\x37.pb.ap_vehstatesigprovider.seat_occupancy.SeatOccupancy\x12Q\n\x0f\x62\x61\x63krowRight_nu\x18\xa4\x04 \x01(\x0e\x32\x37.pb.ap_vehstatesigprovider.seat_occupancy.SeatOccupancy\x12P\n\x0e\x62\x61\x63krowLeft_nu\x18\xdb\x10 \x01(\x0e\x32\x37.pb.ap_vehstatesigprovider.seat_occupancy.SeatOccupancy\"u\n\x1eSeatOccupancyStatus_array_port\x12S\n\x04\x64\x61ta\x18\xd6\x19 \x03(\x0b\x32\x44.pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus'
  ,
  dependencies=[ap__vehstatesigprovider_dot_seat__occupancy__pb2.DESCRIPTOR,])




_SEATOCCUPANCYSTATUS = _descriptor.Descriptor(
  name='SeatOccupancyStatus',
  full_name='pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='frontPsgr_nu', full_name='pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus.frontPsgr_nu', index=0,
      number=356, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='driver_nu', full_name='pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus.driver_nu', index=1,
      number=1645, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='backrowRight_nu', full_name='pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus.backrowRight_nu', index=2,
      number=548, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='backrowLeft_nu', full_name='pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus.backrowLeft_nu', index=3,
      number=2139, type=14, cpp_type=8, label=1,
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
  serialized_end=492,
)


_SEATOCCUPANCYSTATUS_ARRAY_PORT = _descriptor.Descriptor(
  name='SeatOccupancyStatus_array_port',
  full_name='pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus_array_port.data', index=0,
      number=3286, type=11, cpp_type=10, label=3,
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
  serialized_start=494,
  serialized_end=611,
)

_SEATOCCUPANCYSTATUS.fields_by_name['frontPsgr_nu'].enum_type = ap__vehstatesigprovider_dot_seat__occupancy__pb2._SEATOCCUPANCY
_SEATOCCUPANCYSTATUS.fields_by_name['driver_nu'].enum_type = ap__vehstatesigprovider_dot_seat__occupancy__pb2._SEATOCCUPANCY
_SEATOCCUPANCYSTATUS.fields_by_name['backrowRight_nu'].enum_type = ap__vehstatesigprovider_dot_seat__occupancy__pb2._SEATOCCUPANCY
_SEATOCCUPANCYSTATUS.fields_by_name['backrowLeft_nu'].enum_type = ap__vehstatesigprovider_dot_seat__occupancy__pb2._SEATOCCUPANCY
_SEATOCCUPANCYSTATUS_ARRAY_PORT.fields_by_name['data'].message_type = _SEATOCCUPANCYSTATUS
DESCRIPTOR.message_types_by_name['SeatOccupancyStatus'] = _SEATOCCUPANCYSTATUS
DESCRIPTOR.message_types_by_name['SeatOccupancyStatus_array_port'] = _SEATOCCUPANCYSTATUS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SeatOccupancyStatus = _reflection.GeneratedProtocolMessageType('SeatOccupancyStatus', (_message.Message,), {
  'DESCRIPTOR' : _SEATOCCUPANCYSTATUS,
  '__module__' : 'ap_vehstatesigprovider.seat_occupancy_status_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus)
  })
_sym_db.RegisterMessage(SeatOccupancyStatus)

SeatOccupancyStatus_array_port = _reflection.GeneratedProtocolMessageType('SeatOccupancyStatus_array_port', (_message.Message,), {
  'DESCRIPTOR' : _SEATOCCUPANCYSTATUS_ARRAY_PORT,
  '__module__' : 'ap_vehstatesigprovider.seat_occupancy_status_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus_array_port)
  })
_sym_db.RegisterMessage(SeatOccupancyStatus_array_port)


# @@protoc_insertion_point(module_scope)
