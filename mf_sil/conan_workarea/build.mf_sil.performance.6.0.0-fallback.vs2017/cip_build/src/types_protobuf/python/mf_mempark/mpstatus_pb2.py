# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_mempark/mpstatus.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from mf_mempark import memorized_slot_pb2 as mf__mempark_dot_memorized__slot__pb2
from mf_mempark import memorized_parking_status_pb2 as mf__mempark_dot_memorized__parking__status__pb2
from mf_mempark import training_status_pb2 as mf__mempark_dot_training__status__pb2
from mf_mempark import localization_status_pb2 as mf__mempark_dot_localization__status__pb2
from mf_mempark import user_update_request_status_pb2 as mf__mempark_dot_user__update__request__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_mempark/mpstatus.proto',
  package='pb.mf_mempark.mpstatus',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x19mf_mempark/mpstatus.proto\x12\x16pb.mf_mempark.mpstatus\x1a\x1fmf_mempark/memorized_slot.proto\x1a)mf_mempark/memorized_parking_status.proto\x1a mf_mempark/training_status.proto\x1a$mf_mempark/localization_status.proto\x1a+mf_mempark/user_update_request_status.proto\"\xde\x03\n\x08MPStatus\x12\'\n\x1enumStoredMemoryParkingSlots_nu\x18\xf7\x16 \x01(\r\x12K\n\x15memorizedParkingSlots\x18\xcd\x1c \x03(\x0b\x32+.pb.mf_mempark.memorized_slot.MemorizedSlot\x12[\n\x12memoryParkingState\x18\xe2\x06 \x01(\x0e\x32>.pb.mf_mempark.memorized_parking_status.MemorizedParkingStatus\x12\x46\n\x0etrainingStatus\x18\xe4\x04 \x01(\x0e\x32-.pb.mf_mempark.training_status.TrainingStatus\x12R\n\x12localizationStatus\x18\xf0\x0c \x01(\x0e\x32\x35.pb.mf_mempark.localization_status.LocalizationStatus\x12\x63\n\x17userUpdateRequestStatus\x18\xe3\x0c \x01(\x0e\x32\x41.pb.mf_mempark.user_update_request_status.UserUpdateRequestStatus\"F\n\x13MPStatus_array_port\x12/\n\x04\x64\x61ta\x18\x84\x0e \x03(\x0b\x32 .pb.mf_mempark.mpstatus.MPStatus'
  ,
  dependencies=[mf__mempark_dot_memorized__slot__pb2.DESCRIPTOR,mf__mempark_dot_memorized__parking__status__pb2.DESCRIPTOR,mf__mempark_dot_training__status__pb2.DESCRIPTOR,mf__mempark_dot_localization__status__pb2.DESCRIPTOR,mf__mempark_dot_user__update__request__status__pb2.DESCRIPTOR,])




_MPSTATUS = _descriptor.Descriptor(
  name='MPStatus',
  full_name='pb.mf_mempark.mpstatus.MPStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='numStoredMemoryParkingSlots_nu', full_name='pb.mf_mempark.mpstatus.MPStatus.numStoredMemoryParkingSlots_nu', index=0,
      number=2935, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='memorizedParkingSlots', full_name='pb.mf_mempark.mpstatus.MPStatus.memorizedParkingSlots', index=1,
      number=3661, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='memoryParkingState', full_name='pb.mf_mempark.mpstatus.MPStatus.memoryParkingState', index=2,
      number=866, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='trainingStatus', full_name='pb.mf_mempark.mpstatus.MPStatus.trainingStatus', index=3,
      number=612, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='localizationStatus', full_name='pb.mf_mempark.mpstatus.MPStatus.localizationStatus', index=4,
      number=1648, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='userUpdateRequestStatus', full_name='pb.mf_mempark.mpstatus.MPStatus.userUpdateRequestStatus', index=5,
      number=1635, type=14, cpp_type=8, label=1,
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
  serialized_start=247,
  serialized_end=725,
)


_MPSTATUS_ARRAY_PORT = _descriptor.Descriptor(
  name='MPStatus_array_port',
  full_name='pb.mf_mempark.mpstatus.MPStatus_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_mempark.mpstatus.MPStatus_array_port.data', index=0,
      number=1796, type=11, cpp_type=10, label=3,
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
  serialized_start=727,
  serialized_end=797,
)

_MPSTATUS.fields_by_name['memorizedParkingSlots'].message_type = mf__mempark_dot_memorized__slot__pb2._MEMORIZEDSLOT
_MPSTATUS.fields_by_name['memoryParkingState'].enum_type = mf__mempark_dot_memorized__parking__status__pb2._MEMORIZEDPARKINGSTATUS
_MPSTATUS.fields_by_name['trainingStatus'].enum_type = mf__mempark_dot_training__status__pb2._TRAININGSTATUS
_MPSTATUS.fields_by_name['localizationStatus'].enum_type = mf__mempark_dot_localization__status__pb2._LOCALIZATIONSTATUS
_MPSTATUS.fields_by_name['userUpdateRequestStatus'].enum_type = mf__mempark_dot_user__update__request__status__pb2._USERUPDATEREQUESTSTATUS
_MPSTATUS_ARRAY_PORT.fields_by_name['data'].message_type = _MPSTATUS
DESCRIPTOR.message_types_by_name['MPStatus'] = _MPSTATUS
DESCRIPTOR.message_types_by_name['MPStatus_array_port'] = _MPSTATUS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MPStatus = _reflection.GeneratedProtocolMessageType('MPStatus', (_message.Message,), {
  'DESCRIPTOR' : _MPSTATUS,
  '__module__' : 'mf_mempark.mpstatus_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.mpstatus.MPStatus)
  })
_sym_db.RegisterMessage(MPStatus)

MPStatus_array_port = _reflection.GeneratedProtocolMessageType('MPStatus_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MPSTATUS_ARRAY_PORT,
  '__module__' : 'mf_mempark.mpstatus_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_mempark.mpstatus.MPStatus_array_port)
  })
_sym_db.RegisterMessage(MPStatus_array_port)


# @@protoc_insertion_point(module_scope)