# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_tp/stored_waypoint_data.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from lsm_geoml import pose_pod_pb2 as lsm__geoml_dot_pose__pod__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_tp/stored_waypoint_data.proto',
  package='pb.ap_tp.stored_waypoint_data',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n ap_tp/stored_waypoint_data.proto\x12\x1dpb.ap_tp.stored_waypoint_data\x1a\x18lsm_geoml/pose_pod.proto\"V\n\x12StoredWaypointData\x12.\n\x04pose\x18\xe9\x0b \x01(\x0b\x32\x1f.pb.lsm_geoml.pose_pod.Pose_POD\x12\x10\n\x07\x63rv_1pm\x18\xe7\x0e \x01(\x02\"a\n\x1dStoredWaypointData_array_port\x12@\n\x04\x64\x61ta\x18\xd0\x0f \x03(\x0b\x32\x31.pb.ap_tp.stored_waypoint_data.StoredWaypointData'
  ,
  dependencies=[lsm__geoml_dot_pose__pod__pb2.DESCRIPTOR,])




_STOREDWAYPOINTDATA = _descriptor.Descriptor(
  name='StoredWaypointData',
  full_name='pb.ap_tp.stored_waypoint_data.StoredWaypointData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='pose', full_name='pb.ap_tp.stored_waypoint_data.StoredWaypointData.pose', index=0,
      number=1513, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='crv_1pm', full_name='pb.ap_tp.stored_waypoint_data.StoredWaypointData.crv_1pm', index=1,
      number=1895, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_end=179,
)


_STOREDWAYPOINTDATA_ARRAY_PORT = _descriptor.Descriptor(
  name='StoredWaypointData_array_port',
  full_name='pb.ap_tp.stored_waypoint_data.StoredWaypointData_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_tp.stored_waypoint_data.StoredWaypointData_array_port.data', index=0,
      number=2000, type=11, cpp_type=10, label=3,
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
  serialized_start=181,
  serialized_end=278,
)

_STOREDWAYPOINTDATA.fields_by_name['pose'].message_type = lsm__geoml_dot_pose__pod__pb2._POSE_POD
_STOREDWAYPOINTDATA_ARRAY_PORT.fields_by_name['data'].message_type = _STOREDWAYPOINTDATA
DESCRIPTOR.message_types_by_name['StoredWaypointData'] = _STOREDWAYPOINTDATA
DESCRIPTOR.message_types_by_name['StoredWaypointData_array_port'] = _STOREDWAYPOINTDATA_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

StoredWaypointData = _reflection.GeneratedProtocolMessageType('StoredWaypointData', (_message.Message,), {
  'DESCRIPTOR' : _STOREDWAYPOINTDATA,
  '__module__' : 'ap_tp.stored_waypoint_data_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.stored_waypoint_data.StoredWaypointData)
  })
_sym_db.RegisterMessage(StoredWaypointData)

StoredWaypointData_array_port = _reflection.GeneratedProtocolMessageType('StoredWaypointData_array_port', (_message.Message,), {
  'DESCRIPTOR' : _STOREDWAYPOINTDATA_ARRAY_PORT,
  '__module__' : 'ap_tp.stored_waypoint_data_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_tp.stored_waypoint_data.StoredWaypointData_array_port)
  })
_sym_db.RegisterMessage(StoredWaypointData_array_port)


# @@protoc_insertion_point(module_scope)
