# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_commonvehsigprovider/gpsdata.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ap_commonvehsigprovider import hemisphere_pb2 as ap__commonvehsigprovider_dot_hemisphere__pb2
from ap_commonvehsigprovider import gps_receiver_status_pb2 as ap__commonvehsigprovider_dot_gps__receiver__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_commonvehsigprovider/gpsdata.proto',
  package='pb.ap_commonvehsigprovider.gpsdata',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n%ap_commonvehsigprovider/gpsdata.proto\x12\"pb.ap_commonvehsigprovider.gpsdata\x1a(ap_commonvehsigprovider/hemisphere.proto\x1a\x31\x61p_commonvehsigprovider/gps_receiver_status.proto\"\x9e\x06\n\x07GPSData\x12\x1b\n\x12gpsAntennaHeight_m\x18\x8f\x1b \x01(\x02\x12\x17\n\x0egpsLatitude_dd\x18\xdc\x07 \x01(\x11\x12\x17\n\x0egpsLatitude_mm\x18\xf4\x02 \x01(\x02\x12\x18\n\x0fgpsLongitude_dd\x18\x85\x1d \x01(\x11\x12\x18\n\x0fgpsLongitude_mm\x18\xff\x12 \x01(\x02\x12\x15\n\x0cgpsSpeed_mps\x18\x9b\x18 \x01(\x02\x12\"\n\x19gpsR32SpeedOverGround_mps\x18\xe9\x1a \x01(\x02\x12\x1c\n\x13gpsCourseOverGround\x18\xc8\x17 \x01(\x02\x12\x14\n\x0bverticalDOP\x18\xe5\x12 \x01(\x02\x12\x16\n\rhorizontalDOP\x18\xb4\x11 \x01(\x02\x12\x10\n\x07timeDOP\x18\xb4\x1a \x01(\x02\x12\x15\n\x0cgeometricDOP\x18\xbc\x0e \x01(\x02\x12\x14\n\x0bpositionDOP\x18\x90\n \x01(\x02\x12\x16\n\rgpsUtcTime_hh\x18\xcd\x0f \x01(\r\x12\x16\n\rgpsUtcTime_mm\x18\x9d\x11 \x01(\r\x12\x16\n\rgpsUtcTime_ss\x18\x9e\x16 \x01(\r\x12T\n\x18gpsLatitudeHemisphere_nu\x18\xd3\x15 \x01(\x0e\x32\x31.pb.ap_commonvehsigprovider.hemisphere.Hemisphere\x12U\n\x19gpsLongitudeHemisphere_nu\x18\x98\x14 \x01(\x0e\x32\x31.pb.ap_commonvehsigprovider.hemisphere.Hemisphere\x12\x16\n\rgpsDateDay_dd\x18\xf8\x06 \x01(\r\x12\x18\n\x0fgpsDateMonth_mm\x18\x8c\x17 \x01(\r\x12\x17\n\x0egpsDateYear_yy\x18\x85\x1b \x01(\r\x12\x0f\n\x06gpsFix\x18\xed\x1b \x01(\r\x12\x1a\n\x11gpsNoOfSatellites\x18\x96\x01 \x01(\r\x12]\n\x11ReceiverStatus_nu\x18\xd2\r \x01(\x0e\x32\x41.pb.ap_commonvehsigprovider.gps_receiver_status.GpsReceiverStatus\"P\n\x12GPSData_array_port\x12:\n\x04\x64\x61ta\x18\xf5\x0f \x03(\x0b\x32+.pb.ap_commonvehsigprovider.gpsdata.GPSData'
  ,
  dependencies=[ap__commonvehsigprovider_dot_hemisphere__pb2.DESCRIPTOR,ap__commonvehsigprovider_dot_gps__receiver__status__pb2.DESCRIPTOR,])




_GPSDATA = _descriptor.Descriptor(
  name='GPSData',
  full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='gpsAntennaHeight_m', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsAntennaHeight_m', index=0,
      number=3471, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsLatitude_dd', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsLatitude_dd', index=1,
      number=988, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsLatitude_mm', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsLatitude_mm', index=2,
      number=372, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsLongitude_dd', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsLongitude_dd', index=3,
      number=3717, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsLongitude_mm', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsLongitude_mm', index=4,
      number=2431, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsSpeed_mps', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsSpeed_mps', index=5,
      number=3099, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsR32SpeedOverGround_mps', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsR32SpeedOverGround_mps', index=6,
      number=3433, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsCourseOverGround', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsCourseOverGround', index=7,
      number=3016, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='verticalDOP', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.verticalDOP', index=8,
      number=2405, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='horizontalDOP', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.horizontalDOP', index=9,
      number=2228, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timeDOP', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.timeDOP', index=10,
      number=3380, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='geometricDOP', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.geometricDOP', index=11,
      number=1852, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='positionDOP', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.positionDOP', index=12,
      number=1296, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsUtcTime_hh', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsUtcTime_hh', index=13,
      number=1997, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsUtcTime_mm', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsUtcTime_mm', index=14,
      number=2205, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsUtcTime_ss', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsUtcTime_ss', index=15,
      number=2846, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsLatitudeHemisphere_nu', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsLatitudeHemisphere_nu', index=16,
      number=2771, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsLongitudeHemisphere_nu', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsLongitudeHemisphere_nu', index=17,
      number=2584, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsDateDay_dd', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsDateDay_dd', index=18,
      number=888, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsDateMonth_mm', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsDateMonth_mm', index=19,
      number=2956, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsDateYear_yy', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsDateYear_yy', index=20,
      number=3461, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsFix', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsFix', index=21,
      number=3565, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gpsNoOfSatellites', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.gpsNoOfSatellites', index=22,
      number=150, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ReceiverStatus_nu', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData.ReceiverStatus_nu', index=23,
      number=1746, type=14, cpp_type=8, label=1,
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
  serialized_start=171,
  serialized_end=969,
)


_GPSDATA_ARRAY_PORT = _descriptor.Descriptor(
  name='GPSData_array_port',
  full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_commonvehsigprovider.gpsdata.GPSData_array_port.data', index=0,
      number=2037, type=11, cpp_type=10, label=3,
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
  serialized_start=971,
  serialized_end=1051,
)

_GPSDATA.fields_by_name['gpsLatitudeHemisphere_nu'].enum_type = ap__commonvehsigprovider_dot_hemisphere__pb2._HEMISPHERE
_GPSDATA.fields_by_name['gpsLongitudeHemisphere_nu'].enum_type = ap__commonvehsigprovider_dot_hemisphere__pb2._HEMISPHERE
_GPSDATA.fields_by_name['ReceiverStatus_nu'].enum_type = ap__commonvehsigprovider_dot_gps__receiver__status__pb2._GPSRECEIVERSTATUS
_GPSDATA_ARRAY_PORT.fields_by_name['data'].message_type = _GPSDATA
DESCRIPTOR.message_types_by_name['GPSData'] = _GPSDATA
DESCRIPTOR.message_types_by_name['GPSData_array_port'] = _GPSDATA_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

GPSData = _reflection.GeneratedProtocolMessageType('GPSData', (_message.Message,), {
  'DESCRIPTOR' : _GPSDATA,
  '__module__' : 'ap_commonvehsigprovider.gpsdata_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.gpsdata.GPSData)
  })
_sym_db.RegisterMessage(GPSData)

GPSData_array_port = _reflection.GeneratedProtocolMessageType('GPSData_array_port', (_message.Message,), {
  'DESCRIPTOR' : _GPSDATA_ARRAY_PORT,
  '__module__' : 'ap_commonvehsigprovider.gpsdata_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.gpsdata.GPSData_array_port)
  })
_sym_db.RegisterMessage(GPSData_array_port)


# @@protoc_insertion_point(module_scope)
