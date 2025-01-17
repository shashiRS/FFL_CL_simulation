# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_envelope_data.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from us_drv import us_drv_sensor_state_pb2 as us__drv_dot_us__drv__sensor__state__pb2
from us_drv import us_drv_envelope_signal_path_pb2 as us__drv_dot_us__drv__envelope__signal__path__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_envelope_data.proto',
  package='pb.us_drv.us_drv_envelope_data',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n!us_drv/us_drv_envelope_data.proto\x12\x1epb.us_drv.us_drv_envelope_data\x1a\x17\x65\x63o/signal_header.proto\x1a us_drv/us_drv_sensor_state.proto\x1a(us_drv/us_drv_envelope_signal_path.proto\"\xc2\x02\n\x11UsDrvEnvelopeData\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x45\n\x0bsensorState\x18\xbf\x18 \x03(\x0e\x32/.pb.us_drv.us_drv_sensor_state.UsDrvSensorState\x12\x17\n\x0enumSignalPaths\x18\xe0\x0e \x01(\r\x12T\n\x0bsignalPaths\x18\xd8\t \x03(\x0b\x32>.pb.us_drv.us_drv_envelope_signal_path.UsDrvEnvelopeSignalPath\x12\x13\n\nnumSamples\x18\xe1\x07 \x01(\r\x12\x0f\n\x07samples\x18\x16 \x03(\r\"`\n\x1cUsDrvEnvelopeData_array_port\x12@\n\x04\x64\x61ta\x18\x96\x1b \x03(\x0b\x32\x31.pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,us__drv_dot_us__drv__sensor__state__pb2.DESCRIPTOR,us__drv_dot_us__drv__envelope__signal__path__pb2.DESCRIPTOR,])




_USDRVENVELOPEDATA = _descriptor.Descriptor(
  name='UsDrvEnvelopeData',
  full_name='pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sensorState', full_name='pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.sensorState', index=2,
      number=3135, type=14, cpp_type=8, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numSignalPaths', full_name='pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.numSignalPaths', index=3,
      number=1888, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='signalPaths', full_name='pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.signalPaths', index=4,
      number=1240, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numSamples', full_name='pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.numSamples', index=5,
      number=993, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='samples', full_name='pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData.samples', index=6,
      number=22, type=13, cpp_type=3, label=3,
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
  serialized_start=171,
  serialized_end=493,
)


_USDRVENVELOPEDATA_ARRAY_PORT = _descriptor.Descriptor(
  name='UsDrvEnvelopeData_array_port',
  full_name='pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port.data', index=0,
      number=3478, type=11, cpp_type=10, label=3,
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
  serialized_start=495,
  serialized_end=591,
)

_USDRVENVELOPEDATA.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_USDRVENVELOPEDATA.fields_by_name['sensorState'].enum_type = us__drv_dot_us__drv__sensor__state__pb2._USDRVSENSORSTATE
_USDRVENVELOPEDATA.fields_by_name['signalPaths'].message_type = us__drv_dot_us__drv__envelope__signal__path__pb2._USDRVENVELOPESIGNALPATH
_USDRVENVELOPEDATA_ARRAY_PORT.fields_by_name['data'].message_type = _USDRVENVELOPEDATA
DESCRIPTOR.message_types_by_name['UsDrvEnvelopeData'] = _USDRVENVELOPEDATA
DESCRIPTOR.message_types_by_name['UsDrvEnvelopeData_array_port'] = _USDRVENVELOPEDATA_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsDrvEnvelopeData = _reflection.GeneratedProtocolMessageType('UsDrvEnvelopeData', (_message.Message,), {
  'DESCRIPTOR' : _USDRVENVELOPEDATA,
  '__module__' : 'us_drv.us_drv_envelope_data_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData)
  })
_sym_db.RegisterMessage(UsDrvEnvelopeData)

UsDrvEnvelopeData_array_port = _reflection.GeneratedProtocolMessageType('UsDrvEnvelopeData_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USDRVENVELOPEDATA_ARRAY_PORT,
  '__module__' : 'us_drv.us_drv_envelope_data_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_envelope_data.UsDrvEnvelopeData_array_port)
  })
_sym_db.RegisterMessage(UsDrvEnvelopeData_array_port)


# @@protoc_insertion_point(module_scope)
