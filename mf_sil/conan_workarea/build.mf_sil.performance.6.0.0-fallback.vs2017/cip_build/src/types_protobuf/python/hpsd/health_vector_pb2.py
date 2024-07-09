# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: hpsd/health_vector.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='hpsd/health_vector.proto',
  package='pb.hpsd.health_vector',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x18hpsd/health_vector.proto\x12\x15pb.hpsd.health_vector\"\xdd\x07\n\x0cHealthVector\x12\x1f\n\x16hostTemperatureWarning\x18\x8c\x03 \x01(\x08\x12\"\n\x19vehicleCommunicationError\x18\xa9\x13 \x01(\x08\x12 \n\x17onlySafetyCoreAvailable\x18\xda\x03 \x01(\x08\x12\x1e\n\x15hmiCommunicationError\x18\xe5\x13 \x01(\x08\x12\x1b\n\x12odometryUnreliable\x18\xa1\x0f \x01(\x08\x12\"\n\x19\x66rontCamPreProcUnreliable\x18\xcf\x1e \x01(\x08\x12!\n\x18rearCamPreProcUnreliable\x18\x8a\x1f \x01(\x08\x12!\n\x18leftCamPreProcUnreliable\x18\xb5\x11 \x01(\x08\x12\"\n\x19rightCamPreProcUnreliable\x18\xdf\x0f \x01(\x08\x12!\n\x18\x66rontCamVisionUnreliable\x18\x8f\x03 \x01(\x08\x12 \n\x17rearCamVisionUnreliable\x18\xd3\x05 \x01(\x08\x12 \n\x17leftCamVisionUnreliable\x18\xaf\x01 \x01(\x08\x12!\n\x18rightCamVisionUnreliable\x18\xd5\x10 \x01(\x08\x12\x19\n\x10\x66rontCamBlockage\x18\xb4\x02 \x01(\x08\x12\x18\n\x0frearCamBlockage\x18\xe7\x16 \x01(\x08\x12\x18\n\x0fleftCamBlockage\x18\xf5\x0b \x01(\x08\x12\x19\n\x10rightCamBlockage\x18\x86\x18 \x01(\x08\x12#\n\x1a\x66rontUltrasonicsUnreliable\x18\x90\t \x01(\x08\x12\"\n\x19rearUltrasonicsUnreliable\x18\x90\x03 \x01(\x08\x12%\n\x1c\x65nvModelStaticObjsUnreliable\x18\xcc\x1b \x01(\x08\x12/\n&envModelTrafficParticiapantsUnreliable\x18\xee\x1a \x01(\x08\x12*\n!envModelParkingFeaturesUnreliable\x18\xb5\x0f \x01(\x08\x12\x1f\n\x16localizationUnreliable\x18\xc4\x04 \x01(\x08\x12\x16\n\rpdwUnreliable\x18\xec\x1f \x01(\x08\x12\x17\n\x0elscaUnreliable\x18\xd9\x14 \x01(\x08\x12\x16\n\raupUnreliable\x18\xa3\x03 \x01(\x08\x12\x15\n\x0craUnreliable\x18\xe4\x01 \x01(\x08\x12\x15\n\x0chvUnreliable\x18\xf6\x0c \x01(\x08\x12\x13\n\navgaFailed\x18\xcf\x15 \x01(\x08\x12\x18\n\x0fmfManagerFailed\x18\xbe\x02 \x01(\x08\x12\x13\n\nmocoFailed\x18\xf6\x02 \x01(\x08\"M\n\x17HealthVector_array_port\x12\x32\n\x04\x64\x61ta\x18\xde\x18 \x03(\x0b\x32#.pb.hpsd.health_vector.HealthVector'
)




_HEALTHVECTOR = _descriptor.Descriptor(
  name='HealthVector',
  full_name='pb.hpsd.health_vector.HealthVector',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='hostTemperatureWarning', full_name='pb.hpsd.health_vector.HealthVector.hostTemperatureWarning', index=0,
      number=396, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='vehicleCommunicationError', full_name='pb.hpsd.health_vector.HealthVector.vehicleCommunicationError', index=1,
      number=2473, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='onlySafetyCoreAvailable', full_name='pb.hpsd.health_vector.HealthVector.onlySafetyCoreAvailable', index=2,
      number=474, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='hmiCommunicationError', full_name='pb.hpsd.health_vector.HealthVector.hmiCommunicationError', index=3,
      number=2533, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='odometryUnreliable', full_name='pb.hpsd.health_vector.HealthVector.odometryUnreliable', index=4,
      number=1953, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frontCamPreProcUnreliable', full_name='pb.hpsd.health_vector.HealthVector.frontCamPreProcUnreliable', index=5,
      number=3919, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearCamPreProcUnreliable', full_name='pb.hpsd.health_vector.HealthVector.rearCamPreProcUnreliable', index=6,
      number=3978, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='leftCamPreProcUnreliable', full_name='pb.hpsd.health_vector.HealthVector.leftCamPreProcUnreliable', index=7,
      number=2229, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rightCamPreProcUnreliable', full_name='pb.hpsd.health_vector.HealthVector.rightCamPreProcUnreliable', index=8,
      number=2015, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frontCamVisionUnreliable', full_name='pb.hpsd.health_vector.HealthVector.frontCamVisionUnreliable', index=9,
      number=399, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearCamVisionUnreliable', full_name='pb.hpsd.health_vector.HealthVector.rearCamVisionUnreliable', index=10,
      number=723, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='leftCamVisionUnreliable', full_name='pb.hpsd.health_vector.HealthVector.leftCamVisionUnreliable', index=11,
      number=175, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rightCamVisionUnreliable', full_name='pb.hpsd.health_vector.HealthVector.rightCamVisionUnreliable', index=12,
      number=2133, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frontCamBlockage', full_name='pb.hpsd.health_vector.HealthVector.frontCamBlockage', index=13,
      number=308, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearCamBlockage', full_name='pb.hpsd.health_vector.HealthVector.rearCamBlockage', index=14,
      number=2919, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='leftCamBlockage', full_name='pb.hpsd.health_vector.HealthVector.leftCamBlockage', index=15,
      number=1525, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rightCamBlockage', full_name='pb.hpsd.health_vector.HealthVector.rightCamBlockage', index=16,
      number=3078, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frontUltrasonicsUnreliable', full_name='pb.hpsd.health_vector.HealthVector.frontUltrasonicsUnreliable', index=17,
      number=1168, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rearUltrasonicsUnreliable', full_name='pb.hpsd.health_vector.HealthVector.rearUltrasonicsUnreliable', index=18,
      number=400, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='envModelStaticObjsUnreliable', full_name='pb.hpsd.health_vector.HealthVector.envModelStaticObjsUnreliable', index=19,
      number=3532, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='envModelTrafficParticiapantsUnreliable', full_name='pb.hpsd.health_vector.HealthVector.envModelTrafficParticiapantsUnreliable', index=20,
      number=3438, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='envModelParkingFeaturesUnreliable', full_name='pb.hpsd.health_vector.HealthVector.envModelParkingFeaturesUnreliable', index=21,
      number=1973, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='localizationUnreliable', full_name='pb.hpsd.health_vector.HealthVector.localizationUnreliable', index=22,
      number=580, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pdwUnreliable', full_name='pb.hpsd.health_vector.HealthVector.pdwUnreliable', index=23,
      number=4076, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='lscaUnreliable', full_name='pb.hpsd.health_vector.HealthVector.lscaUnreliable', index=24,
      number=2649, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='aupUnreliable', full_name='pb.hpsd.health_vector.HealthVector.aupUnreliable', index=25,
      number=419, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='raUnreliable', full_name='pb.hpsd.health_vector.HealthVector.raUnreliable', index=26,
      number=228, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='hvUnreliable', full_name='pb.hpsd.health_vector.HealthVector.hvUnreliable', index=27,
      number=1654, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='avgaFailed', full_name='pb.hpsd.health_vector.HealthVector.avgaFailed', index=28,
      number=2767, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='mfManagerFailed', full_name='pb.hpsd.health_vector.HealthVector.mfManagerFailed', index=29,
      number=318, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='mocoFailed', full_name='pb.hpsd.health_vector.HealthVector.mocoFailed', index=30,
      number=374, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_start=52,
  serialized_end=1041,
)


_HEALTHVECTOR_ARRAY_PORT = _descriptor.Descriptor(
  name='HealthVector_array_port',
  full_name='pb.hpsd.health_vector.HealthVector_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.hpsd.health_vector.HealthVector_array_port.data', index=0,
      number=3166, type=11, cpp_type=10, label=3,
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
  serialized_start=1043,
  serialized_end=1120,
)

_HEALTHVECTOR_ARRAY_PORT.fields_by_name['data'].message_type = _HEALTHVECTOR
DESCRIPTOR.message_types_by_name['HealthVector'] = _HEALTHVECTOR
DESCRIPTOR.message_types_by_name['HealthVector_array_port'] = _HEALTHVECTOR_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

HealthVector = _reflection.GeneratedProtocolMessageType('HealthVector', (_message.Message,), {
  'DESCRIPTOR' : _HEALTHVECTOR,
  '__module__' : 'hpsd.health_vector_pb2'
  # @@protoc_insertion_point(class_scope:pb.hpsd.health_vector.HealthVector)
  })
_sym_db.RegisterMessage(HealthVector)

HealthVector_array_port = _reflection.GeneratedProtocolMessageType('HealthVector_array_port', (_message.Message,), {
  'DESCRIPTOR' : _HEALTHVECTOR_ARRAY_PORT,
  '__module__' : 'hpsd.health_vector_pb2'
  # @@protoc_insertion_point(class_scope:pb.hpsd.health_vector.HealthVector_array_port)
  })
_sym_db.RegisterMessage(HealthVector_array_port)


# @@protoc_insertion_point(module_scope)
