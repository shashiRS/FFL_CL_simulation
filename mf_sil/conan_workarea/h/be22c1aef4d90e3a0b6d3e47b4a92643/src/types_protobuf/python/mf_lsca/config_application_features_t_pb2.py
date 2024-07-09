# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_lsca/config_application_features_t.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_lsca/config_application_features_t.proto',
  package='pb.mf_lsca.config_application_features_t',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n+mf_lsca/config_application_features_t.proto\x12(pb.mf_lsca.config_application_features_t\"\xbb\x01\n\x1b\x63onfigApplicationFeatures_t\x12+\n\"treatCrowdedPedestriansAsStatic_nu\x18\xff\x0f \x01(\x08\x12+\n\"numberOfPedestriansUntilCrowded_nu\x18\xc2\x04 \x01(\r\x12!\n\x18ignoreDynamicVehicles_nu\x18\xcf\x13 \x01(\x08\x12\x1f\n\x16\x63heckLoDmcHandshake_nu\x18\x91\t \x01(\x08\"~\n&configApplicationFeatures_t_array_port\x12T\n\x04\x64\x61ta\x18\x80\x1c \x03(\x0b\x32\x45.pb.mf_lsca.config_application_features_t.configApplicationFeatures_t'
)




_CONFIGAPPLICATIONFEATURES_T = _descriptor.Descriptor(
  name='configApplicationFeatures_t',
  full_name='pb.mf_lsca.config_application_features_t.configApplicationFeatures_t',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='treatCrowdedPedestriansAsStatic_nu', full_name='pb.mf_lsca.config_application_features_t.configApplicationFeatures_t.treatCrowdedPedestriansAsStatic_nu', index=0,
      number=2047, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numberOfPedestriansUntilCrowded_nu', full_name='pb.mf_lsca.config_application_features_t.configApplicationFeatures_t.numberOfPedestriansUntilCrowded_nu', index=1,
      number=578, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ignoreDynamicVehicles_nu', full_name='pb.mf_lsca.config_application_features_t.configApplicationFeatures_t.ignoreDynamicVehicles_nu', index=2,
      number=2511, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='checkLoDmcHandshake_nu', full_name='pb.mf_lsca.config_application_features_t.configApplicationFeatures_t.checkLoDmcHandshake_nu', index=3,
      number=1169, type=8, cpp_type=7, label=1,
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
  serialized_start=90,
  serialized_end=277,
)


_CONFIGAPPLICATIONFEATURES_T_ARRAY_PORT = _descriptor.Descriptor(
  name='configApplicationFeatures_t_array_port',
  full_name='pb.mf_lsca.config_application_features_t.configApplicationFeatures_t_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_lsca.config_application_features_t.configApplicationFeatures_t_array_port.data', index=0,
      number=3584, type=11, cpp_type=10, label=3,
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
  serialized_start=279,
  serialized_end=405,
)

_CONFIGAPPLICATIONFEATURES_T_ARRAY_PORT.fields_by_name['data'].message_type = _CONFIGAPPLICATIONFEATURES_T
DESCRIPTOR.message_types_by_name['configApplicationFeatures_t'] = _CONFIGAPPLICATIONFEATURES_T
DESCRIPTOR.message_types_by_name['configApplicationFeatures_t_array_port'] = _CONFIGAPPLICATIONFEATURES_T_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

configApplicationFeatures_t = _reflection.GeneratedProtocolMessageType('configApplicationFeatures_t', (_message.Message,), {
  'DESCRIPTOR' : _CONFIGAPPLICATIONFEATURES_T,
  '__module__' : 'mf_lsca.config_application_features_t_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.config_application_features_t.configApplicationFeatures_t)
  })
_sym_db.RegisterMessage(configApplicationFeatures_t)

configApplicationFeatures_t_array_port = _reflection.GeneratedProtocolMessageType('configApplicationFeatures_t_array_port', (_message.Message,), {
  'DESCRIPTOR' : _CONFIGAPPLICATIONFEATURES_T_ARRAY_PORT,
  '__module__' : 'mf_lsca.config_application_features_t_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.config_application_features_t.configApplicationFeatures_t_array_port)
  })
_sym_db.RegisterMessage(configApplicationFeatures_t_array_port)


# @@protoc_insertion_point(module_scope)