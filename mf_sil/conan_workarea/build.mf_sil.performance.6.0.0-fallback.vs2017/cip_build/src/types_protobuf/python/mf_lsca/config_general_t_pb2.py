# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_lsca/config_general_t.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from cml import vec2_df_pod_pb2 as cml_dot_vec2__df__pod__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_lsca/config_general_t.proto',
  package='pb.mf_lsca.config_general_t',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1emf_lsca/config_general_t.proto\x12\x1bpb.mf_lsca.config_general_t\x1a\x15\x63ml/vec2_df_pod.proto\"\xe1\n\n\x0f\x63onfigGeneral_t\x12!\n\x18simpleShapeActualSize_nu\x18\x85\x10 \x01(\r\x12\x34\n\x0bsimpleShape\x18\x8d\x0c \x03(\x0b\x32\x1e.pb.cml.vec2_df_pod.Vec2Df_POD\x12(\n\x1f\x66orwardLeftIndicesActualSize_nu\x18\xd7\x16 \x01(\r\x12\x1b\n\x12\x66orwardLeftIndices\x18\xe8\r \x03(\r\x12)\n forwardRightIndicesActualSize_nu\x18\xb3\x0b \x01(\r\x12\x1c\n\x13\x66orwardRightIndices\x18\xd8\x18 \x03(\r\x12)\n backwardLeftIndicesActualSize_nu\x18\xc9\x02 \x01(\r\x12\x1c\n\x13\x62\x61\x63kwardLeftIndices\x18\xdf\x1e \x03(\r\x12*\n!backwardRightIndicesActualSize_nu\x18\xf1\x16 \x01(\r\x12\x1d\n\x14\x62\x61\x63kwardRightIndices\x18\xaa\x1b \x03(\r\x12,\n#bodyIndexLocationFrontActualSize_nu\x18\xd1\x1d \x01(\r\x12\x1f\n\x16\x62odyIndexLocationFront\x18\x87\x0b \x03(\r\x12+\n\"bodyIndexLocationBackActualSize_nu\x18\xf5\x10 \x01(\r\x12\x1e\n\x15\x62odyIndexLocationBack\x18\x93\x01 \x03(\r\x12+\n\"bodyIndexLocationLeftActualSize_nu\x18\xaf\n \x01(\r\x12\x1e\n\x15\x62odyIndexLocationLeft\x18\xa2\x04 \x03(\r\x12,\n#bodyIndexLocationRightActualSize_nu\x18\xd0\x06 \x01(\r\x12\x1f\n\x16\x62odyIndexLocationRight\x18\xe4\x1c \x03(\r\x12\x30\n\'bodyIndexLocationFrontLeftActualSize_nu\x18\xca\x14 \x01(\r\x12#\n\x1a\x62odyIndexLocationFrontLeft\x18\xcf\x11 \x03(\r\x12\x31\n(bodyIndexLocationFrontRightActualSize_nu\x18\xa0\x11 \x01(\r\x12$\n\x1b\x62odyIndexLocationFrontRight\x18\xb1\x1a \x03(\r\x12/\n&bodyIndexLocationBackLeftActualSize_nu\x18\xde\x1a \x01(\r\x12\"\n\x19\x62odyIndexLocationBackLeft\x18\xc0\x0f \x03(\r\x12\x30\n\'bodyIndexLocationBackRightActualSize_nu\x18\x9c\x17 \x01(\r\x12#\n\x1a\x62odyIndexLocationBackRight\x18\xde\x07 \x03(\r\x12\x15\n\rLscaActive_nu\x18) \x01(\x08\x12\x17\n\x0eonlyBackupMode\x18\x99\x17 \x01(\x08\x12\x1a\n\x11proposalActive_nu\x18\xb3\x16 \x01(\x08\x12\x1c\n\x13resistanceActive_nu\x18\xb4\x03 \x01(\x08\x12\x1d\n\x14\x62rakeStaticActive_nu\x18\x9e\t \x01(\x08\x12\x1e\n\x15\x62rakeDynamicActive_nu\x18\xa4\x04 \x01(\x08\x12$\n\x1b\x64oorProtectionStatActive_nu\x18\x8f\x08 \x01(\x08\x12#\n\x1a\x64oorProtectionDynActive_nu\x18\xc9\x1b \x01(\x08\x12\"\n\x19reverseAssistDynActive_nu\x18\xff\x12 \x01(\x08\x12%\n\x1cpedalMisapplicationActive_nu\x18\xa8\x0b \x01(\x08\"X\n\x1a\x63onfigGeneral_t_array_port\x12:\n\x04\x64\x61ta\x18& \x03(\x0b\x32,.pb.mf_lsca.config_general_t.configGeneral_t'
  ,
  dependencies=[cml_dot_vec2__df__pod__pb2.DESCRIPTOR,])




_CONFIGGENERAL_T = _descriptor.Descriptor(
  name='configGeneral_t',
  full_name='pb.mf_lsca.config_general_t.configGeneral_t',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='simpleShapeActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.simpleShapeActualSize_nu', index=0,
      number=2053, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='simpleShape', full_name='pb.mf_lsca.config_general_t.configGeneral_t.simpleShape', index=1,
      number=1549, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='forwardLeftIndicesActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.forwardLeftIndicesActualSize_nu', index=2,
      number=2903, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='forwardLeftIndices', full_name='pb.mf_lsca.config_general_t.configGeneral_t.forwardLeftIndices', index=3,
      number=1768, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='forwardRightIndicesActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.forwardRightIndicesActualSize_nu', index=4,
      number=1459, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='forwardRightIndices', full_name='pb.mf_lsca.config_general_t.configGeneral_t.forwardRightIndices', index=5,
      number=3160, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='backwardLeftIndicesActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.backwardLeftIndicesActualSize_nu', index=6,
      number=329, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='backwardLeftIndices', full_name='pb.mf_lsca.config_general_t.configGeneral_t.backwardLeftIndices', index=7,
      number=3935, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='backwardRightIndicesActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.backwardRightIndicesActualSize_nu', index=8,
      number=2929, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='backwardRightIndices', full_name='pb.mf_lsca.config_general_t.configGeneral_t.backwardRightIndices', index=9,
      number=3498, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationFrontActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationFrontActualSize_nu', index=10,
      number=3793, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationFront', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationFront', index=11,
      number=1415, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationBackActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationBackActualSize_nu', index=12,
      number=2165, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationBack', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationBack', index=13,
      number=147, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationLeftActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationLeftActualSize_nu', index=14,
      number=1327, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationLeft', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationLeft', index=15,
      number=546, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationRightActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationRightActualSize_nu', index=16,
      number=848, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationRight', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationRight', index=17,
      number=3684, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationFrontLeftActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationFrontLeftActualSize_nu', index=18,
      number=2634, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationFrontLeft', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationFrontLeft', index=19,
      number=2255, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationFrontRightActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationFrontRightActualSize_nu', index=20,
      number=2208, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationFrontRight', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationFrontRight', index=21,
      number=3377, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationBackLeftActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationBackLeftActualSize_nu', index=22,
      number=3422, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationBackLeft', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationBackLeft', index=23,
      number=1984, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationBackRightActualSize_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationBackRightActualSize_nu', index=24,
      number=2972, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bodyIndexLocationBackRight', full_name='pb.mf_lsca.config_general_t.configGeneral_t.bodyIndexLocationBackRight', index=25,
      number=990, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='LscaActive_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.LscaActive_nu', index=26,
      number=41, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='onlyBackupMode', full_name='pb.mf_lsca.config_general_t.configGeneral_t.onlyBackupMode', index=27,
      number=2969, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='proposalActive_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.proposalActive_nu', index=28,
      number=2867, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='resistanceActive_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.resistanceActive_nu', index=29,
      number=436, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='brakeStaticActive_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.brakeStaticActive_nu', index=30,
      number=1182, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='brakeDynamicActive_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.brakeDynamicActive_nu', index=31,
      number=548, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='doorProtectionStatActive_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.doorProtectionStatActive_nu', index=32,
      number=1039, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='doorProtectionDynActive_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.doorProtectionDynActive_nu', index=33,
      number=3529, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='reverseAssistDynActive_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.reverseAssistDynActive_nu', index=34,
      number=2431, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pedalMisapplicationActive_nu', full_name='pb.mf_lsca.config_general_t.configGeneral_t.pedalMisapplicationActive_nu', index=35,
      number=1448, type=8, cpp_type=7, label=1,
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
  serialized_start=87,
  serialized_end=1464,
)


_CONFIGGENERAL_T_ARRAY_PORT = _descriptor.Descriptor(
  name='configGeneral_t_array_port',
  full_name='pb.mf_lsca.config_general_t.configGeneral_t_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_lsca.config_general_t.configGeneral_t_array_port.data', index=0,
      number=38, type=11, cpp_type=10, label=3,
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
  serialized_start=1466,
  serialized_end=1554,
)

_CONFIGGENERAL_T.fields_by_name['simpleShape'].message_type = cml_dot_vec2__df__pod__pb2._VEC2DF_POD
_CONFIGGENERAL_T_ARRAY_PORT.fields_by_name['data'].message_type = _CONFIGGENERAL_T
DESCRIPTOR.message_types_by_name['configGeneral_t'] = _CONFIGGENERAL_T
DESCRIPTOR.message_types_by_name['configGeneral_t_array_port'] = _CONFIGGENERAL_T_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

configGeneral_t = _reflection.GeneratedProtocolMessageType('configGeneral_t', (_message.Message,), {
  'DESCRIPTOR' : _CONFIGGENERAL_T,
  '__module__' : 'mf_lsca.config_general_t_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.config_general_t.configGeneral_t)
  })
_sym_db.RegisterMessage(configGeneral_t)

configGeneral_t_array_port = _reflection.GeneratedProtocolMessageType('configGeneral_t_array_port', (_message.Message,), {
  'DESCRIPTOR' : _CONFIGGENERAL_T_ARRAY_PORT,
  '__module__' : 'mf_lsca.config_general_t_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_lsca.config_general_t.configGeneral_t_array_port)
  })
_sym_db.RegisterMessage(configGeneral_t_array_port)


# @@protoc_insertion_point(module_scope)
