# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_em/us_env_model_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from us_em import dynamic_object_serializable_pb2 as us__em_dot_dynamic__object__serializable__pb2
from us_em import static_object_serializable_pb2 as us__em_dot_static__object__serializable__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_em/us_env_model_port.proto',
  package='pb.us_em.us_env_model_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1dus_em/us_env_model_port.proto\x12\x1apb.us_em.us_env_model_port\x1a\x17\x65\x63o/signal_header.proto\x1a\'us_em/dynamic_object_serializable.proto\x1a&us_em/static_object_serializable.proto\"\xa8\x03\n\x0eUsEnvModelPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12!\n\x18numberOfStaticObjects_u8\x18\xba\x01 \x01(\r\x12\"\n\x19numberOfDynamicObjects_u8\x18\xf1\x1d \x01(\r\x12%\n\x1c\x66irstStatObjOutDetZoneIdx_u8\x18\xa7\x11 \x01(\r\x12$\n\x1b\x66irstDynObjOutDetZoneIdx_u8\x18\xbe\r \x01(\r\x12X\n\x0e\x64ynamicObjects\x18\xe2\x03 \x03(\x0b\x32?.pb.us_em.dynamic_object_serializable.DynamicObjectSerializable\x12U\n\rstaticObjects\x18\x9a\x18 \x03(\x0b\x32=.pb.us_em.static_object_serializable.StaticObjectSerializable\"V\n\x19UsEnvModelPort_array_port\x12\x39\n\x04\x64\x61ta\x18\x95\x1f \x03(\x0b\x32*.pb.us_em.us_env_model_port.UsEnvModelPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,us__em_dot_dynamic__object__serializable__pb2.DESCRIPTOR,us__em_dot_static__object__serializable__pb2.DESCRIPTOR,])




_USENVMODELPORT = _descriptor.Descriptor(
  name='UsEnvModelPort',
  full_name='pb.us_em.us_env_model_port.UsEnvModelPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.us_em.us_env_model_port.UsEnvModelPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.us_em.us_env_model_port.UsEnvModelPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numberOfStaticObjects_u8', full_name='pb.us_em.us_env_model_port.UsEnvModelPort.numberOfStaticObjects_u8', index=2,
      number=186, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numberOfDynamicObjects_u8', full_name='pb.us_em.us_env_model_port.UsEnvModelPort.numberOfDynamicObjects_u8', index=3,
      number=3825, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='firstStatObjOutDetZoneIdx_u8', full_name='pb.us_em.us_env_model_port.UsEnvModelPort.firstStatObjOutDetZoneIdx_u8', index=4,
      number=2215, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='firstDynObjOutDetZoneIdx_u8', full_name='pb.us_em.us_env_model_port.UsEnvModelPort.firstDynObjOutDetZoneIdx_u8', index=5,
      number=1726, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='dynamicObjects', full_name='pb.us_em.us_env_model_port.UsEnvModelPort.dynamicObjects', index=6,
      number=482, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='staticObjects', full_name='pb.us_em.us_env_model_port.UsEnvModelPort.staticObjects', index=7,
      number=3098, type=11, cpp_type=10, label=3,
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
  serialized_start=168,
  serialized_end=592,
)


_USENVMODELPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='UsEnvModelPort_array_port',
  full_name='pb.us_em.us_env_model_port.UsEnvModelPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_em.us_env_model_port.UsEnvModelPort_array_port.data', index=0,
      number=3989, type=11, cpp_type=10, label=3,
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
  serialized_start=594,
  serialized_end=680,
)

_USENVMODELPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_USENVMODELPORT.fields_by_name['dynamicObjects'].message_type = us__em_dot_dynamic__object__serializable__pb2._DYNAMICOBJECTSERIALIZABLE
_USENVMODELPORT.fields_by_name['staticObjects'].message_type = us__em_dot_static__object__serializable__pb2._STATICOBJECTSERIALIZABLE
_USENVMODELPORT_ARRAY_PORT.fields_by_name['data'].message_type = _USENVMODELPORT
DESCRIPTOR.message_types_by_name['UsEnvModelPort'] = _USENVMODELPORT
DESCRIPTOR.message_types_by_name['UsEnvModelPort_array_port'] = _USENVMODELPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsEnvModelPort = _reflection.GeneratedProtocolMessageType('UsEnvModelPort', (_message.Message,), {
  'DESCRIPTOR' : _USENVMODELPORT,
  '__module__' : 'us_em.us_env_model_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_em.us_env_model_port.UsEnvModelPort)
  })
_sym_db.RegisterMessage(UsEnvModelPort)

UsEnvModelPort_array_port = _reflection.GeneratedProtocolMessageType('UsEnvModelPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USENVMODELPORT_ARRAY_PORT,
  '__module__' : 'us_em.us_env_model_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_em.us_env_model_port.UsEnvModelPort_array_port)
  })
_sym_db.RegisterMessage(UsEnvModelPort_array_port)


# @@protoc_insertion_point(module_scope)
