# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_processing/us_processing_params.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from us_processing import us_processing_state_management_cfg_pb2 as us__processing_dot_us__processing__state__management__cfg__pb2
from us_processing import us_processing_echo_pre_processing_cfg_pb2 as us__processing_dot_us__processing__echo__pre__processing__cfg__pb2
from us_processing import us_processing_reflection_tracking_cfg_pb2 as us__processing_dot_us__processing__reflection__tracking__cfg__pb2
from us_processing import us_processing_object_tracking_cfg_pb2 as us__processing_dot_us__processing__object__tracking__cfg__pb2
from us_processing import us_processing_sensor_parameters_pb2 as us__processing_dot_us__processing__sensor__parameters__pb2
from us_processing import us_processing_dyn_params_pb2 as us__processing_dot_us__processing__dyn__params__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_processing/us_processing_params.proto',
  package='pb.us_processing.us_processing_params',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n(us_processing/us_processing_params.proto\x12%pb.us_processing.us_processing_params\x1a\x17\x65\x63o/signal_header.proto\x1a\x36us_processing/us_processing_state_management_cfg.proto\x1a\x39us_processing/us_processing_echo_pre_processing_cfg.proto\x1a\x39us_processing/us_processing_reflection_tracking_cfg.proto\x1a\x35us_processing/us_processing_object_tracking_cfg.proto\x1a\x33us_processing/us_processing_sensor_parameters.proto\x1a,us_processing/us_processing_dyn_params.proto\"\xd3\x05\n\x12UsProcessingParams\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12h\n\nustmParams\x18\x82\x18 \x01(\x0b\x32S.pb.us_processing.us_processing_state_management_cfg.UsProcessingStateManagementCfg\x12m\n\nusepParams\x18\xc7\x0f \x01(\x0b\x32X.pb.us_processing.us_processing_echo_pre_processing_cfg.UsProcessingEchoPreProcessingCfg\x12n\n\nrftrParams\x18\x8c\x12 \x01(\x0b\x32Y.pb.us_processing.us_processing_reflection_tracking_cfg.UsProcessingReflectionTrackingCfg\x12\x66\n\nobjtParams\x18\xb6\x10 \x01(\x0b\x32Q.pb.us_processing.us_processing_object_tracking_cfg.UsProcessingObjectTrackingCfg\x12\x63\n\nsensParams\x18\xca\x13 \x01(\x0b\x32N.pb.us_processing.us_processing_sensor_parameters.UsProcessingSensorParameters\x12T\n\tdynParams\x18\xe4\x1f \x01(\x0b\x32@.pb.us_processing.us_processing_dyn_params.UsProcessingDynParams\"i\n\x1dUsProcessingParams_array_port\x12H\n\x04\x64\x61ta\x18\xb4\x1a \x03(\x0b\x32\x39.pb.us_processing.us_processing_params.UsProcessingParams'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,us__processing_dot_us__processing__state__management__cfg__pb2.DESCRIPTOR,us__processing_dot_us__processing__echo__pre__processing__cfg__pb2.DESCRIPTOR,us__processing_dot_us__processing__reflection__tracking__cfg__pb2.DESCRIPTOR,us__processing_dot_us__processing__object__tracking__cfg__pb2.DESCRIPTOR,us__processing_dot_us__processing__sensor__parameters__pb2.DESCRIPTOR,us__processing_dot_us__processing__dyn__params__pb2.DESCRIPTOR,])




_USPROCESSINGPARAMS = _descriptor.Descriptor(
  name='UsProcessingParams',
  full_name='pb.us_processing.us_processing_params.UsProcessingParams',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.us_processing.us_processing_params.UsProcessingParams.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.us_processing.us_processing_params.UsProcessingParams.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ustmParams', full_name='pb.us_processing.us_processing_params.UsProcessingParams.ustmParams', index=2,
      number=3074, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='usepParams', full_name='pb.us_processing.us_processing_params.UsProcessingParams.usepParams', index=3,
      number=1991, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rftrParams', full_name='pb.us_processing.us_processing_params.UsProcessingParams.rftrParams', index=4,
      number=2316, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='objtParams', full_name='pb.us_processing.us_processing_params.UsProcessingParams.objtParams', index=5,
      number=2102, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sensParams', full_name='pb.us_processing.us_processing_params.UsProcessingParams.sensParams', index=6,
      number=2506, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='dynParams', full_name='pb.us_processing.us_processing_params.UsProcessingParams.dynParams', index=7,
      number=4068, type=11, cpp_type=10, label=1,
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
  serialized_start=437,
  serialized_end=1160,
)


_USPROCESSINGPARAMS_ARRAY_PORT = _descriptor.Descriptor(
  name='UsProcessingParams_array_port',
  full_name='pb.us_processing.us_processing_params.UsProcessingParams_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_processing.us_processing_params.UsProcessingParams_array_port.data', index=0,
      number=3380, type=11, cpp_type=10, label=3,
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
  serialized_start=1162,
  serialized_end=1267,
)

_USPROCESSINGPARAMS.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_USPROCESSINGPARAMS.fields_by_name['ustmParams'].message_type = us__processing_dot_us__processing__state__management__cfg__pb2._USPROCESSINGSTATEMANAGEMENTCFG
_USPROCESSINGPARAMS.fields_by_name['usepParams'].message_type = us__processing_dot_us__processing__echo__pre__processing__cfg__pb2._USPROCESSINGECHOPREPROCESSINGCFG
_USPROCESSINGPARAMS.fields_by_name['rftrParams'].message_type = us__processing_dot_us__processing__reflection__tracking__cfg__pb2._USPROCESSINGREFLECTIONTRACKINGCFG
_USPROCESSINGPARAMS.fields_by_name['objtParams'].message_type = us__processing_dot_us__processing__object__tracking__cfg__pb2._USPROCESSINGOBJECTTRACKINGCFG
_USPROCESSINGPARAMS.fields_by_name['sensParams'].message_type = us__processing_dot_us__processing__sensor__parameters__pb2._USPROCESSINGSENSORPARAMETERS
_USPROCESSINGPARAMS.fields_by_name['dynParams'].message_type = us__processing_dot_us__processing__dyn__params__pb2._USPROCESSINGDYNPARAMS
_USPROCESSINGPARAMS_ARRAY_PORT.fields_by_name['data'].message_type = _USPROCESSINGPARAMS
DESCRIPTOR.message_types_by_name['UsProcessingParams'] = _USPROCESSINGPARAMS
DESCRIPTOR.message_types_by_name['UsProcessingParams_array_port'] = _USPROCESSINGPARAMS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsProcessingParams = _reflection.GeneratedProtocolMessageType('UsProcessingParams', (_message.Message,), {
  'DESCRIPTOR' : _USPROCESSINGPARAMS,
  '__module__' : 'us_processing.us_processing_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_params.UsProcessingParams)
  })
_sym_db.RegisterMessage(UsProcessingParams)

UsProcessingParams_array_port = _reflection.GeneratedProtocolMessageType('UsProcessingParams_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USPROCESSINGPARAMS_ARRAY_PORT,
  '__module__' : 'us_processing.us_processing_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_params.UsProcessingParams_array_port)
  })
_sym_db.RegisterMessage(UsProcessingParams_array_port)


# @@protoc_insertion_point(module_scope)
