# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_processing/us_processing_dyn_params.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_processing/us_processing_dyn_params.proto',
  package='pb.us_processing.us_processing_dyn_params',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n,us_processing/us_processing_dyn_params.proto\x12)pb.us_processing.us_processing_dyn_params\"d\n\x15UsProcessingDynParams\x12\x18\n\x0f\x64ynParDummyChar\x18\xcf\x11 \x01(\r\x12\x16\n\x0e\x64ynParDummyInt\x18n \x01(\r\x12\x19\n\x10\x64ynParDummyFloat\x18\xc7\x12 \x01(\x02\"s\n UsProcessingDynParams_array_port\x12O\n\x04\x64\x61ta\x18\xf6\x1e \x03(\x0b\x32@.pb.us_processing.us_processing_dyn_params.UsProcessingDynParams'
)




_USPROCESSINGDYNPARAMS = _descriptor.Descriptor(
  name='UsProcessingDynParams',
  full_name='pb.us_processing.us_processing_dyn_params.UsProcessingDynParams',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='dynParDummyChar', full_name='pb.us_processing.us_processing_dyn_params.UsProcessingDynParams.dynParDummyChar', index=0,
      number=2255, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='dynParDummyInt', full_name='pb.us_processing.us_processing_dyn_params.UsProcessingDynParams.dynParDummyInt', index=1,
      number=110, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='dynParDummyFloat', full_name='pb.us_processing.us_processing_dyn_params.UsProcessingDynParams.dynParDummyFloat', index=2,
      number=2375, type=2, cpp_type=6, label=1,
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
  serialized_start=91,
  serialized_end=191,
)


_USPROCESSINGDYNPARAMS_ARRAY_PORT = _descriptor.Descriptor(
  name='UsProcessingDynParams_array_port',
  full_name='pb.us_processing.us_processing_dyn_params.UsProcessingDynParams_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_processing.us_processing_dyn_params.UsProcessingDynParams_array_port.data', index=0,
      number=3958, type=11, cpp_type=10, label=3,
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
  serialized_start=193,
  serialized_end=308,
)

_USPROCESSINGDYNPARAMS_ARRAY_PORT.fields_by_name['data'].message_type = _USPROCESSINGDYNPARAMS
DESCRIPTOR.message_types_by_name['UsProcessingDynParams'] = _USPROCESSINGDYNPARAMS
DESCRIPTOR.message_types_by_name['UsProcessingDynParams_array_port'] = _USPROCESSINGDYNPARAMS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsProcessingDynParams = _reflection.GeneratedProtocolMessageType('UsProcessingDynParams', (_message.Message,), {
  'DESCRIPTOR' : _USPROCESSINGDYNPARAMS,
  '__module__' : 'us_processing.us_processing_dyn_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_dyn_params.UsProcessingDynParams)
  })
_sym_db.RegisterMessage(UsProcessingDynParams)

UsProcessingDynParams_array_port = _reflection.GeneratedProtocolMessageType('UsProcessingDynParams_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USPROCESSINGDYNPARAMS_ARRAY_PORT,
  '__module__' : 'us_processing.us_processing_dyn_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_processing.us_processing_dyn_params.UsProcessingDynParams_array_port)
  })
_sym_db.RegisterMessage(UsProcessingDynParams_array_port)


# @@protoc_insertion_point(module_scope)
