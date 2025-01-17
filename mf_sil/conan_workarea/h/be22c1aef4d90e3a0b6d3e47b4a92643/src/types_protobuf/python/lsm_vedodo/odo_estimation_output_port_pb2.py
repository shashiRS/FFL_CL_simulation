# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: lsm_vedodo/odo_estimation_output_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from lsm_vedodo import odo_estimation_pb2 as lsm__vedodo_dot_odo__estimation__pb2
from lsm_vedodo import odo_estimation_prediction_pb2 as lsm__vedodo_dot_odo__estimation__prediction__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='lsm_vedodo/odo_estimation_output_port.proto',
  package='pb.lsm_vedodo.odo_estimation_output_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n+lsm_vedodo/odo_estimation_output_port.proto\x12(pb.lsm_vedodo.odo_estimation_output_port\x1a\x17\x65\x63o/signal_header.proto\x1a\x1flsm_vedodo/odo_estimation.proto\x1a*lsm_vedodo/odo_estimation_prediction.proto\"\xdc\x02\n\x17OdoEstimationOutputPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12I\n\x13odoEstimationBuffer\x18\xb5\x07 \x03(\x0b\x32+.pb.lsm_vedodo.odo_estimation.OdoEstimation\x12^\n\x13odoPredictionBuffer\x18\x80\x0b \x03(\x0b\x32@.pb.lsm_vedodo.odo_estimation_prediction.OdoEstimationPrediction\x12\x43\n\rodoEstimation\x18\xef\x02 \x01(\x0b\x32+.pb.lsm_vedodo.odo_estimation.OdoEstimation\"v\n\"OdoEstimationOutputPort_array_port\x12P\n\x04\x64\x61ta\x18\x9c\x05 \x03(\x0b\x32\x41.pb.lsm_vedodo.odo_estimation_output_port.OdoEstimationOutputPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,lsm__vedodo_dot_odo__estimation__pb2.DESCRIPTOR,lsm__vedodo_dot_odo__estimation__prediction__pb2.DESCRIPTOR,])




_ODOESTIMATIONOUTPUTPORT = _descriptor.Descriptor(
  name='OdoEstimationOutputPort',
  full_name='pb.lsm_vedodo.odo_estimation_output_port.OdoEstimationOutputPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.lsm_vedodo.odo_estimation_output_port.OdoEstimationOutputPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.lsm_vedodo.odo_estimation_output_port.OdoEstimationOutputPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='odoEstimationBuffer', full_name='pb.lsm_vedodo.odo_estimation_output_port.OdoEstimationOutputPort.odoEstimationBuffer', index=2,
      number=949, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='odoPredictionBuffer', full_name='pb.lsm_vedodo.odo_estimation_output_port.OdoEstimationOutputPort.odoPredictionBuffer', index=3,
      number=1408, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='odoEstimation', full_name='pb.lsm_vedodo.odo_estimation_output_port.OdoEstimationOutputPort.odoEstimation', index=4,
      number=367, type=11, cpp_type=10, label=1,
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
  serialized_start=192,
  serialized_end=540,
)


_ODOESTIMATIONOUTPUTPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='OdoEstimationOutputPort_array_port',
  full_name='pb.lsm_vedodo.odo_estimation_output_port.OdoEstimationOutputPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.lsm_vedodo.odo_estimation_output_port.OdoEstimationOutputPort_array_port.data', index=0,
      number=668, type=11, cpp_type=10, label=3,
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
  serialized_start=542,
  serialized_end=660,
)

_ODOESTIMATIONOUTPUTPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_ODOESTIMATIONOUTPUTPORT.fields_by_name['odoEstimationBuffer'].message_type = lsm__vedodo_dot_odo__estimation__pb2._ODOESTIMATION
_ODOESTIMATIONOUTPUTPORT.fields_by_name['odoPredictionBuffer'].message_type = lsm__vedodo_dot_odo__estimation__prediction__pb2._ODOESTIMATIONPREDICTION
_ODOESTIMATIONOUTPUTPORT.fields_by_name['odoEstimation'].message_type = lsm__vedodo_dot_odo__estimation__pb2._ODOESTIMATION
_ODOESTIMATIONOUTPUTPORT_ARRAY_PORT.fields_by_name['data'].message_type = _ODOESTIMATIONOUTPUTPORT
DESCRIPTOR.message_types_by_name['OdoEstimationOutputPort'] = _ODOESTIMATIONOUTPUTPORT
DESCRIPTOR.message_types_by_name['OdoEstimationOutputPort_array_port'] = _ODOESTIMATIONOUTPUTPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

OdoEstimationOutputPort = _reflection.GeneratedProtocolMessageType('OdoEstimationOutputPort', (_message.Message,), {
  'DESCRIPTOR' : _ODOESTIMATIONOUTPUTPORT,
  '__module__' : 'lsm_vedodo.odo_estimation_output_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.lsm_vedodo.odo_estimation_output_port.OdoEstimationOutputPort)
  })
_sym_db.RegisterMessage(OdoEstimationOutputPort)

OdoEstimationOutputPort_array_port = _reflection.GeneratedProtocolMessageType('OdoEstimationOutputPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _ODOESTIMATIONOUTPUTPORT_ARRAY_PORT,
  '__module__' : 'lsm_vedodo.odo_estimation_output_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.lsm_vedodo.odo_estimation_output_port.OdoEstimationOutputPort_array_port)
  })
_sym_db.RegisterMessage(OdoEstimationOutputPort_array_port)


# @@protoc_insertion_point(module_scope)
