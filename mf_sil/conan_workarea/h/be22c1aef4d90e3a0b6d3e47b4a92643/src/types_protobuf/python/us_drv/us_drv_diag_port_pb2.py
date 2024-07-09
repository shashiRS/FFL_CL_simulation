# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: us_drv/us_drv_diag_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from us_drv import us_drv_variant_data_pb2 as us__drv_dot_us__drv__variant__data__pb2
from us_drv import us_drv_sw_errors_pb2 as us__drv_dot_us__drv__sw__errors__pb2
from us_drv import us_drv_asic_errors_pb2 as us__drv_dot_us__drv__asic__errors__pb2
from us_drv import us_drv_sensor_errors_pb2 as us__drv_dot_us__drv__sensor__errors__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='us_drv/us_drv_diag_port.proto',
  package='pb.us_drv.us_drv_diag_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1dus_drv/us_drv_diag_port.proto\x12\x1apb.us_drv.us_drv_diag_port\x1a\x17\x65\x63o/signal_header.proto\x1a us_drv/us_drv_variant_data.proto\x1a\x1dus_drv/us_drv_sw_errors.proto\x1a\x1fus_drv/us_drv_asic_errors.proto\x1a!us_drv/us_drv_sensor_errors.proto\"\x85\x03\n\rUsDrvDiagPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12M\n\x13usDriverVariantData\x18\xb1\x17 \x01(\x0b\x32/.pb.us_drv.us_drv_variant_data.UsDrvVariantData\x12\x44\n\x10usDriverSwErrors\x18\x85\x1c \x01(\x0b\x32).pb.us_drv.us_drv_sw_errors.UsDrvSwErrors\x12\x42\n\nasicErrors\x18\xdb\x1c \x03(\x0b\x32-.pb.us_drv.us_drv_asic_errors.UsDrvAsicErrors\x12H\n\x0csensorErrors\x18\x9b\x01 \x03(\x0b\x32\x31.pb.us_drv.us_drv_sensor_errors.UsDrvSensorErrors\"T\n\x18UsDrvDiagPort_array_port\x12\x38\n\x04\x64\x61ta\x18\xda\x12 \x03(\x0b\x32).pb.us_drv.us_drv_diag_port.UsDrvDiagPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,us__drv_dot_us__drv__variant__data__pb2.DESCRIPTOR,us__drv_dot_us__drv__sw__errors__pb2.DESCRIPTOR,us__drv_dot_us__drv__asic__errors__pb2.DESCRIPTOR,us__drv_dot_us__drv__sensor__errors__pb2.DESCRIPTOR,])




_USDRVDIAGPORT = _descriptor.Descriptor(
  name='UsDrvDiagPort',
  full_name='pb.us_drv.us_drv_diag_port.UsDrvDiagPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.us_drv.us_drv_diag_port.UsDrvDiagPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.us_drv.us_drv_diag_port.UsDrvDiagPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='usDriverVariantData', full_name='pb.us_drv.us_drv_diag_port.UsDrvDiagPort.usDriverVariantData', index=2,
      number=2993, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='usDriverSwErrors', full_name='pb.us_drv.us_drv_diag_port.UsDrvDiagPort.usDriverSwErrors', index=3,
      number=3589, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='asicErrors', full_name='pb.us_drv.us_drv_diag_port.UsDrvDiagPort.asicErrors', index=4,
      number=3675, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sensorErrors', full_name='pb.us_drv.us_drv_diag_port.UsDrvDiagPort.sensorErrors', index=5,
      number=155, type=11, cpp_type=10, label=3,
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
  serialized_start=220,
  serialized_end=609,
)


_USDRVDIAGPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='UsDrvDiagPort_array_port',
  full_name='pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port.data', index=0,
      number=2394, type=11, cpp_type=10, label=3,
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
  serialized_start=611,
  serialized_end=695,
)

_USDRVDIAGPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_USDRVDIAGPORT.fields_by_name['usDriverVariantData'].message_type = us__drv_dot_us__drv__variant__data__pb2._USDRVVARIANTDATA
_USDRVDIAGPORT.fields_by_name['usDriverSwErrors'].message_type = us__drv_dot_us__drv__sw__errors__pb2._USDRVSWERRORS
_USDRVDIAGPORT.fields_by_name['asicErrors'].message_type = us__drv_dot_us__drv__asic__errors__pb2._USDRVASICERRORS
_USDRVDIAGPORT.fields_by_name['sensorErrors'].message_type = us__drv_dot_us__drv__sensor__errors__pb2._USDRVSENSORERRORS
_USDRVDIAGPORT_ARRAY_PORT.fields_by_name['data'].message_type = _USDRVDIAGPORT
DESCRIPTOR.message_types_by_name['UsDrvDiagPort'] = _USDRVDIAGPORT
DESCRIPTOR.message_types_by_name['UsDrvDiagPort_array_port'] = _USDRVDIAGPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

UsDrvDiagPort = _reflection.GeneratedProtocolMessageType('UsDrvDiagPort', (_message.Message,), {
  'DESCRIPTOR' : _USDRVDIAGPORT,
  '__module__' : 'us_drv.us_drv_diag_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_diag_port.UsDrvDiagPort)
  })
_sym_db.RegisterMessage(UsDrvDiagPort)

UsDrvDiagPort_array_port = _reflection.GeneratedProtocolMessageType('UsDrvDiagPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _USDRVDIAGPORT_ARRAY_PORT,
  '__module__' : 'us_drv.us_drv_diag_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_diag_port.UsDrvDiagPort_array_port)
  })
_sym_db.RegisterMessage(UsDrvDiagPort_array_port)


# @@protoc_insertion_point(module_scope)