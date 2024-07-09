# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_vehstatesigprovider/additional_bcmstatus_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_vehstatesigprovider import light_pb2 as ap__vehstatesigprovider_dot_light__pb2
from ap_vehstatesigprovider import ignition_pb2 as ap__vehstatesigprovider_dot_ignition__pb2
from ap_vehstatesigprovider import outer_rear_view_mirror_pb2 as ap__vehstatesigprovider_dot_outer__rear__view__mirror__pb2
from ap_vehstatesigprovider import sunroof_status_pb2 as ap__vehstatesigprovider_dot_sunroof__status__pb2
from ap_vehstatesigprovider import charging_status_pb2 as ap__vehstatesigprovider_dot_charging__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_vehstatesigprovider/additional_bcmstatus_port.proto',
  package='pb.ap_vehstatesigprovider.additional_bcmstatus_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n6ap_vehstatesigprovider/additional_bcmstatus_port.proto\x12\x33pb.ap_vehstatesigprovider.additional_bcmstatus_port\x1a\x17\x65\x63o/signal_header.proto\x1a\"ap_vehstatesigprovider/light.proto\x1a%ap_vehstatesigprovider/ignition.proto\x1a\x33\x61p_vehstatesigprovider/outer_rear_view_mirror.proto\x1a+ap_vehstatesigprovider/sunroof_status.proto\x1a,ap_vehstatesigprovider/charging_status.proto\"\xac\x04\n\x17\x41\x64\x64itionalBCMStatusPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x36\n\x05light\x18\xd2\x06 \x01(\x0b\x32&.pb.ap_vehstatesigprovider.light.Light\x12?\n\x08ignition\x18\xa0\x04 \x01(\x0b\x32,.pb.ap_vehstatesigprovider.ignition.Ignition\x12\x18\n\x0f\x66rontLidOpen_nu\x18\x8d\x06 \x01(\x08\x12\x17\n\x0etankCapOpen_nu\x18\xe5\x19 \x01(\x08\x12h\n\x18outerRearViewMirrorState\x18\xf7\x1c \x01(\x0b\x32\x45.pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror\x12R\n\x10sunroofStatus_nu\x18\x91\x0b \x01(\x0e\x32\x37.pb.ap_vehstatesigprovider.sunroof_status.SunroofStatus\x12T\n\x11\x63hargingStatus_nu\x18\x41 \x01(\x0b\x32\x39.pb.ap_vehstatesigprovider.charging_status.ChargingStatus\"\x81\x01\n\"AdditionalBCMStatusPort_array_port\x12[\n\x04\x64\x61ta\x18\xec\x16 \x03(\x0b\x32L.pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__vehstatesigprovider_dot_light__pb2.DESCRIPTOR,ap__vehstatesigprovider_dot_ignition__pb2.DESCRIPTOR,ap__vehstatesigprovider_dot_outer__rear__view__mirror__pb2.DESCRIPTOR,ap__vehstatesigprovider_dot_sunroof__status__pb2.DESCRIPTOR,ap__vehstatesigprovider_dot_charging__status__pb2.DESCRIPTOR,])




_ADDITIONALBCMSTATUSPORT = _descriptor.Descriptor(
  name='AdditionalBCMStatusPort',
  full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='light', full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort.light', index=2,
      number=850, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ignition', full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort.ignition', index=3,
      number=544, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frontLidOpen_nu', full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort.frontLidOpen_nu', index=4,
      number=781, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tankCapOpen_nu', full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort.tankCapOpen_nu', index=5,
      number=3301, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='outerRearViewMirrorState', full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort.outerRearViewMirrorState', index=6,
      number=3703, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sunroofStatus_nu', full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort.sunroofStatus_nu', index=7,
      number=1425, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='chargingStatus_nu', full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort.chargingStatus_nu', index=8,
      number=65, type=11, cpp_type=10, label=1,
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
  serialized_start=356,
  serialized_end=912,
)


_ADDITIONALBCMSTATUSPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='AdditionalBCMStatusPort_array_port',
  full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort_array_port.data', index=0,
      number=2924, type=11, cpp_type=10, label=3,
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
  serialized_start=915,
  serialized_end=1044,
)

_ADDITIONALBCMSTATUSPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_ADDITIONALBCMSTATUSPORT.fields_by_name['light'].message_type = ap__vehstatesigprovider_dot_light__pb2._LIGHT
_ADDITIONALBCMSTATUSPORT.fields_by_name['ignition'].message_type = ap__vehstatesigprovider_dot_ignition__pb2._IGNITION
_ADDITIONALBCMSTATUSPORT.fields_by_name['outerRearViewMirrorState'].message_type = ap__vehstatesigprovider_dot_outer__rear__view__mirror__pb2._OUTERREARVIEWMIRROR
_ADDITIONALBCMSTATUSPORT.fields_by_name['sunroofStatus_nu'].enum_type = ap__vehstatesigprovider_dot_sunroof__status__pb2._SUNROOFSTATUS
_ADDITIONALBCMSTATUSPORT.fields_by_name['chargingStatus_nu'].message_type = ap__vehstatesigprovider_dot_charging__status__pb2._CHARGINGSTATUS
_ADDITIONALBCMSTATUSPORT_ARRAY_PORT.fields_by_name['data'].message_type = _ADDITIONALBCMSTATUSPORT
DESCRIPTOR.message_types_by_name['AdditionalBCMStatusPort'] = _ADDITIONALBCMSTATUSPORT
DESCRIPTOR.message_types_by_name['AdditionalBCMStatusPort_array_port'] = _ADDITIONALBCMSTATUSPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

AdditionalBCMStatusPort = _reflection.GeneratedProtocolMessageType('AdditionalBCMStatusPort', (_message.Message,), {
  'DESCRIPTOR' : _ADDITIONALBCMSTATUSPORT,
  '__module__' : 'ap_vehstatesigprovider.additional_bcmstatus_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort)
  })
_sym_db.RegisterMessage(AdditionalBCMStatusPort)

AdditionalBCMStatusPort_array_port = _reflection.GeneratedProtocolMessageType('AdditionalBCMStatusPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _ADDITIONALBCMSTATUSPORT_ARRAY_PORT,
  '__module__' : 'ap_vehstatesigprovider.additional_bcmstatus_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.additional_bcmstatus_port.AdditionalBCMStatusPort_array_port)
  })
_sym_db.RegisterMessage(AdditionalBCMStatusPort_array_port)


# @@protoc_insertion_point(module_scope)