# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_trjctl/mfcontrol_status_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_trjctl import lodmc_handshake_failed_status_pb2 as ap__trjctl_dot_lodmc__handshake__failed__status__pb2
from ap_trjctl import ladmc_handshake_failed_status_pb2 as ap__trjctl_dot_ladmc__handshake__failed__status__pb2
from ap_trjctl import lateral_control_saturation_status_pb2 as ap__trjctl_dot_lateral__control__saturation__status__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_trjctl/mfcontrol_status_port.proto',
  package='pb.ap_trjctl.mfcontrol_status_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n%ap_trjctl/mfcontrol_status_port.proto\x12\"pb.ap_trjctl.mfcontrol_status_port\x1a\x17\x65\x63o/signal_header.proto\x1a-ap_trjctl/lodmc_handshake_failed_status.proto\x1a-ap_trjctl/ladmc_handshake_failed_status.proto\x1a\x31\x61p_trjctl/lateral_control_saturation_status.proto\"\xfc\x06\n\x13MFControlStatusPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x1d\n\x14numUsedCtrlPoints_nu\x18\x97\x11 \x01(\r\x12n\n\x1dlodmcHandshakeFailedStatus_nu\x18\x86\x16 \x01(\x0e\x32\x46.pb.ap_trjctl.lodmc_handshake_failed_status.LodmcHandshakeFailedStatus\x12n\n\x1dladmcHandshakeFailedStatus_nu\x18\x92\x0b \x01(\x0e\x32\x46.pb.ap_trjctl.ladmc_handshake_failed_status.LadmcHandshakeFailedStatus\x12\x1e\n\x15\x63orrectGearEngaged_nu\x18\xd2\x0c \x01(\x08\x12\'\n\x1elongitudinalControlFinished_nu\x18\xbb\x16 \x01(\x08\x12\"\n\x19lateralControlFinished_nu\x18\xcc\x18 \x01(\x08\x12%\n\x1clongitudinalControlFailed_nu\x18\xc6\x11 \x01(\x08\x12$\n\x1blateralPathControlFailed_nu\x18\xfe\x1a \x01(\x08\x12)\n longitudinalPathControlFailed_nu\x18\xc0\x05 \x01(\x08\x12 \n\x17lateralControlFailed_nu\x18\x9e\x0c \x01(\x08\x12(\n\x1flongitudinalControlSaturated_nu\x18\x88\x03 \x01(\x08\x12z\n!lateralControlSaturationStatus_nu\x18\xb3\x0b \x01(\x0e\x32N.pb.ap_trjctl.lateral_control_saturation_status.LateralControlSaturationStatus\x12\x1d\n\x14vehStandstillHold_nu\x18\x8a\x13 \x01(\x08\x12 \n\x17vehStandstillSecured_nu\x18\xa8\x08 \x01(\x08\x12%\n\x1c\x64riverSteerIntervDetected_nu\x18\x97\x03 \x01(\x08\"h\n\x1eMFControlStatusPort_array_port\x12\x46\n\x04\x64\x61ta\x18\xc0\x18 \x03(\x0b\x32\x37.pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__trjctl_dot_lodmc__handshake__failed__status__pb2.DESCRIPTOR,ap__trjctl_dot_ladmc__handshake__failed__status__pb2.DESCRIPTOR,ap__trjctl_dot_lateral__control__saturation__status__pb2.DESCRIPTOR,])




_MFCONTROLSTATUSPORT = _descriptor.Descriptor(
  name='MFControlStatusPort',
  full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='numUsedCtrlPoints_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.numUsedCtrlPoints_nu', index=2,
      number=2199, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='lodmcHandshakeFailedStatus_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.lodmcHandshakeFailedStatus_nu', index=3,
      number=2822, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ladmcHandshakeFailedStatus_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.ladmcHandshakeFailedStatus_nu', index=4,
      number=1426, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='correctGearEngaged_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.correctGearEngaged_nu', index=5,
      number=1618, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='longitudinalControlFinished_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.longitudinalControlFinished_nu', index=6,
      number=2875, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='lateralControlFinished_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.lateralControlFinished_nu', index=7,
      number=3148, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='longitudinalControlFailed_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.longitudinalControlFailed_nu', index=8,
      number=2246, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='lateralPathControlFailed_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.lateralPathControlFailed_nu', index=9,
      number=3454, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='longitudinalPathControlFailed_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.longitudinalPathControlFailed_nu', index=10,
      number=704, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='lateralControlFailed_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.lateralControlFailed_nu', index=11,
      number=1566, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='longitudinalControlSaturated_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.longitudinalControlSaturated_nu', index=12,
      number=392, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='lateralControlSaturationStatus_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.lateralControlSaturationStatus_nu', index=13,
      number=1459, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='vehStandstillHold_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.vehStandstillHold_nu', index=14,
      number=2442, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='vehStandstillSecured_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.vehStandstillSecured_nu', index=15,
      number=1064, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='driverSteerIntervDetected_nu', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort.driverSteerIntervDetected_nu', index=16,
      number=407, type=8, cpp_type=7, label=1,
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
  serialized_start=248,
  serialized_end=1140,
)


_MFCONTROLSTATUSPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='MFControlStatusPort_array_port',
  full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort_array_port.data', index=0,
      number=3136, type=11, cpp_type=10, label=3,
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
  serialized_start=1142,
  serialized_end=1246,
)

_MFCONTROLSTATUSPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_MFCONTROLSTATUSPORT.fields_by_name['lodmcHandshakeFailedStatus_nu'].enum_type = ap__trjctl_dot_lodmc__handshake__failed__status__pb2._LODMCHANDSHAKEFAILEDSTATUS
_MFCONTROLSTATUSPORT.fields_by_name['ladmcHandshakeFailedStatus_nu'].enum_type = ap__trjctl_dot_ladmc__handshake__failed__status__pb2._LADMCHANDSHAKEFAILEDSTATUS
_MFCONTROLSTATUSPORT.fields_by_name['lateralControlSaturationStatus_nu'].enum_type = ap__trjctl_dot_lateral__control__saturation__status__pb2._LATERALCONTROLSATURATIONSTATUS
_MFCONTROLSTATUSPORT_ARRAY_PORT.fields_by_name['data'].message_type = _MFCONTROLSTATUSPORT
DESCRIPTOR.message_types_by_name['MFControlStatusPort'] = _MFCONTROLSTATUSPORT
DESCRIPTOR.message_types_by_name['MFControlStatusPort_array_port'] = _MFCONTROLSTATUSPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MFControlStatusPort = _reflection.GeneratedProtocolMessageType('MFControlStatusPort', (_message.Message,), {
  'DESCRIPTOR' : _MFCONTROLSTATUSPORT,
  '__module__' : 'ap_trjctl.mfcontrol_status_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort)
  })
_sym_db.RegisterMessage(MFControlStatusPort)

MFControlStatusPort_array_port = _reflection.GeneratedProtocolMessageType('MFControlStatusPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _MFCONTROLSTATUSPORT_ARRAY_PORT,
  '__module__' : 'ap_trjctl.mfcontrol_status_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.ap_trjctl.mfcontrol_status_port.MFControlStatusPort_array_port)
  })
_sym_db.RegisterMessage(MFControlStatusPort_array_port)


# @@protoc_insertion_point(module_scope)
