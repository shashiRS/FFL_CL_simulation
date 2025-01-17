# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: vc/screen_switch_data_port.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2
from ap_hmitoap import screen_types_pb2 as ap__hmitoap_dot_screen__types__pb2
from vc import blind_spot_view_status_pb2 as vc_dot_blind__spot__view__status__pb2
from vc import transparency_preset_pb2 as vc_dot_transparency__preset__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='vc/screen_switch_data_port.proto',
  package='pb.vc.screen_switch_data_port',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n vc/screen_switch_data_port.proto\x12\x1dpb.vc.screen_switch_data_port\x1a\x17\x65\x63o/signal_header.proto\x1a\x1d\x61p_hmitoap/screen_types.proto\x1a\x1fvc/blind_spot_view_status.proto\x1a\x1cvc/transparency_preset.proto\"\xd3\x03\n\x14ScreenSwitchDataPort\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12K\n\x19HmiOutUserActScreenReq_u8\x18\xf2\x04 \x01(\x0e\x32\'.pb.ap_hmitoap.screen_types.ScreenTypes\x12M\n\x11\x62lindSpotViewType\x18\xf3\x06 \x01(\x0e\x32\x31.pb.vc.blind_spot_view_status.BlindSpotViewStatus\x12\x41\n\x0f\x63urrentViewMode\x18\xf5\x15 \x01(\x0e\x32\'.pb.ap_hmitoap.screen_types.ScreenTypes\x12$\n\x1b\x43lusterScreenResponse_nu_u8\x18\xd4\x18 \x01(\r\x12\x17\n\x0e\x64\x65\x61\x63tivateView\x18\xcf\x1d \x01(\x08\x12J\n\x12transparencyPreset\x18\xee\x02 \x01(\x0e\x32-.pb.vc.transparency_preset.TransparencyPreset\"e\n\x1fScreenSwitchDataPort_array_port\x12\x42\n\x04\x64\x61ta\x18\xf7\x1e \x03(\x0b\x32\x33.pb.vc.screen_switch_data_port.ScreenSwitchDataPort'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,ap__hmitoap_dot_screen__types__pb2.DESCRIPTOR,vc_dot_blind__spot__view__status__pb2.DESCRIPTOR,vc_dot_transparency__preset__pb2.DESCRIPTOR,])




_SCREENSWITCHDATAPORT = _descriptor.Descriptor(
  name='ScreenSwitchDataPort',
  full_name='pb.vc.screen_switch_data_port.ScreenSwitchDataPort',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.vc.screen_switch_data_port.ScreenSwitchDataPort.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.vc.screen_switch_data_port.ScreenSwitchDataPort.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='HmiOutUserActScreenReq_u8', full_name='pb.vc.screen_switch_data_port.ScreenSwitchDataPort.HmiOutUserActScreenReq_u8', index=2,
      number=626, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='blindSpotViewType', full_name='pb.vc.screen_switch_data_port.ScreenSwitchDataPort.blindSpotViewType', index=3,
      number=883, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='currentViewMode', full_name='pb.vc.screen_switch_data_port.ScreenSwitchDataPort.currentViewMode', index=4,
      number=2805, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ClusterScreenResponse_nu_u8', full_name='pb.vc.screen_switch_data_port.ScreenSwitchDataPort.ClusterScreenResponse_nu_u8', index=5,
      number=3156, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='deactivateView', full_name='pb.vc.screen_switch_data_port.ScreenSwitchDataPort.deactivateView', index=6,
      number=3791, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='transparencyPreset', full_name='pb.vc.screen_switch_data_port.ScreenSwitchDataPort.transparencyPreset', index=7,
      number=366, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
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
  serialized_start=187,
  serialized_end=654,
)


_SCREENSWITCHDATAPORT_ARRAY_PORT = _descriptor.Descriptor(
  name='ScreenSwitchDataPort_array_port',
  full_name='pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port.data', index=0,
      number=3959, type=11, cpp_type=10, label=3,
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
  serialized_start=656,
  serialized_end=757,
)

_SCREENSWITCHDATAPORT.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_SCREENSWITCHDATAPORT.fields_by_name['HmiOutUserActScreenReq_u8'].enum_type = ap__hmitoap_dot_screen__types__pb2._SCREENTYPES
_SCREENSWITCHDATAPORT.fields_by_name['blindSpotViewType'].enum_type = vc_dot_blind__spot__view__status__pb2._BLINDSPOTVIEWSTATUS
_SCREENSWITCHDATAPORT.fields_by_name['currentViewMode'].enum_type = ap__hmitoap_dot_screen__types__pb2._SCREENTYPES
_SCREENSWITCHDATAPORT.fields_by_name['transparencyPreset'].enum_type = vc_dot_transparency__preset__pb2._TRANSPARENCYPRESET
_SCREENSWITCHDATAPORT_ARRAY_PORT.fields_by_name['data'].message_type = _SCREENSWITCHDATAPORT
DESCRIPTOR.message_types_by_name['ScreenSwitchDataPort'] = _SCREENSWITCHDATAPORT
DESCRIPTOR.message_types_by_name['ScreenSwitchDataPort_array_port'] = _SCREENSWITCHDATAPORT_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ScreenSwitchDataPort = _reflection.GeneratedProtocolMessageType('ScreenSwitchDataPort', (_message.Message,), {
  'DESCRIPTOR' : _SCREENSWITCHDATAPORT,
  '__module__' : 'vc.screen_switch_data_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
  })
_sym_db.RegisterMessage(ScreenSwitchDataPort)

ScreenSwitchDataPort_array_port = _reflection.GeneratedProtocolMessageType('ScreenSwitchDataPort_array_port', (_message.Message,), {
  'DESCRIPTOR' : _SCREENSWITCHDATAPORT_ARRAY_PORT,
  '__module__' : 'vc.screen_switch_data_port_pb2'
  # @@protoc_insertion_point(class_scope:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
  })
_sym_db.RegisterMessage(ScreenSwitchDataPort_array_port)


# @@protoc_insertion_point(module_scope)
