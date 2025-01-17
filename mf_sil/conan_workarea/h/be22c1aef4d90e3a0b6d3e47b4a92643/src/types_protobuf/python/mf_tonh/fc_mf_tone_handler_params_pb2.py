# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_tonh/fc_mf_tone_handler_params.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_tonh/fc_mf_tone_handler_params.proto',
  package='pb.mf_tonh.fc_mf_tone_handler_params',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\'mf_tonh/fc_mf_tone_handler_params.proto\x12$pb.mf_tonh.fc_mf_tone_handler_params\x1a\x17\x65\x63o/signal_header.proto\"\xa1\x08\n\x18\x46\x43_MF_ToneHandler_Params\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x1c\n\x13TH_ACK_TONE_DELAY_S\x18\xba\x05 \x01(\x02\x12\x1f\n\x16TH_ACK_TONE_DURATION_S\x18\xc7\x0c \x01(\x02\x12\x1f\n\x16TH_ERR_TONE_DURATION_S\x18\xe5\n \x01(\x02\x12\x1d\n\x14TH_SOUND_PULSE_LEN_S\x18\xf5\n \x01(\x02\x12\"\n\x19TH_LVMD_SOUND_PULSE_LEN_S\x18\xab\x11 \x01(\x02\x12\x1d\n\x14TH_CONT_TONE_LIMIT_M\x18\xdc\x14 \x01(\x02\x12 \n\x17TH_PING_PONG_DURATION_S\x18\xb5\x1e \x01(\x02\x12 \n\x17TH_MIN_PAUSE_DURATION_S\x18\xc9\x14 \x01(\x02\x12 \n\x17TH_MAX_PAUSE_DURATION_S\x18\xec\x10 \x01(\x02\x12 \n\x17TH_ACK_PAUSE_DURATION_S\x18\x8b\x1b \x01(\x02\x12$\n\x1bTH_WHP_LOW_PAUSE_DURATION_S\x18\xa4\x03 \x01(\x02\x12%\n\x1cTH_WHP_HIGH_PAUSE_DURATION_S\x18\xb7\n \x01(\x02\x12!\n\x18TH_LSCA_PAUSE_DURATION_S\x18\xc6\n \x01(\x02\x12\x1d\n\x14TH_ACTIVE_DISTANCE_M\x18\xf8\x02 \x01(\x02\x12\x1b\n\x12TH_VOLUME_FRONT_NU\x18\xea\x0c \x01(\r\x12\x1a\n\x11TH_VOLUME_REAR_NU\x18\xbf\x02 \x01(\r\x12\x1d\n\x14TH_ACK_TONE_PITCH_NU\x18\xd6\x17 \x01(\r\x12\"\n\x19TH_ACK_TONE_VOL_OFFSET_NU\x18\xf4\x14 \x01(\r\x12\"\n\x19TH_ERR_TONE_VOL_OFFSET_NU\x18\xd7\n \x01(\r\x12\x1d\n\x14TH_ERR_TONE_PITCH_NU\x18\xc3\x08 \x01(\r\x12\x1e\n\x15TH_WARN_TONE_PITCH_NU\x18\xb8\x17 \x01(\r\x12\"\n\x19TH_WHP_WARN_TONE_PITCH_NU\x18\xb9\x1d \x01(\r\x12\"\n\x19TH_LSCA_LOW_TONE_PITCH_NU\x18\x8e\x1a \x01(\r\x12#\n\x1aTH_LSCA_HIGH_TONE_PITCH_NU\x18\xb9\x1c \x01(\r\x12\x1e\n\x15TH_LVMD_TONE_PITCH_NU\x18\xf9\x08 \x01(\r\x12\x1a\n\x11TH_WEAK_VOLUME_NU\x18\xc8\x1e \x01(\r\x12\x16\n\rTH_ENABLED_NU\x18\xcf\x03 \x01(\x08\x12\x1e\n\x15TH_DIFFERENT_TONES_NU\x18\xb3\x1f \x01(\x08\x12\x19\n\x10TH_ACK_ACTIVE_NU\x18\xcb\t \x01(\x08\"t\n#FC_MF_ToneHandler_Params_array_port\x12M\n\x04\x64\x61ta\x18\xb9\x1e \x03(\x0b\x32>.pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_FC_MF_TONEHANDLER_PARAMS = _descriptor.Descriptor(
  name='FC_MF_ToneHandler_Params',
  full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_ACK_TONE_DELAY_S', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_ACK_TONE_DELAY_S', index=2,
      number=698, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_ACK_TONE_DURATION_S', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_ACK_TONE_DURATION_S', index=3,
      number=1607, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_ERR_TONE_DURATION_S', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_ERR_TONE_DURATION_S', index=4,
      number=1381, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_SOUND_PULSE_LEN_S', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_SOUND_PULSE_LEN_S', index=5,
      number=1397, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_LVMD_SOUND_PULSE_LEN_S', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_LVMD_SOUND_PULSE_LEN_S', index=6,
      number=2219, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_CONT_TONE_LIMIT_M', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_CONT_TONE_LIMIT_M', index=7,
      number=2652, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_PING_PONG_DURATION_S', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_PING_PONG_DURATION_S', index=8,
      number=3893, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_MIN_PAUSE_DURATION_S', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_MIN_PAUSE_DURATION_S', index=9,
      number=2633, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_MAX_PAUSE_DURATION_S', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_MAX_PAUSE_DURATION_S', index=10,
      number=2156, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_ACK_PAUSE_DURATION_S', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_ACK_PAUSE_DURATION_S', index=11,
      number=3467, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_WHP_LOW_PAUSE_DURATION_S', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_WHP_LOW_PAUSE_DURATION_S', index=12,
      number=420, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_WHP_HIGH_PAUSE_DURATION_S', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_WHP_HIGH_PAUSE_DURATION_S', index=13,
      number=1335, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_LSCA_PAUSE_DURATION_S', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_LSCA_PAUSE_DURATION_S', index=14,
      number=1350, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_ACTIVE_DISTANCE_M', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_ACTIVE_DISTANCE_M', index=15,
      number=376, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_VOLUME_FRONT_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_VOLUME_FRONT_NU', index=16,
      number=1642, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_VOLUME_REAR_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_VOLUME_REAR_NU', index=17,
      number=319, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_ACK_TONE_PITCH_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_ACK_TONE_PITCH_NU', index=18,
      number=3030, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_ACK_TONE_VOL_OFFSET_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_ACK_TONE_VOL_OFFSET_NU', index=19,
      number=2676, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_ERR_TONE_VOL_OFFSET_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_ERR_TONE_VOL_OFFSET_NU', index=20,
      number=1367, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_ERR_TONE_PITCH_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_ERR_TONE_PITCH_NU', index=21,
      number=1091, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_WARN_TONE_PITCH_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_WARN_TONE_PITCH_NU', index=22,
      number=3000, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_WHP_WARN_TONE_PITCH_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_WHP_WARN_TONE_PITCH_NU', index=23,
      number=3769, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_LSCA_LOW_TONE_PITCH_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_LSCA_LOW_TONE_PITCH_NU', index=24,
      number=3342, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_LSCA_HIGH_TONE_PITCH_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_LSCA_HIGH_TONE_PITCH_NU', index=25,
      number=3641, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_LVMD_TONE_PITCH_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_LVMD_TONE_PITCH_NU', index=26,
      number=1145, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_WEAK_VOLUME_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_WEAK_VOLUME_NU', index=27,
      number=3912, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_ENABLED_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_ENABLED_NU', index=28,
      number=463, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_DIFFERENT_TONES_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_DIFFERENT_TONES_NU', index=29,
      number=4019, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TH_ACK_ACTIVE_NU', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params.TH_ACK_ACTIVE_NU', index=30,
      number=1227, type=8, cpp_type=7, label=1,
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
  serialized_start=107,
  serialized_end=1164,
)


_FC_MF_TONEHANDLER_PARAMS_ARRAY_PORT = _descriptor.Descriptor(
  name='FC_MF_ToneHandler_Params_array_port',
  full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params_array_port.data', index=0,
      number=3897, type=11, cpp_type=10, label=3,
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
  serialized_start=1166,
  serialized_end=1282,
)

_FC_MF_TONEHANDLER_PARAMS.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_FC_MF_TONEHANDLER_PARAMS_ARRAY_PORT.fields_by_name['data'].message_type = _FC_MF_TONEHANDLER_PARAMS
DESCRIPTOR.message_types_by_name['FC_MF_ToneHandler_Params'] = _FC_MF_TONEHANDLER_PARAMS
DESCRIPTOR.message_types_by_name['FC_MF_ToneHandler_Params_array_port'] = _FC_MF_TONEHANDLER_PARAMS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FC_MF_ToneHandler_Params = _reflection.GeneratedProtocolMessageType('FC_MF_ToneHandler_Params', (_message.Message,), {
  'DESCRIPTOR' : _FC_MF_TONEHANDLER_PARAMS,
  '__module__' : 'mf_tonh.fc_mf_tone_handler_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params)
  })
_sym_db.RegisterMessage(FC_MF_ToneHandler_Params)

FC_MF_ToneHandler_Params_array_port = _reflection.GeneratedProtocolMessageType('FC_MF_ToneHandler_Params_array_port', (_message.Message,), {
  'DESCRIPTOR' : _FC_MF_TONEHANDLER_PARAMS_ARRAY_PORT,
  '__module__' : 'mf_tonh.fc_mf_tone_handler_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_tonh.fc_mf_tone_handler_params.FC_MF_ToneHandler_Params_array_port)
  })
_sym_db.RegisterMessage(FC_MF_ToneHandler_Params_array_port)


# @@protoc_insertion_point(module_scope)
