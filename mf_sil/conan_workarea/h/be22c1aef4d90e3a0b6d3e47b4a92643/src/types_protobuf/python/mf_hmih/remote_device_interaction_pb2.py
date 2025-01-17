# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_hmih/remote_device_interaction.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from mf_hmih import apuser_action_remote_device_pb2 as mf__hmih_dot_apuser__action__remote__device__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_hmih/remote_device_interaction.proto',
  package='pb.mf_hmih.remote_device_interaction',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\'mf_hmih/remote_device_interaction.proto\x12$pb.mf_hmih.remote_device_interaction\x1a)mf_hmih/apuser_action_remote_device.proto\"\xf9\x02\n\x17RemoteDeviceInteraction\x12\x1a\n\x11\x62\x61tteryLevel_perc\x18\x8d\x02 \x01(\x02\x12\x1b\n\x12\x66ingerPositionX_px\x18\xff\x19 \x01(\r\x12\x1b\n\x12\x66ingerPositionY_px\x18\xcb\x14 \x01(\r\x12\x66\n\x1b\x61pUserActionRemoteDevice_nu\x18\xdf\x1a \x01(\x0e\x32@.pb.mf_hmih.apuser_action_remote_device.APUserActionRemoteDevice\x12\x18\n\x0f\x61liveCounter_nu\x18\xde\x1b \x01(\r\x12\x1d\n\x14\x64\x65\x61\x64MansSwitchBtn_nu\x18\xb7\r \x01(\x08\x12\x12\n\tpaired_nu\x18\xf7\t \x01(\x08\x12\x15\n\x0c\x63onnected_nu\x18\xa2\x12 \x01(\x08\x12\x1d\n\x14screenResolutionX_px\x18\x9d\x1f \x01(\r\x12\x1d\n\x14screenResolutionY_px\x18\xa9\x12 \x01(\r\"r\n\"RemoteDeviceInteraction_array_port\x12L\n\x04\x64\x61ta\x18\xc6\t \x03(\x0b\x32=.pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction'
  ,
  dependencies=[mf__hmih_dot_apuser__action__remote__device__pb2.DESCRIPTOR,])




_REMOTEDEVICEINTERACTION = _descriptor.Descriptor(
  name='RemoteDeviceInteraction',
  full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='batteryLevel_perc', full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction.batteryLevel_perc', index=0,
      number=269, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='fingerPositionX_px', full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction.fingerPositionX_px', index=1,
      number=3327, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='fingerPositionY_px', full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction.fingerPositionY_px', index=2,
      number=2635, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='apUserActionRemoteDevice_nu', full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction.apUserActionRemoteDevice_nu', index=3,
      number=3423, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='aliveCounter_nu', full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction.aliveCounter_nu', index=4,
      number=3550, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='deadMansSwitchBtn_nu', full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction.deadMansSwitchBtn_nu', index=5,
      number=1719, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='paired_nu', full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction.paired_nu', index=6,
      number=1271, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='connected_nu', full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction.connected_nu', index=7,
      number=2338, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='screenResolutionX_px', full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction.screenResolutionX_px', index=8,
      number=3997, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='screenResolutionY_px', full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction.screenResolutionY_px', index=9,
      number=2345, type=13, cpp_type=3, label=1,
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
  serialized_start=125,
  serialized_end=502,
)


_REMOTEDEVICEINTERACTION_ARRAY_PORT = _descriptor.Descriptor(
  name='RemoteDeviceInteraction_array_port',
  full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction_array_port.data', index=0,
      number=1222, type=11, cpp_type=10, label=3,
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
  serialized_start=504,
  serialized_end=618,
)

_REMOTEDEVICEINTERACTION.fields_by_name['apUserActionRemoteDevice_nu'].enum_type = mf__hmih_dot_apuser__action__remote__device__pb2._APUSERACTIONREMOTEDEVICE
_REMOTEDEVICEINTERACTION_ARRAY_PORT.fields_by_name['data'].message_type = _REMOTEDEVICEINTERACTION
DESCRIPTOR.message_types_by_name['RemoteDeviceInteraction'] = _REMOTEDEVICEINTERACTION
DESCRIPTOR.message_types_by_name['RemoteDeviceInteraction_array_port'] = _REMOTEDEVICEINTERACTION_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

RemoteDeviceInteraction = _reflection.GeneratedProtocolMessageType('RemoteDeviceInteraction', (_message.Message,), {
  'DESCRIPTOR' : _REMOTEDEVICEINTERACTION,
  '__module__' : 'mf_hmih.remote_device_interaction_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction)
  })
_sym_db.RegisterMessage(RemoteDeviceInteraction)

RemoteDeviceInteraction_array_port = _reflection.GeneratedProtocolMessageType('RemoteDeviceInteraction_array_port', (_message.Message,), {
  'DESCRIPTOR' : _REMOTEDEVICEINTERACTION_ARRAY_PORT,
  '__module__' : 'mf_hmih.remote_device_interaction_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_hmih.remote_device_interaction.RemoteDeviceInteraction_array_port)
  })
_sym_db.RegisterMessage(RemoteDeviceInteraction_array_port)


# @@protoc_insertion_point(module_scope)
