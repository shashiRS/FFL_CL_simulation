# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: eco/vehicle_param.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='eco/vehicle_param.proto',
  package='pb.eco.vehicle_param',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x17\x65\x63o/vehicle_param.proto\x12\x14pb.eco.vehicle_param\x1a\x17\x65\x63o/signal_header.proto\"\x92\x07\n\x0cVehicleParam\x12\x36\n\tsigHeader\x18\xaa\x0b \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12\x14\n\x0bvehicleMass\x18\xad\x13 \x01(\x02\x12\x16\n\raxisLoadDistr\x18\xb3\x05 \x01(\x02\x12\x0f\n\x06length\x18\xda\x0f \x01(\x02\x12\x0e\n\x05width\x18\xaf\x1d \x01(\x02\x12\x12\n\twheelbase\x18\xa8\x10 \x01(\x02\x12\x18\n\x0ftrackWidthFront\x18\xf6\x1c \x01(\x02\x12\x17\n\x0etrackWidthRear\x18\x8f\x1d \x01(\x02\x12\x16\n\roverhangFront\x18\xd4\x01 \x01(\x02\x12\x15\n\x0coverhangRear\x18\xad\x04 \x01(\x02\x12 \n\x17wheelCircumferenceFront\x18\xa9\x18 \x01(\x02\x12\x1f\n\x16wheelCircumferenceRear\x18\xea\x15 \x01(\x02\x12\x19\n\x10\x63\x65nterOfGravityX\x18\xc7\x1f \x01(\x02\x12\x19\n\x10\x63\x65nterOfGravityY\x18\xe6\x1f \x01(\x02\x12\x19\n\x10\x63\x65nterOfGravityZ\x18\x85\x1f \x01(\x02\x12\x1f\n\x16isVehicleMassAvailable\x18\xd0\x01 \x01(\x08\x12!\n\x18isAxisLoadDistrAvailable\x18\xda\n \x01(\x08\x12\x1a\n\x11isLengthAvailable\x18\xf3\x14 \x01(\x08\x12\x19\n\x10isWidthAvailable\x18\xe3\x18 \x01(\x08\x12\x1d\n\x14isWheelbaseAvailable\x18\xf9\x0b \x01(\x08\x12#\n\x1aisTrackWidthFrontAvailable\x18\x8b\x07 \x01(\x08\x12\"\n\x19isTrackWidthRearAvailable\x18\xce\x19 \x01(\x08\x12!\n\x18isOverhangFrontAvailable\x18\xe3\x14 \x01(\x08\x12 \n\x17isOverhangRearAvailable\x18\xaa\x15 \x01(\x08\x12+\n\"isWheelCircumferenceFrontAvailable\x18\x94\x19 \x01(\x08\x12*\n!isWheelCircumferenceRearAvailable\x18\xbe\x18 \x01(\x08\x12$\n\x1bisCenterOfGravityXAvailable\x18\xc7\x15 \x01(\x08\x12$\n\x1bisCenterOfGravityYAvailable\x18\x82\x0b \x01(\x08\x12$\n\x1bisCenterOfGravityZAvailable\x18\xcd\x08 \x01(\x08\"L\n\x17VehicleParam_array_port\x12\x31\n\x04\x64\x61ta\x18\xe6\x18 \x03(\x0b\x32\".pb.eco.vehicle_param.VehicleParam'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_VEHICLEPARAM = _descriptor.Descriptor(
  name='VehicleParam',
  full_name='pb.eco.vehicle_param.VehicleParam',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='sigHeader', full_name='pb.eco.vehicle_param.VehicleParam.sigHeader', index=0,
      number=1450, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='vehicleMass', full_name='pb.eco.vehicle_param.VehicleParam.vehicleMass', index=1,
      number=2477, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='axisLoadDistr', full_name='pb.eco.vehicle_param.VehicleParam.axisLoadDistr', index=2,
      number=691, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='length', full_name='pb.eco.vehicle_param.VehicleParam.length', index=3,
      number=2010, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='width', full_name='pb.eco.vehicle_param.VehicleParam.width', index=4,
      number=3759, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='wheelbase', full_name='pb.eco.vehicle_param.VehicleParam.wheelbase', index=5,
      number=2088, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='trackWidthFront', full_name='pb.eco.vehicle_param.VehicleParam.trackWidthFront', index=6,
      number=3702, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='trackWidthRear', full_name='pb.eco.vehicle_param.VehicleParam.trackWidthRear', index=7,
      number=3727, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='overhangFront', full_name='pb.eco.vehicle_param.VehicleParam.overhangFront', index=8,
      number=212, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='overhangRear', full_name='pb.eco.vehicle_param.VehicleParam.overhangRear', index=9,
      number=557, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='wheelCircumferenceFront', full_name='pb.eco.vehicle_param.VehicleParam.wheelCircumferenceFront', index=10,
      number=3113, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='wheelCircumferenceRear', full_name='pb.eco.vehicle_param.VehicleParam.wheelCircumferenceRear', index=11,
      number=2794, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='centerOfGravityX', full_name='pb.eco.vehicle_param.VehicleParam.centerOfGravityX', index=12,
      number=4039, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='centerOfGravityY', full_name='pb.eco.vehicle_param.VehicleParam.centerOfGravityY', index=13,
      number=4070, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='centerOfGravityZ', full_name='pb.eco.vehicle_param.VehicleParam.centerOfGravityZ', index=14,
      number=3973, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isVehicleMassAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isVehicleMassAvailable', index=15,
      number=208, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isAxisLoadDistrAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isAxisLoadDistrAvailable', index=16,
      number=1370, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isLengthAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isLengthAvailable', index=17,
      number=2675, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isWidthAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isWidthAvailable', index=18,
      number=3171, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isWheelbaseAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isWheelbaseAvailable', index=19,
      number=1529, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isTrackWidthFrontAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isTrackWidthFrontAvailable', index=20,
      number=907, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isTrackWidthRearAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isTrackWidthRearAvailable', index=21,
      number=3278, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isOverhangFrontAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isOverhangFrontAvailable', index=22,
      number=2659, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isOverhangRearAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isOverhangRearAvailable', index=23,
      number=2730, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isWheelCircumferenceFrontAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isWheelCircumferenceFrontAvailable', index=24,
      number=3220, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isWheelCircumferenceRearAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isWheelCircumferenceRearAvailable', index=25,
      number=3134, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isCenterOfGravityXAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isCenterOfGravityXAvailable', index=26,
      number=2759, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isCenterOfGravityYAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isCenterOfGravityYAvailable', index=27,
      number=1410, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isCenterOfGravityZAvailable', full_name='pb.eco.vehicle_param.VehicleParam.isCenterOfGravityZAvailable', index=28,
      number=1101, type=8, cpp_type=7, label=1,
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
  serialized_start=75,
  serialized_end=989,
)


_VEHICLEPARAM_ARRAY_PORT = _descriptor.Descriptor(
  name='VehicleParam_array_port',
  full_name='pb.eco.vehicle_param.VehicleParam_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.eco.vehicle_param.VehicleParam_array_port.data', index=0,
      number=3174, type=11, cpp_type=10, label=3,
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
  serialized_start=991,
  serialized_end=1067,
)

_VEHICLEPARAM.fields_by_name['sigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_VEHICLEPARAM_ARRAY_PORT.fields_by_name['data'].message_type = _VEHICLEPARAM
DESCRIPTOR.message_types_by_name['VehicleParam'] = _VEHICLEPARAM
DESCRIPTOR.message_types_by_name['VehicleParam_array_port'] = _VEHICLEPARAM_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

VehicleParam = _reflection.GeneratedProtocolMessageType('VehicleParam', (_message.Message,), {
  'DESCRIPTOR' : _VEHICLEPARAM,
  '__module__' : 'eco.vehicle_param_pb2'
  # @@protoc_insertion_point(class_scope:pb.eco.vehicle_param.VehicleParam)
  })
_sym_db.RegisterMessage(VehicleParam)

VehicleParam_array_port = _reflection.GeneratedProtocolMessageType('VehicleParam_array_port', (_message.Message,), {
  'DESCRIPTOR' : _VEHICLEPARAM_ARRAY_PORT,
  '__module__' : 'eco.vehicle_param_pb2'
  # @@protoc_insertion_point(class_scope:pb.eco.vehicle_param.VehicleParam_array_port)
  })
_sym_db.RegisterMessage(VehicleParam_array_port)


# @@protoc_insertion_point(module_scope)
