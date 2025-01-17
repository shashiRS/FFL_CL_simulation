# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mf_drvwarnsm/fc_drv_warn_sm_params.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from eco import signal_header_pb2 as eco_dot_signal__header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='mf_drvwarnsm/fc_drv_warn_sm_params.proto',
  package='pb.mf_drvwarnsm.fc_drv_warn_sm_params',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n(mf_drvwarnsm/fc_drv_warn_sm_params.proto\x12%pb.mf_drvwarnsm.fc_drv_warn_sm_params\x1a\x17\x65\x63o/signal_header.proto\"\xb8\r\n\x13\x46\x43_DrvWarnSM_Params\x12\x18\n\x0fuiVersionNumber\x18\xcc\x10 \x01(\r\x12\x37\n\nsSigHeader\x18\x89\x08 \x01(\x0b\x32\".pb.eco.signal_header.SignalHeader\x12#\n\x1a\x44WF_L_SPEED_HYSTERESIS_MPS\x18\xc4\x08 \x01(\x02\x12\x1a\n\x11PDW_L_INIT_TIME_S\x18\x9b\x05 \x01(\x02\x12\'\n\x1ePDW_L_STABILIZATON_WAIT_TIME_S\x18\xa6\x14 \x01(\x02\x12\x1f\n\x17PDW_L_STANDSTILL_TIME_S\x18. \x01(\x02\x12&\n\x1dPDW_L_REV_GEAR_DEB_TIME_MAN_S\x18\xab\t \x01(\x02\x12&\n\x1dPDW_L_REV_GEAR_DEB_TIME_AUT_S\x18\xf4\x14 \x01(\x02\x12!\n\x18PDW_L_VEL_THRESH_OFF_MPS\x18\xa8\x18 \x01(\x02\x12\x1c\n\x13WHP_L_MAX_SPEED_MPS\x18\xf8\x1b \x01(\x02\x12%\n\x1cWHP_L_MIN_WHL_ANGLE_DIFF_RAD\x18\xac\x1b \x01(\x02\x12%\n\x1cPDW_L_ROLLBACK_W_OBST_DIST_M\x18\x9e\x1b \x01(\x02\x12&\n\x1dPDW_L_ROLLBACK_WO_OBST_DIST_M\x18\xd9\x08 \x01(\x02\x12#\n\x1aPDW_L_MAX_DIST_CONT_TONE_M\x18\x91\x02 \x01(\x02\x12&\n\x1dPDW_L_MIN_DIST_REAR_SENSORS_M\x18\xd7\x1e \x01(\x02\x12\x1d\n\x14\x44WF_C_OPS_VARIANT_NU\x18\xa3\x1c \x01(\r\x12#\n\x1a\x44WF_C_TRANSMISSION_TYPE_NU\x18\xf4\x1c \x01(\x08\x12\x1e\n\x15PDW_L_DRIVING_TUBE_NU\x18\x90\x12 \x01(\x08\x12&\n\x1dPDW_L_AUTOMATIC_ACTIVATION_NU\x18\x8a\x13 \x01(\x08\x12!\n\x18PDW_L_AUTO_ACT_BUTTON_NU\x18\x8b\x0f \x01(\x08\x12(\n\x1fPDW_L_AUTOM_ACTIV_STANDSTILL_NU\x18\xcb\x14 \x01(\x08\x12%\n\x1cPDW_L_ROLLBACK_ACTIVATION_NU\x18\xf8\x04 \x01(\x08\x12#\n\x1aPDW_L_DEACTIV_BY_P_GEAR_NU\x18\x99\x1e \x01(\x08\x12 \n\x17PDW_L_DEACTIV_BY_EPB_NU\x18\x97\x11 \x01(\x08\x12\'\n\x1ePDW_L_P_GEAR_RESET_AUTO_ACT_NU\x18\xe2\n \x01(\x08\x12$\n\x1bPDW_L_EPB_RESET_AUTO_ACT_NU\x18\xd1\x11 \x01(\x08\x12$\n\x1bPDW_L_ROLLBACK_FULL_VIEW_NU\x18\xfb\x1c \x01(\x08\x12\"\n\x19PDW_L_ROLLBACK_TRAILER_NU\x18\xa1\x1d \x01(\x08\x12\x1f\n\x16PDW_L_ACT_PDW_BY_AP_NU\x18\xe6\x03 \x01(\x08\x12 \n\x18WHP_L_DEACT_WHP_BY_AP_NU\x18\x11 \x01(\x08\x12%\n\x1c\x44WF_L_TONE_SUPPRESS_BY_AP_NU\x18\xef\r \x01(\x08\x12.\n%DWF_L_TONE_REDUCT_INTER_STANDSTILL_NU\x18\x8b\x0e \x01(\x08\x12-\n$DWF_L_TONE_REDUCT_CONT_STANDSTILL_NU\x18\xce\x11 \x01(\x08\x12&\n\x1dPDW_L_STANDSTILL_INTER_TIME_S\x18\xba\x1b \x01(\x02\x12%\n\x1cPDW_L_STANDSTILL_CONT_TIME_S\x18\x87\x1f \x01(\x02\x12%\n\x1cPDW_L_AUTO_ACTIV_CRITICAL_NU\x18\x95\x02 \x01(\x08\x12\'\n\x1ePDW_L_MAX_DIST_CONT_TONE_FWD_M\x18\xaf\x1e \x01(\x02\x12(\n\x1fPDW_L_MAX_DIST_CONT_TONE_REAR_M\x18\xac\x0c \x01(\x02\x12\'\n\x1ePDW_L_MAX_DIST_CONT_TONE_LAT_M\x18\xdc\r \x01(\x02\x12\"\n\x19PDW_L_CONT_DIST_EXTEND_NU\x18\xc3\r \x01(\x08\x12!\n\x18PDW_AUTO_ACTIV_THRESHOLD\x18\xab\x12 \x01(\x02\x12$\n\x1bPDW_L_CRIT_ACT_PDW_BY_AP_NU\x18\xe1\x18 \x01(\x08\x12\"\n\x19PDW_L_FRONT_SECT_ACTIV_NU\x18\x87\x02 \x01(\x08\x12 \n\x17PDW_L_LAT_SECT_ACTIV_NU\x18\x8c\r \x01(\x08\x12!\n\x18PDW_L_REAR_SECT_ACTIV_NU\x18\xf8\x1d \x01(\x08\"k\n\x1e\x46\x43_DrvWarnSM_Params_array_port\x12I\n\x04\x64\x61ta\x18\x8d\x1f \x03(\x0b\x32:.pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params'
  ,
  dependencies=[eco_dot_signal__header__pb2.DESCRIPTOR,])




_FC_DRVWARNSM_PARAMS = _descriptor.Descriptor(
  name='FC_DrvWarnSM_Params',
  full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='uiVersionNumber', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.uiVersionNumber', index=0,
      number=2124, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sSigHeader', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.sSigHeader', index=1,
      number=1033, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='DWF_L_SPEED_HYSTERESIS_MPS', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.DWF_L_SPEED_HYSTERESIS_MPS', index=2,
      number=1092, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_INIT_TIME_S', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_INIT_TIME_S', index=3,
      number=667, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_STABILIZATON_WAIT_TIME_S', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_STABILIZATON_WAIT_TIME_S', index=4,
      number=2598, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_STANDSTILL_TIME_S', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_STANDSTILL_TIME_S', index=5,
      number=46, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_REV_GEAR_DEB_TIME_MAN_S', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_REV_GEAR_DEB_TIME_MAN_S', index=6,
      number=1195, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_REV_GEAR_DEB_TIME_AUT_S', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_REV_GEAR_DEB_TIME_AUT_S', index=7,
      number=2676, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_VEL_THRESH_OFF_MPS', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_VEL_THRESH_OFF_MPS', index=8,
      number=3112, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='WHP_L_MAX_SPEED_MPS', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.WHP_L_MAX_SPEED_MPS', index=9,
      number=3576, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='WHP_L_MIN_WHL_ANGLE_DIFF_RAD', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.WHP_L_MIN_WHL_ANGLE_DIFF_RAD', index=10,
      number=3500, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_ROLLBACK_W_OBST_DIST_M', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_ROLLBACK_W_OBST_DIST_M', index=11,
      number=3486, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_ROLLBACK_WO_OBST_DIST_M', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_ROLLBACK_WO_OBST_DIST_M', index=12,
      number=1113, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_MAX_DIST_CONT_TONE_M', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_MAX_DIST_CONT_TONE_M', index=13,
      number=273, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_MIN_DIST_REAR_SENSORS_M', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_MIN_DIST_REAR_SENSORS_M', index=14,
      number=3927, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='DWF_C_OPS_VARIANT_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.DWF_C_OPS_VARIANT_NU', index=15,
      number=3619, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='DWF_C_TRANSMISSION_TYPE_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.DWF_C_TRANSMISSION_TYPE_NU', index=16,
      number=3700, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_DRIVING_TUBE_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_DRIVING_TUBE_NU', index=17,
      number=2320, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_AUTOMATIC_ACTIVATION_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_AUTOMATIC_ACTIVATION_NU', index=18,
      number=2442, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_AUTO_ACT_BUTTON_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_AUTO_ACT_BUTTON_NU', index=19,
      number=1931, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_AUTOM_ACTIV_STANDSTILL_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_AUTOM_ACTIV_STANDSTILL_NU', index=20,
      number=2635, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_ROLLBACK_ACTIVATION_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_ROLLBACK_ACTIVATION_NU', index=21,
      number=632, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_DEACTIV_BY_P_GEAR_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_DEACTIV_BY_P_GEAR_NU', index=22,
      number=3865, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_DEACTIV_BY_EPB_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_DEACTIV_BY_EPB_NU', index=23,
      number=2199, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_P_GEAR_RESET_AUTO_ACT_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_P_GEAR_RESET_AUTO_ACT_NU', index=24,
      number=1378, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_EPB_RESET_AUTO_ACT_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_EPB_RESET_AUTO_ACT_NU', index=25,
      number=2257, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_ROLLBACK_FULL_VIEW_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_ROLLBACK_FULL_VIEW_NU', index=26,
      number=3707, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_ROLLBACK_TRAILER_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_ROLLBACK_TRAILER_NU', index=27,
      number=3745, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_ACT_PDW_BY_AP_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_ACT_PDW_BY_AP_NU', index=28,
      number=486, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='WHP_L_DEACT_WHP_BY_AP_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.WHP_L_DEACT_WHP_BY_AP_NU', index=29,
      number=17, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='DWF_L_TONE_SUPPRESS_BY_AP_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.DWF_L_TONE_SUPPRESS_BY_AP_NU', index=30,
      number=1775, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='DWF_L_TONE_REDUCT_INTER_STANDSTILL_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.DWF_L_TONE_REDUCT_INTER_STANDSTILL_NU', index=31,
      number=1803, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='DWF_L_TONE_REDUCT_CONT_STANDSTILL_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.DWF_L_TONE_REDUCT_CONT_STANDSTILL_NU', index=32,
      number=2254, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_STANDSTILL_INTER_TIME_S', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_STANDSTILL_INTER_TIME_S', index=33,
      number=3514, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_STANDSTILL_CONT_TIME_S', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_STANDSTILL_CONT_TIME_S', index=34,
      number=3975, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_AUTO_ACTIV_CRITICAL_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_AUTO_ACTIV_CRITICAL_NU', index=35,
      number=277, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_MAX_DIST_CONT_TONE_FWD_M', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_MAX_DIST_CONT_TONE_FWD_M', index=36,
      number=3887, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_MAX_DIST_CONT_TONE_REAR_M', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_MAX_DIST_CONT_TONE_REAR_M', index=37,
      number=1580, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_MAX_DIST_CONT_TONE_LAT_M', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_MAX_DIST_CONT_TONE_LAT_M', index=38,
      number=1756, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_CONT_DIST_EXTEND_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_CONT_DIST_EXTEND_NU', index=39,
      number=1731, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_AUTO_ACTIV_THRESHOLD', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_AUTO_ACTIV_THRESHOLD', index=40,
      number=2347, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_CRIT_ACT_PDW_BY_AP_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_CRIT_ACT_PDW_BY_AP_NU', index=41,
      number=3169, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_FRONT_SECT_ACTIV_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_FRONT_SECT_ACTIV_NU', index=42,
      number=263, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_LAT_SECT_ACTIV_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_LAT_SECT_ACTIV_NU', index=43,
      number=1676, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='PDW_L_REAR_SECT_ACTIV_NU', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params.PDW_L_REAR_SECT_ACTIV_NU', index=44,
      number=3832, type=8, cpp_type=7, label=1,
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
  serialized_start=109,
  serialized_end=1829,
)


_FC_DRVWARNSM_PARAMS_ARRAY_PORT = _descriptor.Descriptor(
  name='FC_DrvWarnSM_Params_array_port',
  full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params_array_port',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params_array_port.data', index=0,
      number=3981, type=11, cpp_type=10, label=3,
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
  serialized_start=1831,
  serialized_end=1938,
)

_FC_DRVWARNSM_PARAMS.fields_by_name['sSigHeader'].message_type = eco_dot_signal__header__pb2._SIGNALHEADER
_FC_DRVWARNSM_PARAMS_ARRAY_PORT.fields_by_name['data'].message_type = _FC_DRVWARNSM_PARAMS
DESCRIPTOR.message_types_by_name['FC_DrvWarnSM_Params'] = _FC_DRVWARNSM_PARAMS
DESCRIPTOR.message_types_by_name['FC_DrvWarnSM_Params_array_port'] = _FC_DRVWARNSM_PARAMS_ARRAY_PORT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FC_DrvWarnSM_Params = _reflection.GeneratedProtocolMessageType('FC_DrvWarnSM_Params', (_message.Message,), {
  'DESCRIPTOR' : _FC_DRVWARNSM_PARAMS,
  '__module__' : 'mf_drvwarnsm.fc_drv_warn_sm_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params)
  })
_sym_db.RegisterMessage(FC_DrvWarnSM_Params)

FC_DrvWarnSM_Params_array_port = _reflection.GeneratedProtocolMessageType('FC_DrvWarnSM_Params_array_port', (_message.Message,), {
  'DESCRIPTOR' : _FC_DRVWARNSM_PARAMS_ARRAY_PORT,
  '__module__' : 'mf_drvwarnsm.fc_drv_warn_sm_params_pb2'
  # @@protoc_insertion_point(class_scope:pb.mf_drvwarnsm.fc_drv_warn_sm_params.FC_DrvWarnSM_Params_array_port)
  })
_sym_db.RegisterMessage(FC_DrvWarnSM_Params_array_port)


# @@protoc_insertion_point(module_scope)
