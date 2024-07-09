# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ap_hmitoap/screen_types.proto

from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ap_hmitoap/screen_types.proto',
  package='pb.ap_hmitoap.screen_types',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=b'\n\x1d\x61p_hmitoap/screen_types.proto\x12\x1apb.ap_hmitoap.screen_types*\x9f\x18\n\x0bScreenTypes\x12\x10\n\x0c\x42LANK_SCREEN\x10\x00\x12\x0f\n\x0b\x42OWL_VIEW_1\x10\x01\x12\x0f\n\x0b\x42OWL_VIEW_2\x10\x02\x12\x0f\n\x0b\x42OWL_VIEW_3\x10\x03\x12\x0f\n\x0b\x42OWL_VIEW_4\x10\x04\x12\x0f\n\x0b\x42OWL_VIEW_5\x10\x05\x12\x0f\n\x0b\x42OWL_VIEW_6\x10\x06\x12\x0f\n\x0b\x42OWL_VIEW_7\x10\x07\x12\x0f\n\x0b\x42OWL_VIEW_8\x10\x08\x12\x0f\n\x0b\x42OWL_VIEW_9\x10\t\x12\x10\n\x0c\x42OWL_VIEW_10\x10\n\x12\x10\n\x0c\x42OWL_VIEW_11\x10\x0b\x12\x10\n\x0c\x42OWL_VIEW_12\x10\x0c\x12\x10\n\x0c\x42OWL_VIEW_13\x10\r\x12\x10\n\x0c\x42OWL_VIEW_14\x10\x0e\x12\x16\n\x12\x42OWL_VIEW_MODIFIED\x10\x0f\x12\x13\n\x0f\x46RONT_VIEW_FULL\x10\x10\x12\x14\n\x10\x46RONT_VIEW_SPLIT\x10\x11\x12\x18\n\x14PANORAMIC_FRONT_VIEW\x10\x12\x12\x10\n\x0c\x42LACK_SCREEN\x10\x13\x12\x12\n\x0e\x46RONT_TOP_ZOOM\x10\x14\x12\x13\n\x0fLAST_FRONT_VIEW\x10\x1e\x12\x0c\n\x08TOP_VIEW\x10\x1f\x12\x10\n\x0cTOP_VIEW_SFM\x10 \x12\x10\n\x0cTOP_VIEW_PSD\x10!\x12\x10\n\x0cTOP_VIEW_PMD\x10\"\x12\x11\n\rLAST_TOP_VIEW\x10-\x12\x12\n\x0eREAR_VIEW_FULL\x10.\x12\x13\n\x0fREAR_VIEW_SPLIT\x10/\x12\x17\n\x13PANORAMIC_REAR_VIEW\x10\x30\x12\x10\n\x0cTRAILER_VIEW\x10\x31\x12\x11\n\rREAR_TOP_ZOOM\x10\x32\x12\x12\n\x0eLAST_REAR_VIEW\x10<\x12\x13\n\x0fKERB_VIEW_FRONT\x10=\x12\x12\n\x0eKERB_VIEW_REAR\x10>\x12\x12\n\x0eLAST_KERB_VIEW\x10K\x12\x13\n\x0f\x41RA_REAR_NORMAL\x10L\x12\x16\n\x12\x41RA_REAR_IRREGULAR\x10M\x12\x18\n\x14\x41RA_PANORAMIC_NORMAL\x10N\x12\x1b\n\x17\x41RA_PANORAMIC_IRREGULAR\x10O\x12\x13\n\x0fLEFT_VIEW_SPLIT\x10P\x12\x12\n\x0eLEFT_VIEW_FULL\x10Q\x12\x17\n\x13PANORAMIC_LEFT_VIEW\x10R\x12\x14\n\x10RIGHT_VIEW_SPLIT\x10S\x12\x13\n\x0fRIGHT_VIEW_FULL\x10T\x12\x18\n\x14PANORAMIC_RIGHT_VIEW\x10U\x12\x11\n\rLAST_ARA_VIEW\x10Z\x12\x12\n\x0eIPA_FRONT_VIEW\x10[\x12\x11\n\rIPA_REAR_VIEW\x10\\\x12\x10\n\x0cIPA_TOP_VIEW\x10]\x12\x13\n\x0fPARK_FRONT_VIEW\x10^\x12\x12\n\x0ePARK_REAR_VIEW\x10_\x12\x11\n\rPARK_TOP_VIEW\x10`\x12\x18\n\x14PARK_SPACES_TOP_VIEW\x10\x61\x12\x16\n\x12PARK_WARN_TOP_VIEW\x10\x62\x12\x18\n\x14PARK_TRANSPARENT_PDW\x10\x63\x12\x11\n\rLAST_IPA_VIEW\x10i\x12\x0f\n\x0bMORPH_FRONT\x10j\x12\x14\n\x10MORPH_FRONT_FULL\x10k\x12\x14\n\x10LAST_MORPH_FRONT\x10x\x12\x0e\n\nMORPH_REAR\x10y\x12\x13\n\x0fMORPH_REAR_FULL\x10z\x12\r\n\tMORPH_ARA\x10{\x12\x14\n\x0fLAST_MORPH_VIEW\x10\x87\x01\x12\x14\n\x0fSMARTPHONE_VIEW\x10\x97\x01\x12\x14\n\x0f\x42LIND_SPOT_LEFT\x10\x98\x01\x12\x15\n\x10\x42LIND_SPOT_RIGHT\x10\x99\x01\x12\x16\n\x11LAST_SPECIAL_VIEW\x10\xa5\x01\x12\x15\n\x10TRANSPARENT_HOOD\x10\xa6\x01\x12\x16\n\x11TRANSPARENT_ALPHA\x10\xa7\x01\x12\x16\n\x11TRANSPARENT_GHOST\x10\xa8\x01\x12\x16\n\x11TRANSPARENT_TRUNK\x10\xa9\x01\x12\x1c\n\x17TRANSPARENT_ALPHA_SPLIT\x10\xaa\x01\x12\x1b\n\x16TRANSPARENT_HOOD_SPLIT\x10\xab\x01\x12\x1c\n\x17TRANSPARENT_TRUNK_SPLIT\x10\xac\x01\x12\x17\n\x12\x41\x44\x41PTIVE_FULL_BOWL\x10\xad\x01\x12\x10\n\x0bSTATIC_BOWL\x10\xae\x01\x12\x18\n\x13\x41\x44\x41PTIVE_SPLIT_BOWL\x10\xaf\x01\x12\x16\n\x11STATIC_SPLIT_BOWL\x10\xb0\x01\x12\x10\n\x0bRAW_CAMERAS\x10\xb5\x01\x12\x14\n\x0fRAW_CAMERAS_SFM\x10\xb6\x01\x12\x1e\n\x19\x44\x45\x42UG_FRONT_CAMERA_EXTERN\x10\xc8\x01\x12\x1f\n\x1a\x44\x45\x42UG_FRONT_CAMERA_VECTORS\x10\xc9\x01\x12\x1d\n\x18\x44\x45\x42UG_FRONT_CAMERA_EDGES\x10\xca\x01\x12 \n\x1b\x44\x45\x42UG_FRONT_CAMERA_CLEANING\x10\xcb\x01\x12\x1d\n\x18\x44\x45\x42UG_REAR_CAMERA_EXTERN\x10\xcc\x01\x12\x1e\n\x19\x44\x45\x42UG_REAR_CAMERA_VECTORS\x10\xcd\x01\x12\x1c\n\x17\x44\x45\x42UG_REAR_CAMERA_EDGES\x10\xce\x01\x12\x1f\n\x1a\x44\x45\x42UG_REAR_CAMERA_CLEANING\x10\xcf\x01\x12\x1d\n\x18\x44\x45\x42UG_LEFT_CAMERA_EXTERN\x10\xd0\x01\x12\x1e\n\x19\x44\x45\x42UG_LEFT_CAMERA_VECTORS\x10\xd1\x01\x12\x1c\n\x17\x44\x45\x42UG_LEFT_CAMERA_EDGES\x10\xd2\x01\x12\x1f\n\x1a\x44\x45\x42UG_LEFT_CAMERA_CLEANING\x10\xd3\x01\x12\x1e\n\x19\x44\x45\x42UG_RIGHT_CAMERA_EXTERN\x10\xd4\x01\x12\x1f\n\x1a\x44\x45\x42UG_RIGHT_CAMERA_VECTORS\x10\xd5\x01\x12\x1d\n\x18\x44\x45\x42UG_RIGHT_CAMERA_EDGES\x10\xd6\x01\x12 \n\x1b\x44\x45\x42UG_RIGHT_CAMERA_CLEANING\x10\xd7\x01\x12%\n DEBUG_CALIBRATION_FLMC_QUAD_VIEW\x10\xd8\x01\x12$\n\x1f\x44\x45\x42UG_CALIBRATION_PGM_QUAD_VIEW\x10\xd9\x01\x12\x1c\n\x17\x44\x45\x42UG_TESTSCREEN_4_CAMS\x10\xda\x01\x12%\n DEBUG_CALIBRATION_OLMC_QUAD_VIEW\x10\xdb\x01\x12\x16\n\x11\x44\x45\x42UG_COLOUR_BARS\x10\xdc\x01\x12\x16\n\x11\x44\x45\x42UG_RAW_CAMERAS\x10\xdd\x01\x12\x1a\n\x15\x44\x45\x42UG_RAW_CAMERAS_SFM\x10\xde\x01\x12\x19\n\x14\x44\x45\x42UG_CURB_MESH_DEMO\x10\xdf\x01\x12\x15\n\x10\x44\x45\x42UG_HEIGHT_MAP\x10\xe0\x01\x12\x1b\n\x16\x44\x45\x42UG_PGM_QUAD_OVERLAY\x10\xe1\x01\x12\x1e\n\x19\x44\x45\x42UG_IMAGE_HARMONIZATION\x10\xe2\x01\x12#\n\x1ePARKING_IN_SLOT_SELECTION_VIEW\x10\xe3\x01\x12&\n!PARKING_IN_SLOT_CONFIRMATION_VIEW\x10\xe4\x01\x12+\n&PARKING_IN_MANEUVER_PHASE_FORWARD_VIEW\x10\xe5\x01\x12,\n\'PARKING_IN_MANEUVER_PHASE_BACKWARD_VIEW\x10\xe6\x01\x12,\n\'PARKING_IN_MANEUVER_APPROACH_PHASE_VIEW\x10\xe7\x01\x12\x41\n<PARKING_IN_MANEUVER_APPROACH_WHEELSTOPPER_PHASE_FORWARD_VIEW\x10\xe8\x01\x12\x42\n=PARKING_IN_MANEUVER_APPROACH_WHEELSTOPPER_PHASE_BACKWARD_VIEW\x10\xe9\x01\x12)\n$PARKING_OUT_DIRECTION_SELECTION_VIEW\x10\xea\x01\x12&\n!PARKING_OUT_MANEUVER_FORWARD_VIEW\x10\xeb\x01\x12\'\n\"PARKING_OUT_MANEUVER_BACKWARD_VIEW\x10\xec\x01\x12+\n&PARKING_OUT_MANEUVER_FINISH_PHASE_VIEW\x10\xed\x01\x12\x15\n\x10NO_STREAM_CHANGE\x10\xff\x01\x12\t\n\x04SIZE\x10\x80\x02'
)

_SCREENTYPES = _descriptor.EnumDescriptor(
  name='ScreenTypes',
  full_name='pb.ap_hmitoap.screen_types.ScreenTypes',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='BLANK_SCREEN', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_1', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_2', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_3', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_4', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_5', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_6', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_7', index=7, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_8', index=8, number=8,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_9', index=9, number=9,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_10', index=10, number=10,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_11', index=11, number=11,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_12', index=12, number=12,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_13', index=13, number=13,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_14', index=14, number=14,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BOWL_VIEW_MODIFIED', index=15, number=15,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FRONT_VIEW_FULL', index=16, number=16,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FRONT_VIEW_SPLIT', index=17, number=17,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PANORAMIC_FRONT_VIEW', index=18, number=18,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BLACK_SCREEN', index=19, number=19,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FRONT_TOP_ZOOM', index=20, number=20,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LAST_FRONT_VIEW', index=21, number=30,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TOP_VIEW', index=22, number=31,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TOP_VIEW_SFM', index=23, number=32,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TOP_VIEW_PSD', index=24, number=33,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TOP_VIEW_PMD', index=25, number=34,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LAST_TOP_VIEW', index=26, number=45,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REAR_VIEW_FULL', index=27, number=46,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REAR_VIEW_SPLIT', index=28, number=47,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PANORAMIC_REAR_VIEW', index=29, number=48,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TRAILER_VIEW', index=30, number=49,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REAR_TOP_ZOOM', index=31, number=50,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LAST_REAR_VIEW', index=32, number=60,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='KERB_VIEW_FRONT', index=33, number=61,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='KERB_VIEW_REAR', index=34, number=62,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LAST_KERB_VIEW', index=35, number=75,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ARA_REAR_NORMAL', index=36, number=76,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ARA_REAR_IRREGULAR', index=37, number=77,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ARA_PANORAMIC_NORMAL', index=38, number=78,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ARA_PANORAMIC_IRREGULAR', index=39, number=79,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LEFT_VIEW_SPLIT', index=40, number=80,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LEFT_VIEW_FULL', index=41, number=81,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PANORAMIC_LEFT_VIEW', index=42, number=82,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RIGHT_VIEW_SPLIT', index=43, number=83,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RIGHT_VIEW_FULL', index=44, number=84,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PANORAMIC_RIGHT_VIEW', index=45, number=85,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LAST_ARA_VIEW', index=46, number=90,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IPA_FRONT_VIEW', index=47, number=91,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IPA_REAR_VIEW', index=48, number=92,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IPA_TOP_VIEW', index=49, number=93,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_FRONT_VIEW', index=50, number=94,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_REAR_VIEW', index=51, number=95,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_TOP_VIEW', index=52, number=96,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_SPACES_TOP_VIEW', index=53, number=97,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_WARN_TOP_VIEW', index=54, number=98,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARK_TRANSPARENT_PDW', index=55, number=99,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LAST_IPA_VIEW', index=56, number=105,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MORPH_FRONT', index=57, number=106,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MORPH_FRONT_FULL', index=58, number=107,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LAST_MORPH_FRONT', index=59, number=120,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MORPH_REAR', index=60, number=121,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MORPH_REAR_FULL', index=61, number=122,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MORPH_ARA', index=62, number=123,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LAST_MORPH_VIEW', index=63, number=135,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SMARTPHONE_VIEW', index=64, number=151,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BLIND_SPOT_LEFT', index=65, number=152,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BLIND_SPOT_RIGHT', index=66, number=153,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LAST_SPECIAL_VIEW', index=67, number=165,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TRANSPARENT_HOOD', index=68, number=166,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TRANSPARENT_ALPHA', index=69, number=167,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TRANSPARENT_GHOST', index=70, number=168,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TRANSPARENT_TRUNK', index=71, number=169,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TRANSPARENT_ALPHA_SPLIT', index=72, number=170,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TRANSPARENT_HOOD_SPLIT', index=73, number=171,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TRANSPARENT_TRUNK_SPLIT', index=74, number=172,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ADAPTIVE_FULL_BOWL', index=75, number=173,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STATIC_BOWL', index=76, number=174,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ADAPTIVE_SPLIT_BOWL', index=77, number=175,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='STATIC_SPLIT_BOWL', index=78, number=176,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RAW_CAMERAS', index=79, number=181,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RAW_CAMERAS_SFM', index=80, number=182,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_FRONT_CAMERA_EXTERN', index=81, number=200,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_FRONT_CAMERA_VECTORS', index=82, number=201,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_FRONT_CAMERA_EDGES', index=83, number=202,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_FRONT_CAMERA_CLEANING', index=84, number=203,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_REAR_CAMERA_EXTERN', index=85, number=204,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_REAR_CAMERA_VECTORS', index=86, number=205,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_REAR_CAMERA_EDGES', index=87, number=206,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_REAR_CAMERA_CLEANING', index=88, number=207,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_LEFT_CAMERA_EXTERN', index=89, number=208,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_LEFT_CAMERA_VECTORS', index=90, number=209,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_LEFT_CAMERA_EDGES', index=91, number=210,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_LEFT_CAMERA_CLEANING', index=92, number=211,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_RIGHT_CAMERA_EXTERN', index=93, number=212,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_RIGHT_CAMERA_VECTORS', index=94, number=213,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_RIGHT_CAMERA_EDGES', index=95, number=214,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_RIGHT_CAMERA_CLEANING', index=96, number=215,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_CALIBRATION_FLMC_QUAD_VIEW', index=97, number=216,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_CALIBRATION_PGM_QUAD_VIEW', index=98, number=217,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_TESTSCREEN_4_CAMS', index=99, number=218,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_CALIBRATION_OLMC_QUAD_VIEW', index=100, number=219,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_COLOUR_BARS', index=101, number=220,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_RAW_CAMERAS', index=102, number=221,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_RAW_CAMERAS_SFM', index=103, number=222,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_CURB_MESH_DEMO', index=104, number=223,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_HEIGHT_MAP', index=105, number=224,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_PGM_QUAD_OVERLAY', index=106, number=225,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEBUG_IMAGE_HARMONIZATION', index=107, number=226,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARKING_IN_SLOT_SELECTION_VIEW', index=108, number=227,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARKING_IN_SLOT_CONFIRMATION_VIEW', index=109, number=228,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARKING_IN_MANEUVER_PHASE_FORWARD_VIEW', index=110, number=229,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARKING_IN_MANEUVER_PHASE_BACKWARD_VIEW', index=111, number=230,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARKING_IN_MANEUVER_APPROACH_PHASE_VIEW', index=112, number=231,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARKING_IN_MANEUVER_APPROACH_WHEELSTOPPER_PHASE_FORWARD_VIEW', index=113, number=232,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARKING_IN_MANEUVER_APPROACH_WHEELSTOPPER_PHASE_BACKWARD_VIEW', index=114, number=233,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARKING_OUT_DIRECTION_SELECTION_VIEW', index=115, number=234,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARKING_OUT_MANEUVER_FORWARD_VIEW', index=116, number=235,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARKING_OUT_MANEUVER_BACKWARD_VIEW', index=117, number=236,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PARKING_OUT_MANEUVER_FINISH_PHASE_VIEW', index=118, number=237,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NO_STREAM_CHANGE', index=119, number=255,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SIZE', index=120, number=256,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=62,
  serialized_end=3165,
)
_sym_db.RegisterEnumDescriptor(_SCREENTYPES)

ScreenTypes = enum_type_wrapper.EnumTypeWrapper(_SCREENTYPES)
BLANK_SCREEN = 0
BOWL_VIEW_1 = 1
BOWL_VIEW_2 = 2
BOWL_VIEW_3 = 3
BOWL_VIEW_4 = 4
BOWL_VIEW_5 = 5
BOWL_VIEW_6 = 6
BOWL_VIEW_7 = 7
BOWL_VIEW_8 = 8
BOWL_VIEW_9 = 9
BOWL_VIEW_10 = 10
BOWL_VIEW_11 = 11
BOWL_VIEW_12 = 12
BOWL_VIEW_13 = 13
BOWL_VIEW_14 = 14
BOWL_VIEW_MODIFIED = 15
FRONT_VIEW_FULL = 16
FRONT_VIEW_SPLIT = 17
PANORAMIC_FRONT_VIEW = 18
BLACK_SCREEN = 19
FRONT_TOP_ZOOM = 20
LAST_FRONT_VIEW = 30
TOP_VIEW = 31
TOP_VIEW_SFM = 32
TOP_VIEW_PSD = 33
TOP_VIEW_PMD = 34
LAST_TOP_VIEW = 45
REAR_VIEW_FULL = 46
REAR_VIEW_SPLIT = 47
PANORAMIC_REAR_VIEW = 48
TRAILER_VIEW = 49
REAR_TOP_ZOOM = 50
LAST_REAR_VIEW = 60
KERB_VIEW_FRONT = 61
KERB_VIEW_REAR = 62
LAST_KERB_VIEW = 75
ARA_REAR_NORMAL = 76
ARA_REAR_IRREGULAR = 77
ARA_PANORAMIC_NORMAL = 78
ARA_PANORAMIC_IRREGULAR = 79
LEFT_VIEW_SPLIT = 80
LEFT_VIEW_FULL = 81
PANORAMIC_LEFT_VIEW = 82
RIGHT_VIEW_SPLIT = 83
RIGHT_VIEW_FULL = 84
PANORAMIC_RIGHT_VIEW = 85
LAST_ARA_VIEW = 90
IPA_FRONT_VIEW = 91
IPA_REAR_VIEW = 92
IPA_TOP_VIEW = 93
PARK_FRONT_VIEW = 94
PARK_REAR_VIEW = 95
PARK_TOP_VIEW = 96
PARK_SPACES_TOP_VIEW = 97
PARK_WARN_TOP_VIEW = 98
PARK_TRANSPARENT_PDW = 99
LAST_IPA_VIEW = 105
MORPH_FRONT = 106
MORPH_FRONT_FULL = 107
LAST_MORPH_FRONT = 120
MORPH_REAR = 121
MORPH_REAR_FULL = 122
MORPH_ARA = 123
LAST_MORPH_VIEW = 135
SMARTPHONE_VIEW = 151
BLIND_SPOT_LEFT = 152
BLIND_SPOT_RIGHT = 153
LAST_SPECIAL_VIEW = 165
TRANSPARENT_HOOD = 166
TRANSPARENT_ALPHA = 167
TRANSPARENT_GHOST = 168
TRANSPARENT_TRUNK = 169
TRANSPARENT_ALPHA_SPLIT = 170
TRANSPARENT_HOOD_SPLIT = 171
TRANSPARENT_TRUNK_SPLIT = 172
ADAPTIVE_FULL_BOWL = 173
STATIC_BOWL = 174
ADAPTIVE_SPLIT_BOWL = 175
STATIC_SPLIT_BOWL = 176
RAW_CAMERAS = 181
RAW_CAMERAS_SFM = 182
DEBUG_FRONT_CAMERA_EXTERN = 200
DEBUG_FRONT_CAMERA_VECTORS = 201
DEBUG_FRONT_CAMERA_EDGES = 202
DEBUG_FRONT_CAMERA_CLEANING = 203
DEBUG_REAR_CAMERA_EXTERN = 204
DEBUG_REAR_CAMERA_VECTORS = 205
DEBUG_REAR_CAMERA_EDGES = 206
DEBUG_REAR_CAMERA_CLEANING = 207
DEBUG_LEFT_CAMERA_EXTERN = 208
DEBUG_LEFT_CAMERA_VECTORS = 209
DEBUG_LEFT_CAMERA_EDGES = 210
DEBUG_LEFT_CAMERA_CLEANING = 211
DEBUG_RIGHT_CAMERA_EXTERN = 212
DEBUG_RIGHT_CAMERA_VECTORS = 213
DEBUG_RIGHT_CAMERA_EDGES = 214
DEBUG_RIGHT_CAMERA_CLEANING = 215
DEBUG_CALIBRATION_FLMC_QUAD_VIEW = 216
DEBUG_CALIBRATION_PGM_QUAD_VIEW = 217
DEBUG_TESTSCREEN_4_CAMS = 218
DEBUG_CALIBRATION_OLMC_QUAD_VIEW = 219
DEBUG_COLOUR_BARS = 220
DEBUG_RAW_CAMERAS = 221
DEBUG_RAW_CAMERAS_SFM = 222
DEBUG_CURB_MESH_DEMO = 223
DEBUG_HEIGHT_MAP = 224
DEBUG_PGM_QUAD_OVERLAY = 225
DEBUG_IMAGE_HARMONIZATION = 226
PARKING_IN_SLOT_SELECTION_VIEW = 227
PARKING_IN_SLOT_CONFIRMATION_VIEW = 228
PARKING_IN_MANEUVER_PHASE_FORWARD_VIEW = 229
PARKING_IN_MANEUVER_PHASE_BACKWARD_VIEW = 230
PARKING_IN_MANEUVER_APPROACH_PHASE_VIEW = 231
PARKING_IN_MANEUVER_APPROACH_WHEELSTOPPER_PHASE_FORWARD_VIEW = 232
PARKING_IN_MANEUVER_APPROACH_WHEELSTOPPER_PHASE_BACKWARD_VIEW = 233
PARKING_OUT_DIRECTION_SELECTION_VIEW = 234
PARKING_OUT_MANEUVER_FORWARD_VIEW = 235
PARKING_OUT_MANEUVER_BACKWARD_VIEW = 236
PARKING_OUT_MANEUVER_FINISH_PHASE_VIEW = 237
NO_STREAM_CHANGE = 255
SIZE = 256


DESCRIPTOR.enum_types_by_name['ScreenTypes'] = _SCREENTYPES
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)