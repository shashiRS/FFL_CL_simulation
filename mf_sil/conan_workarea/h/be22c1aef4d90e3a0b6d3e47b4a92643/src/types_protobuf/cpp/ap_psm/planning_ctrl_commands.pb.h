// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_psm/planning_ctrl_commands.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_2fplanning_5fctrl_5fcommands_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_2fplanning_5fctrl_5fcommands_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3011000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3011004 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "ap_psm/apstate.pb.h"
#include "ap_psm/applanning_specification.pb.h"
#include "ap_psm/rmstate.pb.h"
#include "ap_psm/gpstate.pb.h"
#include "ap_psm/mpstate.pb.h"
#include "ap_psm/tpstate.pb.h"
#include "ap_psm/rastate.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5fpsm_2fplanning_5fctrl_5fcommands_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fpsm_2fplanning_5fctrl_5fcommands_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[2]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fpsm_2fplanning_5fctrl_5fcommands_2eproto;
namespace pb {
namespace ap_psm {
namespace planning_ctrl_commands {
class PlanningCtrlCommands;
class PlanningCtrlCommandsDefaultTypeInternal;
extern PlanningCtrlCommandsDefaultTypeInternal _PlanningCtrlCommands_default_instance_;
class PlanningCtrlCommands_array_port;
class PlanningCtrlCommands_array_portDefaultTypeInternal;
extern PlanningCtrlCommands_array_portDefaultTypeInternal _PlanningCtrlCommands_array_port_default_instance_;
}  // namespace planning_ctrl_commands
}  // namespace ap_psm
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands* Arena::CreateMaybeMessage<::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands>(Arena*);
template<> ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands_array_port* Arena::CreateMaybeMessage<::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_psm {
namespace planning_ctrl_commands {

// ===================================================================

class PlanningCtrlCommands :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands) */ {
 public:
  PlanningCtrlCommands();
  virtual ~PlanningCtrlCommands();

  PlanningCtrlCommands(const PlanningCtrlCommands& from);
  PlanningCtrlCommands(PlanningCtrlCommands&& from) noexcept
    : PlanningCtrlCommands() {
    *this = ::std::move(from);
  }

  inline PlanningCtrlCommands& operator=(const PlanningCtrlCommands& from) {
    CopyFrom(from);
    return *this;
  }
  inline PlanningCtrlCommands& operator=(PlanningCtrlCommands&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const PlanningCtrlCommands& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PlanningCtrlCommands* internal_default_instance() {
    return reinterpret_cast<const PlanningCtrlCommands*>(
               &_PlanningCtrlCommands_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PlanningCtrlCommands& a, PlanningCtrlCommands& b) {
    a.Swap(&b);
  }
  inline void Swap(PlanningCtrlCommands* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PlanningCtrlCommands* New() const final {
    return CreateMaybeMessage<PlanningCtrlCommands>(nullptr);
  }

  PlanningCtrlCommands* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PlanningCtrlCommands>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PlanningCtrlCommands& from);
  void MergeFrom(const PlanningCtrlCommands& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(PlanningCtrlCommands* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands";
  }
  private:
  inline ::PROTOBUF_NAMESPACE_ID::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fpsm_2fplanning_5fctrl_5fcommands_2eproto);
    return ::descriptor_table_ap_5fpsm_2fplanning_5fctrl_5fcommands_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kApChosenTargetPoseIdNuFieldNumber = 706,
    kTpStateFieldNumber = 821,
    kMpStateFieldNumber = 1527,
    kApStateFieldNumber = 1885,
    kRaStateFieldNumber = 2124,
    kApPlanningSpecificationFieldNumber = 2631,
    kRmStateFieldNumber = 3203,
    kGpStateFieldNumber = 3592,
  };
  // optional uint32 apChosenTargetPoseId_nu = 706;
  bool has_apchosentargetposeid_nu() const;
  private:
  bool _internal_has_apchosentargetposeid_nu() const;
  public:
  void clear_apchosentargetposeid_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 apchosentargetposeid_nu() const;
  void set_apchosentargetposeid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_apchosentargetposeid_nu() const;
  void _internal_set_apchosentargetposeid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional .pb.ap_psm.tpstate.TPState tpState = 821;
  bool has_tpstate() const;
  private:
  bool _internal_has_tpstate() const;
  public:
  void clear_tpstate();
  ::pb::ap_psm::tpstate::TPState tpstate() const;
  void set_tpstate(::pb::ap_psm::tpstate::TPState value);
  private:
  ::pb::ap_psm::tpstate::TPState _internal_tpstate() const;
  void _internal_set_tpstate(::pb::ap_psm::tpstate::TPState value);
  public:

  // optional .pb.ap_psm.mpstate.MPState mpState = 1527;
  bool has_mpstate() const;
  private:
  bool _internal_has_mpstate() const;
  public:
  void clear_mpstate();
  ::pb::ap_psm::mpstate::MPState mpstate() const;
  void set_mpstate(::pb::ap_psm::mpstate::MPState value);
  private:
  ::pb::ap_psm::mpstate::MPState _internal_mpstate() const;
  void _internal_set_mpstate(::pb::ap_psm::mpstate::MPState value);
  public:

  // optional .pb.ap_psm.apstate.APState apState = 1885;
  bool has_apstate() const;
  private:
  bool _internal_has_apstate() const;
  public:
  void clear_apstate();
  ::pb::ap_psm::apstate::APState apstate() const;
  void set_apstate(::pb::ap_psm::apstate::APState value);
  private:
  ::pb::ap_psm::apstate::APState _internal_apstate() const;
  void _internal_set_apstate(::pb::ap_psm::apstate::APState value);
  public:

  // optional .pb.ap_psm.rastate.RAState raState = 2124;
  bool has_rastate() const;
  private:
  bool _internal_has_rastate() const;
  public:
  void clear_rastate();
  ::pb::ap_psm::rastate::RAState rastate() const;
  void set_rastate(::pb::ap_psm::rastate::RAState value);
  private:
  ::pb::ap_psm::rastate::RAState _internal_rastate() const;
  void _internal_set_rastate(::pb::ap_psm::rastate::RAState value);
  public:

  // optional .pb.ap_psm.applanning_specification.APPlanningSpecification apPlanningSpecification = 2631;
  bool has_applanningspecification() const;
  private:
  bool _internal_has_applanningspecification() const;
  public:
  void clear_applanningspecification();
  ::pb::ap_psm::applanning_specification::APPlanningSpecification applanningspecification() const;
  void set_applanningspecification(::pb::ap_psm::applanning_specification::APPlanningSpecification value);
  private:
  ::pb::ap_psm::applanning_specification::APPlanningSpecification _internal_applanningspecification() const;
  void _internal_set_applanningspecification(::pb::ap_psm::applanning_specification::APPlanningSpecification value);
  public:

  // optional .pb.ap_psm.rmstate.RMState rmState = 3203;
  bool has_rmstate() const;
  private:
  bool _internal_has_rmstate() const;
  public:
  void clear_rmstate();
  ::pb::ap_psm::rmstate::RMState rmstate() const;
  void set_rmstate(::pb::ap_psm::rmstate::RMState value);
  private:
  ::pb::ap_psm::rmstate::RMState _internal_rmstate() const;
  void _internal_set_rmstate(::pb::ap_psm::rmstate::RMState value);
  public:

  // optional .pb.ap_psm.gpstate.GPState gpState = 3592;
  bool has_gpstate() const;
  private:
  bool _internal_has_gpstate() const;
  public:
  void clear_gpstate();
  ::pb::ap_psm::gpstate::GPState gpstate() const;
  void set_gpstate(::pb::ap_psm::gpstate::GPState value);
  private:
  ::pb::ap_psm::gpstate::GPState _internal_gpstate() const;
  void _internal_set_gpstate(::pb::ap_psm::gpstate::GPState value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 apchosentargetposeid_nu_;
  int tpstate_;
  int mpstate_;
  int apstate_;
  int rastate_;
  int applanningspecification_;
  int rmstate_;
  int gpstate_;
  friend struct ::TableStruct_ap_5fpsm_2fplanning_5fctrl_5fcommands_2eproto;
};
// -------------------------------------------------------------------

class PlanningCtrlCommands_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands_array_port) */ {
 public:
  PlanningCtrlCommands_array_port();
  virtual ~PlanningCtrlCommands_array_port();

  PlanningCtrlCommands_array_port(const PlanningCtrlCommands_array_port& from);
  PlanningCtrlCommands_array_port(PlanningCtrlCommands_array_port&& from) noexcept
    : PlanningCtrlCommands_array_port() {
    *this = ::std::move(from);
  }

  inline PlanningCtrlCommands_array_port& operator=(const PlanningCtrlCommands_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline PlanningCtrlCommands_array_port& operator=(PlanningCtrlCommands_array_port&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const PlanningCtrlCommands_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PlanningCtrlCommands_array_port* internal_default_instance() {
    return reinterpret_cast<const PlanningCtrlCommands_array_port*>(
               &_PlanningCtrlCommands_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(PlanningCtrlCommands_array_port& a, PlanningCtrlCommands_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(PlanningCtrlCommands_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PlanningCtrlCommands_array_port* New() const final {
    return CreateMaybeMessage<PlanningCtrlCommands_array_port>(nullptr);
  }

  PlanningCtrlCommands_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PlanningCtrlCommands_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PlanningCtrlCommands_array_port& from);
  void MergeFrom(const PlanningCtrlCommands_array_port& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(PlanningCtrlCommands_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands_array_port";
  }
  private:
  inline ::PROTOBUF_NAMESPACE_ID::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fpsm_2fplanning_5fctrl_5fcommands_2eproto);
    return ::descriptor_table_ap_5fpsm_2fplanning_5fctrl_5fcommands_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1528,
  };
  // repeated .pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands data = 1528;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands >*
      mutable_data();
  private:
  const ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands& _internal_data(int index) const;
  ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands* _internal_add_data();
  public:
  const ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands& data(int index) const;
  ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands > data_;
  friend struct ::TableStruct_ap_5fpsm_2fplanning_5fctrl_5fcommands_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PlanningCtrlCommands

// optional uint32 apChosenTargetPoseId_nu = 706;
inline bool PlanningCtrlCommands::_internal_has_apchosentargetposeid_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool PlanningCtrlCommands::has_apchosentargetposeid_nu() const {
  return _internal_has_apchosentargetposeid_nu();
}
inline void PlanningCtrlCommands::clear_apchosentargetposeid_nu() {
  apchosentargetposeid_nu_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PlanningCtrlCommands::_internal_apchosentargetposeid_nu() const {
  return apchosentargetposeid_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PlanningCtrlCommands::apchosentargetposeid_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.apChosenTargetPoseId_nu)
  return _internal_apchosentargetposeid_nu();
}
inline void PlanningCtrlCommands::_internal_set_apchosentargetposeid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  apchosentargetposeid_nu_ = value;
}
inline void PlanningCtrlCommands::set_apchosentargetposeid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_apchosentargetposeid_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.apChosenTargetPoseId_nu)
}

// optional .pb.ap_psm.apstate.APState apState = 1885;
inline bool PlanningCtrlCommands::_internal_has_apstate() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool PlanningCtrlCommands::has_apstate() const {
  return _internal_has_apstate();
}
inline void PlanningCtrlCommands::clear_apstate() {
  apstate_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::pb::ap_psm::apstate::APState PlanningCtrlCommands::_internal_apstate() const {
  return static_cast< ::pb::ap_psm::apstate::APState >(apstate_);
}
inline ::pb::ap_psm::apstate::APState PlanningCtrlCommands::apstate() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.apState)
  return _internal_apstate();
}
inline void PlanningCtrlCommands::_internal_set_apstate(::pb::ap_psm::apstate::APState value) {
  assert(::pb::ap_psm::apstate::APState_IsValid(value));
  _has_bits_[0] |= 0x00000008u;
  apstate_ = value;
}
inline void PlanningCtrlCommands::set_apstate(::pb::ap_psm::apstate::APState value) {
  _internal_set_apstate(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.apState)
}

// optional .pb.ap_psm.applanning_specification.APPlanningSpecification apPlanningSpecification = 2631;
inline bool PlanningCtrlCommands::_internal_has_applanningspecification() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool PlanningCtrlCommands::has_applanningspecification() const {
  return _internal_has_applanningspecification();
}
inline void PlanningCtrlCommands::clear_applanningspecification() {
  applanningspecification_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::pb::ap_psm::applanning_specification::APPlanningSpecification PlanningCtrlCommands::_internal_applanningspecification() const {
  return static_cast< ::pb::ap_psm::applanning_specification::APPlanningSpecification >(applanningspecification_);
}
inline ::pb::ap_psm::applanning_specification::APPlanningSpecification PlanningCtrlCommands::applanningspecification() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.apPlanningSpecification)
  return _internal_applanningspecification();
}
inline void PlanningCtrlCommands::_internal_set_applanningspecification(::pb::ap_psm::applanning_specification::APPlanningSpecification value) {
  assert(::pb::ap_psm::applanning_specification::APPlanningSpecification_IsValid(value));
  _has_bits_[0] |= 0x00000020u;
  applanningspecification_ = value;
}
inline void PlanningCtrlCommands::set_applanningspecification(::pb::ap_psm::applanning_specification::APPlanningSpecification value) {
  _internal_set_applanningspecification(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.apPlanningSpecification)
}

// optional .pb.ap_psm.rmstate.RMState rmState = 3203;
inline bool PlanningCtrlCommands::_internal_has_rmstate() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool PlanningCtrlCommands::has_rmstate() const {
  return _internal_has_rmstate();
}
inline void PlanningCtrlCommands::clear_rmstate() {
  rmstate_ = 0;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::pb::ap_psm::rmstate::RMState PlanningCtrlCommands::_internal_rmstate() const {
  return static_cast< ::pb::ap_psm::rmstate::RMState >(rmstate_);
}
inline ::pb::ap_psm::rmstate::RMState PlanningCtrlCommands::rmstate() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.rmState)
  return _internal_rmstate();
}
inline void PlanningCtrlCommands::_internal_set_rmstate(::pb::ap_psm::rmstate::RMState value) {
  assert(::pb::ap_psm::rmstate::RMState_IsValid(value));
  _has_bits_[0] |= 0x00000040u;
  rmstate_ = value;
}
inline void PlanningCtrlCommands::set_rmstate(::pb::ap_psm::rmstate::RMState value) {
  _internal_set_rmstate(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.rmState)
}

// optional .pb.ap_psm.gpstate.GPState gpState = 3592;
inline bool PlanningCtrlCommands::_internal_has_gpstate() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool PlanningCtrlCommands::has_gpstate() const {
  return _internal_has_gpstate();
}
inline void PlanningCtrlCommands::clear_gpstate() {
  gpstate_ = 0;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::pb::ap_psm::gpstate::GPState PlanningCtrlCommands::_internal_gpstate() const {
  return static_cast< ::pb::ap_psm::gpstate::GPState >(gpstate_);
}
inline ::pb::ap_psm::gpstate::GPState PlanningCtrlCommands::gpstate() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.gpState)
  return _internal_gpstate();
}
inline void PlanningCtrlCommands::_internal_set_gpstate(::pb::ap_psm::gpstate::GPState value) {
  assert(::pb::ap_psm::gpstate::GPState_IsValid(value));
  _has_bits_[0] |= 0x00000080u;
  gpstate_ = value;
}
inline void PlanningCtrlCommands::set_gpstate(::pb::ap_psm::gpstate::GPState value) {
  _internal_set_gpstate(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.gpState)
}

// optional .pb.ap_psm.mpstate.MPState mpState = 1527;
inline bool PlanningCtrlCommands::_internal_has_mpstate() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool PlanningCtrlCommands::has_mpstate() const {
  return _internal_has_mpstate();
}
inline void PlanningCtrlCommands::clear_mpstate() {
  mpstate_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::ap_psm::mpstate::MPState PlanningCtrlCommands::_internal_mpstate() const {
  return static_cast< ::pb::ap_psm::mpstate::MPState >(mpstate_);
}
inline ::pb::ap_psm::mpstate::MPState PlanningCtrlCommands::mpstate() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.mpState)
  return _internal_mpstate();
}
inline void PlanningCtrlCommands::_internal_set_mpstate(::pb::ap_psm::mpstate::MPState value) {
  assert(::pb::ap_psm::mpstate::MPState_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  mpstate_ = value;
}
inline void PlanningCtrlCommands::set_mpstate(::pb::ap_psm::mpstate::MPState value) {
  _internal_set_mpstate(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.mpState)
}

// optional .pb.ap_psm.tpstate.TPState tpState = 821;
inline bool PlanningCtrlCommands::_internal_has_tpstate() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool PlanningCtrlCommands::has_tpstate() const {
  return _internal_has_tpstate();
}
inline void PlanningCtrlCommands::clear_tpstate() {
  tpstate_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::ap_psm::tpstate::TPState PlanningCtrlCommands::_internal_tpstate() const {
  return static_cast< ::pb::ap_psm::tpstate::TPState >(tpstate_);
}
inline ::pb::ap_psm::tpstate::TPState PlanningCtrlCommands::tpstate() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.tpState)
  return _internal_tpstate();
}
inline void PlanningCtrlCommands::_internal_set_tpstate(::pb::ap_psm::tpstate::TPState value) {
  assert(::pb::ap_psm::tpstate::TPState_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  tpstate_ = value;
}
inline void PlanningCtrlCommands::set_tpstate(::pb::ap_psm::tpstate::TPState value) {
  _internal_set_tpstate(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.tpState)
}

// optional .pb.ap_psm.rastate.RAState raState = 2124;
inline bool PlanningCtrlCommands::_internal_has_rastate() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool PlanningCtrlCommands::has_rastate() const {
  return _internal_has_rastate();
}
inline void PlanningCtrlCommands::clear_rastate() {
  rastate_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::pb::ap_psm::rastate::RAState PlanningCtrlCommands::_internal_rastate() const {
  return static_cast< ::pb::ap_psm::rastate::RAState >(rastate_);
}
inline ::pb::ap_psm::rastate::RAState PlanningCtrlCommands::rastate() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.raState)
  return _internal_rastate();
}
inline void PlanningCtrlCommands::_internal_set_rastate(::pb::ap_psm::rastate::RAState value) {
  assert(::pb::ap_psm::rastate::RAState_IsValid(value));
  _has_bits_[0] |= 0x00000010u;
  rastate_ = value;
}
inline void PlanningCtrlCommands::set_rastate(::pb::ap_psm::rastate::RAState value) {
  _internal_set_rastate(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands.raState)
}

// -------------------------------------------------------------------

// PlanningCtrlCommands_array_port

// repeated .pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands data = 1528;
inline int PlanningCtrlCommands_array_port::_internal_data_size() const {
  return data_.size();
}
inline int PlanningCtrlCommands_array_port::data_size() const {
  return _internal_data_size();
}
inline void PlanningCtrlCommands_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands* PlanningCtrlCommands_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands >*
PlanningCtrlCommands_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands_array_port.data)
  return &data_;
}
inline const ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands& PlanningCtrlCommands_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands& PlanningCtrlCommands_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands* PlanningCtrlCommands_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands* PlanningCtrlCommands_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm::planning_ctrl_commands::PlanningCtrlCommands >&
PlanningCtrlCommands_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_psm.planning_ctrl_commands.PlanningCtrlCommands_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace planning_ctrl_commands
}  // namespace ap_psm
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_2fplanning_5fctrl_5fcommands_2eproto
