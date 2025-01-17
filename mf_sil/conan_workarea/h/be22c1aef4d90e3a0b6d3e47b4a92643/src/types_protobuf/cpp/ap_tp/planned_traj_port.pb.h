// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_tp/planned_traj_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5ftp_2fplanned_5ftraj_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5ftp_2fplanned_5ftraj_5fport_2eproto

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
#include "eco/signal_header.pb.h"
#include "ap_tp/planned_traj_type.pb.h"
#include "ap_tp/driving_resistance.pb.h"
#include "ap_tp/planned_traj.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5ftp_2fplanned_5ftraj_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5ftp_2fplanned_5ftraj_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftp_2fplanned_5ftraj_5fport_2eproto;
namespace pb {
namespace ap_tp {
namespace planned_traj_port {
class PlannedTrajPort;
class PlannedTrajPortDefaultTypeInternal;
extern PlannedTrajPortDefaultTypeInternal _PlannedTrajPort_default_instance_;
class PlannedTrajPort_array_port;
class PlannedTrajPort_array_portDefaultTypeInternal;
extern PlannedTrajPort_array_portDefaultTypeInternal _PlannedTrajPort_array_port_default_instance_;
}  // namespace planned_traj_port
}  // namespace ap_tp
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_tp::planned_traj_port::PlannedTrajPort* Arena::CreateMaybeMessage<::pb::ap_tp::planned_traj_port::PlannedTrajPort>(Arena*);
template<> ::pb::ap_tp::planned_traj_port::PlannedTrajPort_array_port* Arena::CreateMaybeMessage<::pb::ap_tp::planned_traj_port::PlannedTrajPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_tp {
namespace planned_traj_port {

// ===================================================================

class PlannedTrajPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_tp.planned_traj_port.PlannedTrajPort) */ {
 public:
  PlannedTrajPort();
  virtual ~PlannedTrajPort();

  PlannedTrajPort(const PlannedTrajPort& from);
  PlannedTrajPort(PlannedTrajPort&& from) noexcept
    : PlannedTrajPort() {
    *this = ::std::move(from);
  }

  inline PlannedTrajPort& operator=(const PlannedTrajPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline PlannedTrajPort& operator=(PlannedTrajPort&& from) noexcept {
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
  static const PlannedTrajPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PlannedTrajPort* internal_default_instance() {
    return reinterpret_cast<const PlannedTrajPort*>(
               &_PlannedTrajPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PlannedTrajPort& a, PlannedTrajPort& b) {
    a.Swap(&b);
  }
  inline void Swap(PlannedTrajPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PlannedTrajPort* New() const final {
    return CreateMaybeMessage<PlannedTrajPort>(nullptr);
  }

  PlannedTrajPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PlannedTrajPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PlannedTrajPort& from);
  void MergeFrom(const PlannedTrajPort& from);
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
  void InternalSwap(PlannedTrajPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_tp.planned_traj_port.PlannedTrajPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftp_2fplanned_5ftraj_5fport_2eproto);
    return ::descriptor_table_ap_5ftp_2fplanned_5ftraj_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDrivingResistanceFieldNumber = 2580,
    kPlannedTrajFieldNumber = 3662,
    kSSigHeaderFieldNumber = 1033,
    kUiVersionNumberFieldNumber = 2124,
    kNumValidCtrlPointsNuFieldNumber = 850,
    kStepInTrajAfterIdxNuFieldNumber = 1359,
    kDrivingForwardReqNuFieldNumber = 3147,
    kTrajValidNuFieldNumber = 125,
    kNewSegmentStartedNuFieldNumber = 1375,
    kIsLastSegmentNuFieldNumber = 1592,
    kTrajTypeNuFieldNumber = 1778,
  };
  // repeated .pb.ap_tp.driving_resistance.DrivingResistance drivingResistance = 2580;
  int drivingresistance_size() const;
  private:
  int _internal_drivingresistance_size() const;
  public:
  void clear_drivingresistance();
  ::pb::ap_tp::driving_resistance::DrivingResistance* mutable_drivingresistance(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::driving_resistance::DrivingResistance >*
      mutable_drivingresistance();
  private:
  const ::pb::ap_tp::driving_resistance::DrivingResistance& _internal_drivingresistance(int index) const;
  ::pb::ap_tp::driving_resistance::DrivingResistance* _internal_add_drivingresistance();
  public:
  const ::pb::ap_tp::driving_resistance::DrivingResistance& drivingresistance(int index) const;
  ::pb::ap_tp::driving_resistance::DrivingResistance* add_drivingresistance();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::driving_resistance::DrivingResistance >&
      drivingresistance() const;

  // repeated .pb.ap_tp.planned_traj.PlannedTraj plannedTraj = 3662;
  int plannedtraj_size() const;
  private:
  int _internal_plannedtraj_size() const;
  public:
  void clear_plannedtraj();
  ::pb::ap_tp::planned_traj::PlannedTraj* mutable_plannedtraj(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::planned_traj::PlannedTraj >*
      mutable_plannedtraj();
  private:
  const ::pb::ap_tp::planned_traj::PlannedTraj& _internal_plannedtraj(int index) const;
  ::pb::ap_tp::planned_traj::PlannedTraj* _internal_add_plannedtraj();
  public:
  const ::pb::ap_tp::planned_traj::PlannedTraj& plannedtraj(int index) const;
  ::pb::ap_tp::planned_traj::PlannedTraj* add_plannedtraj();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::planned_traj::PlannedTraj >&
      plannedtraj() const;

  // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
  bool has_ssigheader() const;
  private:
  bool _internal_has_ssigheader() const;
  public:
  void clear_ssigheader();
  const ::pb::eco::signal_header::SignalHeader& ssigheader() const;
  ::pb::eco::signal_header::SignalHeader* release_ssigheader();
  ::pb::eco::signal_header::SignalHeader* mutable_ssigheader();
  void set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader);
  private:
  const ::pb::eco::signal_header::SignalHeader& _internal_ssigheader() const;
  ::pb::eco::signal_header::SignalHeader* _internal_mutable_ssigheader();
  public:

  // optional uint32 uiVersionNumber = 2124;
  bool has_uiversionnumber() const;
  private:
  bool _internal_has_uiversionnumber() const;
  public:
  void clear_uiversionnumber();
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber() const;
  void set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_uiversionnumber() const;
  void _internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 numValidCtrlPoints_nu = 850;
  bool has_numvalidctrlpoints_nu() const;
  private:
  bool _internal_has_numvalidctrlpoints_nu() const;
  public:
  void clear_numvalidctrlpoints_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 numvalidctrlpoints_nu() const;
  void set_numvalidctrlpoints_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_numvalidctrlpoints_nu() const;
  void _internal_set_numvalidctrlpoints_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 stepInTrajAfterIdx_nu = 1359;
  bool has_stepintrajafteridx_nu() const;
  private:
  bool _internal_has_stepintrajafteridx_nu() const;
  public:
  void clear_stepintrajafteridx_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 stepintrajafteridx_nu() const;
  void set_stepintrajafteridx_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_stepintrajafteridx_nu() const;
  void _internal_set_stepintrajafteridx_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional bool drivingForwardReq_nu = 3147;
  bool has_drivingforwardreq_nu() const;
  private:
  bool _internal_has_drivingforwardreq_nu() const;
  public:
  void clear_drivingforwardreq_nu();
  bool drivingforwardreq_nu() const;
  void set_drivingforwardreq_nu(bool value);
  private:
  bool _internal_drivingforwardreq_nu() const;
  void _internal_set_drivingforwardreq_nu(bool value);
  public:

  // optional bool trajValid_nu = 125;
  bool has_trajvalid_nu() const;
  private:
  bool _internal_has_trajvalid_nu() const;
  public:
  void clear_trajvalid_nu();
  bool trajvalid_nu() const;
  void set_trajvalid_nu(bool value);
  private:
  bool _internal_trajvalid_nu() const;
  void _internal_set_trajvalid_nu(bool value);
  public:

  // optional bool newSegmentStarted_nu = 1375;
  bool has_newsegmentstarted_nu() const;
  private:
  bool _internal_has_newsegmentstarted_nu() const;
  public:
  void clear_newsegmentstarted_nu();
  bool newsegmentstarted_nu() const;
  void set_newsegmentstarted_nu(bool value);
  private:
  bool _internal_newsegmentstarted_nu() const;
  void _internal_set_newsegmentstarted_nu(bool value);
  public:

  // optional bool isLastSegment_nu = 1592;
  bool has_islastsegment_nu() const;
  private:
  bool _internal_has_islastsegment_nu() const;
  public:
  void clear_islastsegment_nu();
  bool islastsegment_nu() const;
  void set_islastsegment_nu(bool value);
  private:
  bool _internal_islastsegment_nu() const;
  void _internal_set_islastsegment_nu(bool value);
  public:

  // optional .pb.ap_tp.planned_traj_type.PlannedTrajType trajType_nu = 1778;
  bool has_trajtype_nu() const;
  private:
  bool _internal_has_trajtype_nu() const;
  public:
  void clear_trajtype_nu();
  ::pb::ap_tp::planned_traj_type::PlannedTrajType trajtype_nu() const;
  void set_trajtype_nu(::pb::ap_tp::planned_traj_type::PlannedTrajType value);
  private:
  ::pb::ap_tp::planned_traj_type::PlannedTrajType _internal_trajtype_nu() const;
  void _internal_set_trajtype_nu(::pb::ap_tp::planned_traj_type::PlannedTrajType value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_tp.planned_traj_port.PlannedTrajPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::driving_resistance::DrivingResistance > drivingresistance_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::planned_traj::PlannedTraj > plannedtraj_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  ::PROTOBUF_NAMESPACE_ID::uint32 numvalidctrlpoints_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 stepintrajafteridx_nu_;
  bool drivingforwardreq_nu_;
  bool trajvalid_nu_;
  bool newsegmentstarted_nu_;
  bool islastsegment_nu_;
  int trajtype_nu_;
  friend struct ::TableStruct_ap_5ftp_2fplanned_5ftraj_5fport_2eproto;
};
// -------------------------------------------------------------------

class PlannedTrajPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_tp.planned_traj_port.PlannedTrajPort_array_port) */ {
 public:
  PlannedTrajPort_array_port();
  virtual ~PlannedTrajPort_array_port();

  PlannedTrajPort_array_port(const PlannedTrajPort_array_port& from);
  PlannedTrajPort_array_port(PlannedTrajPort_array_port&& from) noexcept
    : PlannedTrajPort_array_port() {
    *this = ::std::move(from);
  }

  inline PlannedTrajPort_array_port& operator=(const PlannedTrajPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline PlannedTrajPort_array_port& operator=(PlannedTrajPort_array_port&& from) noexcept {
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
  static const PlannedTrajPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PlannedTrajPort_array_port* internal_default_instance() {
    return reinterpret_cast<const PlannedTrajPort_array_port*>(
               &_PlannedTrajPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(PlannedTrajPort_array_port& a, PlannedTrajPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(PlannedTrajPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PlannedTrajPort_array_port* New() const final {
    return CreateMaybeMessage<PlannedTrajPort_array_port>(nullptr);
  }

  PlannedTrajPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PlannedTrajPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PlannedTrajPort_array_port& from);
  void MergeFrom(const PlannedTrajPort_array_port& from);
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
  void InternalSwap(PlannedTrajPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_tp.planned_traj_port.PlannedTrajPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftp_2fplanned_5ftraj_5fport_2eproto);
    return ::descriptor_table_ap_5ftp_2fplanned_5ftraj_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2579,
  };
  // repeated .pb.ap_tp.planned_traj_port.PlannedTrajPort data = 2579;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_tp::planned_traj_port::PlannedTrajPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::planned_traj_port::PlannedTrajPort >*
      mutable_data();
  private:
  const ::pb::ap_tp::planned_traj_port::PlannedTrajPort& _internal_data(int index) const;
  ::pb::ap_tp::planned_traj_port::PlannedTrajPort* _internal_add_data();
  public:
  const ::pb::ap_tp::planned_traj_port::PlannedTrajPort& data(int index) const;
  ::pb::ap_tp::planned_traj_port::PlannedTrajPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::planned_traj_port::PlannedTrajPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_tp.planned_traj_port.PlannedTrajPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::planned_traj_port::PlannedTrajPort > data_;
  friend struct ::TableStruct_ap_5ftp_2fplanned_5ftraj_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PlannedTrajPort

// optional uint32 uiVersionNumber = 2124;
inline bool PlannedTrajPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool PlannedTrajPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void PlannedTrajPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PlannedTrajPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PlannedTrajPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.planned_traj_port.PlannedTrajPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void PlannedTrajPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  uiversionnumber_ = value;
}
inline void PlannedTrajPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.planned_traj_port.PlannedTrajPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool PlannedTrajPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool PlannedTrajPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& PlannedTrajPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& PlannedTrajPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.planned_traj_port.PlannedTrajPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* PlannedTrajPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_tp.planned_traj_port.PlannedTrajPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* PlannedTrajPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* PlannedTrajPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.planned_traj_port.PlannedTrajPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void PlannedTrajPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(ssigheader_);
  }
  if (ssigheader) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      ssigheader = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, ssigheader, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  ssigheader_ = ssigheader;
  // @@protoc_insertion_point(field_set_allocated:pb.ap_tp.planned_traj_port.PlannedTrajPort.sSigHeader)
}

// optional .pb.ap_tp.planned_traj_type.PlannedTrajType trajType_nu = 1778;
inline bool PlannedTrajPort::_internal_has_trajtype_nu() const {
  bool value = (_has_bits_[0] & 0x00000100u) != 0;
  return value;
}
inline bool PlannedTrajPort::has_trajtype_nu() const {
  return _internal_has_trajtype_nu();
}
inline void PlannedTrajPort::clear_trajtype_nu() {
  trajtype_nu_ = 0;
  _has_bits_[0] &= ~0x00000100u;
}
inline ::pb::ap_tp::planned_traj_type::PlannedTrajType PlannedTrajPort::_internal_trajtype_nu() const {
  return static_cast< ::pb::ap_tp::planned_traj_type::PlannedTrajType >(trajtype_nu_);
}
inline ::pb::ap_tp::planned_traj_type::PlannedTrajType PlannedTrajPort::trajtype_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.planned_traj_port.PlannedTrajPort.trajType_nu)
  return _internal_trajtype_nu();
}
inline void PlannedTrajPort::_internal_set_trajtype_nu(::pb::ap_tp::planned_traj_type::PlannedTrajType value) {
  assert(::pb::ap_tp::planned_traj_type::PlannedTrajType_IsValid(value));
  _has_bits_[0] |= 0x00000100u;
  trajtype_nu_ = value;
}
inline void PlannedTrajPort::set_trajtype_nu(::pb::ap_tp::planned_traj_type::PlannedTrajType value) {
  _internal_set_trajtype_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.planned_traj_port.PlannedTrajPort.trajType_nu)
}

// optional bool drivingForwardReq_nu = 3147;
inline bool PlannedTrajPort::_internal_has_drivingforwardreq_nu() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool PlannedTrajPort::has_drivingforwardreq_nu() const {
  return _internal_has_drivingforwardreq_nu();
}
inline void PlannedTrajPort::clear_drivingforwardreq_nu() {
  drivingforwardreq_nu_ = false;
  _has_bits_[0] &= ~0x00000010u;
}
inline bool PlannedTrajPort::_internal_drivingforwardreq_nu() const {
  return drivingforwardreq_nu_;
}
inline bool PlannedTrajPort::drivingforwardreq_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.planned_traj_port.PlannedTrajPort.drivingForwardReq_nu)
  return _internal_drivingforwardreq_nu();
}
inline void PlannedTrajPort::_internal_set_drivingforwardreq_nu(bool value) {
  _has_bits_[0] |= 0x00000010u;
  drivingforwardreq_nu_ = value;
}
inline void PlannedTrajPort::set_drivingforwardreq_nu(bool value) {
  _internal_set_drivingforwardreq_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.planned_traj_port.PlannedTrajPort.drivingForwardReq_nu)
}

// optional bool trajValid_nu = 125;
inline bool PlannedTrajPort::_internal_has_trajvalid_nu() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool PlannedTrajPort::has_trajvalid_nu() const {
  return _internal_has_trajvalid_nu();
}
inline void PlannedTrajPort::clear_trajvalid_nu() {
  trajvalid_nu_ = false;
  _has_bits_[0] &= ~0x00000020u;
}
inline bool PlannedTrajPort::_internal_trajvalid_nu() const {
  return trajvalid_nu_;
}
inline bool PlannedTrajPort::trajvalid_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.planned_traj_port.PlannedTrajPort.trajValid_nu)
  return _internal_trajvalid_nu();
}
inline void PlannedTrajPort::_internal_set_trajvalid_nu(bool value) {
  _has_bits_[0] |= 0x00000020u;
  trajvalid_nu_ = value;
}
inline void PlannedTrajPort::set_trajvalid_nu(bool value) {
  _internal_set_trajvalid_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.planned_traj_port.PlannedTrajPort.trajValid_nu)
}

// optional bool newSegmentStarted_nu = 1375;
inline bool PlannedTrajPort::_internal_has_newsegmentstarted_nu() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool PlannedTrajPort::has_newsegmentstarted_nu() const {
  return _internal_has_newsegmentstarted_nu();
}
inline void PlannedTrajPort::clear_newsegmentstarted_nu() {
  newsegmentstarted_nu_ = false;
  _has_bits_[0] &= ~0x00000040u;
}
inline bool PlannedTrajPort::_internal_newsegmentstarted_nu() const {
  return newsegmentstarted_nu_;
}
inline bool PlannedTrajPort::newsegmentstarted_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.planned_traj_port.PlannedTrajPort.newSegmentStarted_nu)
  return _internal_newsegmentstarted_nu();
}
inline void PlannedTrajPort::_internal_set_newsegmentstarted_nu(bool value) {
  _has_bits_[0] |= 0x00000040u;
  newsegmentstarted_nu_ = value;
}
inline void PlannedTrajPort::set_newsegmentstarted_nu(bool value) {
  _internal_set_newsegmentstarted_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.planned_traj_port.PlannedTrajPort.newSegmentStarted_nu)
}

// optional bool isLastSegment_nu = 1592;
inline bool PlannedTrajPort::_internal_has_islastsegment_nu() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool PlannedTrajPort::has_islastsegment_nu() const {
  return _internal_has_islastsegment_nu();
}
inline void PlannedTrajPort::clear_islastsegment_nu() {
  islastsegment_nu_ = false;
  _has_bits_[0] &= ~0x00000080u;
}
inline bool PlannedTrajPort::_internal_islastsegment_nu() const {
  return islastsegment_nu_;
}
inline bool PlannedTrajPort::islastsegment_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.planned_traj_port.PlannedTrajPort.isLastSegment_nu)
  return _internal_islastsegment_nu();
}
inline void PlannedTrajPort::_internal_set_islastsegment_nu(bool value) {
  _has_bits_[0] |= 0x00000080u;
  islastsegment_nu_ = value;
}
inline void PlannedTrajPort::set_islastsegment_nu(bool value) {
  _internal_set_islastsegment_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.planned_traj_port.PlannedTrajPort.isLastSegment_nu)
}

// optional uint32 stepInTrajAfterIdx_nu = 1359;
inline bool PlannedTrajPort::_internal_has_stepintrajafteridx_nu() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool PlannedTrajPort::has_stepintrajafteridx_nu() const {
  return _internal_has_stepintrajafteridx_nu();
}
inline void PlannedTrajPort::clear_stepintrajafteridx_nu() {
  stepintrajafteridx_nu_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PlannedTrajPort::_internal_stepintrajafteridx_nu() const {
  return stepintrajafteridx_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PlannedTrajPort::stepintrajafteridx_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.planned_traj_port.PlannedTrajPort.stepInTrajAfterIdx_nu)
  return _internal_stepintrajafteridx_nu();
}
inline void PlannedTrajPort::_internal_set_stepintrajafteridx_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  stepintrajafteridx_nu_ = value;
}
inline void PlannedTrajPort::set_stepintrajafteridx_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_stepintrajafteridx_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.planned_traj_port.PlannedTrajPort.stepInTrajAfterIdx_nu)
}

// repeated .pb.ap_tp.driving_resistance.DrivingResistance drivingResistance = 2580;
inline int PlannedTrajPort::_internal_drivingresistance_size() const {
  return drivingresistance_.size();
}
inline int PlannedTrajPort::drivingresistance_size() const {
  return _internal_drivingresistance_size();
}
inline ::pb::ap_tp::driving_resistance::DrivingResistance* PlannedTrajPort::mutable_drivingresistance(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.planned_traj_port.PlannedTrajPort.drivingResistance)
  return drivingresistance_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::driving_resistance::DrivingResistance >*
PlannedTrajPort::mutable_drivingresistance() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_tp.planned_traj_port.PlannedTrajPort.drivingResistance)
  return &drivingresistance_;
}
inline const ::pb::ap_tp::driving_resistance::DrivingResistance& PlannedTrajPort::_internal_drivingresistance(int index) const {
  return drivingresistance_.Get(index);
}
inline const ::pb::ap_tp::driving_resistance::DrivingResistance& PlannedTrajPort::drivingresistance(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.planned_traj_port.PlannedTrajPort.drivingResistance)
  return _internal_drivingresistance(index);
}
inline ::pb::ap_tp::driving_resistance::DrivingResistance* PlannedTrajPort::_internal_add_drivingresistance() {
  return drivingresistance_.Add();
}
inline ::pb::ap_tp::driving_resistance::DrivingResistance* PlannedTrajPort::add_drivingresistance() {
  // @@protoc_insertion_point(field_add:pb.ap_tp.planned_traj_port.PlannedTrajPort.drivingResistance)
  return _internal_add_drivingresistance();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::driving_resistance::DrivingResistance >&
PlannedTrajPort::drivingresistance() const {
  // @@protoc_insertion_point(field_list:pb.ap_tp.planned_traj_port.PlannedTrajPort.drivingResistance)
  return drivingresistance_;
}

// optional uint32 numValidCtrlPoints_nu = 850;
inline bool PlannedTrajPort::_internal_has_numvalidctrlpoints_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool PlannedTrajPort::has_numvalidctrlpoints_nu() const {
  return _internal_has_numvalidctrlpoints_nu();
}
inline void PlannedTrajPort::clear_numvalidctrlpoints_nu() {
  numvalidctrlpoints_nu_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PlannedTrajPort::_internal_numvalidctrlpoints_nu() const {
  return numvalidctrlpoints_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PlannedTrajPort::numvalidctrlpoints_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.planned_traj_port.PlannedTrajPort.numValidCtrlPoints_nu)
  return _internal_numvalidctrlpoints_nu();
}
inline void PlannedTrajPort::_internal_set_numvalidctrlpoints_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  numvalidctrlpoints_nu_ = value;
}
inline void PlannedTrajPort::set_numvalidctrlpoints_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_numvalidctrlpoints_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.planned_traj_port.PlannedTrajPort.numValidCtrlPoints_nu)
}

// repeated .pb.ap_tp.planned_traj.PlannedTraj plannedTraj = 3662;
inline int PlannedTrajPort::_internal_plannedtraj_size() const {
  return plannedtraj_.size();
}
inline int PlannedTrajPort::plannedtraj_size() const {
  return _internal_plannedtraj_size();
}
inline ::pb::ap_tp::planned_traj::PlannedTraj* PlannedTrajPort::mutable_plannedtraj(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.planned_traj_port.PlannedTrajPort.plannedTraj)
  return plannedtraj_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::planned_traj::PlannedTraj >*
PlannedTrajPort::mutable_plannedtraj() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_tp.planned_traj_port.PlannedTrajPort.plannedTraj)
  return &plannedtraj_;
}
inline const ::pb::ap_tp::planned_traj::PlannedTraj& PlannedTrajPort::_internal_plannedtraj(int index) const {
  return plannedtraj_.Get(index);
}
inline const ::pb::ap_tp::planned_traj::PlannedTraj& PlannedTrajPort::plannedtraj(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.planned_traj_port.PlannedTrajPort.plannedTraj)
  return _internal_plannedtraj(index);
}
inline ::pb::ap_tp::planned_traj::PlannedTraj* PlannedTrajPort::_internal_add_plannedtraj() {
  return plannedtraj_.Add();
}
inline ::pb::ap_tp::planned_traj::PlannedTraj* PlannedTrajPort::add_plannedtraj() {
  // @@protoc_insertion_point(field_add:pb.ap_tp.planned_traj_port.PlannedTrajPort.plannedTraj)
  return _internal_add_plannedtraj();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::planned_traj::PlannedTraj >&
PlannedTrajPort::plannedtraj() const {
  // @@protoc_insertion_point(field_list:pb.ap_tp.planned_traj_port.PlannedTrajPort.plannedTraj)
  return plannedtraj_;
}

// -------------------------------------------------------------------

// PlannedTrajPort_array_port

// repeated .pb.ap_tp.planned_traj_port.PlannedTrajPort data = 2579;
inline int PlannedTrajPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int PlannedTrajPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void PlannedTrajPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_tp::planned_traj_port::PlannedTrajPort* PlannedTrajPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.planned_traj_port.PlannedTrajPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::planned_traj_port::PlannedTrajPort >*
PlannedTrajPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_tp.planned_traj_port.PlannedTrajPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_tp::planned_traj_port::PlannedTrajPort& PlannedTrajPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_tp::planned_traj_port::PlannedTrajPort& PlannedTrajPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.planned_traj_port.PlannedTrajPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_tp::planned_traj_port::PlannedTrajPort* PlannedTrajPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_tp::planned_traj_port::PlannedTrajPort* PlannedTrajPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_tp.planned_traj_port.PlannedTrajPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::planned_traj_port::PlannedTrajPort >&
PlannedTrajPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_tp.planned_traj_port.PlannedTrajPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace planned_traj_port
}  // namespace ap_tp
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5ftp_2fplanned_5ftraj_5fport_2eproto
