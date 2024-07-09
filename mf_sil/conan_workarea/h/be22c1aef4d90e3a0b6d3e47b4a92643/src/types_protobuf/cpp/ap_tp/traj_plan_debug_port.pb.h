// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_tp/traj_plan_debug_port.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5ftp_2ftraj_5fplan_5fdebug_5fport_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5ftp_2ftraj_5fplan_5fdebug_5fport_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5ftp_2ftraj_5fplan_5fdebug_5fport_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5ftp_2ftraj_5fplan_5fdebug_5fport_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftp_2ftraj_5fplan_5fdebug_5fport_2eproto;
namespace pb {
namespace ap_tp {
namespace traj_plan_debug_port {
class TrajPlanDebugPort;
class TrajPlanDebugPortDefaultTypeInternal;
extern TrajPlanDebugPortDefaultTypeInternal _TrajPlanDebugPort_default_instance_;
class TrajPlanDebugPort_array_port;
class TrajPlanDebugPort_array_portDefaultTypeInternal;
extern TrajPlanDebugPort_array_portDefaultTypeInternal _TrajPlanDebugPort_array_port_default_instance_;
}  // namespace traj_plan_debug_port
}  // namespace ap_tp
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort* Arena::CreateMaybeMessage<::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort>(Arena*);
template<> ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort_array_port* Arena::CreateMaybeMessage<::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_tp {
namespace traj_plan_debug_port {

// ===================================================================

class TrajPlanDebugPort :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort) */ {
 public:
  TrajPlanDebugPort();
  virtual ~TrajPlanDebugPort();

  TrajPlanDebugPort(const TrajPlanDebugPort& from);
  TrajPlanDebugPort(TrajPlanDebugPort&& from) noexcept
    : TrajPlanDebugPort() {
    *this = ::std::move(from);
  }

  inline TrajPlanDebugPort& operator=(const TrajPlanDebugPort& from) {
    CopyFrom(from);
    return *this;
  }
  inline TrajPlanDebugPort& operator=(TrajPlanDebugPort&& from) noexcept {
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
  static const TrajPlanDebugPort& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TrajPlanDebugPort* internal_default_instance() {
    return reinterpret_cast<const TrajPlanDebugPort*>(
               &_TrajPlanDebugPort_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(TrajPlanDebugPort& a, TrajPlanDebugPort& b) {
    a.Swap(&b);
  }
  inline void Swap(TrajPlanDebugPort* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TrajPlanDebugPort* New() const final {
    return CreateMaybeMessage<TrajPlanDebugPort>(nullptr);
  }

  TrajPlanDebugPort* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TrajPlanDebugPort>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TrajPlanDebugPort& from);
  void MergeFrom(const TrajPlanDebugPort& from);
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
  void InternalSwap(TrajPlanDebugPort* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftp_2ftraj_5fplan_5fdebug_5fport_2eproto);
    return ::descriptor_table_ap_5ftp_2ftraj_5fplan_5fdebug_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDebugIntFieldNumber = 1230,
    kDebugFloatFieldNumber = 2041,
    kSSigHeaderFieldNumber = 1033,
    kMReplanSuccessfulNuFieldNumber = 699,
    kMStateEntryNuFieldNumber = 2547,
    kMNumOfReplanCallsFieldNumber = 1717,
    kUiVersionNumberFieldNumber = 2124,
    kMTrajPlanStateFieldNumber = 3141,
  };
  // repeated sint32 debugInt = 1230;
  int debugint_size() const;
  private:
  int _internal_debugint_size() const;
  public:
  void clear_debugint();
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_debugint(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::int32 >&
      _internal_debugint() const;
  void _internal_add_debugint(::PROTOBUF_NAMESPACE_ID::int32 value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::int32 >*
      _internal_mutable_debugint();
  public:
  ::PROTOBUF_NAMESPACE_ID::int32 debugint(int index) const;
  void set_debugint(int index, ::PROTOBUF_NAMESPACE_ID::int32 value);
  void add_debugint(::PROTOBUF_NAMESPACE_ID::int32 value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::int32 >&
      debugint() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::int32 >*
      mutable_debugint();

  // repeated float debugFloat = 2041;
  int debugfloat_size() const;
  private:
  int _internal_debugfloat_size() const;
  public:
  void clear_debugfloat();
  private:
  float _internal_debugfloat(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_debugfloat() const;
  void _internal_add_debugfloat(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_debugfloat();
  public:
  float debugfloat(int index) const;
  void set_debugfloat(int index, float value);
  void add_debugfloat(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      debugfloat() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_debugfloat();

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

  // optional bool mReplanSuccessful_nu = 699;
  bool has_mreplansuccessful_nu() const;
  private:
  bool _internal_has_mreplansuccessful_nu() const;
  public:
  void clear_mreplansuccessful_nu();
  bool mreplansuccessful_nu() const;
  void set_mreplansuccessful_nu(bool value);
  private:
  bool _internal_mreplansuccessful_nu() const;
  void _internal_set_mreplansuccessful_nu(bool value);
  public:

  // optional bool mStateEntry_nu = 2547;
  bool has_mstateentry_nu() const;
  private:
  bool _internal_has_mstateentry_nu() const;
  public:
  void clear_mstateentry_nu();
  bool mstateentry_nu() const;
  void set_mstateentry_nu(bool value);
  private:
  bool _internal_mstateentry_nu() const;
  void _internal_set_mstateentry_nu(bool value);
  public:

  // optional uint32 mNumOfReplanCalls = 1717;
  bool has_mnumofreplancalls() const;
  private:
  bool _internal_has_mnumofreplancalls() const;
  public:
  void clear_mnumofreplancalls();
  ::PROTOBUF_NAMESPACE_ID::uint32 mnumofreplancalls() const;
  void set_mnumofreplancalls(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_mnumofreplancalls() const;
  void _internal_set_mnumofreplancalls(::PROTOBUF_NAMESPACE_ID::uint32 value);
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

  // optional uint32 mTrajPlanState = 3141;
  bool has_mtrajplanstate() const;
  private:
  bool _internal_has_mtrajplanstate() const;
  public:
  void clear_mtrajplanstate();
  ::PROTOBUF_NAMESPACE_ID::uint32 mtrajplanstate() const;
  void set_mtrajplanstate(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_mtrajplanstate() const;
  void _internal_set_mtrajplanstate(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::int32 > debugint_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > debugfloat_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  bool mreplansuccessful_nu_;
  bool mstateentry_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 mnumofreplancalls_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  ::PROTOBUF_NAMESPACE_ID::uint32 mtrajplanstate_;
  friend struct ::TableStruct_ap_5ftp_2ftraj_5fplan_5fdebug_5fport_2eproto;
};
// -------------------------------------------------------------------

class TrajPlanDebugPort_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort_array_port) */ {
 public:
  TrajPlanDebugPort_array_port();
  virtual ~TrajPlanDebugPort_array_port();

  TrajPlanDebugPort_array_port(const TrajPlanDebugPort_array_port& from);
  TrajPlanDebugPort_array_port(TrajPlanDebugPort_array_port&& from) noexcept
    : TrajPlanDebugPort_array_port() {
    *this = ::std::move(from);
  }

  inline TrajPlanDebugPort_array_port& operator=(const TrajPlanDebugPort_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline TrajPlanDebugPort_array_port& operator=(TrajPlanDebugPort_array_port&& from) noexcept {
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
  static const TrajPlanDebugPort_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TrajPlanDebugPort_array_port* internal_default_instance() {
    return reinterpret_cast<const TrajPlanDebugPort_array_port*>(
               &_TrajPlanDebugPort_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(TrajPlanDebugPort_array_port& a, TrajPlanDebugPort_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(TrajPlanDebugPort_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TrajPlanDebugPort_array_port* New() const final {
    return CreateMaybeMessage<TrajPlanDebugPort_array_port>(nullptr);
  }

  TrajPlanDebugPort_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TrajPlanDebugPort_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TrajPlanDebugPort_array_port& from);
  void MergeFrom(const TrajPlanDebugPort_array_port& from);
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
  void InternalSwap(TrajPlanDebugPort_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftp_2ftraj_5fplan_5fdebug_5fport_2eproto);
    return ::descriptor_table_ap_5ftp_2ftraj_5fplan_5fdebug_5fport_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2697,
  };
  // repeated .pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort data = 2697;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort >*
      mutable_data();
  private:
  const ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort& _internal_data(int index) const;
  ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort* _internal_add_data();
  public:
  const ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort& data(int index) const;
  ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort > data_;
  friend struct ::TableStruct_ap_5ftp_2ftraj_5fplan_5fdebug_5fport_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// TrajPlanDebugPort

// optional uint32 uiVersionNumber = 2124;
inline bool TrajPlanDebugPort::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool TrajPlanDebugPort::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void TrajPlanDebugPort::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TrajPlanDebugPort::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TrajPlanDebugPort::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void TrajPlanDebugPort::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  uiversionnumber_ = value;
}
inline void TrajPlanDebugPort::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool TrajPlanDebugPort::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool TrajPlanDebugPort::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& TrajPlanDebugPort::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& TrajPlanDebugPort::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* TrajPlanDebugPort::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* TrajPlanDebugPort::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* TrajPlanDebugPort::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void TrajPlanDebugPort::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
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
  // @@protoc_insertion_point(field_set_allocated:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.sSigHeader)
}

// repeated sint32 debugInt = 1230;
inline int TrajPlanDebugPort::_internal_debugint_size() const {
  return debugint_.size();
}
inline int TrajPlanDebugPort::debugint_size() const {
  return _internal_debugint_size();
}
inline void TrajPlanDebugPort::clear_debugint() {
  debugint_.Clear();
}
inline ::PROTOBUF_NAMESPACE_ID::int32 TrajPlanDebugPort::_internal_debugint(int index) const {
  return debugint_.Get(index);
}
inline ::PROTOBUF_NAMESPACE_ID::int32 TrajPlanDebugPort::debugint(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.debugInt)
  return _internal_debugint(index);
}
inline void TrajPlanDebugPort::set_debugint(int index, ::PROTOBUF_NAMESPACE_ID::int32 value) {
  debugint_.Set(index, value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.debugInt)
}
inline void TrajPlanDebugPort::_internal_add_debugint(::PROTOBUF_NAMESPACE_ID::int32 value) {
  debugint_.Add(value);
}
inline void TrajPlanDebugPort::add_debugint(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_add_debugint(value);
  // @@protoc_insertion_point(field_add:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.debugInt)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::int32 >&
TrajPlanDebugPort::_internal_debugint() const {
  return debugint_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::int32 >&
TrajPlanDebugPort::debugint() const {
  // @@protoc_insertion_point(field_list:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.debugInt)
  return _internal_debugint();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::int32 >*
TrajPlanDebugPort::_internal_mutable_debugint() {
  return &debugint_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< ::PROTOBUF_NAMESPACE_ID::int32 >*
TrajPlanDebugPort::mutable_debugint() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.debugInt)
  return _internal_mutable_debugint();
}

// repeated float debugFloat = 2041;
inline int TrajPlanDebugPort::_internal_debugfloat_size() const {
  return debugfloat_.size();
}
inline int TrajPlanDebugPort::debugfloat_size() const {
  return _internal_debugfloat_size();
}
inline void TrajPlanDebugPort::clear_debugfloat() {
  debugfloat_.Clear();
}
inline float TrajPlanDebugPort::_internal_debugfloat(int index) const {
  return debugfloat_.Get(index);
}
inline float TrajPlanDebugPort::debugfloat(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.debugFloat)
  return _internal_debugfloat(index);
}
inline void TrajPlanDebugPort::set_debugfloat(int index, float value) {
  debugfloat_.Set(index, value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.debugFloat)
}
inline void TrajPlanDebugPort::_internal_add_debugfloat(float value) {
  debugfloat_.Add(value);
}
inline void TrajPlanDebugPort::add_debugfloat(float value) {
  _internal_add_debugfloat(value);
  // @@protoc_insertion_point(field_add:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.debugFloat)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
TrajPlanDebugPort::_internal_debugfloat() const {
  return debugfloat_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
TrajPlanDebugPort::debugfloat() const {
  // @@protoc_insertion_point(field_list:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.debugFloat)
  return _internal_debugfloat();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
TrajPlanDebugPort::_internal_mutable_debugfloat() {
  return &debugfloat_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
TrajPlanDebugPort::mutable_debugfloat() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.debugFloat)
  return _internal_mutable_debugfloat();
}

// optional uint32 mNumOfReplanCalls = 1717;
inline bool TrajPlanDebugPort::_internal_has_mnumofreplancalls() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool TrajPlanDebugPort::has_mnumofreplancalls() const {
  return _internal_has_mnumofreplancalls();
}
inline void TrajPlanDebugPort::clear_mnumofreplancalls() {
  mnumofreplancalls_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TrajPlanDebugPort::_internal_mnumofreplancalls() const {
  return mnumofreplancalls_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TrajPlanDebugPort::mnumofreplancalls() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.mNumOfReplanCalls)
  return _internal_mnumofreplancalls();
}
inline void TrajPlanDebugPort::_internal_set_mnumofreplancalls(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  mnumofreplancalls_ = value;
}
inline void TrajPlanDebugPort::set_mnumofreplancalls(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_mnumofreplancalls(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.mNumOfReplanCalls)
}

// optional uint32 mTrajPlanState = 3141;
inline bool TrajPlanDebugPort::_internal_has_mtrajplanstate() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool TrajPlanDebugPort::has_mtrajplanstate() const {
  return _internal_has_mtrajplanstate();
}
inline void TrajPlanDebugPort::clear_mtrajplanstate() {
  mtrajplanstate_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TrajPlanDebugPort::_internal_mtrajplanstate() const {
  return mtrajplanstate_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 TrajPlanDebugPort::mtrajplanstate() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.mTrajPlanState)
  return _internal_mtrajplanstate();
}
inline void TrajPlanDebugPort::_internal_set_mtrajplanstate(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  mtrajplanstate_ = value;
}
inline void TrajPlanDebugPort::set_mtrajplanstate(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_mtrajplanstate(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.mTrajPlanState)
}

// optional bool mReplanSuccessful_nu = 699;
inline bool TrajPlanDebugPort::_internal_has_mreplansuccessful_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool TrajPlanDebugPort::has_mreplansuccessful_nu() const {
  return _internal_has_mreplansuccessful_nu();
}
inline void TrajPlanDebugPort::clear_mreplansuccessful_nu() {
  mreplansuccessful_nu_ = false;
  _has_bits_[0] &= ~0x00000002u;
}
inline bool TrajPlanDebugPort::_internal_mreplansuccessful_nu() const {
  return mreplansuccessful_nu_;
}
inline bool TrajPlanDebugPort::mreplansuccessful_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.mReplanSuccessful_nu)
  return _internal_mreplansuccessful_nu();
}
inline void TrajPlanDebugPort::_internal_set_mreplansuccessful_nu(bool value) {
  _has_bits_[0] |= 0x00000002u;
  mreplansuccessful_nu_ = value;
}
inline void TrajPlanDebugPort::set_mreplansuccessful_nu(bool value) {
  _internal_set_mreplansuccessful_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.mReplanSuccessful_nu)
}

// optional bool mStateEntry_nu = 2547;
inline bool TrajPlanDebugPort::_internal_has_mstateentry_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool TrajPlanDebugPort::has_mstateentry_nu() const {
  return _internal_has_mstateentry_nu();
}
inline void TrajPlanDebugPort::clear_mstateentry_nu() {
  mstateentry_nu_ = false;
  _has_bits_[0] &= ~0x00000004u;
}
inline bool TrajPlanDebugPort::_internal_mstateentry_nu() const {
  return mstateentry_nu_;
}
inline bool TrajPlanDebugPort::mstateentry_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.mStateEntry_nu)
  return _internal_mstateentry_nu();
}
inline void TrajPlanDebugPort::_internal_set_mstateentry_nu(bool value) {
  _has_bits_[0] |= 0x00000004u;
  mstateentry_nu_ = value;
}
inline void TrajPlanDebugPort::set_mstateentry_nu(bool value) {
  _internal_set_mstateentry_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort.mStateEntry_nu)
}

// -------------------------------------------------------------------

// TrajPlanDebugPort_array_port

// repeated .pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort data = 2697;
inline int TrajPlanDebugPort_array_port::_internal_data_size() const {
  return data_.size();
}
inline int TrajPlanDebugPort_array_port::data_size() const {
  return _internal_data_size();
}
inline void TrajPlanDebugPort_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort* TrajPlanDebugPort_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort >*
TrajPlanDebugPort_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort_array_port.data)
  return &data_;
}
inline const ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort& TrajPlanDebugPort_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort& TrajPlanDebugPort_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort* TrajPlanDebugPort_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort* TrajPlanDebugPort_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::traj_plan_debug_port::TrajPlanDebugPort >&
TrajPlanDebugPort_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_tp.traj_plan_debug_port.TrajPlanDebugPort_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace traj_plan_debug_port
}  // namespace ap_tp
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5ftp_2ftraj_5fplan_5fdebug_5fport_2eproto