// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_psm_app/ap_psm_app_consts.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_5fapp_2fap_5fpsm_5fapp_5fconsts_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_5fapp_2fap_5fpsm_5fapp_5fconsts_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5fpsm_5fapp_2fap_5fpsm_5fapp_5fconsts_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fpsm_5fapp_2fap_5fpsm_5fapp_5fconsts_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fpsm_5fapp_2fap_5fpsm_5fapp_5fconsts_2eproto;
namespace pb {
namespace ap_psm_app {
namespace ap_psm_app_consts {
class AP_PSM_APP_Consts;
class AP_PSM_APP_ConstsDefaultTypeInternal;
extern AP_PSM_APP_ConstsDefaultTypeInternal _AP_PSM_APP_Consts_default_instance_;
class AP_PSM_APP_Consts_array_port;
class AP_PSM_APP_Consts_array_portDefaultTypeInternal;
extern AP_PSM_APP_Consts_array_portDefaultTypeInternal _AP_PSM_APP_Consts_array_port_default_instance_;
}  // namespace ap_psm_app_consts
}  // namespace ap_psm_app
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts* Arena::CreateMaybeMessage<::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts>(Arena*);
template<> ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts_array_port* Arena::CreateMaybeMessage<::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_psm_app {
namespace ap_psm_app_consts {

// ===================================================================

class AP_PSM_APP_Consts :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts) */ {
 public:
  AP_PSM_APP_Consts();
  virtual ~AP_PSM_APP_Consts();

  AP_PSM_APP_Consts(const AP_PSM_APP_Consts& from);
  AP_PSM_APP_Consts(AP_PSM_APP_Consts&& from) noexcept
    : AP_PSM_APP_Consts() {
    *this = ::std::move(from);
  }

  inline AP_PSM_APP_Consts& operator=(const AP_PSM_APP_Consts& from) {
    CopyFrom(from);
    return *this;
  }
  inline AP_PSM_APP_Consts& operator=(AP_PSM_APP_Consts&& from) noexcept {
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
  static const AP_PSM_APP_Consts& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const AP_PSM_APP_Consts* internal_default_instance() {
    return reinterpret_cast<const AP_PSM_APP_Consts*>(
               &_AP_PSM_APP_Consts_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(AP_PSM_APP_Consts& a, AP_PSM_APP_Consts& b) {
    a.Swap(&b);
  }
  inline void Swap(AP_PSM_APP_Consts* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline AP_PSM_APP_Consts* New() const final {
    return CreateMaybeMessage<AP_PSM_APP_Consts>(nullptr);
  }

  AP_PSM_APP_Consts* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<AP_PSM_APP_Consts>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const AP_PSM_APP_Consts& from);
  void MergeFrom(const AP_PSM_APP_Consts& from);
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
  void InternalSwap(AP_PSM_APP_Consts* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fpsm_5fapp_2fap_5fpsm_5fapp_5fconsts_2eproto);
    return ::descriptor_table_ap_5fpsm_5fapp_2fap_5fpsm_5fapp_5fconsts_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kNUMMTSDEBUGFREESPACEAPPPSMFieldNumber = 1642,
    kNUMPRECONDITIONFAILUREAPPPSMFieldNumber = 2613,
  };
  // optional uint32 NUM_MTS_DEBUG_FREESPACE_APP_PSM = 1642;
  bool has_num_mts_debug_freespace_app_psm() const;
  private:
  bool _internal_has_num_mts_debug_freespace_app_psm() const;
  public:
  void clear_num_mts_debug_freespace_app_psm();
  ::PROTOBUF_NAMESPACE_ID::uint32 num_mts_debug_freespace_app_psm() const;
  void set_num_mts_debug_freespace_app_psm(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_num_mts_debug_freespace_app_psm() const;
  void _internal_set_num_mts_debug_freespace_app_psm(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 NUM_PRE_CONDITION_FAILURE_APP_PSM = 2613;
  bool has_num_pre_condition_failure_app_psm() const;
  private:
  bool _internal_has_num_pre_condition_failure_app_psm() const;
  public:
  void clear_num_pre_condition_failure_app_psm();
  ::PROTOBUF_NAMESPACE_ID::uint32 num_pre_condition_failure_app_psm() const;
  void set_num_pre_condition_failure_app_psm(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_num_pre_condition_failure_app_psm() const;
  void _internal_set_num_pre_condition_failure_app_psm(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 num_mts_debug_freespace_app_psm_;
  ::PROTOBUF_NAMESPACE_ID::uint32 num_pre_condition_failure_app_psm_;
  friend struct ::TableStruct_ap_5fpsm_5fapp_2fap_5fpsm_5fapp_5fconsts_2eproto;
};
// -------------------------------------------------------------------

class AP_PSM_APP_Consts_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts_array_port) */ {
 public:
  AP_PSM_APP_Consts_array_port();
  virtual ~AP_PSM_APP_Consts_array_port();

  AP_PSM_APP_Consts_array_port(const AP_PSM_APP_Consts_array_port& from);
  AP_PSM_APP_Consts_array_port(AP_PSM_APP_Consts_array_port&& from) noexcept
    : AP_PSM_APP_Consts_array_port() {
    *this = ::std::move(from);
  }

  inline AP_PSM_APP_Consts_array_port& operator=(const AP_PSM_APP_Consts_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline AP_PSM_APP_Consts_array_port& operator=(AP_PSM_APP_Consts_array_port&& from) noexcept {
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
  static const AP_PSM_APP_Consts_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const AP_PSM_APP_Consts_array_port* internal_default_instance() {
    return reinterpret_cast<const AP_PSM_APP_Consts_array_port*>(
               &_AP_PSM_APP_Consts_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(AP_PSM_APP_Consts_array_port& a, AP_PSM_APP_Consts_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(AP_PSM_APP_Consts_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline AP_PSM_APP_Consts_array_port* New() const final {
    return CreateMaybeMessage<AP_PSM_APP_Consts_array_port>(nullptr);
  }

  AP_PSM_APP_Consts_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<AP_PSM_APP_Consts_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const AP_PSM_APP_Consts_array_port& from);
  void MergeFrom(const AP_PSM_APP_Consts_array_port& from);
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
  void InternalSwap(AP_PSM_APP_Consts_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fpsm_5fapp_2fap_5fpsm_5fapp_5fconsts_2eproto);
    return ::descriptor_table_ap_5fpsm_5fapp_2fap_5fpsm_5fapp_5fconsts_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2466,
  };
  // repeated .pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts data = 2466;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts >*
      mutable_data();
  private:
  const ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts& _internal_data(int index) const;
  ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts* _internal_add_data();
  public:
  const ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts& data(int index) const;
  ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts > data_;
  friend struct ::TableStruct_ap_5fpsm_5fapp_2fap_5fpsm_5fapp_5fconsts_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// AP_PSM_APP_Consts

// optional uint32 NUM_MTS_DEBUG_FREESPACE_APP_PSM = 1642;
inline bool AP_PSM_APP_Consts::_internal_has_num_mts_debug_freespace_app_psm() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool AP_PSM_APP_Consts::has_num_mts_debug_freespace_app_psm() const {
  return _internal_has_num_mts_debug_freespace_app_psm();
}
inline void AP_PSM_APP_Consts::clear_num_mts_debug_freespace_app_psm() {
  num_mts_debug_freespace_app_psm_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 AP_PSM_APP_Consts::_internal_num_mts_debug_freespace_app_psm() const {
  return num_mts_debug_freespace_app_psm_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 AP_PSM_APP_Consts::num_mts_debug_freespace_app_psm() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts.NUM_MTS_DEBUG_FREESPACE_APP_PSM)
  return _internal_num_mts_debug_freespace_app_psm();
}
inline void AP_PSM_APP_Consts::_internal_set_num_mts_debug_freespace_app_psm(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  num_mts_debug_freespace_app_psm_ = value;
}
inline void AP_PSM_APP_Consts::set_num_mts_debug_freespace_app_psm(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_num_mts_debug_freespace_app_psm(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts.NUM_MTS_DEBUG_FREESPACE_APP_PSM)
}

// optional uint32 NUM_PRE_CONDITION_FAILURE_APP_PSM = 2613;
inline bool AP_PSM_APP_Consts::_internal_has_num_pre_condition_failure_app_psm() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool AP_PSM_APP_Consts::has_num_pre_condition_failure_app_psm() const {
  return _internal_has_num_pre_condition_failure_app_psm();
}
inline void AP_PSM_APP_Consts::clear_num_pre_condition_failure_app_psm() {
  num_pre_condition_failure_app_psm_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 AP_PSM_APP_Consts::_internal_num_pre_condition_failure_app_psm() const {
  return num_pre_condition_failure_app_psm_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 AP_PSM_APP_Consts::num_pre_condition_failure_app_psm() const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts.NUM_PRE_CONDITION_FAILURE_APP_PSM)
  return _internal_num_pre_condition_failure_app_psm();
}
inline void AP_PSM_APP_Consts::_internal_set_num_pre_condition_failure_app_psm(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  num_pre_condition_failure_app_psm_ = value;
}
inline void AP_PSM_APP_Consts::set_num_pre_condition_failure_app_psm(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_num_pre_condition_failure_app_psm(value);
  // @@protoc_insertion_point(field_set:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts.NUM_PRE_CONDITION_FAILURE_APP_PSM)
}

// -------------------------------------------------------------------

// AP_PSM_APP_Consts_array_port

// repeated .pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts data = 2466;
inline int AP_PSM_APP_Consts_array_port::_internal_data_size() const {
  return data_.size();
}
inline int AP_PSM_APP_Consts_array_port::data_size() const {
  return _internal_data_size();
}
inline void AP_PSM_APP_Consts_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts* AP_PSM_APP_Consts_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts >*
AP_PSM_APP_Consts_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts_array_port.data)
  return &data_;
}
inline const ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts& AP_PSM_APP_Consts_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts& AP_PSM_APP_Consts_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts* AP_PSM_APP_Consts_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts* AP_PSM_APP_Consts_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_psm_app::ap_psm_app_consts::AP_PSM_APP_Consts >&
AP_PSM_APP_Consts_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_psm_app.ap_psm_app_consts.AP_PSM_APP_Consts_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace ap_psm_app_consts
}  // namespace ap_psm_app
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fpsm_5fapp_2fap_5fpsm_5fapp_5fconsts_2eproto
