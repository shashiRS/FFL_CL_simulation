// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_hmih/lsca_warnings.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2flsca_5fwarnings_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2flsca_5fwarnings_2eproto

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
#include "mf_lsca/lsca_warning_status.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fhmih_2flsca_5fwarnings_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fhmih_2flsca_5fwarnings_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fhmih_2flsca_5fwarnings_2eproto;
namespace pb {
namespace mf_hmih {
namespace lsca_warnings {
class LscaWarnings;
class LscaWarningsDefaultTypeInternal;
extern LscaWarningsDefaultTypeInternal _LscaWarnings_default_instance_;
class LscaWarnings_array_port;
class LscaWarnings_array_portDefaultTypeInternal;
extern LscaWarnings_array_portDefaultTypeInternal _LscaWarnings_array_port_default_instance_;
}  // namespace lsca_warnings
}  // namespace mf_hmih
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_hmih::lsca_warnings::LscaWarnings* Arena::CreateMaybeMessage<::pb::mf_hmih::lsca_warnings::LscaWarnings>(Arena*);
template<> ::pb::mf_hmih::lsca_warnings::LscaWarnings_array_port* Arena::CreateMaybeMessage<::pb::mf_hmih::lsca_warnings::LscaWarnings_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_hmih {
namespace lsca_warnings {

// ===================================================================

class LscaWarnings :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.lsca_warnings.LscaWarnings) */ {
 public:
  LscaWarnings();
  virtual ~LscaWarnings();

  LscaWarnings(const LscaWarnings& from);
  LscaWarnings(LscaWarnings&& from) noexcept
    : LscaWarnings() {
    *this = ::std::move(from);
  }

  inline LscaWarnings& operator=(const LscaWarnings& from) {
    CopyFrom(from);
    return *this;
  }
  inline LscaWarnings& operator=(LscaWarnings&& from) noexcept {
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
  static const LscaWarnings& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LscaWarnings* internal_default_instance() {
    return reinterpret_cast<const LscaWarnings*>(
               &_LscaWarnings_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LscaWarnings& a, LscaWarnings& b) {
    a.Swap(&b);
  }
  inline void Swap(LscaWarnings* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LscaWarnings* New() const final {
    return CreateMaybeMessage<LscaWarnings>(nullptr);
  }

  LscaWarnings* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LscaWarnings>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LscaWarnings& from);
  void MergeFrom(const LscaWarnings& from);
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
  void InternalSwap(LscaWarnings* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.lsca_warnings.LscaWarnings";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2flsca_5fwarnings_2eproto);
    return ::descriptor_table_mf_5fhmih_2flsca_5fwarnings_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kWarningWheelNuFieldNumber = 2234,
    kWarningBodyNuFieldNumber = 341,
    kWarningObjectNuFieldNumber = 705,
  };
  // optional .pb.mf_lsca.lsca_warning_status.LSCA_WARNING_STATUS warningWheel_nu = 2234;
  bool has_warningwheel_nu() const;
  private:
  bool _internal_has_warningwheel_nu() const;
  public:
  void clear_warningwheel_nu();
  ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS warningwheel_nu() const;
  void set_warningwheel_nu(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS value);
  private:
  ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS _internal_warningwheel_nu() const;
  void _internal_set_warningwheel_nu(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS value);
  public:

  // optional .pb.mf_lsca.lsca_warning_status.LSCA_WARNING_STATUS warningBody_nu = 341;
  bool has_warningbody_nu() const;
  private:
  bool _internal_has_warningbody_nu() const;
  public:
  void clear_warningbody_nu();
  ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS warningbody_nu() const;
  void set_warningbody_nu(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS value);
  private:
  ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS _internal_warningbody_nu() const;
  void _internal_set_warningbody_nu(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS value);
  public:

  // optional .pb.mf_lsca.lsca_warning_status.LSCA_WARNING_STATUS warningObject_nu = 705;
  bool has_warningobject_nu() const;
  private:
  bool _internal_has_warningobject_nu() const;
  public:
  void clear_warningobject_nu();
  ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS warningobject_nu() const;
  void set_warningobject_nu(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS value);
  private:
  ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS _internal_warningobject_nu() const;
  void _internal_set_warningobject_nu(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.lsca_warnings.LscaWarnings)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int warningwheel_nu_;
  int warningbody_nu_;
  int warningobject_nu_;
  friend struct ::TableStruct_mf_5fhmih_2flsca_5fwarnings_2eproto;
};
// -------------------------------------------------------------------

class LscaWarnings_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.lsca_warnings.LscaWarnings_array_port) */ {
 public:
  LscaWarnings_array_port();
  virtual ~LscaWarnings_array_port();

  LscaWarnings_array_port(const LscaWarnings_array_port& from);
  LscaWarnings_array_port(LscaWarnings_array_port&& from) noexcept
    : LscaWarnings_array_port() {
    *this = ::std::move(from);
  }

  inline LscaWarnings_array_port& operator=(const LscaWarnings_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline LscaWarnings_array_port& operator=(LscaWarnings_array_port&& from) noexcept {
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
  static const LscaWarnings_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LscaWarnings_array_port* internal_default_instance() {
    return reinterpret_cast<const LscaWarnings_array_port*>(
               &_LscaWarnings_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(LscaWarnings_array_port& a, LscaWarnings_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(LscaWarnings_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LscaWarnings_array_port* New() const final {
    return CreateMaybeMessage<LscaWarnings_array_port>(nullptr);
  }

  LscaWarnings_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LscaWarnings_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LscaWarnings_array_port& from);
  void MergeFrom(const LscaWarnings_array_port& from);
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
  void InternalSwap(LscaWarnings_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.lsca_warnings.LscaWarnings_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2flsca_5fwarnings_2eproto);
    return ::descriptor_table_mf_5fhmih_2flsca_5fwarnings_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3630,
  };
  // repeated .pb.mf_hmih.lsca_warnings.LscaWarnings data = 3630;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_hmih::lsca_warnings::LscaWarnings* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::lsca_warnings::LscaWarnings >*
      mutable_data();
  private:
  const ::pb::mf_hmih::lsca_warnings::LscaWarnings& _internal_data(int index) const;
  ::pb::mf_hmih::lsca_warnings::LscaWarnings* _internal_add_data();
  public:
  const ::pb::mf_hmih::lsca_warnings::LscaWarnings& data(int index) const;
  ::pb::mf_hmih::lsca_warnings::LscaWarnings* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::lsca_warnings::LscaWarnings >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.lsca_warnings.LscaWarnings_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::lsca_warnings::LscaWarnings > data_;
  friend struct ::TableStruct_mf_5fhmih_2flsca_5fwarnings_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LscaWarnings

// optional .pb.mf_lsca.lsca_warning_status.LSCA_WARNING_STATUS warningBody_nu = 341;
inline bool LscaWarnings::_internal_has_warningbody_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool LscaWarnings::has_warningbody_nu() const {
  return _internal_has_warningbody_nu();
}
inline void LscaWarnings::clear_warningbody_nu() {
  warningbody_nu_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS LscaWarnings::_internal_warningbody_nu() const {
  return static_cast< ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS >(warningbody_nu_);
}
inline ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS LscaWarnings::warningbody_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.lsca_warnings.LscaWarnings.warningBody_nu)
  return _internal_warningbody_nu();
}
inline void LscaWarnings::_internal_set_warningbody_nu(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS value) {
  assert(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  warningbody_nu_ = value;
}
inline void LscaWarnings::set_warningbody_nu(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS value) {
  _internal_set_warningbody_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.lsca_warnings.LscaWarnings.warningBody_nu)
}

// optional .pb.mf_lsca.lsca_warning_status.LSCA_WARNING_STATUS warningWheel_nu = 2234;
inline bool LscaWarnings::_internal_has_warningwheel_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool LscaWarnings::has_warningwheel_nu() const {
  return _internal_has_warningwheel_nu();
}
inline void LscaWarnings::clear_warningwheel_nu() {
  warningwheel_nu_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS LscaWarnings::_internal_warningwheel_nu() const {
  return static_cast< ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS >(warningwheel_nu_);
}
inline ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS LscaWarnings::warningwheel_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.lsca_warnings.LscaWarnings.warningWheel_nu)
  return _internal_warningwheel_nu();
}
inline void LscaWarnings::_internal_set_warningwheel_nu(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS value) {
  assert(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  warningwheel_nu_ = value;
}
inline void LscaWarnings::set_warningwheel_nu(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS value) {
  _internal_set_warningwheel_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.lsca_warnings.LscaWarnings.warningWheel_nu)
}

// optional .pb.mf_lsca.lsca_warning_status.LSCA_WARNING_STATUS warningObject_nu = 705;
inline bool LscaWarnings::_internal_has_warningobject_nu() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool LscaWarnings::has_warningobject_nu() const {
  return _internal_has_warningobject_nu();
}
inline void LscaWarnings::clear_warningobject_nu() {
  warningobject_nu_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS LscaWarnings::_internal_warningobject_nu() const {
  return static_cast< ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS >(warningobject_nu_);
}
inline ::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS LscaWarnings::warningobject_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.lsca_warnings.LscaWarnings.warningObject_nu)
  return _internal_warningobject_nu();
}
inline void LscaWarnings::_internal_set_warningobject_nu(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS value) {
  assert(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  warningobject_nu_ = value;
}
inline void LscaWarnings::set_warningobject_nu(::pb::mf_lsca::lsca_warning_status::LSCA_WARNING_STATUS value) {
  _internal_set_warningobject_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.lsca_warnings.LscaWarnings.warningObject_nu)
}

// -------------------------------------------------------------------

// LscaWarnings_array_port

// repeated .pb.mf_hmih.lsca_warnings.LscaWarnings data = 3630;
inline int LscaWarnings_array_port::_internal_data_size() const {
  return data_.size();
}
inline int LscaWarnings_array_port::data_size() const {
  return _internal_data_size();
}
inline void LscaWarnings_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_hmih::lsca_warnings::LscaWarnings* LscaWarnings_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.lsca_warnings.LscaWarnings_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::lsca_warnings::LscaWarnings >*
LscaWarnings_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.lsca_warnings.LscaWarnings_array_port.data)
  return &data_;
}
inline const ::pb::mf_hmih::lsca_warnings::LscaWarnings& LscaWarnings_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_hmih::lsca_warnings::LscaWarnings& LscaWarnings_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.lsca_warnings.LscaWarnings_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_hmih::lsca_warnings::LscaWarnings* LscaWarnings_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_hmih::lsca_warnings::LscaWarnings* LscaWarnings_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_hmih.lsca_warnings.LscaWarnings_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::lsca_warnings::LscaWarnings >&
LscaWarnings_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.lsca_warnings.LscaWarnings_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace lsca_warnings
}  // namespace mf_hmih
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2flsca_5fwarnings_2eproto
