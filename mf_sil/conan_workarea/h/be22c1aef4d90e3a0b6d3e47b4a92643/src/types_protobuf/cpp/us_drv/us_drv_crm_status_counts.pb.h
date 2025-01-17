// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_drv/us_drv_crm_status_counts.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fcrm_5fstatus_5fcounts_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fcrm_5fstatus_5fcounts_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fcrm_5fstatus_5fcounts_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fdrv_2fus_5fdrv_5fcrm_5fstatus_5fcounts_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fdrv_2fus_5fdrv_5fcrm_5fstatus_5fcounts_2eproto;
namespace pb {
namespace us_drv {
namespace us_drv_crm_status_counts {
class UsDrvCrmStatusCounts;
class UsDrvCrmStatusCountsDefaultTypeInternal;
extern UsDrvCrmStatusCountsDefaultTypeInternal _UsDrvCrmStatusCounts_default_instance_;
class UsDrvCrmStatusCounts_array_port;
class UsDrvCrmStatusCounts_array_portDefaultTypeInternal;
extern UsDrvCrmStatusCounts_array_portDefaultTypeInternal _UsDrvCrmStatusCounts_array_port_default_instance_;
}  // namespace us_drv_crm_status_counts
}  // namespace us_drv
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts>(Arena*);
template<> ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts_array_port* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_drv {
namespace us_drv_crm_status_counts {

// ===================================================================

class UsDrvCrmStatusCounts :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts) */ {
 public:
  UsDrvCrmStatusCounts();
  virtual ~UsDrvCrmStatusCounts();

  UsDrvCrmStatusCounts(const UsDrvCrmStatusCounts& from);
  UsDrvCrmStatusCounts(UsDrvCrmStatusCounts&& from) noexcept
    : UsDrvCrmStatusCounts() {
    *this = ::std::move(from);
  }

  inline UsDrvCrmStatusCounts& operator=(const UsDrvCrmStatusCounts& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvCrmStatusCounts& operator=(UsDrvCrmStatusCounts&& from) noexcept {
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
  static const UsDrvCrmStatusCounts& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvCrmStatusCounts* internal_default_instance() {
    return reinterpret_cast<const UsDrvCrmStatusCounts*>(
               &_UsDrvCrmStatusCounts_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsDrvCrmStatusCounts& a, UsDrvCrmStatusCounts& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvCrmStatusCounts* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvCrmStatusCounts* New() const final {
    return CreateMaybeMessage<UsDrvCrmStatusCounts>(nullptr);
  }

  UsDrvCrmStatusCounts* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvCrmStatusCounts>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvCrmStatusCounts& from);
  void MergeFrom(const UsDrvCrmStatusCounts& from);
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
  void InternalSwap(UsDrvCrmStatusCounts* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fcrm_5fstatus_5fcounts_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fcrm_5fstatus_5fcounts_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kOverWrittenFieldNumber = 230,
    kDataOkFieldNumber = 367,
    kReservedFieldNumber = 600,
    kTimeoutErrorFieldNumber = 809,
    kCrmErrorFieldNumber = 1391,
    kCrcErrorFieldNumber = 1484,
    kLengthErrorFieldNumber = 3286,
    kTimingErrorFieldNumber = 3704,
  };
  // optional uint32 overWritten = 230;
  bool has_overwritten() const;
  private:
  bool _internal_has_overwritten() const;
  public:
  void clear_overwritten();
  ::PROTOBUF_NAMESPACE_ID::uint32 overwritten() const;
  void set_overwritten(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_overwritten() const;
  void _internal_set_overwritten(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 dataOk = 367;
  bool has_dataok() const;
  private:
  bool _internal_has_dataok() const;
  public:
  void clear_dataok();
  ::PROTOBUF_NAMESPACE_ID::uint32 dataok() const;
  void set_dataok(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_dataok() const;
  void _internal_set_dataok(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 reserved = 600;
  bool has_reserved() const;
  private:
  bool _internal_has_reserved() const;
  public:
  void clear_reserved();
  ::PROTOBUF_NAMESPACE_ID::uint32 reserved() const;
  void set_reserved(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_reserved() const;
  void _internal_set_reserved(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 timeoutError = 809;
  bool has_timeouterror() const;
  private:
  bool _internal_has_timeouterror() const;
  public:
  void clear_timeouterror();
  ::PROTOBUF_NAMESPACE_ID::uint32 timeouterror() const;
  void set_timeouterror(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_timeouterror() const;
  void _internal_set_timeouterror(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 crmError = 1391;
  bool has_crmerror() const;
  private:
  bool _internal_has_crmerror() const;
  public:
  void clear_crmerror();
  ::PROTOBUF_NAMESPACE_ID::uint32 crmerror() const;
  void set_crmerror(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_crmerror() const;
  void _internal_set_crmerror(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 crcError = 1484;
  bool has_crcerror() const;
  private:
  bool _internal_has_crcerror() const;
  public:
  void clear_crcerror();
  ::PROTOBUF_NAMESPACE_ID::uint32 crcerror() const;
  void set_crcerror(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_crcerror() const;
  void _internal_set_crcerror(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 lengthError = 3286;
  bool has_lengtherror() const;
  private:
  bool _internal_has_lengtherror() const;
  public:
  void clear_lengtherror();
  ::PROTOBUF_NAMESPACE_ID::uint32 lengtherror() const;
  void set_lengtherror(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_lengtherror() const;
  void _internal_set_lengtherror(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 timingError = 3704;
  bool has_timingerror() const;
  private:
  bool _internal_has_timingerror() const;
  public:
  void clear_timingerror();
  ::PROTOBUF_NAMESPACE_ID::uint32 timingerror() const;
  void set_timingerror(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_timingerror() const;
  void _internal_set_timingerror(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 overwritten_;
  ::PROTOBUF_NAMESPACE_ID::uint32 dataok_;
  ::PROTOBUF_NAMESPACE_ID::uint32 reserved_;
  ::PROTOBUF_NAMESPACE_ID::uint32 timeouterror_;
  ::PROTOBUF_NAMESPACE_ID::uint32 crmerror_;
  ::PROTOBUF_NAMESPACE_ID::uint32 crcerror_;
  ::PROTOBUF_NAMESPACE_ID::uint32 lengtherror_;
  ::PROTOBUF_NAMESPACE_ID::uint32 timingerror_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fcrm_5fstatus_5fcounts_2eproto;
};
// -------------------------------------------------------------------

class UsDrvCrmStatusCounts_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts_array_port) */ {
 public:
  UsDrvCrmStatusCounts_array_port();
  virtual ~UsDrvCrmStatusCounts_array_port();

  UsDrvCrmStatusCounts_array_port(const UsDrvCrmStatusCounts_array_port& from);
  UsDrvCrmStatusCounts_array_port(UsDrvCrmStatusCounts_array_port&& from) noexcept
    : UsDrvCrmStatusCounts_array_port() {
    *this = ::std::move(from);
  }

  inline UsDrvCrmStatusCounts_array_port& operator=(const UsDrvCrmStatusCounts_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvCrmStatusCounts_array_port& operator=(UsDrvCrmStatusCounts_array_port&& from) noexcept {
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
  static const UsDrvCrmStatusCounts_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvCrmStatusCounts_array_port* internal_default_instance() {
    return reinterpret_cast<const UsDrvCrmStatusCounts_array_port*>(
               &_UsDrvCrmStatusCounts_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsDrvCrmStatusCounts_array_port& a, UsDrvCrmStatusCounts_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvCrmStatusCounts_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvCrmStatusCounts_array_port* New() const final {
    return CreateMaybeMessage<UsDrvCrmStatusCounts_array_port>(nullptr);
  }

  UsDrvCrmStatusCounts_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvCrmStatusCounts_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvCrmStatusCounts_array_port& from);
  void MergeFrom(const UsDrvCrmStatusCounts_array_port& from);
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
  void InternalSwap(UsDrvCrmStatusCounts_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fcrm_5fstatus_5fcounts_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fcrm_5fstatus_5fcounts_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 135,
  };
  // repeated .pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts data = 135;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts >*
      mutable_data();
  private:
  const ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts& _internal_data(int index) const;
  ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* _internal_add_data();
  public:
  const ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts& data(int index) const;
  ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts > data_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fcrm_5fstatus_5fcounts_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsDrvCrmStatusCounts

// optional uint32 dataOk = 367;
inline bool UsDrvCrmStatusCounts::_internal_has_dataok() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool UsDrvCrmStatusCounts::has_dataok() const {
  return _internal_has_dataok();
}
inline void UsDrvCrmStatusCounts::clear_dataok() {
  dataok_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::_internal_dataok() const {
  return dataok_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::dataok() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.dataOk)
  return _internal_dataok();
}
inline void UsDrvCrmStatusCounts::_internal_set_dataok(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  dataok_ = value;
}
inline void UsDrvCrmStatusCounts::set_dataok(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_dataok(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.dataOk)
}

// optional uint32 overWritten = 230;
inline bool UsDrvCrmStatusCounts::_internal_has_overwritten() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool UsDrvCrmStatusCounts::has_overwritten() const {
  return _internal_has_overwritten();
}
inline void UsDrvCrmStatusCounts::clear_overwritten() {
  overwritten_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::_internal_overwritten() const {
  return overwritten_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::overwritten() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.overWritten)
  return _internal_overwritten();
}
inline void UsDrvCrmStatusCounts::_internal_set_overwritten(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  overwritten_ = value;
}
inline void UsDrvCrmStatusCounts::set_overwritten(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_overwritten(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.overWritten)
}

// optional uint32 crmError = 1391;
inline bool UsDrvCrmStatusCounts::_internal_has_crmerror() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool UsDrvCrmStatusCounts::has_crmerror() const {
  return _internal_has_crmerror();
}
inline void UsDrvCrmStatusCounts::clear_crmerror() {
  crmerror_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::_internal_crmerror() const {
  return crmerror_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::crmerror() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.crmError)
  return _internal_crmerror();
}
inline void UsDrvCrmStatusCounts::_internal_set_crmerror(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  crmerror_ = value;
}
inline void UsDrvCrmStatusCounts::set_crmerror(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_crmerror(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.crmError)
}

// optional uint32 reserved = 600;
inline bool UsDrvCrmStatusCounts::_internal_has_reserved() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool UsDrvCrmStatusCounts::has_reserved() const {
  return _internal_has_reserved();
}
inline void UsDrvCrmStatusCounts::clear_reserved() {
  reserved_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::_internal_reserved() const {
  return reserved_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::reserved() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.reserved)
  return _internal_reserved();
}
inline void UsDrvCrmStatusCounts::_internal_set_reserved(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  reserved_ = value;
}
inline void UsDrvCrmStatusCounts::set_reserved(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_reserved(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.reserved)
}

// optional uint32 lengthError = 3286;
inline bool UsDrvCrmStatusCounts::_internal_has_lengtherror() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool UsDrvCrmStatusCounts::has_lengtherror() const {
  return _internal_has_lengtherror();
}
inline void UsDrvCrmStatusCounts::clear_lengtherror() {
  lengtherror_ = 0u;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::_internal_lengtherror() const {
  return lengtherror_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::lengtherror() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.lengthError)
  return _internal_lengtherror();
}
inline void UsDrvCrmStatusCounts::_internal_set_lengtherror(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000040u;
  lengtherror_ = value;
}
inline void UsDrvCrmStatusCounts::set_lengtherror(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_lengtherror(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.lengthError)
}

// optional uint32 timingError = 3704;
inline bool UsDrvCrmStatusCounts::_internal_has_timingerror() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool UsDrvCrmStatusCounts::has_timingerror() const {
  return _internal_has_timingerror();
}
inline void UsDrvCrmStatusCounts::clear_timingerror() {
  timingerror_ = 0u;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::_internal_timingerror() const {
  return timingerror_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::timingerror() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.timingError)
  return _internal_timingerror();
}
inline void UsDrvCrmStatusCounts::_internal_set_timingerror(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000080u;
  timingerror_ = value;
}
inline void UsDrvCrmStatusCounts::set_timingerror(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_timingerror(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.timingError)
}

// optional uint32 timeoutError = 809;
inline bool UsDrvCrmStatusCounts::_internal_has_timeouterror() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UsDrvCrmStatusCounts::has_timeouterror() const {
  return _internal_has_timeouterror();
}
inline void UsDrvCrmStatusCounts::clear_timeouterror() {
  timeouterror_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::_internal_timeouterror() const {
  return timeouterror_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::timeouterror() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.timeoutError)
  return _internal_timeouterror();
}
inline void UsDrvCrmStatusCounts::_internal_set_timeouterror(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  timeouterror_ = value;
}
inline void UsDrvCrmStatusCounts::set_timeouterror(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_timeouterror(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.timeoutError)
}

// optional uint32 crcError = 1484;
inline bool UsDrvCrmStatusCounts::_internal_has_crcerror() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool UsDrvCrmStatusCounts::has_crcerror() const {
  return _internal_has_crcerror();
}
inline void UsDrvCrmStatusCounts::clear_crcerror() {
  crcerror_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::_internal_crcerror() const {
  return crcerror_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 UsDrvCrmStatusCounts::crcerror() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.crcError)
  return _internal_crcerror();
}
inline void UsDrvCrmStatusCounts::_internal_set_crcerror(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  crcerror_ = value;
}
inline void UsDrvCrmStatusCounts::set_crcerror(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_crcerror(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts.crcError)
}

// -------------------------------------------------------------------

// UsDrvCrmStatusCounts_array_port

// repeated .pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts data = 135;
inline int UsDrvCrmStatusCounts_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsDrvCrmStatusCounts_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsDrvCrmStatusCounts_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* UsDrvCrmStatusCounts_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts >*
UsDrvCrmStatusCounts_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts_array_port.data)
  return &data_;
}
inline const ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts& UsDrvCrmStatusCounts_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts& UsDrvCrmStatusCounts_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* UsDrvCrmStatusCounts_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts* UsDrvCrmStatusCounts_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_crm_status_counts::UsDrvCrmStatusCounts >&
UsDrvCrmStatusCounts_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_crm_status_counts.UsDrvCrmStatusCounts_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_drv_crm_status_counts
}  // namespace us_drv
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fcrm_5fstatus_5fcounts_2eproto
