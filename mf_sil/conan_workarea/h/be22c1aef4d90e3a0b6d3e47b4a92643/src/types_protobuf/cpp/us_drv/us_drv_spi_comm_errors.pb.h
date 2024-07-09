// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_drv/us_drv_spi_comm_errors.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fspi_5fcomm_5ferrors_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fspi_5fcomm_5ferrors_2eproto

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
#include "us_drv/us_drv_error_status.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fspi_5fcomm_5ferrors_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fdrv_2fus_5fdrv_5fspi_5fcomm_5ferrors_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fdrv_2fus_5fdrv_5fspi_5fcomm_5ferrors_2eproto;
namespace pb {
namespace us_drv {
namespace us_drv_spi_comm_errors {
class UsDrvSpiCommErrors;
class UsDrvSpiCommErrorsDefaultTypeInternal;
extern UsDrvSpiCommErrorsDefaultTypeInternal _UsDrvSpiCommErrors_default_instance_;
class UsDrvSpiCommErrors_array_port;
class UsDrvSpiCommErrors_array_portDefaultTypeInternal;
extern UsDrvSpiCommErrors_array_portDefaultTypeInternal _UsDrvSpiCommErrors_array_port_default_instance_;
}  // namespace us_drv_spi_comm_errors
}  // namespace us_drv
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors>(Arena*);
template<> ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors_array_port* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_drv {
namespace us_drv_spi_comm_errors {

// ===================================================================

class UsDrvSpiCommErrors :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors) */ {
 public:
  UsDrvSpiCommErrors();
  virtual ~UsDrvSpiCommErrors();

  UsDrvSpiCommErrors(const UsDrvSpiCommErrors& from);
  UsDrvSpiCommErrors(UsDrvSpiCommErrors&& from) noexcept
    : UsDrvSpiCommErrors() {
    *this = ::std::move(from);
  }

  inline UsDrvSpiCommErrors& operator=(const UsDrvSpiCommErrors& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvSpiCommErrors& operator=(UsDrvSpiCommErrors&& from) noexcept {
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
  static const UsDrvSpiCommErrors& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvSpiCommErrors* internal_default_instance() {
    return reinterpret_cast<const UsDrvSpiCommErrors*>(
               &_UsDrvSpiCommErrors_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsDrvSpiCommErrors& a, UsDrvSpiCommErrors& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvSpiCommErrors* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvSpiCommErrors* New() const final {
    return CreateMaybeMessage<UsDrvSpiCommErrors>(nullptr);
  }

  UsDrvSpiCommErrors* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvSpiCommErrors>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvSpiCommErrors& from);
  void MergeFrom(const UsDrvSpiCommErrors& from);
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
  void InternalSwap(UsDrvSpiCommErrors* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fspi_5fcomm_5ferrors_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fspi_5fcomm_5ferrors_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kUndefinedCmdFieldNumber = 940,
    kSpiCrcBistFailedFieldNumber = 2085,
    kPrevCmdIncompleteFieldNumber = 2301,
    kCrcErrorDetectedFieldNumber = 3662,
  };
  // optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus undefinedCmd = 940;
  bool has_undefinedcmd() const;
  private:
  bool _internal_has_undefinedcmd() const;
  public:
  void clear_undefinedcmd();
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus undefinedcmd() const;
  void set_undefinedcmd(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  private:
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus _internal_undefinedcmd() const;
  void _internal_set_undefinedcmd(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  public:

  // optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus spiCrcBistFailed = 2085;
  bool has_spicrcbistfailed() const;
  private:
  bool _internal_has_spicrcbistfailed() const;
  public:
  void clear_spicrcbistfailed();
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus spicrcbistfailed() const;
  void set_spicrcbistfailed(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  private:
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus _internal_spicrcbistfailed() const;
  void _internal_set_spicrcbistfailed(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  public:

  // optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus prevCmdIncomplete = 2301;
  bool has_prevcmdincomplete() const;
  private:
  bool _internal_has_prevcmdincomplete() const;
  public:
  void clear_prevcmdincomplete();
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus prevcmdincomplete() const;
  void set_prevcmdincomplete(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  private:
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus _internal_prevcmdincomplete() const;
  void _internal_set_prevcmdincomplete(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  public:

  // optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus crcErrorDetected = 3662;
  bool has_crcerrordetected() const;
  private:
  bool _internal_has_crcerrordetected() const;
  public:
  void clear_crcerrordetected();
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus crcerrordetected() const;
  void set_crcerrordetected(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  private:
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus _internal_crcerrordetected() const;
  void _internal_set_crcerrordetected(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  public:

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int undefinedcmd_;
  int spicrcbistfailed_;
  int prevcmdincomplete_;
  int crcerrordetected_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fspi_5fcomm_5ferrors_2eproto;
};
// -------------------------------------------------------------------

class UsDrvSpiCommErrors_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors_array_port) */ {
 public:
  UsDrvSpiCommErrors_array_port();
  virtual ~UsDrvSpiCommErrors_array_port();

  UsDrvSpiCommErrors_array_port(const UsDrvSpiCommErrors_array_port& from);
  UsDrvSpiCommErrors_array_port(UsDrvSpiCommErrors_array_port&& from) noexcept
    : UsDrvSpiCommErrors_array_port() {
    *this = ::std::move(from);
  }

  inline UsDrvSpiCommErrors_array_port& operator=(const UsDrvSpiCommErrors_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvSpiCommErrors_array_port& operator=(UsDrvSpiCommErrors_array_port&& from) noexcept {
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
  static const UsDrvSpiCommErrors_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvSpiCommErrors_array_port* internal_default_instance() {
    return reinterpret_cast<const UsDrvSpiCommErrors_array_port*>(
               &_UsDrvSpiCommErrors_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsDrvSpiCommErrors_array_port& a, UsDrvSpiCommErrors_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvSpiCommErrors_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvSpiCommErrors_array_port* New() const final {
    return CreateMaybeMessage<UsDrvSpiCommErrors_array_port>(nullptr);
  }

  UsDrvSpiCommErrors_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvSpiCommErrors_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvSpiCommErrors_array_port& from);
  void MergeFrom(const UsDrvSpiCommErrors_array_port& from);
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
  void InternalSwap(UsDrvSpiCommErrors_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fspi_5fcomm_5ferrors_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fspi_5fcomm_5ferrors_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 679,
  };
  // repeated .pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors data = 679;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors >*
      mutable_data();
  private:
  const ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors& _internal_data(int index) const;
  ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors* _internal_add_data();
  public:
  const ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors& data(int index) const;
  ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors > data_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fspi_5fcomm_5ferrors_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsDrvSpiCommErrors

// optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus prevCmdIncomplete = 2301;
inline bool UsDrvSpiCommErrors::_internal_has_prevcmdincomplete() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool UsDrvSpiCommErrors::has_prevcmdincomplete() const {
  return _internal_has_prevcmdincomplete();
}
inline void UsDrvSpiCommErrors::clear_prevcmdincomplete() {
  prevcmdincomplete_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvSpiCommErrors::_internal_prevcmdincomplete() const {
  return static_cast< ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus >(prevcmdincomplete_);
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvSpiCommErrors::prevcmdincomplete() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors.prevCmdIncomplete)
  return _internal_prevcmdincomplete();
}
inline void UsDrvSpiCommErrors::_internal_set_prevcmdincomplete(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  assert(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  prevcmdincomplete_ = value;
}
inline void UsDrvSpiCommErrors::set_prevcmdincomplete(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  _internal_set_prevcmdincomplete(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors.prevCmdIncomplete)
}

// optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus crcErrorDetected = 3662;
inline bool UsDrvSpiCommErrors::_internal_has_crcerrordetected() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UsDrvSpiCommErrors::has_crcerrordetected() const {
  return _internal_has_crcerrordetected();
}
inline void UsDrvSpiCommErrors::clear_crcerrordetected() {
  crcerrordetected_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvSpiCommErrors::_internal_crcerrordetected() const {
  return static_cast< ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus >(crcerrordetected_);
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvSpiCommErrors::crcerrordetected() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors.crcErrorDetected)
  return _internal_crcerrordetected();
}
inline void UsDrvSpiCommErrors::_internal_set_crcerrordetected(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  assert(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus_IsValid(value));
  _has_bits_[0] |= 0x00000008u;
  crcerrordetected_ = value;
}
inline void UsDrvSpiCommErrors::set_crcerrordetected(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  _internal_set_crcerrordetected(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors.crcErrorDetected)
}

// optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus undefinedCmd = 940;
inline bool UsDrvSpiCommErrors::_internal_has_undefinedcmd() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool UsDrvSpiCommErrors::has_undefinedcmd() const {
  return _internal_has_undefinedcmd();
}
inline void UsDrvSpiCommErrors::clear_undefinedcmd() {
  undefinedcmd_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvSpiCommErrors::_internal_undefinedcmd() const {
  return static_cast< ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus >(undefinedcmd_);
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvSpiCommErrors::undefinedcmd() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors.undefinedCmd)
  return _internal_undefinedcmd();
}
inline void UsDrvSpiCommErrors::_internal_set_undefinedcmd(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  assert(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  undefinedcmd_ = value;
}
inline void UsDrvSpiCommErrors::set_undefinedcmd(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  _internal_set_undefinedcmd(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors.undefinedCmd)
}

// optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus spiCrcBistFailed = 2085;
inline bool UsDrvSpiCommErrors::_internal_has_spicrcbistfailed() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool UsDrvSpiCommErrors::has_spicrcbistfailed() const {
  return _internal_has_spicrcbistfailed();
}
inline void UsDrvSpiCommErrors::clear_spicrcbistfailed() {
  spicrcbistfailed_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvSpiCommErrors::_internal_spicrcbistfailed() const {
  return static_cast< ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus >(spicrcbistfailed_);
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvSpiCommErrors::spicrcbistfailed() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors.spiCrcBistFailed)
  return _internal_spicrcbistfailed();
}
inline void UsDrvSpiCommErrors::_internal_set_spicrcbistfailed(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  assert(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  spicrcbistfailed_ = value;
}
inline void UsDrvSpiCommErrors::set_spicrcbistfailed(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  _internal_set_spicrcbistfailed(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors.spiCrcBistFailed)
}

// -------------------------------------------------------------------

// UsDrvSpiCommErrors_array_port

// repeated .pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors data = 679;
inline int UsDrvSpiCommErrors_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsDrvSpiCommErrors_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsDrvSpiCommErrors_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors* UsDrvSpiCommErrors_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors >*
UsDrvSpiCommErrors_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors_array_port.data)
  return &data_;
}
inline const ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors& UsDrvSpiCommErrors_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors& UsDrvSpiCommErrors_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors* UsDrvSpiCommErrors_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors* UsDrvSpiCommErrors_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_spi_comm_errors::UsDrvSpiCommErrors >&
UsDrvSpiCommErrors_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_spi_comm_errors.UsDrvSpiCommErrors_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_drv_spi_comm_errors
}  // namespace us_drv
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fspi_5fcomm_5ferrors_2eproto
