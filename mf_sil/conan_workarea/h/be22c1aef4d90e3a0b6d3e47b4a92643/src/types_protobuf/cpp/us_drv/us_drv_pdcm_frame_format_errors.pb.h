// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_drv/us_drv_pdcm_frame_format_errors.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fpdcm_5fframe_5fformat_5ferrors_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fpdcm_5fframe_5fformat_5ferrors_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_us_5fdrv_2fus_5fdrv_5fpdcm_5fframe_5fformat_5ferrors_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fdrv_2fus_5fdrv_5fpdcm_5fframe_5fformat_5ferrors_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fdrv_2fus_5fdrv_5fpdcm_5fframe_5fformat_5ferrors_2eproto;
namespace pb {
namespace us_drv {
namespace us_drv_pdcm_frame_format_errors {
class UsDrvPdcmFrameFormatErrors;
class UsDrvPdcmFrameFormatErrorsDefaultTypeInternal;
extern UsDrvPdcmFrameFormatErrorsDefaultTypeInternal _UsDrvPdcmFrameFormatErrors_default_instance_;
class UsDrvPdcmFrameFormatErrors_array_port;
class UsDrvPdcmFrameFormatErrors_array_portDefaultTypeInternal;
extern UsDrvPdcmFrameFormatErrors_array_portDefaultTypeInternal _UsDrvPdcmFrameFormatErrors_array_port_default_instance_;
}  // namespace us_drv_pdcm_frame_format_errors
}  // namespace us_drv
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors>(Arena*);
template<> ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors_array_port* Arena::CreateMaybeMessage<::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_drv {
namespace us_drv_pdcm_frame_format_errors {

// ===================================================================

class UsDrvPdcmFrameFormatErrors :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors) */ {
 public:
  UsDrvPdcmFrameFormatErrors();
  virtual ~UsDrvPdcmFrameFormatErrors();

  UsDrvPdcmFrameFormatErrors(const UsDrvPdcmFrameFormatErrors& from);
  UsDrvPdcmFrameFormatErrors(UsDrvPdcmFrameFormatErrors&& from) noexcept
    : UsDrvPdcmFrameFormatErrors() {
    *this = ::std::move(from);
  }

  inline UsDrvPdcmFrameFormatErrors& operator=(const UsDrvPdcmFrameFormatErrors& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvPdcmFrameFormatErrors& operator=(UsDrvPdcmFrameFormatErrors&& from) noexcept {
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
  static const UsDrvPdcmFrameFormatErrors& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvPdcmFrameFormatErrors* internal_default_instance() {
    return reinterpret_cast<const UsDrvPdcmFrameFormatErrors*>(
               &_UsDrvPdcmFrameFormatErrors_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsDrvPdcmFrameFormatErrors& a, UsDrvPdcmFrameFormatErrors& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvPdcmFrameFormatErrors* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvPdcmFrameFormatErrors* New() const final {
    return CreateMaybeMessage<UsDrvPdcmFrameFormatErrors>(nullptr);
  }

  UsDrvPdcmFrameFormatErrors* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvPdcmFrameFormatErrors>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvPdcmFrameFormatErrors& from);
  void MergeFrom(const UsDrvPdcmFrameFormatErrors& from);
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
  void InternalSwap(UsDrvPdcmFrameFormatErrors* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fpdcm_5fframe_5fformat_5ferrors_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fpdcm_5fframe_5fformat_5ferrors_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataLossFieldNumber = 779,
    kPacketCountErrFieldNumber = 899,
    kNoNewDataFieldNumber = 2244,
    kDataBufferOvrFieldNumber = 3933,
  };
  // optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus dataLoss = 779;
  bool has_dataloss() const;
  private:
  bool _internal_has_dataloss() const;
  public:
  void clear_dataloss();
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus dataloss() const;
  void set_dataloss(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  private:
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus _internal_dataloss() const;
  void _internal_set_dataloss(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  public:

  // optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus packetCountErr = 899;
  bool has_packetcounterr() const;
  private:
  bool _internal_has_packetcounterr() const;
  public:
  void clear_packetcounterr();
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus packetcounterr() const;
  void set_packetcounterr(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  private:
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus _internal_packetcounterr() const;
  void _internal_set_packetcounterr(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  public:

  // optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus noNewData = 2244;
  bool has_nonewdata() const;
  private:
  bool _internal_has_nonewdata() const;
  public:
  void clear_nonewdata();
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus nonewdata() const;
  void set_nonewdata(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  private:
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus _internal_nonewdata() const;
  void _internal_set_nonewdata(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  public:

  // optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus dataBufferOvr = 3933;
  bool has_databufferovr() const;
  private:
  bool _internal_has_databufferovr() const;
  public:
  void clear_databufferovr();
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus databufferovr() const;
  void set_databufferovr(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  private:
  ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus _internal_databufferovr() const;
  void _internal_set_databufferovr(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value);
  public:

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int dataloss_;
  int packetcounterr_;
  int nonewdata_;
  int databufferovr_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fpdcm_5fframe_5fformat_5ferrors_2eproto;
};
// -------------------------------------------------------------------

class UsDrvPdcmFrameFormatErrors_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors_array_port) */ {
 public:
  UsDrvPdcmFrameFormatErrors_array_port();
  virtual ~UsDrvPdcmFrameFormatErrors_array_port();

  UsDrvPdcmFrameFormatErrors_array_port(const UsDrvPdcmFrameFormatErrors_array_port& from);
  UsDrvPdcmFrameFormatErrors_array_port(UsDrvPdcmFrameFormatErrors_array_port&& from) noexcept
    : UsDrvPdcmFrameFormatErrors_array_port() {
    *this = ::std::move(from);
  }

  inline UsDrvPdcmFrameFormatErrors_array_port& operator=(const UsDrvPdcmFrameFormatErrors_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsDrvPdcmFrameFormatErrors_array_port& operator=(UsDrvPdcmFrameFormatErrors_array_port&& from) noexcept {
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
  static const UsDrvPdcmFrameFormatErrors_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsDrvPdcmFrameFormatErrors_array_port* internal_default_instance() {
    return reinterpret_cast<const UsDrvPdcmFrameFormatErrors_array_port*>(
               &_UsDrvPdcmFrameFormatErrors_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsDrvPdcmFrameFormatErrors_array_port& a, UsDrvPdcmFrameFormatErrors_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsDrvPdcmFrameFormatErrors_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsDrvPdcmFrameFormatErrors_array_port* New() const final {
    return CreateMaybeMessage<UsDrvPdcmFrameFormatErrors_array_port>(nullptr);
  }

  UsDrvPdcmFrameFormatErrors_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsDrvPdcmFrameFormatErrors_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsDrvPdcmFrameFormatErrors_array_port& from);
  void MergeFrom(const UsDrvPdcmFrameFormatErrors_array_port& from);
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
  void InternalSwap(UsDrvPdcmFrameFormatErrors_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fdrv_2fus_5fdrv_5fpdcm_5fframe_5fformat_5ferrors_2eproto);
    return ::descriptor_table_us_5fdrv_2fus_5fdrv_5fpdcm_5fframe_5fformat_5ferrors_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1059,
  };
  // repeated .pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors data = 1059;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors >*
      mutable_data();
  private:
  const ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors& _internal_data(int index) const;
  ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors* _internal_add_data();
  public:
  const ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors& data(int index) const;
  ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors > data_;
  friend struct ::TableStruct_us_5fdrv_2fus_5fdrv_5fpdcm_5fframe_5fformat_5ferrors_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsDrvPdcmFrameFormatErrors

// optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus noNewData = 2244;
inline bool UsDrvPdcmFrameFormatErrors::_internal_has_nonewdata() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool UsDrvPdcmFrameFormatErrors::has_nonewdata() const {
  return _internal_has_nonewdata();
}
inline void UsDrvPdcmFrameFormatErrors::clear_nonewdata() {
  nonewdata_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvPdcmFrameFormatErrors::_internal_nonewdata() const {
  return static_cast< ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus >(nonewdata_);
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvPdcmFrameFormatErrors::nonewdata() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors.noNewData)
  return _internal_nonewdata();
}
inline void UsDrvPdcmFrameFormatErrors::_internal_set_nonewdata(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  assert(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  nonewdata_ = value;
}
inline void UsDrvPdcmFrameFormatErrors::set_nonewdata(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  _internal_set_nonewdata(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors.noNewData)
}

// optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus dataBufferOvr = 3933;
inline bool UsDrvPdcmFrameFormatErrors::_internal_has_databufferovr() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UsDrvPdcmFrameFormatErrors::has_databufferovr() const {
  return _internal_has_databufferovr();
}
inline void UsDrvPdcmFrameFormatErrors::clear_databufferovr() {
  databufferovr_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvPdcmFrameFormatErrors::_internal_databufferovr() const {
  return static_cast< ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus >(databufferovr_);
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvPdcmFrameFormatErrors::databufferovr() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors.dataBufferOvr)
  return _internal_databufferovr();
}
inline void UsDrvPdcmFrameFormatErrors::_internal_set_databufferovr(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  assert(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus_IsValid(value));
  _has_bits_[0] |= 0x00000008u;
  databufferovr_ = value;
}
inline void UsDrvPdcmFrameFormatErrors::set_databufferovr(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  _internal_set_databufferovr(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors.dataBufferOvr)
}

// optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus dataLoss = 779;
inline bool UsDrvPdcmFrameFormatErrors::_internal_has_dataloss() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool UsDrvPdcmFrameFormatErrors::has_dataloss() const {
  return _internal_has_dataloss();
}
inline void UsDrvPdcmFrameFormatErrors::clear_dataloss() {
  dataloss_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvPdcmFrameFormatErrors::_internal_dataloss() const {
  return static_cast< ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus >(dataloss_);
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvPdcmFrameFormatErrors::dataloss() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors.dataLoss)
  return _internal_dataloss();
}
inline void UsDrvPdcmFrameFormatErrors::_internal_set_dataloss(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  assert(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  dataloss_ = value;
}
inline void UsDrvPdcmFrameFormatErrors::set_dataloss(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  _internal_set_dataloss(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors.dataLoss)
}

// optional .pb.us_drv.us_drv_error_status.UsDrvErrorStatus packetCountErr = 899;
inline bool UsDrvPdcmFrameFormatErrors::_internal_has_packetcounterr() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool UsDrvPdcmFrameFormatErrors::has_packetcounterr() const {
  return _internal_has_packetcounterr();
}
inline void UsDrvPdcmFrameFormatErrors::clear_packetcounterr() {
  packetcounterr_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvPdcmFrameFormatErrors::_internal_packetcounterr() const {
  return static_cast< ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus >(packetcounterr_);
}
inline ::pb::us_drv::us_drv_error_status::UsDrvErrorStatus UsDrvPdcmFrameFormatErrors::packetcounterr() const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors.packetCountErr)
  return _internal_packetcounterr();
}
inline void UsDrvPdcmFrameFormatErrors::_internal_set_packetcounterr(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  assert(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  packetcounterr_ = value;
}
inline void UsDrvPdcmFrameFormatErrors::set_packetcounterr(::pb::us_drv::us_drv_error_status::UsDrvErrorStatus value) {
  _internal_set_packetcounterr(value);
  // @@protoc_insertion_point(field_set:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors.packetCountErr)
}

// -------------------------------------------------------------------

// UsDrvPdcmFrameFormatErrors_array_port

// repeated .pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors data = 1059;
inline int UsDrvPdcmFrameFormatErrors_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsDrvPdcmFrameFormatErrors_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsDrvPdcmFrameFormatErrors_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors* UsDrvPdcmFrameFormatErrors_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors >*
UsDrvPdcmFrameFormatErrors_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors_array_port.data)
  return &data_;
}
inline const ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors& UsDrvPdcmFrameFormatErrors_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors& UsDrvPdcmFrameFormatErrors_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors* UsDrvPdcmFrameFormatErrors_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors* UsDrvPdcmFrameFormatErrors_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_drv::us_drv_pdcm_frame_format_errors::UsDrvPdcmFrameFormatErrors >&
UsDrvPdcmFrameFormatErrors_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_drv.us_drv_pdcm_frame_format_errors.UsDrvPdcmFrameFormatErrors_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_drv_pdcm_frame_format_errors
}  // namespace us_drv
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fdrv_2fus_5fdrv_5fpdcm_5fframe_5fformat_5ferrors_2eproto
