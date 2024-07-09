// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_tp/parking_box_debug_info.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5ftp_2fparking_5fbox_5fdebug_5finfo_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5ftp_2fparking_5fbox_5fdebug_5finfo_2eproto

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
#include "ap_tp/simple_debug_parking_box.pb.h"
#include "ap_tp/delimiter_distances.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5ftp_2fparking_5fbox_5fdebug_5finfo_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5ftp_2fparking_5fbox_5fdebug_5finfo_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftp_2fparking_5fbox_5fdebug_5finfo_2eproto;
namespace pb {
namespace ap_tp {
namespace parking_box_debug_info {
class ParkingBoxDebugInfo;
class ParkingBoxDebugInfoDefaultTypeInternal;
extern ParkingBoxDebugInfoDefaultTypeInternal _ParkingBoxDebugInfo_default_instance_;
class ParkingBoxDebugInfo_array_port;
class ParkingBoxDebugInfo_array_portDefaultTypeInternal;
extern ParkingBoxDebugInfo_array_portDefaultTypeInternal _ParkingBoxDebugInfo_array_port_default_instance_;
}  // namespace parking_box_debug_info
}  // namespace ap_tp
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo* Arena::CreateMaybeMessage<::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo>(Arena*);
template<> ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo_array_port* Arena::CreateMaybeMessage<::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_tp {
namespace parking_box_debug_info {

// ===================================================================

class ParkingBoxDebugInfo :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo) */ {
 public:
  ParkingBoxDebugInfo();
  virtual ~ParkingBoxDebugInfo();

  ParkingBoxDebugInfo(const ParkingBoxDebugInfo& from);
  ParkingBoxDebugInfo(ParkingBoxDebugInfo&& from) noexcept
    : ParkingBoxDebugInfo() {
    *this = ::std::move(from);
  }

  inline ParkingBoxDebugInfo& operator=(const ParkingBoxDebugInfo& from) {
    CopyFrom(from);
    return *this;
  }
  inline ParkingBoxDebugInfo& operator=(ParkingBoxDebugInfo&& from) noexcept {
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
  static const ParkingBoxDebugInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ParkingBoxDebugInfo* internal_default_instance() {
    return reinterpret_cast<const ParkingBoxDebugInfo*>(
               &_ParkingBoxDebugInfo_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ParkingBoxDebugInfo& a, ParkingBoxDebugInfo& b) {
    a.Swap(&b);
  }
  inline void Swap(ParkingBoxDebugInfo* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ParkingBoxDebugInfo* New() const final {
    return CreateMaybeMessage<ParkingBoxDebugInfo>(nullptr);
  }

  ParkingBoxDebugInfo* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ParkingBoxDebugInfo>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ParkingBoxDebugInfo& from);
  void MergeFrom(const ParkingBoxDebugInfo& from);
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
  void InternalSwap(ParkingBoxDebugInfo* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftp_2fparking_5fbox_5fdebug_5finfo_2eproto);
    return ::descriptor_table_ap_5ftp_2fparking_5fbox_5fdebug_5finfo_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kComfParkingBoxFieldNumber = 2471,
    kMaxParkingBoxFieldNumber = 2943,
    kDelimiterDistancesFieldNumber = 3359,
    kYawAngleRadFieldNumber = 566,
    kLeftEdgeIsFrontNuFieldNumber = 2311,
    kIsResizedNuFieldNumber = 600,
    kHasAngleNuFieldNumber = 1713,
    kHasPositionNuFieldNumber = 651,
    kTargetPosYMFieldNumber = 2644,
    kTargetPosXMFieldNumber = 3428,
  };
  // optional .pb.ap_tp.simple_debug_parking_box.SimpleDebugParkingBox comfParkingBox = 2471;
  bool has_comfparkingbox() const;
  private:
  bool _internal_has_comfparkingbox() const;
  public:
  void clear_comfparkingbox();
  const ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox& comfparkingbox() const;
  ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* release_comfparkingbox();
  ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* mutable_comfparkingbox();
  void set_allocated_comfparkingbox(::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* comfparkingbox);
  private:
  const ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox& _internal_comfparkingbox() const;
  ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* _internal_mutable_comfparkingbox();
  public:

  // optional .pb.ap_tp.simple_debug_parking_box.SimpleDebugParkingBox maxParkingBox = 2943;
  bool has_maxparkingbox() const;
  private:
  bool _internal_has_maxparkingbox() const;
  public:
  void clear_maxparkingbox();
  const ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox& maxparkingbox() const;
  ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* release_maxparkingbox();
  ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* mutable_maxparkingbox();
  void set_allocated_maxparkingbox(::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* maxparkingbox);
  private:
  const ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox& _internal_maxparkingbox() const;
  ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* _internal_mutable_maxparkingbox();
  public:

  // optional .pb.ap_tp.delimiter_distances.DelimiterDistances delimiterDistances = 3359;
  bool has_delimiterdistances() const;
  private:
  bool _internal_has_delimiterdistances() const;
  public:
  void clear_delimiterdistances();
  const ::pb::ap_tp::delimiter_distances::DelimiterDistances& delimiterdistances() const;
  ::pb::ap_tp::delimiter_distances::DelimiterDistances* release_delimiterdistances();
  ::pb::ap_tp::delimiter_distances::DelimiterDistances* mutable_delimiterdistances();
  void set_allocated_delimiterdistances(::pb::ap_tp::delimiter_distances::DelimiterDistances* delimiterdistances);
  private:
  const ::pb::ap_tp::delimiter_distances::DelimiterDistances& _internal_delimiterdistances() const;
  ::pb::ap_tp::delimiter_distances::DelimiterDistances* _internal_mutable_delimiterdistances();
  public:

  // optional float yawAngle_rad = 566;
  bool has_yawangle_rad() const;
  private:
  bool _internal_has_yawangle_rad() const;
  public:
  void clear_yawangle_rad();
  float yawangle_rad() const;
  void set_yawangle_rad(float value);
  private:
  float _internal_yawangle_rad() const;
  void _internal_set_yawangle_rad(float value);
  public:

  // optional bool leftEdgeIsFront_nu = 2311;
  bool has_leftedgeisfront_nu() const;
  private:
  bool _internal_has_leftedgeisfront_nu() const;
  public:
  void clear_leftedgeisfront_nu();
  bool leftedgeisfront_nu() const;
  void set_leftedgeisfront_nu(bool value);
  private:
  bool _internal_leftedgeisfront_nu() const;
  void _internal_set_leftedgeisfront_nu(bool value);
  public:

  // optional bool isResized_nu = 600;
  bool has_isresized_nu() const;
  private:
  bool _internal_has_isresized_nu() const;
  public:
  void clear_isresized_nu();
  bool isresized_nu() const;
  void set_isresized_nu(bool value);
  private:
  bool _internal_isresized_nu() const;
  void _internal_set_isresized_nu(bool value);
  public:

  // optional bool hasAngle_nu = 1713;
  bool has_hasangle_nu() const;
  private:
  bool _internal_has_hasangle_nu() const;
  public:
  void clear_hasangle_nu();
  bool hasangle_nu() const;
  void set_hasangle_nu(bool value);
  private:
  bool _internal_hasangle_nu() const;
  void _internal_set_hasangle_nu(bool value);
  public:

  // optional bool hasPosition_nu = 651;
  bool has_hasposition_nu() const;
  private:
  bool _internal_has_hasposition_nu() const;
  public:
  void clear_hasposition_nu();
  bool hasposition_nu() const;
  void set_hasposition_nu(bool value);
  private:
  bool _internal_hasposition_nu() const;
  void _internal_set_hasposition_nu(bool value);
  public:

  // optional float targetPosY_m = 2644;
  bool has_targetposy_m() const;
  private:
  bool _internal_has_targetposy_m() const;
  public:
  void clear_targetposy_m();
  float targetposy_m() const;
  void set_targetposy_m(float value);
  private:
  float _internal_targetposy_m() const;
  void _internal_set_targetposy_m(float value);
  public:

  // optional float targetPosX_m = 3428;
  bool has_targetposx_m() const;
  private:
  bool _internal_has_targetposx_m() const;
  public:
  void clear_targetposx_m();
  float targetposx_m() const;
  void set_targetposx_m(float value);
  private:
  float _internal_targetposx_m() const;
  void _internal_set_targetposx_m(float value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* comfparkingbox_;
  ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* maxparkingbox_;
  ::pb::ap_tp::delimiter_distances::DelimiterDistances* delimiterdistances_;
  float yawangle_rad_;
  bool leftedgeisfront_nu_;
  bool isresized_nu_;
  bool hasangle_nu_;
  bool hasposition_nu_;
  float targetposy_m_;
  float targetposx_m_;
  friend struct ::TableStruct_ap_5ftp_2fparking_5fbox_5fdebug_5finfo_2eproto;
};
// -------------------------------------------------------------------

class ParkingBoxDebugInfo_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo_array_port) */ {
 public:
  ParkingBoxDebugInfo_array_port();
  virtual ~ParkingBoxDebugInfo_array_port();

  ParkingBoxDebugInfo_array_port(const ParkingBoxDebugInfo_array_port& from);
  ParkingBoxDebugInfo_array_port(ParkingBoxDebugInfo_array_port&& from) noexcept
    : ParkingBoxDebugInfo_array_port() {
    *this = ::std::move(from);
  }

  inline ParkingBoxDebugInfo_array_port& operator=(const ParkingBoxDebugInfo_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline ParkingBoxDebugInfo_array_port& operator=(ParkingBoxDebugInfo_array_port&& from) noexcept {
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
  static const ParkingBoxDebugInfo_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ParkingBoxDebugInfo_array_port* internal_default_instance() {
    return reinterpret_cast<const ParkingBoxDebugInfo_array_port*>(
               &_ParkingBoxDebugInfo_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ParkingBoxDebugInfo_array_port& a, ParkingBoxDebugInfo_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(ParkingBoxDebugInfo_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ParkingBoxDebugInfo_array_port* New() const final {
    return CreateMaybeMessage<ParkingBoxDebugInfo_array_port>(nullptr);
  }

  ParkingBoxDebugInfo_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ParkingBoxDebugInfo_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ParkingBoxDebugInfo_array_port& from);
  void MergeFrom(const ParkingBoxDebugInfo_array_port& from);
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
  void InternalSwap(ParkingBoxDebugInfo_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5ftp_2fparking_5fbox_5fdebug_5finfo_2eproto);
    return ::descriptor_table_ap_5ftp_2fparking_5fbox_5fdebug_5finfo_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3459,
  };
  // repeated .pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo data = 3459;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo >*
      mutable_data();
  private:
  const ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo& _internal_data(int index) const;
  ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo* _internal_add_data();
  public:
  const ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo& data(int index) const;
  ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo > data_;
  friend struct ::TableStruct_ap_5ftp_2fparking_5fbox_5fdebug_5finfo_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ParkingBoxDebugInfo

// optional .pb.ap_tp.simple_debug_parking_box.SimpleDebugParkingBox maxParkingBox = 2943;
inline bool ParkingBoxDebugInfo::_internal_has_maxparkingbox() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || maxparkingbox_ != nullptr);
  return value;
}
inline bool ParkingBoxDebugInfo::has_maxparkingbox() const {
  return _internal_has_maxparkingbox();
}
inline const ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox& ParkingBoxDebugInfo::_internal_maxparkingbox() const {
  const ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* p = maxparkingbox_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox*>(
      &::pb::ap_tp::simple_debug_parking_box::_SimpleDebugParkingBox_default_instance_);
}
inline const ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox& ParkingBoxDebugInfo::maxparkingbox() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.maxParkingBox)
  return _internal_maxparkingbox();
}
inline ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* ParkingBoxDebugInfo::release_maxparkingbox() {
  // @@protoc_insertion_point(field_release:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.maxParkingBox)
  _has_bits_[0] &= ~0x00000002u;
  ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* temp = maxparkingbox_;
  maxparkingbox_ = nullptr;
  return temp;
}
inline ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* ParkingBoxDebugInfo::_internal_mutable_maxparkingbox() {
  _has_bits_[0] |= 0x00000002u;
  if (maxparkingbox_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox>(GetArenaNoVirtual());
    maxparkingbox_ = p;
  }
  return maxparkingbox_;
}
inline ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* ParkingBoxDebugInfo::mutable_maxparkingbox() {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.maxParkingBox)
  return _internal_mutable_maxparkingbox();
}
inline void ParkingBoxDebugInfo::set_allocated_maxparkingbox(::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* maxparkingbox) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(maxparkingbox_);
  }
  if (maxparkingbox) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      maxparkingbox = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, maxparkingbox, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  maxparkingbox_ = maxparkingbox;
  // @@protoc_insertion_point(field_set_allocated:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.maxParkingBox)
}

// optional .pb.ap_tp.simple_debug_parking_box.SimpleDebugParkingBox comfParkingBox = 2471;
inline bool ParkingBoxDebugInfo::_internal_has_comfparkingbox() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || comfparkingbox_ != nullptr);
  return value;
}
inline bool ParkingBoxDebugInfo::has_comfparkingbox() const {
  return _internal_has_comfparkingbox();
}
inline const ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox& ParkingBoxDebugInfo::_internal_comfparkingbox() const {
  const ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* p = comfparkingbox_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox*>(
      &::pb::ap_tp::simple_debug_parking_box::_SimpleDebugParkingBox_default_instance_);
}
inline const ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox& ParkingBoxDebugInfo::comfparkingbox() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.comfParkingBox)
  return _internal_comfparkingbox();
}
inline ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* ParkingBoxDebugInfo::release_comfparkingbox() {
  // @@protoc_insertion_point(field_release:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.comfParkingBox)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* temp = comfparkingbox_;
  comfparkingbox_ = nullptr;
  return temp;
}
inline ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* ParkingBoxDebugInfo::_internal_mutable_comfparkingbox() {
  _has_bits_[0] |= 0x00000001u;
  if (comfparkingbox_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox>(GetArenaNoVirtual());
    comfparkingbox_ = p;
  }
  return comfparkingbox_;
}
inline ::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* ParkingBoxDebugInfo::mutable_comfparkingbox() {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.comfParkingBox)
  return _internal_mutable_comfparkingbox();
}
inline void ParkingBoxDebugInfo::set_allocated_comfparkingbox(::pb::ap_tp::simple_debug_parking_box::SimpleDebugParkingBox* comfparkingbox) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(comfparkingbox_);
  }
  if (comfparkingbox) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      comfparkingbox = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, comfparkingbox, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  comfparkingbox_ = comfparkingbox;
  // @@protoc_insertion_point(field_set_allocated:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.comfParkingBox)
}

// optional .pb.ap_tp.delimiter_distances.DelimiterDistances delimiterDistances = 3359;
inline bool ParkingBoxDebugInfo::_internal_has_delimiterdistances() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || delimiterdistances_ != nullptr);
  return value;
}
inline bool ParkingBoxDebugInfo::has_delimiterdistances() const {
  return _internal_has_delimiterdistances();
}
inline const ::pb::ap_tp::delimiter_distances::DelimiterDistances& ParkingBoxDebugInfo::_internal_delimiterdistances() const {
  const ::pb::ap_tp::delimiter_distances::DelimiterDistances* p = delimiterdistances_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::ap_tp::delimiter_distances::DelimiterDistances*>(
      &::pb::ap_tp::delimiter_distances::_DelimiterDistances_default_instance_);
}
inline const ::pb::ap_tp::delimiter_distances::DelimiterDistances& ParkingBoxDebugInfo::delimiterdistances() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.delimiterDistances)
  return _internal_delimiterdistances();
}
inline ::pb::ap_tp::delimiter_distances::DelimiterDistances* ParkingBoxDebugInfo::release_delimiterdistances() {
  // @@protoc_insertion_point(field_release:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.delimiterDistances)
  _has_bits_[0] &= ~0x00000004u;
  ::pb::ap_tp::delimiter_distances::DelimiterDistances* temp = delimiterdistances_;
  delimiterdistances_ = nullptr;
  return temp;
}
inline ::pb::ap_tp::delimiter_distances::DelimiterDistances* ParkingBoxDebugInfo::_internal_mutable_delimiterdistances() {
  _has_bits_[0] |= 0x00000004u;
  if (delimiterdistances_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::ap_tp::delimiter_distances::DelimiterDistances>(GetArenaNoVirtual());
    delimiterdistances_ = p;
  }
  return delimiterdistances_;
}
inline ::pb::ap_tp::delimiter_distances::DelimiterDistances* ParkingBoxDebugInfo::mutable_delimiterdistances() {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.delimiterDistances)
  return _internal_mutable_delimiterdistances();
}
inline void ParkingBoxDebugInfo::set_allocated_delimiterdistances(::pb::ap_tp::delimiter_distances::DelimiterDistances* delimiterdistances) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(delimiterdistances_);
  }
  if (delimiterdistances) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      delimiterdistances = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, delimiterdistances, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  delimiterdistances_ = delimiterdistances;
  // @@protoc_insertion_point(field_set_allocated:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.delimiterDistances)
}

// optional float yawAngle_rad = 566;
inline bool ParkingBoxDebugInfo::_internal_has_yawangle_rad() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool ParkingBoxDebugInfo::has_yawangle_rad() const {
  return _internal_has_yawangle_rad();
}
inline void ParkingBoxDebugInfo::clear_yawangle_rad() {
  yawangle_rad_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float ParkingBoxDebugInfo::_internal_yawangle_rad() const {
  return yawangle_rad_;
}
inline float ParkingBoxDebugInfo::yawangle_rad() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.yawAngle_rad)
  return _internal_yawangle_rad();
}
inline void ParkingBoxDebugInfo::_internal_set_yawangle_rad(float value) {
  _has_bits_[0] |= 0x00000008u;
  yawangle_rad_ = value;
}
inline void ParkingBoxDebugInfo::set_yawangle_rad(float value) {
  _internal_set_yawangle_rad(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.yawAngle_rad)
}

// optional float targetPosX_m = 3428;
inline bool ParkingBoxDebugInfo::_internal_has_targetposx_m() const {
  bool value = (_has_bits_[0] & 0x00000200u) != 0;
  return value;
}
inline bool ParkingBoxDebugInfo::has_targetposx_m() const {
  return _internal_has_targetposx_m();
}
inline void ParkingBoxDebugInfo::clear_targetposx_m() {
  targetposx_m_ = 0;
  _has_bits_[0] &= ~0x00000200u;
}
inline float ParkingBoxDebugInfo::_internal_targetposx_m() const {
  return targetposx_m_;
}
inline float ParkingBoxDebugInfo::targetposx_m() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.targetPosX_m)
  return _internal_targetposx_m();
}
inline void ParkingBoxDebugInfo::_internal_set_targetposx_m(float value) {
  _has_bits_[0] |= 0x00000200u;
  targetposx_m_ = value;
}
inline void ParkingBoxDebugInfo::set_targetposx_m(float value) {
  _internal_set_targetposx_m(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.targetPosX_m)
}

// optional float targetPosY_m = 2644;
inline bool ParkingBoxDebugInfo::_internal_has_targetposy_m() const {
  bool value = (_has_bits_[0] & 0x00000100u) != 0;
  return value;
}
inline bool ParkingBoxDebugInfo::has_targetposy_m() const {
  return _internal_has_targetposy_m();
}
inline void ParkingBoxDebugInfo::clear_targetposy_m() {
  targetposy_m_ = 0;
  _has_bits_[0] &= ~0x00000100u;
}
inline float ParkingBoxDebugInfo::_internal_targetposy_m() const {
  return targetposy_m_;
}
inline float ParkingBoxDebugInfo::targetposy_m() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.targetPosY_m)
  return _internal_targetposy_m();
}
inline void ParkingBoxDebugInfo::_internal_set_targetposy_m(float value) {
  _has_bits_[0] |= 0x00000100u;
  targetposy_m_ = value;
}
inline void ParkingBoxDebugInfo::set_targetposy_m(float value) {
  _internal_set_targetposy_m(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.targetPosY_m)
}

// optional bool leftEdgeIsFront_nu = 2311;
inline bool ParkingBoxDebugInfo::_internal_has_leftedgeisfront_nu() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool ParkingBoxDebugInfo::has_leftedgeisfront_nu() const {
  return _internal_has_leftedgeisfront_nu();
}
inline void ParkingBoxDebugInfo::clear_leftedgeisfront_nu() {
  leftedgeisfront_nu_ = false;
  _has_bits_[0] &= ~0x00000010u;
}
inline bool ParkingBoxDebugInfo::_internal_leftedgeisfront_nu() const {
  return leftedgeisfront_nu_;
}
inline bool ParkingBoxDebugInfo::leftedgeisfront_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.leftEdgeIsFront_nu)
  return _internal_leftedgeisfront_nu();
}
inline void ParkingBoxDebugInfo::_internal_set_leftedgeisfront_nu(bool value) {
  _has_bits_[0] |= 0x00000010u;
  leftedgeisfront_nu_ = value;
}
inline void ParkingBoxDebugInfo::set_leftedgeisfront_nu(bool value) {
  _internal_set_leftedgeisfront_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.leftEdgeIsFront_nu)
}

// optional bool isResized_nu = 600;
inline bool ParkingBoxDebugInfo::_internal_has_isresized_nu() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool ParkingBoxDebugInfo::has_isresized_nu() const {
  return _internal_has_isresized_nu();
}
inline void ParkingBoxDebugInfo::clear_isresized_nu() {
  isresized_nu_ = false;
  _has_bits_[0] &= ~0x00000020u;
}
inline bool ParkingBoxDebugInfo::_internal_isresized_nu() const {
  return isresized_nu_;
}
inline bool ParkingBoxDebugInfo::isresized_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.isResized_nu)
  return _internal_isresized_nu();
}
inline void ParkingBoxDebugInfo::_internal_set_isresized_nu(bool value) {
  _has_bits_[0] |= 0x00000020u;
  isresized_nu_ = value;
}
inline void ParkingBoxDebugInfo::set_isresized_nu(bool value) {
  _internal_set_isresized_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.isResized_nu)
}

// optional bool hasAngle_nu = 1713;
inline bool ParkingBoxDebugInfo::_internal_has_hasangle_nu() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool ParkingBoxDebugInfo::has_hasangle_nu() const {
  return _internal_has_hasangle_nu();
}
inline void ParkingBoxDebugInfo::clear_hasangle_nu() {
  hasangle_nu_ = false;
  _has_bits_[0] &= ~0x00000040u;
}
inline bool ParkingBoxDebugInfo::_internal_hasangle_nu() const {
  return hasangle_nu_;
}
inline bool ParkingBoxDebugInfo::hasangle_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.hasAngle_nu)
  return _internal_hasangle_nu();
}
inline void ParkingBoxDebugInfo::_internal_set_hasangle_nu(bool value) {
  _has_bits_[0] |= 0x00000040u;
  hasangle_nu_ = value;
}
inline void ParkingBoxDebugInfo::set_hasangle_nu(bool value) {
  _internal_set_hasangle_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.hasAngle_nu)
}

// optional bool hasPosition_nu = 651;
inline bool ParkingBoxDebugInfo::_internal_has_hasposition_nu() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool ParkingBoxDebugInfo::has_hasposition_nu() const {
  return _internal_has_hasposition_nu();
}
inline void ParkingBoxDebugInfo::clear_hasposition_nu() {
  hasposition_nu_ = false;
  _has_bits_[0] &= ~0x00000080u;
}
inline bool ParkingBoxDebugInfo::_internal_hasposition_nu() const {
  return hasposition_nu_;
}
inline bool ParkingBoxDebugInfo::hasposition_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.hasPosition_nu)
  return _internal_hasposition_nu();
}
inline void ParkingBoxDebugInfo::_internal_set_hasposition_nu(bool value) {
  _has_bits_[0] |= 0x00000080u;
  hasposition_nu_ = value;
}
inline void ParkingBoxDebugInfo::set_hasposition_nu(bool value) {
  _internal_set_hasposition_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo.hasPosition_nu)
}

// -------------------------------------------------------------------

// ParkingBoxDebugInfo_array_port

// repeated .pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo data = 3459;
inline int ParkingBoxDebugInfo_array_port::_internal_data_size() const {
  return data_.size();
}
inline int ParkingBoxDebugInfo_array_port::data_size() const {
  return _internal_data_size();
}
inline void ParkingBoxDebugInfo_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo* ParkingBoxDebugInfo_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo >*
ParkingBoxDebugInfo_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo_array_port.data)
  return &data_;
}
inline const ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo& ParkingBoxDebugInfo_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo& ParkingBoxDebugInfo_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo* ParkingBoxDebugInfo_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo* ParkingBoxDebugInfo_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_tp::parking_box_debug_info::ParkingBoxDebugInfo >&
ParkingBoxDebugInfo_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_tp.parking_box_debug_info.ParkingBoxDebugInfo_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace parking_box_debug_info
}  // namespace ap_tp
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5ftp_2fparking_5fbox_5fdebug_5finfo_2eproto