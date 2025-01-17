// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_mempark/redetection_request.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fredetection_5frequest_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fredetection_5frequest_2eproto

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
#include "lsm_geoml/pose_pod.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fmempark_2fredetection_5frequest_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fmempark_2fredetection_5frequest_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto;
namespace pb {
namespace mf_mempark {
namespace redetection_request {
class RedetectionRequest;
class RedetectionRequestDefaultTypeInternal;
extern RedetectionRequestDefaultTypeInternal _RedetectionRequest_default_instance_;
class RedetectionRequest_array_port;
class RedetectionRequest_array_portDefaultTypeInternal;
extern RedetectionRequest_array_portDefaultTypeInternal _RedetectionRequest_array_port_default_instance_;
}  // namespace redetection_request
}  // namespace mf_mempark
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_mempark::redetection_request::RedetectionRequest* Arena::CreateMaybeMessage<::pb::mf_mempark::redetection_request::RedetectionRequest>(Arena*);
template<> ::pb::mf_mempark::redetection_request::RedetectionRequest_array_port* Arena::CreateMaybeMessage<::pb::mf_mempark::redetection_request::RedetectionRequest_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_mempark {
namespace redetection_request {

// ===================================================================

class RedetectionRequest :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.redetection_request.RedetectionRequest) */ {
 public:
  RedetectionRequest();
  virtual ~RedetectionRequest();

  RedetectionRequest(const RedetectionRequest& from);
  RedetectionRequest(RedetectionRequest&& from) noexcept
    : RedetectionRequest() {
    *this = ::std::move(from);
  }

  inline RedetectionRequest& operator=(const RedetectionRequest& from) {
    CopyFrom(from);
    return *this;
  }
  inline RedetectionRequest& operator=(RedetectionRequest&& from) noexcept {
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
  static const RedetectionRequest& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RedetectionRequest* internal_default_instance() {
    return reinterpret_cast<const RedetectionRequest*>(
               &_RedetectionRequest_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(RedetectionRequest& a, RedetectionRequest& b) {
    a.Swap(&b);
  }
  inline void Swap(RedetectionRequest* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline RedetectionRequest* New() const final {
    return CreateMaybeMessage<RedetectionRequest>(nullptr);
  }

  RedetectionRequest* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<RedetectionRequest>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const RedetectionRequest& from);
  void MergeFrom(const RedetectionRequest& from);
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
  void InternalSwap(RedetectionRequest* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.redetection_request.RedetectionRequest";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto);
    return ::descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kStartPoseFieldNumber = 3278,
    kMapIDFieldNumber = 1752,
    kInitialLocalizationRequestFieldNumber = 3409,
  };
  // optional .pb.lsm_geoml.pose_pod.Pose_POD startPose = 3278;
  bool has_startpose() const;
  private:
  bool _internal_has_startpose() const;
  public:
  void clear_startpose();
  const ::pb::lsm_geoml::pose_pod::Pose_POD& startpose() const;
  ::pb::lsm_geoml::pose_pod::Pose_POD* release_startpose();
  ::pb::lsm_geoml::pose_pod::Pose_POD* mutable_startpose();
  void set_allocated_startpose(::pb::lsm_geoml::pose_pod::Pose_POD* startpose);
  private:
  const ::pb::lsm_geoml::pose_pod::Pose_POD& _internal_startpose() const;
  ::pb::lsm_geoml::pose_pod::Pose_POD* _internal_mutable_startpose();
  public:

  // optional uint32 mapID = 1752;
  bool has_mapid() const;
  private:
  bool _internal_has_mapid() const;
  public:
  void clear_mapid();
  ::PROTOBUF_NAMESPACE_ID::uint32 mapid() const;
  void set_mapid(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_mapid() const;
  void _internal_set_mapid(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional bool initialLocalizationRequest = 3409;
  bool has_initiallocalizationrequest() const;
  private:
  bool _internal_has_initiallocalizationrequest() const;
  public:
  void clear_initiallocalizationrequest();
  bool initiallocalizationrequest() const;
  void set_initiallocalizationrequest(bool value);
  private:
  bool _internal_initiallocalizationrequest() const;
  void _internal_set_initiallocalizationrequest(bool value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.redetection_request.RedetectionRequest)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::lsm_geoml::pose_pod::Pose_POD* startpose_;
  ::PROTOBUF_NAMESPACE_ID::uint32 mapid_;
  bool initiallocalizationrequest_;
  friend struct ::TableStruct_mf_5fmempark_2fredetection_5frequest_2eproto;
};
// -------------------------------------------------------------------

class RedetectionRequest_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_mempark.redetection_request.RedetectionRequest_array_port) */ {
 public:
  RedetectionRequest_array_port();
  virtual ~RedetectionRequest_array_port();

  RedetectionRequest_array_port(const RedetectionRequest_array_port& from);
  RedetectionRequest_array_port(RedetectionRequest_array_port&& from) noexcept
    : RedetectionRequest_array_port() {
    *this = ::std::move(from);
  }

  inline RedetectionRequest_array_port& operator=(const RedetectionRequest_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline RedetectionRequest_array_port& operator=(RedetectionRequest_array_port&& from) noexcept {
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
  static const RedetectionRequest_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RedetectionRequest_array_port* internal_default_instance() {
    return reinterpret_cast<const RedetectionRequest_array_port*>(
               &_RedetectionRequest_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(RedetectionRequest_array_port& a, RedetectionRequest_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(RedetectionRequest_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline RedetectionRequest_array_port* New() const final {
    return CreateMaybeMessage<RedetectionRequest_array_port>(nullptr);
  }

  RedetectionRequest_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<RedetectionRequest_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const RedetectionRequest_array_port& from);
  void MergeFrom(const RedetectionRequest_array_port& from);
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
  void InternalSwap(RedetectionRequest_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_mempark.redetection_request.RedetectionRequest_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto);
    return ::descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1424,
  };
  // repeated .pb.mf_mempark.redetection_request.RedetectionRequest data = 1424;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_mempark::redetection_request::RedetectionRequest* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::redetection_request::RedetectionRequest >*
      mutable_data();
  private:
  const ::pb::mf_mempark::redetection_request::RedetectionRequest& _internal_data(int index) const;
  ::pb::mf_mempark::redetection_request::RedetectionRequest* _internal_add_data();
  public:
  const ::pb::mf_mempark::redetection_request::RedetectionRequest& data(int index) const;
  ::pb::mf_mempark::redetection_request::RedetectionRequest* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::redetection_request::RedetectionRequest >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::redetection_request::RedetectionRequest > data_;
  friend struct ::TableStruct_mf_5fmempark_2fredetection_5frequest_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RedetectionRequest

// optional uint32 mapID = 1752;
inline bool RedetectionRequest::_internal_has_mapid() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool RedetectionRequest::has_mapid() const {
  return _internal_has_mapid();
}
inline void RedetectionRequest::clear_mapid() {
  mapid_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 RedetectionRequest::_internal_mapid() const {
  return mapid_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 RedetectionRequest::mapid() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.redetection_request.RedetectionRequest.mapID)
  return _internal_mapid();
}
inline void RedetectionRequest::_internal_set_mapid(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  mapid_ = value;
}
inline void RedetectionRequest::set_mapid(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_mapid(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.redetection_request.RedetectionRequest.mapID)
}

// optional .pb.lsm_geoml.pose_pod.Pose_POD startPose = 3278;
inline bool RedetectionRequest::_internal_has_startpose() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || startpose_ != nullptr);
  return value;
}
inline bool RedetectionRequest::has_startpose() const {
  return _internal_has_startpose();
}
inline const ::pb::lsm_geoml::pose_pod::Pose_POD& RedetectionRequest::_internal_startpose() const {
  const ::pb::lsm_geoml::pose_pod::Pose_POD* p = startpose_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::lsm_geoml::pose_pod::Pose_POD*>(
      &::pb::lsm_geoml::pose_pod::_Pose_POD_default_instance_);
}
inline const ::pb::lsm_geoml::pose_pod::Pose_POD& RedetectionRequest::startpose() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.redetection_request.RedetectionRequest.startPose)
  return _internal_startpose();
}
inline ::pb::lsm_geoml::pose_pod::Pose_POD* RedetectionRequest::release_startpose() {
  // @@protoc_insertion_point(field_release:pb.mf_mempark.redetection_request.RedetectionRequest.startPose)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::lsm_geoml::pose_pod::Pose_POD* temp = startpose_;
  startpose_ = nullptr;
  return temp;
}
inline ::pb::lsm_geoml::pose_pod::Pose_POD* RedetectionRequest::_internal_mutable_startpose() {
  _has_bits_[0] |= 0x00000001u;
  if (startpose_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::lsm_geoml::pose_pod::Pose_POD>(GetArenaNoVirtual());
    startpose_ = p;
  }
  return startpose_;
}
inline ::pb::lsm_geoml::pose_pod::Pose_POD* RedetectionRequest::mutable_startpose() {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.redetection_request.RedetectionRequest.startPose)
  return _internal_mutable_startpose();
}
inline void RedetectionRequest::set_allocated_startpose(::pb::lsm_geoml::pose_pod::Pose_POD* startpose) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(startpose_);
  }
  if (startpose) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      startpose = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, startpose, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  startpose_ = startpose;
  // @@protoc_insertion_point(field_set_allocated:pb.mf_mempark.redetection_request.RedetectionRequest.startPose)
}

// optional bool initialLocalizationRequest = 3409;
inline bool RedetectionRequest::_internal_has_initiallocalizationrequest() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool RedetectionRequest::has_initiallocalizationrequest() const {
  return _internal_has_initiallocalizationrequest();
}
inline void RedetectionRequest::clear_initiallocalizationrequest() {
  initiallocalizationrequest_ = false;
  _has_bits_[0] &= ~0x00000004u;
}
inline bool RedetectionRequest::_internal_initiallocalizationrequest() const {
  return initiallocalizationrequest_;
}
inline bool RedetectionRequest::initiallocalizationrequest() const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.redetection_request.RedetectionRequest.initialLocalizationRequest)
  return _internal_initiallocalizationrequest();
}
inline void RedetectionRequest::_internal_set_initiallocalizationrequest(bool value) {
  _has_bits_[0] |= 0x00000004u;
  initiallocalizationrequest_ = value;
}
inline void RedetectionRequest::set_initiallocalizationrequest(bool value) {
  _internal_set_initiallocalizationrequest(value);
  // @@protoc_insertion_point(field_set:pb.mf_mempark.redetection_request.RedetectionRequest.initialLocalizationRequest)
}

// -------------------------------------------------------------------

// RedetectionRequest_array_port

// repeated .pb.mf_mempark.redetection_request.RedetectionRequest data = 1424;
inline int RedetectionRequest_array_port::_internal_data_size() const {
  return data_.size();
}
inline int RedetectionRequest_array_port::data_size() const {
  return _internal_data_size();
}
inline void RedetectionRequest_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_mempark::redetection_request::RedetectionRequest* RedetectionRequest_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_mempark.redetection_request.RedetectionRequest_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::redetection_request::RedetectionRequest >*
RedetectionRequest_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_mempark.redetection_request.RedetectionRequest_array_port.data)
  return &data_;
}
inline const ::pb::mf_mempark::redetection_request::RedetectionRequest& RedetectionRequest_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_mempark::redetection_request::RedetectionRequest& RedetectionRequest_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_mempark.redetection_request.RedetectionRequest_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_mempark::redetection_request::RedetectionRequest* RedetectionRequest_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_mempark::redetection_request::RedetectionRequest* RedetectionRequest_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_mempark.redetection_request.RedetectionRequest_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_mempark::redetection_request::RedetectionRequest >&
RedetectionRequest_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_mempark.redetection_request.RedetectionRequest_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace redetection_request
}  // namespace mf_mempark
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fmempark_2fredetection_5frequest_2eproto
