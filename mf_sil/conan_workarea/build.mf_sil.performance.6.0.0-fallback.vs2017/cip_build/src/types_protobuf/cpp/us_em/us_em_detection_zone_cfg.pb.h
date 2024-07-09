// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: us_em/us_em_detection_zone_cfg.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_us_5fem_2fus_5fem_5fdetection_5fzone_5fcfg_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_us_5fem_2fus_5fem_5fdetection_5fzone_5fcfg_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_us_5fem_2fus_5fem_5fdetection_5fzone_5fcfg_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_us_5fem_2fus_5fem_5fdetection_5fzone_5fcfg_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_us_5fem_2fus_5fem_5fdetection_5fzone_5fcfg_2eproto;
namespace pb {
namespace us_em {
namespace us_em_detection_zone_cfg {
class UsEmDetectionZoneCfg;
class UsEmDetectionZoneCfgDefaultTypeInternal;
extern UsEmDetectionZoneCfgDefaultTypeInternal _UsEmDetectionZoneCfg_default_instance_;
class UsEmDetectionZoneCfg_array_port;
class UsEmDetectionZoneCfg_array_portDefaultTypeInternal;
extern UsEmDetectionZoneCfg_array_portDefaultTypeInternal _UsEmDetectionZoneCfg_array_port_default_instance_;
}  // namespace us_em_detection_zone_cfg
}  // namespace us_em
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* Arena::CreateMaybeMessage<::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg>(Arena*);
template<> ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg_array_port* Arena::CreateMaybeMessage<::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace us_em {
namespace us_em_detection_zone_cfg {

// ===================================================================

class UsEmDetectionZoneCfg :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg) */ {
 public:
  UsEmDetectionZoneCfg();
  virtual ~UsEmDetectionZoneCfg();

  UsEmDetectionZoneCfg(const UsEmDetectionZoneCfg& from);
  UsEmDetectionZoneCfg(UsEmDetectionZoneCfg&& from) noexcept
    : UsEmDetectionZoneCfg() {
    *this = ::std::move(from);
  }

  inline UsEmDetectionZoneCfg& operator=(const UsEmDetectionZoneCfg& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsEmDetectionZoneCfg& operator=(UsEmDetectionZoneCfg&& from) noexcept {
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
  static const UsEmDetectionZoneCfg& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsEmDetectionZoneCfg* internal_default_instance() {
    return reinterpret_cast<const UsEmDetectionZoneCfg*>(
               &_UsEmDetectionZoneCfg_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UsEmDetectionZoneCfg& a, UsEmDetectionZoneCfg& b) {
    a.Swap(&b);
  }
  inline void Swap(UsEmDetectionZoneCfg* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsEmDetectionZoneCfg* New() const final {
    return CreateMaybeMessage<UsEmDetectionZoneCfg>(nullptr);
  }

  UsEmDetectionZoneCfg* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsEmDetectionZoneCfg>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsEmDetectionZoneCfg& from);
  void MergeFrom(const UsEmDetectionZoneCfg& from);
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
  void InternalSwap(UsEmDetectionZoneCfg* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fem_2fus_5fem_5fdetection_5fzone_5fcfg_2eproto);
    return ::descriptor_table_us_5fem_2fus_5fem_5fdetection_5fzone_5fcfg_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDetZoneBackRightXFieldNumber = 396,
    kDetZoneBackRightYFieldNumber = 429,
    kDetZoneBackLeftYFieldNumber = 1153,
    kDetZoneBackLeftXFieldNumber = 1184,
    kDetZoneFrontLeftYFieldNumber = 3921,
    kDetZoneFrontLeftXFieldNumber = 3952,
    kDetZoneFrontRightYFieldNumber = 4057,
    kDetZoneFrontRightXFieldNumber = 4088,
  };
  // optional float detZoneBackRightX = 396;
  bool has_detzonebackrightx() const;
  private:
  bool _internal_has_detzonebackrightx() const;
  public:
  void clear_detzonebackrightx();
  float detzonebackrightx() const;
  void set_detzonebackrightx(float value);
  private:
  float _internal_detzonebackrightx() const;
  void _internal_set_detzonebackrightx(float value);
  public:

  // optional float detZoneBackRightY = 429;
  bool has_detzonebackrighty() const;
  private:
  bool _internal_has_detzonebackrighty() const;
  public:
  void clear_detzonebackrighty();
  float detzonebackrighty() const;
  void set_detzonebackrighty(float value);
  private:
  float _internal_detzonebackrighty() const;
  void _internal_set_detzonebackrighty(float value);
  public:

  // optional float detZoneBackLeftY = 1153;
  bool has_detzonebacklefty() const;
  private:
  bool _internal_has_detzonebacklefty() const;
  public:
  void clear_detzonebacklefty();
  float detzonebacklefty() const;
  void set_detzonebacklefty(float value);
  private:
  float _internal_detzonebacklefty() const;
  void _internal_set_detzonebacklefty(float value);
  public:

  // optional float detZoneBackLeftX = 1184;
  bool has_detzonebackleftx() const;
  private:
  bool _internal_has_detzonebackleftx() const;
  public:
  void clear_detzonebackleftx();
  float detzonebackleftx() const;
  void set_detzonebackleftx(float value);
  private:
  float _internal_detzonebackleftx() const;
  void _internal_set_detzonebackleftx(float value);
  public:

  // optional float detZoneFrontLeftY = 3921;
  bool has_detzonefrontlefty() const;
  private:
  bool _internal_has_detzonefrontlefty() const;
  public:
  void clear_detzonefrontlefty();
  float detzonefrontlefty() const;
  void set_detzonefrontlefty(float value);
  private:
  float _internal_detzonefrontlefty() const;
  void _internal_set_detzonefrontlefty(float value);
  public:

  // optional float detZoneFrontLeftX = 3952;
  bool has_detzonefrontleftx() const;
  private:
  bool _internal_has_detzonefrontleftx() const;
  public:
  void clear_detzonefrontleftx();
  float detzonefrontleftx() const;
  void set_detzonefrontleftx(float value);
  private:
  float _internal_detzonefrontleftx() const;
  void _internal_set_detzonefrontleftx(float value);
  public:

  // optional float detZoneFrontRightY = 4057;
  bool has_detzonefrontrighty() const;
  private:
  bool _internal_has_detzonefrontrighty() const;
  public:
  void clear_detzonefrontrighty();
  float detzonefrontrighty() const;
  void set_detzonefrontrighty(float value);
  private:
  float _internal_detzonefrontrighty() const;
  void _internal_set_detzonefrontrighty(float value);
  public:

  // optional float detZoneFrontRightX = 4088;
  bool has_detzonefrontrightx() const;
  private:
  bool _internal_has_detzonefrontrightx() const;
  public:
  void clear_detzonefrontrightx();
  float detzonefrontrightx() const;
  void set_detzonefrontrightx(float value);
  private:
  float _internal_detzonefrontrightx() const;
  void _internal_set_detzonefrontrightx(float value);
  public:

  // @@protoc_insertion_point(class_scope:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  float detzonebackrightx_;
  float detzonebackrighty_;
  float detzonebacklefty_;
  float detzonebackleftx_;
  float detzonefrontlefty_;
  float detzonefrontleftx_;
  float detzonefrontrighty_;
  float detzonefrontrightx_;
  friend struct ::TableStruct_us_5fem_2fus_5fem_5fdetection_5fzone_5fcfg_2eproto;
};
// -------------------------------------------------------------------

class UsEmDetectionZoneCfg_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg_array_port) */ {
 public:
  UsEmDetectionZoneCfg_array_port();
  virtual ~UsEmDetectionZoneCfg_array_port();

  UsEmDetectionZoneCfg_array_port(const UsEmDetectionZoneCfg_array_port& from);
  UsEmDetectionZoneCfg_array_port(UsEmDetectionZoneCfg_array_port&& from) noexcept
    : UsEmDetectionZoneCfg_array_port() {
    *this = ::std::move(from);
  }

  inline UsEmDetectionZoneCfg_array_port& operator=(const UsEmDetectionZoneCfg_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline UsEmDetectionZoneCfg_array_port& operator=(UsEmDetectionZoneCfg_array_port&& from) noexcept {
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
  static const UsEmDetectionZoneCfg_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const UsEmDetectionZoneCfg_array_port* internal_default_instance() {
    return reinterpret_cast<const UsEmDetectionZoneCfg_array_port*>(
               &_UsEmDetectionZoneCfg_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(UsEmDetectionZoneCfg_array_port& a, UsEmDetectionZoneCfg_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(UsEmDetectionZoneCfg_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline UsEmDetectionZoneCfg_array_port* New() const final {
    return CreateMaybeMessage<UsEmDetectionZoneCfg_array_port>(nullptr);
  }

  UsEmDetectionZoneCfg_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<UsEmDetectionZoneCfg_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const UsEmDetectionZoneCfg_array_port& from);
  void MergeFrom(const UsEmDetectionZoneCfg_array_port& from);
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
  void InternalSwap(UsEmDetectionZoneCfg_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_us_5fem_2fus_5fem_5fdetection_5fzone_5fcfg_2eproto);
    return ::descriptor_table_us_5fem_2fus_5fem_5fdetection_5fzone_5fcfg_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3007,
  };
  // repeated .pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg data = 3007;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg >*
      mutable_data();
  private:
  const ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg& _internal_data(int index) const;
  ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* _internal_add_data();
  public:
  const ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg& data(int index) const;
  ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg > data_;
  friend struct ::TableStruct_us_5fem_2fus_5fem_5fdetection_5fzone_5fcfg_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UsEmDetectionZoneCfg

// optional float detZoneFrontLeftX = 3952;
inline bool UsEmDetectionZoneCfg::_internal_has_detzonefrontleftx() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool UsEmDetectionZoneCfg::has_detzonefrontleftx() const {
  return _internal_has_detzonefrontleftx();
}
inline void UsEmDetectionZoneCfg::clear_detzonefrontleftx() {
  detzonefrontleftx_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline float UsEmDetectionZoneCfg::_internal_detzonefrontleftx() const {
  return detzonefrontleftx_;
}
inline float UsEmDetectionZoneCfg::detzonefrontleftx() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneFrontLeftX)
  return _internal_detzonefrontleftx();
}
inline void UsEmDetectionZoneCfg::_internal_set_detzonefrontleftx(float value) {
  _has_bits_[0] |= 0x00000020u;
  detzonefrontleftx_ = value;
}
inline void UsEmDetectionZoneCfg::set_detzonefrontleftx(float value) {
  _internal_set_detzonefrontleftx(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneFrontLeftX)
}

// optional float detZoneFrontLeftY = 3921;
inline bool UsEmDetectionZoneCfg::_internal_has_detzonefrontlefty() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool UsEmDetectionZoneCfg::has_detzonefrontlefty() const {
  return _internal_has_detzonefrontlefty();
}
inline void UsEmDetectionZoneCfg::clear_detzonefrontlefty() {
  detzonefrontlefty_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline float UsEmDetectionZoneCfg::_internal_detzonefrontlefty() const {
  return detzonefrontlefty_;
}
inline float UsEmDetectionZoneCfg::detzonefrontlefty() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneFrontLeftY)
  return _internal_detzonefrontlefty();
}
inline void UsEmDetectionZoneCfg::_internal_set_detzonefrontlefty(float value) {
  _has_bits_[0] |= 0x00000010u;
  detzonefrontlefty_ = value;
}
inline void UsEmDetectionZoneCfg::set_detzonefrontlefty(float value) {
  _internal_set_detzonefrontlefty(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneFrontLeftY)
}

// optional float detZoneFrontRightX = 4088;
inline bool UsEmDetectionZoneCfg::_internal_has_detzonefrontrightx() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool UsEmDetectionZoneCfg::has_detzonefrontrightx() const {
  return _internal_has_detzonefrontrightx();
}
inline void UsEmDetectionZoneCfg::clear_detzonefrontrightx() {
  detzonefrontrightx_ = 0;
  _has_bits_[0] &= ~0x00000080u;
}
inline float UsEmDetectionZoneCfg::_internal_detzonefrontrightx() const {
  return detzonefrontrightx_;
}
inline float UsEmDetectionZoneCfg::detzonefrontrightx() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneFrontRightX)
  return _internal_detzonefrontrightx();
}
inline void UsEmDetectionZoneCfg::_internal_set_detzonefrontrightx(float value) {
  _has_bits_[0] |= 0x00000080u;
  detzonefrontrightx_ = value;
}
inline void UsEmDetectionZoneCfg::set_detzonefrontrightx(float value) {
  _internal_set_detzonefrontrightx(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneFrontRightX)
}

// optional float detZoneFrontRightY = 4057;
inline bool UsEmDetectionZoneCfg::_internal_has_detzonefrontrighty() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool UsEmDetectionZoneCfg::has_detzonefrontrighty() const {
  return _internal_has_detzonefrontrighty();
}
inline void UsEmDetectionZoneCfg::clear_detzonefrontrighty() {
  detzonefrontrighty_ = 0;
  _has_bits_[0] &= ~0x00000040u;
}
inline float UsEmDetectionZoneCfg::_internal_detzonefrontrighty() const {
  return detzonefrontrighty_;
}
inline float UsEmDetectionZoneCfg::detzonefrontrighty() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneFrontRightY)
  return _internal_detzonefrontrighty();
}
inline void UsEmDetectionZoneCfg::_internal_set_detzonefrontrighty(float value) {
  _has_bits_[0] |= 0x00000040u;
  detzonefrontrighty_ = value;
}
inline void UsEmDetectionZoneCfg::set_detzonefrontrighty(float value) {
  _internal_set_detzonefrontrighty(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneFrontRightY)
}

// optional float detZoneBackLeftX = 1184;
inline bool UsEmDetectionZoneCfg::_internal_has_detzonebackleftx() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool UsEmDetectionZoneCfg::has_detzonebackleftx() const {
  return _internal_has_detzonebackleftx();
}
inline void UsEmDetectionZoneCfg::clear_detzonebackleftx() {
  detzonebackleftx_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float UsEmDetectionZoneCfg::_internal_detzonebackleftx() const {
  return detzonebackleftx_;
}
inline float UsEmDetectionZoneCfg::detzonebackleftx() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneBackLeftX)
  return _internal_detzonebackleftx();
}
inline void UsEmDetectionZoneCfg::_internal_set_detzonebackleftx(float value) {
  _has_bits_[0] |= 0x00000008u;
  detzonebackleftx_ = value;
}
inline void UsEmDetectionZoneCfg::set_detzonebackleftx(float value) {
  _internal_set_detzonebackleftx(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneBackLeftX)
}

// optional float detZoneBackLeftY = 1153;
inline bool UsEmDetectionZoneCfg::_internal_has_detzonebacklefty() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool UsEmDetectionZoneCfg::has_detzonebacklefty() const {
  return _internal_has_detzonebacklefty();
}
inline void UsEmDetectionZoneCfg::clear_detzonebacklefty() {
  detzonebacklefty_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float UsEmDetectionZoneCfg::_internal_detzonebacklefty() const {
  return detzonebacklefty_;
}
inline float UsEmDetectionZoneCfg::detzonebacklefty() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneBackLeftY)
  return _internal_detzonebacklefty();
}
inline void UsEmDetectionZoneCfg::_internal_set_detzonebacklefty(float value) {
  _has_bits_[0] |= 0x00000004u;
  detzonebacklefty_ = value;
}
inline void UsEmDetectionZoneCfg::set_detzonebacklefty(float value) {
  _internal_set_detzonebacklefty(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneBackLeftY)
}

// optional float detZoneBackRightX = 396;
inline bool UsEmDetectionZoneCfg::_internal_has_detzonebackrightx() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool UsEmDetectionZoneCfg::has_detzonebackrightx() const {
  return _internal_has_detzonebackrightx();
}
inline void UsEmDetectionZoneCfg::clear_detzonebackrightx() {
  detzonebackrightx_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline float UsEmDetectionZoneCfg::_internal_detzonebackrightx() const {
  return detzonebackrightx_;
}
inline float UsEmDetectionZoneCfg::detzonebackrightx() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneBackRightX)
  return _internal_detzonebackrightx();
}
inline void UsEmDetectionZoneCfg::_internal_set_detzonebackrightx(float value) {
  _has_bits_[0] |= 0x00000001u;
  detzonebackrightx_ = value;
}
inline void UsEmDetectionZoneCfg::set_detzonebackrightx(float value) {
  _internal_set_detzonebackrightx(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneBackRightX)
}

// optional float detZoneBackRightY = 429;
inline bool UsEmDetectionZoneCfg::_internal_has_detzonebackrighty() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool UsEmDetectionZoneCfg::has_detzonebackrighty() const {
  return _internal_has_detzonebackrighty();
}
inline void UsEmDetectionZoneCfg::clear_detzonebackrighty() {
  detzonebackrighty_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline float UsEmDetectionZoneCfg::_internal_detzonebackrighty() const {
  return detzonebackrighty_;
}
inline float UsEmDetectionZoneCfg::detzonebackrighty() const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneBackRightY)
  return _internal_detzonebackrighty();
}
inline void UsEmDetectionZoneCfg::_internal_set_detzonebackrighty(float value) {
  _has_bits_[0] |= 0x00000002u;
  detzonebackrighty_ = value;
}
inline void UsEmDetectionZoneCfg::set_detzonebackrighty(float value) {
  _internal_set_detzonebackrighty(value);
  // @@protoc_insertion_point(field_set:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg.detZoneBackRightY)
}

// -------------------------------------------------------------------

// UsEmDetectionZoneCfg_array_port

// repeated .pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg data = 3007;
inline int UsEmDetectionZoneCfg_array_port::_internal_data_size() const {
  return data_.size();
}
inline int UsEmDetectionZoneCfg_array_port::data_size() const {
  return _internal_data_size();
}
inline void UsEmDetectionZoneCfg_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* UsEmDetectionZoneCfg_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg >*
UsEmDetectionZoneCfg_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg_array_port.data)
  return &data_;
}
inline const ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg& UsEmDetectionZoneCfg_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg& UsEmDetectionZoneCfg_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg_array_port.data)
  return _internal_data(index);
}
inline ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* UsEmDetectionZoneCfg_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg* UsEmDetectionZoneCfg_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::us_em::us_em_detection_zone_cfg::UsEmDetectionZoneCfg >&
UsEmDetectionZoneCfg_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.us_em.us_em_detection_zone_cfg.UsEmDetectionZoneCfg_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace us_em_detection_zone_cfg
}  // namespace us_em
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_us_5fem_2fus_5fem_5fdetection_5fzone_5fcfg_2eproto
