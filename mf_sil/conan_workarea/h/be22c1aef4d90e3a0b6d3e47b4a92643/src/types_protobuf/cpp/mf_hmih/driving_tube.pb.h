// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_hmih/driving_tube.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fdriving_5ftube_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fdriving_5ftube_2eproto

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
#include "pdcp/drv_tube_display.pb.h"
#include "mf_hmih/drv_tube_direction.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fhmih_2fdriving_5ftube_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fhmih_2fdriving_5ftube_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fhmih_2fdriving_5ftube_2eproto;
namespace pb {
namespace mf_hmih {
namespace driving_tube {
class DrivingTube;
class DrivingTubeDefaultTypeInternal;
extern DrivingTubeDefaultTypeInternal _DrivingTube_default_instance_;
class DrivingTube_array_port;
class DrivingTube_array_portDefaultTypeInternal;
extern DrivingTube_array_portDefaultTypeInternal _DrivingTube_array_port_default_instance_;
}  // namespace driving_tube
}  // namespace mf_hmih
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_hmih::driving_tube::DrivingTube* Arena::CreateMaybeMessage<::pb::mf_hmih::driving_tube::DrivingTube>(Arena*);
template<> ::pb::mf_hmih::driving_tube::DrivingTube_array_port* Arena::CreateMaybeMessage<::pb::mf_hmih::driving_tube::DrivingTube_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_hmih {
namespace driving_tube {

// ===================================================================

class DrivingTube :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.driving_tube.DrivingTube) */ {
 public:
  DrivingTube();
  virtual ~DrivingTube();

  DrivingTube(const DrivingTube& from);
  DrivingTube(DrivingTube&& from) noexcept
    : DrivingTube() {
    *this = ::std::move(from);
  }

  inline DrivingTube& operator=(const DrivingTube& from) {
    CopyFrom(from);
    return *this;
  }
  inline DrivingTube& operator=(DrivingTube&& from) noexcept {
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
  static const DrivingTube& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DrivingTube* internal_default_instance() {
    return reinterpret_cast<const DrivingTube*>(
               &_DrivingTube_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(DrivingTube& a, DrivingTube& b) {
    a.Swap(&b);
  }
  inline void Swap(DrivingTube* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DrivingTube* New() const final {
    return CreateMaybeMessage<DrivingTube>(nullptr);
  }

  DrivingTube* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DrivingTube>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const DrivingTube& from);
  void MergeFrom(const DrivingTube& from);
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
  void InternalSwap(DrivingTube* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.driving_tube.DrivingTube";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fdriving_5ftube_2eproto);
    return ::descriptor_table_mf_5fhmih_2fdriving_5ftube_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDrvTubeDirectionNuFieldNumber = 273,
    kRearRadiusCmFieldNumber = 1398,
    kFrontRadiusCmFieldNumber = 3127,
    kDrvTubeDisplayNuFieldNumber = 3130,
  };
  // optional .pb.mf_hmih.drv_tube_direction.DrvTubeDirection drvTubeDirection_nu = 273;
  bool has_drvtubedirection_nu() const;
  private:
  bool _internal_has_drvtubedirection_nu() const;
  public:
  void clear_drvtubedirection_nu();
  ::pb::mf_hmih::drv_tube_direction::DrvTubeDirection drvtubedirection_nu() const;
  void set_drvtubedirection_nu(::pb::mf_hmih::drv_tube_direction::DrvTubeDirection value);
  private:
  ::pb::mf_hmih::drv_tube_direction::DrvTubeDirection _internal_drvtubedirection_nu() const;
  void _internal_set_drvtubedirection_nu(::pb::mf_hmih::drv_tube_direction::DrvTubeDirection value);
  public:

  // optional uint32 rearRadius_cm = 1398;
  bool has_rearradius_cm() const;
  private:
  bool _internal_has_rearradius_cm() const;
  public:
  void clear_rearradius_cm();
  ::PROTOBUF_NAMESPACE_ID::uint32 rearradius_cm() const;
  void set_rearradius_cm(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_rearradius_cm() const;
  void _internal_set_rearradius_cm(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 frontRadius_cm = 3127;
  bool has_frontradius_cm() const;
  private:
  bool _internal_has_frontradius_cm() const;
  public:
  void clear_frontradius_cm();
  ::PROTOBUF_NAMESPACE_ID::uint32 frontradius_cm() const;
  void set_frontradius_cm(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_frontradius_cm() const;
  void _internal_set_frontradius_cm(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional .pb.pdcp.drv_tube_display.DrvTubeDisplay drvTubeDisplay_nu = 3130;
  bool has_drvtubedisplay_nu() const;
  private:
  bool _internal_has_drvtubedisplay_nu() const;
  public:
  void clear_drvtubedisplay_nu();
  ::pb::pdcp::drv_tube_display::DrvTubeDisplay drvtubedisplay_nu() const;
  void set_drvtubedisplay_nu(::pb::pdcp::drv_tube_display::DrvTubeDisplay value);
  private:
  ::pb::pdcp::drv_tube_display::DrvTubeDisplay _internal_drvtubedisplay_nu() const;
  void _internal_set_drvtubedisplay_nu(::pb::pdcp::drv_tube_display::DrvTubeDisplay value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.driving_tube.DrivingTube)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int drvtubedirection_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 rearradius_cm_;
  ::PROTOBUF_NAMESPACE_ID::uint32 frontradius_cm_;
  int drvtubedisplay_nu_;
  friend struct ::TableStruct_mf_5fhmih_2fdriving_5ftube_2eproto;
};
// -------------------------------------------------------------------

class DrivingTube_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.driving_tube.DrivingTube_array_port) */ {
 public:
  DrivingTube_array_port();
  virtual ~DrivingTube_array_port();

  DrivingTube_array_port(const DrivingTube_array_port& from);
  DrivingTube_array_port(DrivingTube_array_port&& from) noexcept
    : DrivingTube_array_port() {
    *this = ::std::move(from);
  }

  inline DrivingTube_array_port& operator=(const DrivingTube_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline DrivingTube_array_port& operator=(DrivingTube_array_port&& from) noexcept {
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
  static const DrivingTube_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DrivingTube_array_port* internal_default_instance() {
    return reinterpret_cast<const DrivingTube_array_port*>(
               &_DrivingTube_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(DrivingTube_array_port& a, DrivingTube_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(DrivingTube_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DrivingTube_array_port* New() const final {
    return CreateMaybeMessage<DrivingTube_array_port>(nullptr);
  }

  DrivingTube_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DrivingTube_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const DrivingTube_array_port& from);
  void MergeFrom(const DrivingTube_array_port& from);
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
  void InternalSwap(DrivingTube_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.driving_tube.DrivingTube_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fdriving_5ftube_2eproto);
    return ::descriptor_table_mf_5fhmih_2fdriving_5ftube_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1694,
  };
  // repeated .pb.mf_hmih.driving_tube.DrivingTube data = 1694;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_hmih::driving_tube::DrivingTube* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::driving_tube::DrivingTube >*
      mutable_data();
  private:
  const ::pb::mf_hmih::driving_tube::DrivingTube& _internal_data(int index) const;
  ::pb::mf_hmih::driving_tube::DrivingTube* _internal_add_data();
  public:
  const ::pb::mf_hmih::driving_tube::DrivingTube& data(int index) const;
  ::pb::mf_hmih::driving_tube::DrivingTube* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::driving_tube::DrivingTube >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.driving_tube.DrivingTube_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::driving_tube::DrivingTube > data_;
  friend struct ::TableStruct_mf_5fhmih_2fdriving_5ftube_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// DrivingTube

// optional uint32 frontRadius_cm = 3127;
inline bool DrivingTube::_internal_has_frontradius_cm() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool DrivingTube::has_frontradius_cm() const {
  return _internal_has_frontradius_cm();
}
inline void DrivingTube::clear_frontradius_cm() {
  frontradius_cm_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DrivingTube::_internal_frontradius_cm() const {
  return frontradius_cm_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DrivingTube::frontradius_cm() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.driving_tube.DrivingTube.frontRadius_cm)
  return _internal_frontradius_cm();
}
inline void DrivingTube::_internal_set_frontradius_cm(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  frontradius_cm_ = value;
}
inline void DrivingTube::set_frontradius_cm(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_frontradius_cm(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.driving_tube.DrivingTube.frontRadius_cm)
}

// optional uint32 rearRadius_cm = 1398;
inline bool DrivingTube::_internal_has_rearradius_cm() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool DrivingTube::has_rearradius_cm() const {
  return _internal_has_rearradius_cm();
}
inline void DrivingTube::clear_rearradius_cm() {
  rearradius_cm_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DrivingTube::_internal_rearradius_cm() const {
  return rearradius_cm_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DrivingTube::rearradius_cm() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.driving_tube.DrivingTube.rearRadius_cm)
  return _internal_rearradius_cm();
}
inline void DrivingTube::_internal_set_rearradius_cm(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  rearradius_cm_ = value;
}
inline void DrivingTube::set_rearradius_cm(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_rearradius_cm(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.driving_tube.DrivingTube.rearRadius_cm)
}

// optional .pb.pdcp.drv_tube_display.DrvTubeDisplay drvTubeDisplay_nu = 3130;
inline bool DrivingTube::_internal_has_drvtubedisplay_nu() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool DrivingTube::has_drvtubedisplay_nu() const {
  return _internal_has_drvtubedisplay_nu();
}
inline void DrivingTube::clear_drvtubedisplay_nu() {
  drvtubedisplay_nu_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::pb::pdcp::drv_tube_display::DrvTubeDisplay DrivingTube::_internal_drvtubedisplay_nu() const {
  return static_cast< ::pb::pdcp::drv_tube_display::DrvTubeDisplay >(drvtubedisplay_nu_);
}
inline ::pb::pdcp::drv_tube_display::DrvTubeDisplay DrivingTube::drvtubedisplay_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.driving_tube.DrivingTube.drvTubeDisplay_nu)
  return _internal_drvtubedisplay_nu();
}
inline void DrivingTube::_internal_set_drvtubedisplay_nu(::pb::pdcp::drv_tube_display::DrvTubeDisplay value) {
  assert(::pb::pdcp::drv_tube_display::DrvTubeDisplay_IsValid(value));
  _has_bits_[0] |= 0x00000008u;
  drvtubedisplay_nu_ = value;
}
inline void DrivingTube::set_drvtubedisplay_nu(::pb::pdcp::drv_tube_display::DrvTubeDisplay value) {
  _internal_set_drvtubedisplay_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.driving_tube.DrivingTube.drvTubeDisplay_nu)
}

// optional .pb.mf_hmih.drv_tube_direction.DrvTubeDirection drvTubeDirection_nu = 273;
inline bool DrivingTube::_internal_has_drvtubedirection_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool DrivingTube::has_drvtubedirection_nu() const {
  return _internal_has_drvtubedirection_nu();
}
inline void DrivingTube::clear_drvtubedirection_nu() {
  drvtubedirection_nu_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::pb::mf_hmih::drv_tube_direction::DrvTubeDirection DrivingTube::_internal_drvtubedirection_nu() const {
  return static_cast< ::pb::mf_hmih::drv_tube_direction::DrvTubeDirection >(drvtubedirection_nu_);
}
inline ::pb::mf_hmih::drv_tube_direction::DrvTubeDirection DrivingTube::drvtubedirection_nu() const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.driving_tube.DrivingTube.drvTubeDirection_nu)
  return _internal_drvtubedirection_nu();
}
inline void DrivingTube::_internal_set_drvtubedirection_nu(::pb::mf_hmih::drv_tube_direction::DrvTubeDirection value) {
  assert(::pb::mf_hmih::drv_tube_direction::DrvTubeDirection_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  drvtubedirection_nu_ = value;
}
inline void DrivingTube::set_drvtubedirection_nu(::pb::mf_hmih::drv_tube_direction::DrvTubeDirection value) {
  _internal_set_drvtubedirection_nu(value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.driving_tube.DrivingTube.drvTubeDirection_nu)
}

// -------------------------------------------------------------------

// DrivingTube_array_port

// repeated .pb.mf_hmih.driving_tube.DrivingTube data = 1694;
inline int DrivingTube_array_port::_internal_data_size() const {
  return data_.size();
}
inline int DrivingTube_array_port::data_size() const {
  return _internal_data_size();
}
inline void DrivingTube_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_hmih::driving_tube::DrivingTube* DrivingTube_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.driving_tube.DrivingTube_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::driving_tube::DrivingTube >*
DrivingTube_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.driving_tube.DrivingTube_array_port.data)
  return &data_;
}
inline const ::pb::mf_hmih::driving_tube::DrivingTube& DrivingTube_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_hmih::driving_tube::DrivingTube& DrivingTube_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.driving_tube.DrivingTube_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_hmih::driving_tube::DrivingTube* DrivingTube_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_hmih::driving_tube::DrivingTube* DrivingTube_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_hmih.driving_tube.DrivingTube_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::driving_tube::DrivingTube >&
DrivingTube_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.driving_tube.DrivingTube_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace driving_tube
}  // namespace mf_hmih
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fdriving_5ftube_2eproto
