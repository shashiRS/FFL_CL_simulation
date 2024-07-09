// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/dynamic_object_serializable.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_si_2fdynamic_5fobject_5fserializable_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_si_2fdynamic_5fobject_5fserializable_2eproto

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
#include "si/dynamic_obj_shape_serializable.pb.h"
#include "cml/vec2_df_pod.pb.h"
#include "si/obj_measurement_state.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_si_2fdynamic_5fobject_5fserializable_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_si_2fdynamic_5fobject_5fserializable_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fdynamic_5fobject_5fserializable_2eproto;
namespace pb {
namespace si {
namespace dynamic_object_serializable {
class DynamicObjectSerializable;
class DynamicObjectSerializableDefaultTypeInternal;
extern DynamicObjectSerializableDefaultTypeInternal _DynamicObjectSerializable_default_instance_;
class DynamicObjectSerializable_array_port;
class DynamicObjectSerializable_array_portDefaultTypeInternal;
extern DynamicObjectSerializable_array_portDefaultTypeInternal _DynamicObjectSerializable_array_port_default_instance_;
}  // namespace dynamic_object_serializable
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::si::dynamic_object_serializable::DynamicObjectSerializable* Arena::CreateMaybeMessage<::pb::si::dynamic_object_serializable::DynamicObjectSerializable>(Arena*);
template<> ::pb::si::dynamic_object_serializable::DynamicObjectSerializable_array_port* Arena::CreateMaybeMessage<::pb::si::dynamic_object_serializable::DynamicObjectSerializable_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace si {
namespace dynamic_object_serializable {

// ===================================================================

class DynamicObjectSerializable :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.dynamic_object_serializable.DynamicObjectSerializable) */ {
 public:
  DynamicObjectSerializable();
  virtual ~DynamicObjectSerializable();

  DynamicObjectSerializable(const DynamicObjectSerializable& from);
  DynamicObjectSerializable(DynamicObjectSerializable&& from) noexcept
    : DynamicObjectSerializable() {
    *this = ::std::move(from);
  }

  inline DynamicObjectSerializable& operator=(const DynamicObjectSerializable& from) {
    CopyFrom(from);
    return *this;
  }
  inline DynamicObjectSerializable& operator=(DynamicObjectSerializable&& from) noexcept {
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
  static const DynamicObjectSerializable& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DynamicObjectSerializable* internal_default_instance() {
    return reinterpret_cast<const DynamicObjectSerializable*>(
               &_DynamicObjectSerializable_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(DynamicObjectSerializable& a, DynamicObjectSerializable& b) {
    a.Swap(&b);
  }
  inline void Swap(DynamicObjectSerializable* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DynamicObjectSerializable* New() const final {
    return CreateMaybeMessage<DynamicObjectSerializable>(nullptr);
  }

  DynamicObjectSerializable* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DynamicObjectSerializable>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const DynamicObjectSerializable& from);
  void MergeFrom(const DynamicObjectSerializable& from);
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
  void InternalSwap(DynamicObjectSerializable* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.dynamic_object_serializable.DynamicObjectSerializable";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fdynamic_5fobject_5fserializable_2eproto);
    return ::descriptor_table_si_2fdynamic_5fobject_5fserializable_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kAccelMps2FieldNumber = 1603,
    kObjShapeMFieldNumber = 2729,
    kVelMpsFieldNumber = 3009,
    kHeadingAngleRadFieldNumber = 748,
    kExistenceProbPercFieldNumber = 2424,
    kMeasurementStateNuFieldNumber = 3154,
    kRefObjIDNuFieldNumber = 3288,
  };
  // optional .pb.cml.vec2_df_pod.Vec2Df_POD accel_mps2 = 1603;
  bool has_accel_mps2() const;
  private:
  bool _internal_has_accel_mps2() const;
  public:
  void clear_accel_mps2();
  const ::pb::cml::vec2_df_pod::Vec2Df_POD& accel_mps2() const;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* release_accel_mps2();
  ::pb::cml::vec2_df_pod::Vec2Df_POD* mutable_accel_mps2();
  void set_allocated_accel_mps2(::pb::cml::vec2_df_pod::Vec2Df_POD* accel_mps2);
  private:
  const ::pb::cml::vec2_df_pod::Vec2Df_POD& _internal_accel_mps2() const;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* _internal_mutable_accel_mps2();
  public:

  // optional .pb.si.dynamic_obj_shape_serializable.DynamicObjShapeSerializable objShape_m = 2729;
  bool has_objshape_m() const;
  private:
  bool _internal_has_objshape_m() const;
  public:
  void clear_objshape_m();
  const ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable& objshape_m() const;
  ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable* release_objshape_m();
  ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable* mutable_objshape_m();
  void set_allocated_objshape_m(::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable* objshape_m);
  private:
  const ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable& _internal_objshape_m() const;
  ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable* _internal_mutable_objshape_m();
  public:

  // optional .pb.cml.vec2_df_pod.Vec2Df_POD vel_mps = 3009;
  bool has_vel_mps() const;
  private:
  bool _internal_has_vel_mps() const;
  public:
  void clear_vel_mps();
  const ::pb::cml::vec2_df_pod::Vec2Df_POD& vel_mps() const;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* release_vel_mps();
  ::pb::cml::vec2_df_pod::Vec2Df_POD* mutable_vel_mps();
  void set_allocated_vel_mps(::pb::cml::vec2_df_pod::Vec2Df_POD* vel_mps);
  private:
  const ::pb::cml::vec2_df_pod::Vec2Df_POD& _internal_vel_mps() const;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* _internal_mutable_vel_mps();
  public:

  // optional float headingAngle_rad = 748;
  bool has_headingangle_rad() const;
  private:
  bool _internal_has_headingangle_rad() const;
  public:
  void clear_headingangle_rad();
  float headingangle_rad() const;
  void set_headingangle_rad(float value);
  private:
  float _internal_headingangle_rad() const;
  void _internal_set_headingangle_rad(float value);
  public:

  // optional uint32 existenceProb_perc = 2424;
  bool has_existenceprob_perc() const;
  private:
  bool _internal_has_existenceprob_perc() const;
  public:
  void clear_existenceprob_perc();
  ::PROTOBUF_NAMESPACE_ID::uint32 existenceprob_perc() const;
  void set_existenceprob_perc(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_existenceprob_perc() const;
  void _internal_set_existenceprob_perc(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional .pb.si.obj_measurement_state.ObjMeasurementState measurementState_nu = 3154;
  bool has_measurementstate_nu() const;
  private:
  bool _internal_has_measurementstate_nu() const;
  public:
  void clear_measurementstate_nu();
  ::pb::si::obj_measurement_state::ObjMeasurementState measurementstate_nu() const;
  void set_measurementstate_nu(::pb::si::obj_measurement_state::ObjMeasurementState value);
  private:
  ::pb::si::obj_measurement_state::ObjMeasurementState _internal_measurementstate_nu() const;
  void _internal_set_measurementstate_nu(::pb::si::obj_measurement_state::ObjMeasurementState value);
  public:

  // optional uint32 refObjID_nu = 3288;
  bool has_refobjid_nu() const;
  private:
  bool _internal_has_refobjid_nu() const;
  public:
  void clear_refobjid_nu();
  ::PROTOBUF_NAMESPACE_ID::uint32 refobjid_nu() const;
  void set_refobjid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_refobjid_nu() const;
  void _internal_set_refobjid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.si.dynamic_object_serializable.DynamicObjectSerializable)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* accel_mps2_;
  ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable* objshape_m_;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* vel_mps_;
  float headingangle_rad_;
  ::PROTOBUF_NAMESPACE_ID::uint32 existenceprob_perc_;
  int measurementstate_nu_;
  ::PROTOBUF_NAMESPACE_ID::uint32 refobjid_nu_;
  friend struct ::TableStruct_si_2fdynamic_5fobject_5fserializable_2eproto;
};
// -------------------------------------------------------------------

class DynamicObjectSerializable_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.si.dynamic_object_serializable.DynamicObjectSerializable_array_port) */ {
 public:
  DynamicObjectSerializable_array_port();
  virtual ~DynamicObjectSerializable_array_port();

  DynamicObjectSerializable_array_port(const DynamicObjectSerializable_array_port& from);
  DynamicObjectSerializable_array_port(DynamicObjectSerializable_array_port&& from) noexcept
    : DynamicObjectSerializable_array_port() {
    *this = ::std::move(from);
  }

  inline DynamicObjectSerializable_array_port& operator=(const DynamicObjectSerializable_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline DynamicObjectSerializable_array_port& operator=(DynamicObjectSerializable_array_port&& from) noexcept {
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
  static const DynamicObjectSerializable_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DynamicObjectSerializable_array_port* internal_default_instance() {
    return reinterpret_cast<const DynamicObjectSerializable_array_port*>(
               &_DynamicObjectSerializable_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(DynamicObjectSerializable_array_port& a, DynamicObjectSerializable_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(DynamicObjectSerializable_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DynamicObjectSerializable_array_port* New() const final {
    return CreateMaybeMessage<DynamicObjectSerializable_array_port>(nullptr);
  }

  DynamicObjectSerializable_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DynamicObjectSerializable_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const DynamicObjectSerializable_array_port& from);
  void MergeFrom(const DynamicObjectSerializable_array_port& from);
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
  void InternalSwap(DynamicObjectSerializable_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.si.dynamic_object_serializable.DynamicObjectSerializable_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_si_2fdynamic_5fobject_5fserializable_2eproto);
    return ::descriptor_table_si_2fdynamic_5fobject_5fserializable_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2295,
  };
  // repeated .pb.si.dynamic_object_serializable.DynamicObjectSerializable data = 2295;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::si::dynamic_object_serializable::DynamicObjectSerializable* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::dynamic_object_serializable::DynamicObjectSerializable >*
      mutable_data();
  private:
  const ::pb::si::dynamic_object_serializable::DynamicObjectSerializable& _internal_data(int index) const;
  ::pb::si::dynamic_object_serializable::DynamicObjectSerializable* _internal_add_data();
  public:
  const ::pb::si::dynamic_object_serializable::DynamicObjectSerializable& data(int index) const;
  ::pb::si::dynamic_object_serializable::DynamicObjectSerializable* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::dynamic_object_serializable::DynamicObjectSerializable >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.si.dynamic_object_serializable.DynamicObjectSerializable_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::dynamic_object_serializable::DynamicObjectSerializable > data_;
  friend struct ::TableStruct_si_2fdynamic_5fobject_5fserializable_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// DynamicObjectSerializable

// optional uint32 existenceProb_perc = 2424;
inline bool DynamicObjectSerializable::_internal_has_existenceprob_perc() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool DynamicObjectSerializable::has_existenceprob_perc() const {
  return _internal_has_existenceprob_perc();
}
inline void DynamicObjectSerializable::clear_existenceprob_perc() {
  existenceprob_perc_ = 0u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DynamicObjectSerializable::_internal_existenceprob_perc() const {
  return existenceprob_perc_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DynamicObjectSerializable::existenceprob_perc() const {
  // @@protoc_insertion_point(field_get:pb.si.dynamic_object_serializable.DynamicObjectSerializable.existenceProb_perc)
  return _internal_existenceprob_perc();
}
inline void DynamicObjectSerializable::_internal_set_existenceprob_perc(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  existenceprob_perc_ = value;
}
inline void DynamicObjectSerializable::set_existenceprob_perc(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_existenceprob_perc(value);
  // @@protoc_insertion_point(field_set:pb.si.dynamic_object_serializable.DynamicObjectSerializable.existenceProb_perc)
}

// optional .pb.si.dynamic_obj_shape_serializable.DynamicObjShapeSerializable objShape_m = 2729;
inline bool DynamicObjectSerializable::_internal_has_objshape_m() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || objshape_m_ != nullptr);
  return value;
}
inline bool DynamicObjectSerializable::has_objshape_m() const {
  return _internal_has_objshape_m();
}
inline const ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable& DynamicObjectSerializable::_internal_objshape_m() const {
  const ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable* p = objshape_m_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable*>(
      &::pb::si::dynamic_obj_shape_serializable::_DynamicObjShapeSerializable_default_instance_);
}
inline const ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable& DynamicObjectSerializable::objshape_m() const {
  // @@protoc_insertion_point(field_get:pb.si.dynamic_object_serializable.DynamicObjectSerializable.objShape_m)
  return _internal_objshape_m();
}
inline ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable* DynamicObjectSerializable::release_objshape_m() {
  // @@protoc_insertion_point(field_release:pb.si.dynamic_object_serializable.DynamicObjectSerializable.objShape_m)
  _has_bits_[0] &= ~0x00000002u;
  ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable* temp = objshape_m_;
  objshape_m_ = nullptr;
  return temp;
}
inline ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable* DynamicObjectSerializable::_internal_mutable_objshape_m() {
  _has_bits_[0] |= 0x00000002u;
  if (objshape_m_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable>(GetArenaNoVirtual());
    objshape_m_ = p;
  }
  return objshape_m_;
}
inline ::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable* DynamicObjectSerializable::mutable_objshape_m() {
  // @@protoc_insertion_point(field_mutable:pb.si.dynamic_object_serializable.DynamicObjectSerializable.objShape_m)
  return _internal_mutable_objshape_m();
}
inline void DynamicObjectSerializable::set_allocated_objshape_m(::pb::si::dynamic_obj_shape_serializable::DynamicObjShapeSerializable* objshape_m) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(objshape_m_);
  }
  if (objshape_m) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      objshape_m = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, objshape_m, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  objshape_m_ = objshape_m;
  // @@protoc_insertion_point(field_set_allocated:pb.si.dynamic_object_serializable.DynamicObjectSerializable.objShape_m)
}

// optional .pb.cml.vec2_df_pod.Vec2Df_POD vel_mps = 3009;
inline bool DynamicObjectSerializable::_internal_has_vel_mps() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || vel_mps_ != nullptr);
  return value;
}
inline bool DynamicObjectSerializable::has_vel_mps() const {
  return _internal_has_vel_mps();
}
inline const ::pb::cml::vec2_df_pod::Vec2Df_POD& DynamicObjectSerializable::_internal_vel_mps() const {
  const ::pb::cml::vec2_df_pod::Vec2Df_POD* p = vel_mps_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::cml::vec2_df_pod::Vec2Df_POD*>(
      &::pb::cml::vec2_df_pod::_Vec2Df_POD_default_instance_);
}
inline const ::pb::cml::vec2_df_pod::Vec2Df_POD& DynamicObjectSerializable::vel_mps() const {
  // @@protoc_insertion_point(field_get:pb.si.dynamic_object_serializable.DynamicObjectSerializable.vel_mps)
  return _internal_vel_mps();
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* DynamicObjectSerializable::release_vel_mps() {
  // @@protoc_insertion_point(field_release:pb.si.dynamic_object_serializable.DynamicObjectSerializable.vel_mps)
  _has_bits_[0] &= ~0x00000004u;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* temp = vel_mps_;
  vel_mps_ = nullptr;
  return temp;
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* DynamicObjectSerializable::_internal_mutable_vel_mps() {
  _has_bits_[0] |= 0x00000004u;
  if (vel_mps_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::cml::vec2_df_pod::Vec2Df_POD>(GetArenaNoVirtual());
    vel_mps_ = p;
  }
  return vel_mps_;
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* DynamicObjectSerializable::mutable_vel_mps() {
  // @@protoc_insertion_point(field_mutable:pb.si.dynamic_object_serializable.DynamicObjectSerializable.vel_mps)
  return _internal_mutable_vel_mps();
}
inline void DynamicObjectSerializable::set_allocated_vel_mps(::pb::cml::vec2_df_pod::Vec2Df_POD* vel_mps) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(vel_mps_);
  }
  if (vel_mps) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      vel_mps = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, vel_mps, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  vel_mps_ = vel_mps;
  // @@protoc_insertion_point(field_set_allocated:pb.si.dynamic_object_serializable.DynamicObjectSerializable.vel_mps)
}

// optional .pb.cml.vec2_df_pod.Vec2Df_POD accel_mps2 = 1603;
inline bool DynamicObjectSerializable::_internal_has_accel_mps2() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || accel_mps2_ != nullptr);
  return value;
}
inline bool DynamicObjectSerializable::has_accel_mps2() const {
  return _internal_has_accel_mps2();
}
inline const ::pb::cml::vec2_df_pod::Vec2Df_POD& DynamicObjectSerializable::_internal_accel_mps2() const {
  const ::pb::cml::vec2_df_pod::Vec2Df_POD* p = accel_mps2_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::cml::vec2_df_pod::Vec2Df_POD*>(
      &::pb::cml::vec2_df_pod::_Vec2Df_POD_default_instance_);
}
inline const ::pb::cml::vec2_df_pod::Vec2Df_POD& DynamicObjectSerializable::accel_mps2() const {
  // @@protoc_insertion_point(field_get:pb.si.dynamic_object_serializable.DynamicObjectSerializable.accel_mps2)
  return _internal_accel_mps2();
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* DynamicObjectSerializable::release_accel_mps2() {
  // @@protoc_insertion_point(field_release:pb.si.dynamic_object_serializable.DynamicObjectSerializable.accel_mps2)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::cml::vec2_df_pod::Vec2Df_POD* temp = accel_mps2_;
  accel_mps2_ = nullptr;
  return temp;
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* DynamicObjectSerializable::_internal_mutable_accel_mps2() {
  _has_bits_[0] |= 0x00000001u;
  if (accel_mps2_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::cml::vec2_df_pod::Vec2Df_POD>(GetArenaNoVirtual());
    accel_mps2_ = p;
  }
  return accel_mps2_;
}
inline ::pb::cml::vec2_df_pod::Vec2Df_POD* DynamicObjectSerializable::mutable_accel_mps2() {
  // @@protoc_insertion_point(field_mutable:pb.si.dynamic_object_serializable.DynamicObjectSerializable.accel_mps2)
  return _internal_mutable_accel_mps2();
}
inline void DynamicObjectSerializable::set_allocated_accel_mps2(::pb::cml::vec2_df_pod::Vec2Df_POD* accel_mps2) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(accel_mps2_);
  }
  if (accel_mps2) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      accel_mps2 = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, accel_mps2, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  accel_mps2_ = accel_mps2;
  // @@protoc_insertion_point(field_set_allocated:pb.si.dynamic_object_serializable.DynamicObjectSerializable.accel_mps2)
}

// optional float headingAngle_rad = 748;
inline bool DynamicObjectSerializable::_internal_has_headingangle_rad() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool DynamicObjectSerializable::has_headingangle_rad() const {
  return _internal_has_headingangle_rad();
}
inline void DynamicObjectSerializable::clear_headingangle_rad() {
  headingangle_rad_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float DynamicObjectSerializable::_internal_headingangle_rad() const {
  return headingangle_rad_;
}
inline float DynamicObjectSerializable::headingangle_rad() const {
  // @@protoc_insertion_point(field_get:pb.si.dynamic_object_serializable.DynamicObjectSerializable.headingAngle_rad)
  return _internal_headingangle_rad();
}
inline void DynamicObjectSerializable::_internal_set_headingangle_rad(float value) {
  _has_bits_[0] |= 0x00000008u;
  headingangle_rad_ = value;
}
inline void DynamicObjectSerializable::set_headingangle_rad(float value) {
  _internal_set_headingangle_rad(value);
  // @@protoc_insertion_point(field_set:pb.si.dynamic_object_serializable.DynamicObjectSerializable.headingAngle_rad)
}

// optional .pb.si.obj_measurement_state.ObjMeasurementState measurementState_nu = 3154;
inline bool DynamicObjectSerializable::_internal_has_measurementstate_nu() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool DynamicObjectSerializable::has_measurementstate_nu() const {
  return _internal_has_measurementstate_nu();
}
inline void DynamicObjectSerializable::clear_measurementstate_nu() {
  measurementstate_nu_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::pb::si::obj_measurement_state::ObjMeasurementState DynamicObjectSerializable::_internal_measurementstate_nu() const {
  return static_cast< ::pb::si::obj_measurement_state::ObjMeasurementState >(measurementstate_nu_);
}
inline ::pb::si::obj_measurement_state::ObjMeasurementState DynamicObjectSerializable::measurementstate_nu() const {
  // @@protoc_insertion_point(field_get:pb.si.dynamic_object_serializable.DynamicObjectSerializable.measurementState_nu)
  return _internal_measurementstate_nu();
}
inline void DynamicObjectSerializable::_internal_set_measurementstate_nu(::pb::si::obj_measurement_state::ObjMeasurementState value) {
  assert(::pb::si::obj_measurement_state::ObjMeasurementState_IsValid(value));
  _has_bits_[0] |= 0x00000020u;
  measurementstate_nu_ = value;
}
inline void DynamicObjectSerializable::set_measurementstate_nu(::pb::si::obj_measurement_state::ObjMeasurementState value) {
  _internal_set_measurementstate_nu(value);
  // @@protoc_insertion_point(field_set:pb.si.dynamic_object_serializable.DynamicObjectSerializable.measurementState_nu)
}

// optional uint32 refObjID_nu = 3288;
inline bool DynamicObjectSerializable::_internal_has_refobjid_nu() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool DynamicObjectSerializable::has_refobjid_nu() const {
  return _internal_has_refobjid_nu();
}
inline void DynamicObjectSerializable::clear_refobjid_nu() {
  refobjid_nu_ = 0u;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DynamicObjectSerializable::_internal_refobjid_nu() const {
  return refobjid_nu_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 DynamicObjectSerializable::refobjid_nu() const {
  // @@protoc_insertion_point(field_get:pb.si.dynamic_object_serializable.DynamicObjectSerializable.refObjID_nu)
  return _internal_refobjid_nu();
}
inline void DynamicObjectSerializable::_internal_set_refobjid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000040u;
  refobjid_nu_ = value;
}
inline void DynamicObjectSerializable::set_refobjid_nu(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_refobjid_nu(value);
  // @@protoc_insertion_point(field_set:pb.si.dynamic_object_serializable.DynamicObjectSerializable.refObjID_nu)
}

// -------------------------------------------------------------------

// DynamicObjectSerializable_array_port

// repeated .pb.si.dynamic_object_serializable.DynamicObjectSerializable data = 2295;
inline int DynamicObjectSerializable_array_port::_internal_data_size() const {
  return data_.size();
}
inline int DynamicObjectSerializable_array_port::data_size() const {
  return _internal_data_size();
}
inline void DynamicObjectSerializable_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::si::dynamic_object_serializable::DynamicObjectSerializable* DynamicObjectSerializable_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.si.dynamic_object_serializable.DynamicObjectSerializable_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::dynamic_object_serializable::DynamicObjectSerializable >*
DynamicObjectSerializable_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.si.dynamic_object_serializable.DynamicObjectSerializable_array_port.data)
  return &data_;
}
inline const ::pb::si::dynamic_object_serializable::DynamicObjectSerializable& DynamicObjectSerializable_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::si::dynamic_object_serializable::DynamicObjectSerializable& DynamicObjectSerializable_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.si.dynamic_object_serializable.DynamicObjectSerializable_array_port.data)
  return _internal_data(index);
}
inline ::pb::si::dynamic_object_serializable::DynamicObjectSerializable* DynamicObjectSerializable_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::si::dynamic_object_serializable::DynamicObjectSerializable* DynamicObjectSerializable_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.si.dynamic_object_serializable.DynamicObjectSerializable_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::si::dynamic_object_serializable::DynamicObjectSerializable >&
DynamicObjectSerializable_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.si.dynamic_object_serializable.DynamicObjectSerializable_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace dynamic_object_serializable
}  // namespace si
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_si_2fdynamic_5fobject_5fserializable_2eproto
