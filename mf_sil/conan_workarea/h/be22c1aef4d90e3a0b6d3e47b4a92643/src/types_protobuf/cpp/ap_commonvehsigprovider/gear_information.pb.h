// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_commonvehsigprovider/gear_information.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fcommonvehsigprovider_2fgear_5finformation_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fcommonvehsigprovider_2fgear_5finformation_2eproto

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
#include "ap_commonvehsigprovider/gear_box_ctrl_system_state.pb.h"
#include "ap_commonvehsigprovider/gear.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ap_5fcommonvehsigprovider_2fgear_5finformation_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fcommonvehsigprovider_2fgear_5finformation_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fcommonvehsigprovider_2fgear_5finformation_2eproto;
namespace pb {
namespace ap_commonvehsigprovider {
namespace gear_information {
class GearInformation;
class GearInformationDefaultTypeInternal;
extern GearInformationDefaultTypeInternal _GearInformation_default_instance_;
class GearInformation_array_port;
class GearInformation_array_portDefaultTypeInternal;
extern GearInformation_array_portDefaultTypeInternal _GearInformation_array_port_default_instance_;
}  // namespace gear_information
}  // namespace ap_commonvehsigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_commonvehsigprovider::gear_information::GearInformation* Arena::CreateMaybeMessage<::pb::ap_commonvehsigprovider::gear_information::GearInformation>(Arena*);
template<> ::pb::ap_commonvehsigprovider::gear_information::GearInformation_array_port* Arena::CreateMaybeMessage<::pb::ap_commonvehsigprovider::gear_information::GearInformation_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_commonvehsigprovider {
namespace gear_information {

// ===================================================================

class GearInformation :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_commonvehsigprovider.gear_information.GearInformation) */ {
 public:
  GearInformation();
  virtual ~GearInformation();

  GearInformation(const GearInformation& from);
  GearInformation(GearInformation&& from) noexcept
    : GearInformation() {
    *this = ::std::move(from);
  }

  inline GearInformation& operator=(const GearInformation& from) {
    CopyFrom(from);
    return *this;
  }
  inline GearInformation& operator=(GearInformation&& from) noexcept {
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
  static const GearInformation& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const GearInformation* internal_default_instance() {
    return reinterpret_cast<const GearInformation*>(
               &_GearInformation_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(GearInformation& a, GearInformation& b) {
    a.Swap(&b);
  }
  inline void Swap(GearInformation* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline GearInformation* New() const final {
    return CreateMaybeMessage<GearInformation>(nullptr);
  }

  GearInformation* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<GearInformation>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const GearInformation& from);
  void MergeFrom(const GearInformation& from);
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
  void InternalSwap(GearInformation* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_commonvehsigprovider.gear_information.GearInformation";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fcommonvehsigprovider_2fgear_5finformation_2eproto);
    return ::descriptor_table_ap_5fcommonvehsigprovider_2fgear_5finformation_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kGearCurNuFieldNumber = 322,
    kGearboxCtrlSystemStateNuFieldNumber = 3770,
  };
  // optional .pb.ap_commonvehsigprovider.gear.Gear gearCur_nu = 322;
  bool has_gearcur_nu() const;
  private:
  bool _internal_has_gearcur_nu() const;
  public:
  void clear_gearcur_nu();
  ::pb::ap_commonvehsigprovider::gear::Gear gearcur_nu() const;
  void set_gearcur_nu(::pb::ap_commonvehsigprovider::gear::Gear value);
  private:
  ::pb::ap_commonvehsigprovider::gear::Gear _internal_gearcur_nu() const;
  void _internal_set_gearcur_nu(::pb::ap_commonvehsigprovider::gear::Gear value);
  public:

  // optional .pb.ap_commonvehsigprovider.gear_box_ctrl_system_state.GearBoxCtrlSystemState gearboxCtrlSystemState_nu = 3770;
  bool has_gearboxctrlsystemstate_nu() const;
  private:
  bool _internal_has_gearboxctrlsystemstate_nu() const;
  public:
  void clear_gearboxctrlsystemstate_nu();
  ::pb::ap_commonvehsigprovider::gear_box_ctrl_system_state::GearBoxCtrlSystemState gearboxctrlsystemstate_nu() const;
  void set_gearboxctrlsystemstate_nu(::pb::ap_commonvehsigprovider::gear_box_ctrl_system_state::GearBoxCtrlSystemState value);
  private:
  ::pb::ap_commonvehsigprovider::gear_box_ctrl_system_state::GearBoxCtrlSystemState _internal_gearboxctrlsystemstate_nu() const;
  void _internal_set_gearboxctrlsystemstate_nu(::pb::ap_commonvehsigprovider::gear_box_ctrl_system_state::GearBoxCtrlSystemState value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.gear_information.GearInformation)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int gearcur_nu_;
  int gearboxctrlsystemstate_nu_;
  friend struct ::TableStruct_ap_5fcommonvehsigprovider_2fgear_5finformation_2eproto;
};
// -------------------------------------------------------------------

class GearInformation_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_commonvehsigprovider.gear_information.GearInformation_array_port) */ {
 public:
  GearInformation_array_port();
  virtual ~GearInformation_array_port();

  GearInformation_array_port(const GearInformation_array_port& from);
  GearInformation_array_port(GearInformation_array_port&& from) noexcept
    : GearInformation_array_port() {
    *this = ::std::move(from);
  }

  inline GearInformation_array_port& operator=(const GearInformation_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline GearInformation_array_port& operator=(GearInformation_array_port&& from) noexcept {
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
  static const GearInformation_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const GearInformation_array_port* internal_default_instance() {
    return reinterpret_cast<const GearInformation_array_port*>(
               &_GearInformation_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(GearInformation_array_port& a, GearInformation_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(GearInformation_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline GearInformation_array_port* New() const final {
    return CreateMaybeMessage<GearInformation_array_port>(nullptr);
  }

  GearInformation_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<GearInformation_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const GearInformation_array_port& from);
  void MergeFrom(const GearInformation_array_port& from);
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
  void InternalSwap(GearInformation_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_commonvehsigprovider.gear_information.GearInformation_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fcommonvehsigprovider_2fgear_5finformation_2eproto);
    return ::descriptor_table_ap_5fcommonvehsigprovider_2fgear_5finformation_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2278,
  };
  // repeated .pb.ap_commonvehsigprovider.gear_information.GearInformation data = 2278;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_commonvehsigprovider::gear_information::GearInformation* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::gear_information::GearInformation >*
      mutable_data();
  private:
  const ::pb::ap_commonvehsigprovider::gear_information::GearInformation& _internal_data(int index) const;
  ::pb::ap_commonvehsigprovider::gear_information::GearInformation* _internal_add_data();
  public:
  const ::pb::ap_commonvehsigprovider::gear_information::GearInformation& data(int index) const;
  ::pb::ap_commonvehsigprovider::gear_information::GearInformation* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::gear_information::GearInformation >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_commonvehsigprovider.gear_information.GearInformation_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::gear_information::GearInformation > data_;
  friend struct ::TableStruct_ap_5fcommonvehsigprovider_2fgear_5finformation_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// GearInformation

// optional .pb.ap_commonvehsigprovider.gear_box_ctrl_system_state.GearBoxCtrlSystemState gearboxCtrlSystemState_nu = 3770;
inline bool GearInformation::_internal_has_gearboxctrlsystemstate_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool GearInformation::has_gearboxctrlsystemstate_nu() const {
  return _internal_has_gearboxctrlsystemstate_nu();
}
inline void GearInformation::clear_gearboxctrlsystemstate_nu() {
  gearboxctrlsystemstate_nu_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::pb::ap_commonvehsigprovider::gear_box_ctrl_system_state::GearBoxCtrlSystemState GearInformation::_internal_gearboxctrlsystemstate_nu() const {
  return static_cast< ::pb::ap_commonvehsigprovider::gear_box_ctrl_system_state::GearBoxCtrlSystemState >(gearboxctrlsystemstate_nu_);
}
inline ::pb::ap_commonvehsigprovider::gear_box_ctrl_system_state::GearBoxCtrlSystemState GearInformation::gearboxctrlsystemstate_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.gear_information.GearInformation.gearboxCtrlSystemState_nu)
  return _internal_gearboxctrlsystemstate_nu();
}
inline void GearInformation::_internal_set_gearboxctrlsystemstate_nu(::pb::ap_commonvehsigprovider::gear_box_ctrl_system_state::GearBoxCtrlSystemState value) {
  assert(::pb::ap_commonvehsigprovider::gear_box_ctrl_system_state::GearBoxCtrlSystemState_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  gearboxctrlsystemstate_nu_ = value;
}
inline void GearInformation::set_gearboxctrlsystemstate_nu(::pb::ap_commonvehsigprovider::gear_box_ctrl_system_state::GearBoxCtrlSystemState value) {
  _internal_set_gearboxctrlsystemstate_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.gear_information.GearInformation.gearboxCtrlSystemState_nu)
}

// optional .pb.ap_commonvehsigprovider.gear.Gear gearCur_nu = 322;
inline bool GearInformation::_internal_has_gearcur_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool GearInformation::has_gearcur_nu() const {
  return _internal_has_gearcur_nu();
}
inline void GearInformation::clear_gearcur_nu() {
  gearcur_nu_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::pb::ap_commonvehsigprovider::gear::Gear GearInformation::_internal_gearcur_nu() const {
  return static_cast< ::pb::ap_commonvehsigprovider::gear::Gear >(gearcur_nu_);
}
inline ::pb::ap_commonvehsigprovider::gear::Gear GearInformation::gearcur_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.gear_information.GearInformation.gearCur_nu)
  return _internal_gearcur_nu();
}
inline void GearInformation::_internal_set_gearcur_nu(::pb::ap_commonvehsigprovider::gear::Gear value) {
  assert(::pb::ap_commonvehsigprovider::gear::Gear_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  gearcur_nu_ = value;
}
inline void GearInformation::set_gearcur_nu(::pb::ap_commonvehsigprovider::gear::Gear value) {
  _internal_set_gearcur_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_commonvehsigprovider.gear_information.GearInformation.gearCur_nu)
}

// -------------------------------------------------------------------

// GearInformation_array_port

// repeated .pb.ap_commonvehsigprovider.gear_information.GearInformation data = 2278;
inline int GearInformation_array_port::_internal_data_size() const {
  return data_.size();
}
inline int GearInformation_array_port::data_size() const {
  return _internal_data_size();
}
inline void GearInformation_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_commonvehsigprovider::gear_information::GearInformation* GearInformation_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_commonvehsigprovider.gear_information.GearInformation_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::gear_information::GearInformation >*
GearInformation_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_commonvehsigprovider.gear_information.GearInformation_array_port.data)
  return &data_;
}
inline const ::pb::ap_commonvehsigprovider::gear_information::GearInformation& GearInformation_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_commonvehsigprovider::gear_information::GearInformation& GearInformation_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_commonvehsigprovider.gear_information.GearInformation_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_commonvehsigprovider::gear_information::GearInformation* GearInformation_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_commonvehsigprovider::gear_information::GearInformation* GearInformation_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_commonvehsigprovider.gear_information.GearInformation_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_commonvehsigprovider::gear_information::GearInformation >&
GearInformation_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_commonvehsigprovider.gear_information.GearInformation_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace gear_information
}  // namespace ap_commonvehsigprovider
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fcommonvehsigprovider_2fgear_5finformation_2eproto