// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/outer_rear_view_mirror.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fouter_5frear_5fview_5fmirror_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fouter_5frear_5fview_5fmirror_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fouter_5frear_5fview_5fmirror_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ap_5fvehstatesigprovider_2fouter_5frear_5fview_5fmirror_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2fouter_5frear_5fview_5fmirror_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace outer_rear_view_mirror {
class OuterRearViewMirror;
class OuterRearViewMirrorDefaultTypeInternal;
extern OuterRearViewMirrorDefaultTypeInternal _OuterRearViewMirror_default_instance_;
class OuterRearViewMirror_array_port;
class OuterRearViewMirror_array_portDefaultTypeInternal;
extern OuterRearViewMirror_array_portDefaultTypeInternal _OuterRearViewMirror_array_port_default_instance_;
}  // namespace outer_rear_view_mirror
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror>(Arena*);
template<> ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror_array_port* Arena::CreateMaybeMessage<::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace ap_vehstatesigprovider {
namespace outer_rear_view_mirror {

// ===================================================================

class OuterRearViewMirror :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror) */ {
 public:
  OuterRearViewMirror();
  virtual ~OuterRearViewMirror();

  OuterRearViewMirror(const OuterRearViewMirror& from);
  OuterRearViewMirror(OuterRearViewMirror&& from) noexcept
    : OuterRearViewMirror() {
    *this = ::std::move(from);
  }

  inline OuterRearViewMirror& operator=(const OuterRearViewMirror& from) {
    CopyFrom(from);
    return *this;
  }
  inline OuterRearViewMirror& operator=(OuterRearViewMirror&& from) noexcept {
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
  static const OuterRearViewMirror& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const OuterRearViewMirror* internal_default_instance() {
    return reinterpret_cast<const OuterRearViewMirror*>(
               &_OuterRearViewMirror_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(OuterRearViewMirror& a, OuterRearViewMirror& b) {
    a.Swap(&b);
  }
  inline void Swap(OuterRearViewMirror* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline OuterRearViewMirror* New() const final {
    return CreateMaybeMessage<OuterRearViewMirror>(nullptr);
  }

  OuterRearViewMirror* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<OuterRearViewMirror>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const OuterRearViewMirror& from);
  void MergeFrom(const OuterRearViewMirror& from);
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
  void InternalSwap(OuterRearViewMirror* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fouter_5frear_5fview_5fmirror_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fouter_5frear_5fview_5fmirror_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kLeftOuterRearViewMirrorStateNuFieldNumber = 1918,
    kRightOuterRearViewMirrorStateNuFieldNumber = 3610,
  };
  // optional bool leftOuterRearViewMirrorState_nu = 1918;
  bool has_leftouterrearviewmirrorstate_nu() const;
  private:
  bool _internal_has_leftouterrearviewmirrorstate_nu() const;
  public:
  void clear_leftouterrearviewmirrorstate_nu();
  bool leftouterrearviewmirrorstate_nu() const;
  void set_leftouterrearviewmirrorstate_nu(bool value);
  private:
  bool _internal_leftouterrearviewmirrorstate_nu() const;
  void _internal_set_leftouterrearviewmirrorstate_nu(bool value);
  public:

  // optional bool rightOuterRearViewMirrorState_nu = 3610;
  bool has_rightouterrearviewmirrorstate_nu() const;
  private:
  bool _internal_has_rightouterrearviewmirrorstate_nu() const;
  public:
  void clear_rightouterrearviewmirrorstate_nu();
  bool rightouterrearviewmirrorstate_nu() const;
  void set_rightouterrearviewmirrorstate_nu(bool value);
  private:
  bool _internal_rightouterrearviewmirrorstate_nu() const;
  void _internal_set_rightouterrearviewmirrorstate_nu(bool value);
  public:

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  bool leftouterrearviewmirrorstate_nu_;
  bool rightouterrearviewmirrorstate_nu_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fouter_5frear_5fview_5fmirror_2eproto;
};
// -------------------------------------------------------------------

class OuterRearViewMirror_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror_array_port) */ {
 public:
  OuterRearViewMirror_array_port();
  virtual ~OuterRearViewMirror_array_port();

  OuterRearViewMirror_array_port(const OuterRearViewMirror_array_port& from);
  OuterRearViewMirror_array_port(OuterRearViewMirror_array_port&& from) noexcept
    : OuterRearViewMirror_array_port() {
    *this = ::std::move(from);
  }

  inline OuterRearViewMirror_array_port& operator=(const OuterRearViewMirror_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline OuterRearViewMirror_array_port& operator=(OuterRearViewMirror_array_port&& from) noexcept {
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
  static const OuterRearViewMirror_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const OuterRearViewMirror_array_port* internal_default_instance() {
    return reinterpret_cast<const OuterRearViewMirror_array_port*>(
               &_OuterRearViewMirror_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(OuterRearViewMirror_array_port& a, OuterRearViewMirror_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(OuterRearViewMirror_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline OuterRearViewMirror_array_port* New() const final {
    return CreateMaybeMessage<OuterRearViewMirror_array_port>(nullptr);
  }

  OuterRearViewMirror_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<OuterRearViewMirror_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const OuterRearViewMirror_array_port& from);
  void MergeFrom(const OuterRearViewMirror_array_port& from);
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
  void InternalSwap(OuterRearViewMirror_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ap_5fvehstatesigprovider_2fouter_5frear_5fview_5fmirror_2eproto);
    return ::descriptor_table_ap_5fvehstatesigprovider_2fouter_5frear_5fview_5fmirror_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 377,
  };
  // repeated .pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror data = 377;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror >*
      mutable_data();
  private:
  const ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror& _internal_data(int index) const;
  ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror* _internal_add_data();
  public:
  const ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror& data(int index) const;
  ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror > data_;
  friend struct ::TableStruct_ap_5fvehstatesigprovider_2fouter_5frear_5fview_5fmirror_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// OuterRearViewMirror

// optional bool leftOuterRearViewMirrorState_nu = 1918;
inline bool OuterRearViewMirror::_internal_has_leftouterrearviewmirrorstate_nu() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool OuterRearViewMirror::has_leftouterrearviewmirrorstate_nu() const {
  return _internal_has_leftouterrearviewmirrorstate_nu();
}
inline void OuterRearViewMirror::clear_leftouterrearviewmirrorstate_nu() {
  leftouterrearviewmirrorstate_nu_ = false;
  _has_bits_[0] &= ~0x00000001u;
}
inline bool OuterRearViewMirror::_internal_leftouterrearviewmirrorstate_nu() const {
  return leftouterrearviewmirrorstate_nu_;
}
inline bool OuterRearViewMirror::leftouterrearviewmirrorstate_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror.leftOuterRearViewMirrorState_nu)
  return _internal_leftouterrearviewmirrorstate_nu();
}
inline void OuterRearViewMirror::_internal_set_leftouterrearviewmirrorstate_nu(bool value) {
  _has_bits_[0] |= 0x00000001u;
  leftouterrearviewmirrorstate_nu_ = value;
}
inline void OuterRearViewMirror::set_leftouterrearviewmirrorstate_nu(bool value) {
  _internal_set_leftouterrearviewmirrorstate_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror.leftOuterRearViewMirrorState_nu)
}

// optional bool rightOuterRearViewMirrorState_nu = 3610;
inline bool OuterRearViewMirror::_internal_has_rightouterrearviewmirrorstate_nu() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool OuterRearViewMirror::has_rightouterrearviewmirrorstate_nu() const {
  return _internal_has_rightouterrearviewmirrorstate_nu();
}
inline void OuterRearViewMirror::clear_rightouterrearviewmirrorstate_nu() {
  rightouterrearviewmirrorstate_nu_ = false;
  _has_bits_[0] &= ~0x00000002u;
}
inline bool OuterRearViewMirror::_internal_rightouterrearviewmirrorstate_nu() const {
  return rightouterrearviewmirrorstate_nu_;
}
inline bool OuterRearViewMirror::rightouterrearviewmirrorstate_nu() const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror.rightOuterRearViewMirrorState_nu)
  return _internal_rightouterrearviewmirrorstate_nu();
}
inline void OuterRearViewMirror::_internal_set_rightouterrearviewmirrorstate_nu(bool value) {
  _has_bits_[0] |= 0x00000002u;
  rightouterrearviewmirrorstate_nu_ = value;
}
inline void OuterRearViewMirror::set_rightouterrearviewmirrorstate_nu(bool value) {
  _internal_set_rightouterrearviewmirrorstate_nu(value);
  // @@protoc_insertion_point(field_set:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror.rightOuterRearViewMirrorState_nu)
}

// -------------------------------------------------------------------

// OuterRearViewMirror_array_port

// repeated .pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror data = 377;
inline int OuterRearViewMirror_array_port::_internal_data_size() const {
  return data_.size();
}
inline int OuterRearViewMirror_array_port::data_size() const {
  return _internal_data_size();
}
inline void OuterRearViewMirror_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror* OuterRearViewMirror_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror >*
OuterRearViewMirror_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror_array_port.data)
  return &data_;
}
inline const ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror& OuterRearViewMirror_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror& OuterRearViewMirror_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror_array_port.data)
  return _internal_data(index);
}
inline ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror* OuterRearViewMirror_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror* OuterRearViewMirror_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::ap_vehstatesigprovider::outer_rear_view_mirror::OuterRearViewMirror >&
OuterRearViewMirror_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.ap_vehstatesigprovider.outer_rear_view_mirror.OuterRearViewMirror_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace outer_rear_view_mirror
}  // namespace ap_vehstatesigprovider
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ap_5fvehstatesigprovider_2fouter_5frear_5fview_5fmirror_2eproto