// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_hmih/pdcsectors.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fpdcsectors_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fpdcsectors_2eproto

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
#include "mf_hmih/pdcsector_info.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_mf_5fhmih_2fpdcsectors_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fhmih_2fpdcsectors_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fhmih_2fpdcsectors_2eproto;
namespace pb {
namespace mf_hmih {
namespace pdcsectors {
class PDCSectors;
class PDCSectorsDefaultTypeInternal;
extern PDCSectorsDefaultTypeInternal _PDCSectors_default_instance_;
class PDCSectors_array_port;
class PDCSectors_array_portDefaultTypeInternal;
extern PDCSectors_array_portDefaultTypeInternal _PDCSectors_array_port_default_instance_;
}  // namespace pdcsectors
}  // namespace mf_hmih
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_hmih::pdcsectors::PDCSectors* Arena::CreateMaybeMessage<::pb::mf_hmih::pdcsectors::PDCSectors>(Arena*);
template<> ::pb::mf_hmih::pdcsectors::PDCSectors_array_port* Arena::CreateMaybeMessage<::pb::mf_hmih::pdcsectors::PDCSectors_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_hmih {
namespace pdcsectors {

// ===================================================================

class PDCSectors :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.pdcsectors.PDCSectors) */ {
 public:
  PDCSectors();
  virtual ~PDCSectors();

  PDCSectors(const PDCSectors& from);
  PDCSectors(PDCSectors&& from) noexcept
    : PDCSectors() {
    *this = ::std::move(from);
  }

  inline PDCSectors& operator=(const PDCSectors& from) {
    CopyFrom(from);
    return *this;
  }
  inline PDCSectors& operator=(PDCSectors&& from) noexcept {
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
  static const PDCSectors& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PDCSectors* internal_default_instance() {
    return reinterpret_cast<const PDCSectors*>(
               &_PDCSectors_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PDCSectors& a, PDCSectors& b) {
    a.Swap(&b);
  }
  inline void Swap(PDCSectors* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PDCSectors* New() const final {
    return CreateMaybeMessage<PDCSectors>(nullptr);
  }

  PDCSectors* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PDCSectors>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PDCSectors& from);
  void MergeFrom(const PDCSectors& from);
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
  void InternalSwap(PDCSectors* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.pdcsectors.PDCSectors";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fpdcsectors_2eproto);
    return ::descriptor_table_mf_5fhmih_2fpdcsectors_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kLeftFieldNumber = 114,
    kPDCPSECTORINNERCOORDSXMFieldNumber = 178,
    kRearFieldNumber = 222,
    kPDCPSECTOROUTERCOORDSXMFieldNumber = 445,
    kRightFieldNumber = 776,
    kPDCPSECTOROUTERCOORDSYMFieldNumber = 1677,
    kPDCPSECTORINNERCOORDSYMFieldNumber = 1922,
    kFrontFieldNumber = 2283,
  };
  // repeated .pb.mf_hmih.pdcsector_info.PDCSectorInfo left = 114;
  int left_size() const;
  private:
  int _internal_left_size() const;
  public:
  void clear_left();
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* mutable_left(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >*
      mutable_left();
  private:
  const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& _internal_left(int index) const;
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* _internal_add_left();
  public:
  const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& left(int index) const;
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* add_left();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >&
      left() const;

  // repeated float PDC_P_SECTOR_INNER_COORDS_X_M = 178;
  int pdc_p_sector_inner_coords_x_m_size() const;
  private:
  int _internal_pdc_p_sector_inner_coords_x_m_size() const;
  public:
  void clear_pdc_p_sector_inner_coords_x_m();
  private:
  float _internal_pdc_p_sector_inner_coords_x_m(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_pdc_p_sector_inner_coords_x_m() const;
  void _internal_add_pdc_p_sector_inner_coords_x_m(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_pdc_p_sector_inner_coords_x_m();
  public:
  float pdc_p_sector_inner_coords_x_m(int index) const;
  void set_pdc_p_sector_inner_coords_x_m(int index, float value);
  void add_pdc_p_sector_inner_coords_x_m(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      pdc_p_sector_inner_coords_x_m() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_pdc_p_sector_inner_coords_x_m();

  // repeated .pb.mf_hmih.pdcsector_info.PDCSectorInfo rear = 222;
  int rear_size() const;
  private:
  int _internal_rear_size() const;
  public:
  void clear_rear();
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* mutable_rear(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >*
      mutable_rear();
  private:
  const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& _internal_rear(int index) const;
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* _internal_add_rear();
  public:
  const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& rear(int index) const;
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* add_rear();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >&
      rear() const;

  // repeated float PDC_P_SECTOR_OUTER_COORDS_X_M = 445;
  int pdc_p_sector_outer_coords_x_m_size() const;
  private:
  int _internal_pdc_p_sector_outer_coords_x_m_size() const;
  public:
  void clear_pdc_p_sector_outer_coords_x_m();
  private:
  float _internal_pdc_p_sector_outer_coords_x_m(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_pdc_p_sector_outer_coords_x_m() const;
  void _internal_add_pdc_p_sector_outer_coords_x_m(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_pdc_p_sector_outer_coords_x_m();
  public:
  float pdc_p_sector_outer_coords_x_m(int index) const;
  void set_pdc_p_sector_outer_coords_x_m(int index, float value);
  void add_pdc_p_sector_outer_coords_x_m(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      pdc_p_sector_outer_coords_x_m() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_pdc_p_sector_outer_coords_x_m();

  // repeated .pb.mf_hmih.pdcsector_info.PDCSectorInfo right = 776;
  int right_size() const;
  private:
  int _internal_right_size() const;
  public:
  void clear_right();
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* mutable_right(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >*
      mutable_right();
  private:
  const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& _internal_right(int index) const;
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* _internal_add_right();
  public:
  const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& right(int index) const;
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* add_right();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >&
      right() const;

  // repeated float PDC_P_SECTOR_OUTER_COORDS_Y_M = 1677;
  int pdc_p_sector_outer_coords_y_m_size() const;
  private:
  int _internal_pdc_p_sector_outer_coords_y_m_size() const;
  public:
  void clear_pdc_p_sector_outer_coords_y_m();
  private:
  float _internal_pdc_p_sector_outer_coords_y_m(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_pdc_p_sector_outer_coords_y_m() const;
  void _internal_add_pdc_p_sector_outer_coords_y_m(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_pdc_p_sector_outer_coords_y_m();
  public:
  float pdc_p_sector_outer_coords_y_m(int index) const;
  void set_pdc_p_sector_outer_coords_y_m(int index, float value);
  void add_pdc_p_sector_outer_coords_y_m(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      pdc_p_sector_outer_coords_y_m() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_pdc_p_sector_outer_coords_y_m();

  // repeated float PDC_P_SECTOR_INNER_COORDS_Y_M = 1922;
  int pdc_p_sector_inner_coords_y_m_size() const;
  private:
  int _internal_pdc_p_sector_inner_coords_y_m_size() const;
  public:
  void clear_pdc_p_sector_inner_coords_y_m();
  private:
  float _internal_pdc_p_sector_inner_coords_y_m(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      _internal_pdc_p_sector_inner_coords_y_m() const;
  void _internal_add_pdc_p_sector_inner_coords_y_m(float value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      _internal_mutable_pdc_p_sector_inner_coords_y_m();
  public:
  float pdc_p_sector_inner_coords_y_m(int index) const;
  void set_pdc_p_sector_inner_coords_y_m(int index, float value);
  void add_pdc_p_sector_inner_coords_y_m(float value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
      pdc_p_sector_inner_coords_y_m() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
      mutable_pdc_p_sector_inner_coords_y_m();

  // repeated .pb.mf_hmih.pdcsector_info.PDCSectorInfo front = 2283;
  int front_size() const;
  private:
  int _internal_front_size() const;
  public:
  void clear_front();
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* mutable_front(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >*
      mutable_front();
  private:
  const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& _internal_front(int index) const;
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* _internal_add_front();
  public:
  const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& front(int index) const;
  ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* add_front();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >&
      front() const;

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.pdcsectors.PDCSectors)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo > left_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > pdc_p_sector_inner_coords_x_m_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo > rear_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > pdc_p_sector_outer_coords_x_m_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo > right_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > pdc_p_sector_outer_coords_y_m_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< float > pdc_p_sector_inner_coords_y_m_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo > front_;
  friend struct ::TableStruct_mf_5fhmih_2fpdcsectors_2eproto;
};
// -------------------------------------------------------------------

class PDCSectors_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_hmih.pdcsectors.PDCSectors_array_port) */ {
 public:
  PDCSectors_array_port();
  virtual ~PDCSectors_array_port();

  PDCSectors_array_port(const PDCSectors_array_port& from);
  PDCSectors_array_port(PDCSectors_array_port&& from) noexcept
    : PDCSectors_array_port() {
    *this = ::std::move(from);
  }

  inline PDCSectors_array_port& operator=(const PDCSectors_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline PDCSectors_array_port& operator=(PDCSectors_array_port&& from) noexcept {
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
  static const PDCSectors_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PDCSectors_array_port* internal_default_instance() {
    return reinterpret_cast<const PDCSectors_array_port*>(
               &_PDCSectors_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(PDCSectors_array_port& a, PDCSectors_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(PDCSectors_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PDCSectors_array_port* New() const final {
    return CreateMaybeMessage<PDCSectors_array_port>(nullptr);
  }

  PDCSectors_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PDCSectors_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PDCSectors_array_port& from);
  void MergeFrom(const PDCSectors_array_port& from);
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
  void InternalSwap(PDCSectors_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_hmih.pdcsectors.PDCSectors_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fhmih_2fpdcsectors_2eproto);
    return ::descriptor_table_mf_5fhmih_2fpdcsectors_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 3105,
  };
  // repeated .pb.mf_hmih.pdcsectors.PDCSectors data = 3105;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_hmih::pdcsectors::PDCSectors* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsectors::PDCSectors >*
      mutable_data();
  private:
  const ::pb::mf_hmih::pdcsectors::PDCSectors& _internal_data(int index) const;
  ::pb::mf_hmih::pdcsectors::PDCSectors* _internal_add_data();
  public:
  const ::pb::mf_hmih::pdcsectors::PDCSectors& data(int index) const;
  ::pb::mf_hmih::pdcsectors::PDCSectors* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsectors::PDCSectors >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_hmih.pdcsectors.PDCSectors_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsectors::PDCSectors > data_;
  friend struct ::TableStruct_mf_5fhmih_2fpdcsectors_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PDCSectors

// repeated .pb.mf_hmih.pdcsector_info.PDCSectorInfo left = 114;
inline int PDCSectors::_internal_left_size() const {
  return left_.size();
}
inline int PDCSectors::left_size() const {
  return _internal_left_size();
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectors::mutable_left(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.pdcsectors.PDCSectors.left)
  return left_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >*
PDCSectors::mutable_left() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.pdcsectors.PDCSectors.left)
  return &left_;
}
inline const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& PDCSectors::_internal_left(int index) const {
  return left_.Get(index);
}
inline const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& PDCSectors::left(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsectors.PDCSectors.left)
  return _internal_left(index);
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectors::_internal_add_left() {
  return left_.Add();
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectors::add_left() {
  // @@protoc_insertion_point(field_add:pb.mf_hmih.pdcsectors.PDCSectors.left)
  return _internal_add_left();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >&
PDCSectors::left() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.pdcsectors.PDCSectors.left)
  return left_;
}

// repeated .pb.mf_hmih.pdcsector_info.PDCSectorInfo right = 776;
inline int PDCSectors::_internal_right_size() const {
  return right_.size();
}
inline int PDCSectors::right_size() const {
  return _internal_right_size();
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectors::mutable_right(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.pdcsectors.PDCSectors.right)
  return right_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >*
PDCSectors::mutable_right() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.pdcsectors.PDCSectors.right)
  return &right_;
}
inline const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& PDCSectors::_internal_right(int index) const {
  return right_.Get(index);
}
inline const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& PDCSectors::right(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsectors.PDCSectors.right)
  return _internal_right(index);
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectors::_internal_add_right() {
  return right_.Add();
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectors::add_right() {
  // @@protoc_insertion_point(field_add:pb.mf_hmih.pdcsectors.PDCSectors.right)
  return _internal_add_right();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >&
PDCSectors::right() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.pdcsectors.PDCSectors.right)
  return right_;
}

// repeated .pb.mf_hmih.pdcsector_info.PDCSectorInfo front = 2283;
inline int PDCSectors::_internal_front_size() const {
  return front_.size();
}
inline int PDCSectors::front_size() const {
  return _internal_front_size();
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectors::mutable_front(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.pdcsectors.PDCSectors.front)
  return front_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >*
PDCSectors::mutable_front() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.pdcsectors.PDCSectors.front)
  return &front_;
}
inline const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& PDCSectors::_internal_front(int index) const {
  return front_.Get(index);
}
inline const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& PDCSectors::front(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsectors.PDCSectors.front)
  return _internal_front(index);
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectors::_internal_add_front() {
  return front_.Add();
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectors::add_front() {
  // @@protoc_insertion_point(field_add:pb.mf_hmih.pdcsectors.PDCSectors.front)
  return _internal_add_front();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >&
PDCSectors::front() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.pdcsectors.PDCSectors.front)
  return front_;
}

// repeated .pb.mf_hmih.pdcsector_info.PDCSectorInfo rear = 222;
inline int PDCSectors::_internal_rear_size() const {
  return rear_.size();
}
inline int PDCSectors::rear_size() const {
  return _internal_rear_size();
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectors::mutable_rear(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.pdcsectors.PDCSectors.rear)
  return rear_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >*
PDCSectors::mutable_rear() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.pdcsectors.PDCSectors.rear)
  return &rear_;
}
inline const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& PDCSectors::_internal_rear(int index) const {
  return rear_.Get(index);
}
inline const ::pb::mf_hmih::pdcsector_info::PDCSectorInfo& PDCSectors::rear(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsectors.PDCSectors.rear)
  return _internal_rear(index);
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectors::_internal_add_rear() {
  return rear_.Add();
}
inline ::pb::mf_hmih::pdcsector_info::PDCSectorInfo* PDCSectors::add_rear() {
  // @@protoc_insertion_point(field_add:pb.mf_hmih.pdcsectors.PDCSectors.rear)
  return _internal_add_rear();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsector_info::PDCSectorInfo >&
PDCSectors::rear() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.pdcsectors.PDCSectors.rear)
  return rear_;
}

// repeated float PDC_P_SECTOR_INNER_COORDS_X_M = 178;
inline int PDCSectors::_internal_pdc_p_sector_inner_coords_x_m_size() const {
  return pdc_p_sector_inner_coords_x_m_.size();
}
inline int PDCSectors::pdc_p_sector_inner_coords_x_m_size() const {
  return _internal_pdc_p_sector_inner_coords_x_m_size();
}
inline void PDCSectors::clear_pdc_p_sector_inner_coords_x_m() {
  pdc_p_sector_inner_coords_x_m_.Clear();
}
inline float PDCSectors::_internal_pdc_p_sector_inner_coords_x_m(int index) const {
  return pdc_p_sector_inner_coords_x_m_.Get(index);
}
inline float PDCSectors::pdc_p_sector_inner_coords_x_m(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_INNER_COORDS_X_M)
  return _internal_pdc_p_sector_inner_coords_x_m(index);
}
inline void PDCSectors::set_pdc_p_sector_inner_coords_x_m(int index, float value) {
  pdc_p_sector_inner_coords_x_m_.Set(index, value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_INNER_COORDS_X_M)
}
inline void PDCSectors::_internal_add_pdc_p_sector_inner_coords_x_m(float value) {
  pdc_p_sector_inner_coords_x_m_.Add(value);
}
inline void PDCSectors::add_pdc_p_sector_inner_coords_x_m(float value) {
  _internal_add_pdc_p_sector_inner_coords_x_m(value);
  // @@protoc_insertion_point(field_add:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_INNER_COORDS_X_M)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
PDCSectors::_internal_pdc_p_sector_inner_coords_x_m() const {
  return pdc_p_sector_inner_coords_x_m_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
PDCSectors::pdc_p_sector_inner_coords_x_m() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_INNER_COORDS_X_M)
  return _internal_pdc_p_sector_inner_coords_x_m();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
PDCSectors::_internal_mutable_pdc_p_sector_inner_coords_x_m() {
  return &pdc_p_sector_inner_coords_x_m_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
PDCSectors::mutable_pdc_p_sector_inner_coords_x_m() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_INNER_COORDS_X_M)
  return _internal_mutable_pdc_p_sector_inner_coords_x_m();
}

// repeated float PDC_P_SECTOR_INNER_COORDS_Y_M = 1922;
inline int PDCSectors::_internal_pdc_p_sector_inner_coords_y_m_size() const {
  return pdc_p_sector_inner_coords_y_m_.size();
}
inline int PDCSectors::pdc_p_sector_inner_coords_y_m_size() const {
  return _internal_pdc_p_sector_inner_coords_y_m_size();
}
inline void PDCSectors::clear_pdc_p_sector_inner_coords_y_m() {
  pdc_p_sector_inner_coords_y_m_.Clear();
}
inline float PDCSectors::_internal_pdc_p_sector_inner_coords_y_m(int index) const {
  return pdc_p_sector_inner_coords_y_m_.Get(index);
}
inline float PDCSectors::pdc_p_sector_inner_coords_y_m(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_INNER_COORDS_Y_M)
  return _internal_pdc_p_sector_inner_coords_y_m(index);
}
inline void PDCSectors::set_pdc_p_sector_inner_coords_y_m(int index, float value) {
  pdc_p_sector_inner_coords_y_m_.Set(index, value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_INNER_COORDS_Y_M)
}
inline void PDCSectors::_internal_add_pdc_p_sector_inner_coords_y_m(float value) {
  pdc_p_sector_inner_coords_y_m_.Add(value);
}
inline void PDCSectors::add_pdc_p_sector_inner_coords_y_m(float value) {
  _internal_add_pdc_p_sector_inner_coords_y_m(value);
  // @@protoc_insertion_point(field_add:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_INNER_COORDS_Y_M)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
PDCSectors::_internal_pdc_p_sector_inner_coords_y_m() const {
  return pdc_p_sector_inner_coords_y_m_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
PDCSectors::pdc_p_sector_inner_coords_y_m() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_INNER_COORDS_Y_M)
  return _internal_pdc_p_sector_inner_coords_y_m();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
PDCSectors::_internal_mutable_pdc_p_sector_inner_coords_y_m() {
  return &pdc_p_sector_inner_coords_y_m_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
PDCSectors::mutable_pdc_p_sector_inner_coords_y_m() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_INNER_COORDS_Y_M)
  return _internal_mutable_pdc_p_sector_inner_coords_y_m();
}

// repeated float PDC_P_SECTOR_OUTER_COORDS_X_M = 445;
inline int PDCSectors::_internal_pdc_p_sector_outer_coords_x_m_size() const {
  return pdc_p_sector_outer_coords_x_m_.size();
}
inline int PDCSectors::pdc_p_sector_outer_coords_x_m_size() const {
  return _internal_pdc_p_sector_outer_coords_x_m_size();
}
inline void PDCSectors::clear_pdc_p_sector_outer_coords_x_m() {
  pdc_p_sector_outer_coords_x_m_.Clear();
}
inline float PDCSectors::_internal_pdc_p_sector_outer_coords_x_m(int index) const {
  return pdc_p_sector_outer_coords_x_m_.Get(index);
}
inline float PDCSectors::pdc_p_sector_outer_coords_x_m(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_OUTER_COORDS_X_M)
  return _internal_pdc_p_sector_outer_coords_x_m(index);
}
inline void PDCSectors::set_pdc_p_sector_outer_coords_x_m(int index, float value) {
  pdc_p_sector_outer_coords_x_m_.Set(index, value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_OUTER_COORDS_X_M)
}
inline void PDCSectors::_internal_add_pdc_p_sector_outer_coords_x_m(float value) {
  pdc_p_sector_outer_coords_x_m_.Add(value);
}
inline void PDCSectors::add_pdc_p_sector_outer_coords_x_m(float value) {
  _internal_add_pdc_p_sector_outer_coords_x_m(value);
  // @@protoc_insertion_point(field_add:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_OUTER_COORDS_X_M)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
PDCSectors::_internal_pdc_p_sector_outer_coords_x_m() const {
  return pdc_p_sector_outer_coords_x_m_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
PDCSectors::pdc_p_sector_outer_coords_x_m() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_OUTER_COORDS_X_M)
  return _internal_pdc_p_sector_outer_coords_x_m();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
PDCSectors::_internal_mutable_pdc_p_sector_outer_coords_x_m() {
  return &pdc_p_sector_outer_coords_x_m_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
PDCSectors::mutable_pdc_p_sector_outer_coords_x_m() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_OUTER_COORDS_X_M)
  return _internal_mutable_pdc_p_sector_outer_coords_x_m();
}

// repeated float PDC_P_SECTOR_OUTER_COORDS_Y_M = 1677;
inline int PDCSectors::_internal_pdc_p_sector_outer_coords_y_m_size() const {
  return pdc_p_sector_outer_coords_y_m_.size();
}
inline int PDCSectors::pdc_p_sector_outer_coords_y_m_size() const {
  return _internal_pdc_p_sector_outer_coords_y_m_size();
}
inline void PDCSectors::clear_pdc_p_sector_outer_coords_y_m() {
  pdc_p_sector_outer_coords_y_m_.Clear();
}
inline float PDCSectors::_internal_pdc_p_sector_outer_coords_y_m(int index) const {
  return pdc_p_sector_outer_coords_y_m_.Get(index);
}
inline float PDCSectors::pdc_p_sector_outer_coords_y_m(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_OUTER_COORDS_Y_M)
  return _internal_pdc_p_sector_outer_coords_y_m(index);
}
inline void PDCSectors::set_pdc_p_sector_outer_coords_y_m(int index, float value) {
  pdc_p_sector_outer_coords_y_m_.Set(index, value);
  // @@protoc_insertion_point(field_set:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_OUTER_COORDS_Y_M)
}
inline void PDCSectors::_internal_add_pdc_p_sector_outer_coords_y_m(float value) {
  pdc_p_sector_outer_coords_y_m_.Add(value);
}
inline void PDCSectors::add_pdc_p_sector_outer_coords_y_m(float value) {
  _internal_add_pdc_p_sector_outer_coords_y_m(value);
  // @@protoc_insertion_point(field_add:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_OUTER_COORDS_Y_M)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
PDCSectors::_internal_pdc_p_sector_outer_coords_y_m() const {
  return pdc_p_sector_outer_coords_y_m_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >&
PDCSectors::pdc_p_sector_outer_coords_y_m() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_OUTER_COORDS_Y_M)
  return _internal_pdc_p_sector_outer_coords_y_m();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
PDCSectors::_internal_mutable_pdc_p_sector_outer_coords_y_m() {
  return &pdc_p_sector_outer_coords_y_m_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< float >*
PDCSectors::mutable_pdc_p_sector_outer_coords_y_m() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.pdcsectors.PDCSectors.PDC_P_SECTOR_OUTER_COORDS_Y_M)
  return _internal_mutable_pdc_p_sector_outer_coords_y_m();
}

// -------------------------------------------------------------------

// PDCSectors_array_port

// repeated .pb.mf_hmih.pdcsectors.PDCSectors data = 3105;
inline int PDCSectors_array_port::_internal_data_size() const {
  return data_.size();
}
inline int PDCSectors_array_port::data_size() const {
  return _internal_data_size();
}
inline void PDCSectors_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_hmih::pdcsectors::PDCSectors* PDCSectors_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_hmih.pdcsectors.PDCSectors_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsectors::PDCSectors >*
PDCSectors_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_hmih.pdcsectors.PDCSectors_array_port.data)
  return &data_;
}
inline const ::pb::mf_hmih::pdcsectors::PDCSectors& PDCSectors_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_hmih::pdcsectors::PDCSectors& PDCSectors_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_hmih.pdcsectors.PDCSectors_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_hmih::pdcsectors::PDCSectors* PDCSectors_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_hmih::pdcsectors::PDCSectors* PDCSectors_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_hmih.pdcsectors.PDCSectors_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_hmih::pdcsectors::PDCSectors >&
PDCSectors_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_hmih.pdcsectors.PDCSectors_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace pdcsectors
}  // namespace mf_hmih
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fhmih_2fpdcsectors_2eproto
