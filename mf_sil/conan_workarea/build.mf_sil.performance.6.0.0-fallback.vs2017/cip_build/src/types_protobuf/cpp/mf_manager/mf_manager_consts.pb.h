// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_manager/mf_manager_consts.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_mf_5fmanager_2fmf_5fmanager_5fconsts_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_mf_5fmanager_2fmf_5fmanager_5fconsts_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_mf_5fmanager_2fmf_5fmanager_5fconsts_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_mf_5fmanager_2fmf_5fmanager_5fconsts_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmanager_2fmf_5fmanager_5fconsts_2eproto;
namespace pb {
namespace mf_manager {
namespace mf_manager_consts {
class MF_MANAGER_Consts;
class MF_MANAGER_ConstsDefaultTypeInternal;
extern MF_MANAGER_ConstsDefaultTypeInternal _MF_MANAGER_Consts_default_instance_;
class MF_MANAGER_Consts_array_port;
class MF_MANAGER_Consts_array_portDefaultTypeInternal;
extern MF_MANAGER_Consts_array_portDefaultTypeInternal _MF_MANAGER_Consts_array_port_default_instance_;
}  // namespace mf_manager_consts
}  // namespace mf_manager
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts* Arena::CreateMaybeMessage<::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts>(Arena*);
template<> ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts_array_port* Arena::CreateMaybeMessage<::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace mf_manager {
namespace mf_manager_consts {

// ===================================================================

class MF_MANAGER_Consts :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts) */ {
 public:
  MF_MANAGER_Consts();
  virtual ~MF_MANAGER_Consts();

  MF_MANAGER_Consts(const MF_MANAGER_Consts& from);
  MF_MANAGER_Consts(MF_MANAGER_Consts&& from) noexcept
    : MF_MANAGER_Consts() {
    *this = ::std::move(from);
  }

  inline MF_MANAGER_Consts& operator=(const MF_MANAGER_Consts& from) {
    CopyFrom(from);
    return *this;
  }
  inline MF_MANAGER_Consts& operator=(MF_MANAGER_Consts&& from) noexcept {
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
  static const MF_MANAGER_Consts& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MF_MANAGER_Consts* internal_default_instance() {
    return reinterpret_cast<const MF_MANAGER_Consts*>(
               &_MF_MANAGER_Consts_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MF_MANAGER_Consts& a, MF_MANAGER_Consts& b) {
    a.Swap(&b);
  }
  inline void Swap(MF_MANAGER_Consts* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MF_MANAGER_Consts* New() const final {
    return CreateMaybeMessage<MF_MANAGER_Consts>(nullptr);
  }

  MF_MANAGER_Consts* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MF_MANAGER_Consts>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MF_MANAGER_Consts& from);
  void MergeFrom(const MF_MANAGER_Consts& from);
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
  void InternalSwap(MF_MANAGER_Consts* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmanager_2fmf_5fmanager_5fconsts_2eproto);
    return ::descriptor_table_mf_5fmanager_2fmf_5fmanager_5fconsts_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kAPMMAXNUMTRAJCTRLPOINTSFieldNumber = 788,
    kAPMMAXNUMDRIVINGRESISTANCEFieldNumber = 1967,
  };
  // optional uint32 AP_M_MAX_NUM_TRAJ_CTRL_POINTS = 788;
  bool has_ap_m_max_num_traj_ctrl_points() const;
  private:
  bool _internal_has_ap_m_max_num_traj_ctrl_points() const;
  public:
  void clear_ap_m_max_num_traj_ctrl_points();
  ::PROTOBUF_NAMESPACE_ID::uint32 ap_m_max_num_traj_ctrl_points() const;
  void set_ap_m_max_num_traj_ctrl_points(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_ap_m_max_num_traj_ctrl_points() const;
  void _internal_set_ap_m_max_num_traj_ctrl_points(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 AP_M_MAX_NUM_DRIVING_RESISTANCE = 1967;
  bool has_ap_m_max_num_driving_resistance() const;
  private:
  bool _internal_has_ap_m_max_num_driving_resistance() const;
  public:
  void clear_ap_m_max_num_driving_resistance();
  ::PROTOBUF_NAMESPACE_ID::uint32 ap_m_max_num_driving_resistance() const;
  void set_ap_m_max_num_driving_resistance(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_ap_m_max_num_driving_resistance() const;
  void _internal_set_ap_m_max_num_driving_resistance(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 ap_m_max_num_traj_ctrl_points_;
  ::PROTOBUF_NAMESPACE_ID::uint32 ap_m_max_num_driving_resistance_;
  friend struct ::TableStruct_mf_5fmanager_2fmf_5fmanager_5fconsts_2eproto;
};
// -------------------------------------------------------------------

class MF_MANAGER_Consts_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts_array_port) */ {
 public:
  MF_MANAGER_Consts_array_port();
  virtual ~MF_MANAGER_Consts_array_port();

  MF_MANAGER_Consts_array_port(const MF_MANAGER_Consts_array_port& from);
  MF_MANAGER_Consts_array_port(MF_MANAGER_Consts_array_port&& from) noexcept
    : MF_MANAGER_Consts_array_port() {
    *this = ::std::move(from);
  }

  inline MF_MANAGER_Consts_array_port& operator=(const MF_MANAGER_Consts_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline MF_MANAGER_Consts_array_port& operator=(MF_MANAGER_Consts_array_port&& from) noexcept {
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
  static const MF_MANAGER_Consts_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MF_MANAGER_Consts_array_port* internal_default_instance() {
    return reinterpret_cast<const MF_MANAGER_Consts_array_port*>(
               &_MF_MANAGER_Consts_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(MF_MANAGER_Consts_array_port& a, MF_MANAGER_Consts_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(MF_MANAGER_Consts_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MF_MANAGER_Consts_array_port* New() const final {
    return CreateMaybeMessage<MF_MANAGER_Consts_array_port>(nullptr);
  }

  MF_MANAGER_Consts_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MF_MANAGER_Consts_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MF_MANAGER_Consts_array_port& from);
  void MergeFrom(const MF_MANAGER_Consts_array_port& from);
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
  void InternalSwap(MF_MANAGER_Consts_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_mf_5fmanager_2fmf_5fmanager_5fconsts_2eproto);
    return ::descriptor_table_mf_5fmanager_2fmf_5fmanager_5fconsts_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 1788,
  };
  // repeated .pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts data = 1788;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts >*
      mutable_data();
  private:
  const ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts& _internal_data(int index) const;
  ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts* _internal_add_data();
  public:
  const ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts& data(int index) const;
  ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts > data_;
  friend struct ::TableStruct_mf_5fmanager_2fmf_5fmanager_5fconsts_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MF_MANAGER_Consts

// optional uint32 AP_M_MAX_NUM_TRAJ_CTRL_POINTS = 788;
inline bool MF_MANAGER_Consts::_internal_has_ap_m_max_num_traj_ctrl_points() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool MF_MANAGER_Consts::has_ap_m_max_num_traj_ctrl_points() const {
  return _internal_has_ap_m_max_num_traj_ctrl_points();
}
inline void MF_MANAGER_Consts::clear_ap_m_max_num_traj_ctrl_points() {
  ap_m_max_num_traj_ctrl_points_ = 0u;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MANAGER_Consts::_internal_ap_m_max_num_traj_ctrl_points() const {
  return ap_m_max_num_traj_ctrl_points_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MANAGER_Consts::ap_m_max_num_traj_ctrl_points() const {
  // @@protoc_insertion_point(field_get:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts.AP_M_MAX_NUM_TRAJ_CTRL_POINTS)
  return _internal_ap_m_max_num_traj_ctrl_points();
}
inline void MF_MANAGER_Consts::_internal_set_ap_m_max_num_traj_ctrl_points(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000001u;
  ap_m_max_num_traj_ctrl_points_ = value;
}
inline void MF_MANAGER_Consts::set_ap_m_max_num_traj_ctrl_points(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_ap_m_max_num_traj_ctrl_points(value);
  // @@protoc_insertion_point(field_set:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts.AP_M_MAX_NUM_TRAJ_CTRL_POINTS)
}

// optional uint32 AP_M_MAX_NUM_DRIVING_RESISTANCE = 1967;
inline bool MF_MANAGER_Consts::_internal_has_ap_m_max_num_driving_resistance() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool MF_MANAGER_Consts::has_ap_m_max_num_driving_resistance() const {
  return _internal_has_ap_m_max_num_driving_resistance();
}
inline void MF_MANAGER_Consts::clear_ap_m_max_num_driving_resistance() {
  ap_m_max_num_driving_resistance_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MANAGER_Consts::_internal_ap_m_max_num_driving_resistance() const {
  return ap_m_max_num_driving_resistance_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 MF_MANAGER_Consts::ap_m_max_num_driving_resistance() const {
  // @@protoc_insertion_point(field_get:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts.AP_M_MAX_NUM_DRIVING_RESISTANCE)
  return _internal_ap_m_max_num_driving_resistance();
}
inline void MF_MANAGER_Consts::_internal_set_ap_m_max_num_driving_resistance(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  ap_m_max_num_driving_resistance_ = value;
}
inline void MF_MANAGER_Consts::set_ap_m_max_num_driving_resistance(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_ap_m_max_num_driving_resistance(value);
  // @@protoc_insertion_point(field_set:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts.AP_M_MAX_NUM_DRIVING_RESISTANCE)
}

// -------------------------------------------------------------------

// MF_MANAGER_Consts_array_port

// repeated .pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts data = 1788;
inline int MF_MANAGER_Consts_array_port::_internal_data_size() const {
  return data_.size();
}
inline int MF_MANAGER_Consts_array_port::data_size() const {
  return _internal_data_size();
}
inline void MF_MANAGER_Consts_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts* MF_MANAGER_Consts_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts >*
MF_MANAGER_Consts_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts_array_port.data)
  return &data_;
}
inline const ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts& MF_MANAGER_Consts_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts& MF_MANAGER_Consts_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts_array_port.data)
  return _internal_data(index);
}
inline ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts* MF_MANAGER_Consts_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts* MF_MANAGER_Consts_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::mf_manager::mf_manager_consts::MF_MANAGER_Consts >&
MF_MANAGER_Consts_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.mf_manager.mf_manager_consts.MF_MANAGER_Consts_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace mf_manager_consts
}  // namespace mf_manager
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_mf_5fmanager_2fmf_5fmanager_5fconsts_2eproto