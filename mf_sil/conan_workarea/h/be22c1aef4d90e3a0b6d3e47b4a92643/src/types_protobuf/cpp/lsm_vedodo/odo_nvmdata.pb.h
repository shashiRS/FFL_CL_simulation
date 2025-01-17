// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: lsm_vedodo/odo_nvmdata.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_lsm_5fvedodo_2fodo_5fnvmdata_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_lsm_5fvedodo_2fodo_5fnvmdata_2eproto

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
#include "eco/signal_header.pb.h"
#include "lsm_vedodo/odo_nv_st_whl_ang_cal_t.pb.h"
#include "lsm_vedodo/yw_rate_t.pb.h"
#include "lsm_vedodo/lat_acc_t.pb.h"
#include "lsm_vedodo/odo_nvm_state_t.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_lsm_5fvedodo_2fodo_5fnvmdata_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_lsm_5fvedodo_2fodo_5fnvmdata_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_lsm_5fvedodo_2fodo_5fnvmdata_2eproto;
namespace pb {
namespace lsm_vedodo {
namespace odo_nvmdata {
class OdoNVMData;
class OdoNVMDataDefaultTypeInternal;
extern OdoNVMDataDefaultTypeInternal _OdoNVMData_default_instance_;
class OdoNVMData_array_port;
class OdoNVMData_array_portDefaultTypeInternal;
extern OdoNVMData_array_portDefaultTypeInternal _OdoNVMData_array_port_default_instance_;
}  // namespace odo_nvmdata
}  // namespace lsm_vedodo
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData* Arena::CreateMaybeMessage<::pb::lsm_vedodo::odo_nvmdata::OdoNVMData>(Arena*);
template<> ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData_array_port* Arena::CreateMaybeMessage<::pb::lsm_vedodo::odo_nvmdata::OdoNVMData_array_port>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace pb {
namespace lsm_vedodo {
namespace odo_nvmdata {

// ===================================================================

class OdoNVMData :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.lsm_vedodo.odo_nvmdata.OdoNVMData) */ {
 public:
  OdoNVMData();
  virtual ~OdoNVMData();

  OdoNVMData(const OdoNVMData& from);
  OdoNVMData(OdoNVMData&& from) noexcept
    : OdoNVMData() {
    *this = ::std::move(from);
  }

  inline OdoNVMData& operator=(const OdoNVMData& from) {
    CopyFrom(from);
    return *this;
  }
  inline OdoNVMData& operator=(OdoNVMData&& from) noexcept {
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
  static const OdoNVMData& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const OdoNVMData* internal_default_instance() {
    return reinterpret_cast<const OdoNVMData*>(
               &_OdoNVMData_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(OdoNVMData& a, OdoNVMData& b) {
    a.Swap(&b);
  }
  inline void Swap(OdoNVMData* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline OdoNVMData* New() const final {
    return CreateMaybeMessage<OdoNVMData>(nullptr);
  }

  OdoNVMData* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<OdoNVMData>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const OdoNVMData& from);
  void MergeFrom(const OdoNVMData& from);
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
  void InternalSwap(OdoNVMData* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.lsm_vedodo.odo_nvmdata.OdoNVMData";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_lsm_5fvedodo_2fodo_5fnvmdata_2eproto);
    return ::descriptor_table_lsm_5fvedodo_2fodo_5fnvmdata_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSSigHeaderFieldNumber = 1033,
    kStateFieldNumber = 1214,
    kYwRateFieldNumber = 1269,
    kLatAccFieldNumber = 2459,
    kStWhlAngFieldNumber = 3011,
    kUiVersionNumberFieldNumber = 2124,
  };
  // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
  bool has_ssigheader() const;
  private:
  bool _internal_has_ssigheader() const;
  public:
  void clear_ssigheader();
  const ::pb::eco::signal_header::SignalHeader& ssigheader() const;
  ::pb::eco::signal_header::SignalHeader* release_ssigheader();
  ::pb::eco::signal_header::SignalHeader* mutable_ssigheader();
  void set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader);
  private:
  const ::pb::eco::signal_header::SignalHeader& _internal_ssigheader() const;
  ::pb::eco::signal_header::SignalHeader* _internal_mutable_ssigheader();
  public:

  // optional .pb.lsm_vedodo.odo_nvm_state_t.OdoNvmState_t State = 1214;
  bool has_state() const;
  private:
  bool _internal_has_state() const;
  public:
  void clear_state();
  const ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t& state() const;
  ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t* release_state();
  ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t* mutable_state();
  void set_allocated_state(::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t* state);
  private:
  const ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t& _internal_state() const;
  ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t* _internal_mutable_state();
  public:

  // optional .pb.lsm_vedodo.yw_rate_t.YwRate_t YwRate = 1269;
  bool has_ywrate() const;
  private:
  bool _internal_has_ywrate() const;
  public:
  void clear_ywrate();
  const ::pb::lsm_vedodo::yw_rate_t::YwRate_t& ywrate() const;
  ::pb::lsm_vedodo::yw_rate_t::YwRate_t* release_ywrate();
  ::pb::lsm_vedodo::yw_rate_t::YwRate_t* mutable_ywrate();
  void set_allocated_ywrate(::pb::lsm_vedodo::yw_rate_t::YwRate_t* ywrate);
  private:
  const ::pb::lsm_vedodo::yw_rate_t::YwRate_t& _internal_ywrate() const;
  ::pb::lsm_vedodo::yw_rate_t::YwRate_t* _internal_mutable_ywrate();
  public:

  // optional .pb.lsm_vedodo.lat_acc_t.LatAcc_t LatAcc = 2459;
  bool has_latacc() const;
  private:
  bool _internal_has_latacc() const;
  public:
  void clear_latacc();
  const ::pb::lsm_vedodo::lat_acc_t::LatAcc_t& latacc() const;
  ::pb::lsm_vedodo::lat_acc_t::LatAcc_t* release_latacc();
  ::pb::lsm_vedodo::lat_acc_t::LatAcc_t* mutable_latacc();
  void set_allocated_latacc(::pb::lsm_vedodo::lat_acc_t::LatAcc_t* latacc);
  private:
  const ::pb::lsm_vedodo::lat_acc_t::LatAcc_t& _internal_latacc() const;
  ::pb::lsm_vedodo::lat_acc_t::LatAcc_t* _internal_mutable_latacc();
  public:

  // optional .pb.lsm_vedodo.odo_nv_st_whl_ang_cal_t.OdoNvStWhlAngCal_t StWhlAng = 3011;
  bool has_stwhlang() const;
  private:
  bool _internal_has_stwhlang() const;
  public:
  void clear_stwhlang();
  const ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t& stwhlang() const;
  ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t* release_stwhlang();
  ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t* mutable_stwhlang();
  void set_allocated_stwhlang(::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t* stwhlang);
  private:
  const ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t& _internal_stwhlang() const;
  ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t* _internal_mutable_stwhlang();
  public:

  // optional uint32 uiVersionNumber = 2124;
  bool has_uiversionnumber() const;
  private:
  bool _internal_has_uiversionnumber() const;
  public:
  void clear_uiversionnumber();
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber() const;
  void set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_uiversionnumber() const;
  void _internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:pb.lsm_vedodo.odo_nvmdata.OdoNVMData)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::pb::eco::signal_header::SignalHeader* ssigheader_;
  ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t* state_;
  ::pb::lsm_vedodo::yw_rate_t::YwRate_t* ywrate_;
  ::pb::lsm_vedodo::lat_acc_t::LatAcc_t* latacc_;
  ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t* stwhlang_;
  ::PROTOBUF_NAMESPACE_ID::uint32 uiversionnumber_;
  friend struct ::TableStruct_lsm_5fvedodo_2fodo_5fnvmdata_2eproto;
};
// -------------------------------------------------------------------

class OdoNVMData_array_port :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:pb.lsm_vedodo.odo_nvmdata.OdoNVMData_array_port) */ {
 public:
  OdoNVMData_array_port();
  virtual ~OdoNVMData_array_port();

  OdoNVMData_array_port(const OdoNVMData_array_port& from);
  OdoNVMData_array_port(OdoNVMData_array_port&& from) noexcept
    : OdoNVMData_array_port() {
    *this = ::std::move(from);
  }

  inline OdoNVMData_array_port& operator=(const OdoNVMData_array_port& from) {
    CopyFrom(from);
    return *this;
  }
  inline OdoNVMData_array_port& operator=(OdoNVMData_array_port&& from) noexcept {
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
  static const OdoNVMData_array_port& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const OdoNVMData_array_port* internal_default_instance() {
    return reinterpret_cast<const OdoNVMData_array_port*>(
               &_OdoNVMData_array_port_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(OdoNVMData_array_port& a, OdoNVMData_array_port& b) {
    a.Swap(&b);
  }
  inline void Swap(OdoNVMData_array_port* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline OdoNVMData_array_port* New() const final {
    return CreateMaybeMessage<OdoNVMData_array_port>(nullptr);
  }

  OdoNVMData_array_port* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<OdoNVMData_array_port>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const OdoNVMData_array_port& from);
  void MergeFrom(const OdoNVMData_array_port& from);
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
  void InternalSwap(OdoNVMData_array_port* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "pb.lsm_vedodo.odo_nvmdata.OdoNVMData_array_port";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_lsm_5fvedodo_2fodo_5fnvmdata_2eproto);
    return ::descriptor_table_lsm_5fvedodo_2fodo_5fnvmdata_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2135,
  };
  // repeated .pb.lsm_vedodo.odo_nvmdata.OdoNVMData data = 2135;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData* mutable_data(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData >*
      mutable_data();
  private:
  const ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData& _internal_data(int index) const;
  ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData* _internal_add_data();
  public:
  const ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData& data(int index) const;
  ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData* add_data();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData >&
      data() const;

  // @@protoc_insertion_point(class_scope:pb.lsm_vedodo.odo_nvmdata.OdoNVMData_array_port)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData > data_;
  friend struct ::TableStruct_lsm_5fvedodo_2fodo_5fnvmdata_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// OdoNVMData

// optional uint32 uiVersionNumber = 2124;
inline bool OdoNVMData::_internal_has_uiversionnumber() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool OdoNVMData::has_uiversionnumber() const {
  return _internal_has_uiversionnumber();
}
inline void OdoNVMData::clear_uiversionnumber() {
  uiversionnumber_ = 0u;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 OdoNVMData::_internal_uiversionnumber() const {
  return uiversionnumber_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 OdoNVMData::uiversionnumber() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.uiVersionNumber)
  return _internal_uiversionnumber();
}
inline void OdoNVMData::_internal_set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000020u;
  uiversionnumber_ = value;
}
inline void OdoNVMData::set_uiversionnumber(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_uiversionnumber(value);
  // @@protoc_insertion_point(field_set:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.uiVersionNumber)
}

// optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
inline bool OdoNVMData::_internal_has_ssigheader() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || ssigheader_ != nullptr);
  return value;
}
inline bool OdoNVMData::has_ssigheader() const {
  return _internal_has_ssigheader();
}
inline const ::pb::eco::signal_header::SignalHeader& OdoNVMData::_internal_ssigheader() const {
  const ::pb::eco::signal_header::SignalHeader* p = ssigheader_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::eco::signal_header::SignalHeader*>(
      &::pb::eco::signal_header::_SignalHeader_default_instance_);
}
inline const ::pb::eco::signal_header::SignalHeader& OdoNVMData::ssigheader() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.sSigHeader)
  return _internal_ssigheader();
}
inline ::pb::eco::signal_header::SignalHeader* OdoNVMData::release_ssigheader() {
  // @@protoc_insertion_point(field_release:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.sSigHeader)
  _has_bits_[0] &= ~0x00000001u;
  ::pb::eco::signal_header::SignalHeader* temp = ssigheader_;
  ssigheader_ = nullptr;
  return temp;
}
inline ::pb::eco::signal_header::SignalHeader* OdoNVMData::_internal_mutable_ssigheader() {
  _has_bits_[0] |= 0x00000001u;
  if (ssigheader_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::eco::signal_header::SignalHeader>(GetArenaNoVirtual());
    ssigheader_ = p;
  }
  return ssigheader_;
}
inline ::pb::eco::signal_header::SignalHeader* OdoNVMData::mutable_ssigheader() {
  // @@protoc_insertion_point(field_mutable:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.sSigHeader)
  return _internal_mutable_ssigheader();
}
inline void OdoNVMData::set_allocated_ssigheader(::pb::eco::signal_header::SignalHeader* ssigheader) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(ssigheader_);
  }
  if (ssigheader) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      ssigheader = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, ssigheader, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  ssigheader_ = ssigheader;
  // @@protoc_insertion_point(field_set_allocated:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.sSigHeader)
}

// optional .pb.lsm_vedodo.odo_nv_st_whl_ang_cal_t.OdoNvStWhlAngCal_t StWhlAng = 3011;
inline bool OdoNVMData::_internal_has_stwhlang() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  PROTOBUF_ASSUME(!value || stwhlang_ != nullptr);
  return value;
}
inline bool OdoNVMData::has_stwhlang() const {
  return _internal_has_stwhlang();
}
inline const ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t& OdoNVMData::_internal_stwhlang() const {
  const ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t* p = stwhlang_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t*>(
      &::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::_OdoNvStWhlAngCal_t_default_instance_);
}
inline const ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t& OdoNVMData::stwhlang() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.StWhlAng)
  return _internal_stwhlang();
}
inline ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t* OdoNVMData::release_stwhlang() {
  // @@protoc_insertion_point(field_release:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.StWhlAng)
  _has_bits_[0] &= ~0x00000010u;
  ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t* temp = stwhlang_;
  stwhlang_ = nullptr;
  return temp;
}
inline ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t* OdoNVMData::_internal_mutable_stwhlang() {
  _has_bits_[0] |= 0x00000010u;
  if (stwhlang_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t>(GetArenaNoVirtual());
    stwhlang_ = p;
  }
  return stwhlang_;
}
inline ::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t* OdoNVMData::mutable_stwhlang() {
  // @@protoc_insertion_point(field_mutable:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.StWhlAng)
  return _internal_mutable_stwhlang();
}
inline void OdoNVMData::set_allocated_stwhlang(::pb::lsm_vedodo::odo_nv_st_whl_ang_cal_t::OdoNvStWhlAngCal_t* stwhlang) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(stwhlang_);
  }
  if (stwhlang) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      stwhlang = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, stwhlang, submessage_arena);
    }
    _has_bits_[0] |= 0x00000010u;
  } else {
    _has_bits_[0] &= ~0x00000010u;
  }
  stwhlang_ = stwhlang;
  // @@protoc_insertion_point(field_set_allocated:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.StWhlAng)
}

// optional .pb.lsm_vedodo.yw_rate_t.YwRate_t YwRate = 1269;
inline bool OdoNVMData::_internal_has_ywrate() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  PROTOBUF_ASSUME(!value || ywrate_ != nullptr);
  return value;
}
inline bool OdoNVMData::has_ywrate() const {
  return _internal_has_ywrate();
}
inline const ::pb::lsm_vedodo::yw_rate_t::YwRate_t& OdoNVMData::_internal_ywrate() const {
  const ::pb::lsm_vedodo::yw_rate_t::YwRate_t* p = ywrate_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::lsm_vedodo::yw_rate_t::YwRate_t*>(
      &::pb::lsm_vedodo::yw_rate_t::_YwRate_t_default_instance_);
}
inline const ::pb::lsm_vedodo::yw_rate_t::YwRate_t& OdoNVMData::ywrate() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.YwRate)
  return _internal_ywrate();
}
inline ::pb::lsm_vedodo::yw_rate_t::YwRate_t* OdoNVMData::release_ywrate() {
  // @@protoc_insertion_point(field_release:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.YwRate)
  _has_bits_[0] &= ~0x00000004u;
  ::pb::lsm_vedodo::yw_rate_t::YwRate_t* temp = ywrate_;
  ywrate_ = nullptr;
  return temp;
}
inline ::pb::lsm_vedodo::yw_rate_t::YwRate_t* OdoNVMData::_internal_mutable_ywrate() {
  _has_bits_[0] |= 0x00000004u;
  if (ywrate_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::lsm_vedodo::yw_rate_t::YwRate_t>(GetArenaNoVirtual());
    ywrate_ = p;
  }
  return ywrate_;
}
inline ::pb::lsm_vedodo::yw_rate_t::YwRate_t* OdoNVMData::mutable_ywrate() {
  // @@protoc_insertion_point(field_mutable:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.YwRate)
  return _internal_mutable_ywrate();
}
inline void OdoNVMData::set_allocated_ywrate(::pb::lsm_vedodo::yw_rate_t::YwRate_t* ywrate) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(ywrate_);
  }
  if (ywrate) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      ywrate = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, ywrate, submessage_arena);
    }
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  ywrate_ = ywrate;
  // @@protoc_insertion_point(field_set_allocated:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.YwRate)
}

// optional .pb.lsm_vedodo.lat_acc_t.LatAcc_t LatAcc = 2459;
inline bool OdoNVMData::_internal_has_latacc() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  PROTOBUF_ASSUME(!value || latacc_ != nullptr);
  return value;
}
inline bool OdoNVMData::has_latacc() const {
  return _internal_has_latacc();
}
inline const ::pb::lsm_vedodo::lat_acc_t::LatAcc_t& OdoNVMData::_internal_latacc() const {
  const ::pb::lsm_vedodo::lat_acc_t::LatAcc_t* p = latacc_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::lsm_vedodo::lat_acc_t::LatAcc_t*>(
      &::pb::lsm_vedodo::lat_acc_t::_LatAcc_t_default_instance_);
}
inline const ::pb::lsm_vedodo::lat_acc_t::LatAcc_t& OdoNVMData::latacc() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.LatAcc)
  return _internal_latacc();
}
inline ::pb::lsm_vedodo::lat_acc_t::LatAcc_t* OdoNVMData::release_latacc() {
  // @@protoc_insertion_point(field_release:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.LatAcc)
  _has_bits_[0] &= ~0x00000008u;
  ::pb::lsm_vedodo::lat_acc_t::LatAcc_t* temp = latacc_;
  latacc_ = nullptr;
  return temp;
}
inline ::pb::lsm_vedodo::lat_acc_t::LatAcc_t* OdoNVMData::_internal_mutable_latacc() {
  _has_bits_[0] |= 0x00000008u;
  if (latacc_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::lsm_vedodo::lat_acc_t::LatAcc_t>(GetArenaNoVirtual());
    latacc_ = p;
  }
  return latacc_;
}
inline ::pb::lsm_vedodo::lat_acc_t::LatAcc_t* OdoNVMData::mutable_latacc() {
  // @@protoc_insertion_point(field_mutable:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.LatAcc)
  return _internal_mutable_latacc();
}
inline void OdoNVMData::set_allocated_latacc(::pb::lsm_vedodo::lat_acc_t::LatAcc_t* latacc) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(latacc_);
  }
  if (latacc) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      latacc = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, latacc, submessage_arena);
    }
    _has_bits_[0] |= 0x00000008u;
  } else {
    _has_bits_[0] &= ~0x00000008u;
  }
  latacc_ = latacc;
  // @@protoc_insertion_point(field_set_allocated:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.LatAcc)
}

// optional .pb.lsm_vedodo.odo_nvm_state_t.OdoNvmState_t State = 1214;
inline bool OdoNVMData::_internal_has_state() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || state_ != nullptr);
  return value;
}
inline bool OdoNVMData::has_state() const {
  return _internal_has_state();
}
inline const ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t& OdoNVMData::_internal_state() const {
  const ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t* p = state_;
  return p != nullptr ? *p : *reinterpret_cast<const ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t*>(
      &::pb::lsm_vedodo::odo_nvm_state_t::_OdoNvmState_t_default_instance_);
}
inline const ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t& OdoNVMData::state() const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.State)
  return _internal_state();
}
inline ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t* OdoNVMData::release_state() {
  // @@protoc_insertion_point(field_release:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.State)
  _has_bits_[0] &= ~0x00000002u;
  ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t* temp = state_;
  state_ = nullptr;
  return temp;
}
inline ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t* OdoNVMData::_internal_mutable_state() {
  _has_bits_[0] |= 0x00000002u;
  if (state_ == nullptr) {
    auto* p = CreateMaybeMessage<::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t>(GetArenaNoVirtual());
    state_ = p;
  }
  return state_;
}
inline ::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t* OdoNVMData::mutable_state() {
  // @@protoc_insertion_point(field_mutable:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.State)
  return _internal_mutable_state();
}
inline void OdoNVMData::set_allocated_state(::pb::lsm_vedodo::odo_nvm_state_t::OdoNvmState_t* state) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(state_);
  }
  if (state) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      state = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, state, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  state_ = state;
  // @@protoc_insertion_point(field_set_allocated:pb.lsm_vedodo.odo_nvmdata.OdoNVMData.State)
}

// -------------------------------------------------------------------

// OdoNVMData_array_port

// repeated .pb.lsm_vedodo.odo_nvmdata.OdoNVMData data = 2135;
inline int OdoNVMData_array_port::_internal_data_size() const {
  return data_.size();
}
inline int OdoNVMData_array_port::data_size() const {
  return _internal_data_size();
}
inline void OdoNVMData_array_port::clear_data() {
  data_.Clear();
}
inline ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData* OdoNVMData_array_port::mutable_data(int index) {
  // @@protoc_insertion_point(field_mutable:pb.lsm_vedodo.odo_nvmdata.OdoNVMData_array_port.data)
  return data_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData >*
OdoNVMData_array_port::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:pb.lsm_vedodo.odo_nvmdata.OdoNVMData_array_port.data)
  return &data_;
}
inline const ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData& OdoNVMData_array_port::_internal_data(int index) const {
  return data_.Get(index);
}
inline const ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData& OdoNVMData_array_port::data(int index) const {
  // @@protoc_insertion_point(field_get:pb.lsm_vedodo.odo_nvmdata.OdoNVMData_array_port.data)
  return _internal_data(index);
}
inline ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData* OdoNVMData_array_port::_internal_add_data() {
  return data_.Add();
}
inline ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData* OdoNVMData_array_port::add_data() {
  // @@protoc_insertion_point(field_add:pb.lsm_vedodo.odo_nvmdata.OdoNVMData_array_port.data)
  return _internal_add_data();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::pb::lsm_vedodo::odo_nvmdata::OdoNVMData >&
OdoNVMData_array_port::data() const {
  // @@protoc_insertion_point(field_list:pb.lsm_vedodo.odo_nvmdata.OdoNVMData_array_port.data)
  return data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace odo_nvmdata
}  // namespace lsm_vedodo
}  // namespace pb

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_lsm_5fvedodo_2fodo_5fnvmdata_2eproto
