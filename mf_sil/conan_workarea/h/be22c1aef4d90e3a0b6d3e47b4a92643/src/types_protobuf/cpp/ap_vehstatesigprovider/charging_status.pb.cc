// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/charging_status.proto

#include "ap_vehstatesigprovider/charging_status.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
extern PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ChargingStatus_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace charging_status {
class ChargingStatusDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ChargingStatus> _instance;
} _ChargingStatus_default_instance_;
class ChargingStatus_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ChargingStatus_array_port> _instance;
} _ChargingStatus_array_port_default_instance_;
}  // namespace charging_status
}  // namespace ap_vehstatesigprovider
}  // namespace pb
static void InitDefaultsscc_info_ChargingStatus_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_vehstatesigprovider::charging_status::_ChargingStatus_default_instance_;
    new (ptr) ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ChargingStatus_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_ChargingStatus_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto}, {}};

static void InitDefaultsscc_info_ChargingStatus_array_port_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_vehstatesigprovider::charging_status::_ChargingStatus_array_port_default_instance_;
    new (ptr) ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ChargingStatus_array_port_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_ChargingStatus_array_port_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto}, {
      &scc_info_ChargingStatus_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::charging_status::ChargingStatus, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::charging_status::ChargingStatus, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::charging_status::ChargingStatus, ev_charging_is_installed_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::charging_status::ChargingStatus, chargingconnectorstatus_nu_),
  1,
  0,
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::charging_status::ChargingStatus_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::charging_status::ChargingStatus_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::charging_status::ChargingStatus_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::pb::ap_vehstatesigprovider::charging_status::ChargingStatus)},
  { 9, 15, sizeof(::pb::ap_vehstatesigprovider::charging_status::ChargingStatus_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_vehstatesigprovider::charging_status::_ChargingStatus_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_vehstatesigprovider::charging_status::_ChargingStatus_array_port_default_instance_),
};

const char descriptor_table_protodef_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n,ap_vehstatesigprovider/charging_status"
  ".proto\022)pb.ap_vehstatesigprovider.chargi"
  "ng_status\0326ap_vehstatesigprovider/chargi"
  "ng_connector_status.proto\"\251\001\n\016ChargingSt"
  "atus\022$\n\033ev_charging_is_installed_nu\030\221\013 \001"
  "(\010\022q\n\032chargingConnectorStatus_nu\030\246\010 \001(\0162"
  "L.pb.ap_vehstatesigprovider.charging_con"
  "nector_status.ChargingConnectorStatus\"e\n"
  "\031ChargingStatus_array_port\022H\n\004data\030\367\035 \003("
  "\01329.pb.ap_vehstatesigprovider.charging_s"
  "tatus.ChargingStatus"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto_deps[1] = {
  &::descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fconnector_5fstatus_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto_sccs[2] = {
  &scc_info_ChargingStatus_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto.base,
  &scc_info_ChargingStatus_array_port_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto_once;
static bool descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto = {
  &descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto_initialized, descriptor_table_protodef_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto, "ap_vehstatesigprovider/charging_status.proto", 420,
  &descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto_once, descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto_sccs, descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto::offsets,
  file_level_metadata_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto, 2, file_level_enum_descriptors_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto, file_level_service_descriptors_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto), true);
namespace pb {
namespace ap_vehstatesigprovider {
namespace charging_status {

// ===================================================================

void ChargingStatus::InitAsDefaultInstance() {
}
class ChargingStatus::_Internal {
 public:
  using HasBits = decltype(std::declval<ChargingStatus>()._has_bits_);
  static void set_has_ev_charging_is_installed_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_chargingconnectorstatus_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

ChargingStatus::ChargingStatus()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
}
ChargingStatus::ChargingStatus(const ChargingStatus& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&chargingconnectorstatus_nu_, &from.chargingconnectorstatus_nu_,
    static_cast<size_t>(reinterpret_cast<char*>(&ev_charging_is_installed_nu_) -
    reinterpret_cast<char*>(&chargingconnectorstatus_nu_)) + sizeof(ev_charging_is_installed_nu_));
  // @@protoc_insertion_point(copy_constructor:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
}

void ChargingStatus::SharedCtor() {
  ::memset(&chargingconnectorstatus_nu_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&ev_charging_is_installed_nu_) -
      reinterpret_cast<char*>(&chargingconnectorstatus_nu_)) + sizeof(ev_charging_is_installed_nu_));
}

ChargingStatus::~ChargingStatus() {
  // @@protoc_insertion_point(destructor:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
  SharedDtor();
}

void ChargingStatus::SharedDtor() {
}

void ChargingStatus::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ChargingStatus& ChargingStatus::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ChargingStatus_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto.base);
  return *internal_default_instance();
}


void ChargingStatus::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    ::memset(&chargingconnectorstatus_nu_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&ev_charging_is_installed_nu_) -
        reinterpret_cast<char*>(&chargingconnectorstatus_nu_)) + sizeof(ev_charging_is_installed_nu_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ChargingStatus::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .pb.ap_vehstatesigprovider.charging_connector_status.ChargingConnectorStatus chargingConnectorStatus_nu = 1062;
      case 1062:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_vehstatesigprovider::charging_connector_status::ChargingConnectorStatus_IsValid(val))) {
            _internal_set_chargingconnectorstatus_nu(static_cast<::pb::ap_vehstatesigprovider::charging_connector_status::ChargingConnectorStatus>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(1062, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional bool ev_charging_is_installed_nu = 1425;
      case 1425:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 136)) {
          _Internal::set_has_ev_charging_is_installed_nu(&has_bits);
          ev_charging_is_installed_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* ChargingStatus::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.ap_vehstatesigprovider.charging_connector_status.ChargingConnectorStatus chargingConnectorStatus_nu = 1062;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      1062, this->_internal_chargingconnectorstatus_nu(), target);
  }

  // optional bool ev_charging_is_installed_nu = 1425;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1425, this->_internal_ev_charging_is_installed_nu(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
  return target;
}

size_t ChargingStatus::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional .pb.ap_vehstatesigprovider.charging_connector_status.ChargingConnectorStatus chargingConnectorStatus_nu = 1062;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_chargingconnectorstatus_nu());
    }

    // optional bool ev_charging_is_installed_nu = 1425;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 + 1;
    }

  }
  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ChargingStatus::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
  GOOGLE_DCHECK_NE(&from, this);
  const ChargingStatus* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ChargingStatus>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
    MergeFrom(*source);
  }
}

void ChargingStatus::MergeFrom(const ChargingStatus& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      chargingconnectorstatus_nu_ = from.chargingconnectorstatus_nu_;
    }
    if (cached_has_bits & 0x00000002u) {
      ev_charging_is_installed_nu_ = from.ev_charging_is_installed_nu_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ChargingStatus::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ChargingStatus::CopyFrom(const ChargingStatus& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ChargingStatus::IsInitialized() const {
  return true;
}

void ChargingStatus::InternalSwap(ChargingStatus* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(chargingconnectorstatus_nu_, other->chargingconnectorstatus_nu_);
  swap(ev_charging_is_installed_nu_, other->ev_charging_is_installed_nu_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ChargingStatus::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void ChargingStatus_array_port::InitAsDefaultInstance() {
}
class ChargingStatus_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<ChargingStatus_array_port>()._has_bits_);
};

ChargingStatus_array_port::ChargingStatus_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
}
ChargingStatus_array_port::ChargingStatus_array_port(const ChargingStatus_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
}

void ChargingStatus_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ChargingStatus_array_port_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto.base);
}

ChargingStatus_array_port::~ChargingStatus_array_port() {
  // @@protoc_insertion_point(destructor:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
  SharedDtor();
}

void ChargingStatus_array_port::SharedDtor() {
}

void ChargingStatus_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ChargingStatus_array_port& ChargingStatus_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ChargingStatus_array_port_ap_5fvehstatesigprovider_2fcharging_5fstatus_2eproto.base);
  return *internal_default_instance();
}


void ChargingStatus_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ChargingStatus_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.ap_vehstatesigprovider.charging_status.ChargingStatus data = 3831;
      case 3831:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 186)) {
          ptr = ctx->ParseMessage(_internal_add_data(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* ChargingStatus_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.ap_vehstatesigprovider.charging_status.ChargingStatus data = 3831;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3831, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
  return target;
}

size_t ChargingStatus_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.ap_vehstatesigprovider.charging_status.ChargingStatus data = 3831;
  total_size += 3UL * this->_internal_data_size();
  for (const auto& msg : this->data_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ChargingStatus_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const ChargingStatus_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ChargingStatus_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
    MergeFrom(*source);
  }
}

void ChargingStatus_array_port::MergeFrom(const ChargingStatus_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void ChargingStatus_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ChargingStatus_array_port::CopyFrom(const ChargingStatus_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_vehstatesigprovider.charging_status.ChargingStatus_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ChargingStatus_array_port::IsInitialized() const {
  return true;
}

void ChargingStatus_array_port::InternalSwap(ChargingStatus_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ChargingStatus_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace charging_status
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus* Arena::CreateMaybeMessage< ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus_array_port* Arena::CreateMaybeMessage< ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_vehstatesigprovider::charging_status::ChargingStatus_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
