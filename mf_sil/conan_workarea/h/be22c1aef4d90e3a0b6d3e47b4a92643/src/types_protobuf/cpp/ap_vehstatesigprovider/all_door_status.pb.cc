// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/all_door_status.proto

#include "ap_vehstatesigprovider/all_door_status.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_AllDoorStatus_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace all_door_status {
class AllDoorStatusDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<AllDoorStatus> _instance;
} _AllDoorStatus_default_instance_;
class AllDoorStatus_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<AllDoorStatus_array_port> _instance;
} _AllDoorStatus_array_port_default_instance_;
}  // namespace all_door_status
}  // namespace ap_vehstatesigprovider
}  // namespace pb
static void InitDefaultsscc_info_AllDoorStatus_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_vehstatesigprovider::all_door_status::_AllDoorStatus_default_instance_;
    new (ptr) ::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_AllDoorStatus_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_AllDoorStatus_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto}, {}};

static void InitDefaultsscc_info_AllDoorStatus_array_port_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_vehstatesigprovider::all_door_status::_AllDoorStatus_array_port_default_instance_;
    new (ptr) ::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_AllDoorStatus_array_port_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_AllDoorStatus_array_port_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto}, {
      &scc_info_AllDoorStatus_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus, frontpsgr_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus, driver_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus, rearright_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus, rearleft_nu_),
  3,
  2,
  0,
  1,
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus)},
  { 13, 19, sizeof(::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_vehstatesigprovider::all_door_status::_AllDoorStatus_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_vehstatesigprovider::all_door_status::_AllDoorStatus_array_port_default_instance_),
};

const char descriptor_table_protodef_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n,ap_vehstatesigprovider/all_door_status"
  ".proto\022)pb.ap_vehstatesigprovider.all_do"
  "or_status\032(ap_vehstatesigprovider/door_s"
  "tatus.proto\"\263\002\n\rAllDoorStatus\022H\n\014frontPs"
  "gr_nu\030\276\024 \001(\01621.pb.ap_vehstatesigprovider"
  ".door_status.DoorStatus\022E\n\tdriver_nu\030\347\017 "
  "\001(\01621.pb.ap_vehstatesigprovider.door_sta"
  "tus.DoorStatus\022H\n\014rearRight_nu\030\207\t \001(\01621."
  "pb.ap_vehstatesigprovider.door_status.Do"
  "orStatus\022G\n\013rearLeft_nu\030\225\016 \001(\01621.pb.ap_v"
  "ehstatesigprovider.door_status.DoorStatu"
  "s\"c\n\030AllDoorStatus_array_port\022G\n\004data\030\306\025"
  " \003(\01328.pb.ap_vehstatesigprovider.all_doo"
  "r_status.AllDoorStatus"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto_deps[1] = {
  &::descriptor_table_ap_5fvehstatesigprovider_2fdoor_5fstatus_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto_sccs[2] = {
  &scc_info_AllDoorStatus_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto.base,
  &scc_info_AllDoorStatus_array_port_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto_once;
static bool descriptor_table_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto = {
  &descriptor_table_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto_initialized, descriptor_table_protodef_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto, "ap_vehstatesigprovider/all_door_status.proto", 542,
  &descriptor_table_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto_once, descriptor_table_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto_sccs, descriptor_table_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto::offsets,
  file_level_metadata_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto, 2, file_level_enum_descriptors_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto, file_level_service_descriptors_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto), true);
namespace pb {
namespace ap_vehstatesigprovider {
namespace all_door_status {

// ===================================================================

void AllDoorStatus::InitAsDefaultInstance() {
}
class AllDoorStatus::_Internal {
 public:
  using HasBits = decltype(std::declval<AllDoorStatus>()._has_bits_);
  static void set_has_frontpsgr_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_driver_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_rearright_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_rearleft_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

AllDoorStatus::AllDoorStatus()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
}
AllDoorStatus::AllDoorStatus(const AllDoorStatus& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&rearright_nu_, &from.rearright_nu_,
    static_cast<size_t>(reinterpret_cast<char*>(&frontpsgr_nu_) -
    reinterpret_cast<char*>(&rearright_nu_)) + sizeof(frontpsgr_nu_));
  // @@protoc_insertion_point(copy_constructor:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
}

void AllDoorStatus::SharedCtor() {
  ::memset(&rearright_nu_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&frontpsgr_nu_) -
      reinterpret_cast<char*>(&rearright_nu_)) + sizeof(frontpsgr_nu_));
}

AllDoorStatus::~AllDoorStatus() {
  // @@protoc_insertion_point(destructor:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
  SharedDtor();
}

void AllDoorStatus::SharedDtor() {
}

void AllDoorStatus::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const AllDoorStatus& AllDoorStatus::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_AllDoorStatus_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto.base);
  return *internal_default_instance();
}


void AllDoorStatus::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    ::memset(&rearright_nu_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&frontpsgr_nu_) -
        reinterpret_cast<char*>(&rearright_nu_)) + sizeof(frontpsgr_nu_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* AllDoorStatus::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .pb.ap_vehstatesigprovider.door_status.DoorStatus rearRight_nu = 1159;
      case 1159:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_vehstatesigprovider::door_status::DoorStatus_IsValid(val))) {
            _internal_set_rearright_nu(static_cast<::pb::ap_vehstatesigprovider::door_status::DoorStatus>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(1159, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_vehstatesigprovider.door_status.DoorStatus rearLeft_nu = 1813;
      case 1813:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 168)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_vehstatesigprovider::door_status::DoorStatus_IsValid(val))) {
            _internal_set_rearleft_nu(static_cast<::pb::ap_vehstatesigprovider::door_status::DoorStatus>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(1813, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_vehstatesigprovider.door_status.DoorStatus driver_nu = 2023;
      case 2023:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_vehstatesigprovider::door_status::DoorStatus_IsValid(val))) {
            _internal_set_driver_nu(static_cast<::pb::ap_vehstatesigprovider::door_status::DoorStatus>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(2023, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_vehstatesigprovider.door_status.DoorStatus frontPsgr_nu = 2622;
      case 2622:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 240)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_vehstatesigprovider::door_status::DoorStatus_IsValid(val))) {
            _internal_set_frontpsgr_nu(static_cast<::pb::ap_vehstatesigprovider::door_status::DoorStatus>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(2622, val, mutable_unknown_fields());
          }
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

::PROTOBUF_NAMESPACE_ID::uint8* AllDoorStatus::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.ap_vehstatesigprovider.door_status.DoorStatus rearRight_nu = 1159;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      1159, this->_internal_rearright_nu(), target);
  }

  // optional .pb.ap_vehstatesigprovider.door_status.DoorStatus rearLeft_nu = 1813;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      1813, this->_internal_rearleft_nu(), target);
  }

  // optional .pb.ap_vehstatesigprovider.door_status.DoorStatus driver_nu = 2023;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      2023, this->_internal_driver_nu(), target);
  }

  // optional .pb.ap_vehstatesigprovider.door_status.DoorStatus frontPsgr_nu = 2622;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      2622, this->_internal_frontpsgr_nu(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
  return target;
}

size_t AllDoorStatus::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional .pb.ap_vehstatesigprovider.door_status.DoorStatus rearRight_nu = 1159;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_rearright_nu());
    }

    // optional .pb.ap_vehstatesigprovider.door_status.DoorStatus rearLeft_nu = 1813;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_rearleft_nu());
    }

    // optional .pb.ap_vehstatesigprovider.door_status.DoorStatus driver_nu = 2023;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_driver_nu());
    }

    // optional .pb.ap_vehstatesigprovider.door_status.DoorStatus frontPsgr_nu = 2622;
    if (cached_has_bits & 0x00000008u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_frontpsgr_nu());
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

void AllDoorStatus::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
  GOOGLE_DCHECK_NE(&from, this);
  const AllDoorStatus* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<AllDoorStatus>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
    MergeFrom(*source);
  }
}

void AllDoorStatus::MergeFrom(const AllDoorStatus& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      rearright_nu_ = from.rearright_nu_;
    }
    if (cached_has_bits & 0x00000002u) {
      rearleft_nu_ = from.rearleft_nu_;
    }
    if (cached_has_bits & 0x00000004u) {
      driver_nu_ = from.driver_nu_;
    }
    if (cached_has_bits & 0x00000008u) {
      frontpsgr_nu_ = from.frontpsgr_nu_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void AllDoorStatus::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void AllDoorStatus::CopyFrom(const AllDoorStatus& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool AllDoorStatus::IsInitialized() const {
  return true;
}

void AllDoorStatus::InternalSwap(AllDoorStatus* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(rearright_nu_, other->rearright_nu_);
  swap(rearleft_nu_, other->rearleft_nu_);
  swap(driver_nu_, other->driver_nu_);
  swap(frontpsgr_nu_, other->frontpsgr_nu_);
}

::PROTOBUF_NAMESPACE_ID::Metadata AllDoorStatus::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void AllDoorStatus_array_port::InitAsDefaultInstance() {
}
class AllDoorStatus_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<AllDoorStatus_array_port>()._has_bits_);
};

AllDoorStatus_array_port::AllDoorStatus_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
}
AllDoorStatus_array_port::AllDoorStatus_array_port(const AllDoorStatus_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
}

void AllDoorStatus_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_AllDoorStatus_array_port_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto.base);
}

AllDoorStatus_array_port::~AllDoorStatus_array_port() {
  // @@protoc_insertion_point(destructor:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
  SharedDtor();
}

void AllDoorStatus_array_port::SharedDtor() {
}

void AllDoorStatus_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const AllDoorStatus_array_port& AllDoorStatus_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_AllDoorStatus_array_port_ap_5fvehstatesigprovider_2fall_5fdoor_5fstatus_2eproto.base);
  return *internal_default_instance();
}


void AllDoorStatus_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* AllDoorStatus_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus data = 2758;
      case 2758:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 50)) {
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

::PROTOBUF_NAMESPACE_ID::uint8* AllDoorStatus_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus data = 2758;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2758, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
  return target;
}

size_t AllDoorStatus_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus data = 2758;
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

void AllDoorStatus_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const AllDoorStatus_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<AllDoorStatus_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
    MergeFrom(*source);
  }
}

void AllDoorStatus_array_port::MergeFrom(const AllDoorStatus_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void AllDoorStatus_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void AllDoorStatus_array_port::CopyFrom(const AllDoorStatus_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_vehstatesigprovider.all_door_status.AllDoorStatus_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool AllDoorStatus_array_port::IsInitialized() const {
  return true;
}

void AllDoorStatus_array_port::InternalSwap(AllDoorStatus_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata AllDoorStatus_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace all_door_status
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus* Arena::CreateMaybeMessage< ::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus_array_port* Arena::CreateMaybeMessage< ::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_vehstatesigprovider::all_door_status::AllDoorStatus_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
