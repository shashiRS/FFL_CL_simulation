// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: pdcp/sector_info.proto

#include "pdcp/sector_info.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_pdcp_2fsector_5finfo_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SectorInfo_pdcp_2fsector_5finfo_2eproto;
namespace pb {
namespace pdcp {
namespace sector_info {
class SectorInfoDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<SectorInfo> _instance;
} _SectorInfo_default_instance_;
class SectorInfo_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<SectorInfo_array_port> _instance;
} _SectorInfo_array_port_default_instance_;
}  // namespace sector_info
}  // namespace pdcp
}  // namespace pb
static void InitDefaultsscc_info_SectorInfo_pdcp_2fsector_5finfo_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::pdcp::sector_info::_SectorInfo_default_instance_;
    new (ptr) ::pb::pdcp::sector_info::SectorInfo();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::pdcp::sector_info::SectorInfo::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SectorInfo_pdcp_2fsector_5finfo_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_SectorInfo_pdcp_2fsector_5finfo_2eproto}, {}};

static void InitDefaultsscc_info_SectorInfo_array_port_pdcp_2fsector_5finfo_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::pdcp::sector_info::_SectorInfo_array_port_default_instance_;
    new (ptr) ::pb::pdcp::sector_info::SectorInfo_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::pdcp::sector_info::SectorInfo_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_SectorInfo_array_port_pdcp_2fsector_5finfo_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_SectorInfo_array_port_pdcp_2fsector_5finfo_2eproto}, {
      &scc_info_SectorInfo_pdcp_2fsector_5finfo_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_pdcp_2fsector_5finfo_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_pdcp_2fsector_5finfo_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_pdcp_2fsector_5finfo_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_pdcp_2fsector_5finfo_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::sector_info::SectorInfo, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::sector_info::SectorInfo, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::sector_info::SectorInfo, smallestdistance_m_),
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::sector_info::SectorInfo, dynamicsmallestdistance_m_),
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::sector_info::SectorInfo, sectorid_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::sector_info::SectorInfo, criticalitylevel_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::sector_info::SectorInfo, scanned_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::sector_info::SectorInfo, intersectsdrvtube_nu_),
  1,
  5,
  0,
  4,
  2,
  3,
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::sector_info::SectorInfo_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::sector_info::SectorInfo_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::sector_info::SectorInfo_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 11, sizeof(::pb::pdcp::sector_info::SectorInfo)},
  { 17, 23, sizeof(::pb::pdcp::sector_info::SectorInfo_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::pdcp::sector_info::_SectorInfo_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::pdcp::sector_info::_SectorInfo_array_port_default_instance_),
};

const char descriptor_table_protodef_pdcp_2fsector_5finfo_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\026pdcp/sector_info.proto\022\023pb.pdcp.sector"
  "_info\032\034pdcp/criticality_level.proto\"\342\001\n\n"
  "SectorInfo\022\033\n\022smallestDistance_m\030\203\007 \001(\002\022"
  "\"\n\031dynamicSmallestDistance_m\030\234\021 \001(\002\022\024\n\013s"
  "ectorID_nu\030\342\035 \001(\r\022I\n\023criticalityLevel_nu"
  "\030\326\t \001(\0162+.pb.pdcp.criticality_level.Crit"
  "icalityLevel\022\023\n\nscanned_nu\030\226\010 \001(\010\022\035\n\024int"
  "ersectsDrvTube_nu\030\234\n \001(\010\"G\n\025SectorInfo_a"
  "rray_port\022.\n\004data\030\366\033 \003(\0132\037.pb.pdcp.secto"
  "r_info.SectorInfo"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_pdcp_2fsector_5finfo_2eproto_deps[1] = {
  &::descriptor_table_pdcp_2fcriticality_5flevel_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_pdcp_2fsector_5finfo_2eproto_sccs[2] = {
  &scc_info_SectorInfo_pdcp_2fsector_5finfo_2eproto.base,
  &scc_info_SectorInfo_array_port_pdcp_2fsector_5finfo_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_pdcp_2fsector_5finfo_2eproto_once;
static bool descriptor_table_pdcp_2fsector_5finfo_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_pdcp_2fsector_5finfo_2eproto = {
  &descriptor_table_pdcp_2fsector_5finfo_2eproto_initialized, descriptor_table_protodef_pdcp_2fsector_5finfo_2eproto, "pdcp/sector_info.proto", 377,
  &descriptor_table_pdcp_2fsector_5finfo_2eproto_once, descriptor_table_pdcp_2fsector_5finfo_2eproto_sccs, descriptor_table_pdcp_2fsector_5finfo_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_pdcp_2fsector_5finfo_2eproto::offsets,
  file_level_metadata_pdcp_2fsector_5finfo_2eproto, 2, file_level_enum_descriptors_pdcp_2fsector_5finfo_2eproto, file_level_service_descriptors_pdcp_2fsector_5finfo_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_pdcp_2fsector_5finfo_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_pdcp_2fsector_5finfo_2eproto), true);
namespace pb {
namespace pdcp {
namespace sector_info {

// ===================================================================

void SectorInfo::InitAsDefaultInstance() {
}
class SectorInfo::_Internal {
 public:
  using HasBits = decltype(std::declval<SectorInfo>()._has_bits_);
  static void set_has_smallestdistance_m(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_dynamicsmallestdistance_m(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_sectorid_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_criticalitylevel_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_scanned_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_intersectsdrvtube_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
};

SectorInfo::SectorInfo()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.pdcp.sector_info.SectorInfo)
}
SectorInfo::SectorInfo(const SectorInfo& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&sectorid_nu_, &from.sectorid_nu_,
    static_cast<size_t>(reinterpret_cast<char*>(&dynamicsmallestdistance_m_) -
    reinterpret_cast<char*>(&sectorid_nu_)) + sizeof(dynamicsmallestdistance_m_));
  // @@protoc_insertion_point(copy_constructor:pb.pdcp.sector_info.SectorInfo)
}

void SectorInfo::SharedCtor() {
  ::memset(&sectorid_nu_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&dynamicsmallestdistance_m_) -
      reinterpret_cast<char*>(&sectorid_nu_)) + sizeof(dynamicsmallestdistance_m_));
}

SectorInfo::~SectorInfo() {
  // @@protoc_insertion_point(destructor:pb.pdcp.sector_info.SectorInfo)
  SharedDtor();
}

void SectorInfo::SharedDtor() {
}

void SectorInfo::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const SectorInfo& SectorInfo::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_SectorInfo_pdcp_2fsector_5finfo_2eproto.base);
  return *internal_default_instance();
}


void SectorInfo::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.pdcp.sector_info.SectorInfo)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    ::memset(&sectorid_nu_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&dynamicsmallestdistance_m_) -
        reinterpret_cast<char*>(&sectorid_nu_)) + sizeof(dynamicsmallestdistance_m_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* SectorInfo::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional float smallestDistance_m = 899;
      case 899:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 29)) {
          _Internal::set_has_smallestdistance_m(&has_bits);
          smallestdistance_m_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional bool scanned_nu = 1046;
      case 1046:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 176)) {
          _Internal::set_has_scanned_nu(&has_bits);
          scanned_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.pdcp.criticality_level.CriticalityLevel criticalityLevel_nu = 1238;
      case 1238:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 176)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::pdcp::criticality_level::CriticalityLevel_IsValid(val))) {
            _internal_set_criticalitylevel_nu(static_cast<::pb::pdcp::criticality_level::CriticalityLevel>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(1238, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional bool intersectsDrvTube_nu = 1308;
      case 1308:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 224)) {
          _Internal::set_has_intersectsdrvtube_nu(&has_bits);
          intersectsdrvtube_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional float dynamicSmallestDistance_m = 2204;
      case 2204:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 229)) {
          _Internal::set_has_dynamicsmallestdistance_m(&has_bits);
          dynamicsmallestdistance_m_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional uint32 sectorID_nu = 3810;
      case 3810:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_sectorid_nu(&has_bits);
          sectorid_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* SectorInfo::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.pdcp.sector_info.SectorInfo)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional float smallestDistance_m = 899;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(899, this->_internal_smallestdistance_m(), target);
  }

  // optional bool scanned_nu = 1046;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1046, this->_internal_scanned_nu(), target);
  }

  // optional .pb.pdcp.criticality_level.CriticalityLevel criticalityLevel_nu = 1238;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      1238, this->_internal_criticalitylevel_nu(), target);
  }

  // optional bool intersectsDrvTube_nu = 1308;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1308, this->_internal_intersectsdrvtube_nu(), target);
  }

  // optional float dynamicSmallestDistance_m = 2204;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2204, this->_internal_dynamicsmallestdistance_m(), target);
  }

  // optional uint32 sectorID_nu = 3810;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(3810, this->_internal_sectorid_nu(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.pdcp.sector_info.SectorInfo)
  return target;
}

size_t SectorInfo::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.pdcp.sector_info.SectorInfo)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    // optional uint32 sectorID_nu = 3810;
    if (cached_has_bits & 0x00000001u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_sectorid_nu());
    }

    // optional float smallestDistance_m = 899;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 + 4;
    }

    // optional bool scanned_nu = 1046;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 + 1;
    }

    // optional bool intersectsDrvTube_nu = 1308;
    if (cached_has_bits & 0x00000008u) {
      total_size += 2 + 1;
    }

    // optional .pb.pdcp.criticality_level.CriticalityLevel criticalityLevel_nu = 1238;
    if (cached_has_bits & 0x00000010u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_criticalitylevel_nu());
    }

    // optional float dynamicSmallestDistance_m = 2204;
    if (cached_has_bits & 0x00000020u) {
      total_size += 3 + 4;
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

void SectorInfo::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.pdcp.sector_info.SectorInfo)
  GOOGLE_DCHECK_NE(&from, this);
  const SectorInfo* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<SectorInfo>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.pdcp.sector_info.SectorInfo)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.pdcp.sector_info.SectorInfo)
    MergeFrom(*source);
  }
}

void SectorInfo::MergeFrom(const SectorInfo& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.pdcp.sector_info.SectorInfo)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    if (cached_has_bits & 0x00000001u) {
      sectorid_nu_ = from.sectorid_nu_;
    }
    if (cached_has_bits & 0x00000002u) {
      smallestdistance_m_ = from.smallestdistance_m_;
    }
    if (cached_has_bits & 0x00000004u) {
      scanned_nu_ = from.scanned_nu_;
    }
    if (cached_has_bits & 0x00000008u) {
      intersectsdrvtube_nu_ = from.intersectsdrvtube_nu_;
    }
    if (cached_has_bits & 0x00000010u) {
      criticalitylevel_nu_ = from.criticalitylevel_nu_;
    }
    if (cached_has_bits & 0x00000020u) {
      dynamicsmallestdistance_m_ = from.dynamicsmallestdistance_m_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void SectorInfo::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.pdcp.sector_info.SectorInfo)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SectorInfo::CopyFrom(const SectorInfo& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.pdcp.sector_info.SectorInfo)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SectorInfo::IsInitialized() const {
  return true;
}

void SectorInfo::InternalSwap(SectorInfo* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(sectorid_nu_, other->sectorid_nu_);
  swap(smallestdistance_m_, other->smallestdistance_m_);
  swap(scanned_nu_, other->scanned_nu_);
  swap(intersectsdrvtube_nu_, other->intersectsdrvtube_nu_);
  swap(criticalitylevel_nu_, other->criticalitylevel_nu_);
  swap(dynamicsmallestdistance_m_, other->dynamicsmallestdistance_m_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SectorInfo::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void SectorInfo_array_port::InitAsDefaultInstance() {
}
class SectorInfo_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<SectorInfo_array_port>()._has_bits_);
};

SectorInfo_array_port::SectorInfo_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.pdcp.sector_info.SectorInfo_array_port)
}
SectorInfo_array_port::SectorInfo_array_port(const SectorInfo_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.pdcp.sector_info.SectorInfo_array_port)
}

void SectorInfo_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_SectorInfo_array_port_pdcp_2fsector_5finfo_2eproto.base);
}

SectorInfo_array_port::~SectorInfo_array_port() {
  // @@protoc_insertion_point(destructor:pb.pdcp.sector_info.SectorInfo_array_port)
  SharedDtor();
}

void SectorInfo_array_port::SharedDtor() {
}

void SectorInfo_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const SectorInfo_array_port& SectorInfo_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_SectorInfo_array_port_pdcp_2fsector_5finfo_2eproto.base);
  return *internal_default_instance();
}


void SectorInfo_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.pdcp.sector_info.SectorInfo_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* SectorInfo_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.pdcp.sector_info.SectorInfo data = 3574;
      case 3574:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 178)) {
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

::PROTOBUF_NAMESPACE_ID::uint8* SectorInfo_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.pdcp.sector_info.SectorInfo_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.pdcp.sector_info.SectorInfo data = 3574;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3574, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.pdcp.sector_info.SectorInfo_array_port)
  return target;
}

size_t SectorInfo_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.pdcp.sector_info.SectorInfo_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.pdcp.sector_info.SectorInfo data = 3574;
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

void SectorInfo_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.pdcp.sector_info.SectorInfo_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const SectorInfo_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<SectorInfo_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.pdcp.sector_info.SectorInfo_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.pdcp.sector_info.SectorInfo_array_port)
    MergeFrom(*source);
  }
}

void SectorInfo_array_port::MergeFrom(const SectorInfo_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.pdcp.sector_info.SectorInfo_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void SectorInfo_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.pdcp.sector_info.SectorInfo_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SectorInfo_array_port::CopyFrom(const SectorInfo_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.pdcp.sector_info.SectorInfo_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SectorInfo_array_port::IsInitialized() const {
  return true;
}

void SectorInfo_array_port::InternalSwap(SectorInfo_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SectorInfo_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace sector_info
}  // namespace pdcp
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::pdcp::sector_info::SectorInfo* Arena::CreateMaybeMessage< ::pb::pdcp::sector_info::SectorInfo >(Arena* arena) {
  return Arena::CreateInternal< ::pb::pdcp::sector_info::SectorInfo >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::pdcp::sector_info::SectorInfo_array_port* Arena::CreateMaybeMessage< ::pb::pdcp::sector_info::SectorInfo_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::pdcp::sector_info::SectorInfo_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
