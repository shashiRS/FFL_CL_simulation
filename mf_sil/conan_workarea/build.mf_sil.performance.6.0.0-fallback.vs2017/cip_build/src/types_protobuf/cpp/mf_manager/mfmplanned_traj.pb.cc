// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_manager/mfmplanned_traj.proto

#include "mf_manager/mfmplanned_traj.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_mf_5fmanager_2fmfmplanned_5ftraj_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_MFMPlannedTraj_mf_5fmanager_2fmfmplanned_5ftraj_2eproto;
namespace pb {
namespace mf_manager {
namespace mfmplanned_traj {
class MFMPlannedTrajDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<MFMPlannedTraj> _instance;
} _MFMPlannedTraj_default_instance_;
class MFMPlannedTraj_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<MFMPlannedTraj_array_port> _instance;
} _MFMPlannedTraj_array_port_default_instance_;
}  // namespace mfmplanned_traj
}  // namespace mf_manager
}  // namespace pb
static void InitDefaultsscc_info_MFMPlannedTraj_mf_5fmanager_2fmfmplanned_5ftraj_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_manager::mfmplanned_traj::_MFMPlannedTraj_default_instance_;
    new (ptr) ::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_MFMPlannedTraj_mf_5fmanager_2fmfmplanned_5ftraj_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_MFMPlannedTraj_mf_5fmanager_2fmfmplanned_5ftraj_2eproto}, {}};

static void InitDefaultsscc_info_MFMPlannedTraj_array_port_mf_5fmanager_2fmfmplanned_5ftraj_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_manager::mfmplanned_traj::_MFMPlannedTraj_array_port_default_instance_;
    new (ptr) ::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_MFMPlannedTraj_array_port_mf_5fmanager_2fmfmplanned_5ftraj_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_MFMPlannedTraj_array_port_mf_5fmanager_2fmfmplanned_5ftraj_2eproto}, {
      &scc_info_MFMPlannedTraj_mf_5fmanager_2fmfmplanned_5ftraj_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_mf_5fmanager_2fmfmplanned_5ftraj_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_mf_5fmanager_2fmfmplanned_5ftraj_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5fmanager_2fmfmplanned_5ftraj_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5fmanager_2fmfmplanned_5ftraj_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj, xtrajrareq_m_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj, ytrajrareq_m_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj, yawreq_rad_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj, crvrareq_1pm_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj, distancetostopreq_m_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj, velocitylimitreq_mps_),
  3,
  0,
  1,
  2,
  5,
  4,
  PROTOBUF_FIELD_OFFSET(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 11, sizeof(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj)},
  { 17, 23, sizeof(::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_manager::mfmplanned_traj::_MFMPlannedTraj_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_manager::mfmplanned_traj::_MFMPlannedTraj_array_port_default_instance_),
};

const char descriptor_table_protodef_mf_5fmanager_2fmfmplanned_5ftraj_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n mf_manager/mfmplanned_traj.proto\022\035pb.m"
  "f_manager.mfmplanned_traj\"\247\001\n\016MFMPlanned"
  "Traj\022\025\n\014xTrajRAReq_m\030\221\016 \001(\002\022\025\n\014yTrajRARe"
  "q_m\030\344\010 \001(\002\022\023\n\nyawReq_rad\030\343\n \001(\002\022\025\n\014crvRA"
  "Req_1pm\030\264\r \001(\002\022\034\n\023distanceToStopReq_m\030\372\031"
  " \001(\002\022\035\n\024velocityLimitReq_mps\030\321\023 \001(\002\"Y\n\031M"
  "FMPlannedTraj_array_port\022<\n\004data\030\226\027 \003(\0132"
  "-.pb.mf_manager.mfmplanned_traj.MFMPlann"
  "edTraj"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5fmanager_2fmfmplanned_5ftraj_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5fmanager_2fmfmplanned_5ftraj_2eproto_sccs[2] = {
  &scc_info_MFMPlannedTraj_mf_5fmanager_2fmfmplanned_5ftraj_2eproto.base,
  &scc_info_MFMPlannedTraj_array_port_mf_5fmanager_2fmfmplanned_5ftraj_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5fmanager_2fmfmplanned_5ftraj_2eproto_once;
static bool descriptor_table_mf_5fmanager_2fmfmplanned_5ftraj_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmanager_2fmfmplanned_5ftraj_2eproto = {
  &descriptor_table_mf_5fmanager_2fmfmplanned_5ftraj_2eproto_initialized, descriptor_table_protodef_mf_5fmanager_2fmfmplanned_5ftraj_2eproto, "mf_manager/mfmplanned_traj.proto", 326,
  &descriptor_table_mf_5fmanager_2fmfmplanned_5ftraj_2eproto_once, descriptor_table_mf_5fmanager_2fmfmplanned_5ftraj_2eproto_sccs, descriptor_table_mf_5fmanager_2fmfmplanned_5ftraj_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_mf_5fmanager_2fmfmplanned_5ftraj_2eproto::offsets,
  file_level_metadata_mf_5fmanager_2fmfmplanned_5ftraj_2eproto, 2, file_level_enum_descriptors_mf_5fmanager_2fmfmplanned_5ftraj_2eproto, file_level_service_descriptors_mf_5fmanager_2fmfmplanned_5ftraj_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5fmanager_2fmfmplanned_5ftraj_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5fmanager_2fmfmplanned_5ftraj_2eproto), true);
namespace pb {
namespace mf_manager {
namespace mfmplanned_traj {

// ===================================================================

void MFMPlannedTraj::InitAsDefaultInstance() {
}
class MFMPlannedTraj::_Internal {
 public:
  using HasBits = decltype(std::declval<MFMPlannedTraj>()._has_bits_);
  static void set_has_xtrajrareq_m(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_ytrajrareq_m(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_yawreq_rad(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_crvrareq_1pm(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_distancetostopreq_m(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_velocitylimitreq_mps(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
};

MFMPlannedTraj::MFMPlannedTraj()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
}
MFMPlannedTraj::MFMPlannedTraj(const MFMPlannedTraj& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&ytrajrareq_m_, &from.ytrajrareq_m_,
    static_cast<size_t>(reinterpret_cast<char*>(&distancetostopreq_m_) -
    reinterpret_cast<char*>(&ytrajrareq_m_)) + sizeof(distancetostopreq_m_));
  // @@protoc_insertion_point(copy_constructor:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
}

void MFMPlannedTraj::SharedCtor() {
  ::memset(&ytrajrareq_m_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&distancetostopreq_m_) -
      reinterpret_cast<char*>(&ytrajrareq_m_)) + sizeof(distancetostopreq_m_));
}

MFMPlannedTraj::~MFMPlannedTraj() {
  // @@protoc_insertion_point(destructor:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
  SharedDtor();
}

void MFMPlannedTraj::SharedDtor() {
}

void MFMPlannedTraj::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const MFMPlannedTraj& MFMPlannedTraj::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_MFMPlannedTraj_mf_5fmanager_2fmfmplanned_5ftraj_2eproto.base);
  return *internal_default_instance();
}


void MFMPlannedTraj::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    ::memset(&ytrajrareq_m_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&distancetostopreq_m_) -
        reinterpret_cast<char*>(&ytrajrareq_m_)) + sizeof(distancetostopreq_m_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* MFMPlannedTraj::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional float yTrajRAReq_m = 1124;
      case 1124:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 37)) {
          _Internal::set_has_ytrajrareq_m(&has_bits);
          ytrajrareq_m_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional float yawReq_rad = 1379;
      case 1379:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 29)) {
          _Internal::set_has_yawreq_rad(&has_bits);
          yawreq_rad_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional float crvRAReq_1pm = 1716;
      case 1716:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 165)) {
          _Internal::set_has_crvrareq_1pm(&has_bits);
          crvrareq_1pm_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional float xTrajRAReq_m = 1809;
      case 1809:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 141)) {
          _Internal::set_has_xtrajrareq_m(&has_bits);
          xtrajrareq_m_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional float velocityLimitReq_mps = 2513;
      case 2513:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 141)) {
          _Internal::set_has_velocitylimitreq_mps(&has_bits);
          velocitylimitreq_mps_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional float distanceToStopReq_m = 3322;
      case 3322:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 213)) {
          _Internal::set_has_distancetostopreq_m(&has_bits);
          distancetostopreq_m_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
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

::PROTOBUF_NAMESPACE_ID::uint8* MFMPlannedTraj::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional float yTrajRAReq_m = 1124;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(1124, this->_internal_ytrajrareq_m(), target);
  }

  // optional float yawReq_rad = 1379;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(1379, this->_internal_yawreq_rad(), target);
  }

  // optional float crvRAReq_1pm = 1716;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(1716, this->_internal_crvrareq_1pm(), target);
  }

  // optional float xTrajRAReq_m = 1809;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(1809, this->_internal_xtrajrareq_m(), target);
  }

  // optional float velocityLimitReq_mps = 2513;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2513, this->_internal_velocitylimitreq_mps(), target);
  }

  // optional float distanceToStopReq_m = 3322;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(3322, this->_internal_distancetostopreq_m(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
  return target;
}

size_t MFMPlannedTraj::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    // optional float yTrajRAReq_m = 1124;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 + 4;
    }

    // optional float yawReq_rad = 1379;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 + 4;
    }

    // optional float crvRAReq_1pm = 1716;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 + 4;
    }

    // optional float xTrajRAReq_m = 1809;
    if (cached_has_bits & 0x00000008u) {
      total_size += 2 + 4;
    }

    // optional float velocityLimitReq_mps = 2513;
    if (cached_has_bits & 0x00000010u) {
      total_size += 3 + 4;
    }

    // optional float distanceToStopReq_m = 3322;
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

void MFMPlannedTraj::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
  GOOGLE_DCHECK_NE(&from, this);
  const MFMPlannedTraj* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<MFMPlannedTraj>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
    MergeFrom(*source);
  }
}

void MFMPlannedTraj::MergeFrom(const MFMPlannedTraj& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    if (cached_has_bits & 0x00000001u) {
      ytrajrareq_m_ = from.ytrajrareq_m_;
    }
    if (cached_has_bits & 0x00000002u) {
      yawreq_rad_ = from.yawreq_rad_;
    }
    if (cached_has_bits & 0x00000004u) {
      crvrareq_1pm_ = from.crvrareq_1pm_;
    }
    if (cached_has_bits & 0x00000008u) {
      xtrajrareq_m_ = from.xtrajrareq_m_;
    }
    if (cached_has_bits & 0x00000010u) {
      velocitylimitreq_mps_ = from.velocitylimitreq_mps_;
    }
    if (cached_has_bits & 0x00000020u) {
      distancetostopreq_m_ = from.distancetostopreq_m_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void MFMPlannedTraj::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void MFMPlannedTraj::CopyFrom(const MFMPlannedTraj& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MFMPlannedTraj::IsInitialized() const {
  return true;
}

void MFMPlannedTraj::InternalSwap(MFMPlannedTraj* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(ytrajrareq_m_, other->ytrajrareq_m_);
  swap(yawreq_rad_, other->yawreq_rad_);
  swap(crvrareq_1pm_, other->crvrareq_1pm_);
  swap(xtrajrareq_m_, other->xtrajrareq_m_);
  swap(velocitylimitreq_mps_, other->velocitylimitreq_mps_);
  swap(distancetostopreq_m_, other->distancetostopreq_m_);
}

::PROTOBUF_NAMESPACE_ID::Metadata MFMPlannedTraj::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void MFMPlannedTraj_array_port::InitAsDefaultInstance() {
}
class MFMPlannedTraj_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<MFMPlannedTraj_array_port>()._has_bits_);
};

MFMPlannedTraj_array_port::MFMPlannedTraj_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
}
MFMPlannedTraj_array_port::MFMPlannedTraj_array_port(const MFMPlannedTraj_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
}

void MFMPlannedTraj_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_MFMPlannedTraj_array_port_mf_5fmanager_2fmfmplanned_5ftraj_2eproto.base);
}

MFMPlannedTraj_array_port::~MFMPlannedTraj_array_port() {
  // @@protoc_insertion_point(destructor:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
  SharedDtor();
}

void MFMPlannedTraj_array_port::SharedDtor() {
}

void MFMPlannedTraj_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const MFMPlannedTraj_array_port& MFMPlannedTraj_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_MFMPlannedTraj_array_port_mf_5fmanager_2fmfmplanned_5ftraj_2eproto.base);
  return *internal_default_instance();
}


void MFMPlannedTraj_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* MFMPlannedTraj_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.mf_manager.mfmplanned_traj.MFMPlannedTraj data = 2966;
      case 2966:
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

::PROTOBUF_NAMESPACE_ID::uint8* MFMPlannedTraj_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.mf_manager.mfmplanned_traj.MFMPlannedTraj data = 2966;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2966, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
  return target;
}

size_t MFMPlannedTraj_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.mf_manager.mfmplanned_traj.MFMPlannedTraj data = 2966;
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

void MFMPlannedTraj_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const MFMPlannedTraj_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<MFMPlannedTraj_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
    MergeFrom(*source);
  }
}

void MFMPlannedTraj_array_port::MergeFrom(const MFMPlannedTraj_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void MFMPlannedTraj_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void MFMPlannedTraj_array_port::CopyFrom(const MFMPlannedTraj_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_manager.mfmplanned_traj.MFMPlannedTraj_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MFMPlannedTraj_array_port::IsInitialized() const {
  return true;
}

void MFMPlannedTraj_array_port::InternalSwap(MFMPlannedTraj_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata MFMPlannedTraj_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace mfmplanned_traj
}  // namespace mf_manager
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj* Arena::CreateMaybeMessage< ::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj_array_port* Arena::CreateMaybeMessage< ::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_manager::mfmplanned_traj::MFMPlannedTraj_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
