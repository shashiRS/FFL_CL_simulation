// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_tp/selected_pose_data.proto

#include "ap_tp/selected_pose_data.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_ap_5ftp_2fselected_5fpose_5fdata_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SelectedPoseData_ap_5ftp_2fselected_5fpose_5fdata_2eproto;
namespace pb {
namespace ap_tp {
namespace selected_pose_data {
class SelectedPoseDataDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<SelectedPoseData> _instance;
} _SelectedPoseData_default_instance_;
class SelectedPoseData_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<SelectedPoseData_array_port> _instance;
} _SelectedPoseData_array_port_default_instance_;
}  // namespace selected_pose_data
}  // namespace ap_tp
}  // namespace pb
static void InitDefaultsscc_info_SelectedPoseData_ap_5ftp_2fselected_5fpose_5fdata_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_tp::selected_pose_data::_SelectedPoseData_default_instance_;
    new (ptr) ::pb::ap_tp::selected_pose_data::SelectedPoseData();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_tp::selected_pose_data::SelectedPoseData::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SelectedPoseData_ap_5ftp_2fselected_5fpose_5fdata_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_SelectedPoseData_ap_5ftp_2fselected_5fpose_5fdata_2eproto}, {}};

static void InitDefaultsscc_info_SelectedPoseData_array_port_ap_5ftp_2fselected_5fpose_5fdata_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_tp::selected_pose_data::_SelectedPoseData_array_port_default_instance_;
    new (ptr) ::pb::ap_tp::selected_pose_data::SelectedPoseData_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_tp::selected_pose_data::SelectedPoseData_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_SelectedPoseData_array_port_ap_5ftp_2fselected_5fpose_5fdata_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_SelectedPoseData_array_port_ap_5ftp_2fselected_5fpose_5fdata_2eproto}, {
      &scc_info_SelectedPoseData_ap_5ftp_2fselected_5fpose_5fdata_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ap_5ftp_2fselected_5fpose_5fdata_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ap_5ftp_2fselected_5fpose_5fdata_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5ftp_2fselected_5fpose_5fdata_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5ftp_2fselected_5fpose_5fdata_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::selected_pose_data::SelectedPoseData, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::selected_pose_data::SelectedPoseData, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::selected_pose_data::SelectedPoseData, selectionstatus_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::selected_pose_data::SelectedPoseData, reachedstatus_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::selected_pose_data::SelectedPoseData, distancetostart_m_),
  2,
  1,
  0,
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::selected_pose_data::SelectedPoseData_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::selected_pose_data::SelectedPoseData_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::selected_pose_data::SelectedPoseData_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::pb::ap_tp::selected_pose_data::SelectedPoseData)},
  { 11, 17, sizeof(::pb::ap_tp::selected_pose_data::SelectedPoseData_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_tp::selected_pose_data::_SelectedPoseData_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_tp::selected_pose_data::_SelectedPoseData_array_port_default_instance_),
};

const char descriptor_table_protodef_ap_5ftp_2fselected_5fpose_5fdata_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\036ap_tp/selected_pose_data.proto\022\033pb.ap_"
  "tp.selected_pose_data\032!ap_tp/pose_select"
  "ion_status.proto\032\037ap_tp/pose_reached_sta"
  "tus.proto\"\306\001\n\020SelectedPoseData\022M\n\017select"
  "ionStatus\030\242\n \001(\01623.pb.ap_tp.pose_selecti"
  "on_status.PoseSelectionStatus\022G\n\rreached"
  "Status\030\223\007 \001(\0162/.pb.ap_tp.pose_reached_st"
  "atus.PoseReachedStatus\022\032\n\021distanceToStar"
  "t_m\030\252\024 \001(\002\"[\n\033SelectedPoseData_array_por"
  "t\022<\n\004data\030\361\005 \003(\0132-.pb.ap_tp.selected_pos"
  "e_data.SelectedPoseData"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5ftp_2fselected_5fpose_5fdata_2eproto_deps[2] = {
  &::descriptor_table_ap_5ftp_2fpose_5freached_5fstatus_2eproto,
  &::descriptor_table_ap_5ftp_2fpose_5fselection_5fstatus_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5ftp_2fselected_5fpose_5fdata_2eproto_sccs[2] = {
  &scc_info_SelectedPoseData_ap_5ftp_2fselected_5fpose_5fdata_2eproto.base,
  &scc_info_SelectedPoseData_array_port_ap_5ftp_2fselected_5fpose_5fdata_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5ftp_2fselected_5fpose_5fdata_2eproto_once;
static bool descriptor_table_ap_5ftp_2fselected_5fpose_5fdata_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftp_2fselected_5fpose_5fdata_2eproto = {
  &descriptor_table_ap_5ftp_2fselected_5fpose_5fdata_2eproto_initialized, descriptor_table_protodef_ap_5ftp_2fselected_5fpose_5fdata_2eproto, "ap_tp/selected_pose_data.proto", 423,
  &descriptor_table_ap_5ftp_2fselected_5fpose_5fdata_2eproto_once, descriptor_table_ap_5ftp_2fselected_5fpose_5fdata_2eproto_sccs, descriptor_table_ap_5ftp_2fselected_5fpose_5fdata_2eproto_deps, 2, 2,
  schemas, file_default_instances, TableStruct_ap_5ftp_2fselected_5fpose_5fdata_2eproto::offsets,
  file_level_metadata_ap_5ftp_2fselected_5fpose_5fdata_2eproto, 2, file_level_enum_descriptors_ap_5ftp_2fselected_5fpose_5fdata_2eproto, file_level_service_descriptors_ap_5ftp_2fselected_5fpose_5fdata_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5ftp_2fselected_5fpose_5fdata_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5ftp_2fselected_5fpose_5fdata_2eproto), true);
namespace pb {
namespace ap_tp {
namespace selected_pose_data {

// ===================================================================

void SelectedPoseData::InitAsDefaultInstance() {
}
class SelectedPoseData::_Internal {
 public:
  using HasBits = decltype(std::declval<SelectedPoseData>()._has_bits_);
  static void set_has_selectionstatus(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_reachedstatus(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_distancetostart_m(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

SelectedPoseData::SelectedPoseData()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_tp.selected_pose_data.SelectedPoseData)
}
SelectedPoseData::SelectedPoseData(const SelectedPoseData& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&distancetostart_m_, &from.distancetostart_m_,
    static_cast<size_t>(reinterpret_cast<char*>(&selectionstatus_) -
    reinterpret_cast<char*>(&distancetostart_m_)) + sizeof(selectionstatus_));
  // @@protoc_insertion_point(copy_constructor:pb.ap_tp.selected_pose_data.SelectedPoseData)
}

void SelectedPoseData::SharedCtor() {
  ::memset(&distancetostart_m_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&selectionstatus_) -
      reinterpret_cast<char*>(&distancetostart_m_)) + sizeof(selectionstatus_));
}

SelectedPoseData::~SelectedPoseData() {
  // @@protoc_insertion_point(destructor:pb.ap_tp.selected_pose_data.SelectedPoseData)
  SharedDtor();
}

void SelectedPoseData::SharedDtor() {
}

void SelectedPoseData::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const SelectedPoseData& SelectedPoseData::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_SelectedPoseData_ap_5ftp_2fselected_5fpose_5fdata_2eproto.base);
  return *internal_default_instance();
}


void SelectedPoseData::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_tp.selected_pose_data.SelectedPoseData)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    ::memset(&distancetostart_m_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&selectionstatus_) -
        reinterpret_cast<char*>(&distancetostart_m_)) + sizeof(selectionstatus_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* SelectedPoseData::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .pb.ap_tp.pose_reached_status.PoseReachedStatus reachedStatus = 915;
      case 915:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 152)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_tp::pose_reached_status::PoseReachedStatus_IsValid(val))) {
            _internal_set_reachedstatus(static_cast<::pb::ap_tp::pose_reached_status::PoseReachedStatus>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(915, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_tp.pose_selection_status.PoseSelectionStatus selectionStatus = 1314;
      case 1314:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_tp::pose_selection_status::PoseSelectionStatus_IsValid(val))) {
            _internal_set_selectionstatus(static_cast<::pb::ap_tp::pose_selection_status::PoseSelectionStatus>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(1314, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional float distanceToStart_m = 2602;
      case 2602:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 85)) {
          _Internal::set_has_distancetostart_m(&has_bits);
          distancetostart_m_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* SelectedPoseData::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_tp.selected_pose_data.SelectedPoseData)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.ap_tp.pose_reached_status.PoseReachedStatus reachedStatus = 915;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      915, this->_internal_reachedstatus(), target);
  }

  // optional .pb.ap_tp.pose_selection_status.PoseSelectionStatus selectionStatus = 1314;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      1314, this->_internal_selectionstatus(), target);
  }

  // optional float distanceToStart_m = 2602;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2602, this->_internal_distancetostart_m(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_tp.selected_pose_data.SelectedPoseData)
  return target;
}

size_t SelectedPoseData::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_tp.selected_pose_data.SelectedPoseData)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional float distanceToStart_m = 2602;
    if (cached_has_bits & 0x00000001u) {
      total_size += 3 + 4;
    }

    // optional .pb.ap_tp.pose_reached_status.PoseReachedStatus reachedStatus = 915;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_reachedstatus());
    }

    // optional .pb.ap_tp.pose_selection_status.PoseSelectionStatus selectionStatus = 1314;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_selectionstatus());
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

void SelectedPoseData::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_tp.selected_pose_data.SelectedPoseData)
  GOOGLE_DCHECK_NE(&from, this);
  const SelectedPoseData* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<SelectedPoseData>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_tp.selected_pose_data.SelectedPoseData)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_tp.selected_pose_data.SelectedPoseData)
    MergeFrom(*source);
  }
}

void SelectedPoseData::MergeFrom(const SelectedPoseData& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_tp.selected_pose_data.SelectedPoseData)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      distancetostart_m_ = from.distancetostart_m_;
    }
    if (cached_has_bits & 0x00000002u) {
      reachedstatus_ = from.reachedstatus_;
    }
    if (cached_has_bits & 0x00000004u) {
      selectionstatus_ = from.selectionstatus_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void SelectedPoseData::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_tp.selected_pose_data.SelectedPoseData)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SelectedPoseData::CopyFrom(const SelectedPoseData& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_tp.selected_pose_data.SelectedPoseData)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SelectedPoseData::IsInitialized() const {
  return true;
}

void SelectedPoseData::InternalSwap(SelectedPoseData* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(distancetostart_m_, other->distancetostart_m_);
  swap(reachedstatus_, other->reachedstatus_);
  swap(selectionstatus_, other->selectionstatus_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SelectedPoseData::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void SelectedPoseData_array_port::InitAsDefaultInstance() {
}
class SelectedPoseData_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<SelectedPoseData_array_port>()._has_bits_);
};

SelectedPoseData_array_port::SelectedPoseData_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
}
SelectedPoseData_array_port::SelectedPoseData_array_port(const SelectedPoseData_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
}

void SelectedPoseData_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_SelectedPoseData_array_port_ap_5ftp_2fselected_5fpose_5fdata_2eproto.base);
}

SelectedPoseData_array_port::~SelectedPoseData_array_port() {
  // @@protoc_insertion_point(destructor:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
  SharedDtor();
}

void SelectedPoseData_array_port::SharedDtor() {
}

void SelectedPoseData_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const SelectedPoseData_array_port& SelectedPoseData_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_SelectedPoseData_array_port_ap_5ftp_2fselected_5fpose_5fdata_2eproto.base);
  return *internal_default_instance();
}


void SelectedPoseData_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* SelectedPoseData_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.ap_tp.selected_pose_data.SelectedPoseData data = 753;
      case 753:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 138)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<6026>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* SelectedPoseData_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.ap_tp.selected_pose_data.SelectedPoseData data = 753;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(753, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
  return target;
}

size_t SelectedPoseData_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.ap_tp.selected_pose_data.SelectedPoseData data = 753;
  total_size += 2UL * this->_internal_data_size();
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

void SelectedPoseData_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const SelectedPoseData_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<SelectedPoseData_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
    MergeFrom(*source);
  }
}

void SelectedPoseData_array_port::MergeFrom(const SelectedPoseData_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void SelectedPoseData_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SelectedPoseData_array_port::CopyFrom(const SelectedPoseData_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_tp.selected_pose_data.SelectedPoseData_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SelectedPoseData_array_port::IsInitialized() const {
  return true;
}

void SelectedPoseData_array_port::InternalSwap(SelectedPoseData_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SelectedPoseData_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace selected_pose_data
}  // namespace ap_tp
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::ap_tp::selected_pose_data::SelectedPoseData* Arena::CreateMaybeMessage< ::pb::ap_tp::selected_pose_data::SelectedPoseData >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_tp::selected_pose_data::SelectedPoseData >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::ap_tp::selected_pose_data::SelectedPoseData_array_port* Arena::CreateMaybeMessage< ::pb::ap_tp::selected_pose_data::SelectedPoseData_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_tp::selected_pose_data::SelectedPoseData_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
