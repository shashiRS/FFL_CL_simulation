// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_mempark/redetection_request.proto

#include "mf_mempark/redetection_request.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_lsm_5fgeoml_2fpose_5fpod_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Pose_POD_lsm_5fgeoml_2fpose_5fpod_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_mf_5fmempark_2fredetection_5frequest_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_RedetectionRequest_mf_5fmempark_2fredetection_5frequest_2eproto;
namespace pb {
namespace mf_mempark {
namespace redetection_request {
class RedetectionRequestDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<RedetectionRequest> _instance;
} _RedetectionRequest_default_instance_;
class RedetectionRequest_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<RedetectionRequest_array_port> _instance;
} _RedetectionRequest_array_port_default_instance_;
}  // namespace redetection_request
}  // namespace mf_mempark
}  // namespace pb
static void InitDefaultsscc_info_RedetectionRequest_mf_5fmempark_2fredetection_5frequest_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_mempark::redetection_request::_RedetectionRequest_default_instance_;
    new (ptr) ::pb::mf_mempark::redetection_request::RedetectionRequest();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_mempark::redetection_request::RedetectionRequest::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_RedetectionRequest_mf_5fmempark_2fredetection_5frequest_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_RedetectionRequest_mf_5fmempark_2fredetection_5frequest_2eproto}, {
      &scc_info_Pose_POD_lsm_5fgeoml_2fpose_5fpod_2eproto.base,}};

static void InitDefaultsscc_info_RedetectionRequest_array_port_mf_5fmempark_2fredetection_5frequest_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_mempark::redetection_request::_RedetectionRequest_array_port_default_instance_;
    new (ptr) ::pb::mf_mempark::redetection_request::RedetectionRequest_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_mempark::redetection_request::RedetectionRequest_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_RedetectionRequest_array_port_mf_5fmempark_2fredetection_5frequest_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_RedetectionRequest_array_port_mf_5fmempark_2fredetection_5frequest_2eproto}, {
      &scc_info_RedetectionRequest_mf_5fmempark_2fredetection_5frequest_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_mf_5fmempark_2fredetection_5frequest_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_mf_5fmempark_2fredetection_5frequest_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5fmempark_2fredetection_5frequest_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5fmempark_2fredetection_5frequest_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::redetection_request::RedetectionRequest, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::redetection_request::RedetectionRequest, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::redetection_request::RedetectionRequest, mapid_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::redetection_request::RedetectionRequest, startpose_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::redetection_request::RedetectionRequest, initiallocalizationrequest_),
  1,
  0,
  2,
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::redetection_request::RedetectionRequest_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::redetection_request::RedetectionRequest_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::redetection_request::RedetectionRequest_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::pb::mf_mempark::redetection_request::RedetectionRequest)},
  { 11, 17, sizeof(::pb::mf_mempark::redetection_request::RedetectionRequest_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_mempark::redetection_request::_RedetectionRequest_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_mempark::redetection_request::_RedetectionRequest_array_port_default_instance_),
};

const char descriptor_table_protodef_mf_5fmempark_2fredetection_5frequest_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n$mf_mempark/redetection_request.proto\022!"
  "pb.mf_mempark.redetection_request\032\030lsm_g"
  "eoml/pose_pod.proto\"~\n\022RedetectionReques"
  "t\022\016\n\005mapID\030\330\r \001(\r\0223\n\tstartPose\030\316\031 \001(\0132\037."
  "pb.lsm_geoml.pose_pod.Pose_POD\022#\n\032initia"
  "lLocalizationRequest\030\321\032 \001(\010\"e\n\035Redetecti"
  "onRequest_array_port\022D\n\004data\030\220\013 \003(\01325.pb"
  ".mf_mempark.redetection_request.Redetect"
  "ionRequest"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto_deps[1] = {
  &::descriptor_table_lsm_5fgeoml_2fpose_5fpod_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto_sccs[2] = {
  &scc_info_RedetectionRequest_mf_5fmempark_2fredetection_5frequest_2eproto.base,
  &scc_info_RedetectionRequest_array_port_mf_5fmempark_2fredetection_5frequest_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto_once;
static bool descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto = {
  &descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto_initialized, descriptor_table_protodef_mf_5fmempark_2fredetection_5frequest_2eproto, "mf_mempark/redetection_request.proto", 330,
  &descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto_once, descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto_sccs, descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_mf_5fmempark_2fredetection_5frequest_2eproto::offsets,
  file_level_metadata_mf_5fmempark_2fredetection_5frequest_2eproto, 2, file_level_enum_descriptors_mf_5fmempark_2fredetection_5frequest_2eproto, file_level_service_descriptors_mf_5fmempark_2fredetection_5frequest_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5fmempark_2fredetection_5frequest_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5fmempark_2fredetection_5frequest_2eproto), true);
namespace pb {
namespace mf_mempark {
namespace redetection_request {

// ===================================================================

void RedetectionRequest::InitAsDefaultInstance() {
  ::pb::mf_mempark::redetection_request::_RedetectionRequest_default_instance_._instance.get_mutable()->startpose_ = const_cast< ::pb::lsm_geoml::pose_pod::Pose_POD*>(
      ::pb::lsm_geoml::pose_pod::Pose_POD::internal_default_instance());
}
class RedetectionRequest::_Internal {
 public:
  using HasBits = decltype(std::declval<RedetectionRequest>()._has_bits_);
  static void set_has_mapid(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::pb::lsm_geoml::pose_pod::Pose_POD& startpose(const RedetectionRequest* msg);
  static void set_has_startpose(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_initiallocalizationrequest(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::pb::lsm_geoml::pose_pod::Pose_POD&
RedetectionRequest::_Internal::startpose(const RedetectionRequest* msg) {
  return *msg->startpose_;
}
void RedetectionRequest::clear_startpose() {
  if (startpose_ != nullptr) startpose_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
RedetectionRequest::RedetectionRequest()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_mempark.redetection_request.RedetectionRequest)
}
RedetectionRequest::RedetectionRequest(const RedetectionRequest& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_startpose()) {
    startpose_ = new ::pb::lsm_geoml::pose_pod::Pose_POD(*from.startpose_);
  } else {
    startpose_ = nullptr;
  }
  ::memcpy(&mapid_, &from.mapid_,
    static_cast<size_t>(reinterpret_cast<char*>(&initiallocalizationrequest_) -
    reinterpret_cast<char*>(&mapid_)) + sizeof(initiallocalizationrequest_));
  // @@protoc_insertion_point(copy_constructor:pb.mf_mempark.redetection_request.RedetectionRequest)
}

void RedetectionRequest::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_RedetectionRequest_mf_5fmempark_2fredetection_5frequest_2eproto.base);
  ::memset(&startpose_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&initiallocalizationrequest_) -
      reinterpret_cast<char*>(&startpose_)) + sizeof(initiallocalizationrequest_));
}

RedetectionRequest::~RedetectionRequest() {
  // @@protoc_insertion_point(destructor:pb.mf_mempark.redetection_request.RedetectionRequest)
  SharedDtor();
}

void RedetectionRequest::SharedDtor() {
  if (this != internal_default_instance()) delete startpose_;
}

void RedetectionRequest::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const RedetectionRequest& RedetectionRequest::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_RedetectionRequest_mf_5fmempark_2fredetection_5frequest_2eproto.base);
  return *internal_default_instance();
}


void RedetectionRequest::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_mempark.redetection_request.RedetectionRequest)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(startpose_ != nullptr);
    startpose_->Clear();
  }
  if (cached_has_bits & 0x00000006u) {
    ::memset(&mapid_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&initiallocalizationrequest_) -
        reinterpret_cast<char*>(&mapid_)) + sizeof(initiallocalizationrequest_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* RedetectionRequest::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional uint32 mapID = 1752;
      case 1752:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 192)) {
          _Internal::set_has_mapid(&has_bits);
          mapid_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.lsm_geoml.pose_pod.Pose_POD startPose = 3278;
      case 3278:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 114)) {
          ptr = ctx->ParseMessage(_internal_mutable_startpose(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool initialLocalizationRequest = 3409;
      case 3409:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 136)) {
          _Internal::set_has_initiallocalizationrequest(&has_bits);
          initiallocalizationrequest_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* RedetectionRequest::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_mempark.redetection_request.RedetectionRequest)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional uint32 mapID = 1752;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1752, this->_internal_mapid(), target);
  }

  // optional .pb.lsm_geoml.pose_pod.Pose_POD startPose = 3278;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3278, _Internal::startpose(this), target, stream);
  }

  // optional bool initialLocalizationRequest = 3409;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3409, this->_internal_initiallocalizationrequest(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_mempark.redetection_request.RedetectionRequest)
  return target;
}

size_t RedetectionRequest::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_mempark.redetection_request.RedetectionRequest)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional .pb.lsm_geoml.pose_pod.Pose_POD startPose = 3278;
    if (cached_has_bits & 0x00000001u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *startpose_);
    }

    // optional uint32 mapID = 1752;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_mapid());
    }

    // optional bool initialLocalizationRequest = 3409;
    if (cached_has_bits & 0x00000004u) {
      total_size += 3 + 1;
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

void RedetectionRequest::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_mempark.redetection_request.RedetectionRequest)
  GOOGLE_DCHECK_NE(&from, this);
  const RedetectionRequest* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<RedetectionRequest>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_mempark.redetection_request.RedetectionRequest)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_mempark.redetection_request.RedetectionRequest)
    MergeFrom(*source);
  }
}

void RedetectionRequest::MergeFrom(const RedetectionRequest& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_mempark.redetection_request.RedetectionRequest)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_startpose()->::pb::lsm_geoml::pose_pod::Pose_POD::MergeFrom(from._internal_startpose());
    }
    if (cached_has_bits & 0x00000002u) {
      mapid_ = from.mapid_;
    }
    if (cached_has_bits & 0x00000004u) {
      initiallocalizationrequest_ = from.initiallocalizationrequest_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void RedetectionRequest::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_mempark.redetection_request.RedetectionRequest)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void RedetectionRequest::CopyFrom(const RedetectionRequest& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_mempark.redetection_request.RedetectionRequest)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RedetectionRequest::IsInitialized() const {
  return true;
}

void RedetectionRequest::InternalSwap(RedetectionRequest* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(startpose_, other->startpose_);
  swap(mapid_, other->mapid_);
  swap(initiallocalizationrequest_, other->initiallocalizationrequest_);
}

::PROTOBUF_NAMESPACE_ID::Metadata RedetectionRequest::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void RedetectionRequest_array_port::InitAsDefaultInstance() {
}
class RedetectionRequest_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<RedetectionRequest_array_port>()._has_bits_);
};

RedetectionRequest_array_port::RedetectionRequest_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
}
RedetectionRequest_array_port::RedetectionRequest_array_port(const RedetectionRequest_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
}

void RedetectionRequest_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_RedetectionRequest_array_port_mf_5fmempark_2fredetection_5frequest_2eproto.base);
}

RedetectionRequest_array_port::~RedetectionRequest_array_port() {
  // @@protoc_insertion_point(destructor:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
  SharedDtor();
}

void RedetectionRequest_array_port::SharedDtor() {
}

void RedetectionRequest_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const RedetectionRequest_array_port& RedetectionRequest_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_RedetectionRequest_array_port_mf_5fmempark_2fredetection_5frequest_2eproto.base);
  return *internal_default_instance();
}


void RedetectionRequest_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* RedetectionRequest_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.mf_mempark.redetection_request.RedetectionRequest data = 1424;
      case 1424:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 130)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<11394>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* RedetectionRequest_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.mf_mempark.redetection_request.RedetectionRequest data = 1424;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1424, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
  return target;
}

size_t RedetectionRequest_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.mf_mempark.redetection_request.RedetectionRequest data = 1424;
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

void RedetectionRequest_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const RedetectionRequest_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<RedetectionRequest_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
    MergeFrom(*source);
  }
}

void RedetectionRequest_array_port::MergeFrom(const RedetectionRequest_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void RedetectionRequest_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void RedetectionRequest_array_port::CopyFrom(const RedetectionRequest_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_mempark.redetection_request.RedetectionRequest_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RedetectionRequest_array_port::IsInitialized() const {
  return true;
}

void RedetectionRequest_array_port::InternalSwap(RedetectionRequest_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata RedetectionRequest_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace redetection_request
}  // namespace mf_mempark
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::mf_mempark::redetection_request::RedetectionRequest* Arena::CreateMaybeMessage< ::pb::mf_mempark::redetection_request::RedetectionRequest >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_mempark::redetection_request::RedetectionRequest >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::mf_mempark::redetection_request::RedetectionRequest_array_port* Arena::CreateMaybeMessage< ::pb::mf_mempark::redetection_request::RedetectionRequest_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_mempark::redetection_request::RedetectionRequest_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
