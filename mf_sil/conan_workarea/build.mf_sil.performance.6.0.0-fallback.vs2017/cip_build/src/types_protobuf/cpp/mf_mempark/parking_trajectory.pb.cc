// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_mempark/parking_trajectory.proto

#include "mf_mempark/parking_trajectory.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_eco_2fsignal_5fheader_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_lsm_5fgeoml_2fpose_5fpod_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Pose_POD_lsm_5fgeoml_2fpose_5fpod_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_mf_5fmempark_2fparking_5ftrajectory_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<4> scc_info_ParkingTrajectory_mf_5fmempark_2fparking_5ftrajectory_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_mf_5fmempark_2ftrajectory_5fmeta_5fdata_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_TrajectoryMetaData_mf_5fmempark_2ftrajectory_5fmeta_5fdata_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_mf_5fmempark_2ftrajectory_5fpoint_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_TrajectoryPoint_mf_5fmempark_2ftrajectory_5fpoint_2eproto;
namespace pb {
namespace mf_mempark {
namespace parking_trajectory {
class ParkingTrajectoryDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ParkingTrajectory> _instance;
} _ParkingTrajectory_default_instance_;
class ParkingTrajectory_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ParkingTrajectory_array_port> _instance;
} _ParkingTrajectory_array_port_default_instance_;
}  // namespace parking_trajectory
}  // namespace mf_mempark
}  // namespace pb
static void InitDefaultsscc_info_ParkingTrajectory_mf_5fmempark_2fparking_5ftrajectory_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_mempark::parking_trajectory::_ParkingTrajectory_default_instance_;
    new (ptr) ::pb::mf_mempark::parking_trajectory::ParkingTrajectory();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_mempark::parking_trajectory::ParkingTrajectory::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<4> scc_info_ParkingTrajectory_mf_5fmempark_2fparking_5ftrajectory_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 4, 0, InitDefaultsscc_info_ParkingTrajectory_mf_5fmempark_2fparking_5ftrajectory_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,
      &scc_info_Pose_POD_lsm_5fgeoml_2fpose_5fpod_2eproto.base,
      &scc_info_TrajectoryPoint_mf_5fmempark_2ftrajectory_5fpoint_2eproto.base,
      &scc_info_TrajectoryMetaData_mf_5fmempark_2ftrajectory_5fmeta_5fdata_2eproto.base,}};

static void InitDefaultsscc_info_ParkingTrajectory_array_port_mf_5fmempark_2fparking_5ftrajectory_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_mempark::parking_trajectory::_ParkingTrajectory_array_port_default_instance_;
    new (ptr) ::pb::mf_mempark::parking_trajectory::ParkingTrajectory_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_mempark::parking_trajectory::ParkingTrajectory_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ParkingTrajectory_array_port_mf_5fmempark_2fparking_5ftrajectory_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_ParkingTrajectory_array_port_mf_5fmempark_2fparking_5ftrajectory_2eproto}, {
      &scc_info_ParkingTrajectory_mf_5fmempark_2fparking_5ftrajectory_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_mf_5fmempark_2fparking_5ftrajectory_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_mf_5fmempark_2fparking_5ftrajectory_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5fmempark_2fparking_5ftrajectory_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5fmempark_2fparking_5ftrajectory_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory, uiversionnumber_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory, ssigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory, trajectoryid_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory, startpose_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory, endpose_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory, numvalidtrajpoints_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory, listofpoints_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory, slotid_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory, metadata_),
  6,
  0,
  4,
  3,
  1,
  7,
  ~0u,
  5,
  2,
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_mempark::parking_trajectory::ParkingTrajectory_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 14, sizeof(::pb::mf_mempark::parking_trajectory::ParkingTrajectory)},
  { 23, 29, sizeof(::pb::mf_mempark::parking_trajectory::ParkingTrajectory_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_mempark::parking_trajectory::_ParkingTrajectory_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_mempark::parking_trajectory::_ParkingTrajectory_array_port_default_instance_),
};

const char descriptor_table_protodef_mf_5fmempark_2fparking_5ftrajectory_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n#mf_mempark/parking_trajectory.proto\022 p"
  "b.mf_mempark.parking_trajectory\032\027eco/sig"
  "nal_header.proto\032\030lsm_geoml/pose_pod.pro"
  "to\032!mf_mempark/trajectory_point.proto\032%m"
  "f_mempark/trajectory_meta_data.proto\"\245\003\n"
  "\021ParkingTrajectory\022\030\n\017uiVersionNumber\030\314\020"
  " \001(\r\0227\n\nsSigHeader\030\211\010 \001(\0132\".pb.eco.signa"
  "l_header.SignalHeader\022\024\n\014trajectoryID\030V "
  "\001(\r\0223\n\tstartPose\030\316\031 \001(\0132\037.pb.lsm_geoml.p"
  "ose_pod.Pose_POD\0221\n\007endPose\030\362\024 \001(\0132\037.pb."
  "lsm_geoml.pose_pod.Pose_POD\022\033\n\022numValidT"
  "rajPoints\030\334\031 \001(\r\022F\n\014listOfPoints\030\362\032 \003(\0132"
  "/.pb.mf_mempark.trajectory_point.Traject"
  "oryPoint\022\017\n\006slotID\030\320\001 \001(\r\022I\n\010metaData\030\210\030"
  " \001(\01326.pb.mf_mempark.trajectory_meta_dat"
  "a.TrajectoryMetaData\"b\n\034ParkingTrajector"
  "y_array_port\022B\n\004data\030\260\010 \003(\01323.pb.mf_memp"
  "ark.parking_trajectory.ParkingTrajectory"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5fmempark_2fparking_5ftrajectory_2eproto_deps[4] = {
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
  &::descriptor_table_lsm_5fgeoml_2fpose_5fpod_2eproto,
  &::descriptor_table_mf_5fmempark_2ftrajectory_5fmeta_5fdata_2eproto,
  &::descriptor_table_mf_5fmempark_2ftrajectory_5fpoint_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5fmempark_2fparking_5ftrajectory_2eproto_sccs[2] = {
  &scc_info_ParkingTrajectory_mf_5fmempark_2fparking_5ftrajectory_2eproto.base,
  &scc_info_ParkingTrajectory_array_port_mf_5fmempark_2fparking_5ftrajectory_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5fmempark_2fparking_5ftrajectory_2eproto_once;
static bool descriptor_table_mf_5fmempark_2fparking_5ftrajectory_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5fmempark_2fparking_5ftrajectory_2eproto = {
  &descriptor_table_mf_5fmempark_2fparking_5ftrajectory_2eproto_initialized, descriptor_table_protodef_mf_5fmempark_2fparking_5ftrajectory_2eproto, "mf_mempark/parking_trajectory.proto", 720,
  &descriptor_table_mf_5fmempark_2fparking_5ftrajectory_2eproto_once, descriptor_table_mf_5fmempark_2fparking_5ftrajectory_2eproto_sccs, descriptor_table_mf_5fmempark_2fparking_5ftrajectory_2eproto_deps, 2, 4,
  schemas, file_default_instances, TableStruct_mf_5fmempark_2fparking_5ftrajectory_2eproto::offsets,
  file_level_metadata_mf_5fmempark_2fparking_5ftrajectory_2eproto, 2, file_level_enum_descriptors_mf_5fmempark_2fparking_5ftrajectory_2eproto, file_level_service_descriptors_mf_5fmempark_2fparking_5ftrajectory_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5fmempark_2fparking_5ftrajectory_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5fmempark_2fparking_5ftrajectory_2eproto), true);
namespace pb {
namespace mf_mempark {
namespace parking_trajectory {

// ===================================================================

void ParkingTrajectory::InitAsDefaultInstance() {
  ::pb::mf_mempark::parking_trajectory::_ParkingTrajectory_default_instance_._instance.get_mutable()->ssigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
  ::pb::mf_mempark::parking_trajectory::_ParkingTrajectory_default_instance_._instance.get_mutable()->startpose_ = const_cast< ::pb::lsm_geoml::pose_pod::Pose_POD*>(
      ::pb::lsm_geoml::pose_pod::Pose_POD::internal_default_instance());
  ::pb::mf_mempark::parking_trajectory::_ParkingTrajectory_default_instance_._instance.get_mutable()->endpose_ = const_cast< ::pb::lsm_geoml::pose_pod::Pose_POD*>(
      ::pb::lsm_geoml::pose_pod::Pose_POD::internal_default_instance());
  ::pb::mf_mempark::parking_trajectory::_ParkingTrajectory_default_instance_._instance.get_mutable()->metadata_ = const_cast< ::pb::mf_mempark::trajectory_meta_data::TrajectoryMetaData*>(
      ::pb::mf_mempark::trajectory_meta_data::TrajectoryMetaData::internal_default_instance());
}
class ParkingTrajectory::_Internal {
 public:
  using HasBits = decltype(std::declval<ParkingTrajectory>()._has_bits_);
  static void set_has_uiversionnumber(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static const ::pb::eco::signal_header::SignalHeader& ssigheader(const ParkingTrajectory* msg);
  static void set_has_ssigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_trajectoryid(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static const ::pb::lsm_geoml::pose_pod::Pose_POD& startpose(const ParkingTrajectory* msg);
  static void set_has_startpose(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static const ::pb::lsm_geoml::pose_pod::Pose_POD& endpose(const ParkingTrajectory* msg);
  static void set_has_endpose(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_numvalidtrajpoints(HasBits* has_bits) {
    (*has_bits)[0] |= 128u;
  }
  static void set_has_slotid(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static const ::pb::mf_mempark::trajectory_meta_data::TrajectoryMetaData& metadata(const ParkingTrajectory* msg);
  static void set_has_metadata(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
ParkingTrajectory::_Internal::ssigheader(const ParkingTrajectory* msg) {
  return *msg->ssigheader_;
}
const ::pb::lsm_geoml::pose_pod::Pose_POD&
ParkingTrajectory::_Internal::startpose(const ParkingTrajectory* msg) {
  return *msg->startpose_;
}
const ::pb::lsm_geoml::pose_pod::Pose_POD&
ParkingTrajectory::_Internal::endpose(const ParkingTrajectory* msg) {
  return *msg->endpose_;
}
const ::pb::mf_mempark::trajectory_meta_data::TrajectoryMetaData&
ParkingTrajectory::_Internal::metadata(const ParkingTrajectory* msg) {
  return *msg->metadata_;
}
void ParkingTrajectory::clear_ssigheader() {
  if (ssigheader_ != nullptr) ssigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void ParkingTrajectory::clear_startpose() {
  if (startpose_ != nullptr) startpose_->Clear();
  _has_bits_[0] &= ~0x00000008u;
}
void ParkingTrajectory::clear_endpose() {
  if (endpose_ != nullptr) endpose_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
void ParkingTrajectory::clear_listofpoints() {
  listofpoints_.Clear();
}
void ParkingTrajectory::clear_metadata() {
  if (metadata_ != nullptr) metadata_->Clear();
  _has_bits_[0] &= ~0x00000004u;
}
ParkingTrajectory::ParkingTrajectory()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
}
ParkingTrajectory::ParkingTrajectory(const ParkingTrajectory& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      listofpoints_(from.listofpoints_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_ssigheader()) {
    ssigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.ssigheader_);
  } else {
    ssigheader_ = nullptr;
  }
  if (from._internal_has_endpose()) {
    endpose_ = new ::pb::lsm_geoml::pose_pod::Pose_POD(*from.endpose_);
  } else {
    endpose_ = nullptr;
  }
  if (from._internal_has_metadata()) {
    metadata_ = new ::pb::mf_mempark::trajectory_meta_data::TrajectoryMetaData(*from.metadata_);
  } else {
    metadata_ = nullptr;
  }
  if (from._internal_has_startpose()) {
    startpose_ = new ::pb::lsm_geoml::pose_pod::Pose_POD(*from.startpose_);
  } else {
    startpose_ = nullptr;
  }
  ::memcpy(&trajectoryid_, &from.trajectoryid_,
    static_cast<size_t>(reinterpret_cast<char*>(&numvalidtrajpoints_) -
    reinterpret_cast<char*>(&trajectoryid_)) + sizeof(numvalidtrajpoints_));
  // @@protoc_insertion_point(copy_constructor:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
}

void ParkingTrajectory::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ParkingTrajectory_mf_5fmempark_2fparking_5ftrajectory_2eproto.base);
  ::memset(&ssigheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&numvalidtrajpoints_) -
      reinterpret_cast<char*>(&ssigheader_)) + sizeof(numvalidtrajpoints_));
}

ParkingTrajectory::~ParkingTrajectory() {
  // @@protoc_insertion_point(destructor:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
  SharedDtor();
}

void ParkingTrajectory::SharedDtor() {
  if (this != internal_default_instance()) delete ssigheader_;
  if (this != internal_default_instance()) delete endpose_;
  if (this != internal_default_instance()) delete metadata_;
  if (this != internal_default_instance()) delete startpose_;
}

void ParkingTrajectory::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ParkingTrajectory& ParkingTrajectory::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ParkingTrajectory_mf_5fmempark_2fparking_5ftrajectory_2eproto.base);
  return *internal_default_instance();
}


void ParkingTrajectory::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  listofpoints_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(ssigheader_ != nullptr);
      ssigheader_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(endpose_ != nullptr);
      endpose_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(metadata_ != nullptr);
      metadata_->Clear();
    }
    if (cached_has_bits & 0x00000008u) {
      GOOGLE_DCHECK(startpose_ != nullptr);
      startpose_->Clear();
    }
  }
  if (cached_has_bits & 0x000000f0u) {
    ::memset(&trajectoryid_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&numvalidtrajpoints_) -
        reinterpret_cast<char*>(&trajectoryid_)) + sizeof(numvalidtrajpoints_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ParkingTrajectory::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional uint32 trajectoryID = 86;
      case 86:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 176)) {
          _Internal::set_has_trajectoryid(&has_bits);
          trajectoryid_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 slotID = 208;
      case 208:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 128)) {
          _Internal::set_has_slotid(&has_bits);
          slotid_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
      case 1033:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 74)) {
          ptr = ctx->ParseMessage(_internal_mutable_ssigheader(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 uiVersionNumber = 2124;
      case 2124:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 96)) {
          _Internal::set_has_uiversionnumber(&has_bits);
          uiversionnumber_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.lsm_geoml.pose_pod.Pose_POD endPose = 2674;
      case 2674:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 146)) {
          ptr = ctx->ParseMessage(_internal_mutable_endpose(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.mf_mempark.trajectory_meta_data.TrajectoryMetaData metaData = 3080;
      case 3080:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 66)) {
          ptr = ctx->ParseMessage(_internal_mutable_metadata(), ptr);
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
      // optional uint32 numValidTrajPoints = 3292;
      case 3292:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 224)) {
          _Internal::set_has_numvalidtrajpoints(&has_bits);
          numvalidtrajpoints_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .pb.mf_mempark.trajectory_point.TrajectoryPoint listOfPoints = 3442;
      case 3442:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 146)) {
          ptr = ctx->ParseMessage(_internal_add_listofpoints(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* ParkingTrajectory::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional uint32 trajectoryID = 86;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(86, this->_internal_trajectoryid(), target);
  }

  // optional uint32 slotID = 208;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(208, this->_internal_slotid(), target);
  }

  // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1033, _Internal::ssigheader(this), target, stream);
  }

  // optional uint32 uiVersionNumber = 2124;
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2124, this->_internal_uiversionnumber(), target);
  }

  // optional .pb.lsm_geoml.pose_pod.Pose_POD endPose = 2674;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2674, _Internal::endpose(this), target, stream);
  }

  // optional .pb.mf_mempark.trajectory_meta_data.TrajectoryMetaData metaData = 3080;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3080, _Internal::metadata(this), target, stream);
  }

  // optional .pb.lsm_geoml.pose_pod.Pose_POD startPose = 3278;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3278, _Internal::startpose(this), target, stream);
  }

  // optional uint32 numValidTrajPoints = 3292;
  if (cached_has_bits & 0x00000080u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(3292, this->_internal_numvalidtrajpoints(), target);
  }

  // repeated .pb.mf_mempark.trajectory_point.TrajectoryPoint listOfPoints = 3442;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_listofpoints_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3442, this->_internal_listofpoints(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
  return target;
}

size_t ParkingTrajectory::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.mf_mempark.trajectory_point.TrajectoryPoint listOfPoints = 3442;
  total_size += 3UL * this->_internal_listofpoints_size();
  for (const auto& msg : this->listofpoints_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *ssigheader_);
    }

    // optional .pb.lsm_geoml.pose_pod.Pose_POD endPose = 2674;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *endpose_);
    }

    // optional .pb.mf_mempark.trajectory_meta_data.TrajectoryMetaData metaData = 3080;
    if (cached_has_bits & 0x00000004u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *metadata_);
    }

    // optional .pb.lsm_geoml.pose_pod.Pose_POD startPose = 3278;
    if (cached_has_bits & 0x00000008u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *startpose_);
    }

    // optional uint32 trajectoryID = 86;
    if (cached_has_bits & 0x00000010u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_trajectoryid());
    }

    // optional uint32 slotID = 208;
    if (cached_has_bits & 0x00000020u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_slotid());
    }

    // optional uint32 uiVersionNumber = 2124;
    if (cached_has_bits & 0x00000040u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_uiversionnumber());
    }

    // optional uint32 numValidTrajPoints = 3292;
    if (cached_has_bits & 0x00000080u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_numvalidtrajpoints());
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

void ParkingTrajectory::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
  GOOGLE_DCHECK_NE(&from, this);
  const ParkingTrajectory* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ParkingTrajectory>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
    MergeFrom(*source);
  }
}

void ParkingTrajectory::MergeFrom(const ParkingTrajectory& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  listofpoints_.MergeFrom(from.listofpoints_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_ssigheader()->::pb::eco::signal_header::SignalHeader::MergeFrom(from._internal_ssigheader());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_endpose()->::pb::lsm_geoml::pose_pod::Pose_POD::MergeFrom(from._internal_endpose());
    }
    if (cached_has_bits & 0x00000004u) {
      _internal_mutable_metadata()->::pb::mf_mempark::trajectory_meta_data::TrajectoryMetaData::MergeFrom(from._internal_metadata());
    }
    if (cached_has_bits & 0x00000008u) {
      _internal_mutable_startpose()->::pb::lsm_geoml::pose_pod::Pose_POD::MergeFrom(from._internal_startpose());
    }
    if (cached_has_bits & 0x00000010u) {
      trajectoryid_ = from.trajectoryid_;
    }
    if (cached_has_bits & 0x00000020u) {
      slotid_ = from.slotid_;
    }
    if (cached_has_bits & 0x00000040u) {
      uiversionnumber_ = from.uiversionnumber_;
    }
    if (cached_has_bits & 0x00000080u) {
      numvalidtrajpoints_ = from.numvalidtrajpoints_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ParkingTrajectory::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ParkingTrajectory::CopyFrom(const ParkingTrajectory& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ParkingTrajectory::IsInitialized() const {
  return true;
}

void ParkingTrajectory::InternalSwap(ParkingTrajectory* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  listofpoints_.InternalSwap(&other->listofpoints_);
  swap(ssigheader_, other->ssigheader_);
  swap(endpose_, other->endpose_);
  swap(metadata_, other->metadata_);
  swap(startpose_, other->startpose_);
  swap(trajectoryid_, other->trajectoryid_);
  swap(slotid_, other->slotid_);
  swap(uiversionnumber_, other->uiversionnumber_);
  swap(numvalidtrajpoints_, other->numvalidtrajpoints_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ParkingTrajectory::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void ParkingTrajectory_array_port::InitAsDefaultInstance() {
}
class ParkingTrajectory_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<ParkingTrajectory_array_port>()._has_bits_);
};

ParkingTrajectory_array_port::ParkingTrajectory_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
}
ParkingTrajectory_array_port::ParkingTrajectory_array_port(const ParkingTrajectory_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
}

void ParkingTrajectory_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ParkingTrajectory_array_port_mf_5fmempark_2fparking_5ftrajectory_2eproto.base);
}

ParkingTrajectory_array_port::~ParkingTrajectory_array_port() {
  // @@protoc_insertion_point(destructor:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
  SharedDtor();
}

void ParkingTrajectory_array_port::SharedDtor() {
}

void ParkingTrajectory_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ParkingTrajectory_array_port& ParkingTrajectory_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ParkingTrajectory_array_port_mf_5fmempark_2fparking_5ftrajectory_2eproto.base);
  return *internal_default_instance();
}


void ParkingTrajectory_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ParkingTrajectory_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.mf_mempark.parking_trajectory.ParkingTrajectory data = 1072;
      case 1072:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 130)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<8578>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* ParkingTrajectory_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.mf_mempark.parking_trajectory.ParkingTrajectory data = 1072;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1072, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
  return target;
}

size_t ParkingTrajectory_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.mf_mempark.parking_trajectory.ParkingTrajectory data = 1072;
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

void ParkingTrajectory_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const ParkingTrajectory_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ParkingTrajectory_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
    MergeFrom(*source);
  }
}

void ParkingTrajectory_array_port::MergeFrom(const ParkingTrajectory_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void ParkingTrajectory_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ParkingTrajectory_array_port::CopyFrom(const ParkingTrajectory_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_mempark.parking_trajectory.ParkingTrajectory_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ParkingTrajectory_array_port::IsInitialized() const {
  return true;
}

void ParkingTrajectory_array_port::InternalSwap(ParkingTrajectory_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ParkingTrajectory_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace parking_trajectory
}  // namespace mf_mempark
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::mf_mempark::parking_trajectory::ParkingTrajectory* Arena::CreateMaybeMessage< ::pb::mf_mempark::parking_trajectory::ParkingTrajectory >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_mempark::parking_trajectory::ParkingTrajectory >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::mf_mempark::parking_trajectory::ParkingTrajectory_array_port* Arena::CreateMaybeMessage< ::pb::mf_mempark::parking_trajectory::ParkingTrajectory_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_mempark::parking_trajectory::ParkingTrajectory_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
