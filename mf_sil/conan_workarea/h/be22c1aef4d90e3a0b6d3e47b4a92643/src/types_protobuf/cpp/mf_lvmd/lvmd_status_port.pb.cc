// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mf_lvmd/lvmd_status_port.proto

#include "mf_lvmd/lvmd_status_port.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_LvmdStatusPort_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_mf_5flvmd_2flvmdlead_5fvehicle_5fstatus_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_LVMDLeadVehicleStatus_mf_5flvmd_2flvmdlead_5fvehicle_5fstatus_2eproto;
namespace pb {
namespace mf_lvmd {
namespace lvmd_status_port {
class LvmdStatusPortDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<LvmdStatusPort> _instance;
} _LvmdStatusPort_default_instance_;
class LvmdStatusPort_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<LvmdStatusPort_array_port> _instance;
} _LvmdStatusPort_array_port_default_instance_;
}  // namespace lvmd_status_port
}  // namespace mf_lvmd
}  // namespace pb
static void InitDefaultsscc_info_LvmdStatusPort_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_lvmd::lvmd_status_port::_LvmdStatusPort_default_instance_;
    new (ptr) ::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_LvmdStatusPort_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, 0, InitDefaultsscc_info_LvmdStatusPort_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,
      &scc_info_LVMDLeadVehicleStatus_mf_5flvmd_2flvmdlead_5fvehicle_5fstatus_2eproto.base,}};

static void InitDefaultsscc_info_LvmdStatusPort_array_port_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::mf_lvmd::lvmd_status_port::_LvmdStatusPort_array_port_default_instance_;
    new (ptr) ::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_LvmdStatusPort_array_port_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_LvmdStatusPort_array_port_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto}, {
      &scc_info_LvmdStatusPort_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort, uiversionnumber_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort, ssigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort, lvmdsystemstatus_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort, lvmdwarningtrigger_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort, lvmdwarningstatus_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort, lvmdleadvehiclestatus_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort, numvehiclesinroi_nu_),
  4,
  0,
  6,
  2,
  5,
  1,
  3,
  PROTOBUF_FIELD_OFFSET(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 12, sizeof(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort)},
  { 19, 25, sizeof(::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_lvmd::lvmd_status_port::_LvmdStatusPort_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::mf_lvmd::lvmd_status_port::_LvmdStatusPort_array_port_default_instance_),
};

const char descriptor_table_protodef_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\036mf_lvmd/lvmd_status_port.proto\022\033pb.mf_"
  "lvmd.lvmd_status_port\032\027eco/signal_header"
  ".proto\032\037mf_lvmd/lvmdsystem_status.proto\032"
  "!mf_lvmd/lvmdwarning_trigger.proto\032 mf_l"
  "vmd/lvmdwarning_status.proto\032%mf_lvmd/lv"
  "mdlead_vehicle_status.proto\"\322\003\n\016LvmdStat"
  "usPort\022\030\n\017uiVersionNumber\030\314\020 \001(\r\0227\n\nsSig"
  "Header\030\211\010 \001(\0132\".pb.eco.signal_header.Sig"
  "nalHeader\022L\n\023lvmdSystemStatus_nu\030\335\023 \001(\0162"
  "..pb.mf_lvmd.lvmdsystem_status.LVMDSyste"
  "mStatus\022R\n\025lvmdWarningTrigger_nu\030\346\033 \001(\0162"
  "2.pb.mf_lvmd.lvmdwarning_trigger.LVMDWar"
  "ningTrigger\022O\n\024lvmdWarningStatus_nu\030\213\022 \001"
  "(\01620.pb.mf_lvmd.lvmdwarning_status.LVMDW"
  "arningStatus\022\\\n\030lvmdLeadVehicleStatus_nu"
  "\030\261\017 \001(\01329.pb.mf_lvmd.lvmdlead_vehicle_st"
  "atus.LVMDLeadVehicleStatus\022\034\n\023numVehicle"
  "sinROI_nu\030\215\007 \001(\r\"W\n\031LvmdStatusPort_array"
  "_port\022:\n\004data\030\222\016 \003(\0132+.pb.mf_lvmd.lvmd_s"
  "tatus_port.LvmdStatusPort"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto_deps[5] = {
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
  &::descriptor_table_mf_5flvmd_2flvmdlead_5fvehicle_5fstatus_2eproto,
  &::descriptor_table_mf_5flvmd_2flvmdsystem_5fstatus_2eproto,
  &::descriptor_table_mf_5flvmd_2flvmdwarning_5fstatus_2eproto,
  &::descriptor_table_mf_5flvmd_2flvmdwarning_5ftrigger_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto_sccs[2] = {
  &scc_info_LvmdStatusPort_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto.base,
  &scc_info_LvmdStatusPort_array_port_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto_once;
static bool descriptor_table_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto = {
  &descriptor_table_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto_initialized, descriptor_table_protodef_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto, "mf_lvmd/lvmd_status_port.proto", 785,
  &descriptor_table_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto_once, descriptor_table_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto_sccs, descriptor_table_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto_deps, 2, 5,
  schemas, file_default_instances, TableStruct_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto::offsets,
  file_level_metadata_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto, 2, file_level_enum_descriptors_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto, file_level_service_descriptors_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto), true);
namespace pb {
namespace mf_lvmd {
namespace lvmd_status_port {

// ===================================================================

void LvmdStatusPort::InitAsDefaultInstance() {
  ::pb::mf_lvmd::lvmd_status_port::_LvmdStatusPort_default_instance_._instance.get_mutable()->ssigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
  ::pb::mf_lvmd::lvmd_status_port::_LvmdStatusPort_default_instance_._instance.get_mutable()->lvmdleadvehiclestatus_nu_ = const_cast< ::pb::mf_lvmd::lvmdlead_vehicle_status::LVMDLeadVehicleStatus*>(
      ::pb::mf_lvmd::lvmdlead_vehicle_status::LVMDLeadVehicleStatus::internal_default_instance());
}
class LvmdStatusPort::_Internal {
 public:
  using HasBits = decltype(std::declval<LvmdStatusPort>()._has_bits_);
  static void set_has_uiversionnumber(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static const ::pb::eco::signal_header::SignalHeader& ssigheader(const LvmdStatusPort* msg);
  static void set_has_ssigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_lvmdsystemstatus_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_lvmdwarningtrigger_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_lvmdwarningstatus_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static const ::pb::mf_lvmd::lvmdlead_vehicle_status::LVMDLeadVehicleStatus& lvmdleadvehiclestatus_nu(const LvmdStatusPort* msg);
  static void set_has_lvmdleadvehiclestatus_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_numvehiclesinroi_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
LvmdStatusPort::_Internal::ssigheader(const LvmdStatusPort* msg) {
  return *msg->ssigheader_;
}
const ::pb::mf_lvmd::lvmdlead_vehicle_status::LVMDLeadVehicleStatus&
LvmdStatusPort::_Internal::lvmdleadvehiclestatus_nu(const LvmdStatusPort* msg) {
  return *msg->lvmdleadvehiclestatus_nu_;
}
void LvmdStatusPort::clear_ssigheader() {
  if (ssigheader_ != nullptr) ssigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void LvmdStatusPort::clear_lvmdleadvehiclestatus_nu() {
  if (lvmdleadvehiclestatus_nu_ != nullptr) lvmdleadvehiclestatus_nu_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
LvmdStatusPort::LvmdStatusPort()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
}
LvmdStatusPort::LvmdStatusPort(const LvmdStatusPort& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_ssigheader()) {
    ssigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.ssigheader_);
  } else {
    ssigheader_ = nullptr;
  }
  if (from._internal_has_lvmdleadvehiclestatus_nu()) {
    lvmdleadvehiclestatus_nu_ = new ::pb::mf_lvmd::lvmdlead_vehicle_status::LVMDLeadVehicleStatus(*from.lvmdleadvehiclestatus_nu_);
  } else {
    lvmdleadvehiclestatus_nu_ = nullptr;
  }
  ::memcpy(&lvmdwarningtrigger_nu_, &from.lvmdwarningtrigger_nu_,
    static_cast<size_t>(reinterpret_cast<char*>(&lvmdsystemstatus_nu_) -
    reinterpret_cast<char*>(&lvmdwarningtrigger_nu_)) + sizeof(lvmdsystemstatus_nu_));
  // @@protoc_insertion_point(copy_constructor:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
}

void LvmdStatusPort::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_LvmdStatusPort_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto.base);
  ::memset(&ssigheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&lvmdsystemstatus_nu_) -
      reinterpret_cast<char*>(&ssigheader_)) + sizeof(lvmdsystemstatus_nu_));
}

LvmdStatusPort::~LvmdStatusPort() {
  // @@protoc_insertion_point(destructor:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
  SharedDtor();
}

void LvmdStatusPort::SharedDtor() {
  if (this != internal_default_instance()) delete ssigheader_;
  if (this != internal_default_instance()) delete lvmdleadvehiclestatus_nu_;
}

void LvmdStatusPort::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const LvmdStatusPort& LvmdStatusPort::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_LvmdStatusPort_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto.base);
  return *internal_default_instance();
}


void LvmdStatusPort::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(ssigheader_ != nullptr);
      ssigheader_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(lvmdleadvehiclestatus_nu_ != nullptr);
      lvmdleadvehiclestatus_nu_->Clear();
    }
  }
  if (cached_has_bits & 0x0000007cu) {
    ::memset(&lvmdwarningtrigger_nu_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&lvmdsystemstatus_nu_) -
        reinterpret_cast<char*>(&lvmdwarningtrigger_nu_)) + sizeof(lvmdsystemstatus_nu_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* LvmdStatusPort::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional uint32 numVehiclesinROI_nu = 909;
      case 909:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 104)) {
          _Internal::set_has_numvehiclesinroi_nu(&has_bits);
          numvehiclesinroi_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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
      // optional .pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus lvmdLeadVehicleStatus_nu = 1969;
      case 1969:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 138)) {
          ptr = ctx->ParseMessage(_internal_mutable_lvmdleadvehiclestatus_nu(), ptr);
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
      // optional .pb.mf_lvmd.lvmdwarning_status.LVMDWarningStatus lvmdWarningStatus_nu = 2315;
      case 2315:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 88)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::mf_lvmd::lvmdwarning_status::LVMDWarningStatus_IsValid(val))) {
            _internal_set_lvmdwarningstatus_nu(static_cast<::pb::mf_lvmd::lvmdwarning_status::LVMDWarningStatus>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(2315, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.mf_lvmd.lvmdsystem_status.LVMDSystemStatus lvmdSystemStatus_nu = 2525;
      case 2525:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 232)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::mf_lvmd::lvmdsystem_status::LVMDSystemStatus_IsValid(val))) {
            _internal_set_lvmdsystemstatus_nu(static_cast<::pb::mf_lvmd::lvmdsystem_status::LVMDSystemStatus>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(2525, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.mf_lvmd.lvmdwarning_trigger.LVMDWarningTrigger lvmdWarningTrigger_nu = 3558;
      case 3558:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::mf_lvmd::lvmdwarning_trigger::LVMDWarningTrigger_IsValid(val))) {
            _internal_set_lvmdwarningtrigger_nu(static_cast<::pb::mf_lvmd::lvmdwarning_trigger::LVMDWarningTrigger>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(3558, val, mutable_unknown_fields());
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

::PROTOBUF_NAMESPACE_ID::uint8* LvmdStatusPort::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional uint32 numVehiclesinROI_nu = 909;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(909, this->_internal_numvehiclesinroi_nu(), target);
  }

  // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1033, _Internal::ssigheader(this), target, stream);
  }

  // optional .pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus lvmdLeadVehicleStatus_nu = 1969;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1969, _Internal::lvmdleadvehiclestatus_nu(this), target, stream);
  }

  // optional uint32 uiVersionNumber = 2124;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2124, this->_internal_uiversionnumber(), target);
  }

  // optional .pb.mf_lvmd.lvmdwarning_status.LVMDWarningStatus lvmdWarningStatus_nu = 2315;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      2315, this->_internal_lvmdwarningstatus_nu(), target);
  }

  // optional .pb.mf_lvmd.lvmdsystem_status.LVMDSystemStatus lvmdSystemStatus_nu = 2525;
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      2525, this->_internal_lvmdsystemstatus_nu(), target);
  }

  // optional .pb.mf_lvmd.lvmdwarning_trigger.LVMDWarningTrigger lvmdWarningTrigger_nu = 3558;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      3558, this->_internal_lvmdwarningtrigger_nu(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
  return target;
}

size_t LvmdStatusPort::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000007fu) {
    // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *ssigheader_);
    }

    // optional .pb.mf_lvmd.lvmdlead_vehicle_status.LVMDLeadVehicleStatus lvmdLeadVehicleStatus_nu = 1969;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *lvmdleadvehiclestatus_nu_);
    }

    // optional .pb.mf_lvmd.lvmdwarning_trigger.LVMDWarningTrigger lvmdWarningTrigger_nu = 3558;
    if (cached_has_bits & 0x00000004u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_lvmdwarningtrigger_nu());
    }

    // optional uint32 numVehiclesinROI_nu = 909;
    if (cached_has_bits & 0x00000008u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_numvehiclesinroi_nu());
    }

    // optional uint32 uiVersionNumber = 2124;
    if (cached_has_bits & 0x00000010u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_uiversionnumber());
    }

    // optional .pb.mf_lvmd.lvmdwarning_status.LVMDWarningStatus lvmdWarningStatus_nu = 2315;
    if (cached_has_bits & 0x00000020u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_lvmdwarningstatus_nu());
    }

    // optional .pb.mf_lvmd.lvmdsystem_status.LVMDSystemStatus lvmdSystemStatus_nu = 2525;
    if (cached_has_bits & 0x00000040u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_lvmdsystemstatus_nu());
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

void LvmdStatusPort::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
  GOOGLE_DCHECK_NE(&from, this);
  const LvmdStatusPort* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<LvmdStatusPort>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
    MergeFrom(*source);
  }
}

void LvmdStatusPort::MergeFrom(const LvmdStatusPort& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000007fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_ssigheader()->::pb::eco::signal_header::SignalHeader::MergeFrom(from._internal_ssigheader());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_lvmdleadvehiclestatus_nu()->::pb::mf_lvmd::lvmdlead_vehicle_status::LVMDLeadVehicleStatus::MergeFrom(from._internal_lvmdleadvehiclestatus_nu());
    }
    if (cached_has_bits & 0x00000004u) {
      lvmdwarningtrigger_nu_ = from.lvmdwarningtrigger_nu_;
    }
    if (cached_has_bits & 0x00000008u) {
      numvehiclesinroi_nu_ = from.numvehiclesinroi_nu_;
    }
    if (cached_has_bits & 0x00000010u) {
      uiversionnumber_ = from.uiversionnumber_;
    }
    if (cached_has_bits & 0x00000020u) {
      lvmdwarningstatus_nu_ = from.lvmdwarningstatus_nu_;
    }
    if (cached_has_bits & 0x00000040u) {
      lvmdsystemstatus_nu_ = from.lvmdsystemstatus_nu_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void LvmdStatusPort::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void LvmdStatusPort::CopyFrom(const LvmdStatusPort& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LvmdStatusPort::IsInitialized() const {
  return true;
}

void LvmdStatusPort::InternalSwap(LvmdStatusPort* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(ssigheader_, other->ssigheader_);
  swap(lvmdleadvehiclestatus_nu_, other->lvmdleadvehiclestatus_nu_);
  swap(lvmdwarningtrigger_nu_, other->lvmdwarningtrigger_nu_);
  swap(numvehiclesinroi_nu_, other->numvehiclesinroi_nu_);
  swap(uiversionnumber_, other->uiversionnumber_);
  swap(lvmdwarningstatus_nu_, other->lvmdwarningstatus_nu_);
  swap(lvmdsystemstatus_nu_, other->lvmdsystemstatus_nu_);
}

::PROTOBUF_NAMESPACE_ID::Metadata LvmdStatusPort::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void LvmdStatusPort_array_port::InitAsDefaultInstance() {
}
class LvmdStatusPort_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<LvmdStatusPort_array_port>()._has_bits_);
};

LvmdStatusPort_array_port::LvmdStatusPort_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
}
LvmdStatusPort_array_port::LvmdStatusPort_array_port(const LvmdStatusPort_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
}

void LvmdStatusPort_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_LvmdStatusPort_array_port_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto.base);
}

LvmdStatusPort_array_port::~LvmdStatusPort_array_port() {
  // @@protoc_insertion_point(destructor:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
  SharedDtor();
}

void LvmdStatusPort_array_port::SharedDtor() {
}

void LvmdStatusPort_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const LvmdStatusPort_array_port& LvmdStatusPort_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_LvmdStatusPort_array_port_mf_5flvmd_2flvmd_5fstatus_5fport_2eproto.base);
  return *internal_default_instance();
}


void LvmdStatusPort_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* LvmdStatusPort_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.mf_lvmd.lvmd_status_port.LvmdStatusPort data = 1810;
      case 1810:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 146)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<14482>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* LvmdStatusPort_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.mf_lvmd.lvmd_status_port.LvmdStatusPort data = 1810;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1810, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
  return target;
}

size_t LvmdStatusPort_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.mf_lvmd.lvmd_status_port.LvmdStatusPort data = 1810;
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

void LvmdStatusPort_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const LvmdStatusPort_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<LvmdStatusPort_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
    MergeFrom(*source);
  }
}

void LvmdStatusPort_array_port::MergeFrom(const LvmdStatusPort_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void LvmdStatusPort_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void LvmdStatusPort_array_port::CopyFrom(const LvmdStatusPort_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.mf_lvmd.lvmd_status_port.LvmdStatusPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LvmdStatusPort_array_port::IsInitialized() const {
  return true;
}

void LvmdStatusPort_array_port::InternalSwap(LvmdStatusPort_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata LvmdStatusPort_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace lvmd_status_port
}  // namespace mf_lvmd
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort* Arena::CreateMaybeMessage< ::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort_array_port* Arena::CreateMaybeMessage< ::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::mf_lvmd::lvmd_status_port::LvmdStatusPort_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
