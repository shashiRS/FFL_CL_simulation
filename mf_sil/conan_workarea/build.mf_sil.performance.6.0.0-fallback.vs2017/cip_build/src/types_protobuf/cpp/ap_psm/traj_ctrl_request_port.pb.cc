// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_psm/traj_ctrl_request_port.proto

#include "ap_psm/traj_ctrl_request_port.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_TrajCtrlRequestPort_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_eco_2fsignal_5fheader_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto;
namespace pb {
namespace ap_psm {
namespace traj_ctrl_request_port {
class TrajCtrlRequestPortDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<TrajCtrlRequestPort> _instance;
} _TrajCtrlRequestPort_default_instance_;
class TrajCtrlRequestPort_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<TrajCtrlRequestPort_array_port> _instance;
} _TrajCtrlRequestPort_array_port_default_instance_;
}  // namespace traj_ctrl_request_port
}  // namespace ap_psm
}  // namespace pb
static void InitDefaultsscc_info_TrajCtrlRequestPort_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_psm::traj_ctrl_request_port::_TrajCtrlRequestPort_default_instance_;
    new (ptr) ::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_TrajCtrlRequestPort_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_TrajCtrlRequestPort_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,}};

static void InitDefaultsscc_info_TrajCtrlRequestPort_array_port_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_psm::traj_ctrl_request_port::_TrajCtrlRequestPort_array_port_default_instance_;
    new (ptr) ::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_TrajCtrlRequestPort_array_port_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_TrajCtrlRequestPort_array_port_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto}, {
      &scc_info_TrajCtrlRequestPort_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort, uiversionnumber_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort, ssigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort, drivingmodereq_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort, trajctrlactive_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort, emergencybrakerequest_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort, motioncontrolrequesttype_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort, driverinoutrequesttype_nu_),
  5,
  0,
  3,
  1,
  2,
  6,
  4,
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 12, sizeof(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort)},
  { 19, 25, sizeof(::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_psm::traj_ctrl_request_port::_TrajCtrlRequestPort_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_psm::traj_ctrl_request_port::_TrajCtrlRequestPort_array_port_default_instance_),
};

const char descriptor_table_protodef_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n#ap_psm/traj_ctrl_request_port.proto\022 p"
  "b.ap_psm.traj_ctrl_request_port\032\027eco/sig"
  "nal_header.proto\032\035ap_psm/driving_mode_tr"
  "c.proto\032(ap_psm/motion_control_request_t"
  "ype.proto\032\'ap_psm/driver_in_out_request_"
  "type.proto\"\265\003\n\023TrajCtrlRequestPort\022\030\n\017ui"
  "VersionNumber\030\314\020 \001(\r\0227\n\nsSigHeader\030\211\010 \001("
  "\0132\".pb.eco.signal_header.SignalHeader\022F\n"
  "\021drivingModeReq_nu\030\262\003 \001(\0162*.pb.ap_psm.dr"
  "iving_mode_trc.DrivingModeTRC\022\032\n\021trajCtr"
  "lActive_nu\030\330\t \001(\010\022\036\n\025emergencyBrakeReque"
  "st\030\307\030 \001(\010\022e\n\033MotionControlRequestType_nu"
  "\030\352\020 \001(\0162\?.pb.ap_psm.motion_control_reque"
  "st_type.MotionControlRequestType\022`\n\031driv"
  "erInOutRequestType_nu\030\331\004 \001(\0162<.pb.ap_psm"
  ".driver_in_out_request_type.DriverInOutR"
  "equestType\"f\n\036TrajCtrlRequestPort_array_"
  "port\022D\n\004data\030\245\024 \003(\01325.pb.ap_psm.traj_ctr"
  "l_request_port.TrajCtrlRequestPort"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto_deps[4] = {
  &::descriptor_table_ap_5fpsm_2fdriver_5fin_5fout_5frequest_5ftype_2eproto,
  &::descriptor_table_ap_5fpsm_2fdriving_5fmode_5ftrc_2eproto,
  &::descriptor_table_ap_5fpsm_2fmotion_5fcontrol_5frequest_5ftype_2eproto,
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto_sccs[2] = {
  &scc_info_TrajCtrlRequestPort_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto.base,
  &scc_info_TrajCtrlRequestPort_array_port_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto_once;
static bool descriptor_table_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto = {
  &descriptor_table_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto_initialized, descriptor_table_protodef_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto, "ap_psm/traj_ctrl_request_port.proto", 754,
  &descriptor_table_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto_once, descriptor_table_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto_sccs, descriptor_table_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto_deps, 2, 4,
  schemas, file_default_instances, TableStruct_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto::offsets,
  file_level_metadata_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto, 2, file_level_enum_descriptors_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto, file_level_service_descriptors_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto), true);
namespace pb {
namespace ap_psm {
namespace traj_ctrl_request_port {

// ===================================================================

void TrajCtrlRequestPort::InitAsDefaultInstance() {
  ::pb::ap_psm::traj_ctrl_request_port::_TrajCtrlRequestPort_default_instance_._instance.get_mutable()->ssigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
}
class TrajCtrlRequestPort::_Internal {
 public:
  using HasBits = decltype(std::declval<TrajCtrlRequestPort>()._has_bits_);
  static void set_has_uiversionnumber(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static const ::pb::eco::signal_header::SignalHeader& ssigheader(const TrajCtrlRequestPort* msg);
  static void set_has_ssigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_drivingmodereq_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_trajctrlactive_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_emergencybrakerequest(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_motioncontrolrequesttype_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_driverinoutrequesttype_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
TrajCtrlRequestPort::_Internal::ssigheader(const TrajCtrlRequestPort* msg) {
  return *msg->ssigheader_;
}
void TrajCtrlRequestPort::clear_ssigheader() {
  if (ssigheader_ != nullptr) ssigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
TrajCtrlRequestPort::TrajCtrlRequestPort()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
}
TrajCtrlRequestPort::TrajCtrlRequestPort(const TrajCtrlRequestPort& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_ssigheader()) {
    ssigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.ssigheader_);
  } else {
    ssigheader_ = nullptr;
  }
  ::memcpy(&trajctrlactive_nu_, &from.trajctrlactive_nu_,
    static_cast<size_t>(reinterpret_cast<char*>(&motioncontrolrequesttype_nu_) -
    reinterpret_cast<char*>(&trajctrlactive_nu_)) + sizeof(motioncontrolrequesttype_nu_));
  // @@protoc_insertion_point(copy_constructor:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
}

void TrajCtrlRequestPort::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_TrajCtrlRequestPort_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto.base);
  ::memset(&ssigheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&motioncontrolrequesttype_nu_) -
      reinterpret_cast<char*>(&ssigheader_)) + sizeof(motioncontrolrequesttype_nu_));
}

TrajCtrlRequestPort::~TrajCtrlRequestPort() {
  // @@protoc_insertion_point(destructor:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
  SharedDtor();
}

void TrajCtrlRequestPort::SharedDtor() {
  if (this != internal_default_instance()) delete ssigheader_;
}

void TrajCtrlRequestPort::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const TrajCtrlRequestPort& TrajCtrlRequestPort::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_TrajCtrlRequestPort_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto.base);
  return *internal_default_instance();
}


void TrajCtrlRequestPort::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(ssigheader_ != nullptr);
    ssigheader_->Clear();
  }
  if (cached_has_bits & 0x0000007eu) {
    ::memset(&trajctrlactive_nu_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&motioncontrolrequesttype_nu_) -
        reinterpret_cast<char*>(&trajctrlactive_nu_)) + sizeof(motioncontrolrequesttype_nu_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* TrajCtrlRequestPort::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .pb.ap_psm.driving_mode_trc.DrivingModeTRC drivingModeReq_nu = 434;
      case 434:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 144)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_psm::driving_mode_trc::DrivingModeTRC_IsValid(val))) {
            _internal_set_drivingmodereq_nu(static_cast<::pb::ap_psm::driving_mode_trc::DrivingModeTRC>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(434, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_psm.driver_in_out_request_type.DriverInOutRequestType driverInOutRequestType_nu = 601;
      case 601:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 200)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_psm::driver_in_out_request_type::DriverInOutRequestType_IsValid(val))) {
            _internal_set_driverinoutrequesttype_nu(static_cast<::pb::ap_psm::driver_in_out_request_type::DriverInOutRequestType>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(601, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
      case 1033:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 74)) {
          ptr = ctx->ParseMessage(_internal_mutable_ssigheader(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool trajCtrlActive_nu = 1240;
      case 1240:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 192)) {
          _Internal::set_has_trajctrlactive_nu(&has_bits);
          trajctrlactive_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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
      // optional .pb.ap_psm.motion_control_request_type.MotionControlRequestType MotionControlRequestType_nu = 2154;
      case 2154:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 80)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_psm::motion_control_request_type::MotionControlRequestType_IsValid(val))) {
            _internal_set_motioncontrolrequesttype_nu(static_cast<::pb::ap_psm::motion_control_request_type::MotionControlRequestType>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(2154, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional bool emergencyBrakeRequest = 3143;
      case 3143:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          _Internal::set_has_emergencybrakerequest(&has_bits);
          emergencybrakerequest_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* TrajCtrlRequestPort::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.ap_psm.driving_mode_trc.DrivingModeTRC drivingModeReq_nu = 434;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      434, this->_internal_drivingmodereq_nu(), target);
  }

  // optional .pb.ap_psm.driver_in_out_request_type.DriverInOutRequestType driverInOutRequestType_nu = 601;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      601, this->_internal_driverinoutrequesttype_nu(), target);
  }

  // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1033, _Internal::ssigheader(this), target, stream);
  }

  // optional bool trajCtrlActive_nu = 1240;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1240, this->_internal_trajctrlactive_nu(), target);
  }

  // optional uint32 uiVersionNumber = 2124;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2124, this->_internal_uiversionnumber(), target);
  }

  // optional .pb.ap_psm.motion_control_request_type.MotionControlRequestType MotionControlRequestType_nu = 2154;
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      2154, this->_internal_motioncontrolrequesttype_nu(), target);
  }

  // optional bool emergencyBrakeRequest = 3143;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3143, this->_internal_emergencybrakerequest(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
  return target;
}

size_t TrajCtrlRequestPort::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
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

    // optional bool trajCtrlActive_nu = 1240;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 + 1;
    }

    // optional bool emergencyBrakeRequest = 3143;
    if (cached_has_bits & 0x00000004u) {
      total_size += 3 + 1;
    }

    // optional .pb.ap_psm.driving_mode_trc.DrivingModeTRC drivingModeReq_nu = 434;
    if (cached_has_bits & 0x00000008u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_drivingmodereq_nu());
    }

    // optional .pb.ap_psm.driver_in_out_request_type.DriverInOutRequestType driverInOutRequestType_nu = 601;
    if (cached_has_bits & 0x00000010u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_driverinoutrequesttype_nu());
    }

    // optional uint32 uiVersionNumber = 2124;
    if (cached_has_bits & 0x00000020u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_uiversionnumber());
    }

    // optional .pb.ap_psm.motion_control_request_type.MotionControlRequestType MotionControlRequestType_nu = 2154;
    if (cached_has_bits & 0x00000040u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_motioncontrolrequesttype_nu());
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

void TrajCtrlRequestPort::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
  GOOGLE_DCHECK_NE(&from, this);
  const TrajCtrlRequestPort* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<TrajCtrlRequestPort>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
    MergeFrom(*source);
  }
}

void TrajCtrlRequestPort::MergeFrom(const TrajCtrlRequestPort& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
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
      trajctrlactive_nu_ = from.trajctrlactive_nu_;
    }
    if (cached_has_bits & 0x00000004u) {
      emergencybrakerequest_ = from.emergencybrakerequest_;
    }
    if (cached_has_bits & 0x00000008u) {
      drivingmodereq_nu_ = from.drivingmodereq_nu_;
    }
    if (cached_has_bits & 0x00000010u) {
      driverinoutrequesttype_nu_ = from.driverinoutrequesttype_nu_;
    }
    if (cached_has_bits & 0x00000020u) {
      uiversionnumber_ = from.uiversionnumber_;
    }
    if (cached_has_bits & 0x00000040u) {
      motioncontrolrequesttype_nu_ = from.motioncontrolrequesttype_nu_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void TrajCtrlRequestPort::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void TrajCtrlRequestPort::CopyFrom(const TrajCtrlRequestPort& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TrajCtrlRequestPort::IsInitialized() const {
  return true;
}

void TrajCtrlRequestPort::InternalSwap(TrajCtrlRequestPort* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(ssigheader_, other->ssigheader_);
  swap(trajctrlactive_nu_, other->trajctrlactive_nu_);
  swap(emergencybrakerequest_, other->emergencybrakerequest_);
  swap(drivingmodereq_nu_, other->drivingmodereq_nu_);
  swap(driverinoutrequesttype_nu_, other->driverinoutrequesttype_nu_);
  swap(uiversionnumber_, other->uiversionnumber_);
  swap(motioncontrolrequesttype_nu_, other->motioncontrolrequesttype_nu_);
}

::PROTOBUF_NAMESPACE_ID::Metadata TrajCtrlRequestPort::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void TrajCtrlRequestPort_array_port::InitAsDefaultInstance() {
}
class TrajCtrlRequestPort_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<TrajCtrlRequestPort_array_port>()._has_bits_);
};

TrajCtrlRequestPort_array_port::TrajCtrlRequestPort_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
}
TrajCtrlRequestPort_array_port::TrajCtrlRequestPort_array_port(const TrajCtrlRequestPort_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
}

void TrajCtrlRequestPort_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_TrajCtrlRequestPort_array_port_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto.base);
}

TrajCtrlRequestPort_array_port::~TrajCtrlRequestPort_array_port() {
  // @@protoc_insertion_point(destructor:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
  SharedDtor();
}

void TrajCtrlRequestPort_array_port::SharedDtor() {
}

void TrajCtrlRequestPort_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const TrajCtrlRequestPort_array_port& TrajCtrlRequestPort_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_TrajCtrlRequestPort_array_port_ap_5fpsm_2ftraj_5fctrl_5frequest_5fport_2eproto.base);
  return *internal_default_instance();
}


void TrajCtrlRequestPort_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* TrajCtrlRequestPort_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort data = 2597;
      case 2597:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
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

::PROTOBUF_NAMESPACE_ID::uint8* TrajCtrlRequestPort_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort data = 2597;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2597, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
  return target;
}

size_t TrajCtrlRequestPort_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort data = 2597;
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

void TrajCtrlRequestPort_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const TrajCtrlRequestPort_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<TrajCtrlRequestPort_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
    MergeFrom(*source);
  }
}

void TrajCtrlRequestPort_array_port::MergeFrom(const TrajCtrlRequestPort_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void TrajCtrlRequestPort_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void TrajCtrlRequestPort_array_port::CopyFrom(const TrajCtrlRequestPort_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_psm.traj_ctrl_request_port.TrajCtrlRequestPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TrajCtrlRequestPort_array_port::IsInitialized() const {
  return true;
}

void TrajCtrlRequestPort_array_port::InternalSwap(TrajCtrlRequestPort_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata TrajCtrlRequestPort_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace traj_ctrl_request_port
}  // namespace ap_psm
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort* Arena::CreateMaybeMessage< ::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort_array_port* Arena::CreateMaybeMessage< ::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_psm::traj_ctrl_request_port::TrajCtrlRequestPort_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
