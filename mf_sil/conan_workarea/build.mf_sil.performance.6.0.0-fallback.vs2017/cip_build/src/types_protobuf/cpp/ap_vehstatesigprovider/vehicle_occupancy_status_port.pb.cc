// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/vehicle_occupancy_status_port.proto

#include "ap_vehstatesigprovider/vehicle_occupancy_status_port.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_BeltBuckleStatus_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fseat_5foccupancy_5fstatus_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SeatOccupancyStatus_ap_5fvehstatesigprovider_2fseat_5foccupancy_5fstatus_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<3> scc_info_VehicleOccupancyStatusPort_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_eco_2fsignal_5fheader_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace vehicle_occupancy_status_port {
class VehicleOccupancyStatusPortDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<VehicleOccupancyStatusPort> _instance;
} _VehicleOccupancyStatusPort_default_instance_;
class VehicleOccupancyStatusPort_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<VehicleOccupancyStatusPort_array_port> _instance;
} _VehicleOccupancyStatusPort_array_port_default_instance_;
}  // namespace vehicle_occupancy_status_port
}  // namespace ap_vehstatesigprovider
}  // namespace pb
static void InitDefaultsscc_info_VehicleOccupancyStatusPort_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::_VehicleOccupancyStatusPort_default_instance_;
    new (ptr) ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<3> scc_info_VehicleOccupancyStatusPort_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 3, 0, InitDefaultsscc_info_VehicleOccupancyStatusPort_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,
      &scc_info_SeatOccupancyStatus_ap_5fvehstatesigprovider_2fseat_5foccupancy_5fstatus_2eproto.base,
      &scc_info_BeltBuckleStatus_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto.base,}};

static void InitDefaultsscc_info_VehicleOccupancyStatusPort_array_port_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::_VehicleOccupancyStatusPort_array_port_default_instance_;
    new (ptr) ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_VehicleOccupancyStatusPort_array_port_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_VehicleOccupancyStatusPort_array_port_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto}, {
      &scc_info_VehicleOccupancyStatusPort_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort, uiversionnumber_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort, ssigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort, seatoccupancystatus_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort, beltbucklestatus_),
  3,
  0,
  1,
  2,
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort)},
  { 13, 19, sizeof(::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::_VehicleOccupancyStatusPort_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::_VehicleOccupancyStatusPort_array_port_default_instance_),
};

const char descriptor_table_protodef_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n:ap_vehstatesigprovider/vehicle_occupan"
  "cy_status_port.proto\0227pb.ap_vehstatesigp"
  "rovider.vehicle_occupancy_status_port\032\027e"
  "co/signal_header.proto\0322ap_vehstatesigpr"
  "ovider/seat_occupancy_status.proto\032/ap_v"
  "ehstatesigprovider/belt_buckle_status.pr"
  "oto\"\256\002\n\032VehicleOccupancyStatusPort\022\030\n\017ui"
  "VersionNumber\030\314\020 \001(\r\0227\n\nsSigHeader\030\211\010 \001("
  "\0132\".pb.eco.signal_header.SignalHeader\022b\n"
  "\023seatOccupancyStatus\030\251\n \001(\0132D.pb.ap_vehs"
  "tatesigprovider.seat_occupancy_status.Se"
  "atOccupancyStatus\022Y\n\020beltBuckleStatus\030\255\016"
  " \001(\0132>.pb.ap_vehstatesigprovider.belt_bu"
  "ckle_status.BeltBuckleStatus\"\213\001\n%Vehicle"
  "OccupancyStatusPort_array_port\022b\n\004data\030\325"
  "\002 \003(\0132S.pb.ap_vehstatesigprovider.vehicl"
  "e_occupancy_status_port.VehicleOccupancy"
  "StatusPort"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto_deps[3] = {
  &::descriptor_table_ap_5fvehstatesigprovider_2fbelt_5fbuckle_5fstatus_2eproto,
  &::descriptor_table_ap_5fvehstatesigprovider_2fseat_5foccupancy_5fstatus_2eproto,
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto_sccs[2] = {
  &scc_info_VehicleOccupancyStatusPort_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto.base,
  &scc_info_VehicleOccupancyStatusPort_array_port_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto_once;
static bool descriptor_table_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto = {
  &descriptor_table_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto_initialized, descriptor_table_protodef_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto, "ap_vehstatesigprovider/vehicle_occupancy_status_port.proto", 690,
  &descriptor_table_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto_once, descriptor_table_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto_sccs, descriptor_table_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto_deps, 2, 3,
  schemas, file_default_instances, TableStruct_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto::offsets,
  file_level_metadata_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto, 2, file_level_enum_descriptors_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto, file_level_service_descriptors_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto), true);
namespace pb {
namespace ap_vehstatesigprovider {
namespace vehicle_occupancy_status_port {

// ===================================================================

void VehicleOccupancyStatusPort::InitAsDefaultInstance() {
  ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::_VehicleOccupancyStatusPort_default_instance_._instance.get_mutable()->ssigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
  ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::_VehicleOccupancyStatusPort_default_instance_._instance.get_mutable()->seatoccupancystatus_ = const_cast< ::pb::ap_vehstatesigprovider::seat_occupancy_status::SeatOccupancyStatus*>(
      ::pb::ap_vehstatesigprovider::seat_occupancy_status::SeatOccupancyStatus::internal_default_instance());
  ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::_VehicleOccupancyStatusPort_default_instance_._instance.get_mutable()->beltbucklestatus_ = const_cast< ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus*>(
      ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus::internal_default_instance());
}
class VehicleOccupancyStatusPort::_Internal {
 public:
  using HasBits = decltype(std::declval<VehicleOccupancyStatusPort>()._has_bits_);
  static void set_has_uiversionnumber(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static const ::pb::eco::signal_header::SignalHeader& ssigheader(const VehicleOccupancyStatusPort* msg);
  static void set_has_ssigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::pb::ap_vehstatesigprovider::seat_occupancy_status::SeatOccupancyStatus& seatoccupancystatus(const VehicleOccupancyStatusPort* msg);
  static void set_has_seatoccupancystatus(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus& beltbucklestatus(const VehicleOccupancyStatusPort* msg);
  static void set_has_beltbucklestatus(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
VehicleOccupancyStatusPort::_Internal::ssigheader(const VehicleOccupancyStatusPort* msg) {
  return *msg->ssigheader_;
}
const ::pb::ap_vehstatesigprovider::seat_occupancy_status::SeatOccupancyStatus&
VehicleOccupancyStatusPort::_Internal::seatoccupancystatus(const VehicleOccupancyStatusPort* msg) {
  return *msg->seatoccupancystatus_;
}
const ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus&
VehicleOccupancyStatusPort::_Internal::beltbucklestatus(const VehicleOccupancyStatusPort* msg) {
  return *msg->beltbucklestatus_;
}
void VehicleOccupancyStatusPort::clear_ssigheader() {
  if (ssigheader_ != nullptr) ssigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void VehicleOccupancyStatusPort::clear_seatoccupancystatus() {
  if (seatoccupancystatus_ != nullptr) seatoccupancystatus_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
void VehicleOccupancyStatusPort::clear_beltbucklestatus() {
  if (beltbucklestatus_ != nullptr) beltbucklestatus_->Clear();
  _has_bits_[0] &= ~0x00000004u;
}
VehicleOccupancyStatusPort::VehicleOccupancyStatusPort()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
}
VehicleOccupancyStatusPort::VehicleOccupancyStatusPort(const VehicleOccupancyStatusPort& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_ssigheader()) {
    ssigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.ssigheader_);
  } else {
    ssigheader_ = nullptr;
  }
  if (from._internal_has_seatoccupancystatus()) {
    seatoccupancystatus_ = new ::pb::ap_vehstatesigprovider::seat_occupancy_status::SeatOccupancyStatus(*from.seatoccupancystatus_);
  } else {
    seatoccupancystatus_ = nullptr;
  }
  if (from._internal_has_beltbucklestatus()) {
    beltbucklestatus_ = new ::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus(*from.beltbucklestatus_);
  } else {
    beltbucklestatus_ = nullptr;
  }
  uiversionnumber_ = from.uiversionnumber_;
  // @@protoc_insertion_point(copy_constructor:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
}

void VehicleOccupancyStatusPort::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_VehicleOccupancyStatusPort_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto.base);
  ::memset(&ssigheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&uiversionnumber_) -
      reinterpret_cast<char*>(&ssigheader_)) + sizeof(uiversionnumber_));
}

VehicleOccupancyStatusPort::~VehicleOccupancyStatusPort() {
  // @@protoc_insertion_point(destructor:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
  SharedDtor();
}

void VehicleOccupancyStatusPort::SharedDtor() {
  if (this != internal_default_instance()) delete ssigheader_;
  if (this != internal_default_instance()) delete seatoccupancystatus_;
  if (this != internal_default_instance()) delete beltbucklestatus_;
}

void VehicleOccupancyStatusPort::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const VehicleOccupancyStatusPort& VehicleOccupancyStatusPort::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_VehicleOccupancyStatusPort_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto.base);
  return *internal_default_instance();
}


void VehicleOccupancyStatusPort::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(ssigheader_ != nullptr);
      ssigheader_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(seatoccupancystatus_ != nullptr);
      seatoccupancystatus_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(beltbucklestatus_ != nullptr);
      beltbucklestatus_->Clear();
    }
  }
  uiversionnumber_ = 0u;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* VehicleOccupancyStatusPort::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
      case 1033:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 74)) {
          ptr = ctx->ParseMessage(_internal_mutable_ssigheader(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus seatOccupancyStatus = 1321;
      case 1321:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 74)) {
          ptr = ctx->ParseMessage(_internal_mutable_seatoccupancystatus(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus beltBuckleStatus = 1837;
      case 1837:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 106)) {
          ptr = ctx->ParseMessage(_internal_mutable_beltbucklestatus(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* VehicleOccupancyStatusPort::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1033, _Internal::ssigheader(this), target, stream);
  }

  // optional .pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus seatOccupancyStatus = 1321;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1321, _Internal::seatoccupancystatus(this), target, stream);
  }

  // optional .pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus beltBuckleStatus = 1837;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1837, _Internal::beltbucklestatus(this), target, stream);
  }

  // optional uint32 uiVersionNumber = 2124;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2124, this->_internal_uiversionnumber(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
  return target;
}

size_t VehicleOccupancyStatusPort::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *ssigheader_);
    }

    // optional .pb.ap_vehstatesigprovider.seat_occupancy_status.SeatOccupancyStatus seatOccupancyStatus = 1321;
    if (cached_has_bits & 0x00000002u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *seatoccupancystatus_);
    }

    // optional .pb.ap_vehstatesigprovider.belt_buckle_status.BeltBuckleStatus beltBuckleStatus = 1837;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *beltbucklestatus_);
    }

    // optional uint32 uiVersionNumber = 2124;
    if (cached_has_bits & 0x00000008u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_uiversionnumber());
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

void VehicleOccupancyStatusPort::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
  GOOGLE_DCHECK_NE(&from, this);
  const VehicleOccupancyStatusPort* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<VehicleOccupancyStatusPort>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
    MergeFrom(*source);
  }
}

void VehicleOccupancyStatusPort::MergeFrom(const VehicleOccupancyStatusPort& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_ssigheader()->::pb::eco::signal_header::SignalHeader::MergeFrom(from._internal_ssigheader());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_seatoccupancystatus()->::pb::ap_vehstatesigprovider::seat_occupancy_status::SeatOccupancyStatus::MergeFrom(from._internal_seatoccupancystatus());
    }
    if (cached_has_bits & 0x00000004u) {
      _internal_mutable_beltbucklestatus()->::pb::ap_vehstatesigprovider::belt_buckle_status::BeltBuckleStatus::MergeFrom(from._internal_beltbucklestatus());
    }
    if (cached_has_bits & 0x00000008u) {
      uiversionnumber_ = from.uiversionnumber_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void VehicleOccupancyStatusPort::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void VehicleOccupancyStatusPort::CopyFrom(const VehicleOccupancyStatusPort& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool VehicleOccupancyStatusPort::IsInitialized() const {
  return true;
}

void VehicleOccupancyStatusPort::InternalSwap(VehicleOccupancyStatusPort* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(ssigheader_, other->ssigheader_);
  swap(seatoccupancystatus_, other->seatoccupancystatus_);
  swap(beltbucklestatus_, other->beltbucklestatus_);
  swap(uiversionnumber_, other->uiversionnumber_);
}

::PROTOBUF_NAMESPACE_ID::Metadata VehicleOccupancyStatusPort::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void VehicleOccupancyStatusPort_array_port::InitAsDefaultInstance() {
}
class VehicleOccupancyStatusPort_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<VehicleOccupancyStatusPort_array_port>()._has_bits_);
};

VehicleOccupancyStatusPort_array_port::VehicleOccupancyStatusPort_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
}
VehicleOccupancyStatusPort_array_port::VehicleOccupancyStatusPort_array_port(const VehicleOccupancyStatusPort_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
}

void VehicleOccupancyStatusPort_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_VehicleOccupancyStatusPort_array_port_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto.base);
}

VehicleOccupancyStatusPort_array_port::~VehicleOccupancyStatusPort_array_port() {
  // @@protoc_insertion_point(destructor:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
  SharedDtor();
}

void VehicleOccupancyStatusPort_array_port::SharedDtor() {
}

void VehicleOccupancyStatusPort_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const VehicleOccupancyStatusPort_array_port& VehicleOccupancyStatusPort_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_VehicleOccupancyStatusPort_array_port_ap_5fvehstatesigprovider_2fvehicle_5foccupancy_5fstatus_5fport_2eproto.base);
  return *internal_default_instance();
}


void VehicleOccupancyStatusPort_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* VehicleOccupancyStatusPort_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort data = 341;
      case 341:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 170)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<2730>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* VehicleOccupancyStatusPort_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort data = 341;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(341, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
  return target;
}

size_t VehicleOccupancyStatusPort_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort data = 341;
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

void VehicleOccupancyStatusPort_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const VehicleOccupancyStatusPort_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<VehicleOccupancyStatusPort_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
    MergeFrom(*source);
  }
}

void VehicleOccupancyStatusPort_array_port::MergeFrom(const VehicleOccupancyStatusPort_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void VehicleOccupancyStatusPort_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void VehicleOccupancyStatusPort_array_port::CopyFrom(const VehicleOccupancyStatusPort_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_vehstatesigprovider.vehicle_occupancy_status_port.VehicleOccupancyStatusPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool VehicleOccupancyStatusPort_array_port::IsInitialized() const {
  return true;
}

void VehicleOccupancyStatusPort_array_port::InternalSwap(VehicleOccupancyStatusPort_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata VehicleOccupancyStatusPort_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace vehicle_occupancy_status_port
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort* Arena::CreateMaybeMessage< ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort_array_port* Arena::CreateMaybeMessage< ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_vehstatesigprovider::vehicle_occupancy_status_port::VehicleOccupancyStatusPort_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
