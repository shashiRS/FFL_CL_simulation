// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vc/screen_switch_data_port.proto

#include "vc/screen_switch_data_port.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_vc_2fscreen_5fswitch_5fdata_5fport_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ScreenSwitchDataPort_vc_2fscreen_5fswitch_5fdata_5fport_2eproto;
namespace pb {
namespace vc {
namespace screen_switch_data_port {
class ScreenSwitchDataPortDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ScreenSwitchDataPort> _instance;
} _ScreenSwitchDataPort_default_instance_;
class ScreenSwitchDataPort_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ScreenSwitchDataPort_array_port> _instance;
} _ScreenSwitchDataPort_array_port_default_instance_;
}  // namespace screen_switch_data_port
}  // namespace vc
}  // namespace pb
static void InitDefaultsscc_info_ScreenSwitchDataPort_vc_2fscreen_5fswitch_5fdata_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::vc::screen_switch_data_port::_ScreenSwitchDataPort_default_instance_;
    new (ptr) ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ScreenSwitchDataPort_vc_2fscreen_5fswitch_5fdata_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_ScreenSwitchDataPort_vc_2fscreen_5fswitch_5fdata_5fport_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,}};

static void InitDefaultsscc_info_ScreenSwitchDataPort_array_port_vc_2fscreen_5fswitch_5fdata_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::vc::screen_switch_data_port::_ScreenSwitchDataPort_array_port_default_instance_;
    new (ptr) ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ScreenSwitchDataPort_array_port_vc_2fscreen_5fswitch_5fdata_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_ScreenSwitchDataPort_array_port_vc_2fscreen_5fswitch_5fdata_5fport_2eproto}, {
      &scc_info_ScreenSwitchDataPort_vc_2fscreen_5fswitch_5fdata_5fport_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_vc_2fscreen_5fswitch_5fdata_5fport_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_vc_2fscreen_5fswitch_5fdata_5fport_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_vc_2fscreen_5fswitch_5fdata_5fport_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_vc_2fscreen_5fswitch_5fdata_5fport_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort, uiversionnumber_),
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort, ssigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort, hmioutuseractscreenreq_u8_),
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort, blindspotviewtype_),
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort, currentviewmode_),
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort, clusterscreenresponse_nu_u8_),
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort, deactivateview_),
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort, transparencypreset_),
  5,
  0,
  3,
  4,
  6,
  7,
  1,
  2,
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 13, sizeof(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort)},
  { 21, 27, sizeof(::pb::vc::screen_switch_data_port::ScreenSwitchDataPort_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::vc::screen_switch_data_port::_ScreenSwitchDataPort_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::vc::screen_switch_data_port::_ScreenSwitchDataPort_array_port_default_instance_),
};

const char descriptor_table_protodef_vc_2fscreen_5fswitch_5fdata_5fport_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n vc/screen_switch_data_port.proto\022\035pb.v"
  "c.screen_switch_data_port\032\027eco/signal_he"
  "ader.proto\032\035ap_hmitoap/screen_types.prot"
  "o\032\037vc/blind_spot_view_status.proto\032\034vc/t"
  "ransparency_preset.proto\"\323\003\n\024ScreenSwitc"
  "hDataPort\022\030\n\017uiVersionNumber\030\314\020 \001(\r\0227\n\ns"
  "SigHeader\030\211\010 \001(\0132\".pb.eco.signal_header."
  "SignalHeader\022K\n\031HmiOutUserActScreenReq_u"
  "8\030\362\004 \001(\0162\'.pb.ap_hmitoap.screen_types.Sc"
  "reenTypes\022M\n\021blindSpotViewType\030\363\006 \001(\01621."
  "pb.vc.blind_spot_view_status.BlindSpotVi"
  "ewStatus\022A\n\017currentViewMode\030\365\025 \001(\0162\'.pb."
  "ap_hmitoap.screen_types.ScreenTypes\022$\n\033C"
  "lusterScreenResponse_nu_u8\030\324\030 \001(\r\022\027\n\016dea"
  "ctivateView\030\317\035 \001(\010\022J\n\022transparencyPreset"
  "\030\356\002 \001(\0162-.pb.vc.transparency_preset.Tran"
  "sparencyPreset\"e\n\037ScreenSwitchDataPort_a"
  "rray_port\022B\n\004data\030\367\036 \003(\01323.pb.vc.screen_"
  "switch_data_port.ScreenSwitchDataPort"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto_deps[4] = {
  &::descriptor_table_ap_5fhmitoap_2fscreen_5ftypes_2eproto,
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
  &::descriptor_table_vc_2fblind_5fspot_5fview_5fstatus_2eproto,
  &::descriptor_table_vc_2ftransparency_5fpreset_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto_sccs[2] = {
  &scc_info_ScreenSwitchDataPort_vc_2fscreen_5fswitch_5fdata_5fport_2eproto.base,
  &scc_info_ScreenSwitchDataPort_array_port_vc_2fscreen_5fswitch_5fdata_5fport_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto_once;
static bool descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto = {
  &descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto_initialized, descriptor_table_protodef_vc_2fscreen_5fswitch_5fdata_5fport_2eproto, "vc/screen_switch_data_port.proto", 757,
  &descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto_once, descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto_sccs, descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto_deps, 2, 4,
  schemas, file_default_instances, TableStruct_vc_2fscreen_5fswitch_5fdata_5fport_2eproto::offsets,
  file_level_metadata_vc_2fscreen_5fswitch_5fdata_5fport_2eproto, 2, file_level_enum_descriptors_vc_2fscreen_5fswitch_5fdata_5fport_2eproto, file_level_service_descriptors_vc_2fscreen_5fswitch_5fdata_5fport_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_vc_2fscreen_5fswitch_5fdata_5fport_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_vc_2fscreen_5fswitch_5fdata_5fport_2eproto), true);
namespace pb {
namespace vc {
namespace screen_switch_data_port {

// ===================================================================

void ScreenSwitchDataPort::InitAsDefaultInstance() {
  ::pb::vc::screen_switch_data_port::_ScreenSwitchDataPort_default_instance_._instance.get_mutable()->ssigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
}
class ScreenSwitchDataPort::_Internal {
 public:
  using HasBits = decltype(std::declval<ScreenSwitchDataPort>()._has_bits_);
  static void set_has_uiversionnumber(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static const ::pb::eco::signal_header::SignalHeader& ssigheader(const ScreenSwitchDataPort* msg);
  static void set_has_ssigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_hmioutuseractscreenreq_u8(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_blindspotviewtype(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_currentviewmode(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_clusterscreenresponse_nu_u8(HasBits* has_bits) {
    (*has_bits)[0] |= 128u;
  }
  static void set_has_deactivateview(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_transparencypreset(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
ScreenSwitchDataPort::_Internal::ssigheader(const ScreenSwitchDataPort* msg) {
  return *msg->ssigheader_;
}
void ScreenSwitchDataPort::clear_ssigheader() {
  if (ssigheader_ != nullptr) ssigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
ScreenSwitchDataPort::ScreenSwitchDataPort()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
}
ScreenSwitchDataPort::ScreenSwitchDataPort(const ScreenSwitchDataPort& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_ssigheader()) {
    ssigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.ssigheader_);
  } else {
    ssigheader_ = nullptr;
  }
  ::memcpy(&deactivateview_, &from.deactivateview_,
    static_cast<size_t>(reinterpret_cast<char*>(&clusterscreenresponse_nu_u8_) -
    reinterpret_cast<char*>(&deactivateview_)) + sizeof(clusterscreenresponse_nu_u8_));
  // @@protoc_insertion_point(copy_constructor:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
}

void ScreenSwitchDataPort::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ScreenSwitchDataPort_vc_2fscreen_5fswitch_5fdata_5fport_2eproto.base);
  ::memset(&ssigheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&clusterscreenresponse_nu_u8_) -
      reinterpret_cast<char*>(&ssigheader_)) + sizeof(clusterscreenresponse_nu_u8_));
}

ScreenSwitchDataPort::~ScreenSwitchDataPort() {
  // @@protoc_insertion_point(destructor:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
  SharedDtor();
}

void ScreenSwitchDataPort::SharedDtor() {
  if (this != internal_default_instance()) delete ssigheader_;
}

void ScreenSwitchDataPort::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ScreenSwitchDataPort& ScreenSwitchDataPort::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ScreenSwitchDataPort_vc_2fscreen_5fswitch_5fdata_5fport_2eproto.base);
  return *internal_default_instance();
}


void ScreenSwitchDataPort::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(ssigheader_ != nullptr);
    ssigheader_->Clear();
  }
  if (cached_has_bits & 0x000000feu) {
    ::memset(&deactivateview_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&clusterscreenresponse_nu_u8_) -
        reinterpret_cast<char*>(&deactivateview_)) + sizeof(clusterscreenresponse_nu_u8_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ScreenSwitchDataPort::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .pb.vc.transparency_preset.TransparencyPreset transparencyPreset = 366;
      case 366:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 112)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::vc::transparency_preset::TransparencyPreset_IsValid(val))) {
            _internal_set_transparencypreset(static_cast<::pb::vc::transparency_preset::TransparencyPreset>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(366, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_hmitoap.screen_types.ScreenTypes HmiOutUserActScreenReq_u8 = 626;
      case 626:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 144)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_hmitoap::screen_types::ScreenTypes_IsValid(val))) {
            _internal_set_hmioutuseractscreenreq_u8(static_cast<::pb::ap_hmitoap::screen_types::ScreenTypes>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(626, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.vc.blind_spot_view_status.BlindSpotViewStatus blindSpotViewType = 883;
      case 883:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 152)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::vc::blind_spot_view_status::BlindSpotViewStatus_IsValid(val))) {
            _internal_set_blindspotviewtype(static_cast<::pb::vc::blind_spot_view_status::BlindSpotViewStatus>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(883, val, mutable_unknown_fields());
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
      // optional uint32 uiVersionNumber = 2124;
      case 2124:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 96)) {
          _Internal::set_has_uiversionnumber(&has_bits);
          uiversionnumber_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_hmitoap.screen_types.ScreenTypes currentViewMode = 2805;
      case 2805:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 168)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_hmitoap::screen_types::ScreenTypes_IsValid(val))) {
            _internal_set_currentviewmode(static_cast<::pb::ap_hmitoap::screen_types::ScreenTypes>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(2805, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional uint32 ClusterScreenResponse_nu_u8 = 3156;
      case 3156:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 160)) {
          _Internal::set_has_clusterscreenresponse_nu_u8(&has_bits);
          clusterscreenresponse_nu_u8_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool deactivateView = 3791;
      case 3791:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 120)) {
          _Internal::set_has_deactivateview(&has_bits);
          deactivateview_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* ScreenSwitchDataPort::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.vc.transparency_preset.TransparencyPreset transparencyPreset = 366;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      366, this->_internal_transparencypreset(), target);
  }

  // optional .pb.ap_hmitoap.screen_types.ScreenTypes HmiOutUserActScreenReq_u8 = 626;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      626, this->_internal_hmioutuseractscreenreq_u8(), target);
  }

  // optional .pb.vc.blind_spot_view_status.BlindSpotViewStatus blindSpotViewType = 883;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      883, this->_internal_blindspotviewtype(), target);
  }

  // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1033, _Internal::ssigheader(this), target, stream);
  }

  // optional uint32 uiVersionNumber = 2124;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2124, this->_internal_uiversionnumber(), target);
  }

  // optional .pb.ap_hmitoap.screen_types.ScreenTypes currentViewMode = 2805;
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      2805, this->_internal_currentviewmode(), target);
  }

  // optional uint32 ClusterScreenResponse_nu_u8 = 3156;
  if (cached_has_bits & 0x00000080u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(3156, this->_internal_clusterscreenresponse_nu_u8(), target);
  }

  // optional bool deactivateView = 3791;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(3791, this->_internal_deactivateview(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
  return target;
}

size_t ScreenSwitchDataPort::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *ssigheader_);
    }

    // optional bool deactivateView = 3791;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 + 1;
    }

    // optional .pb.vc.transparency_preset.TransparencyPreset transparencyPreset = 366;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_transparencypreset());
    }

    // optional .pb.ap_hmitoap.screen_types.ScreenTypes HmiOutUserActScreenReq_u8 = 626;
    if (cached_has_bits & 0x00000008u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_hmioutuseractscreenreq_u8());
    }

    // optional .pb.vc.blind_spot_view_status.BlindSpotViewStatus blindSpotViewType = 883;
    if (cached_has_bits & 0x00000010u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_blindspotviewtype());
    }

    // optional uint32 uiVersionNumber = 2124;
    if (cached_has_bits & 0x00000020u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_uiversionnumber());
    }

    // optional .pb.ap_hmitoap.screen_types.ScreenTypes currentViewMode = 2805;
    if (cached_has_bits & 0x00000040u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_currentviewmode());
    }

    // optional uint32 ClusterScreenResponse_nu_u8 = 3156;
    if (cached_has_bits & 0x00000080u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_clusterscreenresponse_nu_u8());
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

void ScreenSwitchDataPort::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
  GOOGLE_DCHECK_NE(&from, this);
  const ScreenSwitchDataPort* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ScreenSwitchDataPort>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
    MergeFrom(*source);
  }
}

void ScreenSwitchDataPort::MergeFrom(const ScreenSwitchDataPort& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_ssigheader()->::pb::eco::signal_header::SignalHeader::MergeFrom(from._internal_ssigheader());
    }
    if (cached_has_bits & 0x00000002u) {
      deactivateview_ = from.deactivateview_;
    }
    if (cached_has_bits & 0x00000004u) {
      transparencypreset_ = from.transparencypreset_;
    }
    if (cached_has_bits & 0x00000008u) {
      hmioutuseractscreenreq_u8_ = from.hmioutuseractscreenreq_u8_;
    }
    if (cached_has_bits & 0x00000010u) {
      blindspotviewtype_ = from.blindspotviewtype_;
    }
    if (cached_has_bits & 0x00000020u) {
      uiversionnumber_ = from.uiversionnumber_;
    }
    if (cached_has_bits & 0x00000040u) {
      currentviewmode_ = from.currentviewmode_;
    }
    if (cached_has_bits & 0x00000080u) {
      clusterscreenresponse_nu_u8_ = from.clusterscreenresponse_nu_u8_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ScreenSwitchDataPort::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ScreenSwitchDataPort::CopyFrom(const ScreenSwitchDataPort& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ScreenSwitchDataPort::IsInitialized() const {
  return true;
}

void ScreenSwitchDataPort::InternalSwap(ScreenSwitchDataPort* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(ssigheader_, other->ssigheader_);
  swap(deactivateview_, other->deactivateview_);
  swap(transparencypreset_, other->transparencypreset_);
  swap(hmioutuseractscreenreq_u8_, other->hmioutuseractscreenreq_u8_);
  swap(blindspotviewtype_, other->blindspotviewtype_);
  swap(uiversionnumber_, other->uiversionnumber_);
  swap(currentviewmode_, other->currentviewmode_);
  swap(clusterscreenresponse_nu_u8_, other->clusterscreenresponse_nu_u8_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ScreenSwitchDataPort::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void ScreenSwitchDataPort_array_port::InitAsDefaultInstance() {
}
class ScreenSwitchDataPort_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<ScreenSwitchDataPort_array_port>()._has_bits_);
};

ScreenSwitchDataPort_array_port::ScreenSwitchDataPort_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
}
ScreenSwitchDataPort_array_port::ScreenSwitchDataPort_array_port(const ScreenSwitchDataPort_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
}

void ScreenSwitchDataPort_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ScreenSwitchDataPort_array_port_vc_2fscreen_5fswitch_5fdata_5fport_2eproto.base);
}

ScreenSwitchDataPort_array_port::~ScreenSwitchDataPort_array_port() {
  // @@protoc_insertion_point(destructor:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
  SharedDtor();
}

void ScreenSwitchDataPort_array_port::SharedDtor() {
}

void ScreenSwitchDataPort_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ScreenSwitchDataPort_array_port& ScreenSwitchDataPort_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ScreenSwitchDataPort_array_port_vc_2fscreen_5fswitch_5fdata_5fport_2eproto.base);
  return *internal_default_instance();
}


void ScreenSwitchDataPort_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ScreenSwitchDataPort_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.vc.screen_switch_data_port.ScreenSwitchDataPort data = 3959;
      case 3959:
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

::PROTOBUF_NAMESPACE_ID::uint8* ScreenSwitchDataPort_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.vc.screen_switch_data_port.ScreenSwitchDataPort data = 3959;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3959, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
  return target;
}

size_t ScreenSwitchDataPort_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.vc.screen_switch_data_port.ScreenSwitchDataPort data = 3959;
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

void ScreenSwitchDataPort_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const ScreenSwitchDataPort_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ScreenSwitchDataPort_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
    MergeFrom(*source);
  }
}

void ScreenSwitchDataPort_array_port::MergeFrom(const ScreenSwitchDataPort_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void ScreenSwitchDataPort_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ScreenSwitchDataPort_array_port::CopyFrom(const ScreenSwitchDataPort_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.vc.screen_switch_data_port.ScreenSwitchDataPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ScreenSwitchDataPort_array_port::IsInitialized() const {
  return true;
}

void ScreenSwitchDataPort_array_port::InternalSwap(ScreenSwitchDataPort_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ScreenSwitchDataPort_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace screen_switch_data_port
}  // namespace vc
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort* Arena::CreateMaybeMessage< ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort >(Arena* arena) {
  return Arena::CreateInternal< ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort_array_port* Arena::CreateMaybeMessage< ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::vc::screen_switch_data_port::ScreenSwitchDataPort_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>