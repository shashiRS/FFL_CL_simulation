// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_psm_app/psmdebug_port.proto

#include "ap_psm_app/psmdebug_port.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_PSMDebugPort_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_eco_2fsignal_5fheader_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto;
namespace pb {
namespace ap_psm_app {
namespace psmdebug_port {
class PSMDebugPortDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<PSMDebugPort> _instance;
} _PSMDebugPort_default_instance_;
class PSMDebugPort_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<PSMDebugPort_array_port> _instance;
} _PSMDebugPort_array_port_default_instance_;
}  // namespace psmdebug_port
}  // namespace ap_psm_app
}  // namespace pb
static void InitDefaultsscc_info_PSMDebugPort_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_psm_app::psmdebug_port::_PSMDebugPort_default_instance_;
    new (ptr) ::pb::ap_psm_app::psmdebug_port::PSMDebugPort();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_psm_app::psmdebug_port::PSMDebugPort::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_PSMDebugPort_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_PSMDebugPort_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,}};

static void InitDefaultsscc_info_PSMDebugPort_array_port_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_psm_app::psmdebug_port::_PSMDebugPort_array_port_default_instance_;
    new (ptr) ::pb::ap_psm_app::psmdebug_port::PSMDebugPort_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_psm_app::psmdebug_port::PSMDebugPort_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_PSMDebugPort_array_port_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_PSMDebugPort_array_port_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto}, {
      &scc_info_PSMDebugPort_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort, uiversionnumber_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort, ssigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort, debugint_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort, debugfloat_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort, statevarppc_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort, statevaresm_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort, statevarvsm_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort, statevardm_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort, statevarrdm_nu_),
  1,
  0,
  ~0u,
  ~0u,
  4,
  6,
  5,
  2,
  3,
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_psm_app::psmdebug_port::PSMDebugPort_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 14, sizeof(::pb::ap_psm_app::psmdebug_port::PSMDebugPort)},
  { 23, 29, sizeof(::pb::ap_psm_app::psmdebug_port::PSMDebugPort_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_psm_app::psmdebug_port::_PSMDebugPort_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_psm_app::psmdebug_port::_PSMDebugPort_array_port_default_instance_),
};

const char descriptor_table_protodef_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\036ap_psm_app/psmdebug_port.proto\022\033pb.ap_"
  "psm_app.psmdebug_port\032\027eco/signal_header"
  ".proto\032\031ap_psm_app/ppcstate.proto\032\031ap_ps"
  "m_app/esmstate.proto\032\031ap_psm_app/vsmstat"
  "e.proto\032\030ap_psm_app/dmstate.proto\032\031ap_ps"
  "m_app/rdmstate.proto\"\255\003\n\014PSMDebugPort\022\030\n"
  "\017uiVersionNumber\030\314\020 \001(\r\0227\n\nsSigHeader\030\211\010"
  " \001(\0132\".pb.eco.signal_header.SignalHeader"
  "\022\021\n\010debugInt\030\316\t \003(\021\022\023\n\ndebugFloat\030\371\017 \003(\002"
  "\0229\n\016stateVarPPC_nu\030\372\024 \001(\0162 .pb.ap_psm_ap"
  "p.ppcstate.PPCState\0229\n\016stateVarESM_nu\030\265\036"
  " \001(\0162 .pb.ap_psm_app.esmstate.ESMState\0229"
  "\n\016stateVarVSM_nu\030\265\026 \001(\0162 .pb.ap_psm_app."
  "vsmstate.VSMState\0226\n\rstateVarDM_nu\030\270\022 \001("
  "\0162\036.pb.ap_psm_app.dmstate.DMState\0229\n\016sta"
  "teVarRDM_nu\030\345\024 \001(\0162 .pb.ap_psm_app.rdmst"
  "ate.RDMState\"S\n\027PSMDebugPort_array_port\022"
  "8\n\004data\030\205\004 \003(\0132).pb.ap_psm_app.psmdebug_"
  "port.PSMDebugPort"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto_deps[6] = {
  &::descriptor_table_ap_5fpsm_5fapp_2fdmstate_2eproto,
  &::descriptor_table_ap_5fpsm_5fapp_2fesmstate_2eproto,
  &::descriptor_table_ap_5fpsm_5fapp_2fppcstate_2eproto,
  &::descriptor_table_ap_5fpsm_5fapp_2frdmstate_2eproto,
  &::descriptor_table_ap_5fpsm_5fapp_2fvsmstate_2eproto,
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto_sccs[2] = {
  &scc_info_PSMDebugPort_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto.base,
  &scc_info_PSMDebugPort_array_port_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto_once;
static bool descriptor_table_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto = {
  &descriptor_table_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto_initialized, descriptor_table_protodef_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto, "ap_psm_app/psmdebug_port.proto", 737,
  &descriptor_table_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto_once, descriptor_table_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto_sccs, descriptor_table_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto_deps, 2, 6,
  schemas, file_default_instances, TableStruct_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto::offsets,
  file_level_metadata_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto, 2, file_level_enum_descriptors_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto, file_level_service_descriptors_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto), true);
namespace pb {
namespace ap_psm_app {
namespace psmdebug_port {

// ===================================================================

void PSMDebugPort::InitAsDefaultInstance() {
  ::pb::ap_psm_app::psmdebug_port::_PSMDebugPort_default_instance_._instance.get_mutable()->ssigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
}
class PSMDebugPort::_Internal {
 public:
  using HasBits = decltype(std::declval<PSMDebugPort>()._has_bits_);
  static void set_has_uiversionnumber(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::pb::eco::signal_header::SignalHeader& ssigheader(const PSMDebugPort* msg);
  static void set_has_ssigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_statevarppc_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_statevaresm_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_statevarvsm_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_statevardm_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_statevarrdm_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
PSMDebugPort::_Internal::ssigheader(const PSMDebugPort* msg) {
  return *msg->ssigheader_;
}
void PSMDebugPort::clear_ssigheader() {
  if (ssigheader_ != nullptr) ssigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
PSMDebugPort::PSMDebugPort()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
}
PSMDebugPort::PSMDebugPort(const PSMDebugPort& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      debugint_(from.debugint_),
      debugfloat_(from.debugfloat_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_ssigheader()) {
    ssigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.ssigheader_);
  } else {
    ssigheader_ = nullptr;
  }
  ::memcpy(&uiversionnumber_, &from.uiversionnumber_,
    static_cast<size_t>(reinterpret_cast<char*>(&statevaresm_nu_) -
    reinterpret_cast<char*>(&uiversionnumber_)) + sizeof(statevaresm_nu_));
  // @@protoc_insertion_point(copy_constructor:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
}

void PSMDebugPort::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_PSMDebugPort_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto.base);
  ::memset(&ssigheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&statevaresm_nu_) -
      reinterpret_cast<char*>(&ssigheader_)) + sizeof(statevaresm_nu_));
}

PSMDebugPort::~PSMDebugPort() {
  // @@protoc_insertion_point(destructor:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
  SharedDtor();
}

void PSMDebugPort::SharedDtor() {
  if (this != internal_default_instance()) delete ssigheader_;
}

void PSMDebugPort::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const PSMDebugPort& PSMDebugPort::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_PSMDebugPort_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto.base);
  return *internal_default_instance();
}


void PSMDebugPort::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  debugint_.Clear();
  debugfloat_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(ssigheader_ != nullptr);
    ssigheader_->Clear();
  }
  if (cached_has_bits & 0x0000007eu) {
    ::memset(&uiversionnumber_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&statevaresm_nu_) -
        reinterpret_cast<char*>(&uiversionnumber_)) + sizeof(statevaresm_nu_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* PSMDebugPort::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // repeated sint32 debugInt = 1230;
      case 1230:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 112)) {
          ptr -= 2;
          do {
            ptr += 2;
            _internal_add_debugint(::PROTOBUF_NAMESPACE_ID::internal::ReadVarintZigZag32(&ptr));
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<9840>(ptr));
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 114) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedSInt32Parser(_internal_mutable_debugint(), ptr, ctx);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated float debugFloat = 2041;
      case 2041:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 205)) {
          ptr -= 2;
          do {
            ptr += 2;
            _internal_add_debugfloat(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr));
            ptr += sizeof(float);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<16333>(ptr));
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 202) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedFloatParser(_internal_mutable_debugfloat(), ptr, ctx);
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
      // optional .pb.ap_psm_app.dmstate.DMState stateVarDM_nu = 2360;
      case 2360:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 192)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_psm_app::dmstate::DMState_IsValid(val))) {
            _internal_set_statevardm_nu(static_cast<::pb::ap_psm_app::dmstate::DMState>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(2360, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_psm_app.rdmstate.RDMState stateVarRDM_nu = 2661;
      case 2661:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_psm_app::rdmstate::RDMState_IsValid(val))) {
            _internal_set_statevarrdm_nu(static_cast<::pb::ap_psm_app::rdmstate::RDMState>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(2661, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_psm_app.ppcstate.PPCState stateVarPPC_nu = 2682;
      case 2682:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 208)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_psm_app::ppcstate::PPCState_IsValid(val))) {
            _internal_set_statevarppc_nu(static_cast<::pb::ap_psm_app::ppcstate::PPCState>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(2682, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_psm_app.vsmstate.VSMState stateVarVSM_nu = 2869;
      case 2869:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 168)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_psm_app::vsmstate::VSMState_IsValid(val))) {
            _internal_set_statevarvsm_nu(static_cast<::pb::ap_psm_app::vsmstate::VSMState>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(2869, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .pb.ap_psm_app.esmstate.ESMState stateVarESM_nu = 3893;
      case 3893:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 168)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::ap_psm_app::esmstate::ESMState_IsValid(val))) {
            _internal_set_statevaresm_nu(static_cast<::pb::ap_psm_app::esmstate::ESMState>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(3893, val, mutable_unknown_fields());
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

::PROTOBUF_NAMESPACE_ID::uint8* PSMDebugPort::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
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

  // repeated sint32 debugInt = 1230;
  for (int i = 0, n = this->_internal_debugint_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteSInt32ToArray(1230, this->_internal_debugint(i), target);
  }

  // repeated float debugFloat = 2041;
  for (int i = 0, n = this->_internal_debugfloat_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2041, this->_internal_debugfloat(i), target);
  }

  // optional uint32 uiVersionNumber = 2124;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2124, this->_internal_uiversionnumber(), target);
  }

  // optional .pb.ap_psm_app.dmstate.DMState stateVarDM_nu = 2360;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      2360, this->_internal_statevardm_nu(), target);
  }

  // optional .pb.ap_psm_app.rdmstate.RDMState stateVarRDM_nu = 2661;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      2661, this->_internal_statevarrdm_nu(), target);
  }

  // optional .pb.ap_psm_app.ppcstate.PPCState stateVarPPC_nu = 2682;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      2682, this->_internal_statevarppc_nu(), target);
  }

  // optional .pb.ap_psm_app.vsmstate.VSMState stateVarVSM_nu = 2869;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      2869, this->_internal_statevarvsm_nu(), target);
  }

  // optional .pb.ap_psm_app.esmstate.ESMState stateVarESM_nu = 3893;
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      3893, this->_internal_statevaresm_nu(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
  return target;
}

size_t PSMDebugPort::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated sint32 debugInt = 1230;
  {
    size_t data_size = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      SInt32Size(this->debugint_);
    total_size += 2 *
                  ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(this->_internal_debugint_size());
    total_size += data_size;
  }

  // repeated float debugFloat = 2041;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_debugfloat_size());
    size_t data_size = 4UL * count;
    total_size += 2 *
                  ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(this->_internal_debugfloat_size());
    total_size += data_size;
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000007fu) {
    // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *ssigheader_);
    }

    // optional uint32 uiVersionNumber = 2124;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_uiversionnumber());
    }

    // optional .pb.ap_psm_app.dmstate.DMState stateVarDM_nu = 2360;
    if (cached_has_bits & 0x00000004u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_statevardm_nu());
    }

    // optional .pb.ap_psm_app.rdmstate.RDMState stateVarRDM_nu = 2661;
    if (cached_has_bits & 0x00000008u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_statevarrdm_nu());
    }

    // optional .pb.ap_psm_app.ppcstate.PPCState stateVarPPC_nu = 2682;
    if (cached_has_bits & 0x00000010u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_statevarppc_nu());
    }

    // optional .pb.ap_psm_app.vsmstate.VSMState stateVarVSM_nu = 2869;
    if (cached_has_bits & 0x00000020u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_statevarvsm_nu());
    }

    // optional .pb.ap_psm_app.esmstate.ESMState stateVarESM_nu = 3893;
    if (cached_has_bits & 0x00000040u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_statevaresm_nu());
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

void PSMDebugPort::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
  GOOGLE_DCHECK_NE(&from, this);
  const PSMDebugPort* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<PSMDebugPort>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
    MergeFrom(*source);
  }
}

void PSMDebugPort::MergeFrom(const PSMDebugPort& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  debugint_.MergeFrom(from.debugint_);
  debugfloat_.MergeFrom(from.debugfloat_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000007fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_ssigheader()->::pb::eco::signal_header::SignalHeader::MergeFrom(from._internal_ssigheader());
    }
    if (cached_has_bits & 0x00000002u) {
      uiversionnumber_ = from.uiversionnumber_;
    }
    if (cached_has_bits & 0x00000004u) {
      statevardm_nu_ = from.statevardm_nu_;
    }
    if (cached_has_bits & 0x00000008u) {
      statevarrdm_nu_ = from.statevarrdm_nu_;
    }
    if (cached_has_bits & 0x00000010u) {
      statevarppc_nu_ = from.statevarppc_nu_;
    }
    if (cached_has_bits & 0x00000020u) {
      statevarvsm_nu_ = from.statevarvsm_nu_;
    }
    if (cached_has_bits & 0x00000040u) {
      statevaresm_nu_ = from.statevaresm_nu_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void PSMDebugPort::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PSMDebugPort::CopyFrom(const PSMDebugPort& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PSMDebugPort::IsInitialized() const {
  return true;
}

void PSMDebugPort::InternalSwap(PSMDebugPort* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  debugint_.InternalSwap(&other->debugint_);
  debugfloat_.InternalSwap(&other->debugfloat_);
  swap(ssigheader_, other->ssigheader_);
  swap(uiversionnumber_, other->uiversionnumber_);
  swap(statevardm_nu_, other->statevardm_nu_);
  swap(statevarrdm_nu_, other->statevarrdm_nu_);
  swap(statevarppc_nu_, other->statevarppc_nu_);
  swap(statevarvsm_nu_, other->statevarvsm_nu_);
  swap(statevaresm_nu_, other->statevaresm_nu_);
}

::PROTOBUF_NAMESPACE_ID::Metadata PSMDebugPort::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void PSMDebugPort_array_port::InitAsDefaultInstance() {
}
class PSMDebugPort_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<PSMDebugPort_array_port>()._has_bits_);
};

PSMDebugPort_array_port::PSMDebugPort_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
}
PSMDebugPort_array_port::PSMDebugPort_array_port(const PSMDebugPort_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
}

void PSMDebugPort_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_PSMDebugPort_array_port_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto.base);
}

PSMDebugPort_array_port::~PSMDebugPort_array_port() {
  // @@protoc_insertion_point(destructor:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
  SharedDtor();
}

void PSMDebugPort_array_port::SharedDtor() {
}

void PSMDebugPort_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const PSMDebugPort_array_port& PSMDebugPort_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_PSMDebugPort_array_port_ap_5fpsm_5fapp_2fpsmdebug_5fport_2eproto.base);
  return *internal_default_instance();
}


void PSMDebugPort_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* PSMDebugPort_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.ap_psm_app.psmdebug_port.PSMDebugPort data = 517;
      case 517:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<4138>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* PSMDebugPort_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.ap_psm_app.psmdebug_port.PSMDebugPort data = 517;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(517, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
  return target;
}

size_t PSMDebugPort_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.ap_psm_app.psmdebug_port.PSMDebugPort data = 517;
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

void PSMDebugPort_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const PSMDebugPort_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<PSMDebugPort_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
    MergeFrom(*source);
  }
}

void PSMDebugPort_array_port::MergeFrom(const PSMDebugPort_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void PSMDebugPort_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PSMDebugPort_array_port::CopyFrom(const PSMDebugPort_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_psm_app.psmdebug_port.PSMDebugPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PSMDebugPort_array_port::IsInitialized() const {
  return true;
}

void PSMDebugPort_array_port::InternalSwap(PSMDebugPort_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata PSMDebugPort_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace psmdebug_port
}  // namespace ap_psm_app
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::ap_psm_app::psmdebug_port::PSMDebugPort* Arena::CreateMaybeMessage< ::pb::ap_psm_app::psmdebug_port::PSMDebugPort >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_psm_app::psmdebug_port::PSMDebugPort >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::ap_psm_app::psmdebug_port::PSMDebugPort_array_port* Arena::CreateMaybeMessage< ::pb::ap_psm_app::psmdebug_port::PSMDebugPort_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_psm_app::psmdebug_port::PSMDebugPort_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
