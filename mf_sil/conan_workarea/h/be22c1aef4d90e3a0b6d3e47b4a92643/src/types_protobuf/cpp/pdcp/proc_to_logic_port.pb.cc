// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: pdcp/proc_to_logic_port.proto

#include "pdcp/proc_to_logic_port.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_pdcp_2fproc_5fto_5flogic_5fport_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ProcToLogicPort_pdcp_2fproc_5fto_5flogic_5fport_2eproto;
namespace pb {
namespace pdcp {
namespace proc_to_logic_port {
class ProcToLogicPortDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ProcToLogicPort> _instance;
} _ProcToLogicPort_default_instance_;
class ProcToLogicPort_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ProcToLogicPort_array_port> _instance;
} _ProcToLogicPort_array_port_default_instance_;
}  // namespace proc_to_logic_port
}  // namespace pdcp
}  // namespace pb
static void InitDefaultsscc_info_ProcToLogicPort_pdcp_2fproc_5fto_5flogic_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::pdcp::proc_to_logic_port::_ProcToLogicPort_default_instance_;
    new (ptr) ::pb::pdcp::proc_to_logic_port::ProcToLogicPort();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::pdcp::proc_to_logic_port::ProcToLogicPort::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ProcToLogicPort_pdcp_2fproc_5fto_5flogic_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_ProcToLogicPort_pdcp_2fproc_5fto_5flogic_5fport_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,}};

static void InitDefaultsscc_info_ProcToLogicPort_array_port_pdcp_2fproc_5fto_5flogic_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::pdcp::proc_to_logic_port::_ProcToLogicPort_array_port_default_instance_;
    new (ptr) ::pb::pdcp::proc_to_logic_port::ProcToLogicPort_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::pdcp::proc_to_logic_port::ProcToLogicPort_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ProcToLogicPort_array_port_pdcp_2fproc_5fto_5flogic_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_ProcToLogicPort_array_port_pdcp_2fproc_5fto_5flogic_5fport_2eproto}, {
      &scc_info_ProcToLogicPort_pdcp_2fproc_5fto_5flogic_5fport_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_pdcp_2fproc_5fto_5flogic_5fport_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_pdcp_2fproc_5fto_5flogic_5fport_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_pdcp_2fproc_5fto_5flogic_5fport_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_pdcp_2fproc_5fto_5flogic_5fport_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::proc_to_logic_port::ProcToLogicPort, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::proc_to_logic_port::ProcToLogicPort, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::proc_to_logic_port::ProcToLogicPort, uiversionnumber_),
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::proc_to_logic_port::ProcToLogicPort, ssigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::proc_to_logic_port::ProcToLogicPort, minimaldistance_m_),
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::proc_to_logic_port::ProcToLogicPort, processingerror_nu_),
  2,
  0,
  1,
  3,
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::proc_to_logic_port::ProcToLogicPort_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::proc_to_logic_port::ProcToLogicPort_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::pdcp::proc_to_logic_port::ProcToLogicPort_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::pb::pdcp::proc_to_logic_port::ProcToLogicPort)},
  { 13, 19, sizeof(::pb::pdcp::proc_to_logic_port::ProcToLogicPort_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::pdcp::proc_to_logic_port::_ProcToLogicPort_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::pdcp::proc_to_logic_port::_ProcToLogicPort_array_port_default_instance_),
};

const char descriptor_table_protodef_pdcp_2fproc_5fto_5flogic_5fport_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\035pdcp/proc_to_logic_port.proto\022\032pb.pdcp"
  ".proc_to_logic_port\032\027eco/signal_header.p"
  "roto\"\235\001\n\017ProcToLogicPort\022\030\n\017uiVersionNum"
  "ber\030\314\020 \001(\r\0227\n\nsSigHeader\030\211\010 \001(\0132\".pb.eco"
  ".signal_header.SignalHeader\022\032\n\021minimalDi"
  "stance_m\030\322\035 \001(\002\022\033\n\022processingError_nu\030\201\025"
  " \001(\010\"X\n\032ProcToLogicPort_array_port\022:\n\004da"
  "ta\030\310\006 \003(\0132+.pb.pdcp.proc_to_logic_port.P"
  "rocToLogicPort"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_pdcp_2fproc_5fto_5flogic_5fport_2eproto_deps[1] = {
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_pdcp_2fproc_5fto_5flogic_5fport_2eproto_sccs[2] = {
  &scc_info_ProcToLogicPort_pdcp_2fproc_5fto_5flogic_5fport_2eproto.base,
  &scc_info_ProcToLogicPort_array_port_pdcp_2fproc_5fto_5flogic_5fport_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_pdcp_2fproc_5fto_5flogic_5fport_2eproto_once;
static bool descriptor_table_pdcp_2fproc_5fto_5flogic_5fport_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_pdcp_2fproc_5fto_5flogic_5fport_2eproto = {
  &descriptor_table_pdcp_2fproc_5fto_5flogic_5fport_2eproto_initialized, descriptor_table_protodef_pdcp_2fproc_5fto_5flogic_5fport_2eproto, "pdcp/proc_to_logic_port.proto", 334,
  &descriptor_table_pdcp_2fproc_5fto_5flogic_5fport_2eproto_once, descriptor_table_pdcp_2fproc_5fto_5flogic_5fport_2eproto_sccs, descriptor_table_pdcp_2fproc_5fto_5flogic_5fport_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_pdcp_2fproc_5fto_5flogic_5fport_2eproto::offsets,
  file_level_metadata_pdcp_2fproc_5fto_5flogic_5fport_2eproto, 2, file_level_enum_descriptors_pdcp_2fproc_5fto_5flogic_5fport_2eproto, file_level_service_descriptors_pdcp_2fproc_5fto_5flogic_5fport_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_pdcp_2fproc_5fto_5flogic_5fport_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_pdcp_2fproc_5fto_5flogic_5fport_2eproto), true);
namespace pb {
namespace pdcp {
namespace proc_to_logic_port {

// ===================================================================

void ProcToLogicPort::InitAsDefaultInstance() {
  ::pb::pdcp::proc_to_logic_port::_ProcToLogicPort_default_instance_._instance.get_mutable()->ssigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
}
class ProcToLogicPort::_Internal {
 public:
  using HasBits = decltype(std::declval<ProcToLogicPort>()._has_bits_);
  static void set_has_uiversionnumber(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static const ::pb::eco::signal_header::SignalHeader& ssigheader(const ProcToLogicPort* msg);
  static void set_has_ssigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_minimaldistance_m(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_processingerror_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
ProcToLogicPort::_Internal::ssigheader(const ProcToLogicPort* msg) {
  return *msg->ssigheader_;
}
void ProcToLogicPort::clear_ssigheader() {
  if (ssigheader_ != nullptr) ssigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
ProcToLogicPort::ProcToLogicPort()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
}
ProcToLogicPort::ProcToLogicPort(const ProcToLogicPort& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_ssigheader()) {
    ssigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.ssigheader_);
  } else {
    ssigheader_ = nullptr;
  }
  ::memcpy(&minimaldistance_m_, &from.minimaldistance_m_,
    static_cast<size_t>(reinterpret_cast<char*>(&processingerror_nu_) -
    reinterpret_cast<char*>(&minimaldistance_m_)) + sizeof(processingerror_nu_));
  // @@protoc_insertion_point(copy_constructor:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
}

void ProcToLogicPort::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ProcToLogicPort_pdcp_2fproc_5fto_5flogic_5fport_2eproto.base);
  ::memset(&ssigheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&processingerror_nu_) -
      reinterpret_cast<char*>(&ssigheader_)) + sizeof(processingerror_nu_));
}

ProcToLogicPort::~ProcToLogicPort() {
  // @@protoc_insertion_point(destructor:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
  SharedDtor();
}

void ProcToLogicPort::SharedDtor() {
  if (this != internal_default_instance()) delete ssigheader_;
}

void ProcToLogicPort::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ProcToLogicPort& ProcToLogicPort::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ProcToLogicPort_pdcp_2fproc_5fto_5flogic_5fport_2eproto.base);
  return *internal_default_instance();
}


void ProcToLogicPort::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(ssigheader_ != nullptr);
    ssigheader_->Clear();
  }
  if (cached_has_bits & 0x0000000eu) {
    ::memset(&minimaldistance_m_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&processingerror_nu_) -
        reinterpret_cast<char*>(&minimaldistance_m_)) + sizeof(processingerror_nu_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ProcToLogicPort::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // optional uint32 uiVersionNumber = 2124;
      case 2124:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 96)) {
          _Internal::set_has_uiversionnumber(&has_bits);
          uiversionnumber_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool processingError_nu = 2689;
      case 2689:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_processingerror_nu(&has_bits);
          processingerror_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional float minimalDistance_m = 3794;
      case 3794:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 149)) {
          _Internal::set_has_minimaldistance_m(&has_bits);
          minimaldistance_m_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* ProcToLogicPort::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
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

  // optional uint32 uiVersionNumber = 2124;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2124, this->_internal_uiversionnumber(), target);
  }

  // optional bool processingError_nu = 2689;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2689, this->_internal_processingerror_nu(), target);
  }

  // optional float minimalDistance_m = 3794;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(3794, this->_internal_minimaldistance_m(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
  return target;
}

size_t ProcToLogicPort::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
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

    // optional float minimalDistance_m = 3794;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 + 4;
    }

    // optional uint32 uiVersionNumber = 2124;
    if (cached_has_bits & 0x00000004u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_uiversionnumber());
    }

    // optional bool processingError_nu = 2689;
    if (cached_has_bits & 0x00000008u) {
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

void ProcToLogicPort::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
  GOOGLE_DCHECK_NE(&from, this);
  const ProcToLogicPort* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ProcToLogicPort>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
    MergeFrom(*source);
  }
}

void ProcToLogicPort::MergeFrom(const ProcToLogicPort& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
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
      minimaldistance_m_ = from.minimaldistance_m_;
    }
    if (cached_has_bits & 0x00000004u) {
      uiversionnumber_ = from.uiversionnumber_;
    }
    if (cached_has_bits & 0x00000008u) {
      processingerror_nu_ = from.processingerror_nu_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ProcToLogicPort::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ProcToLogicPort::CopyFrom(const ProcToLogicPort& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ProcToLogicPort::IsInitialized() const {
  return true;
}

void ProcToLogicPort::InternalSwap(ProcToLogicPort* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(ssigheader_, other->ssigheader_);
  swap(minimaldistance_m_, other->minimaldistance_m_);
  swap(uiversionnumber_, other->uiversionnumber_);
  swap(processingerror_nu_, other->processingerror_nu_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ProcToLogicPort::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void ProcToLogicPort_array_port::InitAsDefaultInstance() {
}
class ProcToLogicPort_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<ProcToLogicPort_array_port>()._has_bits_);
};

ProcToLogicPort_array_port::ProcToLogicPort_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
}
ProcToLogicPort_array_port::ProcToLogicPort_array_port(const ProcToLogicPort_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
}

void ProcToLogicPort_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ProcToLogicPort_array_port_pdcp_2fproc_5fto_5flogic_5fport_2eproto.base);
}

ProcToLogicPort_array_port::~ProcToLogicPort_array_port() {
  // @@protoc_insertion_point(destructor:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
  SharedDtor();
}

void ProcToLogicPort_array_port::SharedDtor() {
}

void ProcToLogicPort_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ProcToLogicPort_array_port& ProcToLogicPort_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ProcToLogicPort_array_port_pdcp_2fproc_5fto_5flogic_5fport_2eproto.base);
  return *internal_default_instance();
}


void ProcToLogicPort_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ProcToLogicPort_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.pdcp.proc_to_logic_port.ProcToLogicPort data = 840;
      case 840:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 66)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<6722>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* ProcToLogicPort_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.pdcp.proc_to_logic_port.ProcToLogicPort data = 840;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(840, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
  return target;
}

size_t ProcToLogicPort_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.pdcp.proc_to_logic_port.ProcToLogicPort data = 840;
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

void ProcToLogicPort_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const ProcToLogicPort_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ProcToLogicPort_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
    MergeFrom(*source);
  }
}

void ProcToLogicPort_array_port::MergeFrom(const ProcToLogicPort_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void ProcToLogicPort_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ProcToLogicPort_array_port::CopyFrom(const ProcToLogicPort_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.pdcp.proc_to_logic_port.ProcToLogicPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ProcToLogicPort_array_port::IsInitialized() const {
  return true;
}

void ProcToLogicPort_array_port::InternalSwap(ProcToLogicPort_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ProcToLogicPort_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace proc_to_logic_port
}  // namespace pdcp
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::pdcp::proc_to_logic_port::ProcToLogicPort* Arena::CreateMaybeMessage< ::pb::pdcp::proc_to_logic_port::ProcToLogicPort >(Arena* arena) {
  return Arena::CreateInternal< ::pb::pdcp::proc_to_logic_port::ProcToLogicPort >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::pdcp::proc_to_logic_port::ProcToLogicPort_array_port* Arena::CreateMaybeMessage< ::pb::pdcp::proc_to_logic_port::ProcToLogicPort_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::pdcp::proc_to_logic_port::ProcToLogicPort_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
