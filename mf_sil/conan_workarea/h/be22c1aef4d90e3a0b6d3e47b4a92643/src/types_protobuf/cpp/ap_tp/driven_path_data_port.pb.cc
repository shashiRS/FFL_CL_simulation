// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_tp/driven_path_data_port.proto

#include "ap_tp/driven_path_data_port.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_DrivenPathDataPort_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_ap_5ftp_2fstored_5fwaypoint_5fdata_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_StoredWaypointData_ap_5ftp_2fstored_5fwaypoint_5fdata_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_eco_2fsignal_5fheader_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto;
namespace pb {
namespace ap_tp {
namespace driven_path_data_port {
class DrivenPathDataPortDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<DrivenPathDataPort> _instance;
} _DrivenPathDataPort_default_instance_;
class DrivenPathDataPort_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<DrivenPathDataPort_array_port> _instance;
} _DrivenPathDataPort_array_port_default_instance_;
}  // namespace driven_path_data_port
}  // namespace ap_tp
}  // namespace pb
static void InitDefaultsscc_info_DrivenPathDataPort_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_tp::driven_path_data_port::_DrivenPathDataPort_default_instance_;
    new (ptr) ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_DrivenPathDataPort_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, 0, InitDefaultsscc_info_DrivenPathDataPort_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,
      &scc_info_StoredWaypointData_ap_5ftp_2fstored_5fwaypoint_5fdata_2eproto.base,}};

static void InitDefaultsscc_info_DrivenPathDataPort_array_port_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_tp::driven_path_data_port::_DrivenPathDataPort_array_port_default_instance_;
    new (ptr) ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_DrivenPathDataPort_array_port_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_DrivenPathDataPort_array_port_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto}, {
      &scc_info_DrivenPathDataPort_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort, uiversionnumber_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort, ssigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort, savecounter_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort, hasvaliddata_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort, storeddrivenpath_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort, numelementsindrivenpath_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort, buffer_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort, numelementsinbuffer_),
  5,
  0,
  4,
  1,
  ~0u,
  3,
  ~0u,
  2,
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 13, sizeof(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort)},
  { 21, 27, sizeof(::pb::ap_tp::driven_path_data_port::DrivenPathDataPort_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_tp::driven_path_data_port::_DrivenPathDataPort_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_tp::driven_path_data_port::_DrivenPathDataPort_array_port_default_instance_),
};

const char descriptor_table_protodef_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n!ap_tp/driven_path_data_port.proto\022\036pb."
  "ap_tp.driven_path_data_port\032\027eco/signal_"
  "header.proto\032 ap_tp/stored_waypoint_data"
  ".proto\"\346\002\n\022DrivenPathDataPort\022\030\n\017uiVersi"
  "onNumber\030\314\020 \001(\r\0227\n\nsSigHeader\030\211\010 \001(\0132\".p"
  "b.eco.signal_header.SignalHeader\022\024\n\013save"
  "Counter\030\322\017 \001(\r\022\025\n\014hasValidData\030\312\023 \001(\010\022L\n"
  "\020storedDrivenPath\030\207\005 \003(\01321.pb.ap_tp.stor"
  "ed_waypoint_data.StoredWaypointData\022 \n\027n"
  "umElementsInDrivenPath\030\354\r \001(\r\022B\n\006buffer\030"
  "\202\026 \003(\01321.pb.ap_tp.stored_waypoint_data.S"
  "toredWaypointData\022\034\n\023numElementsInBuffer"
  "\030\212\014 \001(\r\"b\n\035DrivenPathDataPort_array_port"
  "\022A\n\004data\030\302\031 \003(\01322.pb.ap_tp.driven_path_d"
  "ata_port.DrivenPathDataPort"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto_deps[2] = {
  &::descriptor_table_ap_5ftp_2fstored_5fwaypoint_5fdata_2eproto,
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto_sccs[2] = {
  &scc_info_DrivenPathDataPort_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto.base,
  &scc_info_DrivenPathDataPort_array_port_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto_once;
static bool descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto = {
  &descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto_initialized, descriptor_table_protodef_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto, "ap_tp/driven_path_data_port.proto", 587,
  &descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto_once, descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto_sccs, descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto_deps, 2, 2,
  schemas, file_default_instances, TableStruct_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto::offsets,
  file_level_metadata_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto, 2, file_level_enum_descriptors_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto, file_level_service_descriptors_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto), true);
namespace pb {
namespace ap_tp {
namespace driven_path_data_port {

// ===================================================================

void DrivenPathDataPort::InitAsDefaultInstance() {
  ::pb::ap_tp::driven_path_data_port::_DrivenPathDataPort_default_instance_._instance.get_mutable()->ssigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
}
class DrivenPathDataPort::_Internal {
 public:
  using HasBits = decltype(std::declval<DrivenPathDataPort>()._has_bits_);
  static void set_has_uiversionnumber(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static const ::pb::eco::signal_header::SignalHeader& ssigheader(const DrivenPathDataPort* msg);
  static void set_has_ssigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_savecounter(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_hasvaliddata(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_numelementsindrivenpath(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_numelementsinbuffer(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
DrivenPathDataPort::_Internal::ssigheader(const DrivenPathDataPort* msg) {
  return *msg->ssigheader_;
}
void DrivenPathDataPort::clear_ssigheader() {
  if (ssigheader_ != nullptr) ssigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void DrivenPathDataPort::clear_storeddrivenpath() {
  storeddrivenpath_.Clear();
}
void DrivenPathDataPort::clear_buffer() {
  buffer_.Clear();
}
DrivenPathDataPort::DrivenPathDataPort()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
}
DrivenPathDataPort::DrivenPathDataPort(const DrivenPathDataPort& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      storeddrivenpath_(from.storeddrivenpath_),
      buffer_(from.buffer_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_ssigheader()) {
    ssigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.ssigheader_);
  } else {
    ssigheader_ = nullptr;
  }
  ::memcpy(&hasvaliddata_, &from.hasvaliddata_,
    static_cast<size_t>(reinterpret_cast<char*>(&uiversionnumber_) -
    reinterpret_cast<char*>(&hasvaliddata_)) + sizeof(uiversionnumber_));
  // @@protoc_insertion_point(copy_constructor:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
}

void DrivenPathDataPort::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_DrivenPathDataPort_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto.base);
  ::memset(&ssigheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&uiversionnumber_) -
      reinterpret_cast<char*>(&ssigheader_)) + sizeof(uiversionnumber_));
}

DrivenPathDataPort::~DrivenPathDataPort() {
  // @@protoc_insertion_point(destructor:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
  SharedDtor();
}

void DrivenPathDataPort::SharedDtor() {
  if (this != internal_default_instance()) delete ssigheader_;
}

void DrivenPathDataPort::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const DrivenPathDataPort& DrivenPathDataPort::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_DrivenPathDataPort_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto.base);
  return *internal_default_instance();
}


void DrivenPathDataPort::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  storeddrivenpath_.Clear();
  buffer_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(ssigheader_ != nullptr);
    ssigheader_->Clear();
  }
  if (cached_has_bits & 0x0000003eu) {
    ::memset(&hasvaliddata_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&uiversionnumber_) -
        reinterpret_cast<char*>(&hasvaliddata_)) + sizeof(uiversionnumber_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* DrivenPathDataPort::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.ap_tp.stored_waypoint_data.StoredWaypointData storedDrivenPath = 647;
      case 647:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 58)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_storeddrivenpath(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<5178>(ptr));
        } else goto handle_unusual;
        continue;
      // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
      case 1033:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 74)) {
          ptr = ctx->ParseMessage(_internal_mutable_ssigheader(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 numElementsInBuffer = 1546;
      case 1546:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 80)) {
          _Internal::set_has_numelementsinbuffer(&has_bits);
          numelementsinbuffer_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 numElementsInDrivenPath = 1772;
      case 1772:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 96)) {
          _Internal::set_has_numelementsindrivenpath(&has_bits);
          numelementsindrivenpath_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional uint32 saveCounter = 2002;
      case 2002:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 144)) {
          _Internal::set_has_savecounter(&has_bits);
          savecounter_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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
      // optional bool hasValidData = 2506;
      case 2506:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 80)) {
          _Internal::set_has_hasvaliddata(&has_bits);
          hasvaliddata_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .pb.ap_tp.stored_waypoint_data.StoredWaypointData buffer = 2818;
      case 2818:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_add_buffer(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* DrivenPathDataPort::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.ap_tp.stored_waypoint_data.StoredWaypointData storedDrivenPath = 647;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_storeddrivenpath_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(647, this->_internal_storeddrivenpath(i), target, stream);
  }

  cached_has_bits = _has_bits_[0];
  // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1033, _Internal::ssigheader(this), target, stream);
  }

  // optional uint32 numElementsInBuffer = 1546;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1546, this->_internal_numelementsinbuffer(), target);
  }

  // optional uint32 numElementsInDrivenPath = 1772;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1772, this->_internal_numelementsindrivenpath(), target);
  }

  // optional uint32 saveCounter = 2002;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2002, this->_internal_savecounter(), target);
  }

  // optional uint32 uiVersionNumber = 2124;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2124, this->_internal_uiversionnumber(), target);
  }

  // optional bool hasValidData = 2506;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2506, this->_internal_hasvaliddata(), target);
  }

  // repeated .pb.ap_tp.stored_waypoint_data.StoredWaypointData buffer = 2818;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_buffer_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2818, this->_internal_buffer(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
  return target;
}

size_t DrivenPathDataPort::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.ap_tp.stored_waypoint_data.StoredWaypointData storedDrivenPath = 647;
  total_size += 2UL * this->_internal_storeddrivenpath_size();
  for (const auto& msg : this->storeddrivenpath_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .pb.ap_tp.stored_waypoint_data.StoredWaypointData buffer = 2818;
  total_size += 3UL * this->_internal_buffer_size();
  for (const auto& msg : this->buffer_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *ssigheader_);
    }

    // optional bool hasValidData = 2506;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 + 1;
    }

    // optional uint32 numElementsInBuffer = 1546;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_numelementsinbuffer());
    }

    // optional uint32 numElementsInDrivenPath = 1772;
    if (cached_has_bits & 0x00000008u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_numelementsindrivenpath());
    }

    // optional uint32 saveCounter = 2002;
    if (cached_has_bits & 0x00000010u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_savecounter());
    }

    // optional uint32 uiVersionNumber = 2124;
    if (cached_has_bits & 0x00000020u) {
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

void DrivenPathDataPort::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
  GOOGLE_DCHECK_NE(&from, this);
  const DrivenPathDataPort* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<DrivenPathDataPort>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
    MergeFrom(*source);
  }
}

void DrivenPathDataPort::MergeFrom(const DrivenPathDataPort& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  storeddrivenpath_.MergeFrom(from.storeddrivenpath_);
  buffer_.MergeFrom(from.buffer_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000003fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_ssigheader()->::pb::eco::signal_header::SignalHeader::MergeFrom(from._internal_ssigheader());
    }
    if (cached_has_bits & 0x00000002u) {
      hasvaliddata_ = from.hasvaliddata_;
    }
    if (cached_has_bits & 0x00000004u) {
      numelementsinbuffer_ = from.numelementsinbuffer_;
    }
    if (cached_has_bits & 0x00000008u) {
      numelementsindrivenpath_ = from.numelementsindrivenpath_;
    }
    if (cached_has_bits & 0x00000010u) {
      savecounter_ = from.savecounter_;
    }
    if (cached_has_bits & 0x00000020u) {
      uiversionnumber_ = from.uiversionnumber_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void DrivenPathDataPort::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DrivenPathDataPort::CopyFrom(const DrivenPathDataPort& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DrivenPathDataPort::IsInitialized() const {
  return true;
}

void DrivenPathDataPort::InternalSwap(DrivenPathDataPort* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  storeddrivenpath_.InternalSwap(&other->storeddrivenpath_);
  buffer_.InternalSwap(&other->buffer_);
  swap(ssigheader_, other->ssigheader_);
  swap(hasvaliddata_, other->hasvaliddata_);
  swap(numelementsinbuffer_, other->numelementsinbuffer_);
  swap(numelementsindrivenpath_, other->numelementsindrivenpath_);
  swap(savecounter_, other->savecounter_);
  swap(uiversionnumber_, other->uiversionnumber_);
}

::PROTOBUF_NAMESPACE_ID::Metadata DrivenPathDataPort::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void DrivenPathDataPort_array_port::InitAsDefaultInstance() {
}
class DrivenPathDataPort_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<DrivenPathDataPort_array_port>()._has_bits_);
};

DrivenPathDataPort_array_port::DrivenPathDataPort_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
}
DrivenPathDataPort_array_port::DrivenPathDataPort_array_port(const DrivenPathDataPort_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
}

void DrivenPathDataPort_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_DrivenPathDataPort_array_port_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto.base);
}

DrivenPathDataPort_array_port::~DrivenPathDataPort_array_port() {
  // @@protoc_insertion_point(destructor:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
  SharedDtor();
}

void DrivenPathDataPort_array_port::SharedDtor() {
}

void DrivenPathDataPort_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const DrivenPathDataPort_array_port& DrivenPathDataPort_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_DrivenPathDataPort_array_port_ap_5ftp_2fdriven_5fpath_5fdata_5fport_2eproto.base);
  return *internal_default_instance();
}


void DrivenPathDataPort_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* DrivenPathDataPort_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.ap_tp.driven_path_data_port.DrivenPathDataPort data = 3266;
      case 3266:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
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

::PROTOBUF_NAMESPACE_ID::uint8* DrivenPathDataPort_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.ap_tp.driven_path_data_port.DrivenPathDataPort data = 3266;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3266, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
  return target;
}

size_t DrivenPathDataPort_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.ap_tp.driven_path_data_port.DrivenPathDataPort data = 3266;
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

void DrivenPathDataPort_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const DrivenPathDataPort_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<DrivenPathDataPort_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
    MergeFrom(*source);
  }
}

void DrivenPathDataPort_array_port::MergeFrom(const DrivenPathDataPort_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void DrivenPathDataPort_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DrivenPathDataPort_array_port::CopyFrom(const DrivenPathDataPort_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_tp.driven_path_data_port.DrivenPathDataPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DrivenPathDataPort_array_port::IsInitialized() const {
  return true;
}

void DrivenPathDataPort_array_port::InternalSwap(DrivenPathDataPort_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata DrivenPathDataPort_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace driven_path_data_port
}  // namespace ap_tp
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort* Arena::CreateMaybeMessage< ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort_array_port* Arena::CreateMaybeMessage< ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_tp::driven_path_data_port::DrivenPathDataPort_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
