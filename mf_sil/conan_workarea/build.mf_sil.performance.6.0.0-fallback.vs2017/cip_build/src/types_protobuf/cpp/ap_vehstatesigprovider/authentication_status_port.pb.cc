// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ap_vehstatesigprovider/authentication_status_port.proto

#include "ap_vehstatesigprovider/authentication_status_port.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_AuthenticationStatusPort_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_eco_2fsignal_5fheader_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto;
namespace pb {
namespace ap_vehstatesigprovider {
namespace authentication_status_port {
class AuthenticationStatusPortDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<AuthenticationStatusPort> _instance;
} _AuthenticationStatusPort_default_instance_;
class AuthenticationStatusPort_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<AuthenticationStatusPort_array_port> _instance;
} _AuthenticationStatusPort_array_port_default_instance_;
}  // namespace authentication_status_port
}  // namespace ap_vehstatesigprovider
}  // namespace pb
static void InitDefaultsscc_info_AuthenticationStatusPort_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_vehstatesigprovider::authentication_status_port::_AuthenticationStatusPort_default_instance_;
    new (ptr) ::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_AuthenticationStatusPort_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_AuthenticationStatusPort_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,}};

static void InitDefaultsscc_info_AuthenticationStatusPort_array_port_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::ap_vehstatesigprovider::authentication_status_port::_AuthenticationStatusPort_array_port_default_instance_;
    new (ptr) ::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_AuthenticationStatusPort_array_port_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_AuthenticationStatusPort_array_port_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto}, {
      &scc_info_AuthenticationStatusPort_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort, uiversionnumber_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort, ssigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort, authkeydetected_nu_),
  1,
  0,
  2,
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort)},
  { 11, 17, sizeof(::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_vehstatesigprovider::authentication_status_port::_AuthenticationStatusPort_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::ap_vehstatesigprovider::authentication_status_port::_AuthenticationStatusPort_array_port_default_instance_),
};

const char descriptor_table_protodef_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n7ap_vehstatesigprovider/authentication_"
  "status_port.proto\0224pb.ap_vehstatesigprov"
  "ider.authentication_status_port\032\027eco/sig"
  "nal_header.proto\"\212\001\n\030AuthenticationStatu"
  "sPort\022\030\n\017uiVersionNumber\030\314\020 \001(\r\0227\n\nsSigH"
  "eader\030\211\010 \001(\0132\".pb.eco.signal_header.Sign"
  "alHeader\022\033\n\022authKeyDetected_nu\030\332\025 \001(\010\"\204\001"
  "\n#AuthenticationStatusPort_array_port\022]\n"
  "\004data\030\234\032 \003(\0132N.pb.ap_vehstatesigprovider"
  ".authentication_status_port.Authenticati"
  "onStatusPort"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto_deps[1] = {
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto_sccs[2] = {
  &scc_info_AuthenticationStatusPort_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto.base,
  &scc_info_AuthenticationStatusPort_array_port_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto_once;
static bool descriptor_table_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto = {
  &descriptor_table_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto_initialized, descriptor_table_protodef_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto, "ap_vehstatesigprovider/authentication_status_port.proto", 412,
  &descriptor_table_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto_once, descriptor_table_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto_sccs, descriptor_table_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto::offsets,
  file_level_metadata_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto, 2, file_level_enum_descriptors_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto, file_level_service_descriptors_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto), true);
namespace pb {
namespace ap_vehstatesigprovider {
namespace authentication_status_port {

// ===================================================================

void AuthenticationStatusPort::InitAsDefaultInstance() {
  ::pb::ap_vehstatesigprovider::authentication_status_port::_AuthenticationStatusPort_default_instance_._instance.get_mutable()->ssigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
}
class AuthenticationStatusPort::_Internal {
 public:
  using HasBits = decltype(std::declval<AuthenticationStatusPort>()._has_bits_);
  static void set_has_uiversionnumber(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::pb::eco::signal_header::SignalHeader& ssigheader(const AuthenticationStatusPort* msg);
  static void set_has_ssigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_authkeydetected_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
AuthenticationStatusPort::_Internal::ssigheader(const AuthenticationStatusPort* msg) {
  return *msg->ssigheader_;
}
void AuthenticationStatusPort::clear_ssigheader() {
  if (ssigheader_ != nullptr) ssigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
AuthenticationStatusPort::AuthenticationStatusPort()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
}
AuthenticationStatusPort::AuthenticationStatusPort(const AuthenticationStatusPort& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_ssigheader()) {
    ssigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.ssigheader_);
  } else {
    ssigheader_ = nullptr;
  }
  ::memcpy(&uiversionnumber_, &from.uiversionnumber_,
    static_cast<size_t>(reinterpret_cast<char*>(&authkeydetected_nu_) -
    reinterpret_cast<char*>(&uiversionnumber_)) + sizeof(authkeydetected_nu_));
  // @@protoc_insertion_point(copy_constructor:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
}

void AuthenticationStatusPort::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_AuthenticationStatusPort_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto.base);
  ::memset(&ssigheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&authkeydetected_nu_) -
      reinterpret_cast<char*>(&ssigheader_)) + sizeof(authkeydetected_nu_));
}

AuthenticationStatusPort::~AuthenticationStatusPort() {
  // @@protoc_insertion_point(destructor:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
  SharedDtor();
}

void AuthenticationStatusPort::SharedDtor() {
  if (this != internal_default_instance()) delete ssigheader_;
}

void AuthenticationStatusPort::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const AuthenticationStatusPort& AuthenticationStatusPort::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_AuthenticationStatusPort_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto.base);
  return *internal_default_instance();
}


void AuthenticationStatusPort::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(ssigheader_ != nullptr);
    ssigheader_->Clear();
  }
  if (cached_has_bits & 0x00000006u) {
    ::memset(&uiversionnumber_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&authkeydetected_nu_) -
        reinterpret_cast<char*>(&uiversionnumber_)) + sizeof(authkeydetected_nu_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* AuthenticationStatusPort::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // optional bool authKeyDetected_nu = 2778;
      case 2778:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 208)) {
          _Internal::set_has_authkeydetected_nu(&has_bits);
          authkeydetected_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* AuthenticationStatusPort::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
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
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2124, this->_internal_uiversionnumber(), target);
  }

  // optional bool authKeyDetected_nu = 2778;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(2778, this->_internal_authkeydetected_nu(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
  return target;
}

size_t AuthenticationStatusPort::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
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

    // optional bool authKeyDetected_nu = 2778;
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

void AuthenticationStatusPort::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
  GOOGLE_DCHECK_NE(&from, this);
  const AuthenticationStatusPort* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<AuthenticationStatusPort>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
    MergeFrom(*source);
  }
}

void AuthenticationStatusPort::MergeFrom(const AuthenticationStatusPort& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_ssigheader()->::pb::eco::signal_header::SignalHeader::MergeFrom(from._internal_ssigheader());
    }
    if (cached_has_bits & 0x00000002u) {
      uiversionnumber_ = from.uiversionnumber_;
    }
    if (cached_has_bits & 0x00000004u) {
      authkeydetected_nu_ = from.authkeydetected_nu_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void AuthenticationStatusPort::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void AuthenticationStatusPort::CopyFrom(const AuthenticationStatusPort& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool AuthenticationStatusPort::IsInitialized() const {
  return true;
}

void AuthenticationStatusPort::InternalSwap(AuthenticationStatusPort* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(ssigheader_, other->ssigheader_);
  swap(uiversionnumber_, other->uiversionnumber_);
  swap(authkeydetected_nu_, other->authkeydetected_nu_);
}

::PROTOBUF_NAMESPACE_ID::Metadata AuthenticationStatusPort::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void AuthenticationStatusPort_array_port::InitAsDefaultInstance() {
}
class AuthenticationStatusPort_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<AuthenticationStatusPort_array_port>()._has_bits_);
};

AuthenticationStatusPort_array_port::AuthenticationStatusPort_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
}
AuthenticationStatusPort_array_port::AuthenticationStatusPort_array_port(const AuthenticationStatusPort_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
}

void AuthenticationStatusPort_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_AuthenticationStatusPort_array_port_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto.base);
}

AuthenticationStatusPort_array_port::~AuthenticationStatusPort_array_port() {
  // @@protoc_insertion_point(destructor:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
  SharedDtor();
}

void AuthenticationStatusPort_array_port::SharedDtor() {
}

void AuthenticationStatusPort_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const AuthenticationStatusPort_array_port& AuthenticationStatusPort_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_AuthenticationStatusPort_array_port_ap_5fvehstatesigprovider_2fauthentication_5fstatus_5fport_2eproto.base);
  return *internal_default_instance();
}


void AuthenticationStatusPort_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* AuthenticationStatusPort_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort data = 3356;
      case 3356:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 226)) {
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

::PROTOBUF_NAMESPACE_ID::uint8* AuthenticationStatusPort_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort data = 3356;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3356, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
  return target;
}

size_t AuthenticationStatusPort_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort data = 3356;
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

void AuthenticationStatusPort_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const AuthenticationStatusPort_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<AuthenticationStatusPort_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
    MergeFrom(*source);
  }
}

void AuthenticationStatusPort_array_port::MergeFrom(const AuthenticationStatusPort_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void AuthenticationStatusPort_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void AuthenticationStatusPort_array_port::CopyFrom(const AuthenticationStatusPort_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.ap_vehstatesigprovider.authentication_status_port.AuthenticationStatusPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool AuthenticationStatusPort_array_port::IsInitialized() const {
  return true;
}

void AuthenticationStatusPort_array_port::InternalSwap(AuthenticationStatusPort_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata AuthenticationStatusPort_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace authentication_status_port
}  // namespace ap_vehstatesigprovider
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort* Arena::CreateMaybeMessage< ::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort_array_port* Arena::CreateMaybeMessage< ::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::ap_vehstatesigprovider::authentication_status_port::AuthenticationStatusPort_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>