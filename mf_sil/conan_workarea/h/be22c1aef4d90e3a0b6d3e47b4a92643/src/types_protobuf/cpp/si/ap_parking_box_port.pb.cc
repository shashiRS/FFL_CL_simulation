// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/ap_parking_box_port.proto

#include "si/ap_parking_box_port.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_si_2fap_5fparking_5fbox_5fport_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<3> scc_info_ApParkingBoxPort_si_2fap_5fparking_5fbox_5fport_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_si_2fexternal_5fpose_5fdata_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ExternalPoseData_si_2fexternal_5fpose_5fdata_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_si_2fparking_5fbox_5fserializable_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<4> scc_info_ParkingBoxSerializable_si_2fparking_5fbox_5fserializable_2eproto;
namespace pb {
namespace si {
namespace ap_parking_box_port {
class ApParkingBoxPortDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ApParkingBoxPort> _instance;
} _ApParkingBoxPort_default_instance_;
class ApParkingBoxPort_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ApParkingBoxPort_array_port> _instance;
} _ApParkingBoxPort_array_port_default_instance_;
}  // namespace ap_parking_box_port
}  // namespace si
}  // namespace pb
static void InitDefaultsscc_info_ApParkingBoxPort_si_2fap_5fparking_5fbox_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::ap_parking_box_port::_ApParkingBoxPort_default_instance_;
    new (ptr) ::pb::si::ap_parking_box_port::ApParkingBoxPort();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::ap_parking_box_port::ApParkingBoxPort::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<3> scc_info_ApParkingBoxPort_si_2fap_5fparking_5fbox_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 3, 0, InitDefaultsscc_info_ApParkingBoxPort_si_2fap_5fparking_5fbox_5fport_2eproto}, {
      &scc_info_SignalHeader_eco_2fsignal_5fheader_2eproto.base,
      &scc_info_ParkingBoxSerializable_si_2fparking_5fbox_5fserializable_2eproto.base,
      &scc_info_ExternalPoseData_si_2fexternal_5fpose_5fdata_2eproto.base,}};

static void InitDefaultsscc_info_ApParkingBoxPort_array_port_si_2fap_5fparking_5fbox_5fport_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::ap_parking_box_port::_ApParkingBoxPort_array_port_default_instance_;
    new (ptr) ::pb::si::ap_parking_box_port::ApParkingBoxPort_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::ap_parking_box_port::ApParkingBoxPort_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ApParkingBoxPort_array_port_si_2fap_5fparking_5fbox_5fport_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_ApParkingBoxPort_array_port_si_2fap_5fparking_5fbox_5fport_2eproto}, {
      &scc_info_ApParkingBoxPort_si_2fap_5fparking_5fbox_5fport_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_si_2fap_5fparking_5fbox_5fport_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_si_2fap_5fparking_5fbox_5fport_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_si_2fap_5fparking_5fbox_5fport_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_si_2fap_5fparking_5fbox_5fport_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::si::ap_parking_box_port::ApParkingBoxPort, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::ap_parking_box_port::ApParkingBoxPort, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::ap_parking_box_port::ApParkingBoxPort, uiversionnumber_),
  PROTOBUF_FIELD_OFFSET(::pb::si::ap_parking_box_port::ApParkingBoxPort, ssigheader_),
  PROTOBUF_FIELD_OFFSET(::pb::si::ap_parking_box_port::ApParkingBoxPort, numvalidparkingboxes_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::si::ap_parking_box_port::ApParkingBoxPort, numvalidexternalposes_),
  PROTOBUF_FIELD_OFFSET(::pb::si::ap_parking_box_port::ApParkingBoxPort, parkingboxes_),
  PROTOBUF_FIELD_OFFSET(::pb::si::ap_parking_box_port::ApParkingBoxPort, extposedata_),
  3,
  0,
  2,
  1,
  ~0u,
  ~0u,
  PROTOBUF_FIELD_OFFSET(::pb::si::ap_parking_box_port::ApParkingBoxPort_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::ap_parking_box_port::ApParkingBoxPort_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::ap_parking_box_port::ApParkingBoxPort_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 11, sizeof(::pb::si::ap_parking_box_port::ApParkingBoxPort)},
  { 17, 23, sizeof(::pb::si::ap_parking_box_port::ApParkingBoxPort_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::ap_parking_box_port::_ApParkingBoxPort_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::ap_parking_box_port::_ApParkingBoxPort_array_port_default_instance_),
};

const char descriptor_table_protodef_si_2fap_5fparking_5fbox_5fport_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\034si/ap_parking_box_port.proto\022\031pb.si.ap"
  "_parking_box_port\032\027eco/signal_header.pro"
  "to\032!si/parking_box_serializable.proto\032\033s"
  "i/external_pose_data.proto\"\270\002\n\020ApParking"
  "BoxPort\022\030\n\017uiVersionNumber\030\314\020 \001(\r\0227\n\nsSi"
  "gHeader\030\211\010 \001(\0132\".pb.eco.signal_header.Si"
  "gnalHeader\022 \n\027numValidParkingBoxes_nu\030\225\010"
  " \001(\r\022\036\n\025numValidExternalPoses\030\246\025 \001(\r\022M\n\014"
  "parkingBoxes\030\341\013 \003(\01326.pb.si.parking_box_"
  "serializable.ParkingBoxSerializable\022@\n\013e"
  "xtPoseData\030\236\032 \003(\0132*.pb.si.external_pose_"
  "data.ExternalPoseData\"Y\n\033ApParkingBoxPor"
  "t_array_port\022:\n\004data\030\336\034 \003(\0132+.pb.si.ap_p"
  "arking_box_port.ApParkingBoxPort"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_si_2fap_5fparking_5fbox_5fport_2eproto_deps[3] = {
  &::descriptor_table_eco_2fsignal_5fheader_2eproto,
  &::descriptor_table_si_2fexternal_5fpose_5fdata_2eproto,
  &::descriptor_table_si_2fparking_5fbox_5fserializable_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_si_2fap_5fparking_5fbox_5fport_2eproto_sccs[2] = {
  &scc_info_ApParkingBoxPort_si_2fap_5fparking_5fbox_5fport_2eproto.base,
  &scc_info_ApParkingBoxPort_array_port_si_2fap_5fparking_5fbox_5fport_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_si_2fap_5fparking_5fbox_5fport_2eproto_once;
static bool descriptor_table_si_2fap_5fparking_5fbox_5fport_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fap_5fparking_5fbox_5fport_2eproto = {
  &descriptor_table_si_2fap_5fparking_5fbox_5fport_2eproto_initialized, descriptor_table_protodef_si_2fap_5fparking_5fbox_5fport_2eproto, "si/ap_parking_box_port.proto", 552,
  &descriptor_table_si_2fap_5fparking_5fbox_5fport_2eproto_once, descriptor_table_si_2fap_5fparking_5fbox_5fport_2eproto_sccs, descriptor_table_si_2fap_5fparking_5fbox_5fport_2eproto_deps, 2, 3,
  schemas, file_default_instances, TableStruct_si_2fap_5fparking_5fbox_5fport_2eproto::offsets,
  file_level_metadata_si_2fap_5fparking_5fbox_5fport_2eproto, 2, file_level_enum_descriptors_si_2fap_5fparking_5fbox_5fport_2eproto, file_level_service_descriptors_si_2fap_5fparking_5fbox_5fport_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_si_2fap_5fparking_5fbox_5fport_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_si_2fap_5fparking_5fbox_5fport_2eproto), true);
namespace pb {
namespace si {
namespace ap_parking_box_port {

// ===================================================================

void ApParkingBoxPort::InitAsDefaultInstance() {
  ::pb::si::ap_parking_box_port::_ApParkingBoxPort_default_instance_._instance.get_mutable()->ssigheader_ = const_cast< ::pb::eco::signal_header::SignalHeader*>(
      ::pb::eco::signal_header::SignalHeader::internal_default_instance());
}
class ApParkingBoxPort::_Internal {
 public:
  using HasBits = decltype(std::declval<ApParkingBoxPort>()._has_bits_);
  static void set_has_uiversionnumber(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static const ::pb::eco::signal_header::SignalHeader& ssigheader(const ApParkingBoxPort* msg);
  static void set_has_ssigheader(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_numvalidparkingboxes_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_numvalidexternalposes(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

const ::pb::eco::signal_header::SignalHeader&
ApParkingBoxPort::_Internal::ssigheader(const ApParkingBoxPort* msg) {
  return *msg->ssigheader_;
}
void ApParkingBoxPort::clear_ssigheader() {
  if (ssigheader_ != nullptr) ssigheader_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void ApParkingBoxPort::clear_parkingboxes() {
  parkingboxes_.Clear();
}
void ApParkingBoxPort::clear_extposedata() {
  extposedata_.Clear();
}
ApParkingBoxPort::ApParkingBoxPort()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.ap_parking_box_port.ApParkingBoxPort)
}
ApParkingBoxPort::ApParkingBoxPort(const ApParkingBoxPort& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      parkingboxes_(from.parkingboxes_),
      extposedata_(from.extposedata_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_ssigheader()) {
    ssigheader_ = new ::pb::eco::signal_header::SignalHeader(*from.ssigheader_);
  } else {
    ssigheader_ = nullptr;
  }
  ::memcpy(&numvalidexternalposes_, &from.numvalidexternalposes_,
    static_cast<size_t>(reinterpret_cast<char*>(&uiversionnumber_) -
    reinterpret_cast<char*>(&numvalidexternalposes_)) + sizeof(uiversionnumber_));
  // @@protoc_insertion_point(copy_constructor:pb.si.ap_parking_box_port.ApParkingBoxPort)
}

void ApParkingBoxPort::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ApParkingBoxPort_si_2fap_5fparking_5fbox_5fport_2eproto.base);
  ::memset(&ssigheader_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&uiversionnumber_) -
      reinterpret_cast<char*>(&ssigheader_)) + sizeof(uiversionnumber_));
}

ApParkingBoxPort::~ApParkingBoxPort() {
  // @@protoc_insertion_point(destructor:pb.si.ap_parking_box_port.ApParkingBoxPort)
  SharedDtor();
}

void ApParkingBoxPort::SharedDtor() {
  if (this != internal_default_instance()) delete ssigheader_;
}

void ApParkingBoxPort::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ApParkingBoxPort& ApParkingBoxPort::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ApParkingBoxPort_si_2fap_5fparking_5fbox_5fport_2eproto.base);
  return *internal_default_instance();
}


void ApParkingBoxPort::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.ap_parking_box_port.ApParkingBoxPort)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  parkingboxes_.Clear();
  extposedata_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(ssigheader_ != nullptr);
    ssigheader_->Clear();
  }
  if (cached_has_bits & 0x0000000eu) {
    ::memset(&numvalidexternalposes_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&uiversionnumber_) -
        reinterpret_cast<char*>(&numvalidexternalposes_)) + sizeof(uiversionnumber_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ApParkingBoxPort::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // optional uint32 numValidParkingBoxes_nu = 1045;
      case 1045:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 168)) {
          _Internal::set_has_numvalidparkingboxes_nu(&has_bits);
          numvalidparkingboxes_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .pb.si.parking_box_serializable.ParkingBoxSerializable parkingBoxes = 1505;
      case 1505:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_parkingboxes(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<12042>(ptr));
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
      // optional uint32 numValidExternalPoses = 2726;
      case 2726:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          _Internal::set_has_numvalidexternalposes(&has_bits);
          numvalidexternalposes_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .pb.si.external_pose_data.ExternalPoseData extPoseData = 3358;
      case 3358:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 242)) {
          ptr = ctx->ParseMessage(_internal_add_extposedata(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* ApParkingBoxPort::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.ap_parking_box_port.ApParkingBoxPort)
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

  // optional uint32 numValidParkingBoxes_nu = 1045;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1045, this->_internal_numvalidparkingboxes_nu(), target);
  }

  // repeated .pb.si.parking_box_serializable.ParkingBoxSerializable parkingBoxes = 1505;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_parkingboxes_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1505, this->_internal_parkingboxes(i), target, stream);
  }

  // optional uint32 uiVersionNumber = 2124;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2124, this->_internal_uiversionnumber(), target);
  }

  // optional uint32 numValidExternalPoses = 2726;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2726, this->_internal_numvalidexternalposes(), target);
  }

  // repeated .pb.si.external_pose_data.ExternalPoseData extPoseData = 3358;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_extposedata_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3358, this->_internal_extposedata(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.ap_parking_box_port.ApParkingBoxPort)
  return target;
}

size_t ApParkingBoxPort::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.ap_parking_box_port.ApParkingBoxPort)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.si.parking_box_serializable.ParkingBoxSerializable parkingBoxes = 1505;
  total_size += 2UL * this->_internal_parkingboxes_size();
  for (const auto& msg : this->parkingboxes_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .pb.si.external_pose_data.ExternalPoseData extPoseData = 3358;
  total_size += 3UL * this->_internal_extposedata_size();
  for (const auto& msg : this->extposedata_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional .pb.eco.signal_header.SignalHeader sSigHeader = 1033;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *ssigheader_);
    }

    // optional uint32 numValidExternalPoses = 2726;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_numvalidexternalposes());
    }

    // optional uint32 numValidParkingBoxes_nu = 1045;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_numvalidparkingboxes_nu());
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

void ApParkingBoxPort::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.ap_parking_box_port.ApParkingBoxPort)
  GOOGLE_DCHECK_NE(&from, this);
  const ApParkingBoxPort* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ApParkingBoxPort>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.ap_parking_box_port.ApParkingBoxPort)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.ap_parking_box_port.ApParkingBoxPort)
    MergeFrom(*source);
  }
}

void ApParkingBoxPort::MergeFrom(const ApParkingBoxPort& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.ap_parking_box_port.ApParkingBoxPort)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  parkingboxes_.MergeFrom(from.parkingboxes_);
  extposedata_.MergeFrom(from.extposedata_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_ssigheader()->::pb::eco::signal_header::SignalHeader::MergeFrom(from._internal_ssigheader());
    }
    if (cached_has_bits & 0x00000002u) {
      numvalidexternalposes_ = from.numvalidexternalposes_;
    }
    if (cached_has_bits & 0x00000004u) {
      numvalidparkingboxes_nu_ = from.numvalidparkingboxes_nu_;
    }
    if (cached_has_bits & 0x00000008u) {
      uiversionnumber_ = from.uiversionnumber_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ApParkingBoxPort::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.ap_parking_box_port.ApParkingBoxPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ApParkingBoxPort::CopyFrom(const ApParkingBoxPort& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.ap_parking_box_port.ApParkingBoxPort)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ApParkingBoxPort::IsInitialized() const {
  return true;
}

void ApParkingBoxPort::InternalSwap(ApParkingBoxPort* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  parkingboxes_.InternalSwap(&other->parkingboxes_);
  extposedata_.InternalSwap(&other->extposedata_);
  swap(ssigheader_, other->ssigheader_);
  swap(numvalidexternalposes_, other->numvalidexternalposes_);
  swap(numvalidparkingboxes_nu_, other->numvalidparkingboxes_nu_);
  swap(uiversionnumber_, other->uiversionnumber_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ApParkingBoxPort::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void ApParkingBoxPort_array_port::InitAsDefaultInstance() {
}
class ApParkingBoxPort_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<ApParkingBoxPort_array_port>()._has_bits_);
};

ApParkingBoxPort_array_port::ApParkingBoxPort_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
}
ApParkingBoxPort_array_port::ApParkingBoxPort_array_port(const ApParkingBoxPort_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
}

void ApParkingBoxPort_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ApParkingBoxPort_array_port_si_2fap_5fparking_5fbox_5fport_2eproto.base);
}

ApParkingBoxPort_array_port::~ApParkingBoxPort_array_port() {
  // @@protoc_insertion_point(destructor:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
  SharedDtor();
}

void ApParkingBoxPort_array_port::SharedDtor() {
}

void ApParkingBoxPort_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ApParkingBoxPort_array_port& ApParkingBoxPort_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ApParkingBoxPort_array_port_si_2fap_5fparking_5fbox_5fport_2eproto.base);
  return *internal_default_instance();
}


void ApParkingBoxPort_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ApParkingBoxPort_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.si.ap_parking_box_port.ApParkingBoxPort data = 3678;
      case 3678:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 242)) {
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

::PROTOBUF_NAMESPACE_ID::uint8* ApParkingBoxPort_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.si.ap_parking_box_port.ApParkingBoxPort data = 3678;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3678, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
  return target;
}

size_t ApParkingBoxPort_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.si.ap_parking_box_port.ApParkingBoxPort data = 3678;
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

void ApParkingBoxPort_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const ApParkingBoxPort_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ApParkingBoxPort_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
    MergeFrom(*source);
  }
}

void ApParkingBoxPort_array_port::MergeFrom(const ApParkingBoxPort_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void ApParkingBoxPort_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ApParkingBoxPort_array_port::CopyFrom(const ApParkingBoxPort_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.ap_parking_box_port.ApParkingBoxPort_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ApParkingBoxPort_array_port::IsInitialized() const {
  return true;
}

void ApParkingBoxPort_array_port::InternalSwap(ApParkingBoxPort_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ApParkingBoxPort_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace ap_parking_box_port
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::si::ap_parking_box_port::ApParkingBoxPort* Arena::CreateMaybeMessage< ::pb::si::ap_parking_box_port::ApParkingBoxPort >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::ap_parking_box_port::ApParkingBoxPort >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::si::ap_parking_box_port::ApParkingBoxPort_array_port* Arena::CreateMaybeMessage< ::pb::si::ap_parking_box_port::ApParkingBoxPort_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::ap_parking_box_port::ApParkingBoxPort_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
