// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/diagnostic_event.proto

#include "si/diagnostic_event.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_si_2fdiagnostic_5fevent_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_DiagnosticEvent_si_2fdiagnostic_5fevent_2eproto;
namespace pb {
namespace si {
namespace diagnostic_event {
class DiagnosticEventDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<DiagnosticEvent> _instance;
} _DiagnosticEvent_default_instance_;
class DiagnosticEvent_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<DiagnosticEvent_array_port> _instance;
} _DiagnosticEvent_array_port_default_instance_;
}  // namespace diagnostic_event
}  // namespace si
}  // namespace pb
static void InitDefaultsscc_info_DiagnosticEvent_si_2fdiagnostic_5fevent_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::diagnostic_event::_DiagnosticEvent_default_instance_;
    new (ptr) ::pb::si::diagnostic_event::DiagnosticEvent();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::diagnostic_event::DiagnosticEvent::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_DiagnosticEvent_si_2fdiagnostic_5fevent_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_DiagnosticEvent_si_2fdiagnostic_5fevent_2eproto}, {}};

static void InitDefaultsscc_info_DiagnosticEvent_array_port_si_2fdiagnostic_5fevent_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::diagnostic_event::_DiagnosticEvent_array_port_default_instance_;
    new (ptr) ::pb::si::diagnostic_event::DiagnosticEvent_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::diagnostic_event::DiagnosticEvent_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_DiagnosticEvent_array_port_si_2fdiagnostic_5fevent_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_DiagnosticEvent_array_port_si_2fdiagnostic_5fevent_2eproto}, {
      &scc_info_DiagnosticEvent_si_2fdiagnostic_5fevent_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_si_2fdiagnostic_5fevent_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_si_2fdiagnostic_5fevent_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_si_2fdiagnostic_5fevent_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_si_2fdiagnostic_5fevent_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event::DiagnosticEvent, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event::DiagnosticEvent, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event::DiagnosticEvent, diagnosiseventid_),
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event::DiagnosticEvent, diagnosiseventstatus_),
  1,
  0,
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event::DiagnosticEvent_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event::DiagnosticEvent_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::diagnostic_event::DiagnosticEvent_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::pb::si::diagnostic_event::DiagnosticEvent)},
  { 9, 15, sizeof(::pb::si::diagnostic_event::DiagnosticEvent_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::diagnostic_event::_DiagnosticEvent_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::diagnostic_event::_DiagnosticEvent_array_port_default_instance_),
};

const char descriptor_table_protodef_si_2fdiagnostic_5fevent_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\031si/diagnostic_event.proto\022\026pb.si.diagn"
  "ostic_event\032 eco/diagnosis_event_status."
  "proto\"\200\001\n\017DiagnosticEvent\022\031\n\020diagnosisEv"
  "entID\030\245\031 \001(\r\022R\n\024diagnosisEventStatus\030\346\006 "
  "\001(\01623.pb.eco.diagnosis_event_status.Diag"
  "nosisEventStatus\"T\n\032DiagnosticEvent_arra"
  "y_port\0226\n\004data\030\260\013 \003(\0132\'.pb.si.diagnostic"
  "_event.DiagnosticEvent"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_si_2fdiagnostic_5fevent_2eproto_deps[1] = {
  &::descriptor_table_eco_2fdiagnosis_5fevent_5fstatus_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_si_2fdiagnostic_5fevent_2eproto_sccs[2] = {
  &scc_info_DiagnosticEvent_si_2fdiagnostic_5fevent_2eproto.base,
  &scc_info_DiagnosticEvent_array_port_si_2fdiagnostic_5fevent_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_si_2fdiagnostic_5fevent_2eproto_once;
static bool descriptor_table_si_2fdiagnostic_5fevent_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fdiagnostic_5fevent_2eproto = {
  &descriptor_table_si_2fdiagnostic_5fevent_2eproto_initialized, descriptor_table_protodef_si_2fdiagnostic_5fevent_2eproto, "si/diagnostic_event.proto", 302,
  &descriptor_table_si_2fdiagnostic_5fevent_2eproto_once, descriptor_table_si_2fdiagnostic_5fevent_2eproto_sccs, descriptor_table_si_2fdiagnostic_5fevent_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_si_2fdiagnostic_5fevent_2eproto::offsets,
  file_level_metadata_si_2fdiagnostic_5fevent_2eproto, 2, file_level_enum_descriptors_si_2fdiagnostic_5fevent_2eproto, file_level_service_descriptors_si_2fdiagnostic_5fevent_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_si_2fdiagnostic_5fevent_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_si_2fdiagnostic_5fevent_2eproto), true);
namespace pb {
namespace si {
namespace diagnostic_event {

// ===================================================================

void DiagnosticEvent::InitAsDefaultInstance() {
}
class DiagnosticEvent::_Internal {
 public:
  using HasBits = decltype(std::declval<DiagnosticEvent>()._has_bits_);
  static void set_has_diagnosiseventid(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_diagnosiseventstatus(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

DiagnosticEvent::DiagnosticEvent()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.diagnostic_event.DiagnosticEvent)
}
DiagnosticEvent::DiagnosticEvent(const DiagnosticEvent& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&diagnosiseventstatus_, &from.diagnosiseventstatus_,
    static_cast<size_t>(reinterpret_cast<char*>(&diagnosiseventid_) -
    reinterpret_cast<char*>(&diagnosiseventstatus_)) + sizeof(diagnosiseventid_));
  // @@protoc_insertion_point(copy_constructor:pb.si.diagnostic_event.DiagnosticEvent)
}

void DiagnosticEvent::SharedCtor() {
  ::memset(&diagnosiseventstatus_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&diagnosiseventid_) -
      reinterpret_cast<char*>(&diagnosiseventstatus_)) + sizeof(diagnosiseventid_));
}

DiagnosticEvent::~DiagnosticEvent() {
  // @@protoc_insertion_point(destructor:pb.si.diagnostic_event.DiagnosticEvent)
  SharedDtor();
}

void DiagnosticEvent::SharedDtor() {
}

void DiagnosticEvent::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const DiagnosticEvent& DiagnosticEvent::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_DiagnosticEvent_si_2fdiagnostic_5fevent_2eproto.base);
  return *internal_default_instance();
}


void DiagnosticEvent::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.diagnostic_event.DiagnosticEvent)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    ::memset(&diagnosiseventstatus_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&diagnosiseventid_) -
        reinterpret_cast<char*>(&diagnosiseventstatus_)) + sizeof(diagnosiseventid_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* DiagnosticEvent::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .pb.eco.diagnosis_event_status.DiagnosisEventStatus diagnosisEventStatus = 870;
      case 870:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::pb::eco::diagnosis_event_status::DiagnosisEventStatus_IsValid(val))) {
            _internal_set_diagnosiseventstatus(static_cast<::pb::eco::diagnosis_event_status::DiagnosisEventStatus>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(870, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional uint32 diagnosisEventID = 3237;
      case 3237:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          _Internal::set_has_diagnosiseventid(&has_bits);
          diagnosiseventid_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* DiagnosticEvent::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.diagnostic_event.DiagnosticEvent)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .pb.eco.diagnosis_event_status.DiagnosisEventStatus diagnosisEventStatus = 870;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      870, this->_internal_diagnosiseventstatus(), target);
  }

  // optional uint32 diagnosisEventID = 3237;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(3237, this->_internal_diagnosiseventid(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.diagnostic_event.DiagnosticEvent)
  return target;
}

size_t DiagnosticEvent::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.diagnostic_event.DiagnosticEvent)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // optional .pb.eco.diagnosis_event_status.DiagnosisEventStatus diagnosisEventStatus = 870;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_diagnosiseventstatus());
    }

    // optional uint32 diagnosisEventID = 3237;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_diagnosiseventid());
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

void DiagnosticEvent::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.diagnostic_event.DiagnosticEvent)
  GOOGLE_DCHECK_NE(&from, this);
  const DiagnosticEvent* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<DiagnosticEvent>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.diagnostic_event.DiagnosticEvent)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.diagnostic_event.DiagnosticEvent)
    MergeFrom(*source);
  }
}

void DiagnosticEvent::MergeFrom(const DiagnosticEvent& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.diagnostic_event.DiagnosticEvent)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      diagnosiseventstatus_ = from.diagnosiseventstatus_;
    }
    if (cached_has_bits & 0x00000002u) {
      diagnosiseventid_ = from.diagnosiseventid_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void DiagnosticEvent::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.diagnostic_event.DiagnosticEvent)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DiagnosticEvent::CopyFrom(const DiagnosticEvent& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.diagnostic_event.DiagnosticEvent)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DiagnosticEvent::IsInitialized() const {
  return true;
}

void DiagnosticEvent::InternalSwap(DiagnosticEvent* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(diagnosiseventstatus_, other->diagnosiseventstatus_);
  swap(diagnosiseventid_, other->diagnosiseventid_);
}

::PROTOBUF_NAMESPACE_ID::Metadata DiagnosticEvent::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void DiagnosticEvent_array_port::InitAsDefaultInstance() {
}
class DiagnosticEvent_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<DiagnosticEvent_array_port>()._has_bits_);
};

DiagnosticEvent_array_port::DiagnosticEvent_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.diagnostic_event.DiagnosticEvent_array_port)
}
DiagnosticEvent_array_port::DiagnosticEvent_array_port(const DiagnosticEvent_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.si.diagnostic_event.DiagnosticEvent_array_port)
}

void DiagnosticEvent_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_DiagnosticEvent_array_port_si_2fdiagnostic_5fevent_2eproto.base);
}

DiagnosticEvent_array_port::~DiagnosticEvent_array_port() {
  // @@protoc_insertion_point(destructor:pb.si.diagnostic_event.DiagnosticEvent_array_port)
  SharedDtor();
}

void DiagnosticEvent_array_port::SharedDtor() {
}

void DiagnosticEvent_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const DiagnosticEvent_array_port& DiagnosticEvent_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_DiagnosticEvent_array_port_si_2fdiagnostic_5fevent_2eproto.base);
  return *internal_default_instance();
}


void DiagnosticEvent_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.diagnostic_event.DiagnosticEvent_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* DiagnosticEvent_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.si.diagnostic_event.DiagnosticEvent data = 1456;
      case 1456:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 130)) {
          ptr -= 2;
          do {
            ptr += 2;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<11650>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* DiagnosticEvent_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.diagnostic_event.DiagnosticEvent_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.si.diagnostic_event.DiagnosticEvent data = 1456;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1456, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.diagnostic_event.DiagnosticEvent_array_port)
  return target;
}

size_t DiagnosticEvent_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.diagnostic_event.DiagnosticEvent_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.si.diagnostic_event.DiagnosticEvent data = 1456;
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

void DiagnosticEvent_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.diagnostic_event.DiagnosticEvent_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const DiagnosticEvent_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<DiagnosticEvent_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.diagnostic_event.DiagnosticEvent_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.diagnostic_event.DiagnosticEvent_array_port)
    MergeFrom(*source);
  }
}

void DiagnosticEvent_array_port::MergeFrom(const DiagnosticEvent_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.diagnostic_event.DiagnosticEvent_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void DiagnosticEvent_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.diagnostic_event.DiagnosticEvent_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DiagnosticEvent_array_port::CopyFrom(const DiagnosticEvent_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.diagnostic_event.DiagnosticEvent_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DiagnosticEvent_array_port::IsInitialized() const {
  return true;
}

void DiagnosticEvent_array_port::InternalSwap(DiagnosticEvent_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata DiagnosticEvent_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace diagnostic_event
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::si::diagnostic_event::DiagnosticEvent* Arena::CreateMaybeMessage< ::pb::si::diagnostic_event::DiagnosticEvent >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::diagnostic_event::DiagnosticEvent >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::si::diagnostic_event::DiagnosticEvent_array_port* Arena::CreateMaybeMessage< ::pb::si::diagnostic_event::DiagnosticEvent_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::diagnostic_event::DiagnosticEvent_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
