// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: si/core_slot_prototype.proto

#include "si/core_slot_prototype.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_si_2fcore_5fslot_5fprototype_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_CoreSlotPrototype_si_2fcore_5fslot_5fprototype_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_si_2fslot_5fdimension_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_SlotDimension_si_2fslot_5fdimension_2eproto;
namespace pb {
namespace si {
namespace core_slot_prototype {
class CoreSlotPrototypeDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<CoreSlotPrototype> _instance;
} _CoreSlotPrototype_default_instance_;
class CoreSlotPrototype_array_portDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<CoreSlotPrototype_array_port> _instance;
} _CoreSlotPrototype_array_port_default_instance_;
}  // namespace core_slot_prototype
}  // namespace si
}  // namespace pb
static void InitDefaultsscc_info_CoreSlotPrototype_si_2fcore_5fslot_5fprototype_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::core_slot_prototype::_CoreSlotPrototype_default_instance_;
    new (ptr) ::pb::si::core_slot_prototype::CoreSlotPrototype();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::core_slot_prototype::CoreSlotPrototype::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_CoreSlotPrototype_si_2fcore_5fslot_5fprototype_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_CoreSlotPrototype_si_2fcore_5fslot_5fprototype_2eproto}, {
      &scc_info_SlotDimension_si_2fslot_5fdimension_2eproto.base,}};

static void InitDefaultsscc_info_CoreSlotPrototype_array_port_si_2fcore_5fslot_5fprototype_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::pb::si::core_slot_prototype::_CoreSlotPrototype_array_port_default_instance_;
    new (ptr) ::pb::si::core_slot_prototype::CoreSlotPrototype_array_port();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::pb::si::core_slot_prototype::CoreSlotPrototype_array_port::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_CoreSlotPrototype_array_port_si_2fcore_5fslot_5fprototype_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_CoreSlotPrototype_array_port_si_2fcore_5fslot_5fprototype_2eproto}, {
      &scc_info_CoreSlotPrototype_si_2fcore_5fslot_5fprototype_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_si_2fcore_5fslot_5fprototype_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_si_2fcore_5fslot_5fprototype_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_si_2fcore_5fslot_5fprototype_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_si_2fcore_5fslot_5fprototype_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::pb::si::core_slot_prototype::CoreSlotPrototype, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::core_slot_prototype::CoreSlotPrototype, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::core_slot_prototype::CoreSlotPrototype, enabled_nu_),
  PROTOBUF_FIELD_OFFSET(::pb::si::core_slot_prototype::CoreSlotPrototype, length_m_),
  PROTOBUF_FIELD_OFFSET(::pb::si::core_slot_prototype::CoreSlotPrototype, width_m_),
  2,
  1,
  0,
  PROTOBUF_FIELD_OFFSET(::pb::si::core_slot_prototype::CoreSlotPrototype_array_port, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::pb::si::core_slot_prototype::CoreSlotPrototype_array_port, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::pb::si::core_slot_prototype::CoreSlotPrototype_array_port, data_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::pb::si::core_slot_prototype::CoreSlotPrototype)},
  { 11, 17, sizeof(::pb::si::core_slot_prototype::CoreSlotPrototype_array_port)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::core_slot_prototype::_CoreSlotPrototype_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::pb::si::core_slot_prototype::_CoreSlotPrototype_array_port_default_instance_),
};

const char descriptor_table_protodef_si_2fcore_5fslot_5fprototype_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\034si/core_slot_prototype.proto\022\031pb.si.co"
  "re_slot_prototype\032\027si/slot_dimension.pro"
  "to\"\227\001\n\021CoreSlotPrototype\022\023\n\nenabled_nu\030\247"
  "\010 \001(\010\0226\n\010length_m\030\263\020 \001(\0132#.pb.si.slot_di"
  "mension.SlotDimension\0225\n\007width_m\030\221\r \001(\0132"
  "#.pb.si.slot_dimension.SlotDimension\"[\n\034"
  "CoreSlotPrototype_array_port\022;\n\004data\030\202\021 "
  "\003(\0132,.pb.si.core_slot_prototype.CoreSlot"
  "Prototype"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_si_2fcore_5fslot_5fprototype_2eproto_deps[1] = {
  &::descriptor_table_si_2fslot_5fdimension_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_si_2fcore_5fslot_5fprototype_2eproto_sccs[2] = {
  &scc_info_CoreSlotPrototype_si_2fcore_5fslot_5fprototype_2eproto.base,
  &scc_info_CoreSlotPrototype_array_port_si_2fcore_5fslot_5fprototype_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_si_2fcore_5fslot_5fprototype_2eproto_once;
static bool descriptor_table_si_2fcore_5fslot_5fprototype_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_si_2fcore_5fslot_5fprototype_2eproto = {
  &descriptor_table_si_2fcore_5fslot_5fprototype_2eproto_initialized, descriptor_table_protodef_si_2fcore_5fslot_5fprototype_2eproto, "si/core_slot_prototype.proto", 329,
  &descriptor_table_si_2fcore_5fslot_5fprototype_2eproto_once, descriptor_table_si_2fcore_5fslot_5fprototype_2eproto_sccs, descriptor_table_si_2fcore_5fslot_5fprototype_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_si_2fcore_5fslot_5fprototype_2eproto::offsets,
  file_level_metadata_si_2fcore_5fslot_5fprototype_2eproto, 2, file_level_enum_descriptors_si_2fcore_5fslot_5fprototype_2eproto, file_level_service_descriptors_si_2fcore_5fslot_5fprototype_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_si_2fcore_5fslot_5fprototype_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_si_2fcore_5fslot_5fprototype_2eproto), true);
namespace pb {
namespace si {
namespace core_slot_prototype {

// ===================================================================

void CoreSlotPrototype::InitAsDefaultInstance() {
  ::pb::si::core_slot_prototype::_CoreSlotPrototype_default_instance_._instance.get_mutable()->length_m_ = const_cast< ::pb::si::slot_dimension::SlotDimension*>(
      ::pb::si::slot_dimension::SlotDimension::internal_default_instance());
  ::pb::si::core_slot_prototype::_CoreSlotPrototype_default_instance_._instance.get_mutable()->width_m_ = const_cast< ::pb::si::slot_dimension::SlotDimension*>(
      ::pb::si::slot_dimension::SlotDimension::internal_default_instance());
}
class CoreSlotPrototype::_Internal {
 public:
  using HasBits = decltype(std::declval<CoreSlotPrototype>()._has_bits_);
  static void set_has_enabled_nu(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static const ::pb::si::slot_dimension::SlotDimension& length_m(const CoreSlotPrototype* msg);
  static void set_has_length_m(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::pb::si::slot_dimension::SlotDimension& width_m(const CoreSlotPrototype* msg);
  static void set_has_width_m(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

const ::pb::si::slot_dimension::SlotDimension&
CoreSlotPrototype::_Internal::length_m(const CoreSlotPrototype* msg) {
  return *msg->length_m_;
}
const ::pb::si::slot_dimension::SlotDimension&
CoreSlotPrototype::_Internal::width_m(const CoreSlotPrototype* msg) {
  return *msg->width_m_;
}
void CoreSlotPrototype::clear_length_m() {
  if (length_m_ != nullptr) length_m_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
void CoreSlotPrototype::clear_width_m() {
  if (width_m_ != nullptr) width_m_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
CoreSlotPrototype::CoreSlotPrototype()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.core_slot_prototype.CoreSlotPrototype)
}
CoreSlotPrototype::CoreSlotPrototype(const CoreSlotPrototype& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from._internal_has_width_m()) {
    width_m_ = new ::pb::si::slot_dimension::SlotDimension(*from.width_m_);
  } else {
    width_m_ = nullptr;
  }
  if (from._internal_has_length_m()) {
    length_m_ = new ::pb::si::slot_dimension::SlotDimension(*from.length_m_);
  } else {
    length_m_ = nullptr;
  }
  enabled_nu_ = from.enabled_nu_;
  // @@protoc_insertion_point(copy_constructor:pb.si.core_slot_prototype.CoreSlotPrototype)
}

void CoreSlotPrototype::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_CoreSlotPrototype_si_2fcore_5fslot_5fprototype_2eproto.base);
  ::memset(&width_m_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&enabled_nu_) -
      reinterpret_cast<char*>(&width_m_)) + sizeof(enabled_nu_));
}

CoreSlotPrototype::~CoreSlotPrototype() {
  // @@protoc_insertion_point(destructor:pb.si.core_slot_prototype.CoreSlotPrototype)
  SharedDtor();
}

void CoreSlotPrototype::SharedDtor() {
  if (this != internal_default_instance()) delete width_m_;
  if (this != internal_default_instance()) delete length_m_;
}

void CoreSlotPrototype::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const CoreSlotPrototype& CoreSlotPrototype::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_CoreSlotPrototype_si_2fcore_5fslot_5fprototype_2eproto.base);
  return *internal_default_instance();
}


void CoreSlotPrototype::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.core_slot_prototype.CoreSlotPrototype)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(width_m_ != nullptr);
      width_m_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(length_m_ != nullptr);
      length_m_->Clear();
    }
  }
  enabled_nu_ = false;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* CoreSlotPrototype::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional bool enabled_nu = 1063;
      case 1063:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          _Internal::set_has_enabled_nu(&has_bits);
          enabled_nu_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.si.slot_dimension.SlotDimension width_m = 1681;
      case 1681:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 138)) {
          ptr = ctx->ParseMessage(_internal_mutable_width_m(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .pb.si.slot_dimension.SlotDimension length_m = 2099;
      case 2099:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 154)) {
          ptr = ctx->ParseMessage(_internal_mutable_length_m(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* CoreSlotPrototype::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.core_slot_prototype.CoreSlotPrototype)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional bool enabled_nu = 1063;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1063, this->_internal_enabled_nu(), target);
  }

  // optional .pb.si.slot_dimension.SlotDimension width_m = 1681;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1681, _Internal::width_m(this), target, stream);
  }

  // optional .pb.si.slot_dimension.SlotDimension length_m = 2099;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2099, _Internal::length_m(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.core_slot_prototype.CoreSlotPrototype)
  return target;
}

size_t CoreSlotPrototype::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.core_slot_prototype.CoreSlotPrototype)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional .pb.si.slot_dimension.SlotDimension width_m = 1681;
    if (cached_has_bits & 0x00000001u) {
      total_size += 2 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *width_m_);
    }

    // optional .pb.si.slot_dimension.SlotDimension length_m = 2099;
    if (cached_has_bits & 0x00000002u) {
      total_size += 3 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *length_m_);
    }

    // optional bool enabled_nu = 1063;
    if (cached_has_bits & 0x00000004u) {
      total_size += 2 + 1;
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

void CoreSlotPrototype::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.core_slot_prototype.CoreSlotPrototype)
  GOOGLE_DCHECK_NE(&from, this);
  const CoreSlotPrototype* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<CoreSlotPrototype>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.core_slot_prototype.CoreSlotPrototype)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.core_slot_prototype.CoreSlotPrototype)
    MergeFrom(*source);
  }
}

void CoreSlotPrototype::MergeFrom(const CoreSlotPrototype& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.core_slot_prototype.CoreSlotPrototype)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_width_m()->::pb::si::slot_dimension::SlotDimension::MergeFrom(from._internal_width_m());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_length_m()->::pb::si::slot_dimension::SlotDimension::MergeFrom(from._internal_length_m());
    }
    if (cached_has_bits & 0x00000004u) {
      enabled_nu_ = from.enabled_nu_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void CoreSlotPrototype::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.core_slot_prototype.CoreSlotPrototype)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CoreSlotPrototype::CopyFrom(const CoreSlotPrototype& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.core_slot_prototype.CoreSlotPrototype)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CoreSlotPrototype::IsInitialized() const {
  return true;
}

void CoreSlotPrototype::InternalSwap(CoreSlotPrototype* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(width_m_, other->width_m_);
  swap(length_m_, other->length_m_);
  swap(enabled_nu_, other->enabled_nu_);
}

::PROTOBUF_NAMESPACE_ID::Metadata CoreSlotPrototype::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void CoreSlotPrototype_array_port::InitAsDefaultInstance() {
}
class CoreSlotPrototype_array_port::_Internal {
 public:
  using HasBits = decltype(std::declval<CoreSlotPrototype_array_port>()._has_bits_);
};

CoreSlotPrototype_array_port::CoreSlotPrototype_array_port()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
}
CoreSlotPrototype_array_port::CoreSlotPrototype_array_port(const CoreSlotPrototype_array_port& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      data_(from.data_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
}

void CoreSlotPrototype_array_port::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_CoreSlotPrototype_array_port_si_2fcore_5fslot_5fprototype_2eproto.base);
}

CoreSlotPrototype_array_port::~CoreSlotPrototype_array_port() {
  // @@protoc_insertion_point(destructor:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
  SharedDtor();
}

void CoreSlotPrototype_array_port::SharedDtor() {
}

void CoreSlotPrototype_array_port::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const CoreSlotPrototype_array_port& CoreSlotPrototype_array_port::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_CoreSlotPrototype_array_port_si_2fcore_5fslot_5fprototype_2eproto.base);
  return *internal_default_instance();
}


void CoreSlotPrototype_array_port::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* CoreSlotPrototype_array_port::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .pb.si.core_slot_prototype.CoreSlotPrototype data = 2178;
      case 2178:
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

::PROTOBUF_NAMESPACE_ID::uint8* CoreSlotPrototype_array_port::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .pb.si.core_slot_prototype.CoreSlotPrototype data = 2178;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_data_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2178, this->_internal_data(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
  return target;
}

size_t CoreSlotPrototype_array_port::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .pb.si.core_slot_prototype.CoreSlotPrototype data = 2178;
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

void CoreSlotPrototype_array_port::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  const CoreSlotPrototype_array_port* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<CoreSlotPrototype_array_port>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
    MergeFrom(*source);
  }
}

void CoreSlotPrototype_array_port::MergeFrom(const CoreSlotPrototype_array_port& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
}

void CoreSlotPrototype_array_port::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CoreSlotPrototype_array_port::CopyFrom(const CoreSlotPrototype_array_port& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.si.core_slot_prototype.CoreSlotPrototype_array_port)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CoreSlotPrototype_array_port::IsInitialized() const {
  return true;
}

void CoreSlotPrototype_array_port::InternalSwap(CoreSlotPrototype_array_port* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  data_.InternalSwap(&other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata CoreSlotPrototype_array_port::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace core_slot_prototype
}  // namespace si
}  // namespace pb
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::pb::si::core_slot_prototype::CoreSlotPrototype* Arena::CreateMaybeMessage< ::pb::si::core_slot_prototype::CoreSlotPrototype >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::core_slot_prototype::CoreSlotPrototype >(arena);
}
template<> PROTOBUF_NOINLINE ::pb::si::core_slot_prototype::CoreSlotPrototype_array_port* Arena::CreateMaybeMessage< ::pb::si::core_slot_prototype::CoreSlotPrototype_array_port >(Arena* arena) {
  return Arena::CreateInternal< ::pb::si::core_slot_prototype::CoreSlotPrototype_array_port >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
