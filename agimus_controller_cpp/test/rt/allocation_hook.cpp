// Replaces the global allocation functions of the test executable so that
// every heap allocation or deallocation can be counted, including those made
// inside shared libraries (the dynamic linker resolves their calls to the
// executable's definitions). Forwards to glibc's __libc_* functions.
//
// Do not combine with ASan or another allocator replacement.

#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdlib>
#include <new>

#include "rt_guard.hpp"

extern "C" {
void* __libc_malloc(std::size_t size);
void __libc_free(void* ptr);
void* __libc_calloc(std::size_t n, std::size_t size);
void* __libc_realloc(void* ptr, std::size_t size);
void* __libc_memalign(std::size_t alignment, std::size_t size);
}

namespace {

thread_local bool t_active = false;
thread_local std::size_t t_count = 0;
thread_local std::size_t t_bytes = 0;

// Read once, at static initialisation. Before that it is false.
const bool g_abort_on_alloc = [] {
  const char* value = std::getenv("AGIMUS_RT_ABORT_ON_ALLOC");
  return value != nullptr && value[0] == '1';
}();

inline void record(std::size_t bytes) noexcept {
  if (t_active) {
    ++t_count;
    t_bytes += bytes;
    if (g_abort_on_alloc) {
      std::abort();
    }
  }
}

void* aligned(std::size_t alignment, std::size_t size) noexcept {
  record(size);
  return __libc_memalign(alignment, size);
}

void* new_or_throw(std::size_t size) {
  void* p = std::malloc(size == 0 ? 1 : size);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void* aligned_new_or_throw(std::size_t size, std::align_val_t al) {
  void* p = aligned(static_cast<std::size_t>(al), size == 0 ? 1 : size);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

}  // namespace

namespace agimus_controller::test {

void start_counting() noexcept {
  t_count = 0;
  t_bytes = 0;
  t_active = true;
}

AllocationCount stop_counting() noexcept {
  t_active = false;
  return {t_count, t_bytes};
}

AllocationCount current_count() noexcept { return {t_count, t_bytes}; }

}  // namespace agimus_controller::test

// ---- C allocation functions ------------------------------------------------

extern "C" {

void* malloc(std::size_t size) noexcept {
  record(size);
  return __libc_malloc(size);
}

void free(void* ptr) noexcept {
  if (ptr != nullptr) {
    record(0);
  }
  __libc_free(ptr);
}

void* calloc(std::size_t n, std::size_t size) noexcept {
  record(n * size);
  return __libc_calloc(n, size);
}

void* realloc(void* ptr, std::size_t size) noexcept {
  record(size);
  return __libc_realloc(ptr, size);
}

int posix_memalign(void** out, std::size_t alignment,
                   std::size_t size) noexcept {
  if (alignment == 0 || (alignment & (alignment - 1)) != 0 ||
      alignment % sizeof(void*) != 0) {
    return EINVAL;
  }
  void* p = aligned(alignment, size);
  if (p == nullptr) {
    return ENOMEM;
  }
  *out = p;
  return 0;
}

void* aligned_alloc(std::size_t alignment, std::size_t size) noexcept {
  return aligned(alignment, size);
}

void* memalign(std::size_t alignment, std::size_t size) noexcept {
  return aligned(alignment, size);
}

void* valloc(std::size_t size) noexcept {
  return aligned(static_cast<std::size_t>(sysconf(_SC_PAGESIZE)), size);
}

void* pvalloc(std::size_t size) noexcept {
  const auto page = static_cast<std::size_t>(sysconf(_SC_PAGESIZE));
  return aligned(page, (size + page - 1) / page * page);
}

}  // extern "C"

// ---- C++ allocation functions ----------------------------------------------
// Plain new/delete go through malloc/free above, so each call counts once.

void* operator new(std::size_t size) { return new_or_throw(size); }
void* operator new[](std::size_t size) { return new_or_throw(size); }
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  return std::malloc(size == 0 ? 1 : size);
}
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
  return std::malloc(size == 0 ? 1 : size);
}
void* operator new(std::size_t size, std::align_val_t al) {
  return aligned_new_or_throw(size, al);
}
void* operator new[](std::size_t size, std::align_val_t al) {
  return aligned_new_or_throw(size, al);
}
void* operator new(std::size_t size, std::align_val_t al,
                   const std::nothrow_t&) noexcept {
  return aligned(static_cast<std::size_t>(al), size == 0 ? 1 : size);
}
void* operator new[](std::size_t size, std::align_val_t al,
                     const std::nothrow_t&) noexcept {
  return aligned(static_cast<std::size_t>(al), size == 0 ? 1 : size);
}

void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete(void* p, std::align_val_t,
                     const std::nothrow_t&) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::align_val_t,
                       const std::nothrow_t&) noexcept {
  std::free(p);
}
