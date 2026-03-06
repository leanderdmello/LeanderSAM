#pragma once

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <string>
#include <cerrno>
#include <system_error>

// Each unique shared memory file is defined here
const std::string DEFAULT_RTP_SHARED_MEMORY = "/rtp_payload_data";
const std::string DEFAULT_PCD_SHARED_MEMORY = "/point_cloud_data";

// 2 MiB - sufficient for 720p (960*720*3)
const std::size_t DEFAULT_RTP_MEMORY_SIZE = 1024 * 1024 * 2;
// 100MiB ? what is a good default max?
const std::size_t DEFAULT_PCD_MEMORY_SIZE = 1024 * 1024 * 100;

// Shared memory layout header
struct ShmLayout {
    std::atomic<uint64_t> seq;              // sequence counter (seqlock)
    std::atomic<uint32_t> subscriber_busy;  // 0 or 1
    std::atomic<size_t>   data_len;         // size of actual data written
    size_t                data_capacity;    // max size of data region
};

// Size of mapping (header)
inline std::size_t shm_required_size() {
    return sizeof(ShmLayout);
}

// Create shared memory
inline void shm_create(const std::string& name, int& fd, std::size_t data_capacity) {
    int flags = O_RDWR;
    flags |= O_CREAT;
    fd = shm_open(name.c_str(), flags, 0666);
    if (fd == -1) 
        throw std::system_error(errno, std::generic_category(), "shm_open");

    // truncate a shm file to a specified length 
    if (ftruncate(fd, static_cast<off_t>(shm_required_size() + data_capacity)) == -1) {
        close(fd);
        shm_unlink(name.c_str());
        throw std::system_error(errno, std::generic_category(), "ftruncate");
    }
}

// Mmap shared memory
// Both subscriber and publisher should be able to create memory.
// Subscriber will only open on a published "Data Ready message"
// so no nead to check if there is already memory mapped
inline ShmLayout* shm_open(const std::string& name, int& fd, std::size_t data_capacity) {
    int flags = O_RDWR;
    fd = shm_open(name.c_str(), flags, 0666);
    if (fd == -1)
        throw std::system_error(errno, std::generic_category(), "shm_open");

    void* map = mmap(nullptr, shm_required_size() + data_capacity, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) {
        close(fd);
        throw std::system_error(errno, std::generic_category(), "mmap");
    }

    // Store data_capacity for shm_close
    auto* shm = reinterpret_cast<ShmLayout*>(map);

    shm->data_capacity = data_capacity;
    return shm;
}

// Close shared memory
inline void shm_close(ShmLayout* ptr, int fd) {
    munmap(reinterpret_cast<void*>(ptr), shm_required_size() + ptr->data_capacity);
    close(fd);
}

// Get pointer to data region
inline unsigned char* shm_data_ptr(ShmLayout* shm) {
    // Data is stored after Layout
    return reinterpret_cast<unsigned char*>(shm + 1);
}

// Cleanup shm memory
inline void shm_destroy(const std::string& name) {
    shm_unlink(name.c_str());
}