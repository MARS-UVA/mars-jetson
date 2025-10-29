#pragma once

#include <stdexcept>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdint>

#define MAX_IMAGE_SIZE 1048576

#define SHM_NAME "/front_image_shm"

struct SharedImage {
    std::atomic<uint32_t> frame_id;
    std::atomic<size_t> data_size;
    uint8_t data[MAX_IMAGE_SIZE];
};

class ImageReader {
public:
    ImageReader();
    ~ImageReader();

    void processImage();

private:
    int shm_fd;
    void* shm_ptr;
    SharedImage* shared_img;
    uint32_t last_frame_id;
};
