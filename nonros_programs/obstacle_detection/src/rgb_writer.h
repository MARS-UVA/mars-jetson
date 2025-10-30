#pragma once

#include <stdexcept>
#include <filesystem>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

#define MAX_IMAGE_SIZE 1048576

#define SHM_NAME "/front_image_shm"

struct SharedImage {
    std::atomic<uint32_t> frame_id;
    std::atomic<size_t> data_size;
    uint8_t data[MAX_IMAGE_SIZE];
};

class ImageWriter {
public:
ImageWriter();
    ~ImageWriter();

    void processImage();

private:
    int shm_fd;
    void* shm_ptr;
    SharedImage* shared_img;
};
