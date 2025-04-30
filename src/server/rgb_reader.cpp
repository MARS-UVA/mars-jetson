#include "rgb_reader.h"
#include <opencv2/opencv.hpp>

ImageReader::ImageReader()
{
    shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd < 0)
    {
        throw std::runtime_error("Failed to open shared memory");
    }

    shm_ptr = mmap(0, sizeof(SharedImage), PROT_READ, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED)
    {
        close(shm_fd);
        throw std::runtime_error("Failed to mmap shared memory");
    }

    shared_img = static_cast<SharedImage *>(shm_ptr);
}

cv::Mat ImageReader::processImage()
{
    static uint32_t last_frame_id = 0;
    uint32_t current_frame_id = shared_img->frame_id.load(std::memory_order_acquire);

    if (current_frame_id != last_frame_id)
    {
        last_frame_id = current_frame_id;
        size_t size = shared_img->data_size;

        std::vector<uchar> compressed_buf(shared_img->data, shared_img->data + size);
        cv::Mat decodedImg = cv::imdecode(compressed_buf, cv::IMREAD_GRAYSCALE);

        if (decoded.empty())
        {
            std::err << "Getting image from shared memory to send to control station failed..." << std::endl;
        }
        return decodedImg;
    }
}

ImageReader::~ImageReader()
{
    if (shm_ptr != MAP_FAILED)
    {
        munmap(shm_ptr, sizeof(SharedImage));
    }
    if (shm_fd >= 0)
    {
        close(shm_fd);
    }
}
