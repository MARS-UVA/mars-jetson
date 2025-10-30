#include "rgb_writer.h"
#include <opencv2/opencv.hpp>

ImageWriter::ImageWriter()
{
    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0)
    {
        throw std::runtime_error("Failed to open shared memory");
    }

    ftruncate(shm_fd, sizeof(SharedImage));

    shm_ptr = mmap(0, sizeof(SharedImage), PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED)
    {
        close(shm_fd);
        throw std::runtime_error("Failed to mmap shared memory");
    }

    shared_img = static_cast<SharedImage *>(shm_ptr);
    shared_img->frame_id.store(0, std::memory_order_relaxed);
}

void ImageWriter::processImage(int width, int height, uint8_t *monoBuffer)
{
    cv::Mat image(height, width, CV_8UC1, monoBuffer);

    std::vector<uchar> compressed_buf;
    std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 0};
    cv::imencode(".png", image, compressed_buf, params);

    if (compressed_buf.size() > MAX_IMAGE_SIZE)
    {
        throw std::runtime_error("Compressed image too large for shared memory!");
    }

    shared_img->data_size = compressed_buf.size();
    std::memcpy(shared_img->data, compressed_buf.data(), compressed_buf.size());
    shared_img->frame_id.fetch_add(1, std::memory_order_release);
}

ImageWriter::~ImageWriter()
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
