#ifndef SONIX_H
#define SONIX_H

//#include "sn9c102_config.h"
//#include "sensors/sn9c102_sensor.h"
//#include "sn9c102_devtable.h"
#include "stdlib.h"
#include "stdio.h"
#include "libusb.h"

enum sn9c102_frame_state {
    F_UNUSED,
    F_QUEUED,
    F_GRABBING,
    F_DONE,
    F_ERROR,
};

struct sn9c102_frame_t {
    void* bufmem;
//    struct v4l2_buffer buf;
    enum sn9c102_frame_state state;
//    struct list_head frame;
    unsigned long vma_use_count;
};

enum sn9c102_dev_state {
    DEV_INITIALIZED = 0x01,
    DEV_DISCONNECTED = 0x02,
    DEV_MISCONFIGURED = 0x04,
};

enum sn9c102_io_method {
    IO_NONE = 0,
    IO_READ,
    IO_MMAP,
};

enum sn9c102_stream_state {
    STREAM_OFF = 0,
    STREAM_INTERRUPT,
    STREAM_ON,
};

typedef char sn9c102_sof_header_t[62];

struct sn9c102_sof_t {
    sn9c102_sof_header_t header;
    uint16_t bytesread;
};

struct sn9c102_sysfs_attr {
    uint16_t reg, i2c_reg;
    sn9c102_sof_header_t frame_header;
};

struct sn9c102_module_param {
    uint8_t force_munmap, max_opens;
    uint16_t frame_timeout;
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static DEFINE_MUTEX(sn9c102_sysfs_lock);
#endif

//int sn9c102_write_regs(sn9c102_device *cam , const uint8_t valreg[][], int count);
//int sn9c102_write_reg(libusb_device_handle *dev_handle, uint16_t index, uint8_t value);
//int sn9c102_write_regs(struct sn9c102_device* cam, const uint8_t valreg[][2], int count);
//int sn9c102_write_reg(struct sn9c102_device* cam, uint16_t index, uint8_t value);

//int sn9c102_read_reg(struct sn9c102_device* cam, uint16_t index);
//int sn9c102_pread_reg(struct sn9c102_device* cam, uint16_t index);
//int sn9c102_read_reg(libusb_device_handle *dev_handle, uint16_t index);

struct sn9c102_device {
//    struct video_device* v4ldev;

//    enum sn9c102_bridge bridge;
//    struct sn9c102_sensor sensor;

    libusb_device_handle *dev_handle;
    struct libusb_device *dev;
//    struct urb* urb[SN9C102_URBS];
//    void* transfer_buffer[SN9C102_URBS];
    uint8_t *control_buffer;

//    struct sn9c102_frame_t *frame_current, frame[SN9C102_MAX_FRAMES];
//    struct list_head inqueue, outqueue;
//    u32 frame_count, nbuffers, nreadbuffers;

    enum sn9c102_io_method io;
    enum sn9c102_stream_state stream;

//    struct v4l2_jpegcompression compression;

    struct sn9c102_sysfs_attr sysfs;
    struct sn9c102_sof_t sof;
    uint16_t reg[384];

    struct sn9c102_module_param module_param;

    struct file* priority;
//    struct kref kref;
    enum sn9c102_dev_state state;
//    u8 users;

//    struct completion probe;
//    struct mutex fileop_mutex;
//    spinlock_t queue_lock;
//    wait_queue_head_t wait_open, wait_frame, wait_stream;
};

/*****************************************************************************/
/*
struct sn9c102_device* sn9c102_match_id(struct sn9c102_device* cam, const struct usb_device_id *id)
{
    return usb_match_id(usb_ifnum_to_if(cam->usbdev, 0), id) ? cam : NULL;
}
*/
/*
void sn9c102_attach_sensor(struct sn9c102_device* cam, const struct sn9c102_sensor* sensor)
{
    memcpy(&cam->sensor, sensor, sizeof(struct sn9c102_sensor));
}
*/
/*
enum sn9c102_bridge sn9c102_get_bridge(struct sn9c102_device* cam)
{
    return cam->bridge;
}
*/
/*
struct sn9c102_sensor* sn9c102_get_sensor(struct sn9c102_device* cam)
{
    return &cam->sensor;
}
*/
#endif // SONIX_H

