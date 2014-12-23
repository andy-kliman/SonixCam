#include <string.h>
#include <stdio.h>
#include "sonix.h"

#define DEV_CONFIG 1
#define DEV_INTF 0
#define EP_IN_1 0x81            // От хоста "10000001" #1 , 00000001 - данные, нет синхр, изохронный канал
#define EP_IN_2 0x82            // От хоста "10000010" #2 , 00000010 - данные, нет синхр, канал передачи данных
#define EP_IN_3 0x83            // От хоста "10000011" #3 , 00000011 - данные, нет синхр, канал прерываний
#define EP_OUT 0x01
#define CTRL_IN                 (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define CTRL_OUT                (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)

#define TIMEOUT 500

int main()
{

    int i=0, cnt=0;
    unsigned short VID = 0x045e; //vendor id
    unsigned short PID = 0x00f7; //product id
//    unsigned char EP_out = 0x82; // write into usb  end point
//    unsigned char EP_in  = 0x01; // read from usb end point

//    unsigned char TX[] = {0xC1, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00}; //reqwest
//    unsigned char RX[521]; //answer

    struct libusb_device_descriptor desc; //description of device structure

    libusb_device_handle *dev_handle = NULL;    //a device handle
    libusb_context       *ctx = NULL;           //a libusb session
    libusb_device        *dev = NULL;           //pointer to device
    libusb_device        **devs = NULL;         //pointer to pointer of device
    int r;
//    ssize_t cnt;                                //holding number of devices in list


    //init libusb
    r =  libusb_init(&ctx);
    if(r < 0) {
        printf("Init Error %d\n", r);           // Ошибка инициализации сессии libusb
        return 1;
    }
    libusb_set_debug(ctx, 3);

    // get device list
    cnt = libusb_get_device_list(ctx, &devs);   //get the list of devices
    printf("Количество устройств: %d\n", cnt);

    //get need device descriptor and number
    while ((dev = devs[i++]) != NULL)           //take all connected device one by one
        {
            cnt = libusb_get_device_descriptor(dev, &desc); // look in deckritption of current device
            if(cnt == 0)
                printf("\tУстройство = VID:%04x, PID:%04x, Конф:%04x\n", desc.idVendor, desc.idProduct, desc.bNumConfigurations);

            //test device vendor and product identify
            if( desc.idVendor == VID && desc.idProduct == PID ) // if VID & PID is positive
            {
                // open device
                cnt = libusb_open(dev, &dev_handle);
                if(cnt == 0)
                printf("Открываем наше устройство:\n");
                break;
            }
        }

        // Очищаем список устройств, убрираем ссылки на устройства в нем
        libusb_free_device_list(devs, 1);

        // test to enable device
        if ( dev_handle == NULL )
        {
            printf("Устройство не найдено\n");

            //exit work with usb
            libusb_exit(ctx);
                printf("Закрываем libusb\n");

            return 0;
        }

        // test to device to grubed by kernel
        if(libusb_kernel_driver_active(dev_handle, 0) == 1)
            if(libusb_detach_kernel_driver(dev_handle, 0) == 0) //detach it
                printf("Отсоединяем устройство от драйвера ядра\n");

        // config  device
            cnt = libusb_set_configuration(dev_handle, 1);
            if(cnt == 0)
                printf("Устанавливаем конфигурацию\n");

        // claim interface
            cnt = libusb_claim_interface(dev_handle, 0);
            if( cnt == 0 )
                printf("Запрос %d интерфейса\n", cnt);

//        uint8_t index = valreg[0][1];
//            uint8_t *buff;

//            cnt = sn9c102_write_reg(dev_handle, index, 1);
//            libusb_control_transfer(dev_handle, 0x41, 0x08, index, 0, buff, 1, TIMEOUT);
//        cnt = libusb_control_transfer(dev_handle,
 //                                     0x41,
   //                                   0x08, index, 0,
     //                                 buff, 1, TIMEOUT);
//                    (udev, usb_sndctrlpipe(udev, 0), 0x08, 0x41, index, 0, data, len, SN9C102_CTRL_TIMEOUT);
//        printf("Read from device.\n\tReturn value = %d\n\tAccept bytes = %d\n",cnt);

//static int sn9c102_usb_probe(struct usb_interface* intf, const struct usb_device_id* id)
//{
//    struct libusb_device *dev =
//}

//int sn9c102_write_regs(struct sn9c102_device* cam, const uint8_t valreg[][2], int count)
//{
//    struct libusb_d
//}
        // send data block to device
//        cnt = libusb_bulk_transfer(dev_handle, EP_in, TX, sizeof(TX), &bTransfer, TIMEOUT);
//        printf("Write to device.\n\tReturn value = %d\n\tSend bytes = %s\n",cnt,buff);

        // read answer
//        cnt = libusb_bulk_transfer(dev_handle, EP_out, RX, sizeof(RX), &bTransfer, TIMEOUT);
//        printf("Read from device.\n\tReturn value = %d\n\tAccept bytes = %s\n",cnt,buff);

        //release the claimed interface
        cnt = libusb_release_interface(dev_handle, 0);
        if( cnt == 0 )
            printf("Release 0 interface\n");

        // close usb device
        libusb_close(dev_handle);
            printf("Close device\n");

        //exit work with usb
        libusb_exit(ctx);
            printf("Exit libusb\n");

        //erase dev_handle
        dev_handle = NULL;

        return 0;
}

/*
   Write a sequence of count value/register pairs. Returns -1 after the first
   failed write, or 0 for no errors.
*/

int sn9c102_write_regs(struct sn9c102_device* cam, const uint8_t valreg[][2], int count)
{
    libusb_device_handle* handle = cam->dev_handle;
    uint8_t* buff = cam->control_buffer;
    int i, res;

    for (i = 0; i < count; i++) {
        uint8_t index = valreg[i][1];

        *buff = valreg[i][0];

        res = libusb_control_transfer(handle, 0x41, 0x08, index, 0, buff, 1, TIMEOUT);

        if (res < 0) {
            printf("Failed to write a register (value 0x%02X, index 0x%02X, error %d)", *buff, index, res);
            return -1;
        }

        cam->reg[index] = *buff;
    }

    return 0;
//    libusb_device *dev = cam->dev;
//    printf("\t%d, %s \n",index, data);
//    return libusb_control_transfer(dev_handle, 0x41, 0x08, index, 0, data, len, TIMEOUT);
}

int sn9c102_write_reg(struct sn9c102_device* cam, uint16_t index, uint8_t value)
{
    libusb_device_handle* handle = cam->dev_handle;
    uint8_t* buff = cam->control_buffer;
    int res;
    if (index >= sizeof(cam->reg))
        return -1;

        *buff = value;

        res = libusb_control_transfer(handle, 0x41, 0x08, index, 0, buff, 1, TIMEOUT);
        if (res < 0) {
            printf("Failed to write a register (value 0x%02X, index 0x%02X, error %d)", value, index, res);
            return -1;
        }

        cam->reg[index] = value;

        return 0;
}

/* Внимание: при чтении SN9C10[123] некоторые регистры всегда возвращают 0 */
int sn9c102_read_reg(struct sn9c102_device* cam, uint16_t index)
{
    libusb_device_handle* handle = cam->dev_handle;
    uint8_t* buff = cam->control_buffer;
    int res;

    res = libusb_control_transfer(handle, 0xc1, 0x00, index, 0, buff, 1, TIMEOUT);
    if (res < 0)
        printf("Failed to read a register (index 0x%02X, error %d)",
            index, res);

    return (res >= 0) ? (int)(*buff) : -1;
}

int sn9c102_pread_reg(struct sn9c102_device* cam, uint16_t index)
{
    if (index >= sizeof(cam->reg))
        return -1;

    return cam->reg[index];
}
/*
static int sn9c102_i2c_wait(struct sn9c102_device* cam, const struct sn9c102_sensor* sensor)
{
    int i, r;

    for (i = 1; i <= 5; i++) {
        r = sn9c102_read_reg(cam, 0x08);
        if (r < 0)
            return -EIO;
        if (r & 0x04)
            return 0;
        if (sensor->frequency & SN9C102_I2C_400KHZ)
            udelay(5*16);
        else
            udelay(16*16);
    }
    return -EBUSY;
}
*/
/*
static int sn9c102_i2c_detect_read_error(struct sn9c102_device* cam, const struct sn9c102_sensor* sensor)
{
    int r , err = 0;

    r = sn9c102_read_reg(cam, 0x08);
    if (r < 0)
        err += r;

    if (cam->bridge == BRIDGE_SN9C101 || cam->bridge == BRIDGE_SN9C102) {
        if (!(r & 0x08))
            err += -1;
    } else {
        if (r & 0x08)
            err += -1;
    }

    return err ? -EIO : 0;
}
*/
/*
static int sn9c102_i2c_detect_write_error(struct sn9c102_device* cam, const struct sn9c102_sensor* sensor)
{
    int r;
    r = sn9c102_read_reg(cam, 0x08);
    return (r < 0 || (r >= 0 && (r & 0x08))) ? -EIO : 0;
}
*/
