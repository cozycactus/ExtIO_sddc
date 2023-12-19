#include "LibusbHandler.h"
#include "../config.h"


#include "libusb/libusb.hpp"
#include <condition_variable>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <cinttypes>

LibusbHandler::LibusbHandler() :
	fx3dev (nullptr),
	Fx3IsOn (false),
	devidx (0)
{
    (void)devidx;
	DbgPrintf("\r\nLibusbHandler\r\n");
}


LibusbHandler::~LibusbHandler() // reset USB device and exit
{
	DbgPrintf("\r\n~LibusbHandler destructor\r\n");
	Close();
    //libusb_exit(nullptr);
}

bool LibusbHandler::GetFx3DeviceStreamer()
{
    DbgPrintf("\r\nGetFx3DeviceStreamer\r\n");
    bool r = false;
    if (fx3dev == nullptr) return r;

    return true;
}

bool LibusbHandler::Enumerate(unsigned char& idx, char* lbuf, size_t bufSize, const uint8_t* fw_data, uint32_t fw_size)
{
    DbgPrintf("\r\nEnumerate\r\n");
    bool r = false;
    strncpy(lbuf, "", bufSize);
    lbuf[bufSize - 1] = '\0';
    
    if (fx3dev == nullptr) {
        fx3dev = new FX3Device();
    }

    if (fx3dev == nullptr) {
        return r;
    }

    if(!fx3dev->Open(idx)) {
        return r;
    }

    
    return true;
}

bool LibusbHandler::Open(const uint8_t* fw_data, uint32_t fw_size)
{
    DbgPrintf("\r\nOpen\r\n");
    int r = false;
    if (!GetFx3DeviceStreamer()) {
		DbgPrintf("Failed to open device\n");
		return r;
	}

    fx3dev->device = libusb_get_device(fx3dev->hDevice);
    r = libusb_get_device_descriptor(fx3dev->device, &fx3dev->descriptor);
    if (r < 0) {
        DbgPrintf("Failed to get device descriptor\n");
        return r;
    }

    r = libusb_get_config_descriptor(fx3dev->device, 0, &fx3dev->config);
    if (r < 0) {
        DbgPrintf("Failed to get config descriptor\n");
        return r;
    }

    fx3dev->interface = &fx3dev->config->interface[0];
    fx3dev->altsetting = &fx3dev->interface->altsetting[0];

    for (int i = 0; i < fx3dev->altsetting->bNumEndpoints; i++) {
        const libusb_endpoint_descriptor *ep = &fx3dev->altsetting->endpoint[i];
        if ((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK) {
            if (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                fx3dev->epBulkIn = (libusb_endpoint_descriptor *)ep;
            }
        }
    }
    int speed = libusb_get_device_speed(fx3dev->device);
    
    if (speed != LIBUSB_SPEED_SUPER) {
        DbgPrintf("Device is not operating at SuperSpeed. Please check your USB connection\n");
        return false;
    }
    DbgPrintf("Device is operating at SuperSpeed\n");
    
    r = libusb_get_ss_endpoint_companion_descriptor(NULL, fx3dev->epBulkIn, &fx3dev->ep_comp_desc);
    if (r < 0) {
        DbgPrintf("Failed to get endpoint companion descriptor\n");
        return r;
    }

    fx3dev->epBulkIn->wMaxPacketSize *= (fx3dev->ep_comp_desc->bMaxBurst + 1);

    //unsigned short PktsPerFrame = (fx3dev->epBulkIn->wMaxPacketSize & 0x1800) >> 11;

    long pktSize = fx3dev->epBulkIn->wMaxPacketSize;
    
    long ppx = transferSize / pktSize;
    DbgPrintf("buffer transferSize = %d. packet size = %ld. packets per transfer = %ld\n"
		, transferSize, pktSize, ppx);

	uint8_t data[4];
    GetHardwareInfo((uint32_t*)&data);

    if (data[1] != FIRMWARE_VER_MAJOR ||
		data[2] != FIRMWARE_VER_MINOR)
	{
		DbgPrintf("Firmware version mismatch %d.%d != %d.%d (actual)\n", FIRMWARE_VER_MAJOR, FIRMWARE_VER_MINOR, data[1], data[2]);
		Control(RESETFX3);
		return false;
	}

	Fx3IsOn = true;
	return Fx3IsOn;          // init success
}

bool LibusbHandler::Control(FX3Command command, uint8_t data)
{
    int r = libusb_control_transfer(fx3dev->hDevice,
                                    LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT,
                                    command,
                                    0, // Value
                                    0, // Index
                                    &data,
                                    1, // Length of data
                                    5000); // Timeout in milliseconds

    DbgPrintf("FX3FWControl %x .%x %x\n", r, command, data);
    if (r < 0) {
        // Handle the error
        DbgPrintf("FX3FWControl failed: %s\n", libusb_error_name(r));
        Close();
        return false;
    }
    return true;
}

bool LibusbHandler::Control(FX3Command command, uint32_t data)
{
    int r = libusb_control_transfer(fx3dev->hDevice,
                                    LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT,
                                    command,
                                    0, // Value
                                    0, // Index
                                    (unsigned char*)&data,
                                    sizeof(uint32_t), // Length of data
                                    5000); // Timeout in milliseconds

    DbgPrintf("FX3FWControl %x .%x %x\n", r, command, data);
    if (r < 0) {
        DbgPrintf("FX3FWControl failed: %s\n", libusb_error_name(r));
        Close();
        return false;
    }
    return true;
}

bool LibusbHandler::Control(FX3Command command, uint64_t data)
{
    int r = libusb_control_transfer(fx3dev->hDevice,
                                    LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT,
                                    command,
                                    0, // Value
                                    0, // Index
                                    (unsigned char*)&data,
                                    sizeof(uint64_t), // Length of data
                                    5000); // Timeout in milliseconds

    DbgPrintf("FX3FWControl %x .%x %" PRIx64 "\n", r, command, data);
    if (r < 0) {
        DbgPrintf("FX3FWControl failed: %s\n", libusb_error_name(r));
        Close();
        return false;
    }
    return true;
}

bool LibusbHandler::SetArgument(uint16_t index, uint16_t value)
{
    DbgPrintf("\r\nSetArgument\r\n");
    return true;
}

bool LibusbHandler::GetHardwareInfo(uint32_t* data)
{
    DbgPrintf("\r\nGetHardwareInfo\r\n");


    uint8_t requestType = LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
    uint8_t request = TESTFX3;
    uint16_t value = 0;
    uint16_t index = 0;
    unsigned char buffer[4];
    int length = 4;

#ifdef _DEBUG
    value = 1;
#endif

    int r = libusb_control_transfer(fx3dev->hDevice, requestType, request, value, index, buffer, length, 0);

    if (r < 0) {
        // Handle the error
        DbgPrintf("GetHardwareInfo failed: %d\n", r);
        Close();
        return false;
    }

    // Assuming data is a 32-bit integer and buffer contains 4 bytes
    memcpy(data, buffer, sizeof(uint32_t));
    DbgPrintf("GetHardwareInfo %x .%x %x\n", r, TESTFX3, *data);

    return true;
    
}

bool LibusbHandler::ReadDebugTrace(uint8_t* pdata, uint8_t len)
{
    int r = libusb_control_transfer(fx3dev->hDevice,
                                    LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN,
                                    READINFODEBUG, // This should be your specific request code for reading debug trace
                                    (uint16_t)pdata[0], // Value, assuming pdata[0] holds the relevant value as in your original method
                                    0, // Index
                                    pdata,
                                    len, // Length of data
                                    5000); // Timeout in milliseconds

    if (r < 0) {
        // Handle the error
        //DbgPrintf("ReadDebugTrace failed: %s\n", libusb_error_name(r));
        return false; // Handle error
    }
    return true;
}

bool LibusbHandler::SendI2cbytes(uint8_t i2caddr, uint8_t regaddr, uint8_t* pdata, uint8_t len)
{
    DbgPrintf("\r\nSendI2cbytes\r\n");
    return true;
}

bool LibusbHandler::ReadI2cbytes(uint8_t i2caddr, uint8_t regaddr, uint8_t* pdata, uint8_t len)
{
    DbgPrintf("\r\nReadI2cbytes\r\n");
    return true;
}

bool LibusbHandler::Close(void)
{
    fx3dev->Close();            // close class
	delete fx3dev;              // destroy class
	Fx3IsOn = false;
	return true;
}

#define BLOCK_TIMEOUT (80) // block 65.536 ms timeout is 80

#define USB_READ_CONCURRENT 4

bool LibusbHandler::BeginDataXfer(uint8_t *buffer, long transferSize, void** context)
{
   
    return true;
}

void LibusbHandler::CleanupDataXfer(void** context)
{
    
}


void LibusbHandler::AdcSamplesProcess()
{
    DbgPrintf("AdcSamplesProc thread runs\n");
    int actual_length;

    
    for (int n = 0; n < USB_READ_CONCURRENT; n++) {
        uint8_t* ptr = reinterpret_cast<uint8_t*>(inputbuffer->peekWritePtr(n));
        int r = libusb_bulk_transfer(fx3dev->hDevice, 0x81, ptr, 
                                         transferSize, &actual_length, 70);

            if (r != 0) {
                DbgPrintf("Transfer error: %d\n", r);
                break;
            }

            if (static_cast<uint32_t>(actual_length) < transferSize) {
                DbgPrintf("Only read %d bytes, but requested %u\n", actual_length, transferSize);
            }

        
    }

    int read_idx = 0;
    int buf_idx = 0; // Buffer cycle index

    while (run) {
        
            int actual_length;
            inputbuffer->WriteDone();
            uint8_t* ptr = reinterpret_cast<uint8_t*>(inputbuffer->peekWritePtr(USB_READ_CONCURRENT-1));

            // Perform the synchronous USB transfer
            int r = libusb_bulk_transfer(fx3dev->hDevice, 0x81, ptr, 
                                         transferSize, &actual_length, 70); 

            if (r != 0) {
                DbgPrintf("Transfer error: %d\n", r);
                break;
            }

            if (static_cast<uint32_t>(actual_length) < transferSize) {
                DbgPrintf("Only read %d bytes, but requested %u\n", actual_length, transferSize);
            }

            // Increment buffer index for the next transfer
            buf_idx = (buf_idx + 1) % QUEUE_SIZE;
            read_idx = (read_idx + 1) % USB_READ_CONCURRENT;
        
    }


    DbgPrintf("AdcSamplesProc thread_exit\n");
}

void LibusbHandler::StartStream(ringbuffer<int16_t>& input, int numofblock)
{
    DbgPrintf("\r\nStartStream\r\n");
    // Allocate the context and buffers
	inputbuffer = &input;

	// create the thread
	this->numofblock = numofblock;
	run = true;
	adc_samples_thread = new std::thread(
		[this]() {
			this->AdcSamplesProcess();
		}
	);
}

void LibusbHandler::StopStream()
{
    // set the flag
	run = false;
	adc_samples_thread->join();

	// force exit the thread
	inputbuffer = nullptr;

	delete adc_samples_thread;
}


