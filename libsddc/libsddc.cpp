#include "libsddc.h"
#include "config.h"
#include "RadioHandler.h"
#include "r2iq.h"
#include "fft_mt_r2iq.h"

#include <thread>
#include <chrono>

struct sddc
{
    SDDCStatus status;
    RadioHandlerClass* handler;
    uint8_t led;
    int samplerateidx;
    double freq;
    int output_int16;   // 0 = CF32 I/Q (default, DDC output); 1 = raw real int16 ADC

    sddc_read_async_cb_t callback;
    void *callback_context;
};

// Bridge for the DEFAULT (CF32 I/Q) path: RadioHandler runs the r2iq DDC and emits
// complex float I/Q — `len` complex samples, `data` = 2*len interleaved floats. We
// forward it as CF32 (data_size in bytes = len*2*sizeof(float)). Unused in raw mode.
static void Callback(void* context, const float* data, uint32_t len)
{
    sddc_t *t = (sddc_t*)context;
    if (t == nullptr || t->callback == nullptr)
        return;
    t->callback(len * 2 * (uint32_t)sizeof(float), (uint8_t*)data, t->callback_context);
}

// libsddc_output: the r2iq controller for libsddc, supporting two output modes.
//
// DEFAULT (CF32 I/Q): inherits fft_mt_r2iq and runs the full "real-to-IQ" DSP — the
// SDDC's software digital down-converter. The DSP fills the output ring buffer, which
// RadioHandlerClass::OnDataPacket() drains and forwards via Callback() above as CF32.
//
// RAW (int16): when sddc_set_stream_format() selects raw mode, TurnOn() instead starts
// a worker that drains the *input* ring buffer and hands the unprocessed real 16-bit
// ADC samples straight to the callback, bypassing the DSP. (The LTC2208 is a real
// 16-bit ADC; I/Q only exists after the DSP — so raw gives you the un-downconverted
// stream, e.g. for full-band capture.) The worker is also what keeps the input buffer
// drained so it can't fill and deadlock the USB transfer callbacks.
class libsddc_output : public fft_mt_r2iq {
public:
    explicit libsddc_output(sddc_t* owner) : owner(owner) {}
    ~libsddc_output() override
    {
        this->r2iqOn = false;
        if (rawInput) rawInput->Stop();
        if (rawWorker.joinable()) rawWorker.join();
    }

    void Init(float gain, ringbuffer<int16_t>* input, ringbuffer<float>* obuffers) override
    {
        rawInput = input;
        rawOutput = obuffers;
        fft_mt_r2iq::Init(gain, input, obuffers);
    }

    void TurnOn() override
    {
        if (owner->output_int16)
        {
            this->r2iqOn = true;
            rawInput->Start();
            rawOutput->Start();   // so RadioHandlerClass::OnDataPacket() can be released on Stop
            rawWorker = std::thread([this]() { this->rawLoop(); });
        }
        else
        {
            fft_mt_r2iq::TurnOn();   // run the real DDC; CF32 delivered via Callback()
        }
    }

    void TurnOff() override
    {
        if (owner->output_int16)
        {
            this->r2iqOn = false;
            rawInput->Stop();    // release our worker if blocked in getReadPtr()
            rawOutput->Stop();   // release RadioHandlerClass::OnDataPacket()
            if (rawWorker.joinable())
                rawWorker.join();
        }
        else
        {
            fft_mt_r2iq::TurnOff();
        }
    }

private:
    void rawLoop()
    {
        while (this->r2iqOn)
        {
            const int16_t* block = rawInput->getReadPtr();
            if (!this->r2iqOn)   // woken by Stop() during shutdown
                break;
            uint32_t bytes = (uint32_t)rawInput->getBlockSize() * sizeof(int16_t);
            if (owner->callback != nullptr)
                owner->callback(bytes, (uint8_t*)block, owner->callback_context);
            rawInput->ReadDone();
        }
    }

    sddc_t* owner;
    ringbuffer<int16_t>* rawInput = nullptr;
    ringbuffer<float>* rawOutput = nullptr;
    std::thread rawWorker;
};

int sddc_get_device_count()
{
    return 1;
}

int sddc_get_device_info(struct sddc_device_info **sddc_device_infos)
{
    auto ret = new sddc_device_info();
    const char *todo = "TODO";
    ret->manufacturer = todo;
    ret->product = todo;
    ret->serial_number = todo;

    *sddc_device_infos = ret;

    return 1;
}

int sddc_free_device_info(struct sddc_device_info *sddc_device_infos)
{
    delete sddc_device_infos;
    return 0;
}

sddc_t *sddc_open(int index, const char* imagefile)
{
    auto ret_val = new sddc_t();

    fx3class *fx3 = CreateUsbHandler();
    if (fx3 == nullptr)
    {
        return nullptr;
    }

    // open the firmware
    unsigned char* res_data;
    uint32_t res_size;

    FILE *fp = fopen(imagefile, "rb");
    if (fp == nullptr)
    {
        return nullptr;
    }

    fseek(fp, 0, SEEK_END);
    res_size = ftell(fp);
    res_data = (unsigned char*)malloc(res_size);
    fseek(fp, 0, SEEK_SET);
    if (fread(res_data, 1, res_size, fp) != res_size)
        return nullptr;

    bool openOK = fx3->Open();
    if (!openOK)
        return nullptr;

    ret_val->handler = new RadioHandlerClass();

    // Install our output controller. By default it runs the r2iq DDC and delivers
    // CF32 I/Q via Callback(); sddc_set_stream_format() can switch it to raw int16.
    // The sddc handle is the callback context.
    if (ret_val->handler->Init(fx3, Callback, new libsddc_output(ret_val), ret_val))
    {
        ret_val->status = SDDC_STATUS_READY;
        ret_val->samplerateidx = 0;
    }

    return ret_val;
}

void sddc_close(sddc_t *that)
{
    if (that->handler)
        delete that->handler;
    delete that;
}

enum SDDCStatus sddc_get_status(sddc_t *t)
{
    return t->status;
}

enum SDDCHWModel sddc_get_hw_model(sddc_t *t)
{
    switch(t->handler->getModel())
    {
        case RadioModel::BBRF103:
            return HW_BBRF103;
        case RadioModel::HF103:
            return HW_HF103;
        case RadioModel::RX888:
            return HW_RX888;
        case RadioModel::RX888r2:
            return HW_RX888R2;
        case RadioModel::RX888r3:
            return HW_RX888R3;
        case RadioModel::RX999:
            return HW_RX999;
        default:
            return HW_NORADIO;
    }
}

const char *sddc_get_hw_model_name(sddc_t *t)
{
    return t->handler->getName();
}

uint16_t sddc_get_firmware(sddc_t *t)
{
    return t->handler->GetFirmware();
}

const double *sddc_get_frequency_range(sddc_t *t)
{
    return nullptr;
}

enum RFMode sddc_get_rf_mode(sddc_t *t)
{
    switch(t->handler->GetmodeRF())
    {
        case HFMODE:
            return RFMode::HF_MODE;
        case VHFMODE:
            return RFMode::VHF_MODE;
        default:
            return RFMode::NO_RF_MODE;
    }
}

int sddc_set_rf_mode(sddc_t *t, enum RFMode rf_mode)
{
    switch (rf_mode)
    {
    case VHF_MODE:
        t->handler->UpdatemodeRF(VHFMODE);
        break;
    case HF_MODE:
        t->handler->UpdatemodeRF(HFMODE);
    default:
        return -1;
    }

    return 0;
}

/* LED functions */
int sddc_led_on(sddc_t *t, uint8_t led_pattern)
{
    if (led_pattern & YELLOW_LED)
        t->handler->uptLed(0, true);
    if (led_pattern & RED_LED)
        t->handler->uptLed(1, true);
    if (led_pattern & BLUE_LED)
        t->handler->uptLed(2, true);

    t->led |= led_pattern;

    return 0;
}

int sddc_led_off(sddc_t *t, uint8_t led_pattern)
{
    if (led_pattern & YELLOW_LED)
        t->handler->uptLed(0, false);
    if (led_pattern & RED_LED)
        t->handler->uptLed(1, false);
    if (led_pattern & BLUE_LED)
        t->handler->uptLed(2, false);

    t->led &= ~led_pattern;

    return 0;
}

int sddc_led_toggle(sddc_t *t, uint8_t led_pattern)
{
    t->led = t->led ^ led_pattern;
    if (t->led & YELLOW_LED)
        t->handler->uptLed(0, false);
    if (t->led & RED_LED)
        t->handler->uptLed(1, false);
    if (t->led & BLUE_LED)
        t->handler->uptLed(2, false);

    return 0;
}


/* ADC functions */
int sddc_get_adc_dither(sddc_t *t)
{
    return t->handler->GetDither();
}

int sddc_set_adc_dither(sddc_t *t, int dither)
{
    t->handler->UptDither(dither != 0);
    return 0;
}

int sddc_get_adc_random(sddc_t *t)
{
    return t->handler->GetRand();
}

int sddc_set_adc_random(sddc_t *t, int random)
{
    t->handler->UptRand(random != 0);
    return 0;
}

/* HF block functions */
double sddc_get_hf_attenuation(sddc_t *t)
{
    return 0;
}

int sddc_set_hf_attenuation(sddc_t *t, double attenuation)
{
    return 0;
}

int sddc_get_hf_bias(sddc_t *t)
{
    return t->handler->GetBiasT_HF();
}

int sddc_set_hf_bias(sddc_t *t, int bias)
{
    t->handler->UpdBiasT_HF(bias != 0);
    return 0;
}


/* VHF block and VHF/UHF tuner functions */
double sddc_get_tuner_frequency(sddc_t *t)
{
    return t->freq;
}

int sddc_set_tuner_frequency(sddc_t *t, double frequency)
{
    t->freq = t->handler->TuneLO((uint64_t)frequency);

    return 0;
}

int sddc_get_tuner_rf_attenuations(sddc_t *t, const double *attenuations[])
{
    return 0;
}

double sddc_get_tuner_rf_attenuation(sddc_t *t)
{
    return 0;
}

int sddc_set_tuner_rf_attenuation(sddc_t *t, double attenuation)
{
    //TODO, convert double to index
    t->handler->UpdateattRF(5);
    return 0;
}

int sddc_get_tuner_if_attenuations(sddc_t *t, const double *attenuations[])
{
    // TODO
    return 0;
}

double sddc_get_tuner_if_attenuation(sddc_t *t)
{
    return 0;
}

int sddc_set_tuner_if_attenuation(sddc_t *t, double attenuation)
{
    return 0;
}

int sddc_get_vhf_bias(sddc_t *t)
{
    return t->handler->GetBiasT_VHF();
}

int sddc_set_vhf_bias(sddc_t *t, int bias)
{
    t->handler->UpdBiasT_VHF(bias != 0);
    return 0;
}

double sddc_get_sample_rate(sddc_t *t)
{
    return 0;
}

int sddc_set_sample_rate(sddc_t *t, double sample_rate)
{
    // Rate semantics depend on the output format, so call sddc_set_stream_format()
    // (if at all) BEFORE this.
    if (t->output_int16)
    {
        // Raw mode: no decimation, so the requested rate is the ADC sampling clock.
        // RadioHandler clamps it to [8 MHz, 128 MHz] and reprograms the ADC clock.
        int actual = t->handler->SetSampleRate((int)sample_rate);
        return actual > 0 ? 0 : -1;
    }

    // CF32 I/Q mode: the rate is the *decimated* DDC output rate. The ADC runs at
    // DEFAULT_ADC_FREQ and RadioHandler uses the 6-band model
    // (decimate = 5 - samplerateidx; output = adc / 2^(decimate+1)), matching
    // SoapySDDC. For a 128 MHz ADC: idx 0..5 = 2/4/8/16/32/64 MSps.
    for (int idx = 0; idx <= 5; ++idx)
    {
        uint32_t rate = DEFAULT_ADC_FREQ >> ((5 - idx) + 1);
        if ((int64_t)rate == (int64_t)sample_rate)
        {
            t->samplerateidx = idx;
            return 0;
        }
    }
    return -1;
}

int sddc_set_stream_format(sddc_t *t, enum sddc_stream_format format)
{
    // Must be called before sddc_set_sample_rate() / sddc_start_streaming().
    t->output_int16 = (format == SDDC_STREAM_INT16);
    return 0;
}

int sddc_set_async_params(sddc_t *t, uint32_t frame_size, 
                          uint32_t num_frames, sddc_read_async_cb_t callback,
                          void *callback_context)
{
    // TODO: ignore frame_size, num_frames
    t->callback = callback;
    t->callback_context = callback_context;
    return 0;
}

int sddc_start_streaming(sddc_t *t)
{
    t->handler->Start(t->samplerateidx);
    return 0;
}

int sddc_handle_events(sddc_t *t)
{
    // Data is delivered asynchronously from RadioHandler's worker thread, so there
    // is nothing to pump here. Sleep briefly so a caller polling this in a tight
    // loop (e.g. sddc_stream_test) doesn't spin a CPU core at 100%.
    (void)t;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return 0;
}

int sddc_stop_streaming(sddc_t *t)
{
    t->handler->Stop();
    return 0;
}

int sddc_reset_status(sddc_t *t)
{
    return 0;
}

int sddc_read_sync(sddc_t *t, uint8_t *data, int length, int *transferred)
{
    return 0;
}
