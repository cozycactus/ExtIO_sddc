#include "SoapySDDC.hpp"
#include "config.h"
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Time.hpp>
#include <cstdint>
#include <sys/types.h>
#include <cstring>
#include <algorithm>

const int MAX_SRATE_IDX = 5;  // Maximum sample rate index
const int MIN_SRATE_IDX = 0;  // Minimum sample rate index
const int DEFAULT_SRATE_IDX = 0;          // Default sample rate index

static void _Callback(void *context, const float *data, uint32_t len)
{
    SoapySDDC *sddc = (SoapySDDC *)context;
    sddc->Callback(context, data, len);
}

int SoapySDDC::Callback(void *context, const float *data, uint32_t len)
{
    // DbgPrintf("SoapySDDC::Callback %d\n", len);
    if (_buf_count == numBuffers)
    {
        _overflowEvent = true;
        return 0;
    }

    auto &buff = _buffs[_buf_tail];
    buff.resize(len * bytesPerSample);
    memcpy(buff.data(), data, len * bytesPerSample);
    _buf_tail = (_buf_tail + 1) % numBuffers;

    {
        std::lock_guard<std::mutex> lock(_buf_mutex);
        _buf_count++;
    }
    _buf_cond.notify_one();

    return 0;
}

SoapySDDC::SoapySDDC(const SoapySDR::Kwargs &args) :
    deviceId(-1),
    sampleRate(2000000),
    numBuffers(16),
    biasTee(false),
    dithering(true),
    RFGain(0.0),
    IFGain(0.0),
    Fx3(CreateUsbHandler()),
    adc_frequency(DEFAULT_ADC_FREQ),
    samplerateidx(DEFAULT_SRATE_IDX),
    RFGainMin(0.0),
    RFGainMax(0.0),
    IFGainMin(0.0),
    IFGainMax(0.0)
{
    
    DbgPrintf("SoapySDDC::SoapySDDC\n");
    if (args.count("adcfreq")) {
        try {
            adc_frequency = std::stoi(args.at("adcfreq"));
        } catch (const std::exception& e) {
            DbgPrintf("SoapySDDC::SoapySDDC - Invalid adcfreq value: %s\n", args.at("adcfreq").c_str());
        }
    }
    if (args.count("srateidx")) {
        try {
            samplerateidx = std::stoi(args.at("srateidx"));
            DbgPrintf("Sample rate index set to %d\n", samplerateidx);
        } catch (const std::exception& e) {
            DbgPrintf("Invalid srateidx value: %s\n", args.at("srateidx").c_str());
            samplerateidx = DEFAULT_SRATE_IDX;
        }
    }

     // Calculate the sample rate based on adc_frequency and samplerateidx
    sampleRate = calculateSampleRate(adc_frequency, samplerateidx);
    DbgPrintf("Calculated sample rate: %.2f Hz\n", sampleRate);

    unsigned char idx = 0;
    DevContext devicelist;
    Fx3->Enumerate(idx, devicelist.dev[0]);
    Fx3->Open();
    RadioHandler.Init(Fx3, _Callback, nullptr, this, adc_frequency);
    
    const float* IFGainSteps;
    IFGainStepsCount = RadioHandler.GetIFGainSteps(&IFGainSteps);
    IFGainMin = *std::min_element(IFGainSteps, IFGainSteps + IFGainStepsCount);
    IFGainMax = *std::max_element(IFGainSteps, IFGainSteps + IFGainStepsCount);
    radioName = RadioHandler.getName();
    if (radioName == "RX888")
    {
        RFGainMin = -20.0;
        RFGainMax = 0.0;
    }
    else if (radioName == "RX888 mkII")
    {
        RFGainMin = -20.0;
        RFGainMax = 0.0;
    }

    if (args.count("bias")) {
        try {
            biasTee = std::stoi(args.at("bias")) != 0;
            DbgPrintf("SoapySDDC::SoapySDDC - biasTee set to %d\n", biasTee);
        } catch (const std::exception& e) {
            DbgPrintf("SoapySDDC::SoapySDDC - Invalid bias value: '%s'. Using default biasTee=%d\n",
                    args.at("bias").c_str(), biasTee);
        }
    }
    


    RadioHandler.UpdBiasT_HF(biasTee);
    RadioHandler.UpdBiasT_VHF(biasTee);
    
}

SoapySDDC::~SoapySDDC(void)
{
    DbgPrintf("SoapySDDC::~SoapySDDC\n");
    RadioHandler.Stop();
    delete Fx3;
    Fx3 = nullptr;

    // RadioHandler.Close();
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapySDDC::getDriverKey(void) const
{
    DbgPrintf("SoapySDDC::getDriverKey\n");
    return "SDDC";
}

std::string SoapySDDC::getHardwareKey(void) const
{
    DbgPrintf("SoapySDDC::getHardwareKey\n");
    return std::string(RadioHandler.getName());
}

SoapySDR::Kwargs SoapySDDC::getHardwareInfo(void) const
{
    // key/value pairs for any useful information
    // this also gets printed in --probe
    SoapySDR::Kwargs args;

    args["origin"] = "https://github.com/ik1xpv/ExtIO_sddc";
    args["index"] = std::to_string(deviceId);

    DbgPrintf("SoapySDDC::getHardwareInfo\n");
    return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapySDDC::getNumChannels(const int dir) const
{
    DbgPrintf("SoapySDDC::getNumChannels\n");
    return (dir == SOAPY_SDR_RX) ? 1 : 0;
}

bool SoapySDDC::getFullDuplex(const int, const size_t) const
{
    DbgPrintf("SoapySDDC::getFullDuplex\n");
    return false;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapySDDC::listAntennas(const int direction, const size_t) const
{
    DbgPrintf("SoapySDDC::listAntennas : %d\n", direction);
    std::vector<std::string> antennas;
    if (direction == SOAPY_SDR_TX)
    {
        return antennas;
    }

    antennas.push_back("HF");
    antennas.push_back("VHF");
    // i want to list antennas names in dbgprintf
    for (auto &antenna : antennas)
    {
        DbgPrintf("SoapySDDC::listAntennas : %s\n", antenna.c_str());
    }
    return antennas;
}

// set the selected antenna
void SoapySDDC::setAntenna(const int direction, const size_t, const std::string &name)
{
    DbgPrintf("SoapySDDC::setAntenna : %d\n", direction);
    if (direction != SOAPY_SDR_RX)
    {
        return;
    }
    if (name == "HF")
    {
        currentAntenna = "HF";
        RadioHandler.UpdatemodeRF(HFMODE);
    }
    else if (name == "VHF")
    {
        currentAntenna = "VHF";
        RadioHandler.UpdatemodeRF(VHFMODE);
    }
    else
    {
        currentAntenna = "NONE";
        RadioHandler.UpdBiasT_HF(false);
        RadioHandler.UpdBiasT_VHF(false);
    }

    // what antenna is set print in dbgprintf
    DbgPrintf("SoapySDDC::setAntenna : %s\n", name.c_str());
}

// get the selected antenna
std::string SoapySDDC::getAntenna(const int direction, const size_t) const
{
    DbgPrintf("SoapySDDC::getAntenna\n");

    if (RadioHandler.GetmodeRF() == VHFMODE)
    {
        return "VHF";
    }
    else
    {
        return "HF";
    }
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapySDDC::hasDCOffsetMode(const int, const size_t) const
{
    return false;
}

bool SoapySDDC::hasFrequencyCorrection(const int, const size_t) const
{
    return false;
}

void SoapySDDC::setFrequencyCorrection(const int, const size_t, const double)
{
}

double SoapySDDC::getFrequencyCorrection(const int, const size_t) const
{
    return 0.0;
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapySDDC::listGains(const int, const size_t) const
{
    DbgPrintf("SoapySDDC::listGains\n");
    std::vector<std::string> gains;
    gains.push_back("RF");
    gains.push_back("IF");
    return gains;
}

bool SoapySDDC::hasGainMode(const int, const size_t) const
{
    return false;
}

// void SoapySDDC::setGainMode(const int, const size_t, const bool)

// bool SoapySDDC::getGainMode(const int, const size_t) const

// void SoapySDDC::setGain(const int, const size_t, const double)

void SoapySDDC::setGain(const int, const size_t, const std::string &name, const double value)
{
    DbgPrintf("SoapySDDC::setGain %s = %f\n", name.c_str(), value);

    if (name == "RF" && (radioName == "RX888")) {
        int att = 0;
        if (value <= -15.0)
            att = 0; // 0 dB
        else if (value <= -5.0)
            att = 1; // -10 dB
        else
            att = 2; // -20 dB

        RadioHandler.UpdateattRF(att);
        RFGain = value;

        
    }
    else if (name == "IF") {
        const float* gainSteps = nullptr;
    int numSteps = RadioHandler.GetIFGainSteps(&gainSteps);

    if (numSteps > 0 && gainSteps != nullptr) {
        // Find the closest gain step to the desired value
        auto closest = std::min_element(gainSteps, gainSteps + numSteps, 
            [value](float a, float b) {
                return std::abs(a - value) < std::abs(b - value);
            }
        );

        if (closest != gainSteps + numSteps) {
            float selectedGain = *closest;
            // Set the gain using your hardware's method
            RadioHandler.UpdateIFGain(static_cast<int>(selectedGain));
        }
    }

    }
}

double SoapySDDC::getGain(const int direction, const size_t channel, const std::string &name) const
{
    DbgPrintf("SoapySDDC::getGain\n");
    if (name == "RF" && (radioName == "RX888")) {
        return RFGain;
    }
    else if (name == "IF") {
        return IFGain;
    }
    else
        return 0.0;
}

SoapySDR::Range SoapySDDC::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    DbgPrintf("SoapySDDC::getGainRange %s\n", name.c_str());

    if (name == "RF" && (radioName =="RX888") )
    {
        return SoapySDR::Range(
            RFGainMin, RFGainMax, 10);
    }
    else if (name == "IF") {
        return SoapySDR::Range(
            IFGainMin, IFGainMax, IFGainStepsCount);
    }
    else
        return SoapySDR::Range();
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapySDDC::setFrequency(const int, const size_t, const double frequency, const SoapySDR::Kwargs &)
{
    DbgPrintf("SoapySDDC::setFrequency %f\n", frequency);

    double minFreq = 0.0;
    double maxFreq = 0.0;

    // Determine the frequency range based on the current antenna
    if (currentAntenna == "HF")
    {
        minFreq = 0.0;
        maxFreq = adc_frequency / 2.0; // Nyquist limit
    }
    else if (currentAntenna == "VHF")
    {
        minFreq = 24000000.0;   // 24 MHz
        maxFreq = 1800000000.0; // 1.8 GHz
    }
    else
    {
        DbgPrintf("Invalid antenna type: %s\n", currentAntenna.c_str());
        // Default to the widest possible range or handle as needed
        minFreq = 0.0;
        maxFreq = 1800000000.0; // Max frequency supported
    }

    double adjustedFrequency = frequency;

    // Adjust the frequency to be within the allowed range
    if (frequency < minFreq)
    {
        DbgPrintf("Requested frequency %.2f Hz is below the minimum for antenna %s. Adjusting to %.2f Hz\n",
                  frequency, currentAntenna.c_str(), minFreq);
        adjustedFrequency = minFreq;
    }
    else if (frequency > maxFreq)
    {
        DbgPrintf("Requested frequency %.2f Hz is above the maximum for antenna %s. Adjusting to %.2f Hz\n",
                  frequency, currentAntenna.c_str(), maxFreq);
        adjustedFrequency = maxFreq;
    }

    // Set the center frequency
    centerFrequency = RadioHandler.TuneLO(static_cast<uint64_t>(adjustedFrequency));
}

void SoapySDDC::setFrequency(const int, const size_t, const std::string &, const double frequency, const SoapySDR::Kwargs &)
{
    DbgPrintf("SoapySDDC::setFrequency\n");

    centerFrequency = RadioHandler.TuneLO((uint64_t)frequency);
}

double SoapySDDC::getFrequency(const int, const size_t) const
{
    DbgPrintf("SoapySDDC::getFrequency %f\n", (double)centerFrequency);

    return (double)centerFrequency;
}

double SoapySDDC::getFrequency(const int, const size_t, const std::string &name) const
{
    DbgPrintf("SoapySDDC::getFrequency with name %s\n", name.c_str());
    if (sampleRate == 32000000)
    {
        return 8000000.000000;
    }
    return (double)centerFrequency;
}

std::vector<std::string> SoapySDDC::listFrequencies(const int direction, const size_t channel) const
{
    std::vector<std::string> ret;

    if (channel == 0) {
        ret.push_back("RF");
    }

    return ret;
}

SoapySDR::RangeList SoapySDDC::getFrequencyRange(const int direction, const size_t channel, const std::string &name) const
{
    SoapySDR::RangeList ranges;

    ranges.push_back(SoapySDR::Range(10000, 1800000000));

    return ranges;
}

SoapySDR::ArgInfoList SoapySDDC::getFrequencyArgsInfo(const int, const size_t) const
{
    DbgPrintf("SoapySDDC::getFrequencyArgsInfo\n");
    return SoapySDR::ArgInfoList();
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

double SoapySDDC::calculateSampleRate(uint32_t adcFreq, int samplerateidx)
{

    
    int decimate = 5 - samplerateidx;
    

    if (decimate < 0)
    {
        decimate = 0;
        DbgPrintf("WARNING: decimate mismatch at srate_idx = %d\n", samplerateidx);
    }

    // Calculate the output sample rate
    double outputSampleRate = adcFreq / pow(2.0, decimate + 1);

    return outputSampleRate;
}


void SoapySDDC::setSampleRate(const int, const size_t, const double rate)
{
     DbgPrintf("SoapySDDC::setSampleRate %f\n", rate);

    // Validate that the rate is positive
    if (rate <= 0)
    {
        DbgPrintf("Invalid sample rate: %f\n", rate);
        return;
    }

    uint32_t adcFreq = adc_frequency > 0 ? adc_frequency : DEFAULT_ADC_FREQ;

    // Calculate srate_idx
    int srate_idx;
    int decimate;
    double calculatedRate;
    bool found = false;

    // Try possible srate_idx values to find the closest match
    for (srate_idx = MIN_SRATE_IDX; srate_idx <= MAX_SRATE_IDX; srate_idx++)
    {
        calculatedRate = calculateSampleRate(adcFreq, srate_idx);
        if (fabs(calculatedRate - rate) < 1.0) // Allow small difference
        {
            found = true;
            break;
        }
    }

    if (!found)
    {
        DbgPrintf("Requested sample rate %f Hz not supported.\n", rate);
        return;
    }

    // Set samplerateidx and sampleRate
    samplerateidx = srate_idx;
    sampleRate = calculatedRate;

    DbgPrintf("Set sample rate to %f Hz (srate_idx: %d)\n", sampleRate, samplerateidx);
}

double SoapySDDC::getSampleRate(const int, const size_t) const
{
    DbgPrintf("SoapySDDC::getSampleRate %f\n", sampleRate);
    return sampleRate;
}

std::vector<double> SoapySDDC::listSampleRates(const int, const size_t) const
{
    DbgPrintf("SoapySDDC::listSampleRates\n");
    std::vector<double> results;

    // Determine the maximum srate_idx based on adc_frequency
    int maxSrateIdx = (adc_frequency > N2_BANDSWITCH) ? 5 : 4;

    // Generate sample rates for all valid srate_idx values
    for (int srate_idx = 0; srate_idx <= maxSrateIdx; ++srate_idx)
    {
        double sampleRate = this->calculateSampleRate(adc_frequency, srate_idx);
        results.push_back(sampleRate);
    }

    return results;
}

// void SoapySDDC::setMasterClockRate(const double rate)
// {
//     DbgPrintf("SoapySDDC::setMasterClockRate %f\n", rate);
//     masterClockRate = rate;
// }

// double SoapySDDC::getMasterClockRate(void) const
// {
//     DbgPrintf("SoapySDDC::getMasterClockRate %f\n", masterClockRate);
//     return masterClockRate;
// }

// std::vector<std::string> SoapySDDC::listTimeSources(void) const
// {
//     DbgPrintf("SoapySDDC::listTimeSources\n");
//     std::vector<std::string> sources;
//     sources.push_back("sw_ticks");
//     return sources;
// }

// std::string SoapySDDC::getTimeSource(void) const
// {
//     DbgPrintf("SoapySDDC::getTimeSource\n");
//     return "sw_ticks";
// }

// bool SoapySDDC::hasHardwareTime(const std::string &what) const
// {
//     DbgPrintf("SoapySDDC::hasHardwareTime\n");
//     return what == "" || what == "sw_ticks";
// }

// long long SoapySDDC::getHardwareTime(const std::string &what) const
// {
//     DbgPrintf("SoapySDDC::getHardwareTime\n");
//     return SoapySDR::ticksToTimeNs(ticks, sampleRate);
// }

// void SoapySDDC::setHardwareTime(const long long timeNs, const std::string &what)
// {
//     DbgPrintf("SoapySDDC::setHardwareTime\n");
//     ticks = SoapySDR::timeNsToTicks(timeNs, sampleRate);
// }

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapySDDC::getSettingInfo(void) const
{
    DbgPrintf("SoapySDDC::getSettingInfo\n");

    SoapySDR::ArgInfoList setArgs;

    SoapySDR::ArgInfo biasTeeArg;
    biasTeeArg.key = "biastee";
    biasTeeArg.value = "false";
    biasTeeArg.name = "Bias Tee";
    biasTeeArg.description = "Bias-Tee Mode";
    biasTeeArg.type = SoapySDR::ArgInfo::BOOL;

    setArgs.push_back(biasTeeArg);

    SoapySDR::ArgInfo ditheringArg;

    ditheringArg.key = "dithering";
    ditheringArg.value = "true";
    ditheringArg.name = "Dithering";
    ditheringArg.description = "Dithering Mode";
    ditheringArg.type = SoapySDR::ArgInfo::BOOL;

    setArgs.push_back(ditheringArg);

    SoapySDR::ArgInfo randomizationArg;

    randomizationArg.key = "randomization";
    randomizationArg.value = "true";
    randomizationArg.name = "Randomization";
    randomizationArg.description = "Randomization Mode";
    randomizationArg.type = SoapySDR::ArgInfo::BOOL;

    setArgs.push_back(randomizationArg);

    SoapySDR_logf(SOAPY_SDR_DEBUG, "SETARGS?");

    return setArgs;
}

void SoapySDDC::writeSetting(const std::string &key, const std::string &value)
{
    DbgPrintf("SoapySDDC::writeSetting\n");

    if (key == "biastee")
    {
        biasTee = (value == "true") ? true: false;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "bias tee mode: %s", biasTee ? "true" : "false");
        RadioHandler.UpdBiasT_HF(biasTee ? 1 : 0);
    }
    else if (key == "dithering")
    {
        dithering = (value == "true") ? true : false;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "dithering mode: %s", dithering ? "true" : "false");
        RadioHandler.UptDither(dithering ? 1 : 0);
    }
    else if (key == "randomization")
    {
        randomization = (value == "true") ? true : false;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "randomization mode: %s", randomization ? "true" : "false");
        RadioHandler.UptRand(randomization ? 1 : 0);
    }
}

std::string SoapySDDC::readSetting(const std::string &key) const
{
    DbgPrintf("SoapySDDC::readSetting\n");

    if (key == "biastee") {
        return biasTee?"true":"false";
    } else if (key == "dithering") {
        return dithering?"true":"false";
    } else if (key == "randomization") {
        return randomization?"true":"false";
    }

    SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
    return "";
}

