/*
 * Copyright (c) 2026 The YAAP Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "PAL: OplusSpeakerSia"

#include "OplusSpeakerSia.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <tinyalsa/asoundlib.h>

#include "PalAudioRoute.h"
#include "PalCommon.h"
#include "ResourceManager.h"

#define SIA_MIXER_MIN_MOHMS 5000
#define SIA_MIXER_MAX_MOHMS 8000
#define SIA_MIXER_DEFAULT_MOHMS 6000
#define SIA_MIXER_MIN_F0 400
#define SIA_MIXER_MAX_F0 850

#define SIA_CAL_CH_STRIDE 0x14
#define SIA_CAL_F0_STRIDE 0x10
#define SIA_CAL_TOTAL_STRIDE 0x24

#define SIA_SP_PARAM_ID 0x1800ea03
#define SIA_SP_PARAM_ID_CH23 0x1800ea0a

namespace {

/* Speaker protection SET_CAL command carried in every SP payload */
static constexpr size_t kSpPayloadSize = 0x40;
static constexpr uint8_t kSpChannelSelect = 0x49;
static constexpr uint8_t kSpChannelCmd = 0x3c;
static constexpr uint32_t kSpOpSetCal = 4;

static void putLe32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xff);
    dst[1] = (uint8_t)((value >> 8) & 0xff);
    dst[2] = (uint8_t)((value >> 16) & 0xff);
    dst[3] = (uint8_t)((value >> 24) & 0xff);
}

static int channelIndex(uint32_t ch, uint32_t *param_id)
{
    if (ch == 2 || ch == 3) {
        *param_id = SIA_SP_PARAM_ID_CH23;
        return (int)(ch - 2);
    }

    *param_id = SIA_SP_PARAM_ID;
    return (int)ch;
}

static void appendParam(std::vector<uint8_t> &out, uint32_t miid,
                        uint32_t param_id, const uint8_t *data, size_t len)
{
    size_t aligned = (len + 3) & ~(size_t)3;
    size_t offset = out.size();

    out.resize(offset + 16 + aligned, 0);

    putLe32(out.data() + offset, miid);
    putLe32(out.data() + offset + 4, param_id);
    putLe32(out.data() + offset + 8, (uint32_t)aligned);
    putLe32(out.data() + offset + 12, 0);

    if (len)
        memcpy(out.data() + offset + 16, data, len);
}

}  // namespace

std::shared_ptr<OplusSpeakerDevice> OplusSpeakerSia::obj = nullptr;

OplusSpeakerSia::OplusSpeakerSia()
{
}

OplusSpeakerSia::~OplusSpeakerSia()
{
}

std::shared_ptr<OplusSpeakerDevice> OplusSpeakerSia::getInstance()
{
    if (!obj)
        obj = std::shared_ptr<OplusSpeakerDevice>(new OplusSpeakerSia());
    return obj;
}

std::shared_ptr<OplusSpeakerDevice> OplusSpeakerSia::getObject()
{
    return obj;
}

int OplusSpeakerSia::readMixerInt(const std::string &name, int *value)
{
    if (!value)
        return -EINVAL;

    struct mixer *mixer = mHwMixer;
    if (!mixer) {
        std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
        if (rm)
            rm->getHwAudioMixer(&mixer);
        mHwMixer = mixer;
    }
    if (!mixer)
        return -EINVAL;

    struct mixer_ctl *ctl = mixer_get_ctl_by_name(mixer, name.c_str());
    if (!ctl) {
        PAL_DBG(LOG_TAG, "mixer control %s is not available", name.c_str());
        return -ENOENT;
    }

    *value = mixer_ctl_get_value(ctl, 0);
    return 0;
}

std::string OplusSpeakerSia::speakerName(int ch) const
{
    if (ch >= 0 && ch < (int)mSpkNames.size())
        return mSpkNames[ch];
    return "SpkrLeft";
}

int OplusSpeakerSia::calibrationInfoInit()
{
    mCali.assign(mSpkCount, SipaCaliInfo());

    for (int ch = 0; ch < mSpkCount; ch++) {
        SipaCaliInfo &info = mCali[ch];
        std::string base = speakerName(ch);
        int value = 0;

        if (readMixerInt(base + " Sipa Type", &value)) {
            if (ch == 0) {
                PAL_ERR(LOG_TAG, "missing %s Sipa Type mixer control",
                        base.c_str());
                return -ENOENT;
            }
            PAL_DBG(LOG_TAG, "missing %s Sipa Type mixer control",
                    base.c_str());
        }

        if (!readMixerInt(base + " Sipa min mohms", &value))
            info.min_mohms = value;
        else
            info.min_mohms = SIA_MIXER_MIN_MOHMS;

        if (!readMixerInt(base + " Sipa max mohms", &value))
            info.max_mohms = value;
        else
            info.max_mohms = SIA_MIXER_MAX_MOHMS;

        if (!readMixerInt(base + " Sipa default mohms", &value))
            info.default_mohms = value;
        else
            info.default_mohms = SIA_MIXER_DEFAULT_MOHMS;

        if (!readMixerInt(base + " Sipa Freq Support", &value))
            info.f0_support = value;

        if (!readMixerInt(base + " Sipa min freq", &value))
            info.min_f0 = value;
        else
            info.min_f0 = SIA_MIXER_MIN_F0;

        if (!readMixerInt(base + " Sipa max freq", &value))
            info.max_f0 = value;
        else
            info.max_f0 = SIA_MIXER_MAX_F0;

        PAL_INFO(LOG_TAG,
                 "%s: min=%d mohms max=%d mohms default=%d mohms "
                 "f0_support=%d f0=%d-%d Hz",
                 base.c_str(), info.min_mohms, info.max_mohms,
                 info.default_mohms, info.f0_support, info.min_f0,
                 info.max_f0);
    }

    return 0;
}

int OplusSpeakerSia::readCalibrationFile()
{
    size_t expected = (size_t)mSpkCount * SIA_CAL_TOTAL_STRIDE;
    std::vector<uint8_t> buf(expected);
    ssize_t total = 0;
    int fd;

    if (expected == 0)
        return -ENODATA;

    fd = open(mCalFile.c_str(), O_RDONLY);
    if (fd < 0) {
        PAL_DBG(LOG_TAG, "cannot open %s: %s", mCalFile.c_str(),
                strerror(errno));
        return -ENOENT;
    }

    while (total < (ssize_t)expected) {
        ssize_t ret = read(fd, buf.data() + total, expected - total);
        if (ret < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        if (ret == 0)
            break;
        total += ret;
    }
    close(fd);

    if (total < (ssize_t)expected) {
        PAL_DBG(LOG_TAG, "short calibration file %s (%zd/%zu bytes)",
                mCalFile.c_str(), total, expected);
        return -ENODATA;
    }

    for (int ch = 0; ch < mSpkCount; ch++) {
        int32_t values[5] = {0};
        int32_t f0 = 0;

        memcpy(values, buf.data() + ch * SIA_CAL_CH_STRIDE, sizeof(values));
        memcpy(&f0,
               buf.data() + mSpkCount * SIA_CAL_CH_STRIDE +
                   ch * SIA_CAL_F0_STRIDE,
               sizeof(f0));

        mCali[ch].cal_ok = values[0];
        mCali[ch].r0 = values[1];
        mCali[ch].t0 = values[2];
        mCali[ch].wire_r0 = values[3];
        mCali[ch].a = values[4];
        mCali[ch].f0 = f0;
    }

    PAL_INFO(LOG_TAG, "loaded calibration from %s", mCalFile.c_str());
    return 0;
}

void OplusSpeakerSia::updateCalibrationValue()
{
    mCalibrated = true;

    for (int ch = 0; ch < mSpkCount && ch < (int)mCali.size(); ch++) {
        SipaCaliInfo &info = mCali[ch];
        int32_t default_r0;

        if (info.r0 == 0)
            mCalibrated = false;

        if (info.r0 > info.min_mohms && info.r0 < info.max_mohms)
            continue;

        default_r0 = info.default_mohms;
        if (default_r0 == 0) {
            if (info.min_mohms < 1 || info.max_mohms < 1)
                default_r0 = 0;
            else
                default_r0 = (info.min_mohms + info.max_mohms) / 2;
        }

        info.r0 = default_r0;
        info.f0 = -1;
        info.t0 = -1;
        info.wire_r0 = -1;
    }

    PAL_DBG(LOG_TAG, "calibrated=%d", mCalibrated);
}

int OplusSpeakerSia::init()
{
    struct pal_device_info devinfo = {};
    std::shared_ptr<ResourceManager> rm;

    std::lock_guard<std::mutex> lock(mMutex);

    if (mInitialized)
        return 0;

    rm = ResourceManager::getInstance();
    if (!rm) {
        PAL_ERR(LOG_TAG, "resource manager is not available");
        return -EINVAL;
    }

    if (rm->getHwAudioMixer(&mHwMixer) || !mHwMixer) {
        PAL_ERR(LOG_TAG, "hardware mixer is not available");
        return -EINVAL;
    }

    rm->getDeviceInfo(PAL_DEVICE_OUT_SPEAKER, PAL_STREAM_LOW_LATENCY, "",
                      &devinfo);
    mSpkCount = devinfo.channels > 0 ? devinfo.channels : 2;
    if (mSpkCount < 1)
        mSpkCount = 1;
    if (mSpkCount > 6)
        mSpkCount = 6;

    mSpkNames.clear();
    if (mSpkCount == 2) {
        mSpkNames.push_back("SpkrLeft");
        mSpkNames.push_back("SpkrRight");
    } else {
        static const char *names[] = {"SpkrOne", "SpkrTwo", "SpkrThree",
                                      "SpkrFour", "SpkrFive", "SpkrSix"};
        for (int ch = 0; ch < mSpkCount; ch++)
            mSpkNames.push_back(names[ch]);
    }

    if (calibrationInfoInit()) {
        PAL_ERR(LOG_TAG, "failed to read SIA mixer configuration");
        return -EINVAL;
    }

    readCalibrationFile();
    updateCalibrationValue();

    mInitialized = true;
    PAL_INFO(LOG_TAG, "initialized: speakers=%d calibrated=%d", mSpkCount,
             mCalibrated);
    return 0;
}

bool OplusSpeakerSia::isCalibrated()
{
    if (!mInitialized)
        init();
    return mCalibrated;
}

int OplusSpeakerSia::payloadSPConfig(uint8_t **payload, size_t *size,
                                    uint32_t miid)
{
    std::vector<uint8_t> out;
    uint8_t *buf;

    if (!payload || !size)
        return -EINVAL;

    *payload = nullptr;
    *size = 0;

    if (!mInitialized && init())
        return -EINVAL;

    for (int ch = 0; ch < mSpkCount && ch < (int)mCali.size(); ch++) {
        const SipaCaliInfo &info = mCali[ch];
        uint8_t param[kSpPayloadSize] = {0};
        uint32_t param_id = SIA_SP_PARAM_ID;
        int channel = channelIndex((uint32_t)ch, &param_id);

        param[0] = (uint8_t)channel;
        param[1] = kSpChannelSelect;
        param[2] = kSpChannelCmd;
        param[3] = 0;
        putLe32(param + 4, kSpOpSetCal);
        putLe32(param + 8, (uint32_t)info.r0);
        putLe32(param + 12, (uint32_t)info.t0);
        putLe32(param + 16, (uint32_t)info.wire_r0);
        putLe32(param + 20, (uint32_t)info.a);

        appendParam(out, miid, param_id, param, sizeof(param));
    }

    if (out.empty())
        return 0;

    buf = (uint8_t *)calloc(1, out.size());
    if (!buf)
        return -ENOMEM;

    memcpy(buf, out.data(), out.size());
    *payload = buf;
    *size = out.size();
    return 0;
}

int OplusSpeakerSia::calibrate()
{
    return -EOPNOTSUPP;
}

int OplusSpeakerSia::calibrationAndGetResult(char *result, size_t len,
                                             bool is_ftm)
{
    (void)result;
    (void)len;
    (void)is_ftm;
    return -EOPNOTSUPP;
}

int OplusSpeakerSia::getDeviceCount()
{
    return mSpkCount;
}

int OplusSpeakerSia::getCalibrationRange(char *range, size_t len)
{
    if (!range || !len)
        return -EINVAL;

    range[0] = '\0';

    for (int ch = 0; ch < mSpkCount && ch < (int)mCali.size(); ch++) {
        char tmp[64];

        snprintf(tmp, sizeof(tmp), "%.3f,%.3f",
                 mCali[ch].min_mohms / 1000.0,
                 mCali[ch].max_mohms / 1000.0);

        if (ch)
            strlcat(range, ",", len);
        strlcat(range, tmp, len);
    }

    return 0;
}

int OplusSpeakerSia::getCalibrationResult(char *result, size_t len)
{
    if (!result || !len)
        return -EINVAL;

    snprintf(result, len, "SpkrCalStatus: %d;", mCalibrated ? 0 : -1);
    return 0;
}

int OplusSpeakerSia::getImpedance(char *impedance, size_t len)
{
    if (!impedance || !len)
        return -EINVAL;

    impedance[0] = '\0';

    for (int ch = 0; ch < mSpkCount && ch < (int)mCali.size(); ch++) {
        char tmp[32];

        snprintf(tmp, sizeof(tmp), "%.3f", mCali[ch].r0 / 1000.0);

        if (ch)
            strlcat(impedance, ",", len);
        strlcat(impedance, tmp, len);
    }

    return 0;
}

int OplusSpeakerSia::deviceMute(bool mute)
{
    (void)mute;
    return 0;
}
