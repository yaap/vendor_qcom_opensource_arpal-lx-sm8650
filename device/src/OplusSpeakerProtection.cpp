/*
 * Copyright (c) 2026 The YAAP Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "PAL: OplusSpeakerProtection"

#include "OplusSpeakerProtection.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

#include <cutils/str_parms.h>
#include <tinyalsa/asoundlib.h>

#include "PalAudioRoute.h"
#include "PalCommon.h"
#include "ResourceManager.h"
#include "Session.h"
#include "Stream.h"
#include "kvh2xml.h"

#define TFA_MIXER_CHIP_SELECTOR "TFA_CHIP_SELECTOR"
#define TFA_MIXER_PROFILE "TFA Profile"

namespace {

static int detectSpeakerPaType(std::shared_ptr<ResourceManager> rm)
{
    struct mixer *mixer = nullptr;

    if (!rm || rm->getHwAudioMixer(&mixer) || !mixer)
        return OPLUS_SPEAKER_PA_UNKNOWN;

    if (mixer_get_ctl_by_name(mixer, "TFA_CHIP_SELECTOR"))
        return OPLUS_SPEAKER_PA_TFA98XX;

    if (mixer_get_ctl_by_name(mixer, "SpkrLeft Sipa Type"))
        return OPLUS_SPEAKER_PA_SIA;

    return OPLUS_SPEAKER_PA_UNKNOWN;
}

}  // namespace

OplusSpeakerProtection::OplusSpeakerProtection(
    struct pal_device *device, std::shared_ptr<ResourceManager> Rm)
    : Speaker(device, Rm)
{
    rm = Rm;
    mSpeakerType = detectSpeakerPaType(Rm);
    if (mSpeakerType == OPLUS_SPEAKER_PA_UNKNOWN)
        PAL_DBG(LOG_TAG, "unknown speaker PA type");
    mSpkDev = OplusSpeakerDevice::getInstance(mSpeakerType);
    if (mSpkDev)
        mSpkDev->init();

    mGainAdjustEnabled = ResourceManager::isLowBatterySpeakerProfileEnabled;
    loadGainMapping();
    if (mSpkDev) {
        mSpkDev->setGainAdjustEnable(mGainAdjustEnabled);
        mSpkDev->setGainMapping(mBatToVol);
    }
    mSpkCount = mSpkDev ? mSpkDev->getDeviceCount() : 2;

    PAL_INFO(LOG_TAG, "speaker protection initialized: type=%d speakers=%d",
             mSpeakerType, mSpkCount);
    PAL_INFO(LOG_TAG,
             "config: scene=%d id=%d rotation=%d rotcal=%d diff=%d delay=%d "
             "bat=%d gain_en=%d tuning=%d lleffect=%d lldiff=%d voip=%d "
             "ring=%d llckv=%d chk=%d batmap=%zu",
             mSpkScene, mSpeakerId, (int)mRotation, mRotationCal,
             mDiffLatencyCal, mSpAlgoDelay, mBatCapacity, mGainAdjustEnabled,
             mTuningEnable, mLLEffectEnable, mDiffLatencyEnabled,
             mVoipCKVSupport, mRingtoneCKVSupport, mLLCKVSupport,
             mChkSpkEnable, mBatToVol.size());
}

OplusSpeakerProtection::~OplusSpeakerProtection()
{
    destroyMonitorThread();
    if (mSpkDev)
        mSpkDev->stop();
}

void OplusSpeakerProtection::loadGainMapping()
{
    /* Defaults match the stock waffle resourcemanager.xml. */
    mBatToVol = {{20, 2}, {100, 0}};

    std::ifstream file("/odm/etc/resourcemanager.xml");
    if (!file.is_open()) {
        PAL_DBG(LOG_TAG, "resourcemanager.xml not available");
        return;
    }

    std::vector<std::pair<int, int>> mapping;
    std::string line;
    bool in_mapping = false;

    while (std::getline(file, line)) {
        if (line.find("<smartpa_gain_mapping>") != std::string::npos) {
            in_mapping = true;
            continue;
        }
        if (line.find("</smartpa_gain_mapping>") != std::string::npos)
            break;
        if (!in_mapping || line.find("bat_to_dblvl_map") == std::string::npos)
            continue;

        size_t battery_pos = line.find("battery=\"");
        size_t level_pos = line.find("level=\"");
        if (battery_pos == std::string::npos ||
            level_pos == std::string::npos)
            continue;

        int battery = atoi(line.c_str() + battery_pos +
                           strlen("battery=\""));
        int level = atoi(line.c_str() + level_pos + strlen("level=\""));

        if (battery < 0 || battery > 100 || level < 0 || level > 15) {
            PAL_ERR(LOG_TAG, "invalid gain mapping battery=%d level=%d",
                    battery, level);
            continue;
        }
        mapping.push_back(std::make_pair(battery, level));
    }

    if (mapping.empty()) {
        PAL_DBG(LOG_TAG, "using default gain mapping");
        return;
    }

    std::sort(mapping.begin(), mapping.end());
    mBatToVol = mapping;
    PAL_INFO(LOG_TAG, "gain mapping entries=%zu", mBatToVol.size());
}

int OplusSpeakerProtection::setMixerInt(const char *name, int value)
{
    struct mixer *mixer = nullptr;

    if (!rm || rm->getHwAudioMixer(&mixer) || !mixer)
        return -EINVAL;

    struct mixer_ctl *ctl = mixer_get_ctl_by_name(mixer, name);
    if (!ctl) {
        PAL_DBG(LOG_TAG, "mixer control %s is not available", name);
        return -ENOENT;
    }

    return mixer_ctl_set_value(ctl, 0, value);
}

int OplusSpeakerProtection::setMixerEnum(const char *name, const char *value)
{
    struct mixer *mixer = nullptr;

    if (!rm || rm->getHwAudioMixer(&mixer) || !mixer)
        return -EINVAL;

    struct mixer_ctl *ctl = mixer_get_ctl_by_name(mixer, name);
    if (!ctl) {
        PAL_DBG(LOG_TAG, "mixer control %s is not available", name);
        return -ENOENT;
    }

    return mixer_ctl_set_enum_by_string(ctl, value);
}

int OplusSpeakerProtection::start()
{
    int ret;
    std::lock_guard<std::mutex> lock(mProtectionMutex);
    bool firstStart = (deviceStartStopCount == 0);

    PAL_DBG(LOG_TAG, "start first=%d deviceCount=%d startStopCount=%d",
            firstStart, getDeviceCount(), deviceStartStopCount);

    if (firstStart) {
        if (mSpkDev) {
            mSpkDev->start();
            mSpkDev->lowBatAdjustGainInit();
        }

        /*
         * Calibration must never run while the audio path is live.  Use the
         * R0 values already programmed in the TFA MTP or the driver default
         * impedance; if neither exists, leave the OPlus SP payload disabled
         * and continue through the normal speaker path.  SIA uses its factory
         * calibration and does not support this fallback.
         */
        if (mSpeakerType == OPLUS_SPEAKER_PA_TFA98XX && mSpkDev &&
            !mSpkDev->isCalibrated()) {
            int default_ret = mSpkDev->loadDefaultCalibration();

            if (default_ret)
                PAL_INFO(LOG_TAG,
                         "no calibration/default R0; skipping SP payload");
        }

        /*
         * Device::start() programs customPayload into the backend.  Build it
         * first so this start uses the current session's SP MIID.  Building it
         * after Device::start() would leave the amplifier unprotected until
         * the next start and could program a stale MIID from a previous
         * session.
         */
        updateSPcustomPayload();

        ret = spkrProtProcessingMode(true);
        if (ret)
            PAL_ERR(LOG_TAG, "speaker protection start failed %d", ret);
    }

    ret = Device::start();
    if (ret) {
        PAL_ERR(LOG_TAG, "device start failed %d", ret);
        return ret;
    }

    if (firstStart) {
        mMuteEnable = false;
        mStarted = true;
        createMonitorThread();
    }

    return 0;
}

int OplusSpeakerProtection::stop()
{
    std::lock_guard<std::mutex> lock(mProtectionMutex);
    bool lastStop = (deviceStartStopCount <= 1);

    PAL_DBG(LOG_TAG, "stop last=%d deviceCount=%d startStopCount=%d",
            lastStop, getDeviceCount(), deviceStartStopCount);

    /*
     * A speaker device can be shared by concurrent streams.  Do not mute or
     * tear down the amplifier until the last started stream goes away.
     */
    if (lastStop) {
        destroyMonitorThread();
        spkrProtProcessingMode(false);
        if (mSpkDev)
            mSpkDev->stop();

        mStarted = false;
    }

    return Device::stop();
}

int OplusSpeakerProtection::close()
{
    std::lock_guard<std::mutex> lock(mProtectionMutex);
    int users = getDeviceCount();
    bool lastClose = (mStarted && users <= 1);

    PAL_DBG(LOG_TAG, "close last=%d deviceCount=%d startStopCount=%d",
            lastClose, users, deviceStartStopCount);

    if (lastClose) {
        destroyMonitorThread();
        spkrProtProcessingMode(false);
        if (mSpkDev)
            mSpkDev->stop();

        mStarted = false;
    }

    return Speaker::close();
}

int OplusSpeakerProtection::spkrProtProcessingMode(bool enable)
{
    int ret = 0;

    if (!mSpkDev)
        return -EINVAL;

    if (enable) {
        if (mSpeakerType == OPLUS_SPEAKER_PA_TFA98XX) {
            ret = setMixerInt(TFA_MIXER_CHIP_SELECTOR, 3);
            if (ret && ret != -ENOENT)
                PAL_ERR(LOG_TAG, "failed to select TFA chip: %d", ret);
            setMixerEnum(TFA_MIXER_PROFILE, "speaker");
        }
        mSpkDev->deviceMute(false);
    } else {
        mSpkDev->deviceMute(true);
    }

    return 0;
}

void OplusSpeakerProtection::updateSPcustomPayload()
{
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    std::shared_ptr<Device> dev;
    std::vector<Stream *> activeStreams;
    Stream *stream = nullptr;
    Session *session = nullptr;
    std::string backendName;
    uint8_t *payload = nullptr;
    size_t payloadSize = 0;
    uint32_t miid = 0;

    if (!mSpkDev || !rm)
        return;

    if (customPayload) {
        free(customPayload);
        customPayload = nullptr;
        customPayloadSize = 0;
    }

    dev = Device::getObject(PAL_DEVICE_OUT_SPEAKER);
    if (!dev)
        dev = Device::getInstance(&deviceAttr, rm);

    if (!dev) {
        PAL_ERR(LOG_TAG, "no speaker device instance");
        return;
    }

    if (rm->getActiveStream_l(activeStreams, dev) || activeStreams.empty()) {
        PAL_DBG(LOG_TAG, "no active speaker stream for SP payload");
        return;
    }

    stream = activeStreams[0];
    if (stream->getAssociatedSession(&session) || !session) {
        PAL_ERR(LOG_TAG, "failed to get speaker session");
        return;
    }

    rm->getBackendName(PAL_DEVICE_OUT_SPEAKER, backendName);
    if (session->getMIID(backendName.c_str(), MODULE_SP, &miid)) {
        PAL_ERR(LOG_TAG, "failed to get MODULE_SP miid");
        return;
    }

    mSpkDev->payloadSPConfig(&payload, &payloadSize, miid);
    if (payload && payloadSize) {
        updateCustomPayload(payload, payloadSize);
        free(payload);
    } else {
        PAL_INFO(LOG_TAG, "empty SP payload, skipping");
        return;
    }

    PAL_DBG(LOG_TAG, "updated speaker protection payload, miid=%u size=%zu",
            miid, customPayloadSize);
}

void OplusSpeakerProtection::createMonitorThread()
{
    if (mMonitorThread.joinable())
        return;

    if (!mSpkDev || !mSpkDev->isMonitorEn())
        return;

    mThreadExit = false;
    mMonitorThread = std::thread(&OplusSpeakerProtection::monitorThreadLoop,
                                 this);
}

void OplusSpeakerProtection::destroyMonitorThread()
{
    if (!mMonitorThread.joinable())
        return;

    mThreadExit = true;
    mMonitorCv.notify_all();
    mMonitorThread.join();
}

void OplusSpeakerProtection::monitorThreadLoop()
{
    PAL_DBG(LOG_TAG, "monitor thread started");

    while (!mThreadExit) {
        std::unique_lock<std::mutex> lock(mMonitorMutex);
        mMonitorCv.wait_for(lock, std::chrono::seconds(5),
                            [this] { return mThreadExit.load(); });
        lock.unlock();

        if (mThreadExit)
            break;

        if (checkSpeakerStatus(0, 0) < 0) {
            /*
             * Never mute from this opportunistic monitor.  The TFA status
             * interface is not reliable enough to override the active audio
             * path; the ADSP/TFA protection remains responsible for real
             * faults.
             */
            PAL_ERR(LOG_TAG,
                    "speaker protection status check failed; keeping audio unmuted");
        }
    }

    PAL_DBG(LOG_TAG, "monitor thread stopped");
}

int OplusSpeakerProtection::checkSpeakerStatus(uint32_t param1,
                                                uint32_t param2)
{
    if (!mSpkDev)
        return 0;
    return mSpkDev->checkSpeakerStatus(nullptr, param1, param2);
}

bool OplusSpeakerProtection::isSpeakerInUse()
{
    return mStarted || deviceCount > 0;
}

void OplusSpeakerProtection::setSpeakerParam(int param, int value)
{
    PAL_DBG(LOG_TAG, "set speaker param %d = %d", param, value);

    if (!mSpkDev)
        return;

    if (mSpkDev->isGainChanged(value)) {
        mSpkDev->recordActualGain(value);
        mPreGainVal = value;
    }
}

void OplusSpeakerProtection::updateDynamicEQ()
{
    PAL_DBG(LOG_TAG, "update dynamic EQ");
}

void OplusSpeakerProtection::setSpeakerRotation(int rotation, bool force)
{
    PAL_DBG(LOG_TAG, "set speaker rotation %d force %d", rotation, force);
    mRotation = (pal_speaker_rotation_type)rotation;
}

void OplusSpeakerProtection::setSpeakerVolMute(bool mute)
{
    if (mMuteEnable == mute)
        return;

    mMuteEnable = mute;
    if (mSpkDev)
        mSpkDev->deviceMute(mute);
}

void OplusSpeakerProtection::mute()
{
    setSpeakerVolMute(true);
}

void OplusSpeakerProtection::unmute()
{
    setSpeakerVolMute(false);
}

int OplusSpeakerProtection::setSpeakerCKV(int ckv)
{
    PAL_DBG(LOG_TAG, "set speaker CKV %d", ckv);
    return 0;
}

int OplusSpeakerProtection::getCalibrationData(void **param)
{
    char result[512] = {0};
    int ret;

    if (!mSpkDev)
        return -EINVAL;

    ret = mSpkDev->getCalibrationResult(result, sizeof(result));
    if (ret < 0)
        return ret;

    size_t len = strlen(result);
    memcpy((char *)param, result, len);
    return len;
}

int OplusSpeakerProtection::getFTMParameter(void **param)
{
    char result[512] = {0};
    int ret;

    if (!mSpkDev)
        return -EINVAL;

    ret = mSpkDev->getCalibrationRange(result, sizeof(result));
    if (ret < 0)
        return ret;

    size_t len = strlen(result);
    memcpy((char *)param, result, len);
    return len;
}

int32_t OplusSpeakerProtection::setParameter(uint32_t param_id, void *param)
{
    (void)param;

    PAL_DBG(LOG_TAG, "set parameter %u", param_id);

    return 0;
}

int32_t OplusSpeakerProtection::getParameter(uint32_t param_id, void **param)
{
    PAL_DBG(LOG_TAG, "get parameter %u", param_id);

    switch (param_id) {
        case PAL_PARAM_ID_SP_GET_CAL:
            return getCalibrationData(param);
        case PAL_PARAM_ID_SP_MODE:
            return getFTMParameter(param);
        default:
            PAL_ERR(LOG_TAG, "unsupported parameter %u", param_id);
            return -EINVAL;
    }
}

int OplusSpeakerProtection::setConfigParams(struct str_parms *parms,
                                             char *value, int len)
{
    if (!parms || !value || len <= 0)
        return -EINVAL;

    if (str_parms_get_str(parms, "oplus_check_speaker", value, len) >= 0)
        mChkSpkEnable = !strncmp(value, "true", sizeof("true") - 1);

    if (str_parms_get_str(parms, "speaker_voip_ckv", value, len) >= 0)
        mVoipCKVSupport = !strncmp(value, "true", sizeof("true") - 1);

    return 0;
}
