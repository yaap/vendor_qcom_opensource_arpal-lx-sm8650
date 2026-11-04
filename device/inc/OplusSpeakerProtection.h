/*
 * Copyright (c) 2026 The YAAP Project
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OPLUS_SPEAKER_PROTECTION_H
#define OPLUS_SPEAKER_PROTECTION_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "OplusSpeakerDevice.h"
#include "Speaker.h"

struct str_parms;

class OplusSpeakerProtection : public Speaker {
public:
    OplusSpeakerProtection(struct pal_device *device,
                           std::shared_ptr<ResourceManager> Rm);
    ~OplusSpeakerProtection() override;

    int start() override;
    int stop() override;
    int close() override;

    int32_t setParameter(uint32_t param_id, void *param) override;
    int32_t getParameter(uint32_t param_id, void **param) override;

    int spkrProtProcessingMode(bool enable);
    void updateSPcustomPayload();
    int checkSpeakerStatus(uint32_t param1, uint32_t param2);
    void monitorThreadLoop();
    void createMonitorThread();
    void destroyMonitorThread();
    bool isSpeakerInUse();
    void setSpeakerParam(int param, int value);
    void updateDynamicEQ();
    void setSpeakerRotation(int rotation, bool force);
    void setSpeakerVolMute(bool mute);
    int setSpeakerCKV(int ckv);
    int populateSpeakerCKV(Stream *s,
                           std::vector<std::pair<int, int>> &ckv) override;
    void mute();
    void unmute();
    int setConfigParams(struct str_parms *parms, char *value, int len);

private:
    int getSpeakerScence(Stream *s);
    int getCalibrationData(void **param);
    int getFTMParameter(void **param);
    void loadGainMapping();
    int setMixerInt(const char *name, int value);
    int setMixerEnum(const char *name, const char *value);

    std::shared_ptr<OplusSpeakerDevice> mSpkDev;
    std::mutex mProtectionMutex;
    std::thread mMonitorThread;
    std::atomic<bool> mThreadExit{false};
    std::mutex mMonitorMutex;
    std::condition_variable mMonitorCv;

    bool mStarted = false;
    bool mVoipCKVSupport = true;
    bool mRingtoneCKVSupport = true;
    bool mLLCKVSupport = false;
    bool mChkSpkEnable = true;
    bool mTuningEnable = false;
    bool mMuteEnable = false;
    bool mGainAdjustEnabled = true;
    bool mDiffLatencyEnabled = false;
    bool mLLEffectEnable = false;

    int mSpkScene = 0;
    int mSpeakerType = OPLUS_SPEAKER_PA_TFA98XX;
    int mSpeakerId = 0;
    int mSpkCount = 2;
    int mPreGainVal = -1;
    int mBatCapacity = 100;
    int mRotationCal = 0;
    int mDiffLatencyCal = 0;
    int mSpAlgoDelay = 0;

    pal_speaker_rotation_type mRotation = PAL_SPEAKER_ROTATION_LR;
    std::vector<std::pair<int, int>> mBatToVol;
};

#endif  // OPLUS_SPEAKER_PROTECTION_H
