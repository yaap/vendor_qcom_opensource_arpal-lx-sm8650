/*
 * Copyright (c) 2026 The YAAP Project
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OPLUS_SPEAKER_SIA_H
#define OPLUS_SPEAKER_SIA_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "OplusSpeakerDevice.h"

struct mixer;
struct str_parms;

class OplusSpeakerSia : public OplusSpeakerDevice {
public:
    static std::shared_ptr<OplusSpeakerDevice> getInstance();
    static std::shared_ptr<OplusSpeakerDevice> getObject();

    ~OplusSpeakerSia() override;

    int init() override;
    int payloadSPConfig(uint8_t **payload, size_t *size,
                        uint32_t miid) override;
    int getDeviceCount() override;
    int getCalibrationRange(char *range, size_t len) override;
    int getImpedance(char *impedance, size_t len) override;
    int getCalibrationResult(char *result, size_t len) override;
    int calibrationAndGetResult(char *result, size_t len,
                                bool is_ftm) override;

    int deviceMute(bool mute) override;
    bool isCalibrated() override;
    int calibrate() override;

private:
    OplusSpeakerSia();

    struct SipaCaliInfo {
        int32_t min_mohms = 5000;
        int32_t max_mohms = 8000;
        int32_t default_mohms = 6000;
        int32_t f0_support = 0;
        int32_t min_f0 = 400;
        int32_t max_f0 = 850;
        int32_t f0 = 0;
        int32_t reserved = 0;
        int32_t cal_ok = 0;
        int32_t r0 = 0;
        int32_t t0 = 0;
        int32_t wire_r0 = 0;
        int32_t a = 0;
    };

    int calibrationInfoInit();
    int readCalibrationFile();
    void updateCalibrationValue();
    int readMixerInt(const std::string &name, int *value);
    std::string speakerName(int ch) const;

    static std::shared_ptr<OplusSpeakerDevice> obj;

    std::mutex mMutex;
    struct mixer *mHwMixer = nullptr;
    std::vector<SipaCaliInfo> mCali;
    std::vector<std::string> mSpkNames;
    std::string mCalFile = "/mnt/vendor/persist/media/sipa_w.bin";
    int mSpkCount = 0;
    bool mInitialized = false;
    bool mCalibrated = false;
};

#endif  // OPLUS_SPEAKER_SIA_H
