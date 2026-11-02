/*
 * Copyright (c) 2026 The YAAP Project
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OPLUS_SPEAKER_DEVICE_H
#define OPLUS_SPEAKER_DEVICE_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

class Stream;

enum oplus_speaker_pa_type {
    OPLUS_SPEAKER_PA_UNKNOWN = 0,
    OPLUS_SPEAKER_PA_TFA98XX,
    OPLUS_SPEAKER_PA_SIA,
};

class OplusSpeakerDevice {
public:
    virtual ~OplusSpeakerDevice() = default;

    static std::shared_ptr<OplusSpeakerDevice> getInstance(int type);
    static std::shared_ptr<OplusSpeakerDevice> getObject(int type);

    virtual int init() = 0;
    virtual int payloadSPConfig(uint8_t **payload, size_t *size,
                                uint32_t miid) = 0;
    virtual int getDeviceCount() = 0;
    virtual int getCalibrationRange(char *range, size_t len) = 0;
    virtual int getImpedance(char *impedance, size_t len) = 0;
    virtual int getCalibrationResult(char *result, size_t len) = 0;
    virtual int calibrationAndGetResult(char *result, size_t len,
                                        bool is_ftm) = 0;
    virtual int loadDefaultCalibration() { return -ENOTSUP; }

    virtual int deviceMute(bool mute) = 0;
    virtual bool isCalibrated() = 0;
    virtual int calibrate() = 0;

    virtual bool isMesCheckSupport() { return false; }
    virtual int start() { return 0; }
    virtual int stop() { return 0; }
    virtual int monitorThread() { return 0; }
    virtual bool isMonitorEn() { return false; }
    virtual int setMixerParameter(Stream *stream, uint32_t param_id,
                                  void *payload, size_t size) {
        (void)stream;
        (void)param_id;
        (void)payload;
        (void)size;
        return 0;
    }
    virtual int lowBatAdjustGainInit() { return 0; }
    virtual void setGainAdjustEnable(bool enable) { (void)enable; }
    virtual void setGainMapping(
            const std::vector<std::pair<int, int>> &mapping) {
        (void)mapping;
    }
    virtual bool isGainChanged(int gain) { (void)gain; return false; }
    virtual int recordActualGain(int gain) { (void)gain; return 0; }
    virtual int payloadGainConfig(uint8_t **payload, size_t *size,
                                  uint32_t miid, bool enable) {
        (void)payload;
        (void)size;
        (void)miid;
        (void)enable;
        return 0;
    }
    virtual int payloadRotationConfig(uint8_t **payload, size_t *size,
                                     uint32_t miid) {
        (void)payload;
        (void)size;
        (void)miid;
        return 0;
    }
    virtual bool isRTCheckSupport() { return false; }
    virtual int checkSpeakerStatus(Stream *stream, uint32_t param1,
                                   uint32_t param2) {
        (void)stream;
        (void)param1;
        (void)param2;
        return 0;
    }
    virtual int checkSpeakerGetStepDelay(uint32_t step) {
        (void)step;
        return 0;
    }
    virtual int triggerPaErrFbCheck() { return 0; }
    virtual int getPaVbatlowCount(char *buf, size_t len) {
        (void)buf;
        (void)len;
        return 0;
    }
    virtual int payloadCheckConfig(uint8_t **payload, size_t *size,
                                   uint32_t miid) {
        (void)payload;
        (void)size;
        (void)miid;
        return 0;
    }
};

#endif  // OPLUS_SPEAKER_DEVICE_H
