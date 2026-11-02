/*
 * Copyright (c) 2026 The YAAP Project
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OPLUS_SPEAKER_TFA98XX_H
#define OPLUS_SPEAKER_TFA98XX_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "OplusSpeakerDevice.h"

struct mixer;
struct str_parms;
class Stream;

typedef int (*oplus_tfa98xx_calibration_t)(const char *fw_path, int pcm_dev_id,
                                           int miid, bool aging_flag,
                                           char *result, int result_size);

class OplusSpeakerTfa98xx : public OplusSpeakerDevice {
public:
    static std::shared_ptr<OplusSpeakerDevice> getInstance();
    static std::shared_ptr<OplusSpeakerDevice> getObject();

    ~OplusSpeakerTfa98xx() override;

    int init() override;
    int payloadSPConfig(uint8_t **payload, size_t *size,
                        uint32_t miid) override;
    int getDeviceCount() override;
    int getCalibrationRange(char *range, size_t len) override;
    int getImpedance(char *impedance, size_t len) override;
    int getCalibrationResult(char *result, size_t len) override;
    int calibrationAndGetResult(char *result, size_t len,
                                bool is_ftm) override;
    int loadDefaultCalibration() override;

    int deviceMute(bool mute) override;
    bool isCalibrated() override;
    int calibrate() override;

    bool isMesCheckSupport() override;
    int start() override;
    int stop() override;
    int monitorThread() override;
    bool isMonitorEn() override;
    int setMixerParameter(Stream *stream, uint32_t param_id, void *payload,
                          size_t size) override;
    int lowBatAdjustGainInit() override;
    void setGainAdjustEnable(bool enable) override;
    void setGainMapping(
            const std::vector<std::pair<int, int>> &mapping) override;
    bool isGainChanged(int gain) override;
    int recordActualGain(int gain) override;
    int payloadGainConfig(uint8_t **payload, size_t *size, uint32_t miid,
                          bool enable) override;
    int payloadRotationConfig(uint8_t **payload, size_t *size,
                             uint32_t miid) override;
    bool isRTCheckSupport() override;
    int checkSpeakerStatus(Stream *stream, uint32_t param1,
                           uint32_t param2) override;
    int checkSpeakerGetStepDelay(uint32_t step) override;
    int triggerPaErrFbCheck() override;
    int getPaVbatlowCount(char *buf, size_t len) override;
    int payloadCheckConfig(uint8_t **payload, size_t *size,
                           uint32_t miid) override;

    int deinit();
    int loadTfaCalibrationFunc();
    int setConfigParams(struct str_parms *parms, char *value, int len);
    int getSpkDamageStatus(Stream *stream, uint32_t param1, uint32_t param2);
    int getAlgorithmVersion(Stream *stream, uint32_t param);
    int recordAlgorithmVersion(Stream *stream, uint32_t param);
    int checkAlgorithmVersion(Stream *stream, uint32_t param);
    int checkDriverReady(struct mixer *mixer);

private:
    OplusSpeakerTfa98xx();

    int calibrationInfoInit();
    int updateCalibrationValue();
    int readKernelR0(std::vector<int> &values);
    int readProcImpedance(int &prim, int &sec);
    int forceSpeakerProfile();
    int extractCalibrationVal(const char *result);
    int storeCalibrationVal(const int *values, int count,
                            const char *path);
    int readCalibrationFile(const char *path, std::vector<int> &values);
    int writeCalibrationFile(const char *path, const std::vector<int> &values);
    int openFile(uint32_t addr, const char *name);
    int readFile(char *buf, int len, int fd);
    int closeFile(int fd);
    int getImpedanceFromFile(char *impedance, size_t len);
    int getImpedanceFromMtp(char *impedance, size_t len);
    int checkSpeakerF0(uint32_t param1, int param2);
    int checkSpeakerRe(uint32_t param1, int param2);
    int chkSpkResultFeedback(Stream *stream, uint32_t param1,
                             uint8_t *payload, uint32_t param2);
    int getSpkInfoAndCheckFeedback(Stream *stream, uint32_t param1,
                                   uint32_t param2);
    int setCmdReadyForSpkInfo(Stream *stream, uint32_t param1,
                              uint32_t param2);
    int payloadDspCmd(uint8_t **payload, size_t *size, uint32_t miid,
                      uint32_t param_id, uint8_t *data, size_t data_size);
    int payloadMultiCmd(uint8_t *out, int spk_idx, int r0, int f0);
    int applyProfile(bool enable);
    int readProcValue(const char *name, int &value);
    int readProcString(const char *name, std::string &value);
    bool findTfaProcDir(std::string &dir);
    int readBatteryCapacity() const;
    int currentGainLevel();
    int gainForCapacity(int capacity) const;
    bool isCalibrationValid(const std::vector<int> &values) const;

    static std::shared_ptr<OplusSpeakerDevice> obj;

    std::mutex mMutex;
    void *mTfaLibHandler = nullptr;
    oplus_tfa98xx_calibration_t mTfaCalibration = nullptr;
    struct mixer *mHwMixer = nullptr;

    bool mCalibrating = false;
    bool mCalibrated = false;
    bool mF0Support = true;
    bool mAutoCalSupport = true;
    bool mGainAdjustEnabled = false;
    bool mDriverReady = false;

    int mSpkCount = 2;
    int mChipType = 0;
    int mGainVal = -1;
    int mPreGainVal = -1;
    std::vector<std::pair<int, int>> mGainMapping = {{20, 2}, {100, 0}};

    std::string mFwPath = "/odm/firmware/tfa98xx.cnt";
    std::string mCalFile = "/mnt/vendor/persist/media/tfa_cali.bin";
    std::string mCalF0File = "/mnt/vendor/persist/media/tfa_cali_f0.bin";
    std::vector<int> mCaliValue;
    std::vector<int> mF0Value;
};

#endif  // OPLUS_SPEAKER_TFA98XX_H
