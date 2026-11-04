/*
 * Copyright (c) 2026 The YAAP Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "PAL: OplusSpeakerTfa98xx"

#include "OplusSpeakerTfa98xx.h"

#include <dirent.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>

#include <cutils/str_parms.h>

#include "PalAudioRoute.h"
#include "PalCommon.h"
#include "ResourceManager.h"
#include "Device.h"
#include "Session.h"
#include "Stream.h"
#include "kvh2xml.h"

#define TFA_LIB_PATH "/odm/lib64/libtfa98xx.so"
#define TFA_FW_PATH "/odm/firmware/tfa98xx.cnt"
#define TFA_PROC_PREFIX "tfa98xx-"
#define TFA_MIXER_CHIP_SELECTOR "TFA_CHIP_SELECTOR"
#define TFA_MIXER_PROFILE "TFA Profile"
#define TFA_MIXER_FEEDBACK "TFA_CHECK_FEEDBACK"
#define TFA_MIXER_MUTE "TFA Mute"
#define TFA_TEMP_LIMIT 85
#define TFA_IMPEDANCE_MIN 5000
#define TFA_IMPEDANCE_MAX 8000

std::shared_ptr<OplusSpeakerDevice> OplusSpeakerTfa98xx::obj = nullptr;

OplusSpeakerTfa98xx::OplusSpeakerTfa98xx()
{
}

OplusSpeakerTfa98xx::~OplusSpeakerTfa98xx()
{
    deinit();
}

std::shared_ptr<OplusSpeakerDevice> OplusSpeakerTfa98xx::getInstance()
{
    if (!obj)
        obj = std::shared_ptr<OplusSpeakerDevice>(new OplusSpeakerTfa98xx());
    return obj;
}

std::shared_ptr<OplusSpeakerDevice> OplusSpeakerTfa98xx::getObject()
{
    return obj;
}

int OplusSpeakerTfa98xx::readCalibrationFile(const char *path,
                                             std::vector<int> &values)
{
    FILE *fp = fopen(path, "rb");
    if (!fp)
        return -ENOENT;

    values.clear();
    int value;
    while (fscanf(fp, "%10d", &value) == 1)
        values.push_back(value);

    fclose(fp);
    return values.empty() ? -ENODATA : 0;
}

int OplusSpeakerTfa98xx::writeCalibrationFile(const char *path,
                                              const std::vector<int> &values)
{
    FILE *fp = fopen(path, "wb");
    if (!fp) {
        PAL_ERR(LOG_TAG, "failed to open %s: %s", path, strerror(errno));
        return -errno;
    }

    for (size_t i = 0; i < values.size(); i++)
        fprintf(fp, "%10d", values[i]);
    fflush(fp);
    fsync(fileno(fp));
    fclose(fp);
    return 0;
}

int OplusSpeakerTfa98xx::calibrationInfoInit()
{
    int ret;

    ret = readCalibrationFile(mCalFile.c_str(), mCaliValue);
    if (ret)
        PAL_DBG(LOG_TAG, "no R0 calibration data at %s", mCalFile.c_str());

    ret = readCalibrationFile(mCalF0File.c_str(), mF0Value);
    if (ret)
        PAL_DBG(LOG_TAG, "no F0 calibration data at %s", mCalF0File.c_str());

    mF0Support = true;
    mCalibrated = isCalibrationValid(mCaliValue);
    return mCalibrated ? 0 : -ENODATA;
}

bool OplusSpeakerTfa98xx::isCalibrationValid(
        const std::vector<int> &values) const
{
    if (values.empty())
        return false;

    for (int value : values) {
        if (value < TFA_IMPEDANCE_MIN || value > TFA_IMPEDANCE_MAX)
            return false;
    }

    return true;
}

int OplusSpeakerTfa98xx::loadDefaultCalibration()
{
    std::vector<int> values;

    /*
     * Never run the calibration library in the playback path.  Prefer R0
     * values already programmed in the TFA MTP, then the driver's default
     * impedance.  If neither exists, the caller leaves the OPlus payload
     * disabled and audio continues through the normal speaker path.
     */
    if (!readKernelR0(values)) {
        if (isCalibrationValid(values)) {
            if (values.size() == 1 && mSpkCount > 1)
                values.resize(mSpkCount, values[0]);

            mCaliValue = values;
            mF0Value.clear();
            mCalibrated = true;
            PAL_INFO(LOG_TAG, "using kernel R0 calibration: %d/%d",
                     mCaliValue.size() > 0 ? mCaliValue[0] : 0,
                     mCaliValue.size() > 1 ? mCaliValue[1] : 0);
            return 0;
        }

        PAL_ERR(LOG_TAG, "kernel R0 out of range, trying default impedance");
        values.clear();
    }

    struct mixer *mixer = mHwMixer;
    if (!mixer) {
        std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
        if (rm)
            rm->getHwAudioMixer(&mixer);
    }
    if (!mixer)
        return -EINVAL;

    struct mixer_ctl *ctl = mixer_get_ctl_by_name(mixer,
                                                  "TFA Default Impedance");
    if (!ctl)
        return -ENOENT;

    unsigned int count = mixer_ctl_get_num_values(ctl);
    for (unsigned int i = 0; i < count && i < (unsigned int)mSpkCount; i++) {
        int value = mixer_ctl_get_value(ctl, i);

        if (value >= TFA_IMPEDANCE_MIN && value <= TFA_IMPEDANCE_MAX)
            values.push_back(value);
    }

    if (values.empty())
        return -ENODATA;
    if (values.size() == 1 && mSpkCount > 1)
        values.resize(mSpkCount, values[0]);

    mCaliValue = values;
    mF0Value.clear();
    mCalibrated = true;
    PAL_INFO(LOG_TAG, "using default R0 calibration: %d/%d",
             mCaliValue.size() > 0 ? mCaliValue[0] : 0,
             mCaliValue.size() > 1 ? mCaliValue[1] : 0);
    return 0;
}

int OplusSpeakerTfa98xx::updateCalibrationValue()
{
    int ret = calibrationInfoInit();
    if (!ret)
        mCalibrated = true;
    return ret;
}

int OplusSpeakerTfa98xx::loadTfaCalibrationFunc()
{
    if (mTfaLibHandler)
        return 0;

    if (access(TFA_LIB_PATH, R_OK) != 0) {
        PAL_ERR(LOG_TAG, "%s is not accessible: %s", TFA_LIB_PATH,
                strerror(errno));
        return -ENOENT;
    }

    mTfaLibHandler = dlopen(TFA_LIB_PATH, RTLD_NOW);
    if (!mTfaLibHandler) {
        PAL_ERR(LOG_TAG, "dlopen %s failed: %s", TFA_LIB_PATH, dlerror());
        return -EIO;
    }

    mTfaCalibration =
        (oplus_tfa98xx_calibration_t)dlsym(mTfaLibHandler,
                                           "oplus_tfa98xx_calibration");
    if (!mTfaCalibration) {
        PAL_ERR(LOG_TAG, "dlsym oplus_tfa98xx_calibration failed: %s",
                dlerror());
        dlclose(mTfaLibHandler);
        mTfaLibHandler = nullptr;
        return -EIO;
    }

    PAL_INFO(LOG_TAG, "using %s", TFA_LIB_PATH);
    return 0;
}

int OplusSpeakerTfa98xx::checkDriverReady(struct mixer *mixer)
{
    if (!mixer) {
        PAL_DBG(LOG_TAG, "mixer is NULL");
        return -EINVAL;
    }

    mDriverReady = mixer_get_ctl_by_name(mixer, TFA_MIXER_CHIP_SELECTOR) &&
                   mixer_get_ctl_by_name(mixer, TFA_MIXER_PROFILE);
    if (!mDriverReady)
        PAL_DBG(LOG_TAG, "TFA mixer controls are not available yet");
    return 0;
}

int OplusSpeakerTfa98xx::init()
{
    std::lock_guard<std::mutex> lock(mMutex);
    int ret;

    if (mTfaLibHandler)
        return 0;

    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    if (rm)
        rm->getHwAudioMixer(&mHwMixer);

    checkDriverReady(mHwMixer);

    ret = loadTfaCalibrationFunc();
    if (ret)
        return ret;

    calibrationInfoInit();
    PAL_INFO(LOG_TAG, "initialized: speakers=%d calibrated=%d f0=%d",
             mSpkCount, mCalibrated, mF0Support);
    return 0;
}

int OplusSpeakerTfa98xx::deinit()
{
    std::lock_guard<std::mutex> lock(mMutex);
    if (mTfaLibHandler) {
        dlclose(mTfaLibHandler);
        mTfaLibHandler = nullptr;
        mTfaCalibration = nullptr;
    }
    return 0;
}

int OplusSpeakerTfa98xx::openFile(uint32_t addr, const char *name)
{
    std::string dir;
    (void)addr;

    if (!findTfaProcDir(dir))
        return -ENOENT;

    std::string path = dir + "/" + name;
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
        PAL_DBG(LOG_TAG, "open %s failed: %s", path.c_str(), strerror(errno));
    return fd;
}

int OplusSpeakerTfa98xx::readFile(char *buf, int len, int fd)
{
    if (fd < 0 || !buf || len <= 0)
        return -EINVAL;

    int ret = read(fd, buf, len - 1);
    if (ret < 0)
        return -errno;
    buf[ret] = '\0';
    return ret;
}

int OplusSpeakerTfa98xx::closeFile(int fd)
{
    return fd >= 0 ? close(fd) : 0;
}

bool OplusSpeakerTfa98xx::findTfaProcDir(std::string &dir)
{
    DIR *proc = opendir("/proc");
    if (!proc)
        return false;

    struct dirent *entry;
    while ((entry = readdir(proc)) != nullptr) {
        if (!strncmp(entry->d_name, TFA_PROC_PREFIX,
                     strlen(TFA_PROC_PREFIX))) {
            dir = std::string("/proc/") + entry->d_name;
            closedir(proc);
            return true;
        }
    }

    closedir(proc);
    return false;
}

int OplusSpeakerTfa98xx::readProcString(const char *name, std::string &value)
{
    std::string dir;
    if (!findTfaProcDir(dir))
        return -ENOENT;

    int fd = open((dir + "/" + name).c_str(), O_RDONLY);
    if (fd < 0)
        return -errno;

    char buf[256] = {0};
    int ret = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (ret <= 0)
        return -EIO;

    value.assign(buf);
    while (!value.empty() &&
           (value.back() == '\n' || value.back() == '\r' || value.back() == ' '))
        value.pop_back();
    return 0;
}

int OplusSpeakerTfa98xx::readProcValue(const char *name, int &value)
{
    std::string str;
    if (readProcString(name, str))
        return -ENOENT;

    value = atoi(str.c_str());
    return 0;
}

int OplusSpeakerTfa98xx::readProcImpedance(int &prim, int &sec)
{
    std::string str;

    if (readProcString("impedance", str))
        return -ENOENT;

    prim = -1;
    sec = -1;

    if (sscanf(str.c_str(), " Prim:%d mOhms, Sec:%d mOhms", &prim, &sec) != 2 &&
        sscanf(str.c_str(), "Prim:%d mOhms, Sec:%d mOhms", &prim, &sec) != 2) {
        int only = atoi(str.c_str());
        if (only <= 0)
            return -ENODATA;
        prim = only;
        sec = only;
    }

    if (prim <= 0 || sec <= 0)
        return -ENODATA;

    return 0;
}

int OplusSpeakerTfa98xx::readKernelR0(std::vector<int> &values)
{
    struct mixer *mixer = mHwMixer;

    if (!mixer) {
        std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
        if (rm)
            rm->getHwAudioMixer(&mixer);
    }

    if (!mixer)
        return -EINVAL;

    struct mixer_ctl *ctl = mixer_get_ctl_by_name(mixer, "TFA Calibration");
    if (!ctl)
        ctl = mixer_get_ctl_by_name(mixer, "tfa98xx Calibration");
    if (!ctl) {
        PAL_DBG(LOG_TAG, "TFA Calibration mixer control is not available");
        return -ENOENT;
    }

    unsigned int count = mixer_ctl_get_num_values(ctl);
    std::vector<int> r0;

    for (unsigned int i = 0; i < count && i < (unsigned int)mSpkCount; i++) {
        int value = mixer_ctl_get_value(ctl, i);

        /* RE25 is reported in milliohms by the TFA driver. */
        if (value > 0 && value <= 20000)
            r0.push_back(value);
        else
            PAL_DBG(LOG_TAG, "invalid kernel R0[%u]=%d", i, value);
    }

    if (r0.empty())
        return -ENODATA;

    values = r0;
    return 0;
}

int OplusSpeakerTfa98xx::extractCalibrationVal(const char *result)
{
    char buf[512];
    char *saveptr = nullptr;
    char *token;
    std::vector<int> values;

    if (!result)
        return -EINVAL;

    snprintf(buf, sizeof(buf), "%s", result);

    token = strtok_r(buf, ", ", &saveptr);
    while (token) {
        values.push_back(atoi(token));
        token = strtok_r(nullptr, ", ", &saveptr);
    }

    if (values.empty())
        return -ENODATA;

    mCaliValue.clear();
    mF0Value.clear();

    size_t split = values.size();
    if (mF0Support && values.size() > (size_t)mSpkCount &&
        (values.size() % 2) == 0)
        split = values.size() / 2;

    mCaliValue.assign(values.begin(), values.begin() + split);
    if (split < values.size())
        mF0Value.assign(values.begin() + split, values.end());

    mCalibrated = !mCaliValue.empty();
    return mCalibrated ? 0 : -ENODATA;
}

int OplusSpeakerTfa98xx::storeCalibrationVal(const int *values, int count,
                                             const char *path)
{
    std::vector<int> data;
    for (int i = 0; i < count; i++)
        data.push_back(values[i]);
    return writeCalibrationFile(path, data);
}

int OplusSpeakerTfa98xx::calibrationAndGetResult(char *result, size_t len,
                                                 bool is_ftm)
{
    std::lock_guard<std::mutex> lock(mMutex);
    char cal_result[512] = {0};
    int pcm_dev_id = -1;
    uint32_t miid = 0;
    int ret;

    if (!mTfaCalibration) {
        ret = loadTfaCalibrationFunc();
        if (ret) {
            snprintf(result, len, "SpkrCalStatus: %d;", ret);
            return ret;
        }
    }

    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    std::shared_ptr<Device> dev = Device::getObject(PAL_DEVICE_OUT_SPEAKER);
    std::vector<Stream *> activeStreams;

    if (rm && dev &&
        !rm->getActiveStream_l(activeStreams, dev) && !activeStreams.empty()) {
        Stream *stream = activeStreams[0];
        Session *session = nullptr;
        std::string backendName;

        if (!stream->getAssociatedSession(&session) && session) {
            pcm_dev_id = session->getFrontEndId(PAL_AUDIO_OUTPUT);
            rm->getBackendName(PAL_DEVICE_OUT_SPEAKER, backendName);
            if (session->getMIID(backendName.c_str(), MODULE_SP, &miid))
                miid = 0;
        }
    }

    if (pcm_dev_id < 0) {
        PAL_ERR(LOG_TAG, "no active speaker PCM device for calibration");
        snprintf(result, len, "SpkrCalStatus: %d;", -ENODEV);
        return -ENODEV;
    }

    PAL_INFO(LOG_TAG, "calibrating speaker: pcm_dev_id=%d miid=%u",
             pcm_dev_id, miid);

    ret = mTfaCalibration(mFwPath.c_str(), pcm_dev_id, miid, is_ftm,
                          cal_result, (int)sizeof(cal_result));
    if (ret) {
        PAL_ERR(LOG_TAG, "oplus_tfa98xx_calibration failed: %d", ret);

        /*
         * The F0 stage can fail after R0 has already been written to MTP.
         * Recover the valid R0 values so playback can continue with
         * R0-only protection instead of failing calibration and retrying
         * while audio is active.
         */
        mCaliValue.clear();
        if (ret > 0 && !readKernelR0(mCaliValue) &&
            isCalibrationValid(mCaliValue)) {
            if (mCaliValue.size() == 1 && mSpkCount > 1)
                mCaliValue.resize(mSpkCount, mCaliValue[0]);
            mF0Value.clear();
            mCalibrated = true;

            int write_ret = writeCalibrationFile(mCalFile.c_str(),
                                                 mCaliValue);
            if (write_ret)
                PAL_ERR(LOG_TAG, "failed to save R0-only calibration: %d",
                        write_ret);

            std::ostringstream os;
            os << "SpkrCalStatus: 0; R0: ";
            for (size_t i = 0; i < mCaliValue.size(); i++)
                os << (i ? ", " : "") << mCaliValue[i];
            os << "; F0: ;";
            snprintf(result, len, "%s", os.str().c_str());
            PAL_INFO(LOG_TAG,
                     "F0 calibration failed; using R0-only calibration: %s",
                     result);
            return 0;
        }

        mCaliValue.clear();
        snprintf(result, len, "SpkrCalStatus: %d;", ret);
        return ret < 0 ? ret : -EIO;
    }

    if (extractCalibrationVal(cal_result)) {
        PAL_ERR(LOG_TAG, "failed to parse calibration result: %s", cal_result);
        snprintf(result, len, "SpkrCalStatus: %d;", -EINVAL);
        return -EINVAL;
    }
    if (!isCalibrationValid(mCaliValue)) {
        PAL_ERR(LOG_TAG, "calibration returned out-of-range R0 values");
        mCaliValue.clear();
        mCalibrated = false;
        snprintf(result, len, "SpkrCalStatus: %d;", -ERANGE);
        return -ERANGE;
    }
    if (mCaliValue.size() == 1 && mSpkCount > 1)
        mCaliValue.resize(mSpkCount, mCaliValue[0]);
    if (!mCaliValue.empty()) {
        int write_ret = writeCalibrationFile(mCalFile.c_str(), mCaliValue);
        if (write_ret)
            PAL_ERR(LOG_TAG, "failed to save R0 calibration: %d", write_ret);
    }
    if (!mF0Value.empty()) {
        int write_ret = writeCalibrationFile(mCalF0File.c_str(), mF0Value);
        if (write_ret)
            PAL_ERR(LOG_TAG, "failed to save F0 calibration: %d", write_ret);
    }

    std::ostringstream os;
    os << "SpkrCalStatus: 0; R0: ";
    for (size_t i = 0; i < mCaliValue.size(); i++)
        os << (i ? ", " : "") << mCaliValue[i];
    os << "; F0: ";
    for (size_t i = 0; i < mF0Value.size(); i++)
        os << (i ? ", " : "") << mF0Value[i];
    os << ";";
    snprintf(result, len, "%s", os.str().c_str());
    PAL_INFO(LOG_TAG, "calibration result: %s", result);
    return 0;
}

int OplusSpeakerTfa98xx::calibrate()
{
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mCalibrating)
            return -EBUSY;
        mCalibrating = true;
    }

    char result[512] = {0};
    int ret = calibrationAndGetResult(result, sizeof(result), false);

    {
        std::lock_guard<std::mutex> lock(mMutex);
        mCalibrating = false;
    }

    return ret;
}

bool OplusSpeakerTfa98xx::isCalibrated()
{
    if (!mCalibrated)
        updateCalibrationValue();
    return mCalibrated;
}

int OplusSpeakerTfa98xx::payloadDspCmd(uint8_t **payload, size_t *size,
                                       uint32_t miid, uint32_t param_id,
                                       uint8_t *data, size_t data_size)
{
    size_t old_size;
    size_t data_aligned;
    size_t new_size;
    size_t pad;
    uint8_t *buf;
    uint32_t *header;

    if (!payload || !size || (!data && data_size))
        return -EINVAL;

    old_size = *size;
    data_aligned = (data_size + 3) & ~(size_t)3;
    if (data_aligned > (size_t)-1 - old_size - 16)
        return -EOVERFLOW;

    new_size = old_size + 16 + data_aligned;
    pad = (8 - (new_size & 7)) & 7;
    if (pad > (size_t)-1 - new_size)
        return -EOVERFLOW;
    new_size += pad;

    buf = (uint8_t *)realloc(*payload, new_size);
    if (!buf)
        return -ENOMEM;

    memset(buf + old_size, 0, new_size - old_size);
    header = (uint32_t *)(buf + old_size);
    header[0] = miid;
    header[1] = param_id;
    header[2] = (uint32_t)data_aligned;
    header[3] = 0;

    if (data_size)
        memcpy(buf + old_size + 16, data, data_size);

    *payload = buf;
    *size = new_size;
    return 0;
}

int OplusSpeakerTfa98xx::payloadMultiCmd(uint8_t *out, int spk_idx, int r0,
                                         int f0)
{
    int selector = (spk_idx & 0xf) << 4;

    (void)f0;

    if (!out)
        return -EINVAL;

    /*
     * If neither calibration nor gain adjustment applies, stock returns zero
     * length. If gain adjustment is enabled, it still emits the 3-byte
     * command header followed by the gain block.
     */
    if (!r0 && !mGainAdjustEnabled)
        return 0;

    out[0] = selector;
    out[1] = 0x80;
    out[2] = 0x15;

    int len = 3;
    if (r0) {
        out[3] = 0;
        out[4] = 0;
        out[5] = 2;
        out[6] = selector;
        out[7] = 0x81;
        out[8] = 0x1c;
        out[9] = (r0 >> 16) & 0xff;
        out[10] = (r0 >> 8) & 0xff;
        out[11] = r0 & 0xff;
        len = 12;
    }

    if (mGainAdjustEnabled) {
        int gain = currentGainLevel();

        out[len + 0] = 0;
        out[len + 1] = 0;
        out[len + 2] = 3;
        out[len + 3] = selector;
        out[len + 4] = 0x81;
        out[len + 5] = 0x04;
        out[len + 6] = 0;
        out[len + 7] = 0;
        out[len + 8] = gain & 0xff;
        out[len + 9] = 0;
        out[len + 10] = 0;
        out[len + 11] = gain & 0xff;
        len += 12;
    }

    return len;
}

int OplusSpeakerTfa98xx::payloadSPConfig(uint8_t **payload, size_t *size,
                                         uint32_t miid)
{
    uint8_t block[64] = {0};
    int cmd_len;
    int ret;

    if (!payload || !size)
        return -EINVAL;

    *payload = nullptr;
    *size = 0;

    if (mCaliValue.empty())
        calibrationInfoInit();

    if (mCaliValue.empty()) {
        PAL_INFO(LOG_TAG, "no calibration data, skipping SP payload");
        return 0;
    }

    if (!isCalibrationValid(mCaliValue)) {
        PAL_ERR(LOG_TAG, "invalid R0 calibration, skipping SP payload");
        return 0;
    }

    for (int i = 0; i < mSpkCount && i < 2 && i < (int)mCaliValue.size(); i++) {
        int r0 = mCaliValue[i];
        int first = i * 2;
        int second = first + 1;
        int r0_first = first < (int)mCaliValue.size() ? mCaliValue[first] : r0;
        int r0_second = second < (int)mCaliValue.size() ?
                        mCaliValue[second] : r0_first;
        int offset;

        memset(block, 0, sizeof(block));
        cmd_len = payloadMultiCmd(block, i, r0, 0);

        /*
         * Stock payloadSPConfig still sends a 9-byte calibration block when
         * payloadMultiCmd returns zero.  When it returns a command sequence,
         * insert the 3-byte sub-command header before the R0 block; without
         * the length byte the DSP ignores the calibration data.
         */
        if (cmd_len > 0) {
            block[cmd_len] = 0;
            block[cmd_len + 1] = 0;
            block[cmd_len + 2] = 3;
            offset = cmd_len + 3;
        } else {
            offset = 0;
        }
        block[offset] = (i & 0xf) << 4;
        block[offset + 1] = 0x81;
        block[offset + 2] = 0x05;
        block[offset + 3] = (r0_first >> 16) & 0xff;
        block[offset + 4] = (r0_first >> 8) & 0xff;
        block[offset + 5] = r0_first & 0xff;
        block[offset + 6] = (r0_second >> 16) & 0xff;
        block[offset + 7] = (r0_second >> 8) & 0xff;
        block[offset + 8] = r0_second & 0xff;

        ret = payloadDspCmd(payload, size, miid, 0x1800B921, block,
                            offset + (cmd_len > 0 ? 12 : 9));
        if (ret) {
            free(*payload);
            *payload = nullptr;
            *size = 0;
            return ret;
        }
    }

    return 0;
}

int OplusSpeakerTfa98xx::getDeviceCount()
{
    return mSpkCount;
}

int OplusSpeakerTfa98xx::getCalibrationRange(char *range, size_t len)
{
    return snprintf(range, len,
                    "ch:%d, chip_type %d, minMohms:%d, maxMohms:%d, "
                    "default_r0 %u, f0_enable %d, f0_min %d, f0_max %d",
                    mSpkCount, mChipType, TFA_IMPEDANCE_MIN,
                    TFA_IMPEDANCE_MAX,
                    mCaliValue.empty() ? 0u : (unsigned)mCaliValue[0],
                    mF0Support ? 1 : 0, 0, 0);
}

int OplusSpeakerTfa98xx::getImpedance(char *impedance, size_t len)
{
    std::vector<int> r0;

    if (!readKernelR0(r0) && !r0.empty())
        return snprintf(impedance, len, "%d", r0[0]);

    int prim;
    int sec;
    if (!readProcImpedance(prim, sec))
        return snprintf(impedance, len, "%d", prim);

    if (!mCaliValue.empty())
        return snprintf(impedance, len, "%d", mCaliValue[0]);

    return snprintf(impedance, len, "0");
}

int OplusSpeakerTfa98xx::getCalibrationResult(char *result, size_t len)
{
    if (mCaliValue.empty())
        updateCalibrationValue();

    std::ostringstream os;
    os << "SpkrCalStatus: " << (mCalibrated ? 0 : -1) << "; R0: ";
    for (size_t i = 0; i < mCaliValue.size(); i++)
        os << (i ? ", " : "") << mCaliValue[i];
    os << "; F0: ";
    for (size_t i = 0; i < mF0Value.size(); i++)
        os << (i ? ", " : "") << mF0Value[i];
    os << ";";
    return snprintf(result, len, "%s", os.str().c_str());
}

int OplusSpeakerTfa98xx::deviceMute(bool mute)
{
    struct mixer *mixer = mHwMixer;
    if (!mixer) {
        std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
        if (rm)
            rm->getHwAudioMixer(&mixer);
    }
    if (!mixer)
        return -EINVAL;

    struct mixer_ctl *ctl = mixer_get_ctl_by_name(mixer, TFA_MIXER_MUTE);
    if (!ctl) {
        PAL_DBG(LOG_TAG, "%s mixer control is not available", TFA_MIXER_MUTE);
        return -ENOENT;
    }

    return mixer_ctl_set_enum_by_string(ctl, mute ? "mute" : "unmute");
}

int OplusSpeakerTfa98xx::forceSpeakerProfile()
{
    struct mixer *mixer = mHwMixer;

    if (!mixer) {
        std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
        if (rm)
            rm->getHwAudioMixer(&mixer);
    }
    if (!mixer)
        return -EINVAL;

    struct mixer_ctl *profile = mixer_get_ctl_by_name(mixer,
                                                      TFA_MIXER_PROFILE);
    if (!profile)
        return -ENOENT;

    unsigned int num_enums = mixer_ctl_get_num_enums(profile);
    int speaker = -1;

    for (unsigned int i = 0; i < num_enums; i++) {
        const char *name = mixer_ctl_get_enum_string(profile, i);

        if (name && !strcasecmp(name, "speaker")) {
            speaker = (int)i;
            break;
        }
    }

    if (speaker < 0)
        return -ENOENT;

    int current = mixer_ctl_get_value(profile, 0);

    /*
     * tfa98xx_set_profile() returns early when the requested enum index is
     * already active.  Switch to another profile first so the speaker
     * profile is really reprogrammed after a calibration sequence.
     */
    if (current == speaker) {
        if (num_enums < 2)
            return -ENOTSUP;

        int alternate = (speaker == 0) ? 1 : 0;
        int ret = mixer_ctl_set_value(profile, 0, alternate);
        if (ret)
            return ret;
    }

    return mixer_ctl_set_value(profile, 0, speaker);
}

int OplusSpeakerTfa98xx::applyProfile(bool enable)
{
    struct mixer *mixer = mHwMixer;
    if (!mixer) {
        std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
        if (rm)
            rm->getHwAudioMixer(&mixer);
    }
    if (!mixer)
        return -EINVAL;

    struct mixer_ctl *selector =
        mixer_get_ctl_by_name(mixer, TFA_MIXER_CHIP_SELECTOR);
    struct mixer_ctl *profile =
        mixer_get_ctl_by_name(mixer, TFA_MIXER_PROFILE);
    struct mixer_ctl *feedback =
        mixer_get_ctl_by_name(mixer, TFA_MIXER_FEEDBACK);

    if (selector && enable)
        mixer_ctl_set_value(selector, 0, 3);
    if (profile) {
        if (enable) {
            if (forceSpeakerProfile())
                mixer_ctl_set_enum_by_string(profile, "speaker");
        } else {
            mixer_ctl_set_enum_by_string(profile, "speaker");
        }
    }
    if (feedback)
        mixer_ctl_set_enum_by_string(feedback, enable ? "On" : "Off");

    return 0;
}

int OplusSpeakerTfa98xx::start()
{
    return applyProfile(true);
}

int OplusSpeakerTfa98xx::stop()
{
    return applyProfile(false);
}

int OplusSpeakerTfa98xx::monitorThread()
{
    return 0;
}

bool OplusSpeakerTfa98xx::isMonitorEn()
{
    return mDriverReady;
}

int OplusSpeakerTfa98xx::setMixerParameter(Stream *stream, uint32_t param_id,
                                           void *payload, size_t size)
{
    (void)stream;
    (void)param_id;
    (void)payload;
    (void)size;
    return 0;
}

int OplusSpeakerTfa98xx::lowBatAdjustGainInit()
{
    return 0;
}

void OplusSpeakerTfa98xx::setGainAdjustEnable(bool enable)
{
    mGainAdjustEnabled = enable;
    PAL_INFO(LOG_TAG, "gain adjust enabled=%d", mGainAdjustEnabled);
}

void OplusSpeakerTfa98xx::setGainMapping(
        const std::vector<std::pair<int, int>> &mapping)
{
    if (mapping.empty())
        return;

    mGainMapping = mapping;
    std::sort(mGainMapping.begin(), mGainMapping.end(),
              [](const std::pair<int, int> &a,
                 const std::pair<int, int> &b) {
                  return a.first < b.first;
              });
    PAL_INFO(LOG_TAG, "gain mapping entries=%zu", mGainMapping.size());
}

int OplusSpeakerTfa98xx::readBatteryCapacity() const
{
    int capacity = 100;
    FILE *fp = fopen("/sys/class/power_supply/battery/capacity", "r");

    if (fp) {
        if (fscanf(fp, "%d", &capacity) != 1)
            capacity = 100;
        fclose(fp);
    }

    if (capacity < 0)
        capacity = 0;
    if (capacity > 100)
        capacity = 100;

    return capacity;
}

int OplusSpeakerTfa98xx::gainForCapacity(int capacity) const
{
    /*
     * Mapping entries are battery thresholds: at or below the threshold,
     * use the associated gain level.  Entries are kept in ascending order
     * so the first matching threshold is the closest one.
     */
    for (const auto &entry : mGainMapping) {
        if (capacity <= entry.first)
            return entry.second;
    }

    return mGainMapping.empty() ? 0 : mGainMapping.back().second;
}

int OplusSpeakerTfa98xx::currentGainLevel()
{
    int gain = gainForCapacity(readBatteryCapacity());

    if (gain < 0)
        gain = 0;
    if (gain > 15)
        gain = 15;

    if (gain != mGainVal) {
        mPreGainVal = mGainVal;
        mGainVal = gain;
        PAL_DBG(LOG_TAG, "gain level %d -> %d", mPreGainVal, mGainVal);
    }

    return gain;
}

bool OplusSpeakerTfa98xx::isGainChanged(int gain)
{
    if (!mGainAdjustEnabled)
        return false;
    return gain < 0 || gain > 15 || gain != mGainVal;
}

int OplusSpeakerTfa98xx::recordActualGain(int gain)
{
    if (!mGainAdjustEnabled)
        return 0;

    if (gain < 0)
        gain = 0;
    if (gain > 15)
        gain = 15;

    mPreGainVal = mGainVal;
    mGainVal = gain;
    PAL_DBG(LOG_TAG, "gain changed %d -> %d, auto calibration %d",
            mPreGainVal, mGainVal, mAutoCalSupport);
    return 0;
}

int OplusSpeakerTfa98xx::payloadGainConfig(uint8_t **payload, size_t *size,
                                           uint32_t miid, bool enable)
{
    uint8_t block[9] = {0};
    int ret;

    if (!payload || !size)
        return -EINVAL;

    *payload = nullptr;
    *size = 0;

    if (!enable || !mGainAdjustEnabled || mCaliValue.empty())
        return 0;

    int gain = currentGainLevel();

    for (int i = 0; i < mSpkCount && i < 2 && i < (int)mCaliValue.size(); i++) {
        memset(block, 0, sizeof(block));
        block[0] = (i & 0xf) << 4;
        block[1] = 0x81;
        block[2] = 0x04;
        block[5] = gain & 0xff;
        block[8] = gain & 0xff;

        ret = payloadDspCmd(payload, size, miid, 0x1800B921, block,
                            sizeof(block));
        if (ret) {
            free(*payload);
            *payload = nullptr;
            *size = 0;
            return ret;
        }
    }

    return 0;
}

int OplusSpeakerTfa98xx::payloadRotationConfig(uint8_t **payload, size_t *size,
                                               uint32_t miid)
{
    (void)payload;
    (void)miid;
    if (size)
        *size = 0;
    return 0;
}

bool OplusSpeakerTfa98xx::isRTCheckSupport()
{
    return false;
}

int OplusSpeakerTfa98xx::checkSpeakerStatus(Stream *stream, uint32_t param1,
                                            uint32_t param2)
{
    (void)stream;
    (void)param1;
    (void)param2;

    bool impedance_checked = false;
    std::vector<int> r0;

    if (!readKernelR0(r0)) {
        for (size_t i = 0; i < r0.size(); i++) {
            if (r0[i] < TFA_IMPEDANCE_MIN || r0[i] > TFA_IMPEDANCE_MAX) {
                PAL_ERR(LOG_TAG, "speaker impedance out of range: %d mOhm",
                        r0[i]);
                return -ERANGE;
            }
        }
        impedance_checked = true;
    }

    if (!impedance_checked) {
        int prim;
        int sec;

        if (!readProcImpedance(prim, sec)) {
            if (prim < TFA_IMPEDANCE_MIN || prim > TFA_IMPEDANCE_MAX ||
                sec < TFA_IMPEDANCE_MIN || sec > TFA_IMPEDANCE_MAX) {
                PAL_ERR(LOG_TAG,
                        "speaker impedance out of range: %d/%d mOhm",
                        prim, sec);
                return -ERANGE;
            }
            impedance_checked = true;
        }
    }

    if (!impedance_checked)
        PAL_DBG(LOG_TAG, "speaker impedance source unavailable");

    int value;
    if (!readProcValue("temperature", value) && value > TFA_TEMP_LIMIT) {
        PAL_ERR(LOG_TAG, "speaker over temperature: %d", value);
        return -ERANGE;
    }

    std::string status;
    if (!readProcString("status", status)) {
        if (status.find("damage") != std::string::npos ||
            status.find("error") != std::string::npos) {
            PAL_ERR(LOG_TAG, "speaker status reports %s", status.c_str());
            return -EIO;
        }
    }

    return 0;
}

int OplusSpeakerTfa98xx::checkSpeakerGetStepDelay(uint32_t step)
{
    (void)step;
    return 0;
}

int OplusSpeakerTfa98xx::triggerPaErrFbCheck()
{
    return 0;
}

int OplusSpeakerTfa98xx::getPaVbatlowCount(char *buf, size_t len)
{
    FILE *fp = fopen("/sys/class/power_supply/battery/capacity", "r");
    if (!fp) {
        if (buf && len)
            snprintf(buf, len, "0");
        return 0;
    }

    int capacity = 0;
    if (fscanf(fp, "%d", &capacity) != 1)
        capacity = 0;
    fclose(fp);

    if (buf && len)
        snprintf(buf, len, "%d", capacity < 15 ? 1 : 0);
    return capacity < 15 ? 1 : 0;
}

int OplusSpeakerTfa98xx::payloadCheckConfig(uint8_t **payload, size_t *size,
                                            uint32_t miid)
{
    (void)payload;
    (void)miid;
    if (size)
        *size = 0;
    return 0;
}

bool OplusSpeakerTfa98xx::isMesCheckSupport()
{
    return false;
}

int OplusSpeakerTfa98xx::getSpkDamageStatus(Stream *stream, uint32_t param1,
                                             uint32_t param2)
{
    (void)stream;
    (void)param1;
    (void)param2;
    return checkSpeakerStatus(nullptr, 0, 0);
}

int OplusSpeakerTfa98xx::getAlgorithmVersion(Stream *stream, uint32_t param)
{
    (void)stream;
    (void)param;
    return 0;
}

int OplusSpeakerTfa98xx::recordAlgorithmVersion(Stream *stream, uint32_t param)
{
    (void)stream;
    (void)param;
    return 0;
}

int OplusSpeakerTfa98xx::checkAlgorithmVersion(Stream *stream, uint32_t param)
{
    (void)stream;
    (void)param;
    return 0;
}

int OplusSpeakerTfa98xx::checkSpeakerF0(uint32_t param1, int param2)
{
    (void)param1;
    (void)param2;
    return 0;
}

int OplusSpeakerTfa98xx::checkSpeakerRe(uint32_t param1, int param2)
{
    (void)param1;
    (void)param2;
    return 0;
}

int OplusSpeakerTfa98xx::chkSpkResultFeedback(Stream *stream, uint32_t param1,
                                               uint8_t *payload,
                                               uint32_t param2)
{
    (void)stream;
    (void)param1;
    (void)payload;
    (void)param2;
    return 0;
}

int OplusSpeakerTfa98xx::getSpkInfoAndCheckFeedback(Stream *stream,
                                                    uint32_t param1,
                                                    uint32_t param2)
{
    (void)stream;
    (void)param1;
    (void)param2;
    return 0;
}

int OplusSpeakerTfa98xx::setCmdReadyForSpkInfo(Stream *stream, uint32_t param1,
                                                uint32_t param2)
{
    (void)stream;
    (void)param1;
    (void)param2;
    return 0;
}

int OplusSpeakerTfa98xx::setConfigParams(struct str_parms *parms, char *value,
                                         int len)
{
    if (!parms || !value || len <= 0)
        return -EINVAL;

    if (str_parms_get_str(parms, "speaker_auto_calibration", value, len) >= 0)
        mAutoCalSupport = !strncmp(value, "true", sizeof("true") - 1);

    if (str_parms_get_str(parms, "oplus_check_speaker", value, len) >= 0) {
        bool enabled = !strncmp(value, "true", sizeof("true") - 1);
        if (!enabled)
            mDriverReady = false;
    }

    return 0;
}

int OplusSpeakerTfa98xx::getImpedanceFromFile(char *impedance, size_t len)
{
    if (mCaliValue.empty())
        return snprintf(impedance, len, "0");
    return snprintf(impedance, len, "%d", mCaliValue[0]);
}

int OplusSpeakerTfa98xx::getImpedanceFromMtp(char *impedance, size_t len)
{
    return getImpedance(impedance, len);
}
