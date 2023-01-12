/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: HotwordInterface"
//#define LOG_NDEBUG 0

#include <log/log.h>
#include "HotwordInterface.h"

#define PAL_LOG_ERR             (0x1) /**< error message, represents code bugs that should be debugged and fixed.*/
#define PAL_LOG_INFO            (0x2) /**< info message, additional info to support debug */
#define PAL_LOG_DBG             (0x4) /**< debug message, required at minimum for debug.*/
#define PAL_LOG_VERBOSE         (0x8)/**< verbose message, useful primarily to help developers debug low-level code */

static uint32_t pal_log_lvl = PAL_LOG_ERR | PAL_LOG_INFO | PAL_LOG_DBG;

#define PAL_FATAL(log_tag, arg,...)                                       \
    if (pal_log_lvl & PAL_LOG_ERR) {                              \
        ALOGE("%s: %d: "  arg, __func__, __LINE__, ##__VA_ARGS__);\
        abort();                                                  \
    }

#define PAL_ERR(log_tag, arg,...)                                          \
    if (pal_log_lvl & PAL_LOG_ERR) {                              \
        ALOGE("%s: %d: "  arg, __func__, __LINE__, ##__VA_ARGS__);\
    }
#define PAL_DBG(log_tag, arg,...)                                           \
    if (pal_log_lvl & PAL_LOG_DBG) {                               \
        ALOGD("%s: %d: "  arg, __func__, __LINE__, ##__VA_ARGS__); \
    }
#define PAL_INFO(log_tag, arg,...)                                         \
    if (pal_log_lvl & PAL_LOG_INFO) {                             \
        ALOGI("%s: %d: "  arg, __func__, __LINE__, ##__VA_ARGS__);\
    }
#define PAL_VERBOSE(log_tag, arg,...)                                      \
    if (pal_log_lvl & PAL_LOG_VERBOSE) {                          \
        ALOGV("%s: %d: "  arg, __func__, __LINE__, ##__VA_ARGS__);\
    }

extern "C" int32_t get_vui_interface(struct vui_intf_t *intf,
    vui_intf_param_t *model) {

    int32_t status = 0;
    sound_model_config_t *config = nullptr;

    if (!intf || !model || !model->data)
        return -EINVAL;

    config = (sound_model_config_t *)model->data;
    switch (*config->module_type) {
        case ST_MODULE_TYPE_HW:
            intf->interface = std::make_shared<HotwordInterface>(model);
            break;
        default:
            PAL_ERR(LOG_TAG, "Unsupported module type %d", *config->module_type);
            status = -EINVAL;
            break;
    }

    return status;
}

extern "C" int32_t release_vui_interface(struct vui_intf_t *intf) {
    int32_t status = 0;

    if (!intf)
        return -EINVAL;

    if (intf->interface) {
        intf->interface = nullptr;
    }

    return status;
}

HotwordInterface::HotwordInterface(
    vui_intf_param_t *model) {

    int32_t status = 0;
    struct pal_st_sound_model *sound_model = nullptr;
    std::vector<sound_model_data_t *> model_list;
    sound_model_config_t *config = nullptr;

    custom_event_ = nullptr;
    custom_event_size_ = 0;
    memset(&buffering_config_, 0, sizeof(buffering_config_));

    if (!model || !model->data) {
        PAL_ERR(LOG_TAG, "Invalid input");
        throw std::runtime_error("Invalid input");
    }

    config = (sound_model_config_t *)model->data;
    sound_model = (struct pal_st_sound_model *)config->sound_model;
    status = HotwordInterface::ParseSoundModel(sound_model, model_list);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to parse sound model, status = %d", status);
        throw std::runtime_error("Failed to parse sound model");
    }

    module_type_ = *config->module_type;

    status = RegisterModel(model->stream, sound_model, model_list);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to register sound model, status = %d", status);
        throw std::runtime_error("Failed to register sound model");
    }
}

HotwordInterface::~HotwordInterface() {
    PAL_DBG(LOG_TAG, "Enter");

    if (custom_event_)
        free(custom_event_);

    PAL_DBG(LOG_TAG, "Exit");
}

void HotwordInterface::DetachStream(void *stream) {
    DeregisterModel(stream);
}

int32_t HotwordInterface::SetParameter(
    intf_param_id_t param_id, vui_intf_param_t *param) {

    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");

    if (!param) {
        PAL_ERR(LOG_TAG, "Invalid param");
        return -EINVAL;
    }

    switch (param_id) {
        case PARAM_RECOGNITION_CONFIG: {
            status = ParseRecognitionConfig(param->stream,
                (struct pal_st_recognition_config *)param->data);
            break;
        }
        case PARAM_DETECTION_EVENT: {
            status = ParseDetectionPayload(param->data, param->size);
            break;
        }
        case PARAM_DETECTION_RESULT: {
            UpdateDetectionResult(param->stream, *(uint32_t *)param->data);
            break;
        }
        case PARAM_STREAM_ATTRIBUTES: {
            SetStreamAttributes((struct pal_stream_attributes *)param->data);
            break;
        }
        case PARAM_KEYWORD_DURATION: {
            kw_duration_ = *(uint32_t *)param->data;
            break;
        }
        default:
            PAL_DBG(LOG_TAG, "Unsupported param id %d", param_id);
            break;
    }

    PAL_DBG(LOG_TAG, "Exit, status = %d", status);
    return status;
}

int32_t HotwordInterface::GetParameter(
    intf_param_id_t param_id, vui_intf_param_t *param) {

    int32_t status = 0;

    switch (param_id) {
        case PARAM_INTERFACE_PROPERTY: {
            vui_intf_property_t *property = (vui_intf_property_t *)param->data;
            if (property) {
                property->is_qc_wakeup_config = false;
                property->is_multi_model_supported = false;
            } else {
                PAL_ERR(LOG_TAG, "Invalid property");
                status = -EINVAL;
            }
            break;
        }
        case PARAM_FSTAGE_SOUND_MODEL_TYPE: {
            *(st_module_type_t *)param->data = GetModuleType(param->stream);
            break;
        }
        case PARAM_SOUND_MODEL_LIST: {
            void *s = param->stream;
            sound_model_list_t *sm_list = (sound_model_list_t *)param->data;
            if (sm_info_map_.find(s) != sm_info_map_.end() && sm_info_map_[s]) {
                for (int i = 0; i < sm_info_map_[s]->model_list.size(); i++) {
                    sm_list->sm_list.push_back(sm_info_map_[s]->model_list[i]);
                }
            } else {
                PAL_ERR(LOG_TAG, "stream not registered");
                status = -EINVAL;
            }
            break;
        }
        case PARAM_FSTAGE_BUFFERING_CONFIG: {
            struct buffer_config *config = (struct buffer_config *)param->data;
            GetBufferingConfigs(param->stream, config);
            break;
        }
        case PARAM_DETECTION_EVENT: {
            status = GenerateCallbackEvent(param->stream,
                (struct pal_st_recognition_event **)param->data, &param->size);
            break;
        }
        case PARAM_DETECTION_STREAM: {
            param->stream = GetDetectedStream();
            break;
        }
        case PARAM_SOUND_MODEL_LOAD:
            status = GetSoundModelLoadPayload(param);
            break;
        case PARAM_BUFFERING_CONFIG:
            status = GetBufferingPayload(param);
            break;
        default:
            PAL_ERR(LOG_TAG, "Unsupported param id %d", param_id);
            break;
    }

    return status;
}

int32_t HotwordInterface::ParseSoundModel(
    struct pal_st_sound_model *sound_model,
    std::vector<sound_model_data_t *> &model_list) {

    int32_t status = 0;
    struct pal_st_phrase_sound_model *phrase_sm = nullptr;
    struct pal_st_sound_model *common_sm = nullptr;
    uint8_t *ptr = nullptr;
    uint8_t *sm_data = nullptr;
    int32_t sm_size = 0;
    sound_model_data_t *model_data = nullptr;

    PAL_DBG(LOG_TAG, "Enter");

    if (sound_model->type == PAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        // handle for phrase sound model
        phrase_sm = (struct pal_st_phrase_sound_model *)sound_model;
        sm_size = phrase_sm->common.data_size;
        sm_data = (uint8_t *)calloc(1, sm_size);
        if (!sm_data) {
            PAL_ERR(LOG_TAG, "Failed to allocate memory for sm_data");
            status = -ENOMEM;
            goto error_exit;
        }
        ptr = (uint8_t*)phrase_sm + phrase_sm->common.data_offset;
        ar_mem_cpy(sm_data, sm_size, ptr, sm_size);

        model_data = (sound_model_data_t *)calloc(1, sizeof(sound_model_data_t));
        if (!model_data) {
            status = -ENOMEM;
            PAL_ERR(LOG_TAG, "model_data allocation failed, status %d",
                status);
            goto error_exit;
        }
        model_data->type = ST_SM_ID_SVA_F_STAGE_GMM;
        model_data->data = sm_data;
        model_data->size = sm_size;
        model_list.push_back(model_data);
    } else if (sound_model->type == PAL_SOUND_MODEL_TYPE_GENERIC) {
        // handle for generic sound model
        common_sm = sound_model;
        sm_size = common_sm->data_size;
        sm_data = (uint8_t *)calloc(1, sm_size);
        if (!sm_data) {
            PAL_ERR(LOG_TAG, "Failed to allocate memory for sm_data");
            status = -ENOMEM;
            goto error_exit;
        }
        ptr = (uint8_t*)common_sm + common_sm->data_offset;
        ar_mem_cpy(sm_data, sm_size, ptr, sm_size);

        model_data = (sound_model_data_t *)calloc(1, sizeof(sound_model_data_t));
        if (!model_data) {
            status = -ENOMEM;
            PAL_ERR(LOG_TAG, "model_data allocation failed, status %d",
                status);
            goto error_exit;
        }
        model_data->type = ST_SM_ID_SVA_F_STAGE_GMM;
        model_data->data = sm_data;
        model_data->size = sm_size;
        model_list.push_back(model_data);
    }
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;

error_exit:
    // clean up memory added to model_list in failure case
    for (int i = 0; i < model_list.size(); i++) {
        model_data = model_list[i];
        if (model_data) {
            if (model_data->data)
                free(model_data->data);
            free(model_data);
        }
    }
    model_list.clear();
    if (sm_data)
        free(sm_data);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t HotwordInterface::ParseRecognitionConfig(void *s,
    struct pal_st_recognition_config *config) {

    int32_t status = 0;
    struct sound_model_info *sm_info = nullptr;
    uint32_t hist_buffer_duration = 0;
    uint32_t pre_roll_duration = 0;

    if (sm_info_map_.find(s) != sm_info_map_.end() && sm_info_map_[s]) {
        sm_info = sm_info_map_[s];
        sm_info->rec_config = config;
    } else {
        PAL_ERR(LOG_TAG, "Stream not registered to interface");
        return -EINVAL;
    }

    PAL_DBG(LOG_TAG, "Enter");
    if (!config) {
        PAL_ERR(LOG_TAG, "Invalid config");
        status = -EINVAL;
        goto exit;
    }

    // get history buffer duration from sound trigger platform xml
    hist_buffer_duration = kw_duration_;
    pre_roll_duration = 0;

    sm_info_map_[s]->buf_config.hist_buffer_duration = hist_buffer_duration;
    sm_info_map_[s]->buf_config.pre_roll_duration = pre_roll_duration;

exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

void HotwordInterface::GetBufferingConfigs(void *s,
    struct buffer_config *config) {

    if (sm_info_map_.find(s) != sm_info_map_.end() && sm_info_map_[s]) {
        config->hist_buffer_duration = sm_info_map_[s]->buf_config.hist_buffer_duration;
        config->pre_roll_duration = sm_info_map_[s]->buf_config.pre_roll_duration;
    } else {
        PAL_ERR(LOG_TAG, "Stream not registered to interface");
    }
}

int32_t HotwordInterface::ParseDetectionPayload(void *event, uint32_t size) {
    int32_t status = 0;

    if (!event || size == 0) {
        PAL_ERR(LOG_TAG, "Invalid detection payload");
        return -EINVAL;
    }

    custom_event_ = (uint8_t *)realloc(custom_event_, size);
    if (!custom_event_) {
        PAL_ERR(LOG_TAG, "Failed to allocate memory for detection payload");
        return -ENOMEM;
    }

    ar_mem_cpy(custom_event_, size, event, size);
    custom_event_size_ = size;

    return status;
}

void* HotwordInterface::GetDetectedStream() {
    PAL_DBG(LOG_TAG, "Enter");
    if (sm_info_map_.empty()) {
        PAL_ERR(LOG_TAG, "Unexpected, No streams attached to engine!");
        return nullptr;
    } else if (sm_info_map_.size() == 1) {
        return sm_info_map_.begin()->first;
    } else {
        PAL_ERR(LOG_TAG, "Only single hotword usecase is supported");
        return nullptr;
    }
}

int32_t HotwordInterface::GenerateCallbackEvent(void *s,
    struct pal_st_recognition_event **event,
    uint32_t *size) {

    struct sound_model_info *sm_info = nullptr;
    struct pal_st_phrase_recognition_event *phrase_event = nullptr;
    struct pal_st_generic_recognition_event *generic_event = nullptr;
    size_t event_size = 0;
    uint8_t *opaque_data = nullptr;
    int32_t status = 0;

    if (sm_info_map_.find(s) != sm_info_map_.end() && sm_info_map_[s]) {
        sm_info = sm_info_map_[s];
    } else {
        PAL_ERR(LOG_TAG, "Stream not registered to interface");
        return -EINVAL;
    }

    PAL_DBG(LOG_TAG, "Enter");
    *event = nullptr;
    if (sm_info->type == PAL_SOUND_MODEL_TYPE_KEYPHRASE) {
        event_size = sizeof(struct pal_st_phrase_recognition_event) +
                     custom_event_size_;
        // event allocated will be used/deallocated in stream side
        phrase_event = (struct pal_st_phrase_recognition_event *)
                       calloc(1, event_size);
        if (!phrase_event) {
            PAL_ERR(LOG_TAG, "Failed to alloc memory for recognition event");
            status =  -ENOMEM;
            goto exit;
        }

        phrase_event->num_phrases = sm_info->rec_config->num_phrases;
        memcpy(phrase_event->phrase_extras, sm_info->rec_config->phrases,
               phrase_event->num_phrases *
               sizeof(struct pal_st_phrase_recognition_extra));
        *event = &(phrase_event->common);
        (*event)->status = sm_info->det_result ? PAL_RECOGNITION_STATUS_SUCCESS :
                           PAL_RECOGNITION_STATUS_FAILURE;
        (*event)->type = sm_info->type;
        (*event)->st_handle = (pal_st_handle_t *)this;
        (*event)->capture_available = sm_info->rec_config->capture_requested;
        // TODO: generate capture session
        (*event)->capture_session = 0;
        (*event)->capture_delay_ms = 0;
        (*event)->capture_preamble_ms = 0;
        (*event)->trigger_in_data = true;
        (*event)->data_size = custom_event_size_;
        (*event)->data_offset = sizeof(struct pal_st_phrase_recognition_event);
        (*event)->media_config.sample_rate =
            str_attr_.in_media_config.sample_rate;
        (*event)->media_config.bit_width =
            str_attr_.in_media_config.bit_width;
        (*event)->media_config.ch_info.channels =
            str_attr_.in_media_config.ch_info.channels;
        (*event)->media_config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S16_LE;

        // Filling Opaque data
        opaque_data = (uint8_t *)phrase_event +
                       phrase_event->common.data_offset;
        ar_mem_cpy(opaque_data, custom_event_size_, custom_event_, custom_event_size_);
    } else if (sm_info->type == PAL_SOUND_MODEL_TYPE_GENERIC) {
        event_size = sizeof(struct pal_st_generic_recognition_event) +
                     custom_event_size_;
        // event allocated will be used/deallocated in stream side
        generic_event = (struct pal_st_generic_recognition_event *)
                       calloc(1, event_size);
        if (!generic_event) {
            PAL_ERR(LOG_TAG, "Failed to alloc memory for recognition event");
            status =  -ENOMEM;
            goto exit;

        }

        *event = &(generic_event->common);
        (*event)->status = PAL_RECOGNITION_STATUS_SUCCESS;
        (*event)->type = sm_info->type;
        (*event)->st_handle = (pal_st_handle_t *)this;
        (*event)->capture_available = sm_info->rec_config->capture_requested;
        // TODO: generate capture session
        (*event)->capture_session = 0;
        (*event)->capture_delay_ms = 0;
        (*event)->capture_preamble_ms = 0;
        (*event)->trigger_in_data = true;
        (*event)->data_size = custom_event_size_;
        (*event)->data_offset = sizeof(struct pal_st_generic_recognition_event);
        (*event)->media_config.sample_rate =
            str_attr_.in_media_config.sample_rate;
        (*event)->media_config.bit_width =
            str_attr_.in_media_config.bit_width;
        (*event)->media_config.ch_info.channels =
            str_attr_.in_media_config.ch_info.channels;
        (*event)->media_config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S16_LE;

        // Filling Opaque data
        opaque_data = (uint8_t *)generic_event +
                       generic_event->common.data_offset;
        ar_mem_cpy(opaque_data, custom_event_size_, custom_event_, custom_event_size_);
    }
    *size = event_size;
exit:
    PAL_DBG(LOG_TAG, "Exit");
    return status;
}

void HotwordInterface::UpdateDetectionResult(void *s, uint32_t result) {
    if (sm_info_map_.find(s) != sm_info_map_.end() && sm_info_map_[s]) {
        sm_info_map_[s]->det_result = result;
    }
}

int32_t HotwordInterface::GetSoundModelLoadPayload(vui_intf_param_t *param) {
    void *s = nullptr;
    sound_model_data_t *sm_data = nullptr;

    if (!param) {
        PAL_ERR(LOG_TAG, "Invalid param");
        return -EINVAL;
    }

    s = param->stream;
    if (sm_info_map_.find(s) == sm_info_map_.end()) {
        PAL_DBG(LOG_TAG, "Stream not registered");
        return -EINVAL;
    }

    if (!sm_info_map_[s]) {
        PAL_ERR(LOG_TAG, "Invalid sound model info");
        return -EINVAL;
    }

    sm_data = sm_info_map_[s]->model_list[0];
    param->data = (void *)sm_data->data;
    param->size = sm_data->size;

    return 0;
}

int32_t HotwordInterface::GetBufferingPayload(vui_intf_param_t *param) {
    void *s = nullptr;
    struct sound_model_info *info = nullptr;

    if (!param) {
        PAL_ERR(LOG_TAG, "Invalid param");
        return -EINVAL;
    }

    s = param->stream;
    if (sm_info_map_.find(s) == sm_info_map_.end()) {
        PAL_DBG(LOG_TAG, "Stream not registered");
        return -EINVAL;
    }

    info = sm_info_map_[s];
    if (!info) {
        PAL_ERR(LOG_TAG, "Invalid sound model info");
        return -EINVAL;
    }

    buffering_config_.hist_buffer_duration_msec =
        info->buf_config.hist_buffer_duration;

    buffering_config_.pre_roll_duration_msec =
        info->buf_config.pre_roll_duration;

    param->data = (void *)&buffering_config_;
    param->size = sizeof(buffering_config_);

    return 0;
}

void HotwordInterface::SetStreamAttributes(
    struct pal_stream_attributes *attr) {

    if (!attr) {
        PAL_ERR(LOG_TAG, "Invalid stream attributes");
        return;
    }

    ar_mem_cpy(&str_attr_, sizeof(struct pal_stream_attributes),
        attr, sizeof(struct pal_stream_attributes));
}

int32_t HotwordInterface::RegisterModel(void *s,
    struct pal_st_sound_model *model,
    const std::vector<sound_model_data_t *> model_list) {

    int32_t status = 0;

    if (sm_info_map_.find(s) == sm_info_map_.end()) {
        sm_info_map_[s] = (struct sound_model_info *)calloc(1,
            sizeof(struct sound_model_info));
        if (!sm_info_map_[s]) {
            PAL_ERR(LOG_TAG, "Failed to allocate memory for sm data");
            status = -ENOMEM;
            goto exit;
        }
    }

    sm_info_map_[s]->model_list.clear();
    for (auto iter: model_list) {
        sm_info_map_[s]->model_list.push_back(iter);
    }
    sm_info_map_[s]->model = model;
    sm_info_map_[s]->type = model->type;

exit:
    return status;
}

void HotwordInterface::DeregisterModel(void *s) {
    sound_model_data_t *sm_data = nullptr;

    auto iter = sm_info_map_.find(s);
    if (iter != sm_info_map_.end() && sm_info_map_[s]) {
        for (int i = 0; i < sm_info_map_[s]->model_list.size(); i++) {
            sm_data = sm_info_map_[s]->model_list[i];
            if (sm_data) {
                if (sm_data->data)
                    free(sm_data->data);
                free(sm_data);
            }
        }
        sm_info_map_[s]->model_list.clear();
        free(sm_info_map_[s]);
        sm_info_map_.erase(iter);
    } else {
        PAL_DBG(LOG_TAG, "Cannot deregister unregistered model")
    }
}

uint32_t HotwordInterface::UsToBytes(uint64_t input_us) {
    uint32_t bytes = 0;

    bytes = str_attr_.in_media_config.sample_rate *
            str_attr_.in_media_config.bit_width *
            str_attr_.in_media_config.ch_info.channels * input_us /
            (BITS_PER_BYTE * US_PER_SEC);

    return bytes;
}
