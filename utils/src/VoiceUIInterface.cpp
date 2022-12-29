/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "PAL: VoiceUIInterface"

#include "VoiceUIInterface.h"
#include "SVAInterface.h"
#include "HotwordInterface.h"
#include "CustomVAInterface.h"
#include "PalCommon.h"

int32_t GetVUIInterface(struct vui_intf_t *intf,
    vui_intf_param_t *model) {

    int32_t status = 0;
    sound_model_config_t *config = nullptr;

    if (!intf || !model || !model->data)
        return -EINVAL;

    config = (sound_model_config_t *)model->data;
    switch (*config->module_type) {
        case ST_MODULE_TYPE_GMM:
        case ST_MODULE_TYPE_PDK:
            intf->interface = SVAInterface::Init(model);
            break;
        case ST_MODULE_TYPE_HW:
            intf->interface = HotwordInterface::Init(model);
            break;
        case ST_MODULE_TYPE_CUSTOM_1:
        case ST_MODULE_TYPE_CUSTOM_2:
            intf->interface = CustomVAInterface::Init(model);
            break;
        default:
            break;
    }

    if (!intf->interface) {
        PAL_ERR(LOG_TAG, "Failed to create VoiceUI Interface");
        status = -EINVAL;
    }

    return status;
}
