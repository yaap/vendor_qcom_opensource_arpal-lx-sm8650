/*
 * Copyright (c) 2026 The YAAP Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "PAL: OplusSpeakerDevice"

#include "OplusSpeakerDevice.h"
#include "OplusSpeakerSia.h"
#include "OplusSpeakerTfa98xx.h"

std::shared_ptr<OplusSpeakerDevice> OplusSpeakerDevice::getInstance(int type)
{
    switch (type) {
        case OPLUS_SPEAKER_PA_TFA98XX:
            return OplusSpeakerTfa98xx::getInstance();
        case OPLUS_SPEAKER_PA_SIA:
            return OplusSpeakerSia::getInstance();
        default:
            return nullptr;
    }
}

std::shared_ptr<OplusSpeakerDevice> OplusSpeakerDevice::getObject(int type)
{
    switch (type) {
        case OPLUS_SPEAKER_PA_TFA98XX:
            return OplusSpeakerTfa98xx::getObject();
        case OPLUS_SPEAKER_PA_SIA:
            return OplusSpeakerSia::getObject();
        default:
            return nullptr;
    }
}
