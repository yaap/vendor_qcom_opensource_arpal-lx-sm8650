/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef SVA_INTERFACE_H
#define SVA_INTERFACE_H

#include "VoiceUIInterface.h"
#include "SoundTriggerEngine.h"

class SVAInterface: public VoiceUIInterface {
  public:
    SVAInterface(std::shared_ptr<VUIStreamConfig> sm_cfg);
    ~SVAInterface();

    static int32_t ParseSoundModel(std::shared_ptr<VUIStreamConfig> sm_cfg,
                                   struct pal_st_sound_model *sound_model,
                                   st_module_type_t &first_stage_type,
                                   std::vector<sm_pair_t> &model_list);

    int32_t ParseRecognitionConfig(Stream *s,
                                   struct pal_st_recognition_config *config) override;

    void GetWakeupConfigs(Stream *s,
                          void **config, uint32_t *size) override;
    void GetBufferingConfigs(Stream *s,
                             uint32_t *hist_duration,
                             uint32_t *preroll_duration) override;
    void GetSecondStageConfLevels(Stream *s,
                                  listen_model_indicator_enum type,
                                  uint32_t *level) override;

    void SetSecondStageDetLevels(Stream *s,
                                 listen_model_indicator_enum type,
                                 uint32_t level) override;

    int32_t ParseDetectionPayload(Stream *s, void *event, uint32_t size) override;
    Stream* GetDetectedStream(void *event) override;
    void* GetDetectionEventInfo(Stream *s) override;
    void GetKeywordIndex(Stream *s, uint32_t *start_index, uint32_t *end_index) override;
    void GetKeywordStats(Stream *s, uint64_t *start_ts, uint64_t *end_ts,
                         uint64_t *ftrt_duration) override;
    void UpdateIndices(Stream * s, uint32_t start_idx, uint32_t end_idx) override;
    int32_t GenerateCallbackEvent(Stream *s,
                                  struct pal_st_recognition_event **event,
                                  uint32_t *event_size, bool detection) override;

    void ProcessLab(void *data, uint32_t size) {}

    void UpdateFTRTData(void *data, uint32_t size) {}

    bool IsQCWakeUpConfigUsed() { return true; }

    int32_t UpdateEngineModel(Stream *s,
                              uint8_t *data,
                              uint32_t data_size,
                              struct detection_engine_config_voice_wakeup *wakeup_config,
                              bool add) override;
    int32_t UpdateMergeConfLevelsPayload(SoundModelInfo* src_sm_info,
                                         bool set) override;

  protected:
    int32_t ParseOpaqueConfLevels(struct sound_model_info *info,
                                  void *opaque_conf_levels,
                                  uint32_t version,
                                  uint8_t **out_conf_levels,
                                  uint32_t *out_num_conf_levels);
    int32_t FillConfLevels(struct sound_model_info *info,
                           struct pal_st_recognition_config *config,
                           uint8_t **out_conf_levels,
                           uint32_t *out_num_conf_levels);
    int32_t FillOpaqueConfLevels(uint32_t model_id,
                                 const void *sm_levels_generic,
                                 uint8_t **out_payload,
                                 uint32_t *out_payload_size,
                                 uint32_t version);
    int32_t ParseDetectionPayloadPDK(Stream *s, void *event_data);
    int32_t ParseDetectionPayloadGMM(Stream *s, void *event_data);
    void UpdateKeywordIndex(Stream *s, uint64_t kwd_start_timestamp,
                            uint64_t kwd_end_timestamp,
                            uint64_t ftrt_start_timestamp);
    void PackEventConfLevels(struct sound_model_info *sm_info,
                             uint8_t *opaque_data);
    void FillCallbackConfLevels(struct sound_model_info *sm_info,
                                uint8_t *opaque_data,
                                uint32_t det_keyword_id,
                                uint32_t best_conf_level);
    void CheckAndSetDetectionConfLevels(Stream *s);
    Stream* GetPDKDetectedStream(void *event);
    Stream* GetGMMDetectedStream(void *event);

    int32_t AddSoundModel(Stream *s,
                          uint8_t *data,
                          uint32_t data_size);
    int32_t DeleteSoundModel(Stream *s,
                             struct detection_engine_config_voice_wakeup *wakeup_config);
    int32_t QuerySoundModel(SoundModelInfo *sm_info,
                            uint8_t *data,
                            uint32_t data_size);
    int32_t MergeSoundModels(uint32_t num_models,
                             listen_model_type *in_models[],
                             listen_model_type *out_model);
    int32_t DeleteFromMergedModel(char **keyphrases,
                                  uint32_t num_keyphrases,
                                  listen_model_type *in_model,
                                  listen_model_type *out_model);
    int32_t UpdateMergeConfLevelsWithActiveStreams();

    uint32_t conf_levels_intf_version_;
    st_confidence_levels_info *st_conf_levels_;
    st_confidence_levels_info_v2 *st_conf_levels_v2_;


  private:
    bool sm_merged_;
    struct detection_event {
        union {
            struct detection_event_info event_info_;
            struct detection_event_info_pdk pdk_event_info_;
        };
        uint32_t det_model_id_;
        uint64_t ftrt_size_us_;
        uint64_t start_ts_;
        uint64_t end_ts_;
        uint32_t start_index_;
        uint32_t end_index_;
    };
    std::map<Stream*, struct detection_event*> det_event_info_;
};

#endif
