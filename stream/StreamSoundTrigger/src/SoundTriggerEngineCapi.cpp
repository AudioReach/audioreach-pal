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
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef ATRACE_UNSUPPORTED
#define ATRACE_TAG (ATRACE_TAG_AUDIO | ATRACE_TAG_HAL)
#endif
#define LOG_TAG "PAL: SoundTriggerEngineCapi"

#include "SoundTriggerEngineCapi.h"
#ifdef PAL_CUTILS_SUPPORTED
#include <cutils/trace.h>
#endif
#include <dlfcn.h>

#include "StreamSoundTrigger.h"
#include "SoundTriggerPlatformInfo.h"
#include "VoiceUIInterface.h"
#include "SoundTriggerUtils.h"

#define CNN_BUFFER_LENGTH 10000
#define CNN_FRAME_SIZE 320
#define MAX_UV_POST_TOLERANCE_US 2000000

ST_DBG_DECLARE(static int keyword_detection_cnt = 0);
ST_DBG_DECLARE(static int user_verification_cnt = 0);

void SoundTriggerEngineCapi::BufferThreadLoop(
    SoundTriggerEngineCapi *capi_engine)
{
    StreamSoundTrigger *s = nullptr;
    int32_t status = 0;
    int32_t detection_state = ENGINE_IDLE;

    PAL_DBG(LOG_TAG, "Enter");
    if (!capi_engine) {
        PAL_ERR(LOG_TAG, "Invalid sound trigger capi engine");
        return;
    }

    std::unique_lock<std::mutex> lck(capi_engine->event_mutex_);
    while (!capi_engine->exit_thread_) {
        PAL_VERBOSE(LOG_TAG, "waiting on cond, processing started  = %d",
                    capi_engine->processing_started_);
        // Wait for keyword buffer data from DSP
        if (!capi_engine->processing_started_)
            capi_engine->cv_.wait(lck);
        PAL_VERBOSE(LOG_TAG, "done waiting on cond, exit buffering = %d",
                    capi_engine->exit_buffering_);

        if (capi_engine->exit_thread_) {
            break;
        }

        /*
         * If 1st stage buffering overflows before 2nd stage starts processing,
         * the below functions need to be called to reset the 1st stage session
         * for the next detection. We might be able to check states of the engine
         * to avoid this buffering flag.
         */
        if (capi_engine->exit_buffering_) {
            continue;  // skip over processing if we want to exit already
        }

        if (capi_engine->processing_started_) {
            s = capi_engine->stream_handle_;
            capi_engine->sec_kw_detect_state_ = 0;
            if (capi_engine->detection_type_ ==
                ST_SM_TYPE_KEYWORD_DETECTION) {
                status = capi_engine->StartKeywordDetection();
                /*
                 * StreamSoundTrigger may call stop recognition to second stage
                 * engines when one of the second stage engine reject detection.
                 * So check processing_started_ before notify stream in case
                 * stream has already stopped recognition.
                 */
                if (capi_engine->processing_started_) {
                    if (status)
                        detection_state = KEYWORD_DETECTION_REJECT;
                    else
                        detection_state = capi_engine->detection_state_;
                    lck.unlock();
                    s->SetEngineDetectionState(detection_state);
                    lck.lock();
                }
            } else if (capi_engine->detection_type_ ==
                ST_SM_TYPE_USER_VERIFICATION) {
                if (capi_engine->engine_type_ & ST_SM_ID_SVA_S_STAGE_USER) {
                    status = capi_engine->StartUserVerification(lck);
                } else if (capi_engine->engine_type_ & ST_SM_ID_SVA_S_STAGE_CTIUV) {
                    status = capi_engine->StartTIUserVerification();
                }
                /*
                 * StreamSoundTrigger may call stop recognition to second stage
                 * engines when one of the second stage engine reject detection.
                 * So check processing_started_ before notify stream in case
                 * stream has already stopped recognition.
                 */
                if (capi_engine->processing_started_) {
                    if (status)
                        detection_state = USER_VERIFICATION_REJECT;
                    else
                        detection_state = capi_engine->detection_state_;
                    lck.unlock();
                    s->SetEngineDetectionState(detection_state);
                    lck.lock();
                }
            }
            capi_engine->detection_state_ = ENGINE_IDLE;
            capi_engine->keyword_detected_ = false;
            capi_engine->processing_started_ = false;
        }
    }
    PAL_DBG(LOG_TAG, "Exit");
}

int32_t SoundTriggerEngineCapi::StartKeywordDetection()
{
    int32_t status = 0;
    char *process_input_buff = nullptr;
    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_stream_data_t *stream_input = nullptr;
    sva_result_t *result_cfg_ptr = nullptr;
    int32_t read_size = 0;
    capi_v2_buf_t capi_result;
    size_t lab_buffer_size = 0;
    bool first_buffer_processed = false;
    FILE *keyword_detection_fd = nullptr;
    ChronoSteadyClock_t process_start;
    ChronoSteadyClock_t process_end;
    ChronoSteadyClock_t capi_call_start;
    ChronoSteadyClock_t capi_call_end;
    uint64_t process_duration = 0;
    uint64_t total_capi_process_duration = 0;
    uint64_t total_capi_get_param_duration = 0;
    uint32_t start_idx = 0, end_idx = 0;
    uint32_t ftrt_sz = 0, read_offset = 0;
    uint32_t max_processing_sz = 0, processed_sz = 0;
    vui_intf_param_t param;
    struct keyword_index kw_index {};
    int32_t det_conf_score = 0;

    PAL_DBG(LOG_TAG, "Enter");
    if (!reader_ || !reader_->isEnabled()) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid ring buffer reader");
        goto exit;
    }

    reader_->getIndices(stream_handle_, &start_idx, &end_idx, &ftrt_sz);
    PAL_INFO(LOG_TAG, "start index %d end index %d ", start_idx, end_idx);
    if (start_idx >= end_idx) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid keyword indices");
        goto exit;
    }

    if (start_idx > UsToBytes(kw_start_tolerance_)) {
        read_offset = start_idx - UsToBytes(kw_start_tolerance_);
        ftrt_sz = (end_idx - start_idx) + UsToBytes(kw_start_tolerance_);
        max_processing_sz = (end_idx - start_idx) +
            UsToBytes(kw_start_tolerance_ + kw_end_tolerance_ + data_after_kw_end_);
        reader_->advanceReadOffset(read_offset);
    } else {
        /*
         * When there's no enough data before kw start,
         * just read data from beginning of the detection
         */
        max_processing_sz = end_idx +
            UsToBytes(kw_end_tolerance_ + data_after_kw_end_);
        ftrt_sz = end_idx;
    }
    /*
     * As per requirement in PDK, input buffer size for
     * second stage should be in multiple of 10 ms(10000us).
     */
    ftrt_sz -= ftrt_sz % (UsToBytes(10000));

    lab_buffer_size = buffer_size_;
    buffer_size_ = ftrt_sz;

    if (vui_ptfm_info_->GetEnableDebugDumps()) {
        ST_DBG_FILE_OPEN_WR(keyword_detection_fd, ST_DEBUG_DUMP_LOCATION,
            "keyword_detection", "bin", keyword_detection_cnt);
        PAL_DBG(LOG_TAG, "keyword detection data stored in: keyword_detection_%d.bin",
            keyword_detection_cnt);
        keyword_detection_cnt++;
    }

    memset(&capi_result, 0, sizeof(capi_result));
    process_input_buff = (char*)calloc(1, buffer_size_);
    if (!process_input_buff) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "failed to allocate process input buff, status %d",
                status);
        goto exit;
    }

    stream_input = (capi_v2_stream_data_t *)
                   calloc(1, sizeof(capi_v2_stream_data_t));
    if (!stream_input) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "failed to allocate stream input, status %d", status);
        goto exit;
    }

    stream_input->buf_ptr = (capi_v2_buf_t*)calloc(1, sizeof(capi_v2_buf_t));
    if (!stream_input->buf_ptr) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "failed to allocate stream_input->buf_ptr, status %d",
                status);
        goto exit;
    }

    result_cfg_ptr = (sva_result_t*)calloc(1, sizeof(sva_result_t));
    if (!result_cfg_ptr) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "failed to allocate result cfg ptr status %d", status);
        goto exit;
    }

    process_start = std::chrono::steady_clock::now();
    while (!exit_buffering_ && (processed_sz < max_processing_sz)) {
        if (!reader_->waitForBuffers(buffer_size_))
            continue;

        read_size = reader_->read((void*)process_input_buff, buffer_size_);
        if (read_size == 0) {
            continue;
        } else if (read_size < 0) {
            status = read_size;
            PAL_ERR(LOG_TAG, "Failed to read from buffer, status %d", status);
            goto exit;
        }

        stream_input->bufs_num = 1;
        stream_input->buf_ptr->max_data_len = buffer_size_;
        stream_input->buf_ptr->actual_data_len = read_size;
        stream_input->buf_ptr->data_ptr = (int8_t *)process_input_buff;

        if (vui_ptfm_info_->GetEnableDebugDumps()) {
            ST_DBG_FILE_WRITE(keyword_detection_fd,
                process_input_buff, read_size);
        }

        PAL_VERBOSE(LOG_TAG, "Calling Capi Process");
        capi_call_start = std::chrono::steady_clock::now();
#ifndef ATRACE_UNSUPPORTED
        ATRACE_BEGIN("Second stage KW process");
#endif
        rc = capi_handle_->vtbl_ptr->process(capi_handle_,
            &stream_input, nullptr);
#ifndef ATRACE_UNSUPPORTED
        ATRACE_END();
#endif
        capi_call_end = std::chrono::steady_clock::now();
        total_capi_process_duration +=
            std::chrono::duration_cast<std::chrono::milliseconds>(
                capi_call_end - capi_call_start).count();
        if (CAPI_V2_EFAILED == rc) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "capi process failed, status %d", status);
            goto exit;
        }

        processed_sz += read_size;

        capi_result.data_ptr = (int8_t*)result_cfg_ptr;
        capi_result.actual_data_len = sizeof(sva_result_t);
        capi_result.max_data_len = sizeof(sva_result_t);

        PAL_VERBOSE(LOG_TAG, "Calling Capi get param for status");
        capi_call_start = std::chrono::steady_clock::now();
        rc = capi_handle_->vtbl_ptr->get_param(capi_handle_,
            SVA_ID_RESULT, nullptr, &capi_result);
        capi_call_end = std::chrono::steady_clock::now();
        total_capi_get_param_duration +=
            std::chrono::duration_cast<std::chrono::milliseconds>(
                capi_call_end - capi_call_start).count();
        if (CAPI_V2_EFAILED == rc) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "capi get param failed, status %d", status);
            goto exit;
        }

        det_conf_score = result_cfg_ptr->best_confidence;
        if (result_cfg_ptr->is_detected) {
            exit_buffering_ = true;
            detection_state_ = KEYWORD_DETECTION_SUCCESS;
            start_idx = result_cfg_ptr->start_position * CNN_FRAME_SIZE + read_offset;
            end_idx = result_cfg_ptr->end_position * CNN_FRAME_SIZE + read_offset;
            param.stream = (void *)stream_handle_;
            param.data = (void *)&det_conf_score;
            param.size = sizeof(int32_t);
            vui_intf_->SetParameter(PARAM_SSTAGE_KW_DET_LEVEL, &param);

            kw_index.start_index = start_idx;
            kw_index.end_index = end_idx;
            param.data = (void *)&kw_index;
            param.size = sizeof(struct keyword_index);
            vui_intf_->SetParameter(PARAM_KEYWORD_INDEX, &param);
            PAL_INFO(LOG_TAG, "KWD Second Stage Detected, start index %u, end index %u",
                start_idx, end_idx);
        } else if (processed_sz >= max_processing_sz) {
            detection_state_ = KEYWORD_DETECTION_REJECT;
            param.stream = (void *)stream_handle_;
            param.data = (void *)&det_conf_score;
            param.size = sizeof(int32_t);
            vui_intf_->SetParameter(PARAM_SSTAGE_KW_DET_LEVEL, &param);
            PAL_INFO(LOG_TAG, "KWD Second Stage rejected");
        }
        PAL_INFO(LOG_TAG, "KWD second stage conf level %d, processed %u bytes",
            det_conf_score, processed_sz);

        if (!first_buffer_processed) {
            buffer_size_ = lab_buffer_size;
            first_buffer_processed = true;
        }
    }

exit:

    PAL_INFO(LOG_TAG, "Issuing capi_set_param for param %d",
                   SVA_ID_REINIT_ALL);
    rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
                             SVA_ID_REINIT_ALL, nullptr, nullptr);

    if (CAPI_V2_EOK != rc) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "set param SVA_ID_REINIT_ALL failed, status = %d",
                rc);
    }

    process_end = std::chrono::steady_clock::now();
    process_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        process_end - process_start).count();
    PAL_INFO(LOG_TAG, "KW processing time: Bytes processed %u, Total processing "
        "time %llums, Algo process time %llums, get result time %llums",
        processed_sz, (long long)process_duration,
        (long long)total_capi_process_duration,
        (long long)total_capi_get_param_duration);
    if (vui_ptfm_info_->GetEnableDebugDumps()) {
        ST_DBG_FILE_CLOSE(keyword_detection_fd);
    }

    if (sm_cfg_->IsDetPropSupported(ST_PARAM_KEY_SSTAGE_KW_ENGINE_INFO)) {
        struct st_det_engine_stats engine_info;
        engine_info.version = 0x1;
        engine_info.detection_state = detection_state_;
        engine_info.processed_length =
            BytesToFrames(processed_sz) * MS_PER_SEC / sample_rate_;
        engine_info.total_process_duration = process_duration;
        engine_info.total_capi_process_duration = total_capi_process_duration;
        engine_info.total_capi_get_param_duration = total_capi_get_param_duration;
        param.stream = (void *)stream_handle_;
        param.data = (void *)&engine_info;
        param.size = sizeof(struct st_det_engine_stats);
        vui_intf_->SetParameter(PARAM_SSTAGE_KW_DET_STATS, &param);
    }

    if (reader_)
        reader_->updateState(READER_DISABLED);

    if (process_input_buff)
        free(process_input_buff);
    if (stream_input) {
        if (stream_input->buf_ptr)
            free(stream_input->buf_ptr);
        free(stream_input);
    }
    if (result_cfg_ptr)
        free(result_cfg_ptr);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapi::StartUserVerification(
    std::unique_lock<std::mutex> &lck)
{
    int32_t status = 0;
    char *process_input_buff = nullptr;
    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_stream_data_t *stream_input = nullptr;
    capi_v2_buf_t capi_uv_ptr;
    stage2_uv_wrapper_result *result_cfg_ptr = nullptr;
    stage2_uv_wrapper_stage1_uv_score_t *uv_cfg_ptr = nullptr;
    int32_t read_size = 0;
    capi_v2_buf_t capi_result;
    FILE *user_verification_fd = nullptr;
    ChronoSteadyClock_t process_start;
    ChronoSteadyClock_t process_end;
    ChronoSteadyClock_t capi_call_start;
    ChronoSteadyClock_t capi_call_end;
    uint64_t process_duration = 0;
    uint64_t total_capi_process_duration = 0;
    uint64_t total_capi_get_param_duration = 0;
    uint32_t start_idx = 0, end_idx = 0;
    uint32_t ftrt_sz = 0, read_offset = 0;
    uint32_t max_processing_sz = 0, processed_sz = 0;
    st_module_type_t fstage_module_type;
    vui_intf_param_t param = {};
    int32_t det_conf_score = 0;
    uint32_t process_count = 0, max_process_count = 0;
    uint32_t sec_kw_end_index = 0;
    uint32_t write_offset = 0, size_to_read = 0;
    uint32_t fstage_est_proc_sz = 0, sstage_est_proc_sz = 0;
    bool is_post_tolerance_processed = false;

    PAL_DBG(LOG_TAG, "Enter");

    status = UpdateUVScratchParam();
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to update UV scratch param");
        goto exit;
    }

    if (!reader_ || !reader_->isEnabled()) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid ring buffer reader");
        goto exit;
    }

    reader_->getIndices(stream_handle_, &start_idx, &end_idx, &ftrt_sz);
    PAL_INFO(LOG_TAG, "start index %u end index %u ", start_idx, end_idx);
    if (start_idx >= end_idx) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid keyword indices");
        goto exit;
    }

    if (start_idx > UsToBytes(data_before_kw_start_)) {
        read_offset = start_idx - UsToBytes(data_before_kw_start_);
        /* advance the offset to ensure we are reading at the right place */
        reader_->advanceReadOffset(read_offset);
    }
    fstage_est_proc_sz = end_idx + UsToBytes(kw_end_tolerance_) - read_offset;
    PAL_INFO(LOG_TAG, "processing size %u", fstage_est_proc_sz);

    if (vui_ptfm_info_->GetEnableDebugDumps()) {
        ST_DBG_FILE_OPEN_WR(user_verification_fd, ST_DEBUG_DUMP_LOCATION,
            "user_verification", "bin", user_verification_cnt);
        PAL_DBG(LOG_TAG, "User Verification data stored in: user_verification_%d.bin",
            user_verification_cnt);
        user_verification_cnt++;
    }

    memset(&capi_uv_ptr, 0, sizeof(capi_uv_ptr));
    memset(&capi_result, 0, sizeof(capi_result));

    max_processing_sz = end_idx + UsToBytes(kw_end_tolerance_) +
        UsToBytes(MAX_UV_POST_TOLERANCE_US);
    process_input_buff = (char*)calloc(1, max_processing_sz);
    if (!process_input_buff) {
        PAL_ERR(LOG_TAG, "failed to allocate process input buff");
        status = -ENOMEM;
        goto exit;
    }

    stream_input = (capi_v2_stream_data_t *)
                   calloc(1, sizeof(capi_v2_stream_data_t));
    if (!stream_input) {
        PAL_ERR(LOG_TAG, "failed to allocate stream input");
        status = -ENOMEM;
        goto exit;
    }

    stream_input->buf_ptr = (capi_v2_buf_t*)calloc(1, sizeof(capi_v2_buf_t));
    if (!stream_input->buf_ptr) {
        PAL_ERR(LOG_TAG, "failed to allocate buf ptr");
        status = -ENOMEM;
        goto exit;
    }

    result_cfg_ptr = (stage2_uv_wrapper_result*)
                     calloc(1, sizeof(stage2_uv_wrapper_result));
    if (!result_cfg_ptr) {
        PAL_ERR(LOG_TAG, "failed to allocate result cfg ptr");
        status = -ENOMEM;
        goto exit;
    }

    uv_cfg_ptr = (stage2_uv_wrapper_stage1_uv_score_t *)
                 calloc(1, sizeof(stage2_uv_wrapper_stage1_uv_score_t));
    if (!uv_cfg_ptr) {
        PAL_ERR(LOG_TAG, "failed to allocate uv cfg ptr");
        status = -ENOMEM;
        goto exit;
    }

    param.stream = (void *)stream_handle_;
    param.data = (void *)&fstage_module_type;
    param.size = sizeof(st_module_type_t);
    vui_intf_->GetParameter(PARAM_FSTAGE_SOUND_MODEL_TYPE, &param);
    if (fstage_module_type == ST_MODULE_TYPE_GMM) {
        param.data = (void *)&uv_cfg_ptr->stage1_uv_score;
        param.size = sizeof(int32_t);
        status = vui_intf_->GetParameter(PARAM_FSTAGE_DETECTION_UV_SCORE, &param);
        if (status) {
            PAL_ERR(LOG_TAG, "Failed to get uv score from first stage");
            goto exit;
        }

        capi_uv_ptr.data_ptr = (int8_t *)uv_cfg_ptr;
        capi_uv_ptr.actual_data_len = sizeof(stage2_uv_wrapper_stage1_uv_score_t);
        capi_uv_ptr.max_data_len = sizeof(stage2_uv_wrapper_stage1_uv_score_t);

        PAL_VERBOSE(LOG_TAG, "Issuing capi_set_param for param %d",
                    STAGE2_UV_WRAPPER_ID_SVA_UV_SCORE);
        rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
            STAGE2_UV_WRAPPER_ID_SVA_UV_SCORE, nullptr, &capi_uv_ptr);
        if (CAPI_V2_EOK != rc) {
            PAL_ERR(LOG_TAG, "set param STAGE2_UV_WRAPPER_ID_SVA_UV_SCORE failed with %d",
                    rc);
            status = -EINVAL;
            goto reinit;
        }
    }

    /*
     * For Second stage UV processing, we have following options
     * as input buffer end for process:
     *     1. first stage kw end index + post tolerance
     *     2. second stage kw end index
     * When UV processing starts, wait until there's enough data
     * equivalent to one of above options and then starts UV process
     * with that amount of data. If UV is detected, then populate
     * detection result and send it to stream. If UV is rejected,
     * then wait again until there's enough data equivalent to the
     * rest option and then starts UV process again. After process
     * done, send detection result to stream.
     */
    max_process_count =
        stream_handle_->isModelLoaded(ST_SM_ID_SVA_S_STAGE_PDK) ? 2 : 1;
    process_start = std::chrono::steady_clock::now();
    while (!exit_buffering_ && (process_count <= max_process_count)) {
        if (process_count == max_process_count) {
            detection_state_ = USER_VERIFICATION_REJECT;
            param.stream = (void *)stream_handle_;
            param.data = (void *)&det_conf_score;
            param.size = sizeof(int32_t);
            vui_intf_->SetParameter(PARAM_SSTAGE_UV_DET_LEVEL ,&param);
            PAL_INFO(LOG_TAG, "UV Second Stage Rejected");
            goto reinit;
        }

        /* update if second stage KW detected/rejected
         * or post-tolerance data is ready
         */
        if (sec_kw_detect_state_ == KEYWORD_DETECTION_SUCCESS) {
            sec_kw_detect_state_ = 0;
            param.stream = stream_handle_;
            status = vui_intf_->GetParameter(PARAM_KEYWORD_INDEX, &param);
            if (!param.data) {
                PAL_ERR(LOG_TAG, "Failed to get kw index");
                process_count++;
                continue;
            }
            sec_kw_end_index = ((keyword_index *)param.data)->end_index;
            if (sec_kw_end_index > read_offset) {
                sstage_est_proc_sz = sec_kw_end_index - read_offset;
            } else {
                PAL_ERR(LOG_TAG, "Invalid second stage kw end index %d",
                    sec_kw_end_index);
                process_count++;
                continue;
            }
            size_to_read = sstage_est_proc_sz;
        } else if (sec_kw_detect_state_ == KEYWORD_DETECTION_REJECT) {
            sec_kw_detect_state_ = 0;
            break;
        } else if (!is_post_tolerance_processed &&
                   (reader_->getUnreadSize() + processed_sz >=
                    fstage_est_proc_sz)) {
            size_to_read = fstage_est_proc_sz;
            is_post_tolerance_processed = true;
        } else {
            size_to_read = 0;
            // wait for 10ms when UV process is not ready to start
            cv_.wait_for(lck, std::chrono::microseconds(CNN_BUFFER_LENGTH));
            continue;
        }

        if (processed_sz < size_to_read) {
            size_to_read -= processed_sz;
            read_size = reader_->read(
                (uint8_t*)process_input_buff + write_offset, size_to_read);
            if (read_size == 0) {
                continue;
            } else if (read_size < 0) {
                status = read_size;
                PAL_ERR(LOG_TAG, "Failed to read from buffer, status %d", status);
                goto reinit;
            }
            write_offset += read_size;
            stream_input->bufs_num = 1;
            stream_input->buf_ptr->max_data_len = max_processing_sz;
            stream_input->buf_ptr->actual_data_len = write_offset;
            stream_input->buf_ptr->data_ptr = (int8_t *)process_input_buff;
            processed_sz += read_size;
        } else if (processed_sz > size_to_read) { // unlikely to happen, just add in case
            stream_input->bufs_num = 1;
            stream_input->buf_ptr->max_data_len = max_processing_sz;
            stream_input->buf_ptr->actual_data_len = size_to_read;
            stream_input->buf_ptr->data_ptr = (int8_t *)process_input_buff;
        } else {
            /* when sstage kw end idx = fstage end idx + post tolerance,
             * no need to process with same size again, just skip the process.
             */
            PAL_INFO(LOG_TAG, "Same process size with first iteration, skip");
            process_count++;
            continue;
        }

        if (vui_ptfm_info_->GetEnableDebugDumps()) {
            ST_DBG_FILE_WRITE(user_verification_fd,
                process_input_buff, read_size);
        }

        PAL_VERBOSE(LOG_TAG, "Calling Capi Process\n");
        capi_call_start = std::chrono::steady_clock::now();
#ifndef ATRACE_UNSUPPORTED
        ATRACE_BEGIN("Second stage uv process");
#endif
        rc = capi_handle_->vtbl_ptr->process(capi_handle_,
            &stream_input, nullptr);
        process_count++;
#ifndef ATRACE_UNSUPPORTED
        ATRACE_END();
#endif
        capi_call_end = std::chrono::steady_clock::now();
        total_capi_process_duration +=
            std::chrono::duration_cast<std::chrono::milliseconds>(
                capi_call_end - capi_call_start).count();
        if (rc != CAPI_V2_EOK) {
            PAL_ERR(LOG_TAG, "capi process failed with %d", rc);
            status = -EINVAL;
            goto reinit;
        }

        capi_result.data_ptr = (int8_t*)result_cfg_ptr;
        capi_result.actual_data_len = sizeof(stage2_uv_wrapper_result);
        capi_result.max_data_len = sizeof(stage2_uv_wrapper_result);

        PAL_VERBOSE(LOG_TAG, "Calling Capi get param for result\n");
        capi_call_start = std::chrono::steady_clock::now();
#ifndef ATRACE_UNSUPPORTED
        ATRACE_BEGIN("Second stage uv get result");
#endif
        rc = capi_handle_->vtbl_ptr->get_param(capi_handle_,
            STAGE2_UV_WRAPPER_ID_RESULT, nullptr, &capi_result);
#ifndef ATRACE_UNSUPPORTED
        ATRACE_END();
#endif
        capi_call_end = std::chrono::steady_clock::now();
        total_capi_get_param_duration +=
            std::chrono::duration_cast<std::chrono::milliseconds>(
                capi_call_end - capi_call_start).count();
        if (CAPI_V2_EFAILED == rc) {
            PAL_ERR(LOG_TAG, "capi get param failed\n");
            status = -EINVAL;
            goto reinit;
        }

        det_conf_score = (int32_t)result_cfg_ptr->final_user_score;
        PAL_INFO(LOG_TAG, "UV second stage conf level %d, processing %u bytes",
            det_conf_score, processed_sz);
        if (result_cfg_ptr->is_detected) {
            exit_buffering_ = true;
            detection_state_ = USER_VERIFICATION_SUCCESS;
            param.stream = (void *)stream_handle_;
            param.data = (void *)&det_conf_score;
            param.size = sizeof(int32_t);
            vui_intf_->SetParameter(PARAM_SSTAGE_UV_DET_LEVEL ,&param);
            PAL_INFO(LOG_TAG, "UV Second Stage Detected");
            goto reinit;
        }

        if (process_count < max_process_count) {
            /* Reinit the UV module for next processing */
            PAL_DBG(LOG_TAG, "%s: Issuing capi_set_param for param %d", __func__,
                    STAGE2_UV_WRAPPER_ID_REINIT);
            rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
                STAGE2_UV_WRAPPER_ID_REINIT, nullptr, nullptr);
            if (CAPI_V2_EOK != rc) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG,
                    "set_param STAGE2_UV_WRAPPER_ID_REINIT failed, status = %d",
                    status);
                goto exit;
            }
        }
    }

reinit:
    if (CAPI_V2_ENETRESET != rc) {
        /* Reinit the UV module except SSR occurrence */
        PAL_DBG(LOG_TAG, "%s: Issuing capi_set_param for param %d", __func__,
            STAGE2_UV_WRAPPER_ID_REINIT);
        rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
            STAGE2_UV_WRAPPER_ID_REINIT, nullptr, nullptr);
        if (CAPI_V2_EOK != rc) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "set_param STAGE2_UV_WRAPPER_ID_REINIT failed, status = %d",
                    status);
        }
    }

exit:
    process_end = std::chrono::steady_clock::now();
    process_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        process_end - process_start).count();
    PAL_INFO(LOG_TAG, "UV processing time: Bytes processed %u, Total processing "
        "time %llums, Algo process time %llums, get result time %llums",
        processed_sz, (long long)process_duration,
        (long long)total_capi_process_duration,
        (long long)total_capi_get_param_duration);
    if (vui_ptfm_info_->GetEnableDebugDumps()) {
        ST_DBG_FILE_CLOSE(user_verification_fd);
    }

    if (sm_cfg_->IsDetPropSupported(ST_PARAM_KEY_SSTAGE_UV_ENGINE_INFO)) {
        struct st_det_engine_stats engine_info;
        engine_info.version = 1;
        if (CAPI_V2_ENETRESET == rc) {
            /* USER_VERIFICATION_REJECT will be notified to the client. */
            engine_info.detection_state = SSTAGE_SUBSYSTEM_RESTART;
        } else {
            engine_info.detection_state = detection_state_;
        }
        engine_info.processed_length =
            BytesToFrames(processed_sz) * MS_PER_SEC / sample_rate_;
        engine_info.total_process_duration = process_duration;
        engine_info.total_capi_process_duration = total_capi_process_duration;
        engine_info.total_capi_get_param_duration = total_capi_get_param_duration;
        param.stream = (void *)stream_handle_;
        param.data = (void *)&engine_info;
        param.size = sizeof(struct st_det_engine_stats);
        vui_intf_->SetParameter(PARAM_SSTAGE_UV_DET_STATS, &param);
    }

    if (reader_)
        reader_->updateState(READER_DISABLED);

    if (process_input_buff)
        free(process_input_buff);
    if (stream_input) {
        if (stream_input->buf_ptr)
            free(stream_input->buf_ptr);
        free(stream_input);
    }
    if (result_cfg_ptr)
        free(result_cfg_ptr);
    if (uv_cfg_ptr)
        free(uv_cfg_ptr);
    if (scratch_param_.scratch_ptr) {
        free(scratch_param_.scratch_ptr);
        scratch_param_.scratch_ptr = nullptr;
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapi::StartTIUserVerification()
{
    int32_t status = 0;
    char *process_input_buff = nullptr;
    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_stream_data_t *stream_input = nullptr;
    stage2_uv_wrapper_result *result_cfg_ptr = nullptr;
    int32_t read_size = 0;
    capi_v2_buf_t capi_result;
    FILE *user_verification_fd = nullptr;
    ChronoSteadyClock_t process_start;
    ChronoSteadyClock_t process_end;
    ChronoSteadyClock_t capi_call_start;
    ChronoSteadyClock_t capi_call_end;
    uint64_t process_duration = 0;
    uint64_t total_capi_process_duration = 0;
    uint64_t total_capi_get_param_duration = 0;
    uint32_t hist_data_size = 0, min_hist_data_size = 0;
    uint32_t frame_size = 0, proc_size = 0;
    uint32_t processed_sz = 0;
    vui_intf_param_t param = {};
    tiuv_detection_result_t tiuv_detection_result = {};
    struct buffer_config buf_config = {};
    int32_t det_conf_score = 0;

    PAL_DBG(LOG_TAG, "Enter");

    status = UpdateUVScratchParam();
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to update UV scratch param");
        goto exit;
    }

    if (!reader_ || !reader_->isEnabled()) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid ring buffer reader");
        goto exit;
    }

    if (vui_ptfm_info_->GetEnableDebugDumps()) {
        ST_DBG_FILE_OPEN_WR(user_verification_fd, ST_DEBUG_DUMP_LOCATION,
            "user_verification", "bin", user_verification_cnt);
        PAL_DBG(LOG_TAG, "User Verification data stored in: user_verification_%d.bin",
            user_verification_cnt);
        user_verification_cnt++;
    }

    param.stream = (void *)stream_handle_;
    param.data = (void *)&buf_config;
    param.size = sizeof(struct buffer_config);
    vui_intf_->GetParameter(PARAM_FSTAGE_BUFFERING_CONFIG, &param);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to get buffering config, status %d", status);
        goto exit;
    }

    hist_data_size = reader_->getUnreadSize() % reader_->getBufferSize();
    min_hist_data_size = UsToBytes(
        buf_config.hist_buffer_duration * US_PER_SEC / MS_PER_SEC);
    if (hist_data_size < min_hist_data_size)
        hist_data_size = min_hist_data_size;
    frame_size = UsToBytes(
        ss_cfg_->GetProcFrameSize() * US_PER_SEC / MS_PER_SEC);
    proc_size = hist_data_size;

    memset(&capi_result, 0, sizeof(capi_result));

    process_input_buff = (char*)calloc(1, proc_size);
    if (!process_input_buff) {
        PAL_ERR(LOG_TAG, "failed to allocate process input buff");
        status = -ENOMEM;
        goto exit;
    }

    stream_input = (capi_v2_stream_data_t *)
                   calloc(1, sizeof(capi_v2_stream_data_t));
    if (!stream_input) {
        PAL_ERR(LOG_TAG, "failed to allocate stream input");
        status = -ENOMEM;
        goto exit;
    }

    stream_input->buf_ptr = (capi_v2_buf_t*)calloc(1, sizeof(capi_v2_buf_t));
    if (!stream_input->buf_ptr) {
        PAL_ERR(LOG_TAG, "failed to allocate buf ptr");
        status = -ENOMEM;
        goto exit;
    }

    result_cfg_ptr = (stage2_uv_wrapper_result*)
                     calloc(1, sizeof(stage2_uv_wrapper_result));
    if (!result_cfg_ptr) {
        PAL_ERR(LOG_TAG, "failed to allocate result cfg ptr");
        status = -ENOMEM;
        goto exit;
    }

    process_start = std::chrono::steady_clock::now();
    while (!exit_buffering_) {
        if (!reader_->waitForBuffers(proc_size))
            continue;

        read_size = reader_->read((void*)process_input_buff, proc_size);
        if (read_size == 0) {
            continue;
        } else if (read_size < 0) {
            status = read_size;
            PAL_ERR(LOG_TAG, "Failed to read from buffer, status %d", status);
            goto exit;
        }
        stream_input->bufs_num = 1;
        stream_input->buf_ptr->max_data_len = proc_size;
        stream_input->buf_ptr->actual_data_len = read_size;
        stream_input->buf_ptr->data_ptr = (int8_t *)process_input_buff;

        if (vui_ptfm_info_->GetEnableDebugDumps()) {
            ST_DBG_FILE_WRITE(user_verification_fd,
                process_input_buff, read_size);
        }

        PAL_VERBOSE(LOG_TAG, "Calling Capi Process\n");
        capi_call_start = std::chrono::steady_clock::now();
#ifndef ATRACE_UNSUPPORTED
        ATRACE_BEGIN("Second stage uv process");
#endif
        rc = capi_handle_->vtbl_ptr->process(capi_handle_,
            &stream_input, nullptr);
#ifndef ATRACE_UNSUPPORTED
        ATRACE_END();
#endif
        capi_call_end = std::chrono::steady_clock::now();
        total_capi_process_duration +=
            std::chrono::duration_cast<std::chrono::milliseconds>(
                capi_call_end - capi_call_start).count();
        if (CAPI_V2_EFAILED == rc) {
            PAL_ERR(LOG_TAG, "capi process failed\n");
            status = -EINVAL;
            goto exit;
        }

        processed_sz += read_size;

        capi_result.data_ptr = (int8_t*)result_cfg_ptr;
        capi_result.actual_data_len = sizeof(stage2_uv_wrapper_result);
        capi_result.max_data_len = sizeof(stage2_uv_wrapper_result);

        PAL_VERBOSE(LOG_TAG, "Calling Capi get param for result\n");
        capi_call_start = std::chrono::steady_clock::now();
#ifndef ATRACE_UNSUPPORTED
        ATRACE_BEGIN("Second stage TI-UV get result");
#endif
        rc = capi_handle_->vtbl_ptr->get_param(capi_handle_,
            STAGE2_UV_WRAPPER_ID_RESULT, nullptr, &capi_result);
#ifndef ATRACE_UNSUPPORTED
        ATRACE_END();
#endif
        capi_call_end = std::chrono::steady_clock::now();
        total_capi_get_param_duration +=
            std::chrono::duration_cast<std::chrono::milliseconds>(
                capi_call_end - capi_call_start).count();
        if (CAPI_V2_EFAILED == rc) {
            PAL_ERR(LOG_TAG, "capi get param failed\n");
            status = -EINVAL;
            goto exit;
        }

        det_conf_score = (int32_t)result_cfg_ptr->final_user_score;
        if (result_cfg_ptr->is_detected) {
            exit_buffering_ = true;
            detection_state_ = USER_VERIFICATION_SUCCESS;
            PAL_INFO(LOG_TAG, "TI-UV Second Stage Detected");
            tiuv_detection_result.detection_status = detection_state_;
            tiuv_detection_result.user_score = det_conf_score;
            param.stream = (void *)stream_handle_;
            param.data = (void *)&tiuv_detection_result;
            param.size = sizeof(int32_t);
            vui_intf_->SetParameter(PARAM_TIUV_DETECTION_RESULT, &param);
        } else if (result_cfg_ptr->timeout) {
            exit_buffering_ = true;
            detection_state_ = USER_VERIFICATION_REJECT;
            PAL_INFO(LOG_TAG, "TI-UV Second Stage Rejected due to timeout");
            tiuv_detection_result.detection_status = detection_state_;
            tiuv_detection_result.user_score = det_conf_score;
            param.stream = (void *)stream_handle_;
            param.data = (void *)&tiuv_detection_result;
            param.size = sizeof(int32_t);
            vui_intf_->SetParameter(PARAM_TIUV_DETECTION_RESULT, &param);
        }
        PAL_INFO(LOG_TAG, "TI-UV second stage conf level %d, processing %u bytes",
            det_conf_score, processed_sz);
        if (proc_size == hist_data_size)
            proc_size = frame_size;
    }

exit:
    process_end = std::chrono::steady_clock::now();
    process_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        process_end - process_start).count();
    PAL_INFO(LOG_TAG, "TI-UV processing time: Bytes processed %u, Total processing "
        "time %llums, Algo process time %llums, get result time %llums",
        processed_sz, (long long)process_duration,
        (long long)total_capi_process_duration,
        (long long)total_capi_get_param_duration);
    if (vui_ptfm_info_->GetEnableDebugDumps()) {
        ST_DBG_FILE_CLOSE(user_verification_fd);
    }

    /* Reinit the UV module */
    PAL_DBG(LOG_TAG, "%s: Issuing capi_set_param for param %d", __func__,
            STAGE2_UV_WRAPPER_ID_REINIT);
    rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
        STAGE2_UV_WRAPPER_ID_REINIT, nullptr, nullptr);
    if (CAPI_V2_EOK != rc) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "set_param STAGE2_UV_WRAPPER_ID_REINIT failed, status = %d",
                status);
    }

    if (reader_)
        reader_->updateState(READER_DISABLED);

    if (process_input_buff)
        free(process_input_buff);
    if (stream_input) {
        if (stream_input->buf_ptr)
            free(stream_input->buf_ptr);
        free(stream_input);
    }
    if (result_cfg_ptr)
        free(result_cfg_ptr);
    if (scratch_param_.scratch_ptr) {
        free(scratch_param_.scratch_ptr);
        scratch_param_.scratch_ptr = nullptr;
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

SoundTriggerEngineCapi::SoundTriggerEngineCapi(
    StreamSoundTrigger *s,
    listen_model_indicator_enum type,
    std::shared_ptr<VUIStreamConfig> sm_cfg)
{
    int32_t status = 0;
    struct pal_stream_attributes attr;

    PAL_DBG(LOG_TAG, "Enter");
    engine_type_ = type;
    sm_cfg_ = sm_cfg;
    vui_intf_ = nullptr;
    processing_started_ = false;
    sm_data_ = nullptr;
    exit_thread_ = false;
    exit_buffering_ = false;
    reader_ = nullptr;
    buffer_ = nullptr;
    stream_handle_ = s;
    confidence_threshold_ = 0;
    detection_state_ = ENGINE_IDLE;
    capi_handle_ = nullptr;
    capi_lib_handle_ = nullptr;
    capi_init_ = nullptr;
    keyword_detected_ = false;
    sec_kw_detect_state_ = 0;
    memset(&in_model_buffer_param_, 0, sizeof(in_model_buffer_param_));
    memset(&scratch_param_, 0, sizeof(scratch_param_));

    vui_ptfm_info_ = VoiceUIPlatformInfo::GetInstance();
    if (!vui_ptfm_info_) {
        PAL_ERR(LOG_TAG, "No voice UI platform info present");
        throw std::runtime_error("No voice UI platform info present");
    }

    kw_start_tolerance_ = sm_cfg_->GetKwStartTolerance();
    kw_end_tolerance_ = sm_cfg_->GetKwEndTolerance();
    data_before_kw_start_ = sm_cfg_->GetDataBeforeKwStart();
    data_after_kw_end_ = sm_cfg_->GetDataAfterKwEnd();

    ss_cfg_ = sm_cfg_->GetVUISecondStageConfig(engine_type_);
    if (!ss_cfg_) {
        PAL_ERR(LOG_TAG, "Failed to get second stage config");
        throw std::runtime_error("Failed to get second stage config");
    }

    status = s->getStreamAttributes(&attr);
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to get stream attributes");
        throw std::runtime_error("Failed to get stream attributes");
    }

    sample_rate_ = attr.in_media_config.sample_rate;
    bit_width_ = attr.in_media_config.bit_width;
    channels_ = attr.in_media_config.ch_info.channels;
    detection_type_ = ss_cfg_->GetDetectionType();
    lib_name_ = ss_cfg_->GetLibName();
    buffer_size_ = UsToBytes(CNN_BUFFER_LENGTH);

    // TODO: ST_SM_TYPE_CUSTOM_DETECTION
    if (detection_type_ == ST_SM_TYPE_KEYWORD_DETECTION) {
        capi_handle_ = (capi_v2_t *)calloc(1,
            sizeof(capi_v2_t) + sizeof(char *));
    } else if (detection_type_ == ST_SM_TYPE_USER_VERIFICATION) {
        capi_handle_ = (capi_v2_t *)calloc(1,
            sizeof(capi_v2_t) + (2 * sizeof(char *)));
    }

    if (!capi_handle_) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG, "failed to allocate capi handle = %d", status);
        /* handle here */
        goto err_exit;
    }

    capi_lib_handle_ = dlopen(lib_name_.c_str(), RTLD_NOW);
    if (!capi_lib_handle_) {
        status = -ENOMEM;
        PAL_ERR(LOG_TAG,  "failed to open capi so = %d", status);
        /* handle here */
        goto err_exit;
    }

    dlerror();

    capi_init_ = (capi_v2_init_f)dlsym(capi_lib_handle_, "capi_v2_init");

    if (!capi_init_) {
        PAL_ERR(LOG_TAG,  "failed to map capi init function = %d", status);
        /* handle here */
        goto err_exit;
    }

    return;
err_exit:
    if (capi_handle_) {
        free(capi_handle_);
        capi_handle_ = nullptr;
    }
    if (capi_lib_handle_) {
        dlclose(capi_lib_handle_);
        capi_lib_handle_ = nullptr;
    }
    PAL_ERR(LOG_TAG, "constructor exit status = %d", status);
}

SoundTriggerEngineCapi::~SoundTriggerEngineCapi()
{
    PAL_DBG(LOG_TAG, "Enter");
    /*
     * join thread if it is not joined, sometimes
     * stop/unload may fail before deconstruction.
     */
    if (buffer_thread_handler_.joinable()) {
        processing_started_ = false;
        std::unique_lock<std::mutex> lck(event_mutex_);
        exit_thread_ = true;
        exit_buffering_ = true;
        cv_.notify_one();
        lck.unlock();
        buffer_thread_handler_.join();
        PAL_INFO(LOG_TAG, "Thread joined");
    }
    if (buffer_) {
        delete buffer_;
    }
    if (reader_) {
        delete reader_;
    }
    if (capi_lib_handle_) {
        dlclose(capi_lib_handle_);
        capi_lib_handle_ = nullptr;
    }
    if (capi_handle_) {
        capi_handle_->vtbl_ptr = nullptr;
        free(capi_handle_);
        capi_handle_ = nullptr;
    }
    vui_intf_ = nullptr;
    PAL_DBG(LOG_TAG, "Exit");
}

int32_t SoundTriggerEngineCapi::StartSoundEngine()
{
    int32_t status = 0;
    processing_started_ = false;
    exit_thread_ = false;
    exit_buffering_ = false;
    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_buf_t capi_buf;

    PAL_DBG(LOG_TAG, "Enter");
    if (detection_type_ == ST_SM_TYPE_KEYWORD_DETECTION) {
        sva_threshold_config_t *threshold_cfg = nullptr;
        threshold_cfg = (sva_threshold_config_t*)
                        calloc(1, sizeof(sva_threshold_config_t));
        if (!threshold_cfg) {
            status = -ENOMEM;
            PAL_ERR(LOG_TAG, "threshold cfg calloc failed, status %d", status);
            return status;
        }
        capi_buf.data_ptr = (int8_t*) threshold_cfg;
        capi_buf.actual_data_len = sizeof(sva_threshold_config_t);
        capi_buf.max_data_len = sizeof(sva_threshold_config_t);
        threshold_cfg->smm_threshold = confidence_threshold_;

        PAL_DBG(LOG_TAG, "Keyword detection confidence level = %d",
            threshold_cfg->smm_threshold);

        status = capi_handle_->vtbl_ptr->set_param(capi_handle_,
            SVA_ID_THRESHOLD_CONFIG, nullptr, &capi_buf);
        free(threshold_cfg);
        if (CAPI_V2_EOK != status) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "set param SVA_ID_THRESHOLD_CONFIG failed with %d",
                    status);
            return status;
        }

        PAL_VERBOSE(LOG_TAG, "Issuing capi_set_param for param %d",
                    SVA_ID_REINIT_ALL);
        status = capi_handle_->vtbl_ptr->set_param(capi_handle_,
            SVA_ID_REINIT_ALL, nullptr, nullptr);

        if (CAPI_V2_EOK != status) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "set param SVA_ID_REINIT_ALL failed, status = %d",
                    status);
            return status;
        }
        detection_state_ = KEYWORD_DETECTION_PENDING;
    } else if (detection_type_ == ST_SM_TYPE_USER_VERIFICATION) {
        if (engine_type_ & ST_SM_ID_SVA_S_STAGE_USER) {
            stage2_uv_wrapper_threshold_config_t *threshold_cfg = nullptr;

            threshold_cfg = (stage2_uv_wrapper_threshold_config_t *)
                calloc(1, sizeof(stage2_uv_wrapper_threshold_config_t));
            if (!threshold_cfg) {
                PAL_ERR(LOG_TAG, "failed to allocate threshold cfg");
                status = -ENOMEM;
                return status;
            }

            capi_buf.data_ptr = (int8_t *)threshold_cfg;
            capi_buf.actual_data_len = sizeof(stage2_uv_wrapper_threshold_config_t);
            capi_buf.max_data_len = sizeof(stage2_uv_wrapper_threshold_config_t);
            threshold_cfg->threshold = confidence_threshold_;
            threshold_cfg->anti_spoofing_enabled = 0;
            threshold_cfg->anti_spoofing_threshold = 0;

            PAL_DBG(LOG_TAG, "User Verification confidence level = %d",
                    threshold_cfg->threshold);

            rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
                STAGE2_UV_WRAPPER_ID_THRESHOLD, nullptr, &capi_buf);
            free(threshold_cfg);
            if (CAPI_V2_EOK != rc) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "set param %d failed with %d",
                        STAGE2_UV_WRAPPER_ID_THRESHOLD, rc);
                return status;
            }
        } else if (engine_type_ & ST_SM_ID_SVA_S_STAGE_CTIUV) {
            stage2_uv_wrapper_ctiuv_config_t *ctiuv_config = nullptr;
            vui_intf_param_t param = {};

            param.stream = (void *)stream_handle_;
            param.size = sizeof(tiuv_threshold_config_t);
            status = vui_intf_->GetParameter(PARAM_TIUV_THRESHOLD_CONFIG, &param);
            if (status) {
                PAL_ERR(LOG_TAG, "failed to get tiuv cfg");
                return status;
            }
            ctiuv_config = (stage2_uv_wrapper_ctiuv_config_t *)param.data;

            capi_buf.data_ptr = (int8_t *)ctiuv_config;
            capi_buf.actual_data_len = sizeof(stage2_uv_wrapper_ctiuv_config_t);
            capi_buf.max_data_len = sizeof(stage2_uv_wrapper_ctiuv_config_t);

            PAL_DBG(LOG_TAG, "User Verification confidence level = %d",
                    ctiuv_config->tiuv_thresholds[0]);

            rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
                STAGE2_UV_WRAPPER_ID_CTIUV_PARAMS, nullptr, &capi_buf);
            if (CAPI_V2_EOK != rc) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "set param %d failed with %d",
                        STAGE2_UV_WRAPPER_ID_CTIUV_PARAMS, rc);
                return status;
            }
        }
        detection_state_ =  USER_VERIFICATION_PENDING;
    }

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapi::StopSoundEngine()
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    {
        processing_started_ = false;
        std::lock_guard<std::mutex> lck(event_mutex_);
        exit_thread_ = true;
        exit_buffering_ = true;

        cv_.notify_one();
    }
    if (buffer_thread_handler_.joinable()) {
        PAL_DBG(LOG_TAG, "Thread joined");
        buffer_thread_handler_.join();
    }
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapi::UpdateConfThreshold(StreamSoundTrigger *s)
{
    vui_intf_param_t param;

    if (!vui_intf_) {
        PAL_ERR(LOG_TAG, "No vui interface present");
        return -EINVAL;
    }

    param.stream = (void *)s;
    param.data = (void *)&confidence_threshold_;
    param.size = sizeof(int32_t);
    if (engine_type_ & ST_SM_ID_SVA_S_STAGE_KWD)
        vui_intf_->GetParameter(PARAM_SSTAGE_KW_CONF_LEVEL, &param);
    else
        vui_intf_->GetParameter(PARAM_SSTAGE_UV_CONF_LEVEL, &param);

    PAL_INFO(LOG_TAG, "Confidence threshold: %d for sound model 0x%x",
        confidence_threshold_, engine_type_);

    return 0;
}

int32_t SoundTriggerEngineCapi::UpdateUVScratchParam()
{
    int32_t status = 0;
    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_buf_t capi_uv_ptr;

    if (!scratch_param_.scratch_size) {
        PAL_DBG(LOG_TAG, "skip param update as scratch param size is 0");
        return 0;
    }

    capi_uv_ptr.data_ptr = (int8_t *)&scratch_param_;
    capi_uv_ptr.actual_data_len = sizeof(scratch_param_);
    capi_uv_ptr.max_data_len = sizeof(scratch_param_);
    scratch_param_.scratch_ptr = (int8_t *)calloc(1, scratch_param_.scratch_size);
    PAL_INFO(LOG_TAG, "Allocated scratch memory with size %d", scratch_param_.scratch_size);
    if (scratch_param_.scratch_ptr == NULL) {
        PAL_ERR(LOG_TAG, "failed to allocate the scratch memory");
        return -ENOMEM;
    }

    PAL_VERBOSE(LOG_TAG, "Issuing capi_set STAGE2_UV_WRAPPER_ID_SCRATCH_PARAM");
    rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
                       STAGE2_UV_WRAPPER_ID_SCRATCH_PARAM,
                       NULL,
                       &capi_uv_ptr);
    if (CAPI_V2_EFAILED == rc) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "capi set param STAGE2_UV_WRAPPER_ID_SCRATCH_PARAM failed, status %d", status);
        free(scratch_param_.scratch_ptr);
        scratch_param_.scratch_ptr = NULL;
    }
    return status;
}

int32_t SoundTriggerEngineCapi::LoadSoundModel(StreamSoundTrigger *s __unused,
    sound_model_data_t *sm_data)
{
    int32_t status = 0;
    capi_v2_err_t rc = CAPI_V2_EOK;
    capi_v2_proplist_t init_set_proplist;
    capi_v2_prop_t sm_prop_ptr;
    capi_v2_buf_t capi_uv_ptr;
    bool is_client_handling_ssr = false;
    stage2_uv_wrapper_ssr_recovery_config_t *cfg = nullptr;
    stage2_uv_wrapper_model_backend_type_t *sm_backend_type = nullptr;
    capi_v2_buf_t capi_buf;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    if (!sm_data || !sm_data->data) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Invalid sound model data, status %d", status);
        goto exit;
    }

    if (!capi_handle_) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "Capi handle not allocated, status %d", status);
        goto exit;
    }

    stream_handle_ = s;
    sm_data_ = sm_data->data;
    sm_data_size_ = sm_data->size;

    sm_prop_ptr.id = CAPI_V2_CUSTOM_INIT_DATA;
    sm_prop_ptr.payload.data_ptr = (int8_t *)sm_data_;
    sm_prop_ptr.payload.actual_data_len = sm_data_size_;
    sm_prop_ptr.payload.max_data_len = sm_data_size_;
    init_set_proplist.props_num = 1;
    init_set_proplist.prop_ptr = &sm_prop_ptr;

    memset(&capi_uv_ptr, 0, sizeof(capi_uv_ptr));
    memset(&in_model_buffer_param_, 0, sizeof(in_model_buffer_param_));
    memset(&scratch_param_, 0, sizeof(scratch_param_));

    PAL_VERBOSE(LOG_TAG, "Issuing capi_init");
    rc = capi_init_(capi_handle_, &init_set_proplist);

    if (rc != CAPI_V2_EOK) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "capi_init status is %d, exiting, status %d",
                rc, status);
        goto exit;
    }

    if (!capi_handle_->vtbl_ptr) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "capi_handle->vtbl_ptr is nullptr, exiting, status %d",
                status);
        goto exit;
    }

    buffer_thread_handler_ =
        std::thread(SoundTriggerEngineCapi::BufferThreadLoop, this);

    if (!buffer_thread_handler_.joinable()) {
        status = -EINVAL;
        PAL_ERR(LOG_TAG, "failed to create buffer thread = %d", status);
        goto exit;
    }

    if (detection_type_ == ST_SM_TYPE_USER_VERIFICATION) {
        PAL_VERBOSE(LOG_TAG, "Issuing capi_get STAGE2_UV_WRAPPER_ID_INMODEL_BUFFER_SIZE");

        capi_uv_ptr.data_ptr = (int8_t *)&in_model_buffer_param_;
        capi_uv_ptr.actual_data_len = sizeof(in_model_buffer_param_);
        capi_uv_ptr.max_data_len = sizeof(in_model_buffer_param_);

        rc = capi_handle_->vtbl_ptr->get_param(capi_handle_,
                           STAGE2_UV_WRAPPER_ID_INMODEL_BUFFER_SIZE,
                           NULL,
                           &capi_uv_ptr);

        if (CAPI_V2_EFAILED == rc) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "capi_get STAGE2_UV_WRAPPER_ID_INMODEL_BUFFER_SIZE param failed, %d",
                    rc);
            goto exit;
        }

        if (in_model_buffer_param_.scratch_size == 0) {
            capi_uv_ptr.data_ptr = (int8_t *)&scratch_param_;
            capi_uv_ptr.actual_data_len = sizeof(scratch_param_);
            capi_uv_ptr.max_data_len = sizeof(scratch_param_);

            PAL_VERBOSE(LOG_TAG, "Issuing capi_get STAGE2_UV_WRAPPER_ID_SCRATCH_PARAM");
            rc = capi_handle_->vtbl_ptr->get_param(capi_handle_,
                               STAGE2_UV_WRAPPER_ID_SCRATCH_PARAM,
                               NULL,
                               &capi_uv_ptr);

            if (CAPI_V2_EFAILED == rc) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "capi get param STAGE2_UV_WRAPPER_ID_SCRATCH_PARAM failed, %d",
                        rc);
                goto exit;
            }
        }

        sm_backend_type = (stage2_uv_wrapper_model_backend_type_t *)
                     calloc(1,sizeof(stage2_uv_wrapper_model_backend_type_t));
        if (!sm_backend_type) {
            PAL_ERR(LOG_TAG, "Failed to allocate memory for backend type");
            status = -ENOMEM;
            goto exit;
        }

        capi_buf.data_ptr = (int8_t*) sm_backend_type;
        capi_buf.actual_data_len = sizeof(stage2_uv_wrapper_model_backend_type_t);
        capi_buf.max_data_len = sizeof(stage2_uv_wrapper_ssr_recovery_config_t);

        PAL_VERBOSE(LOG_TAG, "Issuing capi_get STAGE2_UV_WRAPPER_ID_MODEL_BACKEND_TYPE");
        rc = capi_handle_->vtbl_ptr->get_param(capi_handle_,
                           STAGE2_UV_WRAPPER_ID_MODEL_BACKEND_TYPE,
                           NULL,
                           &capi_buf);

        if (CAPI_V2_EFAILED == rc) {
            status = -EINVAL;
            PAL_ERR(LOG_TAG, "capi get param STAGE2_UV_WRAPPER_ID_MODEL_BACKEND_TYPE failed, %d",
                    rc);
            goto exit;
        }
        if (sm_backend_type->backend_type == STAGE2_UV_WRAPPER_MODEL_BACKEND_TYPE_QNN) {
            is_client_handling_ssr = sm_cfg_->IsClientHandleSSR();
            cfg = (stage2_uv_wrapper_ssr_recovery_config_t *)
                calloc(1, sizeof(stage2_uv_wrapper_ssr_recovery_config_t));
            if (!cfg) {
                PAL_ERR(LOG_TAG, "Failed to allocate SSR cfg");
                status = -ENOMEM;
                goto exit;
            }

            capi_buf.data_ptr = (int8_t*) cfg;
            capi_buf.actual_data_len = sizeof(stage2_uv_wrapper_ssr_recovery_config_t);
            capi_buf.max_data_len = sizeof(stage2_uv_wrapper_ssr_recovery_config_t);
            if (is_client_handling_ssr) {
                cfg->client_handling_ssr = 1;
                sm_data->is_persistent = false;
            } else {
                cfg->client_handling_ssr = 0;
            }

            PAL_DBG(LOG_TAG, "Is Client Handling SSR = %d",
               cfg->client_handling_ssr);

            rc = capi_handle_->vtbl_ptr->set_param(capi_handle_,
               STAGE2_UV_WRAPPER_ID_SSR_RECOVERY_CONFIG, nullptr, &capi_buf);
            if (CAPI_V2_EOK != rc) {
                status = -EINVAL;
                PAL_ERR(LOG_TAG, "set param %d failed with %d",
                        STAGE2_UV_WRAPPER_ID_SSR_RECOVERY_CONFIG, rc);
                goto exit;
            }
        }
    }
exit:
    if (sm_backend_type)
        free(sm_backend_type);
    if (cfg)
        free(cfg);

    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapi::UnloadSoundModel(StreamSoundTrigger *s __unused)
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter, Issuing capi_end");
    std::lock_guard<std::mutex> lck(mutex_);
    status = StopSoundEngine();
    if (status) {
        PAL_ERR(LOG_TAG, "Failed to stop sound engine, status = %d", status);
    }

    status = capi_handle_->vtbl_ptr->end(capi_handle_);
    if (status != CAPI_V2_EOK) {
        PAL_ERR(LOG_TAG, "Capi end function failed, status = %d",
            status);
        status = -EINVAL;
        goto exit;
    }

exit:
    if (scratch_param_.scratch_ptr) {
        free(scratch_param_.scratch_ptr);
        scratch_param_.scratch_ptr = NULL;
    }
    PAL_DBG(LOG_TAG, "Exit, status %d", status);
    return status;
}

int32_t SoundTriggerEngineCapi::SetBufferReader(PalRingBufferReader *reader)
{
    if (!reader) {
        PAL_DBG(LOG_TAG, "Null reader");
        return -EINVAL;
    }
    reader_ = reader;
    return 0;
}

int32_t SoundTriggerEngineCapi::StartRecognition(StreamSoundTrigger *s)
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    status = UpdateConfThreshold(s);
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Failed to update confidence threshold, status = %d",
            status);
        goto exit;
    }

    status = StartSoundEngine();
    if (0 != status) {
        PAL_ERR(LOG_TAG, "Failed to start sound engine, status = %d", status);
        goto exit;
    }

exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapi::RestartRecognition(StreamSoundTrigger *s __unused)
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    processing_started_ = false;
    {
        exit_buffering_ = true;
        std::lock_guard<std::mutex> event_lck(event_mutex_);
        cv_.notify_all();
    }
    if (reader_) {
        reader_->reset();
    } else {
        status = -EINVAL;
        goto exit;
    }

exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

int32_t SoundTriggerEngineCapi::StopRecognition(StreamSoundTrigger *s __unused)
{
    int32_t status = 0;

    PAL_DBG(LOG_TAG, "Enter");
    std::lock_guard<std::mutex> lck(mutex_);
    processing_started_ = false;
    {
        exit_buffering_ = true;
        std::lock_guard<std::mutex> event_lck(event_mutex_);
        cv_.notify_all();
    }
    if (reader_) {
        reader_->reset();
    } else {
        status = -EINVAL;
        goto exit;
    }

exit:
    PAL_DBG(LOG_TAG, "Exit, status %d", status);

    return status;
}

void SoundTriggerEngineCapi::SetDetected(int32_t detection_state)
{
    PAL_DBG(LOG_TAG, "SetDetected %d", detection_state);
    std::lock_guard<std::mutex> lck(event_mutex_);
    if (detection_state == GMM_DETECTED) {
        reader_->updateState(READER_ENABLED);
        processing_started_ = true;
        exit_buffering_ = !processing_started_;
        cv_.notify_one();
    } else if (detection_state == KEYWORD_DETECTION_SUCCESS) {
        if (!processing_started_)
            return;
        /*
         * NOTE: Applicable to Second stage UV only
         * Second stage KW detected, wake up processing thread
         * to start processing with index from second stage KW
         */
        PAL_INFO(LOG_TAG, "Second stage KW detection success");
        sec_kw_detect_state_ = detection_state;
        cv_.notify_one();
    } else if (detection_state == KEYWORD_DETECTION_REJECT) {
        if (!processing_started_)
            return;
        /*
         * NOTE: Applicable to Second stage UV only
         * Second stage KW rejected, set exit_buffering_
         * to true to exit processing
         */
        exit_buffering_ = true;
        PAL_INFO(LOG_TAG, "exit UV processing as KW detection fails");
        sec_kw_detect_state_ = detection_state;
        cv_.notify_one();
    } else {
        PAL_ERR(LOG_TAG, "Invalid detection state");
    }
}
