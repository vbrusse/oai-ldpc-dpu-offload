/*
 * Copyright (c) 2023 NVIDIA CORPORATION AND AFFILIATES.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of
 *       conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the NVIDIA CORPORATION nor the names of its contributors may be used
 *       to endorse or promote products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TOR (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Original filename: comch_data_path_high_speed_common.c
 *
 * Filename: nrLDPC_data_path_high_speed_common.c
 *
 * DOCA Communication Channel Client API customized by: Vlademir Brusse
 *
 * Date: 2025/09/30
 *
 */

#include <time.h>

#include <doca_buf.h>
#include <doca_comch.h>
#include <doca_comch_consumer.h>
#include <doca_comch_producer.h>
#include <doca_ctx.h>
#include <doca_dev.h>
#include <doca_error.h>
#include <doca_log.h>
#include <doca_pe.h>

/* #include "comch_ctrl_path_common.h" */
#include "nrLDPC_ctrl_path_common.h"
#include "nrLDPC_data_path_high_speed_common.h"
#include "common.h"



/*
doca_error_t ldpc_encoder_kernel(struct ldpc_encod_params_t *params);           VBrusse
doca_error_t ldpc_decoder_kernel(struct ldpc_decod_params_t *params);           VBrusse */



DOCA_LOG_REGISTER(NRLDPC_COMMON);

void clean_local_mem_bufs(struct local_mem_bufs *local)
{
        doca_error_t result;
        void *mem;
        size_t mem_size;

        if (local == NULL)
                return;

        if (local->need_alloc_mem == true) {
                result = doca_mmap_get_memrange(local->mmap, &mem, &mem_size);
                if (result != DOCA_SUCCESS) {
                        DOCA_LOG_ERR("Unable to get mmap memrange: %s", doca_error_get_descr(result));
                        return;
                }
                free(mem);
        }
        local->mem = NULL;

        result = doca_mmap_destroy(local->mmap);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to destroy mmap: %s", doca_error_get_descr(result));
                return;
        }
        local->mmap = NULL;

        result = doca_buf_inventory_destroy(local->buf_inv);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to destroy inventory: %s", doca_error_get_descr(result));
                return;
        }
        local->buf_inv = NULL;
}

doca_error_t init_local_mem_bufs(struct local_mem_bufs *local, struct doca_dev *dev, size_t buf_len, size_t max_bufs)
{
        doca_error_t result;

        if (local->need_alloc_mem == true) {
                void *aligned_ptr = NULL;
                size_t total_size = max_bufs * buf_len;

                // posix_memalign requires a pointer-to-pointer, a power-of-2 alignment (64),
                // and the total size in bytes. It returns 0 on success.
                if (posix_memalign(&aligned_ptr, 64, total_size) != 0) {
                        result = DOCA_ERROR_NO_MEMORY;
                        DOCA_LOG_ERR("Unable to alloc 64B aligned memory to mmap: %s", doca_error_get_descr(result));
                        return result;
                }

                // Cast and assign the safely allocated pointer to your struct
                local->mem = (char *)aligned_ptr;

                /* Optional but highly recommended: Zero out the freshly allocated memory block */
                memset(local->mem, 0, total_size);


/*              local->mem = (char *)malloc(max_bufs * buf_len);
                if (local->mem == NULL) {
                        result = DOCA_ERROR_NO_MEMORY;
                        DOCA_LOG_ERR("Unable to alloc memory to mmap: %s", doca_error_get_descr(result));
                        return result;
                }
*/
        }

        result = doca_buf_inventory_create(max_bufs, &(local->buf_inv));
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Unable to create inventory: %s", doca_error_get_descr(result));
                goto free_mem;
        }

        result = doca_buf_inventory_start(local->buf_inv);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Unable to start inventory: %s", doca_error_get_descr(result));
                goto destroy_inv;
        }

        result = doca_mmap_create(&local->mmap);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Unable to create mmap: %s", doca_error_get_descr(result));
                goto destroy_inv;
        }

        result = doca_mmap_add_dev(local->mmap, dev);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Unable to add device to mmap: %s", doca_error_get_descr(result));
                goto destroy_mmap;
        }

        result = doca_mmap_set_permissions(local->mmap, DOCA_ACCESS_FLAG_PCI_READ_WRITE);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Unable to set permission to mmap: %s", doca_error_get_descr(result));
                goto destroy_mmap;
        }

        result = doca_mmap_set_memrange(local->mmap, local->mem, max_bufs * buf_len);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Unable to set memrange to mmap: %s", doca_error_get_descr(result));
                goto destroy_mmap;
        }

        result = doca_mmap_start(local->mmap);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Unable to start mmap: %s", doca_error_get_descr(result));
                goto destroy_mmap;
        }

        return DOCA_SUCCESS;

destroy_mmap:
        doca_mmap_destroy(local->mmap);
        local->mmap = NULL;
destroy_inv:
        doca_buf_inventory_destroy(local->buf_inv);
        local->buf_inv = NULL;
free_mem:
        if (local->need_alloc_mem == true) {
                free(local->mem);
                local->mem = NULL;
        }
        return result;
}

void clean_comch_producer(struct doca_comch_producer *producer, struct doca_pe *pe)
{
        doca_error_t result;

        if (producer != NULL) {
                result = doca_comch_producer_destroy(producer);
                if (result != DOCA_SUCCESS)
                        DOCA_LOG_ERR("Failed to destroy producer properly with error=%s", doca_error_get_name(result));
        }

        if (pe != NULL) {
                result = doca_pe_destroy(pe);
                if (result != DOCA_SUCCESS)
                        DOCA_LOG_ERR("Failed to destroy pe properly with error=%s", doca_error_get_name(result));
        }
}

doca_error_t init_comch_producer(struct doca_comch_connection *connection,
                                 struct comch_producer_cb_config *cfg,
                                 struct doca_comch_producer **producer,
                                 struct doca_pe **pe)
{
        doca_error_t result;
        struct doca_ctx *ctx;
        union doca_data user_data;

        result = doca_pe_create(pe);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed creating pe with error = %s", doca_error_get_name(result));
                return result;
        }

        result = doca_comch_producer_create(connection, producer);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to create producer with error = %s", doca_error_get_name(result));
                goto destroy_pe;
        }

        ctx = doca_comch_producer_as_ctx(*producer);

        result = doca_pe_connect_ctx(*pe, ctx);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed adding pe context to producer with error = %s", doca_error_get_name(result));
                goto destroy_producer;
        }

        result = doca_ctx_set_state_changed_cb(ctx, cfg->ctx_state_changed_cb);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed setting state change callback with error = %s", doca_error_get_name(result));
                goto destroy_producer;
        }

        result = doca_comch_producer_task_send_set_conf(*producer,
                                                        cfg->send_task_comp_cb,
                                                        cfg->send_task_comp_err_cb,
                                                        CC_DATA_PATH_TASK_NUM);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed setting producer send task cbs with error = %s", doca_error_get_name(result));
                goto destroy_producer;
        }

        user_data.ptr = cfg->ctx_user_data;
        result = doca_ctx_set_user_data(ctx, user_data);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to set ctx user data with error = %s", doca_error_get_name(result));
                goto destroy_producer;
        }

        result = doca_ctx_start(ctx);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to start producer context with error = %s", doca_error_get_name(result));
                goto destroy_producer;
        }

        return DOCA_SUCCESS;

destroy_producer:
        doca_comch_producer_destroy(*producer);
        *producer = NULL;
destroy_pe:
        doca_pe_destroy(*pe);
        *pe = NULL;
        return result;
}

void clean_comch_consumer(struct doca_comch_consumer *consumer, struct doca_pe *pe)
{
        doca_error_t result;

        if (consumer != NULL) {
                result = doca_comch_consumer_destroy(consumer);
                if (result != DOCA_SUCCESS)
                        DOCA_LOG_ERR("Failed to destroy consumer properly with error = %s",
                                     doca_error_get_name(result));
        }

        if (pe != NULL) {
                result = doca_pe_destroy(pe);
                if (result != DOCA_SUCCESS)
                        DOCA_LOG_ERR("Failed to destroy pe properly with error = %s", doca_error_get_name(result));
        }
}

doca_error_t init_comch_consumer(struct doca_comch_connection *connection,
                                 struct doca_mmap *user_mmap,
                                 struct comch_consumer_cb_config *cfg,
                                 struct doca_comch_consumer **consumer,
                                 struct doca_pe **pe)
{
        doca_error_t result;
        struct doca_ctx *ctx;
        union doca_data user_data;

        result = doca_pe_create(pe);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed creating pe with error = %s", doca_error_get_name(result));
                return result;
        }

        result = doca_comch_consumer_create(connection, user_mmap, consumer);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to create consumer with error = %s", doca_error_get_name(result));
                goto destroy_pe;
        }

        ctx = doca_comch_consumer_as_ctx(*consumer);

        result = doca_pe_connect_ctx(*pe, ctx);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed adding pe context to server with error = %s", doca_error_get_name(result));
                goto destroy_consumer;
        }

        result = doca_ctx_set_state_changed_cb(ctx, cfg->ctx_state_changed_cb);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed setting state change callback with error = %s", doca_error_get_name(result));
                goto destroy_consumer;
        }

        result = doca_comch_consumer_task_post_recv_set_conf(*consumer,
                                                             cfg->recv_task_comp_cb,
                                                             cfg->recv_task_comp_err_cb,
                                                             CC_DATA_PATH_TASK_NUM);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed setting consumer recv task cbs with error = %s", doca_error_get_name(result));
                goto destroy_consumer;
        }

        user_data.ptr = cfg->ctx_user_data;
        result = doca_ctx_set_user_data(ctx, user_data);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to set ctx user data with error = %s", doca_error_get_name(result));
                goto destroy_consumer;
        }

        result = doca_ctx_start(ctx);
        if (result != DOCA_ERROR_IN_PROGRESS) {
                DOCA_LOG_ERR("Failed to start consumer context with error = %s", doca_error_get_name(result));
                goto destroy_consumer;
        }

        return DOCA_SUCCESS;

destroy_consumer:
        doca_comch_consumer_destroy(*consumer);
        *consumer = NULL;
destroy_pe:
        doca_pe_destroy(*pe);
        *pe = NULL;
        return result;
}

/**
 * Callback for producer send task successful completion
 *
 * @task [in]: Send task object
 * @task_user_data [in]: User data for task
 * @ctx_user_data [in]: User data for context
 */
static void producer_send_task_completion_callback(struct doca_comch_producer_task_send *task,
                                                   union doca_data task_user_data,
                                                   union doca_data ctx_user_data)
{
        struct comch_data_path_objects *data_path;
        const struct doca_buf *buf;

        (void)task_user_data;

        data_path = (struct comch_data_path_objects *)(ctx_user_data.ptr);
        data_path->producer_result = DOCA_SUCCESS;



        /* strcpy(data_path->text, CC_LDPC_REQ); */                                                             /* VBrusse */
        /* DOCA_LOG_INFO("Producer task enviou mensagem: %s", data_path->text);                                    VBrusse */
        /*
        DOCA_LOG_INFO("*** Input Block ===> %s", (char *)((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->inputBlock);
        DOCA_LOG_INFO("*** bg = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->bg);
        DOCA_LOG_INFO("*** z = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->z);
        DOCA_LOG_INFO("*** k = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->k);
        DOCA_LOG_INFO("*** len_filler_bits = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->len_filler_bits);
        */
        /* DOCA_LOG_INFO("\n\n\n*** Output Block ===> %s\n\n", (char *)((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->outputBlock); */            /* VBrusse */



        DOCA_LOG_INFO("Producer task sent successfully");

        buf = doca_comch_producer_task_send_get_buf(task);
        (void)doca_buf_dec_refcount((struct doca_buf *)buf, NULL);
        doca_task_free(doca_comch_producer_task_send_as_task(task));
        (void)doca_ctx_stop(doca_comch_producer_as_ctx(data_path->producer));
}

/**
 * Callback for producer send task completion with error
 *
 * @task [in]: Send task object
 * @task_user_data [in]: User data for task
 * @ctx_user_data [in]: User data for context
 */
static void producer_send_task_completion_err_callback(struct doca_comch_producer_task_send *task,
                                                       union doca_data task_user_data,
                                                       union doca_data ctx_user_data)
{
        struct comch_data_path_objects *data_path;
        const struct doca_buf *buf;

        (void)task_user_data;

        data_path = (struct comch_data_path_objects *)(ctx_user_data.ptr);
        data_path->producer_result = doca_task_get_status(doca_comch_producer_task_send_as_task(task));
        DOCA_LOG_ERR("Producer message failed to send with error = %s",
                     doca_error_get_name(data_path->producer_result));

        buf = doca_comch_producer_task_send_get_buf(task);
        (void)doca_buf_dec_refcount((struct doca_buf *)buf, NULL);
        doca_task_free(doca_comch_producer_task_send_as_task(task));
        (void)doca_ctx_stop(doca_comch_producer_as_ctx(data_path->producer));
}

/**
 * Use producers to send a msg
 *
 * @data_path [in]: CC data path resources
 * @return: DOCA_SUCCESS on success and DOCA_ERROR otherwise
 */
static doca_error_t producer_send_msg(struct comch_data_path_objects *data_path)
{
        struct doca_comch_producer_task_send *producer_task;
        struct doca_buf *buf;
        struct doca_task *task_obj;
        doca_error_t result;

        struct timespec ts = {
                .tv_sec = 0,
                .tv_nsec = SLEEP_IN_NANOS,
        };



        void *p_ldpc_data;                                      /* VBrusse */


        if (data_path->pldpc_enc_pars != NULL) {
                p_ldpc_data = data_path->pldpc_enc_pars;
        }

        if (data_path->pldpc_dec_pars != NULL) {
                p_ldpc_data = data_path->pldpc_dec_pars;
        }



        result = doca_buf_inventory_buf_get_by_data(data_path->producer_mem.buf_inv,
                                                    data_path->producer_mem.mmap,
                                                    /*(void *)(data_path->text)*/ /*(void *)(data_path->pldpc_enc_pars)*/ (void *)p_ldpc_data,
                                                    /* strnlen(data_path->text, CC_DATA_PATH_MAX_MSG_SIZE)*/ /*sizeof (struct ldpc_encod_params_t)*/ data_path->size_ldpc_data,
                                                    &buf);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to get doca buf from producer mmap with error = %s", doca_error_get_name(result));
                return result;
        }

        result = doca_comch_producer_task_send_alloc_init(data_path->producer,
                                                          buf,
                                                          NULL,
                                                          0,
                                                          data_path->remote_consumer_id,
                                                          &producer_task);
        if (result != DOCA_SUCCESS) {
                (void)doca_buf_dec_refcount(buf, NULL);
                DOCA_LOG_ERR("Failed to get allocate task from producer with error = %s", doca_error_get_name(result));
                return result;
        }

        task_obj = doca_comch_producer_task_send_as_task(producer_task);
        do {
                result = doca_task_submit(task_obj);
                if (result == DOCA_ERROR_AGAIN)
                        nanosleep(&ts, &ts);
        } while (result == DOCA_ERROR_AGAIN);
        if (result != DOCA_SUCCESS) {
                (void)doca_buf_dec_refcount(buf, NULL);
                doca_task_free(task_obj);
                DOCA_LOG_ERR("Failed submitting send task with error = %s", doca_error_get_name(result));
                return result;
        }

        return DOCA_SUCCESS;
}

/**
 * Callback triggered whenever CC producer context state changes
 *
 * @user_data [in]: User data associated with the CC producer context.
 * @ctx [in]: The CC client context that had a state change
 * @prev_state [in]: Previous context state
 * @next_state [in]: Next context state (context is already in this state when the callback is called)
 */
static void producer_state_changed_callback(const union doca_data user_data,
                                            struct doca_ctx *ctx,
                                            enum doca_ctx_states prev_state,
                                            enum doca_ctx_states next_state)
{
        (void)ctx;
        (void)prev_state;

        struct comch_data_path_objects *data_path = (struct comch_data_path_objects *)user_data.ptr;

        switch (next_state) {
        case DOCA_CTX_STATE_IDLE:
                DOCA_LOG_INFO("CC producer context has been stopped");
                /* We can stop progressing the PE */
                data_path->producer_finish = true;
                break;
        case DOCA_CTX_STATE_STARTING:
                /**
                 * The context is in starting state.
                 */
                DOCA_LOG_INFO("CC producer context entered into starting state");
                break;
        case DOCA_CTX_STATE_RUNNING:
                DOCA_LOG_INFO("CC producer context is running. Posting message to consumer, waiting finish");
                data_path->producer_result = producer_send_msg(data_path);
                if (data_path->producer_result != DOCA_SUCCESS) {
                        DOCA_LOG_ERR("Failed to submit producer send task with error = %s",
                                     doca_error_get_name(data_path->producer_result));
                        (void)doca_ctx_stop(doca_comch_producer_as_ctx(data_path->producer));
                }
                break;
        case DOCA_CTX_STATE_STOPPING:
                /**
                 * The context is in stopping, this can happen when fatal error encountered or when stopping context.
                 * doca_pe_progress() will cause all tasks to be flushed, and finally transition state to idle
                 */
                DOCA_LOG_INFO("CC producer context entered into stopping state");
                break;
        default:
                break;
        }
}

/**
 * Callback for consumer post recv task successful completion
 *
 * @task [in]: Recv task object
 * @task_user_data [in]: User data for task
 * @ctx_user_data [in]: User data for context
 */
static void consumer_recv_task_completion_callback(struct doca_comch_consumer_task_post_recv *task,
                                                   union doca_data task_user_data,
                                                   union doca_data ctx_user_data)
{
        struct comch_data_path_objects *data_path;
        size_t recv_msg_len;
        void *recv_msg;
        struct doca_buf *buf;

        (void)task_user_data;



        /* struct ldpc_encod_params_t *params;                          VBrusse */



        data_path = (struct comch_data_path_objects *)(ctx_user_data.ptr);

        buf = doca_comch_consumer_task_post_recv_get_buf(task);

        data_path->consumer_result = doca_buf_get_data(buf, &recv_msg);
        if (data_path->consumer_result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to get data address from DOCA buf with error = %s",
                             doca_error_get_name(data_path->consumer_result));
                goto err_out;
        }

        data_path->consumer_result = doca_buf_get_data_len(buf, &recv_msg_len);
        if (data_path->consumer_result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to get data length from DOCA buf with error = %s",
                             doca_error_get_name(data_path->consumer_result));
                goto err_out;
        }

        // DOCA_LOG_INFO("Message received: '%.*s'", (int)recv_msg_len, (char *)recv_msg);              // VBrusse: original code



        DOCA_LOG_INFO("*** Received message length = %ld", (long)recv_msg_len);
        /* params = (struct ldpc_encod_params_t *)recv_msg; */
        if (data_path->pldpc_enc_pars != NULL) {
                data_path->pldpc_enc_pars = (struct ldpc_encod_params_t *)recv_msg;
        }

        if (data_path->pldpc_dec_pars != NULL) {
                data_path->pldpc_dec_pars = (struct ldpc_decod_params_t *)recv_msg;
                // struct ldpc_decod_params_t *params = data_path->pldpc_dec_pars;

                DOCA_LOG_INFO("====================> LLRs <====================");

/*              for (int i= 0; i < params->n; i++) {
                        printf("%d ", params->llrs[i]);
                        // printf("%d ", *params->llrs + i));
                }
                printf("\n");
*/
                /*DOCA_LOG_INFO("*** bg = %d", ((struct ldpc_decod_params_t *)(data_path->pldpc_dec_pars))->bg);
                DOCA_LOG_INFO("*** z = %d", ((struct ldpc_decod_params_t *)(data_path->pldpc_dec_pars))->z);
                DOCA_LOG_INFO("*** num_its = %d", ((struct ldpc_decod_params_t *)(data_path->pldpc_dec_pars))->num_its);*/
        }



err_out:
        (void)doca_buf_dec_refcount(buf, NULL);
        doca_task_free(doca_comch_consumer_task_post_recv_as_task(task));
        (void)doca_ctx_stop(doca_comch_consumer_as_ctx(data_path->consumer));
}

/**
 * Callback for consumer post recv task completion with error
 *
 * @task [in]: Send task object
 * @task_user_data [in]: User data for task
 * @ctx_user_data [in]: User data for context
 */
static void consumer_recv_task_completion_err_callback(struct doca_comch_consumer_task_post_recv *task,
                                                       union doca_data task_user_data,
                                                       union doca_data ctx_user_data)
{
        struct comch_data_path_objects *data_path;
        struct doca_buf *buf;

        (void)task_user_data;

        data_path = (struct comch_data_path_objects *)(ctx_user_data.ptr);
        data_path->consumer_result = doca_task_get_status(doca_comch_consumer_task_post_recv_as_task(task));
        DOCA_LOG_ERR("Consumer failed to recv message with error = %s",
                     doca_error_get_name(data_path->consumer_result));

        buf = doca_comch_consumer_task_post_recv_get_buf(task);
        (void)doca_buf_dec_refcount(buf, NULL);
        doca_task_free(doca_comch_consumer_task_post_recv_as_task(task));
        (void)doca_ctx_stop(doca_comch_consumer_as_ctx(data_path->consumer));
}

/**
 * Use consumer to recv a msg
 *
 * @data_path [in]: CC data path resources
 * @return: DOCA_SUCCESS on success and DOCA_ERROR otherwise
 */
static doca_error_t consumer_recv_msg(struct comch_data_path_objects *data_path)
{
        struct doca_comch_consumer_task_post_recv *consumer_task;
        struct doca_buf *buf;
        struct doca_task *task_obj;
        doca_error_t result;

        /* Receive msg from server */
        result = doca_buf_inventory_buf_get_by_addr(data_path->consumer_mem.buf_inv,
                                                    data_path->consumer_mem.mmap,
                                                    data_path->consumer_mem.mem,
                                                    /*CC_DATA_PATH_MAX_MSG_SIZE*/data_path->size_ldpc_data,
                                                    &buf);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to get doca buf from consumer mmap with error = %s", doca_error_get_name(result));
                return result;
        }

        result = doca_comch_consumer_task_post_recv_alloc_init(data_path->consumer, buf, &consumer_task);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to allocate task for consumer with error = %s", doca_error_get_name(result));
                return result;
        }

        task_obj = doca_comch_consumer_task_post_recv_as_task(consumer_task);
        result = doca_task_submit(task_obj);
        if (result != DOCA_SUCCESS) {
                (void)doca_buf_dec_refcount(buf, NULL);
                doca_task_free(task_obj);
                DOCA_LOG_ERR("Failed submitting send task with error = %s", doca_error_get_name(result));
                return result;
        }

        return DOCA_SUCCESS;
}

/**
 * Callback triggered whenever CC consumer context state changes
 *
 * @user_data [in]: User data associated with the CC consumer context
 * @ctx [in]: The CC consumer context that had a state change
 * @prev_state [in]: Previous context state
 * @next_state [in]: Next context state (context is already in this state when the callback is called)
 */
static void consumer_state_changed_callback(const union doca_data user_data,
                                            struct doca_ctx *ctx,
                                            enum doca_ctx_states prev_state,
                                            enum doca_ctx_states next_state)
{
        (void)ctx;
        (void)prev_state;

        struct comch_data_path_objects *data_path = (struct comch_data_path_objects *)user_data.ptr;

        switch (next_state) {
        case DOCA_CTX_STATE_IDLE:
                DOCA_LOG_INFO("CC consumer context has been stopped");

                /* A move to stop from non running/stopping state means there's been an error */
                if ((prev_state != DOCA_CTX_STATE_RUNNING) && (prev_state != DOCA_CTX_STATE_STOPPING))
                        data_path->consumer_result = DOCA_ERROR_UNEXPECTED;

                /* We can stop progressing the PE */
                data_path->consumer_finish = true;
                break;
        case DOCA_CTX_STATE_STARTING:
                /**
                 * The context is in starting state.
                 */
                DOCA_LOG_INFO(
                        "CC consumer context entered into starting state. Waiting consumer producer negotiation finish");
                break;
        case DOCA_CTX_STATE_RUNNING:
                DOCA_LOG_INFO("CC consumer context is running. Receiving message from producer, waiting finish");
                data_path->consumer_result = consumer_recv_msg(data_path);
                if (data_path->consumer_result != DOCA_SUCCESS) {
                        DOCA_LOG_ERR("Failed to submit consumer recv task with error = %s",
                                     doca_error_get_name(data_path->consumer_result));
                        (void)doca_ctx_stop(doca_comch_consumer_as_ctx(data_path->consumer));
                }
                break;
        case DOCA_CTX_STATE_STOPPING:
                /**
                 * The context is in stopping, this can happen when fatal error encountered or when stopping context.
                 * doca_pe_progress() will cause all tasks to be flushed, and finally transition state to idle
                 */
                DOCA_LOG_INFO("CC consumer context entered into stopping state");
                break;
        default:
                break;
        }
}

doca_error_t comch_data_path_send_msg(struct comch_data_path_objects *data_path)
{
        doca_error_t result;
        struct local_mem_bufs *pmem = &data_path->producer_mem;
        struct timespec ts = {
                .tv_sec = 0,
                .tv_nsec = SLEEP_IN_NANOS,
        };
        struct comch_producer_cb_config producer_cb_cfg = {.send_task_comp_cb = producer_send_task_completion_callback,
                                                           .send_task_comp_err_cb =
                                                                   producer_send_task_completion_err_callback,
                                                           .ctx_user_data = data_path,
                                                           .ctx_state_changed_cb = producer_state_changed_callback};

        /* When remote_consumer_id != 1, it means the remote_consumer is ready to use */
        while (data_path->remote_consumer_id == 0) {
                if (doca_pe_progress(data_path->pe) == 0)
                        nanosleep(&ts, &ts);
        }

        if (data_path->remote_consumer_id == INVALID_CONSUMER_ID)
                return DOCA_ERROR_UNEXPECTED;

        /*      Original in comments by VBrusse
         * Need a cc producer to send message to server
         * Based on user input, to setup producer's mmap and doca_buf infrastructure
         *
        pmem->mem = (void *)data_path->text;
        pmem->need_alloc_mem = false;
        result = init_local_mem_bufs(pmem, data_path->hw_dev, strnlen(data_path->text, CC_DATA_PATH_MAX_MSG_SIZE), 1);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to producer memory with error = %s", doca_error_get_name(result));
                return result;
        }
        */



        /*       modified by VBrusse
         * Need a cc producer to send back the results of the LDPC decoder/encoder to client (host)
         * Based on results output, to setup producer's mmap and doca_buf infrastructure to send message to host
         */
        pmem->need_alloc_mem = false;


        if (data_path->pldpc_enc_pars != NULL) {
                pmem->mem = (struct ldpc_encod_params_t *)data_path->pldpc_enc_pars;                            // VBrusse
        }

        if (data_path->pldpc_dec_pars != NULL) {
                pmem->mem = (struct ldpc_decod_params_t *)data_path->pldpc_dec_pars;                            // VBrusse
        }


        /* Round up the data structure size to the nearest 64-byte cache line boundary */
        size_t aligned_size = (data_path->size_ldpc_data + 63) & ~63;

        // result = init_local_mem_bufs(pmem, data_path->hw_dev, //sizeof(struct ldpc_encod_params_t)// data_path->size_ldpc_data, 1);           // VBrusse
        result = init_local_mem_bufs(pmem, data_path->hw_dev, aligned_size, 1);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to producer memory with error = %s", doca_error_get_name(result));
                return result;
        }


        /*
         * Before this line runs, data_path->pldpc_enc_pars points to a temporary piece of memory (likely on the local heap or stack) where your host data was temporarily staged.
         *
         * cmem->mem is the official, high-speed, hardware-mapped memory block that DOCA’s internal allocator just provisioned for the DPU to share with the host.
         *
         * This line physically copies the raw bits of your configuration parameters (like code block size, lifting size, base graph, etc.) out of that temporary staging area and
         * drops them straight into the official DOCA shared memory buffer. It updates the address stored inside data_path->pldpc_enc_pars so that it now points directly to cmem->mem.
         *
         * Why this is critical: Later on in your execution pipeline (when the DPU actually calls the underlying accelerator hardware or processes a packet), the rest of your program
         * is going to look at data_path->pldpc_enc_pars to find the execution parameters.
         *
         * By updating this tracking pointer, you ensure that the rest of your system is reading the parameters from the true, hardware-aligned DOCA memory block, rather than trying
         * to look back at the old, unaligned staging area.
         */

        /* Now safely copy your parameters from data_path into the aligned memory segment */
        /* Shared Memory - data_path->consumer_mem = data_path->producer_mem */
        if (data_path->pldpc_enc_pars != NULL) {                                                                // VBrusse
                memcpy(pmem->mem, data_path->pldpc_enc_pars, sizeof(struct ldpc_encod_params_t));               // Copy the host RAM to shared memory pmem (mmap)
                data_path->pldpc_enc_pars = pmem->mem;                                                          // Point the host RAM pointer to shared memory pmem->mem
        }

        if (data_path->pldpc_dec_pars != NULL) {                                                                // VBrusse
                memcpy(pmem->mem, data_path->pldpc_dec_pars, sizeof(struct ldpc_decod_params_t));
                data_path->pldpc_dec_pars = pmem->mem;
        }



        /* Init a cc producer */
        result = init_comch_producer(data_path->connection,
                                     &producer_cb_cfg,
                                     &(data_path->producer),
                                     &(data_path->producer_pe));
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to init a producer with error = %s", doca_error_get_name(result));
                clean_local_mem_bufs(pmem);
                return result;
        }

        /* Send msg to server */
        while (data_path->producer_finish == false) {
                if (doca_pe_progress(data_path->producer_pe) == 0)
                        nanosleep(&ts, &ts);
        }



        /* DOCA_LOG_INFO("Mensagem enviada: %s", data_path->text); */                    /* VBrusse */
        /* VBrusse */
        /* DOCA_LOG_INFO("\n\n\n*** Producer Shared Memory - data_path->producer_mem->mem ===> %s", (char *)data_path->producer_mem.mem);
        DOCA_LOG_INFO("\n\n\n*** Consumer Shared Memory - data_path->consumer_mem->mem ===> %s", (char *)data_path->consumer_mem.mem);          VBrusse */

/*      DOCA_LOG_INFO("\n\n\n*** Estou no send_msg *** Input Block ===> %s", (char *)((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->inputBlock);
        DOCA_LOG_INFO("*** bg = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->bg);
        DOCA_LOG_INFO("*** z = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->z);
        DOCA_LOG_INFO("*** k = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->k);
        DOCA_LOG_INFO("*** len_filler_bits = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->len_filler_bits);

        DOCA_LOG_INFO("\n*** Estou no send_msg *** Output Block ===> %s\n\n", (char *)((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->outputBlock); VBrusse
*/


        /* VBrusse - Free producer's mmap and doca_buf resources - Original */
        /* clean_comch_producer(data_path->producer, data_path->producer_pe);
        data_path->producer = NULL;
        data_path->producer_pe = NULL;
        clean_local_mem_bufs(pmem); */



        return data_path->producer_result;
}

doca_error_t comch_data_path_recv_msg(struct comch_data_path_objects *data_path)
{
        doca_error_t result;
        struct local_mem_bufs *cmem = &data_path->consumer_mem;
        struct timespec ts = {
                .tv_sec = 0,
                .tv_nsec = SLEEP_IN_NANOS,
        };
        struct comch_consumer_cb_config consumer_cb_cfg = {.recv_task_comp_cb = consumer_recv_task_completion_callback,
                                                           .recv_task_comp_err_cb =
                                                                   consumer_recv_task_completion_err_callback,
                                                           .ctx_user_data = data_path,
                                                           .ctx_state_changed_cb = consumer_state_changed_callback};
        /*      Original in comments by VBrusse
         * Need a cc consumer to recv message from server
         * Setup consumer's mmap and doca_buf infrastructure
         *
        cmem->need_alloc_mem = true;
        result = init_local_mem_bufs(cmem, data_path->hw_dev, CC_DATA_PATH_MAX_MSG_SIZE, 1);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to consumer memory with error = %s", doca_error_get_name(result));
                return result;
        }
        */



        //DOCA_LOG_INFO("data_path = %p", data_path);
        //DOCA_LOG_INFO("data_path->hw_dev = %p", data_path->hw_dev);
        //DOCA_LOG_INFO("data_path->consumer_mem = %p", &data_path->consumer_mem);
        //DOCA_LOG_INFO("data_path->size_ldpc_data = %d", data_path->size_ldpc_data);
        //DOCA_LOG_INFO("data_path->pe = %p", data_path->pe);
        //DOCA_LOG_INFO("data_path->consumer_pe = %p", data_path->consumer_pe);

        /*      modified by VBrusse
         * Need a cc consumer to recv message (task) from client (host)
         * Setup consumer's mmap and doca_buf infrastructure for input parameters of LDPC decoder/encoder to recv message from host
         */
        cmem->need_alloc_mem = true;


        if (data_path->pldpc_enc_pars != NULL) {
                cmem->mem = (struct ldpc_encod_params_t *)data_path->pldpc_enc_pars;                            // VBrusse
        }

        if (data_path->pldpc_dec_pars != NULL) {
                cmem->mem = (struct ldpc_decod_params_t *)data_path->pldpc_dec_pars;                            // VBrusse
        }


        /* Round up the data structure size to the nearest 64-byte cache line boundary */
        size_t aligned_size = (data_path->size_ldpc_data + 63) & ~63;

        // result = init_local_mem_bufs(cmem, data_path->hw_dev, //sizeof(struct ldpc_encod_params_t)// data_path->size_ldpc_data, 1);
        result = init_local_mem_bufs(cmem, data_path->hw_dev, aligned_size, 1);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to consumer memory with error = %s", doca_error_get_name(result));
                return result;
        }


        /*
         * Before this line runs, data_path->pldpc_enc_pars points to a temporary piece of memory (likely on the local heap or stack) where your host data was temporarily staged.
         *
         * cmem->mem is the official, high-speed, hardware-mapped memory block that DOCA’s internal allocator just provisioned for the DPU to share with the host.
         *
         * This line physically copies the raw bits of your configuration parameters (like code block size, lifting size, base graph, etc.) out of that temporary staging area and
         * drops them straight into the official DOCA shared memory buffer. It updates the address stored inside data_path->pldpc_enc_pars so that it now points directly to cmem->mem.
         *
         * Why this is critical: Later on in your execution pipeline (when the DPU actually calls the underlying accelerator hardware or processes a packet), the rest of your program
         * is going to look at data_path->pldpc_enc_pars to find the execution parameters.
         *
         * By updating this tracking pointer, you ensure that the rest of your system is reading the parameters from the true, hardware-aligned DOCA memory block, rather than trying
         * to look back at the old, unaligned staging area.
         */

        /* Now safely copy your parameters from data_path into the aligned memory segment */
        /* Shared Memory - data_path->consumer_mem = data_path->producer_mem */
        if (data_path->pldpc_enc_pars != NULL) {                                                                // VBrusse
                memcpy(cmem->mem, data_path->pldpc_enc_pars, sizeof(struct ldpc_encod_params_t));               // Copy the host RAM to shared memory cmem (mmap)
                data_path->pldpc_enc_pars = cmem->mem;                                                          // Point the host RAM pointer to shared memory cmem->mem
        }

        if (data_path->pldpc_dec_pars != NULL) {                                                                // VBrusse
                memcpy(cmem->mem, data_path->pldpc_dec_pars, sizeof(struct ldpc_decod_params_t));
                data_path->pldpc_dec_pars = cmem->mem;
        }



        /* Init a consumer */
        result = init_comch_consumer(data_path->connection,
                                     cmem->mmap,
                                     &consumer_cb_cfg,
                                     &(data_path->consumer),
                                     &(data_path->consumer_pe));
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to init a consumer with error = %s", doca_error_get_name(result));
                clean_local_mem_bufs(cmem);
                return result;
        }

        /* Receive msg from server */
        while (data_path->consumer_finish == false) {
                if (doca_pe_progress(data_path->pe) == 0)
                        nanosleep(&ts, &ts);
                if (doca_pe_progress(data_path->consumer_pe) == 0)
                        nanosleep(&ts, &ts);
        }



/*      DOCA_LOG_INFO("*** Input Block before ldpc callbck ===> %s\n\n\n", (char *)((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->inputBlock); VBrusse */
/*      DOCA_LOG_INFO("\n\n\n*** Consumer Shared Memory - cmem->mem ===> %s", (char *)cmem->mem);               VBrusse
        DOCA_LOG_INFO("\n\n\n*** Input Block ===> %s", (char *)((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->inputBlock);
        DOCA_LOG_INFO("*** bg = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->bg);
        DOCA_LOG_INFO("*** z = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->z);
        DOCA_LOG_INFO("*** k = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->k);
        DOCA_LOG_INFO("*** len_filler_bits = %d\n\n", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->len_filler_bits);

        DOCA_LOG_INFO("\n\n\n*** Consumer Shared Memory - data_path->consumer_mem->mem ===> %s", (char *)data_path->consumer_mem.mem);          VBrusse
        DOCA_LOG_INFO("\n\n\n*** Producer Shared Memory - data_path->producer_mem->mem ===> %s", (char *)data_path->producer_mem.mem);
*/
/*      DOCA_LOG_INFO("\n\n\n*** Input Block before ldpc call ===> %s", (char *)((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->inputBlock);       VBrusse */
/*      DOCA_LOG_INFO("Mensagem recebida: ====================> LLRs <====================");
        for (int i= 0; i < 12; i++) {
                printf("%d ", ((struct ldpc_decod_params_t *)(data_path->pldpc_dec_pars))->llrs[i]);
        }
        printf("\n");
        DOCA_LOG_INFO("*** bg = %d", ((struct ldpc_decod_params_t *)(data_path->pldpc_dec_pars))->bg);
        DOCA_LOG_INFO("*** z = %d", ((struct ldpc_decod_params_t *)(data_path->pldpc_dec_pars))->z);
        DOCA_LOG_INFO("*** num_its = %d", ((struct ldpc_decod_params_t *)(data_path->pldpc_dec_pars))->num_its);
*/
/*      DOCA_LOG_INFO("*** len_filler_bits = %d\n\n", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->len_filler_bits);    */

        /* After receiving and processing the message, check the callback */            /* VBrusse */
        /* Change the comch_data_path_recv_msg function to accept the callback and invoke it after processing the received message. */

        if (data_path->ldpc_enc_callback != NULL) {

                // DOCA_LOG_INFO("About to call ldpc_encoder_callback in comch_data_path_recv_msg");

                doca_error_t result = data_path->ldpc_enc_callback(data_path->pldpc_enc_pars); // trigger the LDPC encoder callback, only when the server has received msg from the client

                if (result == DOCA_SUCCESS) {
                        DOCA_LOG_INFO("LDPC encoder callback has been triggered successfully");
                } else {
                        DOCA_LOG_INFO("Failed to execute the LDPC encoder callback");
                }
        }/* else {
                DOCA_LOG_INFO("No LDPC encoder callback registered");
        } */

        if (data_path->ldpc_dec_callback != NULL) {

                // DOCA_LOG_INFO("About to call ldpc_decoder_callback in comch_data_path_recv_msg");


                /* DOCA_LOG_INFO("***** bg = %d\n\n", ((struct ldpc_decod_params_t *)(data_path->pldpc_dec_pars))->bg); */


                doca_error_t result = data_path->ldpc_dec_callback(data_path->pldpc_dec_pars); // trigger the LDPC decoder callback, only when the server has received msg from the client

                if (result == DOCA_SUCCESS) {
                        DOCA_LOG_INFO("LDPC decoder callback has been triggered successfully");
                } else {
                        DOCA_LOG_INFO("Failed to execute the LDPC decoder callback");
                }
        }/* else {
                DOCA_LOG_INFO("No LDPC decoder callback registered");
        } */


        /* VBrusse - Free consumer's mmap and doca_buf resources - Original */
        /* clean_comch_consumer(data_path->consumer, data_path->consumer_pe);
        data_path->consumer = NULL;
        data_path->consumer_pe = NULL;
        clean_local_mem_bufs(cmem); */

        return data_path->consumer_result;
}
