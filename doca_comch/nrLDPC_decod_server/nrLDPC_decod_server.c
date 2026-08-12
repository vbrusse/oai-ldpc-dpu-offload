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
 * Original filename: comch_data_path_high_speed_server_sample.c
 *
 * Filename: nrLDPC_decod_server.c
 *
 * DOCA Communication Channel Server API customized by: Vlademir Brusse
 *
 * Date: 2025/10/14
 *
 */

#include <signal.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <doca_buf.h>
#include <doca_buf_inventory.h>
#include <doca_comch.h>
#include <doca_comch_consumer.h>
#include <doca_comch_producer.h>
#include <doca_ctx.h>
#include <doca_dev.h>
#include <doca_error.h>
#include <doca_log.h>
#include <doca_mmap.h>
#include <doca_pe.h>

#include <errno.h>                                      /* VBrusse */
#include <unistd.h>                                     /* VBrusse - for usleep */
#include <sys/times.h>                                  /* VBrusse - Needed for struct tms and times() */
#include <armral.h>                                     /* VBrusse */
/* #include <semaphore.h> */                            // VBrusse

/* #include "comch_ctrl_path_common.h" */
#include "nrLDPC_ctrl_path_common.h"
#include "nrLDPC_data_path_high_speed_common.h"
#include "common.h"


#define FILLER_LLR_VAL 127                              /* Strong '0' */


DOCA_LOG_REGISTER(NRLDPC_DECOD_SERVER);

/* Sample's objects */
struct comch_data_path_server_objects {
        struct doca_dev *hw_dev;                   /* Device used in the sample */
        struct doca_dev_rep *rep_dev;              /* Device representor used in the sample */
        struct doca_pe *pe;                        /* PE object used in the sample */
        struct doca_comch_server *server;          /* Server object used in the sample*/
        struct doca_comch_connection *connection;  /* Connection object used in the sample*/
        doca_error_t server_result;                /* Holds result will be updated in server callbacks */
        bool server_finish;                        /* Controls whether server progress loop should be run */
        bool data_path_test_started;               /* Indicate whether we can start data_path test */
        bool data_path_test_stopped;               /* Indicate whether we can stop data_path test */
        struct comch_data_path_objects *data_path; /* Data path objects */
};



                                                        /* Why "static":
                                                           Lifetime: Entire program duration (like global variables)
                                                           Scope: Limited to the block or file where declared */
static int task_counter = 1;                            /* Global task counter */
sem_t request_done;

/* struct tms buffer;                                   // used for CPU ticks
clock_t start_ticks, end_ticks;

uint64_t start_ns, end_ns, cycles;
double cpu_freq_hz = 2200000000.0;
*/
static struct timespec ts_start, ts_end;
static struct timespec wall_start, wall_end;



/**
 * "armral_ldpc_decode_block" - Performs decoding of LDPC using a layered min-sum algorithm. This is an iterative algorithm which takes 8-bit log-likelihood ratios (LLRs) and calculates
 * the most likely codeword by calculating updates using information available from the parity checks in the LDPC graph. LLRs are updated after evaluating checks in a 'layer', where a
 * layer is assumed to contain the same number of checks as the lifting size z. There are 46 layers in base graph 1, and 42 layers in base graph 2. Decoding is performed for a single
 * code block.
 *
 * Release: armral v26.01
 *
 * Parameters
 *
 * n [in]
 *      A read-only parameter of type uint32_t.
 *
 *      The length of llrs, subject to the following constraints. Let m := n when using explicit filler bits (default), and m := n + len_filler_bits when using implicit filler bits. For
 *      base graph 1, m must be greater than 20 * z and less than or equal to 66 * z. For base graph 2, m must be greater than 8 * z and less than or equal to 50 * z.
 *
 * llrs [in]
 *      A read-only parameter of type const int8_t *.
 *
 *      The initial LLRs to use in the decoding. This is typically the output after demodulation and rate recovery.
 *
 * bg [in]
 *      A read-only parameter of type armral_ldpc_graph_t.
 *
 *      The type of base graph to use for the decoding.
 *
 * z [in]
 *      A read-only parameter of type uint32_t.
 *
 *      The lifting size. Valid values of the lifting size are described in table 5.3.2-1 in TS 38.212.
 *
 * len_filler_bits [in]
 *      A read-only parameter of type uint32_t.
 *
 *      The number of filler bits. As per TS 38.212, section 5.2.2, filler bits insertion is needed to ensure that the code block segments have a valid length and are a multiple of the
 *      lifting size. Filler bits are used to calculate CRC internally. For base graph 1, len_filler_bits must be less than 20 * z , for base graph 2, it must be less than 8 * z.
 *
 * data_out [out]
 *      A write-only parameter of type uint8_t *.
 *
 *      The decoded bits. These are of length 22 * z for base graph 1 and 10 * z for base graph 2. It is assumed that the array data_out is able to store this many bits.
 *
 * its_max [in]
 *      A read-only parameter of type uint32_t.
 *
 *      The maximum number of iterations of the LDPC decoder. The algorithm may terminate after fewer iterations if the current candidate codeword passes all the parity checks, or if
 *      it satisfies the CRC check.
 *
 * its_out [out]
 *      A write-only parameter of type uint32_t *.
 *
 *      Number of iterations taken to complete LDPC Decoding.
 *
 * options [in]
 *      A read-only parameter of type uint32_t.
 *
 *      See the documentation above for a summary of available options. If you want to use the default options, set the options parameter to either 0 or ARMRAL_LDPC_DEFAULT_OPTIONS.
 *
 * Returns
 *
 * Returns ARMRAL_SUCCESS on success, ARMRAL_ARGUMENT_ERROR if an input parameter is incorrect, or ARMRAL_FAIL if the CRC check for convergence fails.
 *
 */
armral_status armral_ldpc_decode_block(uint32_t n, const int8_t *llrs,
                                       armral_ldpc_graph_t bg, uint32_t z,
                                       uint32_t len_filler_bits,
                                       uint8_t *data_out, uint32_t its_max,
                                       uint32_t *its_out, uint32_t options);

/* // Release: 26.01
armral_status armral_ldpc_rate_recovery(armral_ldpc_graph_t bg, uint32_t z,
                                        uint32_t e, uint32_t nref,
                                        uint32_t len_filler_bits, uint32_t k,
                                        uint32_t rv, armral_modulation_type mod,
                                        const int8_t *src, int8_t *dst);
*/



/**
 * Callback for server send task successful completion
 *
 * @task [in]: Send task object
 * @task_user_data [in]: User data for task
 * @ctx_user_data [in]: User data for context
 */
static void server_send_task_completion_callback(struct doca_comch_task_send *task,
                                                 union doca_data task_user_data,
                                                 union doca_data ctx_user_data)
{
        struct comch_data_path_server_objects *sample_objects;

        (void)task_user_data;

        sample_objects = (struct comch_data_path_server_objects *)ctx_user_data.ptr;
        sample_objects->server_result = DOCA_SUCCESS;



/*      DOCA_LOG_INFO("Server enviou mensagem ao Client = %s", sample_objects->data_path->text);          VBrusse */



        DOCA_LOG_INFO("Server task sent successfully");
        doca_task_free(doca_comch_task_send_as_task(task));
}

/**
 * Callback for server send task completion with error
 *
 * @task [in]: Send task object
 * @task_user_data [in]: User data for task
 * @ctx_user_data [in]: User data for context
 */
static void server_send_task_completion_err_callback(struct doca_comch_task_send *task,
                                                     union doca_data task_user_data,
                                                     union doca_data ctx_user_data)
{
        struct comch_data_path_server_objects *sample_objects;

        (void)task_user_data;

        sample_objects = (struct comch_data_path_server_objects *)ctx_user_data.ptr;
        sample_objects->server_result = doca_task_get_status(doca_comch_task_send_as_task(task));
        DOCA_LOG_ERR("Message failed to send with error = %s", doca_error_get_name(sample_objects->server_result));
        doca_task_free(doca_comch_task_send_as_task(task));
        (void)doca_ctx_stop(doca_comch_server_as_ctx(sample_objects->server));
}

/**
 * Server sends a message to client
 *
 * @sample_objects [in]: The sample object to use
 * @msg [in]: The msg to send
 * @len [in]: The msg length
 * @return: DOCA_SUCCESS on success and DOCA_ERROR otherwise
 */
static doca_error_t server_send_msg(struct comch_data_path_server_objects *sample_objects, const char *msg, size_t len)
{
        doca_error_t result;
        struct doca_comch_task_send *task;



        /* strcpy((char *)msg, CC_LDPC_RES);                                                      VBrusse */
        /* DOCA_LOG_INFO("Servidor enviou essa mensagem: '%.*s'", (int)len, msg);                 VBrusse inseriu */



        result = doca_comch_server_task_send_alloc_init(sample_objects->server,
                                                        sample_objects->connection,
                                                        (void *)msg,                            /* Formata msg para o client em msg */
                                                        len,
                                                        &task);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to allocate server task with error = %s", doca_error_get_name(result));
                return result;
        }

        result = doca_task_submit(doca_comch_task_send_as_task(task));                          /* Server envia msg para o Client */
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to send server task with error = %s", doca_error_get_name(result));
                doca_task_free(doca_comch_task_send_as_task(task));
                return result;
        }

        return DOCA_SUCCESS;
}

/**
 * Callback for server message recv event
 *
 * @event [in]: Recv event object
 * @recv_buffer [in]: Message buffer
 * @msg_len [in]: Message len
 * @comch_connection [in]: Connection the message was received on
 */
static void server_message_recv_callback(struct doca_comch_event_msg_recv *event,
                                         uint8_t *recv_buffer,
                                         uint32_t msg_len,
                                         struct doca_comch_connection *comch_connection)
{
        union doca_data user_data;
        struct doca_comch_server *comch_server;
        struct comch_data_path_server_objects *sample_objects;
        doca_error_t result;

        (void)event;

        DOCA_LOG_INFO("Message received: '%.*s'", (int)msg_len, recv_buffer);

        comch_server = doca_comch_server_get_server_ctx(comch_connection);

        result = doca_ctx_get_user_data(doca_comch_server_as_ctx(comch_server), &user_data);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to get user data from ctx with error = %s", doca_error_get_name(result));
                return;
        }

        sample_objects = (struct comch_data_path_server_objects *)user_data.ptr;
        sample_objects->connection = comch_connection;

        if ((msg_len == strlen(STR_START_DATA_PATH_TEST)) &&
            (strncmp(STR_START_DATA_PATH_TEST, (char *)recv_buffer, msg_len) == 0)) {
                result = server_send_msg(sample_objects, STR_START_DATA_PATH_TEST, strlen(STR_START_DATA_PATH_TEST));
                if (result != DOCA_SUCCESS) {
                        DOCA_LOG_ERR("Failed to submit send task with error = %s", doca_error_get_name(result));
                        (void)doca_ctx_stop(doca_comch_server_as_ctx(sample_objects->server));
                        return;
                }
                sample_objects->data_path_test_started = true;
        } else if ((msg_len == strlen(STR_STOP_DATA_PATH_TEST)) &&
                   (strncmp(STR_STOP_DATA_PATH_TEST, (char *)recv_buffer, msg_len) == 0)) {


                // Ignore stale stop if producer is still active (data path not done yet)
                if (sample_objects->data_path->producer_finish == false &&                                      // VBrusse has added this check
                    sample_objects->data_path->remote_consumer_id == 0) {    // consumer not yet registered
                        DOCA_LOG_INFO("Ignoring stale stop_data_path_test - producer not ready");
                        return;
                }


                result = server_send_msg(sample_objects, STR_STOP_DATA_PATH_TEST, strlen(STR_STOP_DATA_PATH_TEST));
                if (result != DOCA_SUCCESS) {
                        DOCA_LOG_ERR("Failed to submit send task with error = %s", doca_error_get_name(result));
                        (void)doca_ctx_stop(doca_comch_server_as_ctx(sample_objects->server));
                        return;
                }
                sample_objects->data_path_test_stopped = true;
                sample_objects->data_path->remote_consumer_id = INVALID_CONSUMER_ID;
                (void)doca_ctx_stop(doca_comch_server_as_ctx(sample_objects->server));
        }
}

/**
 * Callback for connection event
 *
 * @event [in]: Connection event object
 * @comch_connection [in]: Connection object
 * @change_success [in]: Whether the connection was successful or not
 */
static void server_connection_event_callback(struct doca_comch_event_connection_status_changed *event,
                                             struct doca_comch_connection *comch_connection,
                                             uint8_t change_success)
{
        union doca_data user_data;
        struct doca_comch_server *comch_server;
        struct comch_data_path_server_objects *sample_objects;
        doca_error_t result;

        if (change_success == 0) {
                DOCA_LOG_ERR("Failed connection received");
                return;
        }

        (void)event;

        comch_server = doca_comch_server_get_server_ctx(comch_connection);

        result = doca_ctx_get_user_data(doca_comch_server_as_ctx(comch_server), &user_data);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to get user data from ctx with error = %s", doca_error_get_name(result));
                return;
        }

        sample_objects = (struct comch_data_path_server_objects *)user_data.ptr;
        sample_objects->connection = comch_connection;
}

/**
 * Callback for disconnection event
 *
 * @event [in]: Connection event object
 * @comch_connection [in]: Connection object
 * @change_success [in]: Whether the disconnection was successful or not
 */
static void server_disconnection_event_callback(struct doca_comch_event_connection_status_changed *event,
                                                struct doca_comch_connection *comch_connection,
                                                uint8_t change_success)
{
        (void)event;
        (void)comch_connection;

        if (change_success == 0)
                DOCA_LOG_ERR("Failed disconnection received");
}

/**
 * Callback triggered whenever CC server context state changes
 *
 * @user_data [in]: User data associated with the CC server context.
 * @ctx [in]: The CC server context that had a state change
 * @prev_state [in]: Previous context state
 * @next_state [in]: Next context state (context is already in this state when the callback is called)
 */
static void server_state_changed_callback(const union doca_data user_data,
                                          struct doca_ctx *ctx,
                                          enum doca_ctx_states prev_state,
                                          enum doca_ctx_states next_state)
{
        (void)ctx;
        (void)prev_state;

        struct comch_data_path_server_objects *sample_objects = (struct comch_data_path_server_objects *)user_data.ptr;

        switch (next_state) {
        case DOCA_CTX_STATE_IDLE:
                DOCA_LOG_INFO("CC server context has been stopped");
                /* We can stop progressing the PE */
                sample_objects->server_finish = true;
                break;
        case DOCA_CTX_STATE_STARTING:
                /**
                 * The context is in starting state, this is unexpected for CC server.
                 */
                DOCA_LOG_ERR("CC server context entered into starting state. Unexpected transition");
                break;
        case DOCA_CTX_STATE_RUNNING:
                DOCA_LOG_INFO("CC server context is running. Waiting for clients to connect");
                break;
        case DOCA_CTX_STATE_STOPPING:
                /**
                 * The context is in stopping, this can happen when fatal error encountered or when stopping context.
                 * doca_pe_progress() will cause all tasks to be flushed, and finally transition state to idle
                 */
                DOCA_LOG_INFO("CC server context entered into stopping state. Terminating connections with clients");
                break;
        default:
                break;
        }
}

/**
 * Callback for new consumer arrival event
 *
 * @event [in]: New remote consumer event object
 * @comch_connection [in]: The connection related to the consumer
 * @id [in]: The ID of the new remote consumer
 */
static void new_consumer_callback(struct doca_comch_event_consumer *event,
                                  struct doca_comch_connection *comch_connection,
                                  uint32_t id)
{
        union doca_data user_data;
        struct doca_comch_server *comch_server;
        struct comch_data_path_server_objects *sample_objects;
        doca_error_t result;

        /* This argument is not in use */
        (void)event;

        comch_server = doca_comch_server_get_server_ctx(comch_connection);

        result = doca_ctx_get_user_data(doca_comch_server_as_ctx(comch_server), &user_data);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to get user data from ctx with error = %s", doca_error_get_name(result));
                return;
        }

        sample_objects = (struct comch_data_path_server_objects *)(user_data.ptr);
        sample_objects->data_path->remote_consumer_id = id;

        DOCA_LOG_INFO("Got a new remote consumer with ID = [%d]", id);
}

/**
 * Callback for expired consumer arrival event
 *
 * @event [in]: Expired remote consumer event object
 * @comch_connection [in]: The connection related to the consumer
 * @id [in]: The ID of the expired remote consumer
 */
void expired_consumer_callback(struct doca_comch_event_consumer *event,
                               struct doca_comch_connection *comch_connection,
                               uint32_t id)
{
        /* These arguments are not in use */
        (void)event;
        (void)comch_connection;
        (void)id;
}

/**
 * Clean all sample resources
 *
 * @sample_objects [in]: Sample objects struct to clean
 */
static void clean_comch_data_path_server_objects(struct comch_data_path_server_objects *sample_objects)
{
        doca_error_t result;
        struct timespec ts = {
                .tv_sec = 0,
                .tv_nsec = SLEEP_IN_NANOS,
        };

        if (sample_objects == NULL)
                return;

        /* Exchange message with server to make connection is reliable */
        while (sample_objects->data_path_test_stopped == false) {
                if (doca_pe_progress(sample_objects->pe) == 0)
                        nanosleep(&ts, &ts);
        }
        while (sample_objects->server_finish == false) {
                if (doca_pe_progress(sample_objects->pe) == 0)
                        nanosleep(&ts, &ts);
        }

        clean_comch_ctrl_path_server(sample_objects->server, sample_objects->pe);
        sample_objects->server = NULL;
        sample_objects->pe = NULL;

        if (sample_objects->rep_dev != NULL) {
                result = doca_dev_rep_close(sample_objects->rep_dev);
                if (result != DOCA_SUCCESS)
                        DOCA_LOG_ERR("Failed to close rep device properly with error = %s",
                                     doca_error_get_name(result));

                sample_objects->rep_dev = NULL;
        }

        if (sample_objects->hw_dev != NULL) {
                result = doca_dev_close(sample_objects->hw_dev);
                if (result != DOCA_SUCCESS)
                        DOCA_LOG_ERR("Failed to close hw device properly with error = %s", doca_error_get_name(result));

                sample_objects->hw_dev = NULL;
        }
}



/* VBrusse
 * terminate_comch_data_path_decod_server - This function frees resources allocated by DPU (server) used for an LDPC offloading.
 *
 */
void terminate_comch_data_path_decod_server(struct comch_data_path_objects *data_path)
{
        struct timespec ts = {
                .tv_sec = 0,
                .tv_nsec = SLEEP_IN_NANOS,
        };

        // Wait for producer to fully stop before destroying
        if (data_path->producer_pe != NULL) {
                while (data_path->producer_finish == false) {
                        if (doca_pe_progress(data_path->producer_pe) == 0)
                                nanosleep(&ts, &ts);
                }
                doca_pe_progress(data_path->producer_pe);
                doca_pe_progress(data_path->producer_pe);
        }

        /* VBrusse - Free producer's mmap and doca_buf resources - Original */
        clean_comch_producer(data_path->producer, data_path->producer_pe);
        data_path->producer = NULL;
        data_path->producer_pe = NULL;
        clean_local_mem_bufs(&data_path->producer_mem);


        /* code added by VBrusse - Free consumer's mmap and doca_buf resources */
        clean_comch_consumer(data_path->consumer, data_path->consumer_pe);
        data_path->consumer = NULL;
        data_path->consumer_pe = NULL;
        clean_local_mem_bufs(&data_path->consumer_mem);
}


/*
uint64_t get_time_ns()
{

        struct timespec ts;

        clock_gettime(CLOCK_MONOTONIC_RAW, &ts);

        return ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

void start_cpu_time()
{

        start_ticks = times(&buffer);
        start_ns = get_time_ns();
}

void end_cpu_time()
{

        end_ticks = times(&buffer);
        end_ns = get_time_ns();
        cycles = (end_ns - start_ns) * (cpu_freq_hz / 1e9);                             // Convert time (nanoseconds) to cycles

        DOCA_LOG_INFO("start_ticks: %ld", start_ticks);
        DOCA_LOG_INFO("end_ticks: %ld", end_ticks);
        DOCA_LOG_INFO("CPU Ticks elapsed: %ld", end_ticks - start_ticks);

        long ticks_per_sec = sysconf(_SC_CLK_TCK);                                      // Get clock ticks per seconds

        DOCA_LOG_INFO("User time: %lf us", (double)buffer.tms_utime * 1000.0 / ticks_per_sec);
        DOCA_LOG_INFO("System time: %lf us", (double)buffer.tms_stime * 1000.0 / ticks_per_sec);
        DOCA_LOG_INFO("Elapsed time: %lf us", (double)(end_ticks - start_ticks) * 1000.0 / ticks_per_sec);

        printf("\n");
        DOCA_LOG_INFO("Time Duration - Elapsed time (ns): %lu", end_ns - start_ns);             // Fine-grained latency measurements (e.g., instruction execution time)
        DOCA_LOG_INFO("Execution Time - Elapsed time (us): %lu", (end_ns - start_ns) / 1000);   // Function execution time, system calls, event response time
        DOCA_LOG_INFO("Estimated CPU Cycles: %lu", cycles);                                     // Low-level performance profiling (e.g., PMCCNTR_EL0)
        printf("\n");
}
*/

static inline void start_time()
{
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts_start);
        clock_gettime(CLOCK_MONOTONIC_RAW, &wall_start);
}

static inline void end_time(double *cpu_us, double *wall_us)
{
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts_end);
        clock_gettime(CLOCK_MONOTONIC_RAW, &wall_end);

        uint64_t cpu_ns = (ts_end.tv_sec - ts_start.tv_sec) * 1000000000ULL + (ts_end.tv_nsec - ts_start.tv_nsec);

        uint64_t wall_ns = (wall_end.tv_sec - wall_start.tv_sec) * 1000000000ULL + (wall_end.tv_nsec - wall_start.tv_nsec);

        *cpu_us  = cpu_ns  / 1000.0;
        *wall_us = wall_ns / 1000.0;
}



/**
 * Initialize sample resources
 *
 * @server_name [in]: Server name to connect to
 * @dev_pci_addr [in]: PCI address to connect over
 * @dev_rep_pci_addr [in]: PCI address for the representor
 * @text [in]: Message to send to the client
 * @sample_objects [in]: Sample objects struct to initialize
 * @return: DOCA_SUCCESS on success and DOCA_ERROR otherwise
 */
/* static doca_error_t init_comch_data_path_server_objects(const char *server_name,
                                                        const char *dev_pci_addr,
                                                        const char *dev_rep_pci_addr,
                                                        const char *text,
                                                        struct comch_data_path_server_objects *sample_objects) */
static doca_error_t init_comch_data_path_server_objects(const char *server_name,
                                                        const char *dev_pci_addr,
                                                        const char *dev_rep_pci_addr,
                                                        struct comch_data_path_server_objects *sample_objects)
{
        doca_error_t result;
        struct comch_ctrl_path_server_cb_config server_cb_cfg = {
                .send_task_comp_cb = server_send_task_completion_callback,
                .send_task_comp_err_cb = server_send_task_completion_err_callback,
                .msg_recv_cb = server_message_recv_callback,
                .server_connection_event_cb = server_connection_event_callback,
                .server_disconnection_event_cb = server_disconnection_event_callback,
                .data_path_mode = true,
                .new_consumer_cb = new_consumer_callback,
                .expired_consumer_cb = expired_consumer_callback,
                .ctx_user_data = sample_objects,
                .ctx_state_changed_cb = server_state_changed_callback};
        struct timespec ts = {
                .tv_sec = 0,
                .tv_nsec = SLEEP_IN_NANOS,
        };

        /* sample_objects->data_path->text = text;                                      VBrusse */



        /* Open DOCA device according to the given PCI address */
        result = open_doca_device_with_pci(dev_pci_addr, NULL, &(sample_objects->hw_dev));
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to open DOCA device based on PCI address");
                return result;
        }
        sample_objects->data_path->hw_dev = sample_objects->hw_dev;

        /* Open DOCA device representor according to the given PCI address */
        result = open_doca_device_rep_with_pci(sample_objects->hw_dev,
                                               DOCA_DEVINFO_REP_FILTER_NET,
                                               dev_rep_pci_addr,
                                               &(sample_objects->rep_dev));
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to open DOCA device representor based on PCI address");
                goto close_hw_dev;
        }

        /* Init CC server */
        result = init_comch_ctrl_path_server(server_name,
                                             sample_objects->hw_dev,
                                             sample_objects->rep_dev,
                                             &server_cb_cfg,
                                             &(sample_objects->server),
                                             &(sample_objects->pe));
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Fail init cc server with error = %s", doca_error_get_name(result));
                goto close_rep_dev;
        }
        sample_objects->data_path->pe = sample_objects->pe;

        /* Wait start_data_path_test msg from client, so that server can get the connection information */
        while (sample_objects->connection == NULL) {
                if (doca_pe_progress(sample_objects->pe) == 0)
                        nanosleep(&ts, &ts);
        }
        sample_objects->data_path->connection = sample_objects->connection;

        /* Exchange message with client to make connection is reliable */
        while (sample_objects->data_path_test_started == false) {
                if (doca_pe_progress(sample_objects->pe) == 0)
                        nanosleep(&ts, &ts);
        }

        return DOCA_SUCCESS;

close_rep_dev:
        (void)doca_dev_rep_close(sample_objects->rep_dev);
close_hw_dev:
        (void)doca_dev_close(sample_objects->hw_dev);
        return result;
}

/*
 * Stop server and relay instruction to client
 *
 * @sample_objects [in]: data path configuration for server
 */
static void handle_error_state(struct comch_data_path_server_objects *sample_objects)
{
        doca_error_t result;

        result = server_send_msg(sample_objects, STR_STOP_DATA_PATH_TEST, strlen(STR_STOP_DATA_PATH_TEST));
        if (result != DOCA_SUCCESS)
                DOCA_LOG_ERR("Failed to submit send task with error = %s", doca_error_get_name(result));

        sample_objects->data_path_test_stopped = true;
        (void)doca_ctx_stop(doca_comch_server_as_ctx(sample_objects->server));
}

        /**
         * For EXPLICIT Filler Bits Mode:
         *
         *  If ARMRAL_LDPC_FILLER_BITS_EXPLICIT (default) is set in options, the input LLRs must be in the format
         *
         *      [ message LLRs | filler LLRs | parity LLRs ]
         *
         *      n_expl = K_prime + len_filler_bits + (N - K_cb)
         *
         * where
         *      len_filler_bits = K_cb - K_prime
         *
         *      For LDPC base graph 1, 𝐾𝑐𝑏=8448 bits.
         *      For LDPC base graph 2, 𝐾𝑐𝑏=3840 bits.
         *
         */
        /**
         * IMPLICIT Filler Bits Mode:
         *
         * If ARMRAL_LDPC_FILLER_BITS_IMPLICIT is set in options, the input LLRs must be in the format
         *
         *      [ message LLRs | parity LLRs ]
         *
         *      n_implicit = K_prime + (N - K_cb)
         *
         * where
         *      K_prime - actual information buts in the code block
         *      N: total codeword length (including all bits)
         *      K_cb - maximum information bits for the base graph
         *
         */

/**
 * Example with your numbers:
 *
 *      K_prime = 824
 *      len_filler_bits = 56
 *      parity_bits = 3520
 *
 *      INPUT (EXPLICIT format - msg_llrs):
 *      ┌─────────────┬──────────────┬─────────────┐
 *      │  Message    │   Filler     │   Parity    │
 *      │  824 LLRs   │   56 LLRs    │  3520 LLRs  │
 *      │  [0..823]   │  [824..879]  │ [880..4399] │
 *      └─────────────┴──────────────┴─────────────┘
 *
 *
 *      OUTPUT (IMPLICIT format - llrs_out):
 *      ┌─────────────┬─────────────┐
 *      │  Message    │   Parity    │
 *      │  824 LLRs   │  3520 LLRs  │
 *      │  [0..823]   │  [824..4343]│
 *      └─────────────┴─────────────┘
 *                    ↑
 *             Fillers removed!
 *
 */




#define CRC16_POLY 0x1021

// =======================================================
//              FAST CRC16-CCITT (OAI Compatible)
// =======================================================
static uint16_t crc16_table[256];
static int crc16_initialized = 0;

static void crc16_table_init(void)
{
    for (int i = 0; i < 256; i++) {
        uint16_t crc = (uint16_t)i << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ CRC16_POLY;
            else
                crc <<= 1;
        }
        crc16_table[i] = crc;
    }
    crc16_initialized = 1;
}

static uint16_t crc16_ccitt_fast(const uint8_t *data, size_t len)
{
    if (!crc16_initialized)
        crc16_table_init();

    uint16_t crc = 0x0000; // OAI CRC16 init value for LDPC
    for (size_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

// =======================================================
//              Utility: Bit Reversal per Byte
// =======================================================
static inline uint8_t reverse_bits(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

// =======================================================
//              CRC16 Verification (Post-Decoder)
// =======================================================
void verify_ldpc_crc16(uint8_t *data_out, uint32_t kp_bits)
{
    uint32_t total_bytes = kp_bits / 8;
    if (total_bytes < 3) {
        printf("[CRC16] Too short to contain CRC16.\n");
        return;
    }

    uint32_t info_bytes = total_bytes - 2;
    uint8_t *info = data_out;
    uint16_t crc_extracted = (data_out[info_bytes] << 8) | data_out[info_bytes + 1];
    uint16_t crc_calc = crc16_ccitt_fast(info, info_bytes);

    printf("\n***** CRC16 Verification (Fast Table Version)\n");
    printf("***** Info bytes: %u, Total bytes: %u\n", info_bytes, total_bytes);
    printf("***** Extracted CRC (BE): 0x%04X\n", crc_extracted);
    // printf("Calculated CRC:     0x%04X --> %s\n", crc_calc, (crc_calc == crc_extracted) ? "PASS" : "FAIL");
    printf("***** Calculated CRC:     0x%04X\n", crc_calc);

    // Swap endian (LE case)
    uint16_t crc_extracted_le = (data_out[info_bytes + 1] << 8) | data_out[info_bytes];
    // printf("Extracted CRC (LE): 0x%04X --> %s\n", crc_extracted_le, (crc_calc == crc_extracted_le) ? "PASS" : "FAIL");
    printf("***** Extracted CRC (LE): 0x%04X\n", crc_extracted_le);

    // Optional: check with bit-reversed info (for reverse bit order)
    uint8_t reversed[2048];
    if (info_bytes <= sizeof(reversed)) {
        for (uint32_t i = 0; i < info_bytes; i++)
            reversed[i] = reverse_bits(info[i]);
        uint16_t crc_calc_rev = crc16_ccitt_fast(reversed, info_bytes);
        // printf("Bit-Reversed CRC check: 0x%04X --> %s\n", crc_calc_rev, (crc_calc_rev == crc_extracted || crc_calc_rev == crc_extracted_le) ? "PASS" : "FAIL");
        printf("***** Bit-Reversed CRC check: 0x%04X\n", crc_calc_rev);
    }
}










/*
 * "ldpc_decoder_kernel" - This function computes the LDPC kernel as a task on DPU offloading it from the host's DU stack.
 * The ldpc decoder function is provided by ArmRAL.
 *
 * data_path->pldpc_decod_params.inputBlock [in]:
 * data_path->pldpc_decod_params.bg [in]:
 * data_path->pldpc_decod_params.z [in]:
 * data_path->pldpc_decod_params.len_filler_bits [in]:
 * data_path->pldpc_decod_params.outputBlock [out]:
 *
 * @return: DOCA_SUCCESS on success and DOCA_ERROR otherwise
 */
doca_error_t ldpc_decoder_kernel(void *arg)
{
        doca_error_t result;
        armral_status status;                                   /* status to indicate success or failure of armral execution */

        double cpu_timeus, wall_timeus;

        // Cast the argument to its expected type and proceed
        struct ldpc_decod_params_t *params = (struct ldpc_decod_params_t *)arg;
        result = DOCA_SUCCESS;
        status = ARMRAL_SUCCESS;

        DOCA_LOG_INFO("********** Starting decoding **********");

        // ---------------------------------------------------------------------
        // Derive key parameters correctly
        // ---------------------------------------------------------------------
        armral_ldpc_graph_t bg;

        // Base Graph mapping from OAI to ARM RAL
        if (params->bg == 1) {
                bg = LDPC_BASE_GRAPH_1;
                // DOCA_LOG_INFO("*** LDPC_BASE_GRAPH_1 = %d", LDPC_BASE_GRAPH_1);
        } else {
                bg = LDPC_BASE_GRAPH_2;
                // DOCA_LOG_INFO("*** LDPC_BASE_GRAPH_2 = %d", LDPC_BASE_GRAPH_2);
        }

        uint32_t Kc = (params->bg == 1) ? 68 : 52;              /* VBrusse - Kc = Number of columns - OAI bg=1 means BG1 */
        uint32_t Kb = (params->bg == 1) ? 22 : 10;              /* VBrusse - Kb = Information columns - OAI bg=1 means BG1 */

        uint32_t Kcb = Kb * params->z;                          /* // Information bits + filler */
        uint32_t Kprime = params->Kprime;

        uint32_t len_filler_bits = Kcb - Kprime;

        uint32_t N = Kc * params->z;
        uint32_t punctured = 2 * params->z;                     // Always 2Z punctured

        uint32_t n = N - punctured;

        // uint32_t n_cb = (Kc - 2) * params->z;                // n = n_cb

        // uint32_t parity = (Kc * params->z) - punctured - (Kb * params->z);

        // uint32_t armral_n = (Kc - 2) * params->z;


        // ---------------------------------------------------------------------
        // Sanity checks (CRITICAL)
        // ---------------------------------------------------------------------
/*      if (params->n != expected_n) {
                DOCA_LOG_WARN("Fixing n: got=%u expected=%u", params->n, expected_n);
                params->n = expected_n;
        }

        if (params->len_filler_bits != F) {
                DOCA_LOG_ERR("Invalid filler bits: got=%u expected=%u", params->len_filler_bits, F);
                return DOCA_ERROR_INVALID_VALUE;
        }
*/
        if (params->n > CC_LDPC_DEC_IN_BLOCK_LEN) {
                DOCA_LOG_ERR("LLR buffer too small: n=%u max=%u", params->n, CC_LDPC_DEC_IN_BLOCK_LEN);
                return DOCA_ERROR_INVALID_VALUE;
        }


        // ---------------------------------------------------------------------
        // CRC checking and options parameter
        // ---------------------------------------------------------------------
        uint32_t crc_option = 0;
        switch (params->crc_type) {
                case 0:  crc_option = ARMRAL_LDPC_CRC_24A; break;
                case 1:  crc_option = ARMRAL_LDPC_CRC_24B; break;
                case 2:  crc_option = ARMRAL_LDPC_CRC_16;  break;
                default: crc_option = ARMRAL_LDPC_CRC_NO;  break;
        }

        params->options = ARMRAL_LDPC_FILLER_BITS_EXPLICIT | crc_option;
        // params->options = ARMRAL_LDPC_FILLER_BITS_EXPLICIT;
/*      params->options = ARMRAL_LDPC_FILLER_BITS_EXPLICIT
                | ARMRAL_LDPC_CRC_24B
                | ARMRAL_LDPC_PARITY_CHECK_DISABLE;   // 0x100 | 0x10 | 0x400 = 0x510
*/

        int8_t armral_ulsch_llr_np[27000] = {0};

        // Get actual size of llr array
        // uint32_t size = sizeof(params->llrs) / sizeof(int8_t);
        // printf("size of llr buffer = %d\n", size);


        // skip punctured columns
        memset(armral_ulsch_llr_np, 0, n);
        memcpy(armral_ulsch_llr_np, &params->llrs[punctured], n);

        /* Override filler-bit LLRs with strong positives (known-zero bits) */
/*      uint32_t filler_start = params->Kprime - punctured;
        for (uint32_t i = 0; i < len_filler_bits; i++) {
                armral_ulsch_llr_np[filler_start + i] = 127;
        }
*/

/*      const int8_t *decode_llrs = armral_ulsch_llr_np;                                        // default - BG2

        // VBrusse - rate recovery only for BG1
        static int8_t armral_rate_rec_dst[27000] = {0};
        if (params->bg == 1) {
                memset(armral_rate_rec_dst, 0, n);
                armral_status rr_status = armral_ldpc_rate_recovery(bg,
                                                                    params->z,
                                                                    params->e,
                                                                    n,                          // VBrusse - nref = n without punctured
                                                                    len_filler_bits,
                                                                    Kcb,
                                                                    0,                          // rv hardcoded
                                                                    ARMRAL_MOD_QPSK,            // mod = 2 in OAI
                                                                    &params->llrs[punctured],
                                                                    armral_rate_rec_dst);
                if (rr_status != ARMRAL_SUCCESS)
                        DOCA_LOG_ERR("Rate recovery failed: %d", rr_status);
                else
                        decode_llrs = armral_rate_rec_dst;
        }
*/

        /*
         * Data representation - Decoder input
         *      - Every single soft bit requires its own int8_t container, the memory footprint matches the rule: 1 LLR = 1 byte.
         *
         *      -128            -127                        0                       +127
         *        |---------------|-------------------------|-------------------------|
         *    Reserved      Strong Hard "1"               Uncertain                Strong Hard "0"
         *                (Very likely a 1)             (Pure Noise)              (Very likely a 0)
         *
         *      Data Type: int8_t (Values range strictly from -128 to +127). Unpacked (1 soft bit/byte) - Soft LLRS (N bytes)
         *
         *      The Sign Rule:
         *              Positive values ($> 0$) mean the radio receiver believes the original bit was a 0.
         *              Negative values ($< 0$) mean the radio receiver believes the original bit was a 1.
         *
         *      The Magnitude Rule: The absolute value represents the reliability (confidence) of the measurement.
         *              +127 or -127 mean maximum hardware confidence.
         *              0 means complete uncertainty (equal probability of being a 1 or a 0).
         *
         * Data representation - Decoder output
         *      Data type: uint8_t. Packed (8 bits/byte). Restored Data (K bits).
         */

        DOCA_LOG_INFO("*** n = %d, Kc=%d, z=%d, OAI bg=%d, Arm bg=%d", n, Kc, params->z, params->bg, bg);
        // DOCA_LOG_INFO("*** n = %d > %d and <= %d for BG2", n, 8 * params->z, 50 * params->z);
        DOCA_LOG_INFO("*** Kprime = %d", params->Kprime);
        DOCA_LOG_INFO("*** bg = %d", bg);
        DOCA_LOG_INFO("*** z = %d", params->z);
        DOCA_LOG_INFO("*** e = %d", params->e);
        DOCA_LOG_INFO("*** its_max = %d", params->its_max);
        DOCA_LOG_INFO("*** crc_type = %d", params->crc_type);
        DOCA_LOG_INFO("*** Kcb = %d", Kcb);
        DOCA_LOG_INFO("*** len_filler_bits = %d", len_filler_bits);
        DOCA_LOG_INFO("*** options = 0x%x\n", params->options);


        // ---------------------------------------------------------------------
        // Decode
        // ---------------------------------------------------------------------

        params->its_out = 0;

        start_time();
        // start_cpu_time();

        status = armral_ldpc_decode_block(n,    // full LDPC codeword length AFTER rate recovery
                                          (const int8_t *)armral_ulsch_llr_np,          // must NOT include punctured columns
                                          // decode_llrs,                               // use with armral_ldpc_rate_recovery
                                          bg,
                                          params->z,
                                          len_filler_bits,                              // F
                                          params->data_out,
                                          params->its_max,
                                          &params->its_out,
                                          params->options);

        end_time(&cpu_timeus, &wall_timeus);
        // end_cpu_time();


        // printf("\n*** DPU Decoder ==> CPU time: %.2f us | Wall time: %.2f us\n", cpu_timeus, wall_timeus);                   // VBrusse - time in us (microseconds)
        // printf("\n*** DPU Decoder ==> CPU time: %.3f ms | Wall time: %.3f ms\n", cpu_timeus/1000.0, wall_timeus/1000.0);     // VBrusse - time in ms (miliseconds)
        printf("[ldpc_decoder_kernel]*** DPU Decoder ==> CPU time: %.3f ms          iterations = %d\n\n", cpu_timeus/1000.0, params->its_out);  // VBrusse - time in ms (miliseconds)
        // DOCA_LOG_INFO("CPU time: %.2f us | Wall time: %.2f us", cpu_timeus, wall_timeus);

        DOCA_LOG_INFO("ArmRAL LDPC decoder status = %d", status);

        if (status != ARMRAL_SUCCESS) {
                DOCA_LOG_ERR("LDPC decoder failed (status=%d)", status);
                result = DOCA_ERROR_UNKNOWN;
        } else {
                DOCA_LOG_INFO("LDPC decoder succeeded.");
                result = DOCA_SUCCESS;
        }


        uint32_t Kprime_bytes = (Kprime + 7) / 8;

        printf("\n[ldpc_decoder_kernel] *** LDPC Block Decoded: %d bytes\n", Kprime_bytes);
        printf("decimal: ");
        for (uint32_t i = 0; i < 32; i++) {
                printf("%u ", params->data_out[i]);
        }
        printf("\n");
        printf("hex: ");
        for (uint32_t i = 0; i < 32; i++) {
                printf("%02x ", params->data_out[i]);
        }
        printf("\n\n");


        return result;
}



/*
 * register_ldpc_dec_callback - Registration of the LDPC decoder callback function.
 *
 */
void register_ldpc_dec_callback(struct comch_data_path_objects *data_path, doca_error_t (*callback)(void *))
{
    if (data_path == NULL || callback == NULL) {
        DOCA_LOG_INFO("Invalid parameters for callback registration");
        return;
    }

    data_path->ldpc_dec_callback = callback;

    data_path->ldpc_enc_callback = NULL;
}

/*
 * Initialize a task circular queue
 * Initialize the task queue
 */
void task_queue_init(struct task_queue_t *q)
{

        q->head = 0;
        q->tail = 0;
        q->count = 0;
        q->keep_running = 1;

        pthread_mutex_init(&q->lock, NULL);
        pthread_cond_init(&q->not_empty, NULL);
        pthread_cond_init(&q->not_full, NULL);

}

/*
 * Enqueue task - Add a task to the queue
 *
 */
void enqueue(struct task_queue_t *q, struct task_t *task)
{

        pthread_mutex_lock(&q->lock);

        // Wait if the queue is full
        while (q->count == CC_TASKS_QUEUE_SIZE) {
                DOCA_LOG_INFO("Queue is full. Waiting...");
                pthread_cond_wait(&q->not_full, &q->lock);
        }

        // Assign a unique task ID
        task->task_id = task_counter++;

        // Add task to the queue
        q->tasks[q->tail] = task;
        q->tail = (q->tail + 1) % CC_TASKS_QUEUE_SIZE;
        q->count++;

        DOCA_LOG_INFO("Enqueued task ID %d. Queue count: %d", task->task_id, q->count);

        pthread_cond_signal(&q->not_empty); // Signal that the queue is not empty
        pthread_mutex_unlock(&q->lock);

}

/*
 * Dequeue task - Get/Remove a task from the queue
 *
 */
struct task_t *dequeue(struct task_queue_t *q)
{

        pthread_mutex_lock(&q->lock);

        // Wait if the queue is empty
        while (q->count == 0) {
                DOCA_LOG_INFO("Queue is empty. Waiting...");
                pthread_cond_wait(&q->not_empty, &q->lock);
        }

        // If not keep running is signaled, exit function
/*
        if (!q->keep_running) {
            pthread_mutex_unlock(&q->lock);
            break;
        }
*/
        // Get/Remove task from the queue
        struct task_t *task = q->tasks[q->head];  // Return pointer, not copy
        q->head = (q->head + 1) % CC_TASKS_QUEUE_SIZE;
        q->count--;

        DOCA_LOG_INFO("Dequeued task ID %d. Queue count: %d", task->task_id, q->count);

        pthread_cond_signal(&q->not_full);
        pthread_mutex_unlock(&q->lock);


        return task;
}

void task_queue_terminate(struct task_queue_t *t_queue)
{

        pthread_mutex_lock(&t_queue->lock);
        t_queue->keep_running = true;                           // Signal all threads to exit
        pthread_cond_broadcast(&t_queue->not_empty);            // Wake up all threads
        pthread_mutex_unlock(&t_queue->lock);

        DOCA_LOG_INFO("All threads signaled to shut down gracefully\n");
}

void cleanup(struct task_queue_t *q, pthread_t *threads)
{

        task_queue_terminate(q);                                // Signal threads to exit

        for (int i = 0; i < CC_THREADS_POOL_SIZE; i++) {
                pthread_join(threads[i], NULL);                 // Wait for threads to exit
        }

        pthread_mutex_destroy(&q->lock);
        pthread_cond_destroy(&q->not_empty);
        pthread_cond_destroy(&q->not_full);

        // pthread_mutex_destroy(&mutex_lock);
}

/*
 * Worker thread function
 *
 */
// Threads retrieve and process tasks as they become available.
void *worker_thread(void *args)
{

        // struct task_queue_t *t_queue = (struct task_queue_t *)arg;                   // Use pointer to the shared queue
        // struct circular_queue_t t_queue = *(struct circular_queue_t *)arg;
        // int tid =*(int *)args;
        // struct task_queue_t *t_queue = (struct task_queue_t *)arg;                   // Access the queue safely
        struct thread_arg_t *thread_arg = (struct thread_arg_t *)args;                  // threads args:        arg1 = task_queue
        int tid = *thread_arg->pthread_id;                                              //                      arg2 = thread_id


        /* Looking at your worker thread, now that each task has its own embedded data_path, the mutexes are no longer needed for recv and send — each thread works on its own
         * independent data.
         * Remove all mutex locks/unlocks around recv and send.
         */

        // int pthread_res;
        doca_error_t result = DOCA_SUCCESS;
/*
        struct timespec ts = {                                                          // Signal client that server is ready for next request
                .tv_sec = 0,
                .tv_nsec = SLEEP_IN_NANOS,
        };
*/


        // while (keep_running) {                                       // Allows each thread to keep running and processing tasks
                                                                        // Ensures threads don’t terminate after processing a single task, which would
                                                                        // require recreating them for each task

        while (1) {
                // Dequeue a task and process it

                // struct task_t *task = malloc(sizeof(struct task_t)); // Dynamically allocated

                struct task_t *task = dequeue(thread_arg->task_queue);  /* Worker gets the next work */

                // 1. Save pointer BEFORE freeing task (already at top of while loop)
                struct comch_data_path_server_objects *task_sample_objects = (struct comch_data_path_server_objects *)task->sample_objects;

                // If not keep running is signaled, exit loop

                if (!thread_arg->task_queue->keep_running) {
                        break;
                }

                DOCA_LOG_INFO("Thread %lu, thread_id %d: Handling task = %d", pthread_self(), tid, task->task_id);

                /* Execute the task */



                /* pthread_res = pthread_mutex_lock(&cc_mutex);                 // Lock the Communication Channel
                if (pthread_res != 0) {
                        DOCA_LOG_ERR("Failed to lock Comm Channel for writing=%d", errno);
                        // ctx->results->sendto_result = DOCA_ERROR_OPERATING_SYSTEM;
                        return NULL;
                } */


                result = comch_data_path_recv_msg(&task->data_path);
                if (result != DOCA_SUCCESS) {
                        // Client handles tear down - in case of server error it must be started from server
                        handle_error_state(task_sample_objects);
                        // goto exit;

                        free(task->sample_objects);
                        free(task);

                        return NULL;
                }


                /* pthread_res = pthread_mutex_unlock(&cc_mutex);               // Unlock the Communication Channel
                if (pthread_res != 0) {
                        DOCA_LOG_ERR("Failed to unlock Comm Channel for writing=%d", errno);
                        // ctx->results->sendto_result = DOCA_ERROR_OPERATING_SYSTEM;
                        return NULL;
                } */


/*              DOCA_LOG_INFO("*** AFTER recv - LDPC block decoded:");
                for (int i = 0; i < ((struct ldpc_decod_params_t *)(task->data_path->pldpc_dec_pars))->kp; i++) {
                        printf("%d ", ((struct ldpc_decod_params_t *)(task->data_path->pldpc_dec_pars))->data_out[i]);
                }
                printf("\n"); */

/*
                int Kprime = (((struct ldpc_decod_params_t *)(task->data_path->pldpc_dec_pars))->Kprime + 7) / 8;               // Kprime in bytes
                for (int i = 0; i < Kprime; i++) {                                                                              // Esse for estava em mar-13
                        printf("%u ", (uint8_t)((struct ldpc_decod_params_t *)(task->data_path->pldpc_dec_pars))->data_out[i]);
                        // printf("%d ", (uint8_t)((struct ldpc_decod_params_t *)(task->data_path->pldpc_dec_pars))->data_out[i]);
                }
                printf("\n\n");
*/


                /* pthread_res = pthread_mutex_lock(&cc_mutex);                 // Lock the Communication Channel
                if (pthread_res != 0) {
                        DOCA_LOG_ERR("Failed to lock Comm Channel for writing=%d", errno);
                        // ctx->results->sendto_result = DOCA_ERROR_OPERATING_SYSTEM;
                        return NULL;
                } */


                result = comch_data_path_send_msg(&task->data_path);
                if (result != DOCA_SUCCESS) {
                        handle_error_state(task_sample_objects);
                        // goto exit;

                        free(task->sample_objects);
                        free(task);

                        return NULL;
                }


                /* pthread_res = pthread_mutex_unlock(&cc_mutex);               // Unlock the Communication Channel
                if (pthread_res != 0) {
                        DOCA_LOG_ERR("Failed to unlock Comm Channel for writing=%d", errno);
                        // ctx->results->sendto_result = DOCA_ERROR_OPERATING_SYSTEM;
                        return NULL;
                } */



                /* DOCA_LOG_INFO("*** Input Block ===> %s", (char *)((struct ldpc_decod_params_t *)(task->data_path->pldpc_dec_pars))->inputBlock); */
                /* DOCA_LOG_INFO("LLRs (Log-Likelihood Ratios) values:");
                for (uint8_t i = 0; i < CC_LLR_SIZE; i++) {
                        printf("%d ", ((struct ldpc_decod_params_t *)(task->data_path->pldpc_dec_pars))->llrs[i]);
                }
                printf("\n");
                DOCA_LOG_INFO("*** bg = %d", ((struct ldpc_decod_params_t *)(task->data_path->pldpc_dec_pars))->bg);
                DOCA_LOG_INFO("*** z = %d", ((struct ldpc_decod_params_t *)(task->data_path->pldpc_dec_pars))->z);
                DOCA_LOG_INFO("*** cdr_idx = %d", ((struct ldpc_decod_params_t *)(task->data_path->pldpc_dec_pars))->crc_idx);
                DOCA_LOG_INFO("*** max_its = %d", ((struct ldpc_decod_params_t *)(task->data_path->pldpc_dec_pars))->max_its);
                for (int i = 0; i < CC_LLR_SIZE; i++) {
                        printf("%d ", *(((struct ldpc_decod_params_t *)(task->data_path->pldpc_dec_pars))->data_out + i));
                }
                printf("\n"); */

                //nanosleep(&ts, &ts);



                /* 2. Free DOCA data path resources first - added by VBrusse */
                terminate_comch_data_path_decod_server(&task->data_path);        /* Free resources allocated by DPU */


                // 3. Signal client ready for next request BEFORE cleaning sample_objects
/*              task_sample_objects->server_result = DOCA_ERROR_IN_PROGRESS;
                result = server_send_msg(task_sample_objects, STR_START_DATA_PATH_TEST, strlen(STR_START_DATA_PATH_TEST));
                if (result != DOCA_SUCCESS) {
                        DOCA_LOG_ERR("Failed to send ready signal: %s", doca_error_get_name(result));
                }
*/

                // 4. Clean sample_objects - waits for data_path_test_stopped and server_finish
                //    MUST be after server_send_msg so flags get set correctly
                clean_comch_data_path_server_objects(task_sample_objects);


                /* 5. Free heap objects - inner first, outer last */
                // Free all heap allocated objects
                // free(task->data_path.pldpc_dec_pars);        b               // free ldpc params
                free(task->sample_objects);                                     // free sample_objects struct
                free(task);                                                     /* Free task memory */

                sem_post(&request_done);                                        // signal main lop that the current request is done

                // long ticks_per_sec = sysconf(_SC_CLK_TCK);
                // double elapsed_time = (double)(end_ticks - start_ticks) / ticks_per_sec;

                // double elapsed_time = (end_nanos.tv_sec - start_nanos.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
                // double elapsed_time = end_nanos.tv_sec - start_nanos.tv_sec;

                // DOCA_LOG_INFO("CPU Cycles elapsed: %llu\n", end_cycles - start_cycles);
                // DOCA_LOG_INFO("CPU Ticks elapsed: %ld (%.6f sec)\n", (end_ticks - start_ticks), elapsed_time);
                // DOCA_LOG_INFO("Elapsed time: %.9f sec\n", elapsed_time);

                // DOCA_LOG_INFO("*** start:  %ld   finish:  %ld", start, end);
                // DOCA_LOG_INFO("*** elapsed time:  %ld CPU ticks/Clock ticks", end - start);

                // DOCA_LOG_INFO("User time: %lf sec", (double)buffer.tms_utime / ticks_per_sec);
                // DOCA_LOG_INFO("System time: %lf sec", (double)buffer.tms_stime / ticks_per_sec);
                // DOCA_LOG_INFO("Elapsed time: %lf sec", (double)(end - start) / ticks_per_sec);

                // DOCA_LOG_INFO("Thread %lu: Called start_nrLDPC_decod_server... with result = %d", pthread_self(), result);

                // usleep(1000);                                // Sleep for 10ms to simulate work
        }

        return NULL;

}



/**
 * Run comch_server sample
 *
 * @server_name [in]: Server name to connect to
 * @dev_pci_addr [in]: PCI address to connect over
 * @rep_pci_addr [in]: PCI address for the representor
 * @text [in]: Message to send to the server
 * @return: DOCA_SUCCESS on success and DOCA_ERROR otherwise
 */
/* doca_error_t start_comch_data_path_server_sample(const char *server_name,
                                                 const char *dev_pci_addr,
                                                 const char *rep_pci_addr,
                                                 const char *text) */
doca_error_t start_nrLDPC_decod_server(const char *server_name,
                                       const char *dev_pci_addr,
                                       const char *rep_pci_addr,
                                       struct ldpc_decod_params_t *pldpc_decod_params,
                                       struct task_queue_t *ptask_queue)
{
        doca_error_t result;
        // struct comch_data_path_server_objects sample_objects = {0};
        // struct comch_data_path_objects data_path = {0};                      // Just use a local struct and copy directly into task


        // 1. Allocate per-request ldpc params on heap (NOT stack)
        struct ldpc_decod_params_t *ldpc_dec_params = malloc(sizeof(struct ldpc_decod_params_t));
        memcpy(ldpc_dec_params, pldpc_decod_params, sizeof(struct ldpc_decod_params_t));

        // 2. Allocate task FIRST
        struct task_t *task = malloc(sizeof(struct task_t));
        memset(task, 0, sizeof(struct task_t));
        task->data_path.pldpc_dec_pars = ldpc_dec_params;
        task->data_path.pldpc_enc_pars = NULL;
        task->data_path.size_ldpc_data = sizeof(struct ldpc_decod_params_t);


        register_ldpc_dec_callback(&task->data_path, ldpc_decoder_kernel);


        // 3. Allocate sample_objects AFTER task
        struct comch_data_path_server_objects *sample_objects = malloc(sizeof(struct comch_data_path_server_objects));
        memset(sample_objects, 0, sizeof(*sample_objects));
        sample_objects->data_path = &task->data_path;          // now task exists!


        /* result = init_comch_data_path_server_objects(server_name, dev_pci_addr, rep_pci_addr, text, &sample_objects); */
        result = init_comch_data_path_server_objects(server_name, dev_pci_addr, rep_pci_addr, sample_objects);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to initialize sample with error = %s", doca_error_get_name(result));
                /* return result;                               Original */

                free(ldpc_dec_params);
                free(sample_objects);
                free(task);

                // return result;
                goto exit;
        }


        // 4. Enqueue task
        task->sample_objects = sample_objects;

        enqueue(ptask_queue, task);                             /* Submit task to the thread pool */


/*
        result = comch_data_path_recv_msg(&data_path);
        if (result != DOCA_SUCCESS) {
                Client handles tear down - in case of server error it must be started from server
                handle_error_state(&sample_objects);
                goto exit;
        }

        result = comch_data_path_send_msg(&data_path);
        if (result != DOCA_SUCCESS) {
                handle_error_state(&sample_objects);
                goto exit;
        }
*/

exit:
        // 5. DO NOT call clean_comch_data_path_server_objects here!
        //    Worker thread owns cleanup now
        return DOCA_SUCCESS;  // return immediately, worker handles the rest

        // clean_comch_data_path_server_objects(&sample_objects);

        // return result != DOCA_SUCCESS ? result : sample_objects.server_result;
}
