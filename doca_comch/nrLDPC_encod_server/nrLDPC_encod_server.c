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
 * Filename: nrLDPC_encod_server.c
 *
 * DOCA Communication Channel Server API customized by: Vlademir Brusse
 *
 * Date: 2025/09/19
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


/* #include "comch_ctrl_path_common.h" */
#include "nrLDPC_ctrl_path_common.h"
#include "nrLDPC_data_path_high_speed_common.h"
#include "common.h"

DOCA_LOG_REGISTER(NRLDPC_ENCOD_SERVER);



// #define CPU_FREQUENCY_HZ 2000000000                  // CPU Frequency in Hz (2.0 GHz)
// #define BF2_CPU_FREQ_HZ 2000000000ULL                // Define the BlueField-2 CPU frequency in Hz (2 GHz = 2,000,000,000 cycles per second)



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
/* static pthread_mutex_t cc_mutex = PTHREAD_MUTEX_INITIALIZER; */      /* mutex for the Communication Channel -  PCIe channel hw resource */
static int task_counter = 1;                            /* Global task counter */
sem_t request_done;

/* struct tms buffer;                                   // used for CPU ticks
clock_t start_ticks, end_ticks;

uint64_t start_ns, end_ns, cycles;
double cpu_freq_hz = 2200000000.0;
*/
static struct timespec ts_start, ts_end;
static struct timespec wall_start, wall_end;



/*
 * "armral_ldpc_encode_block" - Performs encoding using LDPC as laid out in the 3GPP Technical Specification (TS) 38.212. Encoding is performed for a single code block.
 *
 * The length of the code block is determined from the lifting size z and base graph. For example, for base graph 1 the length of a code block is 68 * z bits, and for base graph 2 the
 * length of the code block is 52 * z bits. The output from the encoding begins at the third column of the base graph. The first two columns are punctured, as per section 5.3.2 of
 * TS 38.212. The number of encoded bits returned from this function is 66 * z for base graph 1, and 50 * z for base graph 2. The values of z are limited to those in table 5.3.2-1 in
 * TS 38.212.
 *
 * The number of information bits in a code block is determined by the lifting size and base graph. For base graph 1 the number of information bits per code block is 22 * z. For base
 * graph 2 the number of information bits per code block is 10 * z. It is assumed that the correct number of input bits is passed into this function.
 *
 * Release: armral v26.01
 *
 * Parameters
 * data_in [in]
 *      A read-only parameter of type const uint8_t *.
 *
 *      The information bits to encode. It is assumed that the number of bits stored in data_in fits into a single code block. The number of information bits is assumed to be 22 * z
 *      for base graph 1, and 10 * z for base graph 2.
 *
 * bg [in]
 *      A read-only parameter of type armral_ldpc_graph_t.
 *
 *      Identifier for the base graph to use for encoding. TS 38.212 defines two base graphs in table 5.3.2-2 and 5.3.2-3. The base graph, in combination with the lifting size z,
 *      determines the block size and the graph to use for encoding the block.
 *
 * z [in]
 *      A read-only parameter of type uint32_t.
 *
 *      The lifting size. Valid values of the lifting size are described in table 5.3.2-1 in TS 38.212.
 *
 * len_filler_bits []
 *      A read-only parameter of type uint32_t.
 *
 *      The number of filler bits. As per TS 38.212, section 5.2.2, filler bits insertion is needed to ensure that the code block segments have a valid length and are a multiple of the
 *      lifting size.
 *
 * data_out [out]
 *      A write-only parameter of type uint8_t *.
 *
 *      The codeword to be transmitted. data_out has the first two columns for the base graphs punctured, and contains the information and calculated parity bits after encoding.
 *
 *
 * Returns
 *      An armral_status value that indicates success or failure.
 *
 */
armral_status armral_ldpc_encode_block(const uint8_t *data_in,
                                       armral_ldpc_graph_t bg, uint32_t z,
                                       uint32_t len_filler_bits,
                                       uint8_t *data_out);



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
                                                        (void *)msg,
                                                        len,
                                                        &task);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to allocate server task with error = %s", doca_error_get_name(result));
                return result;
        }

        result = doca_task_submit(doca_comch_task_send_as_task(task));
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
                    sample_objects->data_path->remote_consumer_id == 0) {    //consumer not yet registered
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
 * terminate_comch_data_path_encod_server - This function frees resources allocated by DPU (server) used for an LDPC offloading.
 *
 */
void terminate_comch_data_path_encod_server(struct comch_data_path_objects *data_path)
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

        /* sample_objects->data_path->text = text;                                      VBrusse: comment */



        /* sample_objects->data_path->pldpc_enc_pars = pldpc_encod_params;              VBrusse */



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



/* VBrusse
 * ldpc_encoder_kernel - This function computes the LDPC kernel as a task on DPU offloading it from the host's DU stack.
 * The ldpc encoder function is provided by ArmRAL.
 *
 * data_path->pldpc_encod_params.inputBlock [in]:
 * data_path->pldpc_encod_params.bg [in]:
 * data_path->pldpc_encod_params.z [in]:
 * data_path->pldpc_encod_params.len_filler_bits [in]:
 * data_path->pldpc_encod_params.outputBlock [out]:
 *
 * @return: DOCA_SUCCESS on success and DOCA_ERROR otherwise
 */
/* doca_error_t ldpc_encoder_kernel(void *input_params) */
doca_error_t ldpc_encoder_kernel(void *arg)
{
        doca_error_t result;
        armral_status status;                                   /* status to indicate success or failure of armral execution */

        double cpu_timeus, wall_timeus;

        if (arg == NULL) {
                DOCA_LOG_INFO("*** No argument provided, using default behavior\n\n");
                // Perform default operations
                return DOCA_SUCCESS;
        }

        // Cast the argument to its expected type and proceed
        struct ldpc_encod_params_t *params = (struct ldpc_encod_params_t *)arg;
        result = DOCA_SUCCESS;
        status = ARMRAL_SUCCESS;


/*      Steps to Create the Test Input:

        1. Generate the Information Block:

                Create a K-bit binary sequence (e.g., random 512 bits).

                Input block (512 bits)

        2. LDPC Encoding:

                Apply the LDPC encoding algorithm based on your selected base graph and lifting size (Zc).

                The output encoded block will be 1536 bits for this example (because the code rate is 1/3, the encoded size is
                three times the input size).

        3. Encoded Output:

                After LDPC encoding with rate 1/3, the encoded block will contain the original 512 information bits and 1024
                parity bits.

                Output block (1536 bits)


        Verifying the LDPC Algorithm:
                - Run your LDPC encoder on this input block.

                - Ensure that the output block is exactly 3 times the input size, and validate the encoded bits by running the
                  LDPC decoder (to ensure proper decoding and error correction).
*/

                                                                /* single block = a random binary sequence of length 512 */
/*
        Encoded Block Size (N): for code rate 𝑅 = 1/3, the encoded block size is: 𝑁=𝐾/𝑅 = 512×3 = 1536 bits.
*/

        /* Run ldpc encoding kernel for a single block */
        /*
         * armral_ldpc_encode_block -
         *
         * data_path->pldpc_encod_params.inputBlock [in]:
         * data_path->pldpc_encod_params.bg [in]:
         * data_path->pldpc_encod_params.z [in]:
         * data_path->pldpc_encod_params.len_filler_bits [in]:
         * data_path->pldpc_encod_params.outputBlock [out]:
         */

/*
        DOCA_LOG_INFO("Servidor recebe data_path->pldpc_enc_pars do Cliente");
        DOCA_LOG_INFO("*** Input Block ===> %s", (char *)((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->inputBlock);
        DOCA_LOG_INFO("*** bg = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->bg);
        DOCA_LOG_INFO("*** z = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->z);
        DOCA_LOG_INFO("*** k = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->k);
        DOCA_LOG_INFO("*** len_filler_bits = %d", ((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->len_filler_bits);
*/
        armral_ldpc_graph_t bg;

        // Base Graph mapping from OAI to ARM RAL
        if (params->bg == 1) {
               bg = LDPC_BASE_GRAPH_1;                  // bg = 0
        } else {
               bg = LDPC_BASE_GRAPH_2;                  // bg = 1
        }

        // K = K′ + F;                                          // K′ (Kprime / kp) = payload + CRC bits
        // uint32_t Kb = (bg == LDPC_BASE_GRAPH_1) ? 22 : 10;
        // uint32_t K = Kb * params->z;                         // K = total encoder input bits

        uint32_t Kc = (bg == LDPC_BASE_GRAPH_1) ? 68 : 52;
        uint32_t N = Kc * params->z;

        /* Data representation - Encoder input
         *      - Packed Bits (Standard) - OAI uses this format
         *        8 information bits are packed into a single byte. A byte 0xAA represents the bit sequence 10101010.
         *        The raw information bits (K).
         *
         *      - Data type: uint8_t. Unpacked Bits (One bit per byte). Systematic Data (K bits).
         *        Every single bit is exploded into its own byte. A 0 is stored as 0x00, and a 1 is stored as 0x01 (the remaining 7 bits of the byte are wasted).
         *
         * Data representation - Encoder output
         *      - Data type: uint8_t. Packed Bits (8 bits/byte). Data + Parity (N bits).
         *        The original bits + the generated parity check bits (N).
         */

/*      int32_t K_bytes = (params->k + 7) / 8;
        printf("\n[ldpc_encoder_kernel]*** LDPC Encoder Input Block: %d bytes\n\n", K_bytes);
        for (int i = 0; i < K_bytes; i++) {
                printf("%u ", params->inputBlock[i]);
        }
        printf("\n\n");
*/

        DOCA_LOG_INFO("*** bg = %d", params->bg);
        DOCA_LOG_INFO("*** z = %d", params->z);
        DOCA_LOG_INFO("*** k = %d", params->k);
        DOCA_LOG_INFO("*** kb = %d", params->kb);
        DOCA_LOG_INFO("*** len_filler_bits = %d", params->len_filler_bits);


        DOCA_LOG_INFO("********** Starting encoding **********");


        start_time();
        // start_cpu_time();


        status = armral_ldpc_encode_block((const uint8_t *)params->inputBlock,
                                          bg,
                                          params->z,
                                          params->len_filler_bits,
                                          (uint8_t *)params->outputBlock);

        end_time(&cpu_timeus, &wall_timeus);
        // end_cpu_time();

        // printf("\n*** DPU Encoder ==> CPU time: %.2f us | Wall time: %.2f us\n", cpu_timeus, wall_timeus);                 // VBrusse - time in us (microseconds)
        // printf("\n*** DPU Encoder ==> CPU time: %.3f ms | Wall time: %.3f ms\n", cpu_timeus/1000.0, wall_timeus/1000.0);   // VBrusse - time in ms (miliseconds)
        printf("\n[ldpc_encoder_kernel]*** DPU Encoder ==> CPU time: %.3f ms\n\n", cpu_timeus/1000.0);                                               // VBrusse - time in ms (miliseconds)
        // DOCA_LOG_INFO("ArmRAL LDPC encoder compute status = %d", status);

        DOCA_LOG_INFO("ArmRAL LDPC encoder status = %d", status);

        if (status != ARMRAL_SUCCESS) {
                DOCA_LOG_ERR("LDPC decoder failed (status=%d)", status);
                result = DOCA_ERROR_UNKNOWN;                                     /* return = 1 */
        } else {
                DOCA_LOG_INFO("LDPC encoder succeeded.");
                result = DOCA_SUCCESS;
        }


        int N_bytes = (N + 7) / 8;
        printf("\n[ldpc_encoder_kernel] *** LDPC Block Encoded: %d bytes\n", N_bytes);
        printf("decimal: ");
        // for (int i = 0; i < N_bytes; i++) {
        for (int i = 0; i < 32; i++) {
                printf("%u ", params->outputBlock[i]);
        }
        printf("\n");
        printf("hex: ");
        for (uint32_t i = 0; i < 32; i++)
                printf("%02x ", params->outputBlock[i]);
        printf("\n\n");



        /* LLRS mapping for a quantized LLR setup for each bit in a 128-bit LDPC block, it'll map each bit in the binary sequence to an LLR value
         */
        /* The type of base graph to use for the decoding. */
        /* dec_params.bg = params->bg; */

        /* The lifting size. */
        /* dec_params.z= params->z; */

        /* The index of the bit where the CRC attached to the code block begins. If there is no CRC attached, set this to ARMRAL_LDPC_NO_CRC. */
        /* dec_params.crc_idx = ARMRAL_LDPC_NO_CRC; */

        /* The maximum number of iterations of the LDPC decoder to run. The algorithm may terminate after fewer iterations if the current candidate
           codeword passes all the parity checks, or if it satisfies the CRC check. */
        /* dec_params.num_its = 10; */



        return result;
}



/*
 * register_ldpc_enc_callback - Registration of the LDPC encoder callback function.
 *
 */
void register_ldpc_enc_callback(struct comch_data_path_objects *data_path, doca_error_t (*callback)(void *))
{
        if (data_path == NULL || callback == NULL) {
                DOCA_LOG_INFO("Invalid parameters for LDPC callback registration");
                return;
        }

        data_path->ldpc_enc_callback = callback;

        data_path->ldpc_dec_callback = NULL;
}



/*
 *
 * pmccntr_el0 is a special ARM register that counts CPU cycles.
 * The inline assembly reads the cycle counter before and after execution.
 *
 */
/*
static inline uint64_t read_cpu_cycles()
{

        uint64_t val;

        // asm volatile("mrs %0, pmccntr_el0" : "=r"(val));

        // return val;


        // Check if the CPU supports the performance counter instruction
        #if defined(__aarch64__)
                // Try reading the pmccntr_el0 register if supported
                asm volatile("mrs %0, pmccntr_el0" : "=r"(val));
        #else
                // Fallback for unsupported hardware/architecture
                // Use an alternative method, like reading the time or a different counter
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                val = (uint64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
        #endif

        return val;
}
*/
/*
static inline uint64_t read_cpu_cycles() {
    uint64_t cycles;

    // Read the ARM performance counter register (pmccntr_el0)
    asm volatile(
        "mrs %0, pmccntr_el0"   // Read the CPU cycle counter
        : "=r" (cycles)          // Output operand: store value in 'cycles'
    );

    return cycles;
}
*/



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

        // int pthread_res;
        doca_error_t result = DOCA_SUCCESS;

/*      struct timespec ts = {
                .tv_sec = 0,
                .tv_nsec = SLEEP_IN_NANOS,
        };
*/


        // while (keep_running) {                                       // Allows each thread to keep running and processing tasks
                                                                        // Ensures threads don’t terminate after processing a single task, which would
                                                                        // require recreating them for each task

        while (1) {                                                     // Use a global flag to signal shutdown1) {
                // Dequeue a task and process it

                // struct task_t *task = malloc(sizeof(struct task_t)); // Dynamically allocated

                struct task_t *task = dequeue(thread_arg->task_queue);  /* Worker gets the next work */

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
                        /* Client handles tear down - in case of server error it must be started from server */
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


/*              int32_t K_bytes = (((struct ldpc_encod_params_t *)(task->data_path.pldpc_enc_pars))->k + 7) / 8;
                printf("*** LDPC Encoder Input Block: %d\n bytes", K_bytes);
                for (int i = 0; i < K_bytes; i++) {
                        printf("%u ", ((struct ldpc_encod_params_t *)(task->data_path.pldpc_enc_pars))->inputBlock[i]);
                }
                printf("\n\n");
*/
                DOCA_LOG_INFO("*** bg = %d", ((struct ldpc_encod_params_t *)(task->data_path.pldpc_enc_pars))->bg);
                DOCA_LOG_INFO("*** z = %d", ((struct ldpc_encod_params_t *)(task->data_path.pldpc_enc_pars))->z);
                DOCA_LOG_INFO("*** k = %d", ((struct ldpc_encod_params_t *)(task->data_path.pldpc_enc_pars))->k);
                DOCA_LOG_INFO("*** kb = %d", ((struct ldpc_encod_params_t *)(task->data_path.pldpc_enc_pars))->kb);
                DOCA_LOG_INFO("*** len_filler_bits = %d", ((struct ldpc_encod_params_t *)(task->data_path.pldpc_enc_pars))->len_filler_bits);


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


/*              printf("\n[worker_thread]*** LDPC Encoder Output Block: %d bytes\n", K_bytes);
                for (int i = 0; i < K_bytes; i++) {
                        printf("%u ", ((struct ldpc_encod_params_t *)(task->data_path.pldpc_enc_pars))->outputBlock[i]);
                }
                printf("\n\n");
*/

                // 1. Free DOCA data path resources
                terminate_comch_data_path_encod_server(&task->data_path);       /* Free resources allocated by DPU */

                // 2. Signal client ready for next request
                task_sample_objects->server_result = DOCA_ERROR_IN_PROGRESS;
                result = server_send_msg(task_sample_objects, STR_START_DATA_PATH_TEST, strlen(STR_START_DATA_PATH_TEST));
                if (result != DOCA_SUCCESS) {
                        DOCA_LOG_ERR("Failed to send ready signal: %s", doca_error_get_name(result));
                }

                // 3. Clean sample_objects DOCA resources
                clean_comch_data_path_server_objects(task_sample_objects);

                // 4. Free heap objects - DO NOT free pldpc_enc_pars (freed by clean_local_mem_bufs)
                free(task->sample_objects);
                free(task);

                sem_post(&request_done);   // signal main loop that current request is done


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

                // DOCA_LOG_INFO("Thread %lu: Called start_nrLDPC_encod_server... with result = %d", pthread_self(), result);

                // usleep(1000);                                // Sleep for 10ms to simulate work
        }

        return NULL;

}



/**
 * Run comch_client sample
 *
 * @server_name [in]: Server name to connect to
 * @dev_pci_addr [in]: PCI address to connect over
 * @rep_pci_addr [in]: PCI address for the representor
 * @text [in]: Message to send to the server
 * @pldpc_encod_params [in/out]: Address of the structure to send to server
 * @ptask_queue [in]:
 * @return: DOCA_SUCCESS on success and DOCA_ERROR otherwise
 */
/* doca_error_t start_comch_data_path_server_sample(const char *server_name,
                                                 const char *dev_pci_addr,
                                                 const char *rep_pci_addr,
                                                 const char *text) */
doca_error_t start_nrLDPC_encod_server(const char *server_name,
                                       const char *dev_pci_addr,
                                       const char *rep_pci_addr,
                                       struct ldpc_encod_params_t *pldpc_encod_params,
                                       struct task_queue_t *ptask_queue)
{
        doca_error_t result;
        // struct comch_data_path_server_objects sample_objects = {0};
        // struct comch_data_path_objects data_path = {0};

        /* Key changes from original:
                - sample_objects — heap, not stack ✓
                - data_path — embedded in task, not stack ✓
                - task->data_path — filled directly, not via pointer ✓
                - sample_objects->data_path = &task->data_path — callbacks update task directly ✓
                - init_thread_sync — removed (move to main) ✓
                - clean_comch_data_path_server_objects — removed (worker thread owns it) ✓
        */

        // 1. Allocate ldpc params on heap
        struct ldpc_encod_params_t *ldpc_enc_params = malloc(sizeof(struct ldpc_encod_params_t));
        memcpy(ldpc_enc_params, pldpc_encod_params, sizeof(struct ldpc_encod_params_t));

        // 2. Allocate task FIRST
        struct task_t *task = malloc(sizeof(struct task_t));
        memset(task, 0, sizeof(struct task_t));


        // 3. Fill task->data_path directly
        task->data_path.pldpc_enc_pars = ldpc_enc_params;
        task->data_path.pldpc_dec_pars = NULL;
        task->data_path.size_ldpc_data = sizeof(struct ldpc_encod_params_t);


        register_ldpc_enc_callback(&task->data_path, ldpc_encoder_kernel);              /* VBrusse: Register the LDPC encoder callback function */


        // 4. Allocate sample_objects on heap AFTER task
        struct comch_data_path_server_objects *sample_objects = malloc(sizeof(struct comch_data_path_server_objects));
        memset(sample_objects, 0, sizeof(*sample_objects));
        sample_objects->data_path = &task->data_path;                                   // points to task directly



        /* result = init_thread_sync(&cc_mutex);                                        // VBrusse - Initiate Comm Channel mutex
        if (result != DOCA_SUCCESS)
                return result; */



        /* result = init_comch_data_path_server_objects(server_name, dev_pci_addr, rep_pci_addr, text, &sample_objects); */
        result = init_comch_data_path_server_objects(server_name, dev_pci_addr, rep_pci_addr, sample_objects);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to initialize sample with error = %s", doca_error_get_name(result));
                /* return result;                               Original */

                free(ldpc_enc_params);
                free(sample_objects);
                free(task);
                goto exit;
        }


        // 5. Enqueue task
        task->sample_objects = sample_objects;

        enqueue(ptask_queue, task);                              /* Submit task to the thread pool */


/*
        stop_threads = 1;
        for (int i = 0; i < CC_THREADS_POOL_SIZE; i++) {
                pthread_join(thread_pool[i], NULL);
        }

*/

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


        /* VBrusse */
        // DOCA_LOG_INFO("\n\n\n*** =====> nrLDPC_encod_server DEPOIS do send_msg <=====");
        /* DOCA_LOG_INFO("*** Input Block ===> %s", (char *)((struct ldpc_encod_params_t *)(data_path->pldpc_enc_pars))->inputBlock); */

        // DOCA_LOG_INFO("*** Input Block ===> %s", (char *)((struct ldpc_encod_params_t *)(data_path.pldpc_enc_pars))->inputBlock);
        // DOCA_LOG_INFO("*** bg = %d", ((struct ldpc_encod_params_t *)(data_path.pldpc_enc_pars))->bg);
        // DOCA_LOG_INFO("*** z = %d", ((struct ldpc_encod_params_t *)(data_path.pldpc_enc_pars))->z);
        // DOCA_LOG_INFO("*** k = %d", ((struct ldpc_encod_params_t *)(data_path.pldpc_enc_pars))->k);
        // DOCA_LOG_INFO("*** len_filler_bits = %d", ((struct ldpc_encod_params_t *)(data_path.pldpc_enc_pars))->len_filler_bits);
        // DOCA_LOG_INFO("*** Output Block ===> %s\n\n", (char *)((struct ldpc_encod_params_t *)(data_path.pldpc_enc_pars))->outputBlock);

        /* added by VBrusse */
        /* terminate_comch_data_path_encod_server(&data_path); */                           /* free resources allocated by DPU */



exit:
        // 5. DO NOT call clean_comch_data_path_server_objects here!
        //    Worker thread owns cleanup now
        return DOCA_SUCCESS;  // return immediately, worker handles the rest

        // clean_comch_data_path_server_objects(&sample_objects);

        // return result != DOCA_SUCCESS ? result : sample_objects.server_result;
}
