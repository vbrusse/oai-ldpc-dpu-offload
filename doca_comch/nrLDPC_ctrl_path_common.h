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
 * Original filename: comch_ctrl_path_common.h
 *
 * Filename: nrLDPC_ctrl_path_common.h
 *
 * Customized by: Vlademir Brusse
 *
 * Date: 2026/01/19
 *
 */
/*
#ifndef COMCH_COMMON_H_
#define COMCH_COMMON_H_
*/
#ifndef NRLDPC_COMMON_H_
#define NRLDPC_COMMON_H_

// At the top of nrLDPC_ctrl_path_common.h:
#ifndef NRLDPC_CTRL_PATH_COMMON_H
#define NRLDPC_CTRL_PATH_COMMON_H

#include "nrLDPC_data_path_high_speed_common.h"  // ← add this line

#endif // NRLDPC_CTRL_PATH_COMMON_H

#include <stdbool.h>

#include <doca_comch.h>
#include <doca_ctx.h>
#include <doca_dev.h>
#include <doca_error.h>
#include <doca_pe.h>


#include <semaphore.h>                                  /* VBrusse */
                                                        // static = visible in ONE file only
                                                        // - worker_thread in nrLDPC_decod_server.c
                                                        //   cannot see it if defined in nrLDPC_decod.c

                                                        // extern = visible across multiple files
/* extern pthread_mutex_t cc_mutex; */                  // - visible everywhere
extern sem_t request_done;


/* #include <stdatomic.h>                       VBrusse */
#include <pthread.h>
/* #include <armral.h> */                               /* VBrusse - this file is already included in nrLDPC_encod_server.c - solve this redundancy later on */



#define CC_MODULE_CLIENT_DEC 0                          /* client decoder module */
#define CC_MODULE_CLIENT_ENC 1                          /* client encoder module */
#define MAX_SAMPLE_TXT_SIZE 4080                        /* Maximum size of user input text for the sample */
/* #define MAX_SAMPLE_TXT_SIZE 10240 */
#define MAX_TXT_SIZE (MAX_SAMPLE_TXT_SIZE + 1)          /* Maximum size of input text */
#define SLEEP_IN_NANOS (10 * 1000)                      /* Sample tasks every 10 microseconds */
#define CC_MAX_MSG_SIZE 10176                           /* Comm Channel maximum message size */


/*
 * For Base Graph 1 (BG1), Kb is typically used for large block sizes (up to 8448 bits).
 * For Base Graph 2 (BG2), Kb is used for smaller block sizes (up to 3840 bits).
 *
 * It has been reserved 10 blocks of 8448 bits (the maximum large block size) due to the coding rate.
 *
 * Ex: Encoded Block Size (N): for code rate R = 1/3, the encoded block size is: N=K/R = 512×3 = 1536 bits.
 *
 * To accommodate 10 blocks of 8448 bits, you first need to convert the total number of bits to bytes:
 *
 * Total bits=10×8448=84480 bits
 *
 * Since 1 byte = 8 bits, the total number of bytes is:
 *
 * Total bytes=84480/8=10560 bytes
 *
 * Therefore, the array length should be 10560 bytes to accommodate 10 blocks of 8448 bits.
 *
 *
 * 1. Determine Code Block Size
 *      For BG1 (K = 8448 bits)
 *              • Output bits: K = 8448 (maximum)
 *              • Bytes: 8448 / 8 = 1056 bytes
 *      For BG2 (K = 3840 bits)
 *              • Output bits: K = 3840 (maximum)
 *              • Bytes: 3840 / 8 = 480 bytes
 *
 * 2. Calculate Based on Lifting Size (Z)
 *      The actual decoded size is:
 *              decoded_bits = (K / Z) * Z_calculated
 *      Where:
 *              • Z = Lifting size (from 3GPP Table 5.3.2-1, e.g., 96 in your logs)
 *              • K = Base graph info bits (8448 for BG1, 3840 for BG2)
 *              • Z_calculated = Effective lifting after rate matching
 * Example
 *      • BG2 + Z = 96:
 *              o Info bits per block: K = 3840 / 96 = 40
 *              o Decoded bits: 40 * 96 = 3840 (same as BG2 max)
 *              o Output bytes: 3840 / 8 = 480 bytes
 *      • BG1 + Z = 96
 *              o Infor bits per block: K = 8448 / 96 = 88
 *              o Decoded bits: 88 * 96 = 8448 (same as BG1 max)
 *              o Output bytes: 8448 / 8 = 1056
 *
 * 3. Match to LLR Input (ex 2352 bytes = OAI Samples)
 *      LLR input is 2352 bytes:
 *              • Likely corresponds to rate-matched length (E) for one segment.
 *              • For BG2 + Z=96, typical rate-matched sizes are multiples of 2*Z = 192 bytes.
 *              • 2352 bytes / 192 = 12.25 → Likely 12 segments of 196 bytes each (adjusted for CRC).
 *
 * For implementation:
 *      • BG2 (most common for small blocks)
 *              uint8_t decoded_output[480];                            // 3840 bits = 480 bytes
 *      • BG1 (if used for larger blocks)
 *              uint8_t decoded_output[1056];                           // 8448 bits = 1056 bytes
 *
 */

/*
 * LLRs buffer size = N = 68 * Z for BG1 and N = 52 * Z for BG2.
 * Let consider the worst case BG1, so if Z = 96 in samples case and 3GPP defines N = 68 * Z (BG1) as the number of bytes in the LDPC codeblock
 *
 * N = 68 * 96 = 6528 bytes, each LLR is stored in 1 byte (int8_t).
 *
 * It will be used the LLR buffer sized defined by OAI:
 *
 *      #define NR_LDPC_MAX_NUM_LLR 27000       - Maximum number of possible input LLR = NR_LDPC_NCOL_BG1*NR_LDPC_ZMAX
 *
 */
/* #define CC_LDPC_ENC_BLOCK_LEN 1056 */                                /* Encoder Input/Output block length 1056 bytes = 8448 bits = 22*Z */
/* #define CC_LDPC_IN_BLOCK_LEN 27000 */                                /* The length of llrs. llrs buffer length shall be calculate as length 68 * Z t or BG1 and 52 * Z for BG2.
                                                                           The length shall be 68 * 384 = 26112 bits, but let it be as the same as NR_LDPC_MAX_NUM_LLR from OAI */

/* #define NR_LDPC_MAX_NUM_LLR 27000 */                                 /* Maximum number of possible input LLR = NR_LDPC_NCOL_BG1*NR_LDPC_ZMAX */
/* #define CC_LDPC_OUT_BLOCK_LEN 22*384 */                              /* The buffer length of the decoded bits. These are of length 22 * z for BG1 1 and 10 * z for BG2.
                                                                           The buffer length of the decoded bits are 22*384 = 8448 bits = 1056 bytes */
// BG1: Kb=22, Zmax=384 → K_cb = 22*384 = 8448 bits = 1056 bytes
// BG2: Kb=10, Zmax=384 → K_cb = 10*384 = 3840 bits =  480 bytes
// Max = 1056 bytes
#define CC_LDPC_ENC_IN_BLOCK_LEN  1056   /* Encoder input: max K_cb_bytes = 22*384/8 = 1056 bytes (BG1) */

// BG1: Kc=68, Zmax=384 → N = 68*384 = 26112 bits = 3264 bytes
// BG2: Kc=52, Zmax=384 → N = 52*384 = 19968 bits = 2496 bytes
// Max = 3264 bytes
#define CC_LDPC_ENC_OUT_BLOCK_LEN 3264   /* Encoder output: max N_bytes = 68*384/8 = 3264 bytes (BG1) */

// Decoder input = LLRs = n = N - 2*Z (punctured)
// BG1: max n = 68*384 - 2*384 = 66*384 = 25344 bits = 25344 bytes (unpacked, 1 LLR per byte)
// BG2: max n = 52*384 - 2*384 = 50*384 = 19200 bits = 19200 bytes (unpacked)
// Max = 25344 bytes
#define CC_LDPC_DEC_IN_BLOCK_LEN  27000/*25344*/  /* Decoder input: max LLRs = (68-2)*384 = 25344 bytes (BG1) */


// Decoder output = data_out = Kprime bytes (packed)
// BG1: Kb=22, Zmax=384 → K_cb = 1056 bytes
// BG2: Kb=10, Zmax=384 → K_cb =  480 bytes
// Max = 1056 bytes
#define CC_LDPC_DEC_OUT_BLOCK_LEN 1056   /* Decoder output: max data_out = 22*384/8 = 1056 bytes (BG1) */


#define CC_LDPC_OUT_BLOCK_LEN (68 * 384) / 8                            /* 3.264 bytes */
#define CC_LLR_SIZE 2360                                                /* decoding data: LDPC block length - 3072 to fit OAI Samples and based on param E of OAI interface */

/*
 * data_out buffer size is the K value
 *
 * In case of samples, K = 904 bits
 *
 * data_out buffer size = 904 / 8 = 113 bytes
 *
 */

/*
 * From /openairinterface5g/openair1/PHY/CODING/coding_defs.h
 */
#define CRC24_A 0
#define CRC24_B 1
#define CRC16 2
#define CRC8 3

#define CC_NUM_DPU_CORES 8                                              // BlueField-2 - Pin to core (0-7 for 8 cores), BlueField-3 - Pin to core (0-15 for 16 cores)
#define CC_THREADS_POOL_SIZE CC_NUM_DPU_CORES                           // Number threads in the pool is the number of DPU logical cores
#define CC_TASKS_QUEUE_SIZE 64                                          // Number of the Tasks in the queue. Queue capacity


struct server_objects_t {
        const char *server_name;
        const char *dev_pci_addr;
        const char *rep_pci_addr;
        struct task_queue_t *ptask_queue;
        struct task_t *ptask;
        void *sample_objects;
        int *thread_id;
};


struct task_t {                                         // Struct to represent a Task
        int task_id;                                    // Unique ID for the task
        // struct server_objects_t task_objects;
        // struct ldpc_encod_params_t *pldpc_encod_params;
        void *sample_objects;                           // task sample_objects
        struct comch_data_path_objects data_path;       // Argument for the task function - this field cannot be a poiner -> race conditions
        // pthread_mutex_t *mutex;
        doca_error_t result;                            // Result of the task execution
        int duration;                                   // Simulated processing time
};

struct task_queue_t {                                   // Struct to represent a Task circular queue
        struct task_t *tasks[CC_TASKS_QUEUE_SIZE];
        int head;                                       // Head of the task queue
        int tail;                                       // Tail of the task queue
        int count;                                      // Task counter
        int keep_running;                               // Flag to signal thread termination
        pthread_mutex_t lock;                           // Lock for the task queue
        pthread_cond_t not_full;                        // Condition variable for the task queue
        pthread_cond_t not_empty;                       // Condition variable for the task queue
};

struct thread_arg_t {                                   // Struct to represent the worker thread args
        struct task_queue_t *task_queue;
        int *pthread_id;
};



struct worker_queue_t {
        _Atomic size_t head;
        _Atomic size_t tail;
        struct task_t buffer[CC_TASKS_QUEUE_SIZE];
        pthread_t thread;                               // Store thread handle
        int worker_id;
};

struct global_node_t {
        struct task_t task;
        _Atomic(struct global_node_t *)next;
};

struct thread_pool_t {
        struct worker_queue_t workers[CC_NUM_DPU_CORES];   // CC_NUM_DPU_CORES = MAX_WORKERS
        _Atomic(struct global_node_t *)global_head;
        _Atomic(struct global_node_t *)global_tail;
        _Atomic int active_workers;
        _Atomic bool shutdown;
};




/*
struct lf_queue_t {
        void *buffer[CC_TASKS_QUEUE_SIZE];
        atomic_size_t head;
        atomic_size_t tail;
};

struct ldpc_task_t {
        int task_id;                                    // Unique ID for the task
        int thread_ids[CC_NUM_DPU_CORES];
        struct comch_data_path_objects *data_path;
};

struct ldpc_thread_arg_t {                              // Struct to represent the worker thread args
        struct lf_queue_t *queue;
        int thread_id;
};
*/


/* Definition of the encoder/decoder structs with memory range alignment for best performance using CPU memory, align address to 64B (cache-line size) */
struct __attribute__((aligned(64))) ldpc_encod_params_t {               /* Struct to represent/store the lppc encoding data input/output parameters */
        uint8_t inputBlock[CC_LDPC_ENC_IN_BLOCK_LEN] __attribute__((aligned(64)));      /* Input block <===== Testar shared memory com o host definindo este pointer e o array no host */
        /* unsigned char inputBlock[CC_LDPC_BLOCK_LEN]; */              /* "uint8_t" and "unsigned char" are interchangeable in terms of memory layout */
        uint8_t bg;                                                     /* Base Graph (BG). LDPC_BASE_GRAPH_1 = 0, LDPC_BASE_GRAPH_2 = 1 */
        uint32_t z;                                                     /* Lifting Size (Zc) */
        uint32_t k;                                                     /* Size in bits of the code segments - Block length (K) */
        uint32_t kb;                                                    /* Number of lifting sizes to fit the payload */
        uint32_t len_filler_bits;                                       /* Filler bits to pad the input block - Number of "Filler" bits */
        uint8_t outputBlock[CC_LDPC_ENC_OUT_BLOCK_LEN] __attribute__((aligned(64)));    /* Output block - codeword */
        /* unsigned char outputBlock[CC_LDPC_BLOCK_LEN]; */             /* "uint8_t" and "unsigned char" are interchangeable in terms of memory layout */
};

struct __attribute__((aligned(64))) ldpc_decod_params_t {               /* Struct to represent/store the ldpc decoding data input/output parameters */
        uint32_t n;                                                     /* The length of llrs. llrs buffer length shall be calculate as length 68 * Z t or BG1 and 52 * Z for BG2 */
        /* const int8_t *llrs; */                                       /* Pointer to the LLRs (soft values) */
        /* int e; */                                                    /* input llr segment size */
        uint32_t e;
        int8_t llrs[CC_LDPC_DEC_IN_BLOCK_LEN] __attribute__((aligned(64)));/* The initial LLRs to use in the decoding. This is typically the output after demodulation and rate recovery.
                                                                           LLRs (soft values) buffer - this host memory block shall be declared as an array, it can not be a pointer */
        /* int Kprime; */                                               /**< Size of the payload bits and CRC bits in the code block */
        uint32_t Kprime;
        uint32_t kp;                                                    /* 'Kprime' is the K' (= K_cb) in the standard 3GPP TS 38.212 section 5.2.2. It is the number of the payload bits                                                                            per uncoded segment. In other word, it is the number of useful bits in the output of the decoder */
        /* uint8_t bg; */                                               /* The type of base graph to use for the decoding. LDPC_BASE_GRAPH_1 = 0, LDPC_BASE_GRAPH_2 = 1 */
        uint32_t bg;
        uint32_t z;                                                     /* The Lifting Factor / Lifting Size. Valid values are described in table 5.3.2-1 in TS 38.212. Maximum Z = 384 */
        uint32_t len_filler_bits;                                       /* The number of filler bits. As per TS 38.212, section 5.2.2, filler bits insertion is needed to ensure that the
                                                                           code block segments have a valid length and are a multiple of the lifting size. Filler bits are used to
                                                                           calculate CRC internally. This is assumed to be a multiple of 8 */
        uint8_t data_out[CC_LDPC_DEC_OUT_BLOCK_LEN] __attribute__((aligned(64)));       /* The decoded bits. These are of length 22 * z for base graph 1 and 10 * z for base graph 2. It
                                                                                           is assumed that the array data_out is able to store this many bits. */
        /* uint8_t *data_out; */
        uint32_t its_max;                                               /* The maximum number of iterations of the LDPC decoder. The algorithm may terminate after fewer iterations if
                                                                           the current candidate codeword passes all the parity checks, or if it satisfies the CRC check. */
        /* int crc_type; */                                             /**< Size and type of the parity check bits (16, 24A or 24B) */
        uint32_t crc_type;
        uint32_t its_out;
        uint32_t options;                                               /* See the documentation above for a summary of available options. If you want to use the default options, set
                                                                           the options parameter to either 0 or ARMRAL_LDPC_DEFAULT_OPTIONS. */
};



struct comch_config {
        char comch_dev_pci_addr[DOCA_DEVINFO_PCI_ADDR_SIZE];            /* Comm Channel DOCA device PCI address */
        char comch_dev_rep_pci_addr[DOCA_DEVINFO_REP_PCI_ADDR_SIZE];    /* Comm Channel DOCA device representor PCI address */
        char text[MAX_TXT_SIZE];                                        /* Text to send to Comm Channel server */
        uint32_t text_size;                                             /* Text size to send to Comm Channel server */

        struct ldpc_encod_params_t ldpc_encod_params;                   /* VBrusse: struct with the data/input parameters of the LDPC encoder */
        struct ldpc_decod_params_t ldpc_decod_params;                   /* VBrusse: struct with the data/input parameters of the LDPC decoder */
};

struct comch_ctrl_path_client_cb_config {
        /* User specified callback when task completed successfully */
        doca_comch_task_send_completion_cb_t send_task_comp_cb;
        /* User specified callback when task completed with error */
        doca_comch_task_send_completion_cb_t send_task_comp_err_cb;
        /* User specified callback when a message is received */
        doca_comch_event_msg_recv_cb_t msg_recv_cb;
        /* Whether need to configure data_path related event callback */
        bool data_path_mode;
        /* User specified callback when a new consumer registered */
        doca_comch_event_consumer_cb_t new_consumer_cb;
        /* User specified callback when a consumer expired event occurs */
        doca_comch_event_consumer_cb_t expired_consumer_cb;
        /* User specified context data */
        void *ctx_user_data;
        /* User specified PE context state changed event callback */
        doca_ctx_state_changed_callback_t ctx_state_changed_cb;
};

struct comch_ctrl_path_server_cb_config {
        /* User specified callback when task completed successfully */
        doca_comch_task_send_completion_cb_t send_task_comp_cb;
        /* User specified callback when task completed with error */
        doca_comch_task_send_completion_cb_t send_task_comp_err_cb;
        /* User specified callback when a message is received */
        doca_comch_event_msg_recv_cb_t msg_recv_cb;
        /* User specified callback when server receives a new connection */
        doca_comch_event_connection_status_changed_cb_t server_connection_event_cb;
        /* User specified callback when server finds a disconnected connection */
        doca_comch_event_connection_status_changed_cb_t server_disconnection_event_cb;
        /* Whether need to configure data_path related event callback */
        bool data_path_mode;
        /* User specified callback when a new consumer registered */
        doca_comch_event_consumer_cb_t new_consumer_cb;
        /* User specified callback when a consumer expired event occurs */
        doca_comch_event_consumer_cb_t expired_consumer_cb;
        /* User specified context data */
        void *ctx_user_data;
        /* User specified PE context state changed event callback */
        doca_ctx_state_changed_callback_t ctx_state_changed_cb;
};

/*
 * Register the command line parameters for the DOCA CC samples
 *
 * @return: DOCA_SUCCESS on success and DOCA_ERROR otherwise
 */
doca_error_t register_comch_params(void);

/**
 * Clean client and its PE
 *
 * @client [in]: Client object to clean
 * @pe [in]: Client PE object to clean
 */
void clean_comch_ctrl_path_client(struct doca_comch_client *client, struct doca_pe *pe);

/**
 * Initialize a cc client and its PE
 *
 * @server_name [in]: Server name to connect to
 * @hw_dev [in]: Device to use
 * @cb_cfg [in]: Client callback configuration
 * @client [out]: Client object struct to initialize
 * @pe [out]: Client PE object struct to initialize
 * @return: DOCA_SUCCESS on success and DOCA_ERROR otherwise
 */
doca_error_t init_comch_ctrl_path_client(const char *server_name,
                                         struct doca_dev *hw_dev,
                                         struct comch_ctrl_path_client_cb_config *cb_cfg,
                                         struct doca_comch_client **client,
                                         struct doca_pe **pe);

/**
 * Clean server and its PE
 *
 * @server [in]: Server object to clean
 * @pe [in]: Server PE object to clean
 */
void clean_comch_ctrl_path_server(struct doca_comch_server *server, struct doca_pe *pe);

/**
 * Initialize a cc server and its PE
 *
 * @server_name [in]: Server name to connect to
 * @hw_dev [in]: Device to use
 * @rep_dev [in]: Representor device to use
 * @cb_cfg [in]: Server callback configuration
 * @server [out]: Server object struct to initialize
 * @pe [out]: Server PE object struct to initialize
 * @return: DOCA_SUCCESS on success and DOCA_ERROR otherwise
 */
doca_error_t init_comch_ctrl_path_server(const char *server_name,
                                         struct doca_dev *hw_dev,
                                         struct doca_dev_rep *rep_dev,
                                         struct comch_ctrl_path_server_cb_config *cb_cfg,
                                         struct doca_comch_server **server,
                                         struct doca_pe **pe);

/* #endif // COMCH_COMMON_H_ */
#endif // NRLDPC_COMMON_H_
