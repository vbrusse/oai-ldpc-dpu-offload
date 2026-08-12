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
 * Original filename: comch_data_path_high_speed_server_main.c
 *
 * Filename: nrLDPC_encod.c
 *
 * DOCA Communication Channel Server API customized by: Vlademir Brusse
 *
 * Date: 2025/09/24
 *
 */

#define _GNU_SOURCE                                     /* VBrusse - Enable GNU extensions */
                                                        /* Some functions like pthread_setaffinity_np are declared only if specific feature macros are defined */

#include <stdio.h>                                      /* VBrusse */
#include <sched.h>                                      /* VBrusse - for pinning cpu */


#include <stdlib.h>

#include <doca_argp.h>
#include <doca_dev.h>
#include <doca_log.h>

/* #include "comch_ctrl_path_common.h" */
#include "nrLDPC_ctrl_path_common.h"

#define DEFAULT_PCI_ADDR "03:00.0"
#define DEFAULT_REP_PCI_ADDR "b1:00.0"
#define DEFAULT_MESSAGE "Message from the server"       /* VBrusse */

DOCA_LOG_REGISTER(NRLDPC_ENCOD_SERVER::MAIN);


extern doca_error_t nrLDPC_argp_init(int type, void *cfg);
void task_queue_init(struct task_queue_t *q);
void cleanup(struct task_queue_t *q, pthread_t *threads);
void *worker_thread(void *arg);



/* DOCA comch encoder server's logic */
doca_error_t start_nrLDPC_encod_server(const char *server_name,
                                       const char *dev_pci_addr,
                                       const char *rep_pci_addr,
                                       struct ldpc_encod_params_t *pldpc_encod_params,
                                       struct task_queue_t *ptask_queue);


/*
 * "nrLDPC_encod_server" - This server exposures the Offloading Service of the 5G NR LDPC function
 * compute to the DPU to execute the DOCA Communication Channel API client using the ConnectX adapter and
 * the PCIe interface (nrLDPC_encod) to run the ArmRAL LDPC encoder kernel. This function makes the function
 * offloading and leverages the DPU's hardware acceleration capabilities to efficiently execute the LDPC
 * encoding task, potentially improving performance and reducing the CPU load.
 *
 * @argc [in]: Command line arguments size
 * @argv [in]: Array of command line arguments
 *
 * @return: EXIT_SUCCESS on success and EXIT_FAILURE otherwise
 */

/*
 * Sample main function
 *
 * @argc [in]: Command line arguments size
 * @argv [in]: Array of command line arguments
 * @return: EXIT_SUCCESS on success and EXIT_FAILURE otherwise
 */
int main(int argc, char **argv)
{
        struct comch_config cfg;
        const char *server_name = "nrLDPC_encod_server";
        doca_error_t result;
        // struct doca_log_backend *sdk_log;
        int exit_status = EXIT_FAILURE;


        pthread_t thread_pool[CC_THREADS_POOL_SIZE];                                    /* Threads pool size */
        int thread_ids[CC_THREADS_POOL_SIZE];

        cpu_set_t cpuset;                                                               /* CPU affinity */


        /* Set the default configuration values */
        /* strcpy(cfg.comch_dev_pci_addr, DEFAULT_PCI_ADDR); */
        /* strcpy(cfg.comch_dev_rep_pci_addr, DEFAULT_REP_PCI_ADDR); */
        /* strcpy(cfg.text, DEFAULT_MESSAGE); */                                        /* VBrusse */


        strcpy(cfg.comch_dev_pci_addr, "03:00.0");
        strcpy(cfg.comch_dev_rep_pci_addr, "2a:00.0");

        result = nrLDPC_argp_init(CC_MODULE_CLIENT_ENC, &cfg);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to init ARGP: %s", doca_error_get_descr(result));
                // retcode = -1;
                goto sample_exit;
        }


        /* Register a logger backend */
/*      result = doca_log_backend_create_standard();
        if (result != DOCA_SUCCESS)
                goto sample_exit;
*/
        /* Register a logger backend for internal SDK errors and warnings */
/*      result = doca_log_backend_create_with_file_sdk(stderr, &sdk_log);
        if (result != DOCA_SUCCESS)
                goto sample_exit;
        result = doca_log_backend_set_sdk_level(sdk_log, DOCA_LOG_LEVEL_WARNING);
        if (result != DOCA_SUCCESS)
                goto sample_exit;
*/
        DOCA_LOG_INFO("Starting the sample");

        /* Parse cmdline/json arguments */
/*      result = doca_argp_init(NULL, &cfg);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to init ARGP resources: %s", doca_error_get_descr(result));
                goto sample_exit;
        }

        result = register_comch_params();
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to register Comm Channel server sample parameters: %s",
                             doca_error_get_descr(result));
                goto argp_cleanup;
        }

        result = doca_argp_start(argc, argv);
        if (result != DOCA_SUCCESS) {
                DOCA_LOG_ERR("Failed to parse sample input: %s", doca_error_get_descr(result));
                goto argp_cleanup;
        }
*/
        /* Start the server */



        struct task_queue_t *task_queue = malloc(sizeof(struct task_queue_t));                  /* Define task circular queue */

        task_queue_init(task_queue);                                                            /* Initialize the queue */

        // 1. Init semaphore ONCE here
        sem_init(&request_done, 0, 0);

        // 2. Give each thread its OWN thread_arg
        struct thread_arg_t thread_args[CC_THREADS_POOL_SIZE];

        // Create the thread pool, then wait for them to finish
        DOCA_LOG_INFO("%d threads will be created", CC_THREADS_POOL_SIZE);

        for (int i = 0; i < CC_THREADS_POOL_SIZE; i++) {                                        /* Spawn the worker threads according to number of DPU logical cores */
                thread_ids[i] = i;                                                              /* core=1 <=> id=1, ..., core=8 <=> id=8 */
                thread_args[i].task_queue = task_queue;                                         // shared queue OK
                thread_args[i].pthread_id = &thread_ids[i];                                     // own ID

                if (pthread_create(&thread_pool[i], NULL, worker_thread, &thread_args[i]) != 0) {   /* Create the worker threads */
                        DOCA_LOG_INFO("Failed to create the thread %d", thread_ids[i]);
                }

                DOCA_LOG_INFO("Thread %d has been created successful, %lu", thread_ids[i], thread_pool[i]);
                DOCA_LOG_INFO("task_queue.thread_id %d", *thread_args[i].pthread_id);


                // Set thread affinity to a specific core
                // Initialize the CPU set
                CPU_ZERO(&cpuset);

                // Add the thread_id (core) to the CPU set
                /* core_id = i + 1; */                                  /* cpu id = 1 to 8 */
                /* CPU_SET(core_id, &cpuset); */
                CPU_SET(thread_ids[i] % CC_NUM_DPU_CORES, &cpuset);     /* BlueField-2 - Pin to core (0-7 for 8 cores), BlueField-3 - Pin to core (0-15 for 16 cores) */

                // Set the CPU affinity for the current thread
                // Pin threads to cores (pthread_setaffinity_np()) to avoid excessive context switching
                if (pthread_setaffinity_np(thread_pool[i], sizeof(cpu_set_t), &cpuset) != 0) {
                        DOCA_LOG_INFO("Failed to set pthread affinity %d", thread_ids[i]);
                }
                                                                                        /* mod(8) for BlueField-2 and mod(16) for BlueField-3 */
                DOCA_LOG_INFO("Thread_id %d pinned to core %d", thread_ids[i], thread_ids[i] % CC_NUM_DPU_CORES);
        }


        DOCA_LOG_INFO("LDPC Encoding Server running...");                       /* VBrusse */


        while (1) {


                result = start_nrLDPC_encod_server(server_name,
                                                   cfg.comch_dev_pci_addr,
                                                   cfg.comch_dev_rep_pci_addr,
                                                   &cfg.ldpc_encod_params,
                                                   task_queue);
                if (result != DOCA_SUCCESS) {
                        DOCA_LOG_ERR("Failed to run the sample: %s", doca_error_get_descr(result));
                        goto argp_cleanup;
                }

                sem_wait(&request_done);                                        // wait for worker before next iteration
        }

        DOCA_LOG_INFO("LDPC Encoding Server stopping...");                      /* VBrusse */

        cleanup(task_queue, &thread_pool[0]);

        free(task_queue);                                                       /* Destroe the queue when does not need it anymore */

        sem_destroy(&request_done);


        exit_status = EXIT_SUCCESS;

argp_cleanup:
        doca_argp_destroy();
sample_exit:
        if (exit_status == EXIT_SUCCESS)
                DOCA_LOG_INFO("Sample finished successfully");
        else
                DOCA_LOG_INFO("Sample finished with errors");
        return exit_status;
}
