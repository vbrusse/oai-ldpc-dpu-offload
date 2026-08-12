#
# Copyright (c) 2023-2024 NVIDIA CORPORATION AND AFFILIATES.  All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright notice, this list of
#       conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright notice, this list of
#       conditions and the following disclaimer in the documentation and/or other materials
#       provided with the distribution.
#     * Neither the name of the NVIDIA CORPORATION nor the names of its contributors may be used
#       to endorse or promote products derived from this software without specific prior written
#       permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TOR (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#
# Filename: meson.build - configuration file for LDPC decoder sever
#
# Customized by: Vlademir Brusse
#
# Date: 2026/06/30
#

project('DOCA_LDPC', 'C', 'CPP',
        # Get version number from file.
        version: run_command(find_program('cat'),
                files('../../../VERSION'), check: true).stdout().strip(),
        license: 'BSD-3',
        default_options: ['buildtype=debug'],
        meson_version: '>= 0.61.2'
)

SAMPLE_NAME = 'nrLDPC_decod'

# Comment this line to restore warnings of experimental DOCA features
add_project_arguments('-D DOCA_ALLOW_EXPERIMENTAL_API', language: ['c', 'cpp'])

sample_dependencies = []
# Required for all DOCA programs
sample_dependencies += dependency('doca-common')
# The DOCA library of the sample itself
sample_dependencies += dependency('doca-comch')
# Utility DOCA library for executables
sample_dependencies += dependency('doca-argp')

# Add pthread dependency
pthread_dep = dependency('threads')

# Get C++ compiler
cpp = meson.get_compiler('cpp')

# Find ArmRAL shared libraries using the C++ compiler
armral_dep = cpp.find_library('armral', dirs: '/home/vlademir/armral/build')
armral_utils_dep = cpp.find_library('armral_utils', dirs: '/home/vlademir/armral/build')

sample_srcs = [
        # The sample itself
        SAMPLE_NAME + '_server.c',
        # Main function for the sample's executable
        SAMPLE_NAME + '.c',
        # Common code for the DOCA library samples
        #'../comch_ctrl_path_common.c',
        '../nrLDPC_ctrl_path_common.c',
        '../nrLDPC_data_path_high_speed_common.c',
        # Common code for all DOCA samples
        '../../common.c',
]

sample_inc_dirs  = []
# Common DOCA library logic
sample_inc_dirs += include_directories('..')
# Common DOCA logic (samples)
sample_inc_dirs += include_directories('../..')
# Common DOCA logic
sample_inc_dirs += include_directories('../../..')
# Common DOCA logic (applications)
sample_inc_dirs += include_directories('../../../applications/common/')
# ArmRAL (ldpc kernel)
#sample_inc_dirs += include_directories('/home/vlademir/armral/ral-armral-25.01/include')
sample_inc_dirs += include_directories('/home/vlademir/armral/ral-armral-26.01/include')

# Add ArmRAL libraries to the dependencies list
sample_dependencies += [armral_dep, armral_utils_dep, pthread_dep]

executable(SAMPLE_NAME + '_server', sample_srcs,
        c_args : '-Wno-missing-braces',
        dependencies : sample_dependencies,
        include_directories: sample_inc_dirs,
        install: false)
