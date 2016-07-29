/****************************************************************************
 *
 *   Copyright (C) 2015-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file main.cpp
 *
 * This is the main() of PX4 for POSIX.
 *
 * The application is designed as a daemon/server app with multiple clients.
 * Both, the server and the client is started using this main() function.
 *
 * If the executable is called with its usual name 'px4', it will start the
 * server. However, if it is started with an alias starting with 'px4-' such
 * as 'px4-navigator', it will start as a client and try to connect to the
 * server.
 *
 * The alias for all modules need to be created using the build system.
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <fstream>
#include <string>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include <px4_log.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_log.h>

#include "px4_middleware.h"
#include "DriverFramework.hpp"
#include "px4_middleware.h"
#include "px4_daemon/client.h"
#include "px4_daemon/server.h"
#include "px4_daemon/pxh.h"


static const char *LOCK_FILE_PATH = "/tmp/px4_lock";

#ifndef PATH_MAX
#define PATH_MAX 1024
#endif


static bool _exit_requested = false;


namespace px4
{
void init_once(void);
}

extern "C" {
	static void _SigIntHandler(int sig_num);
	static void _SigFpeHandler(int sig_num);
}

static void register_sig_handler();
static void set_cpu_scaling();
static void run_startup_bash_script(const char *commands_file);
static void use_chroot();
static void wait_to_exit();
static bool is_already_running();
static void print_usage();


int main(int argc, char **argv)
{
	bool is_client = false;
	bool chroot_on = false;
	bool pxh_off = false;
	char *commands_file = nullptr;

	/* Symlinks point to all commands that can be used as a client
	 * with a 'px4-' prefix. */

	const char prefix[] = "px4-";

	if (strstr(argv[0], prefix)) {
		is_client = true;
	}

	if (argc < 2) {
		PX4_ERR("Not enough arguments.");
		print_usage();
		return -1;
	}

	if (is_client) {

		if (!is_already_running()) {
			PX4_ERR("PX4 daemon not running yet");
			return -1;
		}

		/* Remove the prefix by moving argv[0] and 1 more for the 0 termination. */
		memmove(argv[0], argv[0] + strlen(prefix), strlen(argv[0]) + 1);

		px4_daemon::Client client;
		client.generate_uuid();
		client.register_sig_handler();
		return client.process_args(argc, (const char **)argv);

	} else {
		/* Server/daemon apps need to parse the command line arguments. */
		for (int i = 1; i < argc; ++i) {
			if (argv[i][0] == '-') {

				if (strcmp(argv[i], "-h") == 0) {
					print_usage();
					return 0;

				} else if (strcmp(argv[i], "-d") == 0) {
					pxh_off = true;

				} else if (strcmp(argv[i], "-c") == 0) {
					chroot_on = true;

				} else {
					PX4_ERR("Unknown/unhandled parameter: %s", argv[i]);
					print_usage();
					return 1;
				}

			} else {
				// This is an argument that does not have '-' it must be the file name.
				std::ifstream infile(argv[i]);

				if (infile.good()) {
					infile.close();
					commands_file = argv[i];

				} else {
					PX4_ERR("Error opening file: %s", argv[i]);
					return -1;
				}
			}
		}

		if (is_already_running()) {
			PX4_ERR("PX4 daemon already running");
			return -1;
		}

		if (chroot_on) {
			// Lock this application in the current working dir
			// this is not an attempt to secure the environment,
			// rather, to replicate a deployed file system.
			use_chroot();
		}

		px4_daemon::Server server;
		server.start();

		register_sig_handler();
		set_cpu_scaling();

		DriverFramework::Framework::initialize();

		px4::init_once();
		px4::init(argc, argv, "px4");

		run_startup_bash_script(commands_file);

		// We now block here until we need to exit.
		if (pxh_off) {
			wait_to_exit();

		} else {
			px4_daemon::Pxh pxh;
			pxh.run_pxh();
		}

		// When we exit, we need to stop muorb on Snapdragon.

#ifdef __PX4_POSIX_EAGLE
		// Sending muorb stop is needed if it is running to exit cleanly.
		// TODO: we should check with px4_task_is_running("muorb") before stopping it.
		std::string muorb_stop_cmd("muorb stop");
		px4_daemon::Pxh::process_line(muorb_stop_cmd, true);
#endif

		std::string shutdown_cmd("shutdown");
		px4_daemon::Pxh::process_line(shutdown_cmd, true);

		return OK;
	}
}

static void register_sig_handler()
{
	struct sigaction sig_int;
	memset(&sig_int, 0, sizeof(struct sigaction));
	sig_int.sa_handler = _SigIntHandler;
	sig_int.sa_flags = 0;// not SA_RESTART!;

	struct sigaction sig_fpe;
	memset(&sig_fpe, 0, sizeof(struct sigaction));
	sig_fpe.sa_handler = _SigFpeHandler;
	sig_fpe.sa_flags = 0;// not SA_RESTART!;

	// We want to ignore if a PIPE has been closed.
	struct sigaction sig_pipe;
	memset(&sig_pipe, 0, sizeof(struct sigaction));
	sig_pipe.sa_handler = SIG_IGN;

	sigaction(SIGINT, &sig_int, NULL);
	//sigaction(SIGTERM, &sig_int, NULL);
	sigaction(SIGFPE, &sig_fpe, NULL);
	sigaction(SIGPIPE, &sig_pipe, NULL);
}

static void _SigIntHandler(int sig_num)
{
	fflush(stdout);
	printf("\nExiting...\n");
	fflush(stdout);
	px4_daemon::Pxh::stop();
	_exit_requested = true;
}

static void _SigFpeHandler(int sig_num)
{
	fflush(stdout);
	printf("\nfloating point exception\n");
	PX4_BACKTRACE();
	fflush(stdout);
}

static void set_cpu_scaling()
{
#ifdef __PX4_POSIX_EAGLE
	// On Snapdragon we miss updates in sdlog2 unless all 4 CPUs are run
	// at the maximum frequency all the time.
	// Interestingely, cpu0 and cpu3 set the scaling for all 4 CPUs on Snapdragon.
	system("echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
	system("echo performance > /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor");

	// Alternatively we could also raise the minimum frequency to save some power,
	// unfortunately this still lead to some drops.
	//system("echo 1190400 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq");
#endif
}

static void run_startup_bash_script(const char *commands_file)
{
	std::string bash_command("bash ");

	bash_command += commands_file;

	PX4_INFO("Calling bash script: %s", bash_command.c_str());

	int ret = system(bash_command.c_str());

	if (ret == 0) {
		PX4_INFO("Startup script returned successfully");

	} else {
		PX4_WARN("Startup script returned with return value: %d", ret);
	}
}

static void use_chroot()
{
	char pwd_path[PATH_MAX];
	const char *folderpath = "/../rootfs/";

	if (nullptr == getcwd(pwd_path, sizeof(pwd_path))) {
		PX4_ERR("Failed acquiring working dir, abort.");
		exit(1);
	}

	if (nullptr == strcat(pwd_path, folderpath)) {
		PX4_ERR("Failed completing path, abort.");
		exit(1);
	}

	if (chroot(pwd_path)) {
		PX4_ERR("Failed chrooting application, path: %s, error: %s.", pwd_path, strerror(errno));
		exit(1);
	}

	if (chdir("/")) {
		PX4_ERR("Failed changing to root dir, path: %s, error: %s.", pwd_path, strerror(errno));
		exit(1);
	}
}

static void wait_to_exit()
{
	while (!_exit_requested) {
		usleep(100000);
	}
}

static void print_usage()
{
	printf("Usage for Server/daemon process: \n");
	printf("\n");
	printf("    px4 [-h|-c|-d] <startup_file>\n");
	printf("\n");
	printf("    <startup_config> bash start script to be used as startup\n");
	printf("        -h           help/usage information\n");
	printf("        -c           use chroot\n");
	printf("        -d           daemon mode, don't start pxh shell\n");
	printf("\n");
	printf("Usage for client: \n");
	printf("\n");
	printf("    px4-MODULE command using an alias.\n");
	printf("        e.g.: px4-commander status\n");
}

static bool is_already_running()
{
	struct flock fl;
	int fd = open(LOCK_FILE_PATH, O_RDWR | O_CREAT, 0666);

	if (fd < 0) {
		return false;
	}

	fl.l_type   = F_WRLCK;
	fl.l_whence = SEEK_SET;
	fl.l_start  = 0;
	fl.l_len    = 0;
	fl.l_pid    = getpid();

	if (fcntl(fd, F_SETLK, &fl) == -1) {
		// We failed to create a file lock, must be already locked.

		if (errno == EACCES || errno == EAGAIN) {
			return true;
		}
	}

	return false;
}

bool px4_exit_requested(void)
{
	return _exit_requested;
}
