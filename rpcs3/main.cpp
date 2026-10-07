#include "stdafx.h"
#include "rpcs3.h"
#include "Crypto/decrypt_binaries.h"

#include <cstdlib>

LOG_CHANNEL(sys_log, "SYS");

int main(int argc, char** argv)
{
	// RB_DECRYPT=<file>[;<file>...]: decrypt PS3 binaries (firmware SPRX, SELF) like Utilities -> Decrypt PS3 Binaries,
	// without the GUI, then exit (native rewrite: the firmware's libsre for ps3recomp). No KLIC prompt: firmware files
	// do not need one.
	if (const char* list = std::getenv("RB_DECRYPT"); list && *list)
	{
		std::vector<std::string> modules;

		for (const auto& path : fmt::split(list, {";"}))
		{
			modules.push_back(path);
		}

		decrypt_binaries_t decrypter(std::move(modules));
		decrypter.decrypt();
		return decrypter.done() ? 0 : 1;
	}

	const int exit_code = run_rpcs3(argc, argv);
	sys_log.notice("RPCS3 terminated with exit code %d", exit_code);
	return exit_code;
}
