cd ~/vdsuitfull-bvh-exporter

python3 - <<'PY'
from pathlib import Path

p = Path("src/main.cpp")
s = p.read_text()

if "#include <cstdio>" not in s:
    s = s.replace(
        "#include <cstring>\n#include <ctime>\n",
        "#include <cstring>\n#include <cstdio>\n#include <cstdlib>\n#include <ctime>\n"
    )

marker = "using namespace VDSuitMiniDevice;\n\nnamespace {\n"
insert = """using namespace VDSuitMiniDevice;

// Keep our console UI visible even when vendor SDK writes noisy printf/stdout logs.
// After this macro, existing std::cout calls become std::cerr calls.
#define cout cerr

namespace {

void redirectVendorStdout()
{
    const char* keep = std::getenv("VDSUIT_KEEP_SDK_STDOUT");
    if (keep && std::strcmp(keep, "0") != 0) return;

    std::fflush(stdout);
    if (std::freopen("/tmp/vdsuit_sdk_stdout.log", "a", stdout)) {
        std::setvbuf(stdout, nullptr, _IOLBF, 0);
    }
}

"""

if "void redirectVendorStdout()" not in s:
    s = s.replace(marker, insert)

old = '''    std::string libPath = chooseLibPath(options.libPath);
    std::cout << "SDK library: " << libPath << "\\n";

    SdkApi sdk;
    if (!sdk.load(libPath)) return 2;
'''

new = '''    std::string libPath = chooseLibPath(options.libPath);
    std::cout << "SDK library: " << libPath << "\\n";
    redirectVendorStdout();

    SdkApi sdk;
    if (!sdk.load(libPath)) return 2;
'''

if "redirectVendorStdout();" not in s:
    s = s.replace(old, new)

p.write_text(s)
print("patched src/main.cpp")
PY