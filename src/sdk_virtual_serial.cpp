#include "sdk_virtual_serial.h"

#include <dlfcn.h>

#include <list>
#include <mutex>
#include <string>

namespace {

using PortList = std::list<std::string>;

std::mutex gVirtualSerialMutex;
std::string gVirtualSerialPath;
void* gSdkLibraryHandle = nullptr;

const char* kGetPortNoSymbol =
    "_ZN11SerialPortM9GetPortNoERNSt7__cxx114listINS0_12basic_stringIcSt11char_traitsIcESaIcEEESaIS6_EEES9_";

} // namespace

namespace vdsuit_wireless {

void setSdkVirtualSerialPath(const std::string& path)
{
    std::lock_guard<std::mutex> lock(gVirtualSerialMutex);
    // SerialPortM::Open prepends "/dev/" to names returned by GetPortNo.
    // openpty(3), conversely, reports an absolute /dev/pts/N path.
    const std::string devicePrefix = "/dev/";
    gVirtualSerialPath = path.compare(0, devicePrefix.size(), devicePrefix) == 0
        ? path.substr(devicePrefix.size())
        : path;
}

void clearSdkVirtualSerialPath()
{
    setSdkVirtualSerialPath(std::string());
}

void setSdkLibraryHandle(void* handle)
{
    std::lock_guard<std::mutex> lock(gVirtualSerialMutex);
    gSdkLibraryHandle = handle;
}

} // namespace vdsuit_wireless

// This declaration intentionally mirrors the class and method name exported by
// the proprietary SDK. No object fields are accessed; only the Itanium C++ ABI
// method symbol is interposed.
class SerialPortM {
public:
    static bool GetPortNo(PortList& primary, PortList& secondary);
};

__attribute__((visibility("default"), noinline))
bool SerialPortM::GetPortNo(PortList& primary, PortList& secondary)
{
    std::string virtualPath;
    void* sdkHandle = nullptr;
    {
        std::lock_guard<std::mutex> lock(gVirtualSerialMutex);
        virtualPath = gVirtualSerialPath;
        sdkHandle = gSdkLibraryHandle;
    }

    if (!virtualPath.empty()) {
        primary.push_back(virtualPath);
        return true;
    }

    if (!sdkHandle) return false;
    using OriginalFunction = bool (*)(PortList&, PortList&);
    dlerror();
    OriginalFunction original = reinterpret_cast<OriginalFunction>(
        dlsym(sdkHandle, kGetPortNoSymbol));
    if (dlerror() || !original) return false;
    return original(primary, secondary);
}
