#pragma once

#include <string>

namespace vdsuit_wireless {

// Enables the SerialPortM::GetPortNo interposer exported by the executable.
// The SDK then opens this pseudo-terminal path instead of enumerating USB.
void setSdkVirtualSerialPath(const std::string& path);
void clearSdkVirtualSerialPath();

// Supplies the SDK dlopen handle so normal USB enumeration can call through to
// the vendor implementation when virtual serial mode is disabled.
void setSdkLibraryHandle(void* handle);

} // namespace vdsuit_wireless
