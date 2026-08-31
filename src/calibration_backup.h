#pragma once

#include <string>
#include <vector>

namespace vdsuit {

struct CalibrationFileState {
    std::string fileName;
    bool exists = false;
    bool readable = false;
    long long modifiedSeconds = 0;
    long modifiedNanoseconds = 0;
    std::string contents;
};

struct CalibrationBackupResult {
    bool anyFileChanged = false;
    std::vector<std::string> savedPaths;
    std::vector<std::string> errors;
};

std::string calibrationDirectoryForCurrentExecutable();
std::string calibrationSnapshotTimestamp();
std::vector<CalibrationFileState> captureCalibrationFileStates(
    const std::string& directory);
CalibrationBackupResult backupChangedCalibrationFiles(
    const std::string& directory,
    const std::vector<CalibrationFileState>& before,
    const std::string& timestamp);

} // namespace vdsuit
