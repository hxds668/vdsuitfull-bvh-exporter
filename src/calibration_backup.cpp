#include "calibration_backup.h"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <limits.h>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

namespace vdsuit {
namespace {

const char* const kCalibrationFileNames[] = {
    "CQ_0.xml",
    "CQ_HR_0.xml",
    "CQ_HL_0.xml"
};

std::string joinPath(const std::string& directory, const std::string& fileName)
{
    if (directory.empty()) return fileName;
    if (directory.back() == '/') return directory + fileName;
    return directory + '/' + fileName;
}

bool readWholeFile(const std::string& path, std::string& contents)
{
    std::ifstream input(path.c_str(), std::ios::binary);
    if (!input) return false;
    std::ostringstream buffer;
    buffer << input.rdbuf();
    if (input.bad()) return false;
    contents = buffer.str();
    return true;
}

CalibrationFileState captureFileState(const std::string& directory,
                                      const std::string& fileName)
{
    CalibrationFileState state;
    state.fileName = fileName;

    const std::string path = joinPath(directory, fileName);
    struct stat info {};
    if (stat(path.c_str(), &info) != 0 || !S_ISREG(info.st_mode)) return state;

    state.exists = true;
    state.modifiedSeconds = static_cast<long long>(info.st_mtim.tv_sec);
    state.modifiedNanoseconds = info.st_mtim.tv_nsec;
    state.readable = readWholeFile(path, state.contents);
    return state;
}

bool fileStateChanged(const CalibrationFileState& before,
                      const CalibrationFileState& after)
{
    if (before.exists != after.exists) return true;
    if (!after.exists) return false;
    return before.modifiedSeconds != after.modifiedSeconds ||
           before.modifiedNanoseconds != after.modifiedNanoseconds ||
           before.readable != after.readable ||
           before.contents != after.contents;
}

std::string timestampedFileName(const std::string& fileName,
                                const std::string& timestamp,
                                int suffix)
{
    std::string stem = fileName;
    std::string extension;
    std::size_t dot = fileName.rfind('.');
    if (dot != std::string::npos) {
        stem = fileName.substr(0, dot);
        extension = fileName.substr(dot);
    }

    std::ostringstream name;
    name << stem << '_' << timestamp;
    if (suffix > 0) name << '_' << suffix;
    name << extension;
    return name.str();
}

std::string chooseUnusedBackupPath(const std::string& directory,
                                   const std::string& fileName,
                                   const std::string& timestamp)
{
    for (int suffix = 0; suffix < 10000; ++suffix) {
        std::string path = joinPath(
            directory, timestampedFileName(fileName, timestamp, suffix));
        if (access(path.c_str(), F_OK) != 0) return path;
    }
    return std::string();
}

bool writeExclusiveFile(const std::string& path,
                        const std::string& contents,
                        std::string& error)
{
    int descriptor = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
    if (descriptor < 0) {
        error = std::strerror(errno);
        return false;
    }

    std::size_t offset = 0;
    while (offset < contents.size()) {
        ssize_t written = write(
            descriptor, contents.data() + offset, contents.size() - offset);
        if (written < 0) {
            if (errno == EINTR) continue;
            error = std::strerror(errno);
            close(descriptor);
            unlink(path.c_str());
            return false;
        }
        if (written == 0) {
            error = "write returned zero bytes";
            close(descriptor);
            unlink(path.c_str());
            return false;
        }
        offset += static_cast<std::size_t>(written);
    }

    if (close(descriptor) != 0) {
        error = std::strerror(errno);
        unlink(path.c_str());
        return false;
    }
    return true;
}

const CalibrationFileState* findState(
    const std::vector<CalibrationFileState>& states,
    const std::string& fileName)
{
    for (const CalibrationFileState& state : states) {
        if (state.fileName == fileName) return &state;
    }
    return nullptr;
}

} // namespace

std::string calibrationDirectoryForCurrentExecutable()
{
    char executablePath[PATH_MAX + 1];
    ssize_t length = readlink("/proc/self/exe", executablePath, PATH_MAX);
    if (length <= 0) return std::string();
    executablePath[length] = '\0';

    std::string path(executablePath);
    std::size_t separator = path.rfind('/');
    if (separator == std::string::npos) return "CalibrationFiles";
    return path.substr(0, separator) + "/CalibrationFiles";
}

std::string calibrationSnapshotTimestamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm localTime {};
    localtime_r(&time, &localTime);
    long long milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;

    std::ostringstream timestamp;
    timestamp << std::put_time(&localTime, "%Y%m%d_%H%M%S")
              << '_' << std::setw(3) << std::setfill('0') << milliseconds;
    return timestamp.str();
}

std::vector<CalibrationFileState> captureCalibrationFileStates(
    const std::string& directory)
{
    std::vector<CalibrationFileState> states;
    for (const char* fileName : kCalibrationFileNames) {
        states.push_back(captureFileState(directory, fileName));
    }
    return states;
}

CalibrationBackupResult backupChangedCalibrationFiles(
    const std::string& directory,
    const std::vector<CalibrationFileState>& before,
    const std::string& timestamp)
{
    CalibrationBackupResult result;
    if (directory.empty()) {
        result.errors.push_back("could not resolve the executable CalibrationFiles directory");
        return result;
    }

    std::vector<CalibrationFileState> after = captureCalibrationFileStates(directory);
    for (const CalibrationFileState& current : after) {
        const CalibrationFileState* previous = findState(before, current.fileName);
        CalibrationFileState missingPrevious;
        missingPrevious.fileName = current.fileName;
        if (!previous) previous = &missingPrevious;
        if (!fileStateChanged(*previous, current)) continue;

        result.anyFileChanged = true;
        if (!current.exists) {
            result.errors.push_back(current.fileName + " was removed instead of saved");
            continue;
        }
        if (!current.readable) {
            result.errors.push_back("could not read " + current.fileName + " after calibration");
            continue;
        }

        std::string backupPath = chooseUnusedBackupPath(
            directory, current.fileName, timestamp);
        if (backupPath.empty()) {
            result.errors.push_back("could not choose a unique backup name for " + current.fileName);
            continue;
        }

        std::string error;
        if (!writeExclusiveFile(backupPath, current.contents, error)) {
            result.errors.push_back("could not save " + backupPath + ": " + error);
            continue;
        }
        result.savedPaths.push_back(backupPath);
    }
    return result;
}

} // namespace vdsuit
