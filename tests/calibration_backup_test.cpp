#include "calibration_backup.h"

#include <cassert>
#include <cctype>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

using namespace vdsuit;

namespace {

void testTimestampFormat()
{
    std::string timestamp = calibrationSnapshotTimestamp();
    assert(timestamp.size() == 19);
    assert(timestamp[8] == '_');
    assert(timestamp[15] == '_');
    for (std::size_t i = 0; i < timestamp.size(); ++i) {
        if (i == 8 || i == 15) continue;
        assert(std::isdigit(static_cast<unsigned char>(timestamp[i])) != 0);
    }
}

std::string joinPath(const std::string& directory, const std::string& name)
{
    return directory + '/' + name;
}

void writeFile(const std::string& path, const std::string& contents)
{
    std::ofstream output(path.c_str(), std::ios::binary);
    assert(output);
    output << contents;
    output.close();
    assert(output);
}

std::string readFile(const std::string& path)
{
    std::ifstream input(path.c_str(), std::ios::binary);
    assert(input);
    return std::string(
        (std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
}

void testChangedFilesAreTimestampedWithoutReplacingCanonicalFiles()
{
    char directoryTemplate[] = "/tmp/vdsuit_calibration_backup_test_XXXXXX";
    char* createdDirectory = mkdtemp(directoryTemplate);
    assert(createdDirectory != nullptr);
    std::string directory(createdDirectory);

    const std::string body = joinPath(directory, "CQ_0.xml");
    const std::string right = joinPath(directory, "CQ_HR_0.xml");
    const std::string left = joinPath(directory, "CQ_HL_0.xml");
    writeFile(body, "body-before");
    writeFile(right, "right-before");
    writeFile(left, "left-before");

    std::vector<CalibrationFileState> before =
        captureCalibrationFileStates(directory);
    writeFile(body, "body-after");

    const std::string timestamp = "20260827_204501_123";
    CalibrationBackupResult first =
        backupChangedCalibrationFiles(directory, before, timestamp);
    assert(first.anyFileChanged);
    assert(first.errors.empty());
    assert(first.savedPaths.size() == 1);
    assert(first.savedPaths[0] == joinPath(directory, "CQ_0_" + timestamp + ".xml"));
    assert(readFile(first.savedPaths[0]) == "body-after");
    assert(readFile(body) == "body-after");
    assert(access(joinPath(directory, "CQ_HR_0_" + timestamp + ".xml").c_str(), F_OK) != 0);

    before = captureCalibrationFileStates(directory);
    writeFile(body, "body-newer");
    CalibrationBackupResult second =
        backupChangedCalibrationFiles(directory, before, timestamp);
    assert(second.errors.empty());
    assert(second.savedPaths.size() == 1);
    assert(second.savedPaths[0] == joinPath(directory, "CQ_0_" + timestamp + "_1.xml"));
    assert(readFile(second.savedPaths[0]) == "body-newer");

    unlink(first.savedPaths[0].c_str());
    unlink(second.savedPaths[0].c_str());
    unlink(body.c_str());
    unlink(right.c_str());
    unlink(left.c_str());
    assert(rmdir(directory.c_str()) == 0);
}

void testNoChangeCreatesNoBackup()
{
    char directoryTemplate[] = "/tmp/vdsuit_calibration_no_change_test_XXXXXX";
    char* createdDirectory = mkdtemp(directoryTemplate);
    assert(createdDirectory != nullptr);
    std::string directory(createdDirectory);

    const std::string body = joinPath(directory, "CQ_0.xml");
    writeFile(body, "unchanged");
    std::vector<CalibrationFileState> before =
        captureCalibrationFileStates(directory);

    CalibrationBackupResult result = backupChangedCalibrationFiles(
        directory, before, "20260827_204502_456");
    assert(!result.anyFileChanged);
    assert(result.savedPaths.empty());
    assert(result.errors.empty());

    unlink(body.c_str());
    assert(rmdir(directory.c_str()) == 0);
}

} // namespace

int main()
{
    testTimestampFormat();
    testChangedFilesAreTimestampedWithoutReplacingCanonicalFiles();
    testNoChangeCreatesNoBackup();
    std::cout << "calibration_backup_test: OK\n";
    return 0;
}
