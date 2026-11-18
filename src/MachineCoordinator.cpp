#include "MachineCoordinator.hpp"

#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

namespace machine {

MachineCoordinator::MachineCoordinator(SetOutput setOutput, StopOutputs stopOutputs)
    : setOutput_(std::move(setOutput)), stopOutputs_(std::move(stopOutputs)) {
    if (!setOutput_ || !stopOutputs_) {
        throw std::invalid_argument("Arduino output callbacks must be provided");
    }
}

void MachineCoordinator::runWithOutputs(
    const std::vector<OutputCommand>& outputs,
    const std::function<void()>& motionAction) {
    if (outputs.empty()) {
        throw std::invalid_argument("At least one Arduino output is required");
    }
    if (!motionAction) {
        throw std::invalid_argument("XYZ motion action must be provided");
    }
    std::set<std::string> names;
    for (const auto& output : outputs) {
        if (output.name.empty() || !names.insert(output.name).second) {
            throw std::invalid_argument("Arduino output names must be non-empty and unique");
        }
    }

    // Serialize complete jobs; outputMutex_ remains available to interruptOutputs().
    std::lock_guard<std::mutex> sequenceLock(sequenceMutex_);
    try {
        {
            std::lock_guard<std::mutex> outputLock(outputMutex_);
            for (const auto& output : outputs) {
                setOutput_(output.name, output.value);
            }
        }
        // Do not hold outputMutex_ during synchronous motion, so outputs can be interrupted.
        motionAction();
    } catch (const std::exception& actionError) {
        try {
            std::lock_guard<std::mutex> outputLock(outputMutex_);
            stopOutputs_();
        } catch (const std::exception& stopError) {
            throw std::runtime_error(
                "Coordinated operation failed (" + std::string(actionError.what()) +
                ") and Arduino outputs could not be confirmed safe (" +
                stopError.what() + ")");
        }
        throw;
    } catch (...) {
        try {
            std::lock_guard<std::mutex> outputLock(outputMutex_);
            stopOutputs_();
        } catch (...) {
            throw std::runtime_error(
                "Coordinated operation failed and Arduino outputs could not be "
                "confirmed safe");
        }
        throw;
    }
    std::lock_guard<std::mutex> outputLock(outputMutex_);
    stopOutputs_();
}

void MachineCoordinator::runOutputAfterMotion(
    const OutputCommand& output,
    std::chrono::milliseconds duration,
    const std::function<void()>& motionAction) {
    if (output.name.empty()) {
        throw std::invalid_argument("Arduino output name must not be empty");
    }
    if (duration <= std::chrono::milliseconds::zero()) {
        throw std::invalid_argument("Arduino output duration must be positive");
    }
    if (!motionAction) {
        throw std::invalid_argument("XYZ motion action must be provided");
    }

    // Keep regular jobs ordered while still permitting an asynchronous output stop.
    std::lock_guard<std::mutex> sequenceLock(sequenceMutex_);
    try {
        motionAction();
        {
            std::lock_guard<std::mutex> outputLock(outputMutex_);
            setOutput_(output.name, output.value);
        }
        std::this_thread::sleep_for(duration);
    } catch (const std::exception& actionError) {
        try {
            std::lock_guard<std::mutex> outputLock(outputMutex_);
            stopOutputs_();
        } catch (const std::exception& stopError) {
            throw std::runtime_error(
                "Coordinated operation failed (" + std::string(actionError.what()) +
                ") and Arduino outputs could not be confirmed safe (" +
                stopError.what() + ")");
        } catch (...) {
            throw std::runtime_error(
                "Coordinated operation failed and Arduino outputs could not be "
                "confirmed safe");
        }
        throw;
    } catch (...) {
        try {
            std::lock_guard<std::mutex> outputLock(outputMutex_);
            stopOutputs_();
        } catch (...) {
            throw std::runtime_error(
                "Coordinated operation failed and Arduino outputs could not be "
                "confirmed safe");
        }
        throw;
    }

    std::lock_guard<std::mutex> outputLock(outputMutex_);
    stopOutputs_();
}

void MachineCoordinator::runOutputForDuration(
    const OutputCommand& output,
    std::chrono::milliseconds duration) {
    runOutputAfterMotion(output, duration, [] {});
}

void MachineCoordinator::interruptOutputs() {
    std::lock_guard<std::mutex> outputLock(outputMutex_);
    stopOutputs_();
}

} // namespace machine
