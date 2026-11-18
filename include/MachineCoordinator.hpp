#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace machine {

struct OutputCommand {
    std::string name;
    int value;
};

/**
 * Coordinates Arduino outputs with synchronous XYZ operations.
 *
 * Motion actions are expected to return only when the motion controller has
 * completed its operation. runWithOutputs activates outputs before motion;
 * runOutputAfterMotion activates one output after motion for a fixed duration.
 * Both methods request safe outputs after failures.
 */
class MachineCoordinator {
public:
    using SetOutput = std::function<void(const std::string&, int)>;
    using StopOutputs = std::function<void()>;

    MachineCoordinator(SetOutput setOutput, StopOutputs stopOutputs);

    void runWithOutputs(const std::vector<OutputCommand>& outputs,
                        const std::function<void()>& motionAction);

    /**
     * Complete a synchronous motion action, activate one Arduino output for
     * the requested duration, then stop outputs. The output is stopped if the
     * motion, output activation, or wait fails. Duration is a minimum; system
     * scheduling can make the actual active time slightly longer.
     */
    void runOutputAfterMotion(const OutputCommand& output,
                              std::chrono::milliseconds duration,
                              const std::function<void()>& motionAction);

    /** Activate one output for a fixed duration, then stop all outputs. */
    void runOutputForDuration(const OutputCommand& output,
                              std::chrono::milliseconds duration);

    /** Interrupt Arduino outputs; this does not stop an active XYZ move. */
    void interruptOutputs();

private:
    SetOutput setOutput_;
    StopOutputs stopOutputs_;
    std::mutex sequenceMutex_;
    std::mutex outputMutex_;
};

} // namespace machine
