#include "MachineCoordinator.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

void expect(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void testOutputsRemainActiveThroughSynchronousMotion() {
    std::vector<std::string> events;
    machine::MachineCoordinator coordinator(
        [&events](const std::string& name, int value) {
            events.push_back("SET " + name + " " + std::to_string(value));
        },
        [&events] { events.push_back("STOP"); });

    coordinator.runWithOutputs({{"pump", 1}, {"indicator", 1}},
                               [&events] { events.push_back("XYZ MOVE COMPLETE"); });

    expect(events == std::vector<std::string>{
                         "SET pump 1", "SET indicator 1", "XYZ MOVE COMPLETE", "STOP"},
           "Outputs should be acknowledged before motion and stopped after completion");
}

void testMotionFailureStopsOutputs() {
    std::vector<std::string> events;
    machine::MachineCoordinator coordinator(
        [&events](const std::string&, int) { events.push_back("SET"); },
        [&events] { events.push_back("STOP"); });

    bool originalFailurePreserved = false;
    try {
        coordinator.runWithOutputs({{"pump", 1}}, [] {
            throw std::runtime_error("motion failed");
        });
    } catch (const std::runtime_error& error) {
        originalFailurePreserved = std::string(error.what()) == "motion failed";
    }
    expect(originalFailurePreserved, "The original motion failure should be rethrown");
    expect(events == std::vector<std::string>{"SET", "STOP"},
           "Arduino outputs should be stopped when motion fails");
}

void testOutputRunsAfterMotionForRequestedDuration() {
    std::vector<std::string> events;
    machine::MachineCoordinator coordinator(
        [&events](const std::string& name, int value) {
            events.push_back("SET " + name + " " + std::to_string(value));
        },
        [&events] { events.push_back("STOP"); });

    const auto start = std::chrono::steady_clock::now();
    coordinator.runOutputAfterMotion(
        {"pump_driver", 255}, std::chrono::milliseconds(50),
        [&events] { events.push_back("MOVE X=110 COMPLETE"); });
    const auto elapsed = std::chrono::steady_clock::now() - start;

    expect(events == std::vector<std::string>{
                         "MOVE X=110 COMPLETE", "SET pump_driver 255", "STOP"},
           "Pump output should activate only after the X move has completed");
    expect(elapsed >= std::chrono::milliseconds(50),
           "Pump output should remain active for the requested duration");
}

void testOutputAfterMotionFailureDoesNotActivatePump() {
    std::vector<std::string> events;
    machine::MachineCoordinator coordinator(
        [&events](const std::string& name, int value) {
            events.push_back("SET " + name + " " + std::to_string(value));
        },
        [&events] { events.push_back("STOP"); });

    bool originalFailurePreserved = false;
    try {
        coordinator.runOutputAfterMotion(
            {"pump_driver", 255}, std::chrono::milliseconds(1), [&] {
                events.push_back("MOVE FAILED");
                throw std::runtime_error("motion failed");
            });
    } catch (const std::runtime_error& error) {
        originalFailurePreserved = std::string(error.what()) == "motion failed";
    }
    expect(originalFailurePreserved, "The original motion failure should be rethrown");
    expect(events == std::vector<std::string>{"MOVE FAILED", "STOP"},
           "Pump output must not activate after failed motion and outputs must stop");
}

void testOutputAfterMotionStopsOnActivationFailure() {
    int stopCount = 0;
    machine::MachineCoordinator coordinator(
        [](const std::string&, int) { throw std::runtime_error("set failed"); },
        [&] { ++stopCount; });

    bool originalFailurePreserved = false;
    try {
        coordinator.runOutputAfterMotion(
            {"pump_driver", 255}, std::chrono::milliseconds(1), [] {});
    } catch (const std::runtime_error& error) {
        originalFailurePreserved = std::string(error.what()) == "set failed";
    }
    expect(originalFailurePreserved, "Output activation failure should be reported");
    expect(stopCount == 1, "Output stop should be requested after activation failure");
}

void testOutputForDurationActivatesAndStops() {
    std::vector<std::string> events;
    machine::MachineCoordinator coordinator(
        [&events](const std::string& name, int value) {
            events.push_back("SET " + name + " " + std::to_string(value));
        },
        [&events] { events.push_back("STOP"); });

    coordinator.runOutputForDuration(
        {"pump_driver", 128}, std::chrono::milliseconds(10));
    expect(events == std::vector<std::string>{"SET pump_driver 128", "STOP"},
           "Timed output should activate and then stop Arduino outputs");
}

void testOutputAfterMotionValidatesArguments() {
    machine::MachineCoordinator coordinator(
        [](const std::string&, int) {},
        [] {});

    bool rejectedDuration = false;
    try {
        coordinator.runOutputAfterMotion({"pump_driver", 255},
                                         std::chrono::milliseconds::zero(), [] {});
    } catch (const std::invalid_argument&) {
        rejectedDuration = true;
    }
    expect(rejectedDuration, "Non-positive output duration must be rejected");

    bool rejectedName = false;
    try {
        coordinator.runOutputAfterMotion({"", 255}, std::chrono::milliseconds(1), [] {});
    } catch (const std::invalid_argument&) {
        rejectedName = true;
    }
    expect(rejectedName, "Empty output name must be rejected");
}

void testExplicitInterruptStopsOutputs() {
    int stopCount = 0;
    machine::MachineCoordinator coordinator(
        [](const std::string&, int) {},
        [&stopCount] { ++stopCount; });
    coordinator.interruptOutputs();
    expect(stopCount == 1, "Interrupt should immediately request safe Arduino outputs");
}

void testInterruptCanStopOutputsDuringMotion() {
    std::mutex mutex;
    std::condition_variable condition;
    bool motionStarted = false;
    bool continueMotion = false;
    int stopCount = 0;
    machine::MachineCoordinator coordinator(
        [](const std::string&, int) {},
        [&] {
            std::lock_guard<std::mutex> lock(mutex);
            ++stopCount;
            condition.notify_all();
        });

    std::exception_ptr workerFailure;
    std::thread worker([&] {
        try {
            coordinator.runWithOutputs({{"pump", 1}}, [&] {
                std::unique_lock<std::mutex> lock(mutex);
                motionStarted = true;
                condition.notify_all();
                condition.wait(lock, [&] { return continueMotion; });
            });
        } catch (...) {
            workerFailure = std::current_exception();
        }
    });

    {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait_for(lock, std::chrono::seconds(1), [&] { return motionStarted; });
    }
    expect(motionStarted, "Synchronous motion callback should start");
    coordinator.interruptOutputs();
    {
        std::lock_guard<std::mutex> lock(mutex);
        expect(stopCount == 1, "Interrupt should stop outputs while motion is still active");
        continueMotion = true;
    }
    condition.notify_all();
    worker.join();
    if (workerFailure) std::rethrow_exception(workerFailure);
    expect(stopCount == 2, "Outputs should be stopped again after motion completes");
}

} // namespace

int main() {
    try {
        testOutputsRemainActiveThroughSynchronousMotion();
        testMotionFailureStopsOutputs();
        testOutputRunsAfterMotionForRequestedDuration();
        testOutputAfterMotionFailureDoesNotActivatePump();
        testOutputAfterMotionStopsOnActivationFailure();
        testOutputForDurationActivatesAndStops();
        testOutputAfterMotionValidatesArguments();
        testExplicitInterruptStopsOutputs();
        testInterruptCanStopOutputsDuringMotion();
    } catch (const std::exception& error) {
        std::cerr << "Test failure: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "All MachineCoordinator tests passed\n";
    return EXIT_SUCCESS;
}
