#include "GpioController.hpp"
#include "MachineCoordinator.hpp"
#include "MotionController.hpp"

#include <chrono>
#include <thread>

void moveWithIndicator(motion::MotionController& xyz,
                       arduino::GpioController& arduino) {
    machine::MachineCoordinator coordinator(
        [&arduino](const std::string& name, int value) {
            arduino.setOutput(name, value);
        },
        [&arduino] {
            arduino.stopAll();
        });

    coordinator.runWithOutputs({{"indicator", 1}}, [&xyz] {
        xyz.moveTo({100.0, 50.0, 20.0}, 600.0);
    });
}

void moveToCupThenDispense(motion::MotionController& xyz,
                           machine::MachineCoordinator& coordinator) {
    coordinator.runOutputAfterMotion(
        {"pump_driver", 1}, std::chrono::seconds(5), [&xyz] {
            xyz.moveTo(motion::Axis::X, 110.0, 600.0);
        });
}
