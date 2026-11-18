#include "EasyMachineLanguage.hpp"
#include "GpioController.hpp"
#include "MachineCoordinator.hpp"
#include "MarlinTransport.hpp"
#include "MotionController.hpp"

#include <chrono>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

struct Options {
    std::string marlinPort;
    std::string beltName{"A"};
    std::string arduinoConfigPath;
    std::string programPath;
    double maximumX{};
    double maximumY{};
    double maximumZ{};
    double maximumMoveMm{};
    double maximumFeedrateMmPerMinute{};
    long long maximumDurationMs{};
    long long commandTimeoutMs{};
    int marlinBaud{115200};
    std::filesystem::path configPath;
};

struct Instruction {
    enum class Kind { Move, Dispense, Wait, Assert };
    Kind kind;
    motion::Position target{0.0, 0.0, 0.0};
    std::array<bool, 3> movedAxes{false, false, false};
    double feedrateMmPerMinute{0.0};
    std::string outputName;
    int outputValue{0};
    long long durationMs{0};
    motion::Axis assertionAxis{motion::Axis::X};
    std::string comparison;
    double assertionValue{0.0};
};

void printUsage(const char* executable) {
    std::cout << "Usage: " << executable
              << " [--config FILE]\n"
              << "Default config: machine-coordinator.conf in the current directory.\n\n"
        << "Supported ASM instructions:\n"
        << "  <belt> X<target-mm> [Y<target-mm>] [Z<target-mm>] [F<feedrate-mm/min>]\n"
        << "  DISPENSE <DIGITAL-output> <duration-ms>\n"
        << "  WAIT <duration-ms>\n"
        << "  ASSERT <belt>.<X|Y|Z> <op> <number>\n"
        << "FOR/IF/ELSE are provided by EasyMachineLanguage.\n"
        << "The runner validates XYZ and requires interactive confirmation.\n";
}

std::string trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

double parseDouble(const std::string& text, const std::string& name) {
    std::size_t consumed = 0;
    double value = 0.0;
    try {
        value = std::stod(text, &consumed);
    } catch (const std::exception&) {
        throw std::invalid_argument("Invalid value for " + name + ": " + text);
    }
    if (consumed != text.size() || !std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Value for " + name + " must be finite and positive");
    }
    return value;
}

long long parseInteger(const std::string& text, const std::string& name) {
    std::size_t consumed = 0;
    long long value = 0;
    try {
        value = std::stoll(text, &consumed);
    } catch (const std::exception&) {
        throw std::invalid_argument("Invalid value for " + name + ": " + text);
    }
    if (consumed != text.size() || value <= 0) {
        throw std::invalid_argument("Value for " + name + " must be a positive integer");
    }
    return value;
}

Options loadOptions(const std::filesystem::path& configPath) {
    std::ifstream config(configPath);
    if (!config) {
        throw std::runtime_error("Cannot open machine config: " + configPath.string());
    }

    std::map<std::string, std::string> values;
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(config, line)) {
        ++lineNumber;
        const auto comment = line.find('#');
        if (comment != std::string::npos) line.erase(comment);
        line = trim(line);
        if (line.empty()) continue;
        const auto separator = line.find('=');
        if (separator == std::string::npos) {
            throw std::runtime_error(configPath.string() + ":" +
                                     std::to_string(lineNumber) +
                                     ": expected key = value");
        }
        const std::string key = trim(line.substr(0, separator));
        const std::string value = trim(line.substr(separator + 1));
        if (key.empty() || value.empty()) {
            throw std::runtime_error(configPath.string() + ":" +
                                     std::to_string(lineNumber) +
                                     ": config key and value must not be empty");
        }
        if (!values.emplace(key, value).second) {
            throw std::runtime_error(configPath.string() + ":" +
                                     std::to_string(lineNumber) +
                                     ": duplicate config key: " + key);
        }
    }
    if (config.bad()) {
        throw std::runtime_error("Error reading machine config: " + configPath.string());
    }

    Options options;
    options.configPath = std::filesystem::absolute(configPath).lexically_normal();
    const auto take = [&values, &configPath](const std::string& key) {
        const auto found = values.find(key);
        if (found == values.end()) {
            throw std::runtime_error(configPath.string() + ": missing config key: " + key);
        }
        const std::string value = found->second;
        values.erase(found);
        return value;
    };
    const auto resolvePath = [&options](const std::string& value) {
        // Resolve relative paths against the config file, not the caller's working directory.
        const std::filesystem::path path(value);
        return (path.is_absolute() ? path : options.configPath.parent_path() / path)
            .lexically_normal().string();
    };

    options.marlinPort = take("marlin_port");
    const auto beltName = values.find("belt_name");
    if (beltName != values.end()) {
        options.beltName = beltName->second;
        values.erase(beltName);
    }
    if (options.beltName.empty() ||
        std::any_of(options.beltName.begin(), options.beltName.end(), [](unsigned char ch) {
            return std::isspace(ch) != 0;
        })) {
        throw std::invalid_argument("belt_name must be a non-empty single token");
    }
    options.arduinoConfigPath = resolvePath(take("arduino_config"));
    options.programPath = resolvePath(take("program"));
    if (std::filesystem::path(options.programPath).extension() != ".asm") {
        throw std::invalid_argument("Configured program file must use the .asm extension");
    }
    options.maximumX = parseDouble(take("max_x_mm"), "max_x_mm");
    options.maximumY = parseDouble(take("max_y_mm"), "max_y_mm");
    options.maximumZ = parseDouble(take("max_z_mm"), "max_z_mm");
    options.maximumMoveMm = parseDouble(take("max_move_mm"), "max_move_mm");
    options.maximumFeedrateMmPerMinute =
        parseDouble(take("max_feedrate_mm_min"), "max_feedrate_mm_min");
    options.maximumDurationMs = parseInteger(take("max_duration_ms"), "max_duration_ms");
    options.commandTimeoutMs =
        parseInteger(take("command_timeout_ms"), "command_timeout_ms");
    const auto baud = values.find("marlin_baud");
    if (baud != values.end()) {
        const long long parsed = parseInteger(baud->second, "marlin_baud");
        if (parsed > std::numeric_limits<int>::max() ||
            (parsed != 9600 && parsed != 19200 && parsed != 38400 &&
             parsed != 57600 && parsed != 115200 && parsed != 230400)) {
            throw std::invalid_argument("Unsupported marlin_baud: " + baud->second);
        }
        options.marlinBaud = static_cast<int>(parsed);
        values.erase(baud);
    }
    if (!values.empty()) {
        throw std::runtime_error(configPath.string() + ": unknown config key: " +
                                 values.begin()->first);
    }
    if (options.marlinPort.empty()) {
        throw std::invalid_argument("marlin_port must not be empty");
    }
    if (options.maximumDurationMs >
            std::numeric_limits<std::chrono::milliseconds::rep>::max() ||
        options.commandTimeoutMs >
            std::numeric_limits<std::chrono::milliseconds::rep>::max()) {
        throw std::invalid_argument("Duration values exceed the supported range");
    }
    if (options.commandTimeoutMs < options.maximumDurationMs) {
        throw std::invalid_argument("command_timeout_ms must be at least max_duration_ms");
    }
    return options;
}

Options parseOptions(int argc, char* argv[]) {
    std::filesystem::path configPath = "machine-coordinator.conf";
    bool configSpecified = false;
    for (int index = 1; index < argc; ++index) {
        const std::string key = argv[index];
        if (key == "--help" || key == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (key == "--config" && index + 1 < argc) {
            if (configSpecified) throw std::invalid_argument("Duplicate option: --config");
            configPath = argv[++index];
            configSpecified = true;
            continue;
        }
        if (key.rfind("--", 0) != 0 || index + 1 >= argc) {
            throw std::invalid_argument("Unknown option or missing value: " + key);
        }
        throw std::invalid_argument("Unknown option: " + key);
    }
    return loadOptions(configPath);
}

Instruction parseInstruction(const std::string& text,
                             const std::string& beltName,
                             double defaultFeedrateMmPerMinute) {
    std::istringstream input(text);
    std::vector<std::string> words;
    std::string word;
    while (input >> word) words.push_back(word);
    if (words.empty()) throw std::invalid_argument("Instruction must not be empty");

    if (words[0] == beltName) {
        Instruction instruction{};
        instruction.kind = Instruction::Kind::Move;
        instruction.feedrateMmPerMinute = defaultFeedrateMmPerMinute;
        bool feedrateSeen = false;
        for (std::size_t index = 1; index < words.size(); ++index) {
            const std::string& token = words[index];
            if (token.size() > 1 && token[0] == 'F') {
                if (feedrateSeen || index + 1 != words.size()) {
                    throw std::invalid_argument("F<feedrate> must appear once at the end");
                }
                std::size_t consumed = 0;
                try {
                    instruction.feedrateMmPerMinute = std::stod(token.substr(1), &consumed);
                } catch (const std::exception&) {
                    throw std::invalid_argument("F feedrate must be a number");
                }
                if (consumed != token.size() - 1 ||
                    !std::isfinite(instruction.feedrateMmPerMinute) ||
                    instruction.feedrateMmPerMinute <= 0.0) {
                    throw std::invalid_argument("F feedrate must be finite and positive");
                }
                feedrateSeen = true;
                continue;
            }

            const std::size_t axisIndex =
                token[0] == 'X' ? 0 : token[0] == 'Y' ? 1 : token[0] == 'Z' ? 2 : 3;
            if (axisIndex == 3 || token.size() == 1 ||
                instruction.movedAxes[axisIndex]) {
                throw std::invalid_argument(
                    "Expected unique X<mm>, Y<mm>, Z<mm> coordinates followed by optional F<mm/min>");
            }
            std::size_t consumed = 0;
            double target = 0.0;
            try {
                target = std::stod(token.substr(1), &consumed);
            } catch (const std::exception&) {
                throw std::invalid_argument("Axis target must be a number: " + token);
            }
            if (consumed != token.size() - 1 || !std::isfinite(target)) {
                throw std::invalid_argument("Axis target must be finite: " + token);
            }
            if (axisIndex == 0) instruction.target.x = target;
            else if (axisIndex == 1) instruction.target.y = target;
            else instruction.target.z = target;
            instruction.movedAxes[axisIndex] = true;
        }
        if (std::none_of(instruction.movedAxes.begin(), instruction.movedAxes.end(),
                         [](bool moved) { return moved; })) {
            throw std::invalid_argument("A belt move must specify at least one axis target");
        }
        return instruction;
    }

    if (words[0] == "DISPENSE") {
        if (words.size() != 3) {
            throw std::invalid_argument(
                "DISPENSE syntax is: DISPENSE <DIGITAL-output> <duration-ms>");
        }
        Instruction instruction{};
        instruction.kind = Instruction::Kind::Dispense;
        instruction.outputName = words[1];
        instruction.outputValue = 1;
        instruction.durationMs = parseInteger(words[2], "DISPENSE duration");
        return instruction;
    }

    if (words[0] == "WAIT") {
        if (words.size() != 2) {
            throw std::invalid_argument("WAIT syntax is: WAIT <duration-ms>");
        }
        Instruction instruction{};
        instruction.kind = Instruction::Kind::Wait;
        instruction.durationMs = parseInteger(words[1], "WAIT duration");
        return instruction;
    }

    if (words[0] == "ASSERT") {
        if (words.size() != 4 ||
            words[1].compare(0, beltName.size() + 1, beltName + ".") != 0 ||
            (words[2] != "<" && words[2] != "<=" && words[2] != "==" &&
             words[2] != "!=" && words[2] != ">=" && words[2] != ">")) {
            throw std::invalid_argument(
                "ASSERT syntax is: ASSERT <belt>.<X|Y|Z> <op> <number>");
        }
        Instruction instruction{};
        instruction.kind = Instruction::Kind::Assert;
        const std::string axis = words[1].substr(beltName.size() + 1);
        if (axis == "X") instruction.assertionAxis = motion::Axis::X;
        else if (axis == "Y") instruction.assertionAxis = motion::Axis::Y;
        else if (axis == "Z") instruction.assertionAxis = motion::Axis::Z;
        else throw std::invalid_argument("ASSERT axis must be X, Y, or Z");
        instruction.comparison = words[2];
        std::size_t consumed = 0;
        try {
            instruction.assertionValue = std::stod(words[3], &consumed);
        } catch (const std::exception&) {
            throw std::invalid_argument("ASSERT value must be a number");
        }
        if (consumed != words[3].size() || !std::isfinite(instruction.assertionValue)) {
            throw std::invalid_argument("ASSERT value must be a finite number");
        }
        return instruction;
    }

    throw std::invalid_argument("Unknown instruction: " + words[0]);
}

bool compare(double lhs, const std::string& comparison, double rhs) {
    if (comparison == "<") return lhs < rhs;
    if (comparison == "<=") return lhs <= rhs;
    if (comparison == "==") return lhs == rhs;
    if (comparison == "!=") return lhs != rhs;
    if (comparison == ">=") return lhs >= rhs;
    if (comparison == ">") return lhs > rhs;
    throw std::invalid_argument("Invalid ASSERT operator");
}

motion::Axis parsePositionAxis(const std::string& identifier,
                               const std::string& beltName) {
    if (identifier == beltName + ".X" || identifier == "position.x") return motion::Axis::X;
    if (identifier == beltName + ".Y" || identifier == "position.y") return motion::Axis::Y;
    if (identifier == beltName + ".Z" || identifier == "position.z") return motion::Axis::Z;
    throw std::invalid_argument("Unknown motion identifier: " + identifier);
}

double coordinate(const motion::Position& position, motion::Axis axis) {
    switch (axis) {
    case motion::Axis::X: return position.x;
    case motion::Axis::Y: return position.y;
    case motion::Axis::Z: return position.z;
    }
    throw std::invalid_argument("Invalid axis");
}

class MotionProgramHost final : public easyml::ProgramHost {
public:
    MotionProgramHost(motion::MotionController& xyz,
                      machine::MachineCoordinator& coordinator,
                      const Options& options,
                      const std::map<std::string, arduino::OutputPin>& outputs,
                      motion::MotionLimits limits,
                      motion::Position initialPosition)
        : xyz_(xyz),
          coordinator_(coordinator),
          options_(options),
          outputs_(outputs),
          limits_(limits),
          position_(initialPosition) {}

    std::unique_ptr<easyml::ProgramHost> cloneForValidation() const override {
        return std::make_unique<MotionProgramHost>(*this);
    }

    void validateInstruction(const std::string& instruction,
                             const easyml::SourceLocation&) override {
        const Instruction parsed = parseInstruction(
            instruction, options_.beltName, options_.maximumFeedrateMmPerMinute);
        if (parsed.kind == Instruction::Kind::Move) {
            // Advance a simulated position so every later target is checked before hardware moves.
            motion::Position target = position_;
            if (parsed.movedAxes[0]) target.x = parsed.target.x;
            if (parsed.movedAxes[1]) target.y = parsed.target.y;
            if (parsed.movedAxes[2]) target.z = parsed.target.z;
            const std::array<double, 3> coordinates{target.x, target.y, target.z};
            const std::array<double, 3> maximums{
                limits_.maximumPosition.x, limits_.maximumPosition.y,
                limits_.maximumPosition.z};
            for (std::size_t index = 0; index < coordinates.size(); ++index) {
                if (coordinates[index] < 0.0 || coordinates[index] > maximums[index]) {
                    throw std::out_of_range("Belt move target exceeds configured axis limits");
                }
            }
            const double dx = target.x - position_.x;
            const double dy = target.y - position_.y;
            const double dz = target.z - position_.z;
            const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (distance > limits_.maximumDistancePerMoveMm) {
                throw std::out_of_range("Belt move exceeds configured maximum distance per move");
            }
            if (parsed.feedrateMmPerMinute > limits_.maximumFeedrateMmPerMinute) {
                throw std::out_of_range("Belt move exceeds configured maximum feedrate");
            }
            if (distance * 60000.0 / parsed.feedrateMmPerMinute >
                static_cast<double>(limits_.maximumDuration.count())) {
                throw std::out_of_range("Belt move exceeds configured maximum duration");
            }
            position_ = target;
            return;
        }

        if (parsed.kind == Instruction::Kind::Dispense) {
            // DISPENSE is intentionally restricted to a configured DIGITAL pump output.
            const auto output = outputs_.find(parsed.outputName);
            if (output == outputs_.end() ||
                output->second.mode != arduino::OutputMode::Digital) {
                throw std::invalid_argument(
                    "DISPENSE requires a configured DIGITAL output: " + parsed.outputName);
            }
            if (parsed.durationMs > 30000 ||
                totalDispenseMs_ + parsed.durationMs > 60000 ||
                dispenseCount_ >= 12) {
                throw std::out_of_range(
                    "Program exceeds the dispense limit (30 s each, 60 s total, 12 cycles)");
            }
            totalDispenseMs_ += parsed.durationMs;
            ++dispenseCount_;
            return;
        }

        if (parsed.kind == Instruction::Kind::Wait && parsed.durationMs > 60000) {
            throw std::out_of_range("WAIT duration must not exceed 60000 ms");
        }
        if (parsed.kind == Instruction::Kind::Assert) {
            const double value = parsed.assertionAxis == motion::Axis::X ? position_.x :
                                 parsed.assertionAxis == motion::Axis::Y ? position_.y :
                                                                          position_.z;
            if (!compare(value, parsed.comparison, parsed.assertionValue)) {
                throw std::runtime_error("ASSERT failed: " + instruction);
            }
        }
    }

    void executeInstruction(const std::string& instruction,
                            const easyml::SourceLocation& location,
                            std::ostream& output) override {
        const Instruction parsed = parseInstruction(
            instruction, options_.beltName, options_.maximumFeedrateMmPerMinute);
        if (parsed.kind == Instruction::Kind::Move) {
            output << location.source << ':' << location.line << ": " << options_.beltName;
            if (parsed.movedAxes[0]) output << " X" << parsed.target.x;
            if (parsed.movedAxes[1]) output << " Y" << parsed.target.y;
            if (parsed.movedAxes[2]) output << " Z" << parsed.target.z;
            output << " F" << parsed.feedrateMmPerMinute << '\n';
            motion::Position target = position_;
            if (parsed.movedAxes[0]) target.x = parsed.target.x;
            if (parsed.movedAxes[1]) target.y = parsed.target.y;
            if (parsed.movedAxes[2]) target.z = parsed.target.z;
            xyz_.moveTo(target, parsed.feedrateMmPerMinute);
            position_ = target;
            return;
        }
        if (parsed.kind == Instruction::Kind::Dispense) {
            output << location.source << ':' << location.line << ": DISPENSE "
                   << parsed.outputName << ' ' << parsed.outputValue << ' '
                   << parsed.durationMs << " ms\n";
            coordinator_.runOutputForDuration(
                {parsed.outputName, parsed.outputValue},
                std::chrono::milliseconds(parsed.durationMs));
            return;
        }
        if (parsed.kind == Instruction::Kind::Assert) {
            const double value = parsed.assertionAxis == motion::Axis::X ? position_.x :
                                 parsed.assertionAxis == motion::Axis::Y ? position_.y :
                                                                          position_.z;
            if (!compare(value, parsed.comparison, parsed.assertionValue)) {
                throw std::runtime_error("ASSERT failed: " + instruction);
            }
            output << location.source << ':' << location.line << ": ASSERT passed\n";
            return;
        }
        output << location.source << ':' << location.line << ": WAIT "
               << parsed.durationMs << " ms\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(parsed.durationMs));
    }

    void validateIdentifier(const std::string& identifier,
                            const easyml::SourceLocation&) const override {
        (void)parsePositionAxis(identifier, options_.beltName);
    }

    double resolveIdentifier(const std::string& identifier,
                             const easyml::SourceLocation&) override {
        return coordinate(position_, parsePositionAxis(identifier, options_.beltName));
    }

    void mergeValidatedBranches(const easyml::ProgramHost& trueBranch,
                                const easyml::ProgramHost& falseBranch) override {
        const auto* trueMotion = dynamic_cast<const MotionProgramHost*>(&trueBranch);
        const auto* falseMotion = dynamic_cast<const MotionProgramHost*>(&falseBranch);
        if (trueMotion == nullptr || falseMotion == nullptr) {
            throw std::invalid_argument("Cannot merge incompatible validation branches");
        }
        // Following absolute moves are safe only if either runtime branch leaves the same endpoint.
        if (trueMotion->position_.x != falseMotion->position_.x ||
            trueMotion->position_.y != falseMotion->position_.y ||
            trueMotion->position_.z != falseMotion->position_.z) {
            throw std::invalid_argument(
                "IF branches must end at the same XYZ position for safe subsequent moves");
        }
        position_ = trueMotion->position_;
        totalDispenseMs_ =
            std::max(trueMotion->totalDispenseMs_, falseMotion->totalDispenseMs_);
        dispenseCount_ = std::max(trueMotion->dispenseCount_, falseMotion->dispenseCount_);
    }

private:
    motion::MotionController& xyz_;
    machine::MachineCoordinator& coordinator_;
    const Options& options_;
    const std::map<std::string, arduino::OutputPin>& outputs_;
    motion::MotionLimits limits_;
    motion::Position position_;
    long long totalDispenseMs_{0};
    int dispenseCount_{0};
};

bool confirmHoming() {
    std::cout << "Homing will move all XYZ axes. Clear the machine and verify "
                 "the emergency stop is accessible.\n"
              << "Start XYZ homing now? [y/N] (N skips homing and continues to run confirmation) "
              << std::flush;
    std::string response;
    if (!std::getline(std::cin, response)) {
        throw std::runtime_error("Could not read the XYZ homing choice");
    }
    return response == "y" || response == "Y";
}

bool confirmExecution(const std::string& programPath,
                      motion::Position startPosition,
                      bool homingCompleted) {
    std::cout << "Review the validated ASM file: " << programPath << '\n'
              << (homingCompleted
                      ? "Homing is complete. "
                      : "Homing was skipped. Verify the reported coordinates match "
                        "the physical machine position. ")
              << "Starting at X=" << startPosition.x << " Y=" << startPosition.y
              << " Z=" << startPosition.z << " mm. Confirm clear travel and "
                 "pump-driver safety.\n"
              << "Execute this hardware program? [y/N] " << std::flush;
    std::string response;
    if (!std::getline(std::cin, response)) {
        throw std::runtime_error("Could not read the ASM execution choice");
    }
    return response == "y" || response == "Y";
}

double parseCoordinate(const std::string& text, const std::string& axis) {
    std::size_t consumed = 0;
    double value = 0.0;
    try {
        value = std::stod(text, &consumed);
    } catch (const std::exception&) {
        throw std::invalid_argument("Invalid " + axis + " coordinate: " + text);
    }
    if (consumed != text.size() || !std::isfinite(value)) {
        throw std::invalid_argument("Invalid " + axis + " coordinate: " + text);
    }
    return value;
}

motion::Position confirmOrCorrectPosition(motion::MotionController& xyz,
                                          motion::Position reportedPosition) {
    std::cout << "Marlin-reported position: X=" << reportedPosition.x
              << " Y=" << reportedPosition.y << " Z=" << reportedPosition.z << " mm\n"
              << "Are these coordinates physically correct? [y/N] "
              << std::flush;
    std::string response;
    if (!std::getline(std::cin, response)) {
        throw std::runtime_error("Could not read the coordinate confirmation");
    }
    if (response == "y" || response == "Y") {
        return reportedPosition;
    }
    if (!response.empty() && response != "n" && response != "N") {
        throw std::invalid_argument("Enter y to keep the coordinates or n to correct them");
    }

    std::cout << "With the machine stationary, enter its physically verified X Y Z "
                 "coordinates in mm: "
              << std::flush;
    std::string coordinates;
    if (!std::getline(std::cin, coordinates)) {
        throw std::runtime_error("Could not read the corrected XYZ coordinates");
    }
    std::istringstream values(coordinates);
    std::string xText;
    std::string yText;
    std::string zText;
    std::string extra;
    if (!(values >> xText >> yText >> zText) || (values >> extra)) {
        throw std::invalid_argument("Enter exactly three coordinates: X Y Z");
    }
    const motion::Position corrected{
        parseCoordinate(xText, "X"),
        parseCoordinate(yText, "Y"),
        parseCoordinate(zText, "Z")};
    std::cout << "Setting Marlin's logical XYZ position to the entered coordinates; "
                 "the machine will not move.\n";
    xyz.setCurrentPosition(corrected);
    const motion::Position synchronized = xyz.synchronizePositionFromFirmware();
    std::cout << "Marlin now reports X=" << synchronized.x << " Y=" << synchronized.y
              << " Z=" << synchronized.z << " mm\n";
    return synchronized;
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        const Options options = parseOptions(argc, argv);
        if (!::isatty(STDIN_FILENO)) {
            throw std::runtime_error("Interactive terminal required for physical-motion confirmation");
        }

        std::ifstream programFile(options.programPath);
        if (!programFile) {
            throw std::runtime_error("Cannot open ASM program: " + options.programPath);
        }
        const easyml::Program program = easyml::Program::parse(programFile, options.programPath);

        std::ifstream configFile(options.arduinoConfigPath);
        if (!configFile) {
            throw std::runtime_error("Cannot open Arduino config: " + options.arduinoConfigPath);
        }
        auto arduinoConfig =
            arduino::GpioConfiguration::load(configFile, options.arduinoConfigPath);
        const auto configuredOutputs = arduinoConfig.outputs;
        arduino::GpioController arduino(std::move(arduinoConfig));

        const motion::MotionLimits limits{
            {options.maximumX, options.maximumY, options.maximumZ},
            options.maximumMoveMm,
            options.maximumFeedrateMmPerMinute,
            std::chrono::milliseconds(options.maximumDurationMs),
            std::chrono::milliseconds(options.commandTimeoutMs)};
        auto marlinTransport =
            std::make_unique<motion::MarlinTransport>(options.marlinPort, options.marlinBaud);
        auto* marlinTransportView = marlinTransport.get();
        motion::MotionController xyz(std::move(marlinTransport), limits);

        machine::MachineCoordinator coordinator(
            [&arduino](const std::string& name, int value) {
                arduino.setOutput(name, value);
                arduino.checkHealth();
            },
            [&arduino] { arduino.stopAll(); });

        // Check the complete program against the configured limits at the homed origin
        // before connecting to either controller or initiating any physical motion.
        MotionProgramHost startupValidator(
            xyz, coordinator, options, configuredOutputs, limits,
            {0.0, 0.0, 0.0});
        program.validate(startupValidator);
        std::cout << "ASM preflight passed at the homed origin; configured limits are satisfied.\n";

        arduino.connect();
        arduino.stopAll();
        xyz.connect();
        const bool homingCompleted = confirmHoming();
        motion::Position startPosition{};
        if (homingCompleted) {
            try {
                marlinTransportView->sendCommand(
                    "G28", std::chrono::milliseconds(options.commandTimeoutMs));
            } catch (const std::exception& homingError) {
                try {
                    marlinTransportView->sendCommand("M410", std::chrono::seconds(2));
                } catch (const std::exception& quickstopError) {
                    throw std::runtime_error(
                        "XYZ homing failed (" + std::string(homingError.what()) +
                        "); M410 quickstop request failed (" + quickstopError.what() +
                        "). Motion may still be active and position is unknown");
                }
                throw std::runtime_error(
                    "XYZ homing failed (" + std::string(homingError.what()) +
                    "); M410 quickstop was acknowledged, but a physical stop is not verified");
            }
            std::cout << "XYZ homing completed.\n";
            // Trust the origin only after the G28 homing command has completed.
            startPosition = xyz.synchronizePositionFromFirmware();
            std::cout << "Marlin reports X=" << startPosition.x << " Y=" << startPosition.y
                      << " Z=" << startPosition.z << " mm\n";
            if (startPosition.x != 0.0 || startPosition.y != 0.0 ||
                startPosition.z != 0.0) {
                throw std::runtime_error(
                    "XYZ homing did not establish the expected X=0 Y=0 Z=0 origin");
            }
        } else {
            std::cout << "XYZ homing skipped; reading the current firmware position.\n";
            startPosition = xyz.synchronizePositionFromFirmware();
            std::cout << "Marlin reports X=" << startPosition.x << " Y=" << startPosition.y
                      << " Z=" << startPosition.z << " mm\n";
            startPosition = confirmOrCorrectPosition(xyz, startPosition);
        }
        // Validate again against the actual starting position, especially when homing was skipped.
        MotionProgramHost startupPositionValidator(
            xyz, coordinator, options, configuredOutputs, limits, startPosition);
        program.validate(startupPositionValidator);
        if (!confirmExecution(options.programPath, startPosition, homingCompleted)) {
            throw std::runtime_error("ASM execution was not confirmed");
        }
        arduino.checkHealth();

        MotionProgramHost executor(xyz, coordinator, options, configuredOutputs, limits,
                                   startPosition);
        try {
            program.execute(executor, std::cout);
        } catch (const std::exception& programError) {
            try {
                coordinator.interruptOutputs();
            } catch (const std::exception& stopError) {
                throw std::runtime_error(
                    "ASM execution failed (" + std::string(programError.what()) +
                    ") and Arduino outputs could not be confirmed safe (" +
                    stopError.what() + ")");
            }
            throw;
        }
        std::cout << "ASM program completed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MachineCoordinator: " << error.what() << '\n';
        return 1;
    }
}
