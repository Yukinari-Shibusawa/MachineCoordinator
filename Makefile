BUILD_DIR ?= build
CMAKE ?= cmake
CTEST ?= ctest
BUILD_COORDINATED_EXAMPLE ?= ON
ARDUINO_CLI_EXECUTABLE ?= $(shell command -v arduino-cli 2>/dev/null || printf '%s' '$(HOME)/.local/bin/arduino-cli')
ARDUINO_FQBN ?= arduino:renesas_uno:minima
ARDUINO_UPLOAD_PORT ?=

.PHONY: all configure build test firmware-arduino upload-arduino clean

all: build

configure:
	$(CMAKE) -S . -B $(BUILD_DIR) \
		-DBUILD_COORDINATED_EXAMPLE=$(BUILD_COORDINATED_EXAMPLE) \
		-DARDUINO_CLI_EXECUTABLE="$(ARDUINO_CLI_EXECUTABLE)" \
		-DARDUINO_FQBN="$(ARDUINO_FQBN)" \
		-DARDUINO_UPLOAD_PORT="$(ARDUINO_UPLOAD_PORT)"

build: configure
	$(CMAKE) --build $(BUILD_DIR)
	$(CMAKE) --build $(BUILD_DIR) --target arduino_firmware

test: configure
	$(CMAKE) --build $(BUILD_DIR)
	$(CTEST) --test-dir $(BUILD_DIR) --output-on-failure

firmware-arduino: configure
	$(CMAKE) --build $(BUILD_DIR) --target arduino_firmware

upload-arduino: configure
	@test -n "$(ARDUINO_UPLOAD_PORT)" || { echo "Set ARDUINO_UPLOAD_PORT (example: /dev/ttyACM0)" >&2; exit 2; }
	$(CMAKE) --build $(BUILD_DIR) --target upload_arduino_firmware

clean:
	$(CMAKE) --build $(BUILD_DIR) --target clean
