#
# See this doc for options.
#   https://github.com/arduino/Arduino/blob/master/build/shared/manpage.adoc
#
# You will need to install follwing libraries under ~/Arduino/libraries to build.
# - PID_v1
# - Adafruit-MAX31855-library
#
#arduino --verify --pref build.path=./build-cli --board arduino:avr:diecimila -v src/HeadlessReflowOven.ino
arduino --upload --pref build.path=./build-cli --board arduino:avr:diecimila -v --port /dev/ttyUSB0 src/HeadlessReflowOven.ino
