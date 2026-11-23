# Firmware Unit Tests

A unit test runs one small part of the code with chosen inputs and checks the result. The unit tests of the Teensy firmware run on your PC, without a Teensy. The tested code talks to stand-ins for the hardware and for the ROS connection, which the tests call fakes. [PlatformIO](https://platformio.org/), the tool that builds the firmware, also builds the tests into programs for the PC and runs them ([PlatformIO's unit testing](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html)).

This page runs one test first. Then it explains which of the two test frameworks a new test uses, and how to write a test suite. CI runs every suite on every pull request ([Firmware unit tests](ci.md#firmware-unit-tests) on the Testing and CI page).

## Run the tests

### One test

The goal of this first run is to check that a register write sends the register byte followed by the value byte. The test [`WireBusTest.WriteRegisterSendsRegisterAndValue`](#tests-that-share-a-setup-a-fixture) checks exactly that, for the firmware's I2C bus code. Run it in the [firmware container](firmware-container.md), which needs only Docker, from the root folder of your diffbot clone:

```console
.docker/firmware/run.sh pio test \
  -e native_googletest \
  --filter googletest/test_wire_bus \
  --program-arg=--gtest_filter=WireBusTest.WriteRegisterSendsRegisterAndValue
```

The test passed when PlatformIO prints its name with `[PASSED]`, and the last line counts one test case that succeeded:

```console
WireBusTest.WriteRegisterSendsRegisterAndValue	[PASSED]
…
================== 1 test cases: 1 succeeded in 00:00:00.496 ==================
```

The first run of an environment downloads what it needs from PlatformIO's registry, so it needs the internet. For [`native_googletest`](#how-platformio-keeps-the-suites-apart) that is [GoogleTest](#which-framework-for-a-new-test), for `native` Unity and [rosserial](../processing_units/teensy-mcu.md#other-boards-such-as-an-arduino). The libraries land in `diffbot_firmware/.pio/libdeps/<environment>`. Later runs of the same environment work without the internet. To be able to run every suite offline, run the command for every suite below once while you are online.

The command has four parts:

- **[`.docker/firmware/run.sh`](firmware-container.md#the-script):** our script that runs a command in the firmware container, in the folder [`diffbot_firmware`](firmware-container.md#the-container).
- **`pio test -e native_googletest`:** PlatformIO's test command, with the environment that builds the GoogleTest suites. An environment is a section of `platformio.ini` that says what to build and how.
- **`--filter googletest/test_wire_bus`:** the PlatformIO test suite, a folder below [`diffbot_firmware/test`]({{ firmware_tests_src_url }}/diffbot_firmware/test) whose name starts with `test_`. PlatformIO builds each of these folders into one test program. `--filter` selects folders, not single tests.
- **`--program-arg=--gtest_filter=…`:** an option that PlatformIO passes on to the test program ([`pio test`'s options](https://docs.platformio.org/en/latest/core/userguide/cmd_test.html)). GoogleTest's option `--gtest_filter` runs only the tests whose full name matches ([Running a Subset of the Tests](https://google.github.io/googletest/advanced.html#running-a-subset-of-the-tests)).

A test's full name has two parts. GoogleTest groups the tests of a program into GoogleTest test suites, and the name before the dot is that group, here `WireBusTest`. The name after the dot is the test. The program of [`googletest/test_wire_bus`](#write-a-googletest-suite) has twelve tests in two GoogleTest test suites, `WireBusTest` and [`ShortReads/WireBusShortReadTest`](#one-behavior-with-several-inputs-a-parameterized-test). PlatformIO counts each test as a test case.

### All suites, one environment or one suite

Each of these commands runs more than one test. Run them from the same folder.

Run every suite, as CI does:

```console
.docker/firmware/run.sh pio test \
  -e native -e native_googletest
```

Run only the [Unity](#which-framework-for-a-new-test) suites:

```console
.docker/firmware/run.sh pio test \
  -e native
```

Run only the GoogleTest suites:

```console
.docker/firmware/run.sh pio test \
  -e native_googletest
```

Run one Unity suite, here the PID controller's:

```console
.docker/firmware/run.sh pio test \
  -e native --filter test_pid
```

Run one GoogleTest suite, here the I2C bus code's:

```console
.docker/firmware/run.sh pio test \
  -e native_googletest \
  --filter googletest/test_wire_bus
```

PlatformIO prints each test's name with `[PASSED]` or `[FAILED]`. At the end, a summary lists each PlatformIO test suite with its status, and the last line counts the test cases: 101 for every suite.

??? example "The output of every suite: the GoogleTest suite's tests and the summary"

    ```console
    Processing googletest/test_wire_bus in native_googletest environment
    --------------------------------------------------------------------------------
    Building...
    Testing...
    WireBusTest.WriteRegisterSendsRegisterAndValue	[PASSED]
    WireBusTest.WriteRegisterFailsWhenTheAddressIsNotAcknowledged	[PASSED]
    WireBusTest.WriteRegisterFailsWhenTheAddressIsNotAcknowledgedAfterASuccessfulWrite	[PASSED]
    WireBusTest.WriteRegisterPassesOnTheAddressRegisterAndValue	[PASSED]
    WireBusTest.ReadRegistersUsesARepeatedStart	[PASSED]
    WireBusTest.ReadRegistersPassesOnTheAddressAndRegister	[PASSED]
    WireBusTest.ReadRegistersReturnsAllBytesOfASample	[PASSED]
    WireBusTest.ReadRegistersFailsWithoutAReadRequestWhenTheRegisterWriteFails	[PASSED]
    ShortReads/WireBusShortReadTest.ReadRegistersFailsAndDrainsTheBytes/ReportedShortAllBytesThere	[PASSED]
    ShortReads/WireBusShortReadTest.ReadRegistersFailsAndDrainsTheBytes/BothShort	[PASSED]
    ShortReads/WireBusShortReadTest.ReadRegistersFailsAndDrainsTheBytes/ReportedInFullOneByteMissing	[PASSED]
    ShortReads/WireBusShortReadTest.ReadRegistersFailsAndDrainsTheBytes/ReportedOneShortAllBytesThere	[PASSED]
    ---- native_googletest:googletest/test_wire_bus [PASSED] Took 1.98 seconds ----

    =================================== SUMMARY ===================================
    Environment        Test                               Status    Duration
    -----------------  ---------------------------------  --------  ------------
    native             test_wheel_control                 PASSED    00:00:00.476
    native             test_synced_node_handle            PASSED    00:00:00.482
    native             test_ros_wheel_command_subscriber  PASSED    00:00:00.511
    native             test_pid                           PASSED    00:00:00.448
    native             test_imu_publisher                 PASSED    00:00:00.520
    native             test_command_timeout               PASSED    00:00:00.450
    native             test_bno055                        PASSED    00:00:00.478
    native_googletest  googletest/test_wire_bus           PASSED    00:00:01.985
    ================ 101 test cases: 101 succeeded in 00:00:05.349 ================
    ```

With PlatformIO installed on your PC instead, in the virtual environment that [Firmware Container](firmware-container.md) sets up, call that environment's `pio` in the folder [`diffbot_firmware`]({{ firmware_tests_src_url }}/diffbot_firmware): `~/.venvs/platformio/bin/pio test -e native -e native_googletest`. That route needs a C++17 compiler on the PC. It wasn't tested with the GoogleTest environment.

### What a failed test looks like

To see what a failure looks like, change the tested code on purpose. [`readRegisters()`](#one-behavior-with-several-inputs-a-parameterized-test) in [`wire_bus.h`]({{ firmware_tests_src_url }}/diffbot_firmware/lib/imu/wire_bus.h#L40) compares two counts with the requested [`length`]({{ firmware_tests_src_url }}/diffbot_firmware/lib/imu/wire_bus.h#L31): the count that `requestFrom()` reports, and the bytes that arrived, `wire_.available()`.

??? example "`readRegisters()`, lines 31–49 of `wire_bus.h`"

    `readRegisters()` is [lines 31–49 of `wire_bus.h`]({{ firmware_tests_src_url }}/diffbot_firmware/lib/imu/wire_bus.h#L31-L49). Line 40 compares both counts with `length`, and the loop after it reads the bytes that arrived when a count is short:

    ```cpp title="wire_bus.h: readRegisters()" linenums="31"
    --8<-- "{{ firmware_tests_src_raw }}/diffbot_firmware/lib/imu/wire_bus.h:31:49"
    ```

This edit replaces both comparisons with one comparison of the two counts with each other:

```diff
-if (wire_.requestFrom(address, (uint8_t)length) != length || wire_.available() != (int)length)
+const int reported = wire_.requestFrom(address, (uint8_t)length);
+if (reported != wire_.available())
```

With that edit, run the wire bus suite from the root folder of your diffbot clone:

```console
.docker/firmware/run.sh pio test \
  -e native_googletest \
  --filter googletest/test_wire_bus
```

The suite fails in one case, `BothShort`:

!!! failure "`BothShort` failed"

    `readRegisters()` returned `true`. The test expects `false`.

    The read requested 18 bytes. `requestFrom()` reported 4, and 4 bytes arrived. The changed check finds 4 equal to 4.

PlatformIO marks the case with `[FAILED]` and names the line of the failed check, [`test_wire_bus.cpp:254`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L254). The other eleven tests pass. The summary counts 13 test cases: the twelve tests, and one more with the status `ERRORED` for the program's exit status.

??? example "PlatformIO's output of the failed run"

    ```console
    WireBusTest.WriteRegisterSendsRegisterAndValue	[PASSED]
    WireBusTest.WriteRegisterFailsWhenTheAddressIsNotAcknowledged	[PASSED]
    WireBusTest.WriteRegisterFailsWhenTheAddressIsNotAcknowledgedAfterASuccessfulWrite	[PASSED]
    WireBusTest.WriteRegisterPassesOnTheAddressRegisterAndValue	[PASSED]
    WireBusTest.ReadRegistersUsesARepeatedStart	[PASSED]
    WireBusTest.ReadRegistersPassesOnTheAddressAndRegister	[PASSED]
    WireBusTest.ReadRegistersReturnsAllBytesOfASample	[PASSED]
    WireBusTest.ReadRegistersFailsWithoutAReadRequestWhenTheRegisterWriteFails	[PASSED]
    ShortReads/WireBusShortReadTest.ReadRegistersFailsAndDrainsTheBytes/ReportedShortAllBytesThere	[PASSED]
    test/googletest/test_wire_bus/test_wire_bus.cpp:254: ShortReads/WireBusShortReadTest.ReadRegistersFailsAndDrainsTheBytes/BothShort: Failure	[FAILED]
    ShortReads/WireBusShortReadTest.ReadRegistersFailsAndDrainsTheBytes/ReportedInFullOneByteMissing	[PASSED]
    ShortReads/WireBusShortReadTest.ReadRegistersFailsAndDrainsTheBytes/ReportedOneShortAllBytesThere	[PASSED]
    Program received signal SIGHUP (Hangup)
    ---- native_googletest:googletest/test_wire_bus [ERRORED] Took 2.02 seconds ----

    =================================== SUMMARY ===================================
    Environment        Test                      Status    Duration
    -----------------  ------------------------  --------  ------------
    native_googletest  googletest/test_wire_bus  ERRORED   00:00:02.015

    ============ 13 test cases: 1 failed, 11 succeeded in 00:00:02.015 ============
    ```

Add `-v` at the end of the same command, and PlatformIO also prints [GoogleTest](#which-framework-for-a-new-test)'s own output, which says why the check failed:

??? example "GoogleTest's output for the case `BothShort`, with `-v`"

    ```console
    [ RUN      ] ShortReads/WireBusShortReadTest.ReadRegistersFailsAndDrainsTheBytes/BothShort
    test/googletest/test_wire_bus/test_wire_bus.cpp:254: Failure
    Value of: bus_.readRegisters(DEVICE_ADDRESS, READ_REGISTER, data.data(), data.size())
      Actual: true
    Expected: false

    [  FAILED  ] ShortReads/WireBusShortReadTest.ReadRegistersFailsAndDrainsTheBytes/BothShort, where GetParam() = requestFrom() reports 4 bytes, 4 bytes arrive (0 ms)
    ```

GoogleTest's output has two parts to read:

- **The check that failed:** its file and line, the expression, its value and the expected value.
- **The test's name:** the name ends with the case, `BothShort`, and the case's values follow `where GetParam() =`. The other three cases ran too, and passed.

After the tests of a suite with a failure, PlatformIO 6.2.0 prints `Program received signal SIGHUP (Hangup)` and gives the suite the status `ERRORED`. In this run, no signal was sent. The test program ends with the exit status 1 when a test fails, and PlatformIO reads a nonzero status as a signal's number. Signal 1 is SIGHUP. A program that really received the signal SIGHUP gets the same message. So the message doesn't tell you what went wrong: look for a failed check above it. A Unity suite with one failed test shows the same line.

??? example "Why exit status 1 reads as SIGHUP: `raise_for_status()` in PlatformIO 6.2.0"

    PlatformIO calls [`raise_for_status()`](https://github.com/platformio/platformio-core/blob/v6.2.0/platformio/test/runners/readers/native.py#L113-L114) with the test program's exit status when that status isn't 0. The function turns [`abs(return_code)`](https://github.com/platformio/platformio-core/blob/v6.2.0/platformio/test/runners/readers/native.py#L119) into a signal, so the exit status 1 and a program killed by signal 1, which Python reports as -1, both become SIGHUP. The function is [lines 116–128 of `native.py`](https://github.com/platformio/platformio-core/blob/v6.2.0/platformio/test/runners/readers/native.py#L116-L128):

    ```python title="native.py: raise_for_status()" linenums="116"
    --8<-- "https://raw.githubusercontent.com/platformio/platformio-core/v6.2.0/platformio/test/runners/readers/native.py:116:128"
    ```

Undo the edit before you go on.

## Which framework for a new test

The firmware's suites are written with two test frameworks, [Unity](https://github.com/ThrowTheSwitch/Unity/tree/v2.6.1) and [GoogleTest](https://github.com/google/googletest/tree/v1.17.0). A test framework is the library that provides the checks and reports which tests passed. Which framework a test uses depends on the code that it tests:

- **A new test of portable code:** GoogleTest. The suite's folder is `test/googletest/test_<name>/`, and the environment [`native_googletest`](#how-platformio-keeps-the-suites-apart) runs it. Portable code is code that builds on the PC. That is the firmware's core, the logic that includes no Arduino, [rosserial](../processing_units/teensy-mcu.md#other-boards-such-as-an-arduino) or other vendor library header ([Ports and adapters]({{ diffbot_repo_url }}/diffbot_firmware/README.md#ports-and-adapters) in the firmware's README). An adapter connects the core to a library or to the hardware, and most adapters need the Arduino core. An adapter that takes its library as a template argument, as [`WireBusT`]({{ firmware_tests_src_url }}/diffbot_firmware/lib/imu/wire_bus.h#L17-L18) does below, is portable code too.
- **A test of the rosserial code:** Unity. Its three suites use rosserial's node handle with a fake serial port, and only the environment `native` has the rosserial library. Their folders are `test/test_<name>/`, and `native` runs them. The suites of the BNO055 driver, the command timeout, the PID controller and the wheel control are Unity suites too. A Unity suite of the core moves to GoogleTest when its code changes.
- **A test that has to run on the Teensy itself:** Unity. PlatformIO runs Unity on microcontrollers, and GoogleTest only on the PC and ESP boards ([GoogleTest in PlatformIO](https://docs.platformio.org/en/latest/advanced/unit-testing/frameworks/googletest.html)). The firmware has no such suite yet. Its checks on the Teensy are sketches that you run by hand, such as [`test/encoders`]({{ firmware_tests_src_url }}/diffbot_firmware/test/encoders), and the commands on this page leave them out.

GoogleTest is a C++ framework. Tests that share a setup put it into a fixture, and a test that has to run with several values is written once and runs once per value. A failure names the value. GoogleTest also finds the tests by itself, while a Unity suite lists each test by hand. [Unity](https://github.com/ThrowTheSwitch/Unity) is a small framework written in C for microcontrollers ([Unity in PlatformIO](https://docs.platformio.org/en/latest/advanced/unit-testing/frameworks/unity.html)). Its checks are C macros, and the firmware's Unity suites are C++ files too. ROS uses GoogleTest for its C++ tests, in ROS 1 ([gtest](http://wiki.ros.org/gtest) on the ROS wiki) and in ROS 2 ([Writing Basic Tests with C++ with GTest](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Cpp.html)).

[`test/googletest`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest) holds one suite, [`test_wire_bus`](#write-a-googletest-suite). It tests `WireBusT` ([`wire_bus.h`]({{ firmware_tests_src_url }}/diffbot_firmware/lib/imu/wire_bus.h)), the adapter that carries out the BNO055 driver's I2C transfers with Arduino's Wire library ([How the firmware reads it](../packages/imu/driver-bno055.md#how-the-firmware-reads-it) on the IMU page). `WireBusT` takes the Wire library's class as a template argument. On the Teensy that is Arduino's [`TwoWire`](https://github.com/PaulStoffregen/Wire/blob/eec0c444a0c8daadcd6f9da8df085b574b9ad916/WireIMXRT.h#L41). The suite passes its fake, [`FakeWire`](#tests-that-share-a-setup-a-fixture), so the adapter builds and runs on the PC too.

### How PlatformIO keeps the suites apart

One environment builds its suites with one framework. So the firmware has an environment for each framework, and each suite's folder decides which environment runs it:

```mermaid
graph TB
  subgraph ENVIRONMENTS [platformio.ini: environments]
    NATIVE[native:<br/>Unity]
    GOOGLETEST[native_googletest:<br/>GoogleTest]
  end
  subgraph SUITES [suites]
    UNITY_SUITES[test/test_bno055,<br/>test/test_pid, ...]
    GOOGLETEST_SUITES[test/googletest/<br/>test_wire_bus]
  end
  NATIVE -->|builds and runs| UNITY_SUITES
  GOOGLETEST -->|builds and runs| GOOGLETEST_SUITES
```

PlatformIO takes every folder below [`test`]({{ firmware_tests_src_url }}/diffbot_firmware/test) whose name starts with `test_` as a PlatformIO test suite, and names the suite by its path: [`test_pid`]({{ firmware_tests_src_url }}/diffbot_firmware/test/test_pid), or [`googletest/test_wire_bus`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus) ([Test Hierarchy](https://docs.platformio.org/en/latest/advanced/unit-testing/structure/hierarchy.html)). The settings [`test_filter`](https://github.com/platformio/platformio-docs/blob/3f4cd06920b15ad5d231de8d600e37de9696d50b/projectconf/sections/env/options/test/test_filter.rst) and [`test_ignore`](https://github.com/platformio/platformio-docs/blob/3f4cd06920b15ad5d231de8d600e37de9696d50b/projectconf/sections/env/options/test/test_ignore.rst) of [`platformio.ini`]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini) compare patterns with that path:

- **The section [`[env]`]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini#L21-L22):** sets [`test_ignore = googletest/*`]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini#L22). Its options hold for every environment that doesn't set them itself ([Common `[env]`](https://docs.platformio.org/en/latest/projectconf/sections/env/index.html#common-env)). So the two Teensy environments and [`native`]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini#L53-L68) ignore every suite below [`test/googletest`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest).
- **`native`:** [`test_framework = unity`]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini#L55), and `test_filter` lists its [Unity](#which-framework-for-a-new-test) suites by name.
- **[`native_googletest`]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini#L74-L88):** [`test_framework = googletest`]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini#L76), [`test_filter = googletest/*`]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini#L77), and its own [`test_ignore = test_*`]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini#L80), which matches the Unity suites. A new GoogleTest suite needs no change of the file.

A `--filter` on the command line replaces the environment's `test_filter`, but not its `test_ignore`. So a call that names a suite for the wrong environment builds nothing with the wrong framework: `pio test -e native --filter "googletest/*"` runs no suite. PlatformIO then reports `0 test cases` and no error. CI also checks the result: its unit test job fails when no test case of one of the two environments passed.

The two excerpts of `platformio.ini` show these settings. The first one is the shared setting, [lines 14–22]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini#L14-L22):

```ini title="platformio.ini: the shared setting" linenums="14"
--8<-- "{{ firmware_tests_src_raw }}/diffbot_firmware/platformio.ini:14:22"
```

??? example "The two environments for the PC, lines 49–88 of `platformio.ini`"

    The second excerpt holds the two environments for the PC, [lines 49–88]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini#L49-L88):

    ```ini title="platformio.ini: the two environments for the PC" linenums="49"
    --8<-- "{{ firmware_tests_src_raw }}/diffbot_firmware/platformio.ini:49:88"
    ```

    - **[`platform = native`](https://github.com/platformio/platformio-docs/blob/3f4cd06920b15ad5d231de8d600e37de9696d50b/projectconf/sections/env/options/platform/platform.rst):** both environments build programs for the PC ([PlatformIO's native platform](https://docs.platformio.org/en/latest/platforms/native.html)).
    - **[`test_framework`](https://github.com/platformio/platformio-docs/blob/3f4cd06920b15ad5d231de8d600e37de9696d50b/projectconf/sections/env/options/test/test_framework.rst):** `unity` or `googletest`. PlatformIO downloads the framework's library and reads the test program's output in that framework's format.
    - **[`lib_deps`](https://github.com/platformio/platformio-docs/blob/3f4cd06920b15ad5d231de8d600e37de9696d50b/projectconf/sections/env/options/library/lib_deps.rst):** `native` gets the [rosserial](../processing_units/teensy-mcu.md#other-boards-such-as-an-arduino) library, which three of its suites use. `native_googletest` gets GoogleTest in one exact version, 1.17.0, so that every PC and CI build the tests with the same GoogleTest. The environment has no rosserial library: a GoogleTest suite that includes a rosserial header doesn't build.
    - **[`build_flags`](https://github.com/platformio/platformio-docs/blob/3f4cd06920b15ad5d231de8d600e37de9696d50b/projectconf/sections/env/options/build/build_flags.rst):** [`${env:native.build_flags}`]({{ firmware_tests_src_url }}/diffbot_firmware/platformio.ini#L88) takes the value of `native`, so both environments compile as C++17 with the same warnings ([interpolation of values](https://docs.platformio.org/en/latest/projectconf/interpolation.html)).

## Write a GoogleTest suite

A new suite starts as one file with one test. A fixture comes in when several tests share a setup, and a parameterized test when one behavior needs several inputs. The wire bus suite, [`test_wire_bus.cpp`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp), shows both.

### A first suite: one file, one test

This first suite checks that a [`CommandTimeout`]({{ firmware_tests_src_url }}/diffbot_firmware/lib/command_timeout/command_timeout.h#L24-L52) ([`command_timeout.h`]({{ firmware_tests_src_url }}/diffbot_firmware/lib/command_timeout/command_timeout.h)) [starts timed out]({{ firmware_tests_src_url }}/diffbot_firmware/lib/command_timeout/command_timeout.h#L51), so that the wheels don't turn before the first command. Create the folder `diffbot_firmware/test/googletest/test_first_suite/`, and in it the file `test_first_suite.cpp` with this content:

```cpp title="test/googletest/test_first_suite/test_first_suite.cpp"
// A first GoogleTest suite: one test of CommandTimeout (lib/command_timeout/command_timeout.h)
#include <gtest/gtest.h>

#include <stdint.h>

#include "command_timeout.h"

// The timeout of the tested CommandTimeout, in milliseconds
constexpr uint32_t TEST_TIMEOUT_MS = 1000;

TEST(CommandTimeoutTest, IsTimedOutWhenConstructed)
{
    const diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);

    EXPECT_TRUE(timeout.timedOut());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

Run the suite from the root folder of your diffbot clone:

```console
.docker/firmware/run.sh pio test \
  -e native_googletest \
  --filter googletest/test_first_suite
```

The test passed when PlatformIO prints `CommandTimeoutTest.IsTimedOutWhenConstructed` with `[PASSED]`, and the last line counts `1 test cases: 1 succeeded`. The environment [`native_googletest`](#how-platformio-keeps-the-suites-apart) found the new folder by itself, without a change of `platformio.ini`.

Then check the new file's includes, also from the root folder of your diffbot clone. The include check [`tools/check_includes.sh`]({{ firmware_tests_src_url }}/diffbot_firmware/tools/check_includes.sh) gives clang-tidy the flags that `pio test` compiles the suite with, and reports each name whose header the file doesn't include itself ([Check the includes]({{ firmware_tests_src_url }}/AGENTS.md#check-the-includes) in the diffbot repository's `AGENTS.md`):

```console
.docker/firmware/run.sh tools/check_includes.sh native_googletest \
  test/googletest/test_first_suite/test_first_suite.cpp
```

The command ends with the exit status 0 also when clang-tidy prints a warning, so read its output: no `misc-include-cleaner` warning may name the new file. This first suite has none. It includes `<gtest/gtest.h>` for `TEST` and `EXPECT_TRUE`, `<stdint.h>` for `uint32_t`, and `command_timeout.h` for `CommandTimeout`.

The file has four parts:

- **The includes:** [`<gtest/gtest.h>`](https://github.com/google/googletest/blob/v1.17.0/googletest/include/gtest/gtest.h) for [GoogleTest](#which-framework-for-a-new-test), and the header of the tested code. PlatformIO finds the headers in the folders of [`diffbot_firmware/lib`]({{ firmware_tests_src_url }}/diffbot_firmware/lib). A suite that uses a fake from [`test/fakes`]({{ firmware_tests_src_url }}/diffbot_firmware/test/fakes) includes it as `"../../fakes/<file>"`.
- **[`TEST(CommandTimeoutTest, IsTimedOutWhenConstructed)`](https://github.com/google/googletest/blob/v1.17.0/docs/reference/testing.md#test-test):** defines a test. The first argument is the GoogleTest test suite, the second the test's name. The name says the subject, the result and the condition, the project's rule for test names ([AGENTS.md]({{ firmware_tests_src_url }}/AGENTS.md#tests) of the diffbot repository). GoogleTest's names have no underscores ([GoogleTest's FAQ](https://google.github.io/googletest/faq.html#why-should-test-suite-names-and-test-names-not-contain-underscore)), so each word starts with a capital letter.
- **[`EXPECT_TRUE(timeout.timedOut())`](https://github.com/google/googletest/blob/v1.17.0/docs/reference/assertions.md#expect_true-expect_true):** a check that [`timedOut()`]({{ firmware_tests_src_url }}/diffbot_firmware/lib/command_timeout/command_timeout.h#L45-L46) returns true. A failed [`EXPECT_`](https://github.com/google/googletest/blob/v1.17.0/docs/reference/assertions.md#assertions-reference) check marks the test as failed and lets the test go on, so one run shows every failed check. A failed `ASSERT_` check returns from the function that it is in. In a test's body, that ends the test, for a check whose failure makes the rest pointless ([Assertions](https://google.github.io/googletest/primer.html#assertions) in GoogleTest's primer).
- **[`main()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L276-L283):** PlatformIO's GoogleTest library comes without a `main()` function, so each suite has its own ([GoogleTest in PlatformIO](https://docs.platformio.org/en/latest/advanced/unit-testing/frameworks/googletest.html)). [`InitGoogleTest()`](https://github.com/google/googletest/blob/v1.17.0/docs/reference/testing.md#initgoogletest-initgoogletest) comes first: it reads GoogleTest's options from the command line, such as `--gtest_filter` ([Writing the main() Function](https://google.github.io/googletest/primer.html#writing-the-main-function) in the primer). Then [`RUN_ALL_TESTS()`](https://github.com/google/googletest/blob/v1.17.0/docs/reference/testing.md#run_all_tests-run_all_tests) runs the tests registered in the program that GoogleTest's filter selects: all of them, unless `--gtest_filter` names some, as in the first command on this page. A Unity suite runs a test only if `main()` names it in a [`RUN_TEST`]({{ firmware_tests_src_url }}/diffbot_firmware/test/test_command_timeout/test_command_timeout.cpp#L118-L131) line.

`main()` returns `RUN_ALL_TESTS()`'s result as the program's exit status, which is 1 when anything failed. The example on PlatformIO's GoogleTest page returns 0 and leaves the result to the output that PlatformIO reads. In our check with PlatformIO 6.2.0, a failure outside a test's body, in a fixture's [`SetUpTestSuite()`](https://github.com/google/googletest/blob/v1.17.0/docs/reference/testing.md#setuptestsuite-testsetuptestsuite), then ended with `pio test` reporting success. With the exit status 1, the run fails.

This first suite tests code that a Unity suite, [`test_command_timeout`]({{ firmware_tests_src_url }}/diffbot_firmware/test/test_command_timeout), tests already. Delete its folder when you are done. A real suite takes the name of the code it tests: `test/googletest/test_<name>/test_<name>.cpp`.

### Tests that share a setup: a fixture

The wire bus suite has eight tests of [`WireBusT`](#which-framework-for-a-new-test) that all need a fake Wire library and a `WireBusT` on it. A fixture is a class with the objects that every test of a group needs. The fixture [`WireBusTest`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L120-L127) holds a [`FakeWire`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L77-L118) and a `WireBusT` on it, lines 121–128:

```cpp title="test_wire_bus.cpp: the fixture" linenums="120"
--8<-- "{{ firmware_tests_src_raw }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp:120:127"
```

GoogleTest creates a new `WireBusTest` object for each test ([Test Fixtures](https://google.github.io/googletest/primer.html#same-data-multiple-tests) in the primer). So each test starts with an empty [`wire_`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L125), and the order of the tests doesn't matter.

`FakeWire` stands in for Arduino's Wire library. It records each call of [`beginTransmission()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L89), [`write()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L90-L94), [`endTransmission()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L95-L99) and [`requestFrom()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L100-L108) as text, such as `beginTransmission(0x28)` or `write(0x3D)`, in the list [`calls`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L83), in the order of the calls. [`available()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L109) and [`read()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L110-L117) only hand out the received bytes, and `calls` doesn't list them. `FakeWire` also returns the results that a test chooses, for example [`end_result`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L85) as the result of `endTransmission()`. The functions [`beginTransmissionCall()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L61-L66), [`writeCall()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L67), [`endTransmissionCall()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L68-L71) and [`requestFromCall()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L72-L75) write a call as text. `FakeWire` records its calls with them, and the tests write the calls that they expect with the same functions.

??? example "`FakeWire` and the functions that write a call as text, lines 53–118"

    The functions and the fake are [lines 53–118]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L53-L118):

    ```cpp title="test_wire_bus.cpp: FakeWire" linenums="53"
    --8<-- "{{ firmware_tests_src_raw }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp:53:118"
    ```

[`TEST_F`](https://github.com/google/googletest/blob/v1.17.0/docs/reference/testing.md#test_f-test_f) defines a test that uses a fixture. Its first argument is the fixture, its second the test's name. The four tests of [`writeRegister()`]({{ firmware_tests_src_url }}/diffbot_firmware/lib/imu/wire_bus.h#L23-L29) are [lines 129–165]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L129-L165):

```cpp title="test_wire_bus.cpp: the tests of writeRegister()" linenums="129"
--8<-- "{{ firmware_tests_src_raw }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp:129:165"
```

- **The members:** `wire_` and [`bus_`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L126) are the fixture's. The first test calls [`bus_.writeRegister()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L131) and then compares [`wire_.calls`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L136) with [`expected_calls`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L133-L135): the device's address, the register, the value, and the end of the transfer with a stop.
- **The values:** they have names, which the top of the file defines, such as [`DEVICE_ADDRESS`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L19-L22), [`WRITTEN_REGISTER`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L23), [`WRITTEN_VALUE`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L24), [`ADDRESS_NOT_ACKNOWLEDGED`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L36-L38) and [`WITH_STOP`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L40-L42).
- **A scenario in phases:** the third test checks two calls on the same `bus_`, in numbered phases. A write that the device acknowledges comes first, and the next write, which no device acknowledges, has to fail. Its first check is an [`ASSERT_`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L149-L150) check, because a failed first write would leave nothing to check in phase 2.
- **A second set of values:** the fourth test writes [`RESET_VALUE`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L33) to [`RESET_REGISTER`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L32) at [`ALTERNATE_DEVICE_ADDRESS`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L27-L31), 0x29, the BNO055's address when its ADR pin is high. Each of these values differs from those of the first three tests, so the test shows that `WireBusT` passes on the values that it gets.

The pictures show each of the four tests as a time line: the values that the test sets on `wire_`, its call of `bus_`, the calls that `WireBusT` makes on `FakeWire`, the result, and the checks. In the second test, and in the second write of the third test, `endTransmission()` returns 2, and that write fails:

--8<-- "includes/firmware-unit-tests-write-register.html"

The tests of [`readRegisters()`]({{ firmware_tests_src_url }}/diffbot_firmware/lib/imu/wire_bus.h#L31-L49) compare `wire_.calls` the same way. In a read, the register write ends with a repeated start, `endTransmission(false)`, and only then comes the read request, `requestFrom()`. After a register write that no device acknowledges, no read request may follow. A read of [`CHIP_ID_REGISTER`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L34) from `ALTERNATE_DEVICE_ADDRESS` checks that `WireBusT` passes on that register, and that address in both the register write and the read request.

The pictures show the four tests of `readRegisters()` as time lines. After `requestFrom()`, `readRegisters()` asks `available()` how many bytes arrived and reads them with `read()`, and `calls` lists neither. Three of the tests compare `wire_.calls`, and the test of a sample's bytes compares the bytes in `data` instead. In the last test, the calls end after the register write:

--8<-- "includes/firmware-unit-tests-read-registers.html"

??? example "The four tests of `readRegisters()`, lines 167–219"

    The tests are [lines 167–219 of `test_wire_bus.cpp`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L167-L219). The third, `ReadRegistersReturnsAllBytesOfASample`, is the successful read of the next section's table:

    ```cpp title="test_wire_bus.cpp: the tests of readRegisters()" linenums="167"
    --8<-- "{{ firmware_tests_src_raw }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp:167:219"
    ```

### One behavior with several inputs: a parameterized test

A read can come back short in several ways, and [`readRegisters()`]({{ firmware_tests_src_url }}/diffbot_firmware/lib/imu/wire_bus.h#L38-L45) has to fail for each of them. The BNO055 driver reads a sample's 18 data bytes in one transfer. `readRegisters()` asks Wire's [`requestFrom()`](https://github.com/PaulStoffregen/Wire/blob/eec0c444a0c8daadcd6f9da8df085b574b9ad916/WireIMXRT.cpp#L155-L163) for that requested length, and two counts come back:

- **Reported:** the count that `requestFrom()` returns.
- **Arrived:** the bytes that are really there, which [`available()`](https://github.com/PaulStoffregen/Wire/blob/eec0c444a0c8daadcd6f9da8df085b574b9ad916/WireIMXRT.h#L109-L111) tells.

`readRegisters()` returns `true` only when both counts are the requested length. Otherwise it returns `false`, and it reads the bytes that arrived out of the Wire library, so that none of them is left for the next read. The table shows the successful read and the four short reads of the suite, each with 18 bytes requested, and what `readRegisters()` returns:

| Read | Reported | Arrived | Returns |
|:--|--:|--:|:--|
| A sample's data | 18 | 18 | `true` |
| Both short | 4 | 4 | `false` |
| Reported short, all bytes there | 4 | 18 | `false` |
| Reported one short, all bytes there | 17 | 18 | `false` |
| Reported in full, one byte missing | 18 | 17 | `false` |

After each of the five reads, no byte may be left in the Wire library. The successful read copies its 18 bytes into [`data`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L197), and a short read drains the bytes that arrived. The test [`ReadRegistersReturnsAllBytesOfASample`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L195-L205) checks that read and the 18 bytes in `data`. The four short reads are the cases of a parameterized test. Their names in the code join the words of the first column, such as [`BothShort`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L270-L271).

The case `BothShort` is a read that the Teensy 4's Wire library can deliver when it can't get the bus. Its `requestFrom()` then returns 4, and it returns before it empties its receive buffer. So `available()` then counts what an earlier read left there. In `BothShort`, that is 4 bytes, and both counts agree with each other, but not with the 18 requested bytes. The start of `requestFrom()` in the Teensy 4's Wire library is [lines 155–163 of `WireIMXRT.cpp`](https://github.com/PaulStoffregen/Wire/blob/eec0c444a0c8daadcd6f9da8df085b574b9ad916/WireIMXRT.cpp#L155-L163):

```cpp title="WireIMXRT.cpp: the start of requestFrom()" linenums="155"
--8<-- "https://raw.githubusercontent.com/PaulStoffregen/Wire/eec0c444a0c8daadcd6f9da8df085b574b9ad916/WireIMXRT.cpp:155:163"
```

The four cases are examples of short reads, not every possible one. Two have a short reported count with all bytes there, 4 and 17 of 18. One has both counts short, and one has a byte missing behind a full count. They are four inputs to one behavior: `readRegisters()` fails and drains the bytes. A parameterized test is written once and runs once per case. The type [`ShortRead`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L225-L231) describes a case, and the fixture [`WireBusShortReadTest`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L240-L243) adds the case to [`WireBusTest`](#tests-that-share-a-setup-a-fixture), [lines 221–243]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L221-L243):

```cpp title="test_wire_bus.cpp: the type of a case and the fixture" linenums="221"
--8<-- "{{ firmware_tests_src_raw }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp:221:243"
```

[`TEST_P`](https://github.com/google/googletest/blob/v1.17.0/docs/reference/testing.md#test_p-test_p) defines the test, and [`INSTANTIATE_TEST_SUITE_P`](https://github.com/google/googletest/blob/v1.17.0/docs/reference/testing.md#instantiate_test_suite_p-instantiate_test_suite_p) lists the cases, [lines 245–274]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L245-L274):

```cpp title="test_wire_bus.cpp: the parameterized test and its cases" linenums="245"
--8<-- "{{ firmware_tests_src_raw }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp:245:274"
```

- **[`GetParam()`](https://github.com/google/googletest/blob/v1.17.0/docs/reference/testing.md#withparaminterface-withparaminterface):** returns the case of the current run. The test passes the case's two counts to [`wire_`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L250-L251) and calls `readRegisters()`. Then it checks the result, that the read request asked for a sample's data, [`SAMPLE_LENGTH_BYTES`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L50-L51), and that [`wire_.available()`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L262) is 0.
- **`::testing::Values(…)`:** holds the four cases. Their counts come from `SAMPLE_LENGTH_BYTES` and [`COUNT_AFTER_FAILED_BUS_ACQUISITION`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L221-L223).
- **The last argument:** turns a case into the end of the test's name, such as `BothShort` ([names for the cases in GoogleTest's advanced guide](https://github.com/google/googletest/blob/v1.17.0/docs/advanced.md#specifying-names-for-value-parameterized-test-parameters)).
- **[`operator<<`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L233-L238):** writes a case as text. GoogleTest prints that text after `where GetParam() =` when the case fails ([how GoogleTest prints values](https://google.github.io/googletest/advanced.html#teaching-googletest-how-to-print-your-values)).

The pictures show the four cases as time lines. Each case starts with the two counts that it sets on `wire_` and ends with the three checks. `readRegisters()` calls `available()` in its comparison only when `requestFrom()` reports the full 18 bytes: `||` stops at the first comparison that is true. The loop after it reads the bytes that arrived until `available()` returns 0:

--8<-- "includes/firmware-unit-tests-short-reads.html"

Written as a loop inside one [Unity](#which-framework-for-a-new-test) test, the cases would end at the first one that fails. Unity's message would name the line of the check, and not the case.

### See each test fail once

Break the behavior that one of your checks promises, on purpose, run the suite, read the failure, and undo the change. Then you know that the check can fail. For example, [`EXPECT_EQ(wire_.available(), 0)`]({{ firmware_tests_src_url }}/diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp#L262) promises that a short read leaves no byte behind. Without the loop that drains the bytes in [`readRegisters()`]({{ firmware_tests_src_url }}/diffbot_firmware/lib/imu/wire_bus.h#L42-L43), all four short-read cases fail at that check. A change that breaks a different behavior can pass a test that is focused on one behavior. Another test of the suite has to catch that change.

### Before the pull request

Run every suite, with the first command of [All suites, one environment or one suite](#all-suites-one-environment-or-one-suite), as CI does. Check the includes of each C++ file that you changed, as for the [first suite](#a-first-suite-one-file-one-test).
