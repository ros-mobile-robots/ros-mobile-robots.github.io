"""Tests for scripts/figures/firmware_unit_tests_figures.py, the wire bus tests' pictures.

Run from the repository root: python -m unittest discover -s tests
"""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Dict
import unittest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

import firmware_unit_tests_figures as figures  # noqa: E402

SCRIPT = ROOT / 'scripts' / 'figures' / 'firmware_unit_tests_figures.py'
INCLUDE_NAMES = ['firmware-unit-tests-read-registers.html',
                 'firmware-unit-tests-short-reads.html',
                 'firmware-unit-tests-write-register.html']
# With these variables, Python's default encoding for a text file is ASCII: the C locale, without
# Python's own switch to UTF-8 for that locale
ASCII_LOCALE = {'LC_ALL': 'C', 'LANG': 'C', 'PYTHONCOERCECLOCALE': '0', 'PYTHONUTF8': '0'}
# The names of the tests and cases in test_wire_bus.cpp, in the order of the file
WIRE_BUS_TESTS = ['WriteRegisterSendsRegisterAndValue',
                  'WriteRegisterFailsWhenTheAddressIsNotAcknowledged',
                  'WriteRegisterFailsWhenTheAddressIsNotAcknowledgedAfterASuccessfulWrite',
                  'WriteRegisterPassesOnTheAddressRegisterAndValue',
                  'ReadRegistersUsesARepeatedStart',
                  'ReadRegistersPassesOnTheAddressAndRegister',
                  'ReadRegistersReturnsAllBytesOfASample',
                  'ReadRegistersFailsWithoutAReadRequestWhenTheRegisterWriteFails']
SHORT_READ_CASES = ['ReportedShortAllBytesThere', 'BothShort', 'ReportedInFullOneByteMissing',
                    'ReportedOneShortAllBytesThere']


def run_with_an_ascii_locale() -> Dict[str, bytes]:
    """Run the script with an ASCII locale and return the includes it writes, by file name.

    The script runs in an empty folder, so the repository's includes stay as they are.
    """
    with tempfile.TemporaryDirectory() as folder:
        includes = Path(folder) / 'includes'
        includes.mkdir()
        run = subprocess.run([sys.executable, str(SCRIPT)], cwd=folder,
                             env={**os.environ, **ASCII_LOCALE}, capture_output=True, text=True)
        if run.returncode != 0:
            raise RuntimeError(f'{SCRIPT.name} failed: {run.stderr.strip()}')
        return {include.name: include.read_bytes() for include in includes.iterdir()}


class TestIncludes(unittest.TestCase):
    """The script writes the repository's includes, whatever the locale's encoding is.

    The includes have characters outside ASCII, such as the arrow before a result.
    """

    def test_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        written = run_with_an_ascii_locale()
        self.assertEqual(sorted(written), INCLUDE_NAMES)
        for name in INCLUDE_NAMES:
            committed = (ROOT / 'includes' / name).read_bytes()
            self.assertEqual(written[name], committed, f'{name} is not what {SCRIPT.name} writes')


class TestPanels(unittest.TestCase):
    """Every test and every case of the wire bus suite has one picture."""

    def test_pictures_name_each_test_of_the_suite_once_in_the_order_of_the_file(self) -> None:
        panels = figures.write_register_panels() + figures.read_register_panels()
        self.assertEqual([panel.name for panel in panels], WIRE_BUS_TESTS)

    def test_pictures_name_each_short_read_case_once_in_the_order_of_the_file(self) -> None:
        self.assertEqual([case.name for case in figures.SHORT_READS], SHORT_READ_CASES)


class TestCallTexts(unittest.TestCase):
    """The calls are written as FakeWire records them in the test file."""

    def test_hex_byte_writes_two_capital_hex_digits_after_0x(self) -> None:
        self.assertEqual(figures.hex_byte(0x0A), '0x0A')

    def test_end_transmission_call_writes_a_repeated_start_as_false(self) -> None:
        self.assertEqual(figures.end_transmission_call(figures.WITH_REPEATED_START),
                         'endTransmission(false)')

    def test_request_from_call_writes_the_length_in_decimal(self) -> None:
        self.assertEqual(figures.request_from_call(figures.DEVICE_ADDRESS,
                                                   figures.SAMPLE_LENGTH_BYTES),
                         'requestFrom(0x28, 18)')

    def test_breakable_name_may_break_before_each_capital_but_the_first(self) -> None:
        self.assertEqual(figures.breakable_name('BothShort'), '<code>Both<wbr>Short</code>')


class TestShortReads(unittest.TestCase):
    """The short-read cases come back short, and their lines follow readRegisters()."""

    def test_check_short_reads_accepts_the_scripts_cases(self) -> None:
        figures.check_short_reads(figures.SHORT_READS)

    def test_check_short_reads_raises_for_a_case_that_reads_in_full(self) -> None:
        full = figures.ShortRead('Full', figures.SAMPLE_LENGTH_BYTES, figures.SAMPLE_LENGTH_BYTES)
        with self.assertRaises(ValueError) as raised:
            figures.check_short_reads([full])
        self.assertEqual(str(raised.exception), 'not a short read: Full')

    def test_delivered_bytes_count_up_from_the_first_byte_and_leave_the_rest_empty(
            self) -> None:
        count = figures.COUNT_AFTER_FAILED_BUS_ACQUISITION
        received = figures.delivered_bytes(count, figures.SAMPLE_LENGTH_BYTES)
        self.assertEqual(received[:count], (0x10, 0x11, 0x12, 0x13))
        self.assertEqual(set(received[count:]), {None})

    def test_short_read_lines_compare_available_only_when_the_full_count_is_reported(
            self) -> None:
        for case in figures.SHORT_READS:
            calls = [line.call for line in figures.short_read_lines(case)
                     if isinstance(line, figures.WireCall)]
            full_count = case.reported_count == figures.SAMPLE_LENGTH_BYTES
            self.assertEqual('available()' in calls, full_count, case.name)

    def test_short_read_lines_note_how_many_bytes_are_missing_behind_a_full_count(
            self) -> None:
        missing = 3
        case = figures.ShortRead('ReportedInFullThreeBytesMissing', figures.SAMPLE_LENGTH_BYTES,
                                 figures.SAMPLE_LENGTH_BYTES - missing)
        notes = [line.note for line in figures.short_read_lines(case)
                 if isinstance(line, figures.WireCall) and line.call == 'available()']
        self.assertEqual(notes, ['3 bytes short'])

    def test_shortfall_note_says_one_byte_for_a_single_missing_byte(self) -> None:
        self.assertEqual(figures.shortfall_note(1), 'one byte short')

    def test_short_read_lines_read_every_byte_that_arrived(self) -> None:
        for case in figures.SHORT_READS:
            calls = [line.call for line in figures.short_read_lines(case)
                     if isinstance(line, figures.WireCall)]
            self.assertEqual(calls[-1], f'read() × {case.delivered_count}', case.name)


if __name__ == '__main__':
    unittest.main()
