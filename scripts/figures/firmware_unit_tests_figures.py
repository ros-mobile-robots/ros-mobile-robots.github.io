"""Generate the pictures of the wire bus tests on docs/development/firmware-unit-tests.md.

The tests are those of diffbot's diffbot_firmware/test/googletest/test_wire_bus/test_wire_bus.cpp,
at the commit that mkdocs.yml pins as firmware_tests_src_url. The script writes one include per
group of tests, with one picture per test:

- includes/firmware-unit-tests-write-register.html: the four tests of writeRegister().
- includes/firmware-unit-tests-read-registers.html: the four tests of readRegisters() with the
  fixture WireBusTest.
- includes/firmware-unit-tests-short-reads.html: the four cases of the parameterized test
  ReadRegistersFailsAndDrainsTheBytes.

Each picture is a time line of one test, as inline SVG that follows the site's light and dark
colors. Time runs down. The test calls bus_, the WireBusT, at the top. WireBusT's calls of
FakeWire follow, indented, and the result comes back at the bottom. The checks follow.

The constants below are the test file's, and the calls are written as FakeWire writes them. The
script stops if a short-read case isn't short. Change the values together with the test file and
the pin.

Run from the repository root after changing the figures:

    python3 scripts/figures/firmware_unit_tests_figures.py
"""

from enum import Enum
import html
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple
from typing import Union

from board_drawings import Drawing
from board_drawings import escape
from board_drawings import NUMBER_DISC_RADIUS
from board_drawings import style_element
from board_drawings import svg_element

# test_wire_bus.cpp's values, lines 22–51 and 223. The first set is the BNO055's address, its
# mode register with the value for the IMU mode, and its first data register
DEVICE_ADDRESS = 0x28
WRITTEN_REGISTER = 0x3D
WRITTEN_VALUE = 0x08
READ_REGISTER = 0x08
# The second set: the address when the breakout's ADR pin is high, the system trigger register
# with the value that resets the chip, and the chip ID register
ALTERNATE_DEVICE_ADDRESS = 0x29
RESET_REGISTER = 0x3F
RESET_VALUE = 0x20
CHIP_ID_REGISTER = 0x00
# endTransmission()'s results: 0 is success, FakeWire's default. 2 means that no device
# acknowledged the address
SUCCESS = 0
ADDRESS_NOT_ACKNOWLEDGED = 2
# endTransmission()'s argument: a stop, or a repeated start
WITH_STOP = True
WITH_REPEATED_START = False
# The first byte that FakeWire delivers to a read. Each further byte is one higher
FIRST_DELIVERED_BYTE = 0x10
READ_LENGTH_BYTES = 3
SAMPLE_LENGTH_BYTES = 18
COUNT_AFTER_FAILED_BUS_ACQUISITION = 4

WIDTH = 340
# The x of the test's own lines, of the time line, and of the columns of WireBusT's calls
LEFT_X = 8
TIME_X = 17
CALL_X = 30
RESULT_X = 188
NOTE_X = 220
RIGHT_X = 334
# The right edge of the box around the compared calls. calls holds a call's text, not its result,
# so the box ends before the column of the results
CALLS_BOX_RIGHT = 183
# The space between the test's call of bus_ and a box of compared calls, for the box's number
CALLS_BOX_GAP = 6
# The vertical distances between lines, in px
ROW_HEIGHT = 20
CHECK_HEIGHT = 19
# A byte of a row of bytes
CELL_WIDTH = 16
CELL_HEIGHT = 16

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
RULES = (
    ('.md-typeset figure.wbt-figure', 'margin:1em auto;width:100%;max-width:none'),
    ('.wbt-grid', 'display:flex;flex-wrap:wrap;justify-content:center;align-items:flex-start;'
                  'gap:1.2em 1em'),
    ('.md-typeset figure.wbt-picture', 'margin:0;width:16.6rem;max-width:100%'),
    ('.md-typeset figure.wbt-picture figcaption', 'margin:.3em 0 0;max-width:none'),
    ('.md-typeset figure.wbt-picture figcaption code', 'overflow-wrap:anywhere'),
    ('.wbt', 'width:100%;height:auto;--wbt-check:#1e63c4;--wbt-check-fill:rgba(30,99,196,.07);'
             '--wbt-set:#b45309;font-family:var(--md-text-font-family,Roboto),sans-serif'),
    ('[data-md-color-scheme=slate] .wbt',
     '--wbt-check:#82b1ff;--wbt-check-fill:rgba(130,177,255,.09);--wbt-set:#ffb74d'),
    ('.wbt text', f'fill:{FG};font-size:12px'),
    ('.wbt .mono', 'font-family:var(--md-code-font-family,"Roboto Mono"),monospace;'
                   'font-size:10.5px'),
    ('.wbt .byte', 'font-family:var(--md-code-font-family,"Roboto Mono"),monospace;'
                   'font-size:9.5px'),
    ('.wbt .note', f'font-size:11px;fill:{FG_LIGHT}'),
    ('.wbt .phase', 'font-weight:700'),
    ('.wbt .set', 'fill:var(--wbt-set);font-weight:700'),
    ('.wbt .light', f'fill:{FG_LIGHT}'),
    ('.wbt .not-called', f'fill:{FG_LIGHT};text-decoration:line-through'),
    ('.wbt .time-line', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3'),
    ('.wbt .arrow-head', f'fill:{FG_LIGHT}'),
    ('.wbt .recorded', f'fill:{FG}'),
    ('.wbt .not-recorded', f'fill:var(--md-default-bg-color);stroke:{FG};stroke-width:1.2'),
    ('.wbt .missing', f'fill:var(--md-default-bg-color);stroke:{FG_LIGHT};stroke-width:1.2;'
                      'stroke-dasharray:2 2'),
    ('.wbt .calls-box', 'fill:var(--wbt-check-fill);stroke:var(--wbt-check);stroke-width:1.4'),
    ('.wbt .cell', f'fill:var(--md-code-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.wbt .empty-cell', f'fill:none;stroke:{FG_LIGHT};stroke-width:1;stroke-dasharray:2 2'),
    ('.wbt .reported', 'fill:none;stroke:var(--wbt-set);stroke-width:1.6'),
    ('.wbt .badge', 'fill:var(--wbt-check)'),
    ('.wbt .badge-number', 'fill:var(--md-default-bg-color);font-size:11px;font-weight:700'),
)


def hex_byte(value: int) -> str:
    """Return a byte as FakeWire writes it, such as 0x28: hexByte() of the test file."""
    return f'0x{value:02X}'


def begin_transmission_call(address: int) -> str:
    """Return the call as FakeWire records it: beginTransmissionCall() of the test file."""
    return f'beginTransmission({hex_byte(address)})'


def write_call(value: int) -> str:
    """Return the call as FakeWire records it: writeCall() of the test file."""
    return f'write({hex_byte(value)})'


def end_transmission_call(stop: bool) -> str:
    """Return the call as FakeWire records it: endTransmissionCall() of the test file."""
    return f'endTransmission({"true" if stop else "false"})'


def request_from_call(address: int, length: int) -> str:
    """Return the call as FakeWire records it: requestFromCall() of the test file."""
    return f'requestFrom({hex_byte(address)}, {length})'


class Mark(Enum):
    """How a line of WireBusT's calls marks the call on the time line."""

    # A call that FakeWire records in calls: a filled dot
    RECORDED = 'recorded'
    # available() and read(), which calls doesn't list: an open dot
    NOT_RECORDED = 'not-recorded'
    # A call that the test expects not to happen: a dashed dot, the call struck through
    NOT_CALLED = 'missing'


class WireCall(NamedTuple):
    """A line of the time line: one call of FakeWire by WireBusT, and what FakeWire returns."""

    call: str
    note: str
    mark: Mark = Mark.RECORDED
    result: str = ''
    # Whether the test chose the result before the call, by a member of wire_
    result_set_by_test: bool = False


class ByteRow(NamedTuple):
    """A row of bytes, such as the Wire library's receive buffer after requestFrom().

    values holds one entry per byte that the read requested: the byte's value, or None for a byte
    that didn't arrive. reported is the count that requestFrom() reports when the test chose it,
    and 0 otherwise. badge is the number of the check that compares the row, or 0.
    """

    label: str
    values: Tuple[Optional[int], ...]
    reported: int = 0
    badge: int = 0


# A line inside the call of bus_
Line = Union[WireCall, ByteRow]


class CheckedCall(NamedTuple):
    """A call that the test makes itself after its call of bus_, with its check's number."""

    call: str
    result: str
    note: str
    badge: int


# What the test checks after its call of bus_: a row of bytes or a call of its own
After = Union[ByteRow, CheckedCall]


class Setting(NamedTuple):
    """A member of wire_ that the test sets before its call, and what the value means."""

    code: str
    note: str


class Phase(NamedTuple):
    """One call of bus_ by the test: the settings before it, its lines and its result.

    calls_badge is the number of the check that compares the recorded calls, or 0 when no check
    compares them. after is what the test checks after the call, or None.
    """

    label: str
    settings: Tuple[Setting, ...]
    call: str
    lines: Tuple[Line, ...]
    result: str
    result_badge: int
    calls_badge: int = 0
    after: Optional[After] = None


class Check(NamedTuple):
    """A check of the test: its number on the time line, its macro and what it compares."""

    badge: int
    macro: str
    text: str


class PictureKind(Enum):
    """What a picture's name names."""

    TEST = 'test'
    # A case of a parameterized test
    CASE = 'case'


class Picture(NamedTuple):
    """The picture of one test: its name, a sentence for its caption, its phases and checks.

    caption is HTML.
    """

    name: str
    caption: str
    phases: Tuple[Phase, ...]
    checks: Tuple[Check, ...]
    kind: PictureKind = PictureKind.TEST


def code(text: str) -> str:
    """Return text as inline code in HTML."""
    return f'<code>{html.escape(text)}</code>'


def breakable_name(name: str) -> str:
    """Return a test's name as inline code that may break before each capital and slash."""
    pieces: List[str] = []
    for character in name:
        if (character.isupper() or character == '/') and pieces:
            pieces.append('<wbr>')
        pieces.append(html.escape(character))
    return f'<code>{"".join(pieces)}</code>'


def register_write(address: int, register: int, value: int,
                   end_result: int = SUCCESS) -> Tuple[WireCall, ...]:
    """Return the calls of writeRegister(), with endTransmission()'s result.

    A result other than SUCCESS is one that the test chose.
    """
    acknowledged = end_result == SUCCESS
    return (WireCall(begin_transmission_call(address), 'starts a transfer'),
            WireCall(write_call(register), 'the register'),
            WireCall(write_call(value), 'the value'),
            WireCall(end_transmission_call(WITH_STOP),
                     'stop: frees the bus' if acknowledged else 'not acknowledged',
                     result=str(end_result), result_set_by_test=not acknowledged))


def register_pointer(address: int, register: int,
                     end_result: int = SUCCESS) -> Tuple[WireCall, ...]:
    """Return the calls of readRegisters() that set the register to read, before the request."""
    acknowledged = end_result == SUCCESS
    return (WireCall(begin_transmission_call(address), 'starts a transfer'),
            WireCall(write_call(register), 'the register'),
            WireCall(end_transmission_call(WITH_REPEATED_START),
                     'repeated start' if acknowledged else 'not acknowledged',
                     result=str(end_result), result_set_by_test=not acknowledged))


def delivered_bytes(count: int, requested: int) -> Tuple[Optional[int], ...]:
    """Return the receive buffer after FakeWire delivered count of the requested bytes."""
    return tuple(FIRST_DELIVERED_BYTE + index if index < count else None
                 for index in range(requested))


def successful_read(address: int, register: int, length: int) -> Tuple[Line, ...]:
    """Return the lines of a read that FakeWire serves in full."""
    return (*register_pointer(address, register),
            WireCall(request_from_call(address, length), 'read request', result=str(length)),
            WireCall('available()', f'{length} bytes arrived', Mark.NOT_RECORDED, str(length)),
            WireCall(f'read() × {length}', 'into data', Mark.NOT_RECORDED))


def write_register_pictures() -> Tuple[Picture, ...]:
    """Return the pictures of the four tests of writeRegister()."""
    write_call_text = f'bus_.writeRegister({hex_byte(DEVICE_ADDRESS)}, ' \
                      f'{hex_byte(WRITTEN_REGISTER)}, {hex_byte(WRITTEN_VALUE)})'
    not_acknowledged = Setting(f'wire_.end_result = {ADDRESS_NOT_ACKNOWLEDGED}',
                               'no device acknowledges')
    returns_true = Check(1, 'EXPECT_TRUE', 'the call returns true')
    returns_false = Check(1, 'EXPECT_FALSE', 'the call returns false')
    calls_in_the_box = Check(2, 'EXPECT_EQ', 'wire_.calls holds the boxed calls')
    acknowledged_write = register_write(DEVICE_ADDRESS, WRITTEN_REGISTER, WRITTEN_VALUE)
    failed_write = register_write(DEVICE_ADDRESS, WRITTEN_REGISTER, WRITTEN_VALUE,
                                  ADDRESS_NOT_ACKNOWLEDGED)
    return (
        Picture('WriteRegisterSendsRegisterAndValue',
                'The device acknowledges: four calls, and the result true.',
                (Phase('', (), write_call_text, acknowledged_write, 'true', 1, calls_badge=2),),
                (returns_true, calls_in_the_box)),
        Picture('WriteRegisterFailsWhenTheAddressIsNotAcknowledged',
                f'{code("endTransmission()")} returns {ADDRESS_NOT_ACKNOWLEDGED}, so the result '
                'is false. The test checks only the result.',
                (Phase('', (not_acknowledged,), write_call_text, failed_write, 'false', 1),),
                (returns_false,)),
        Picture('WriteRegisterFailsWhenTheAddressIsNotAcknowledgedAfterASuccessfulWrite',
                f'Two writes on the same {code("bus_")}: the first one succeeds, and the second '
                'one fails.',
                (Phase('1. A write that the device acknowledges', (), write_call_text,
                       acknowledged_write, 'true', 1),
                 Phase('2. The next write, not acknowledged', (not_acknowledged,),
                       write_call_text, failed_write, 'false', 2)),
                (Check(1, 'ASSERT_TRUE', 'the first call returns true'),
                 Check(2, 'EXPECT_FALSE', 'the second call returns false'))),
        Picture('WriteRegisterPassesOnTheAddressRegisterAndValue',
                f'The second set of values, {hex_byte(ALTERNATE_DEVICE_ADDRESS)}, '
                f'{hex_byte(RESET_REGISTER)} and {hex_byte(RESET_VALUE)}, arrives unchanged in '
                'the calls.',
                (Phase('', (), f'bus_.writeRegister({hex_byte(ALTERNATE_DEVICE_ADDRESS)}, '
                               f'{hex_byte(RESET_REGISTER)}, {hex_byte(RESET_VALUE)})',
                       register_write(ALTERNATE_DEVICE_ADDRESS, RESET_REGISTER, RESET_VALUE),
                       'true', 1, calls_badge=2),),
                (returns_true, calls_in_the_box)),
    )


def read_call(address: int, register: int, length: int) -> str:
    """Return the test's call of readRegisters() with the values of its arguments."""
    return (f'bus_.readRegisters({hex_byte(address)}, {hex_byte(register)}, data.data(), '
            f'{length})')


def read_register_pictures() -> Tuple[Picture, ...]:
    """Return the pictures of the four tests of readRegisters() with the fixture WireBusTest."""
    returns_true = Check(1, 'EXPECT_TRUE', 'the call returns true')
    calls_in_the_box = Check(2, 'EXPECT_EQ', 'wire_.calls holds the boxed calls')
    sample = delivered_bytes(SAMPLE_LENGTH_BYTES, SAMPLE_LENGTH_BYTES)
    failed_read_lines: Tuple[Line, ...] = (
        *register_pointer(DEVICE_ADDRESS, READ_REGISTER, ADDRESS_NOT_ACKNOWLEDGED),
        WireCall(request_from_call(DEVICE_ADDRESS, READ_LENGTH_BYTES), 'not called',
                 Mark.NOT_CALLED))
    sample_lines: Tuple[Line, ...] = (
        *register_pointer(DEVICE_ADDRESS, READ_REGISTER),
        WireCall(request_from_call(DEVICE_ADDRESS, SAMPLE_LENGTH_BYTES), 'read request',
                 result=str(SAMPLE_LENGTH_BYTES)),
        ByteRow(f'In the Wire library: {SAMPLE_LENGTH_BYTES} bytes', sample),
        WireCall('available()', f'{SAMPLE_LENGTH_BYTES} bytes arrived', Mark.NOT_RECORDED,
                 str(SAMPLE_LENGTH_BYTES)),
        WireCall(f'read() × {SAMPLE_LENGTH_BYTES}', 'into data', Mark.NOT_RECORDED))
    return (
        Picture('ReadRegistersUsesARepeatedStart',
                'The register write ends with a repeated start, and the read request follows.',
                (Phase('', (), read_call(DEVICE_ADDRESS, READ_REGISTER, READ_LENGTH_BYTES),
                       successful_read(DEVICE_ADDRESS, READ_REGISTER, READ_LENGTH_BYTES),
                       'true', 1, calls_badge=2),),
                (returns_true, calls_in_the_box)),
        Picture('ReadRegistersPassesOnTheAddressAndRegister',
                f'The address {hex_byte(ALTERNATE_DEVICE_ADDRESS)} and the register '
                f'{hex_byte(CHIP_ID_REGISTER)} arrive unchanged in the calls.',
                (Phase('', (),
                       read_call(ALTERNATE_DEVICE_ADDRESS, CHIP_ID_REGISTER, READ_LENGTH_BYTES),
                       successful_read(ALTERNATE_DEVICE_ADDRESS, CHIP_ID_REGISTER,
                                       READ_LENGTH_BYTES),
                       'true', 1, calls_badge=2),),
                (returns_true, calls_in_the_box)),
        Picture('ReadRegistersReturnsAllBytesOfASample',
                f'All {SAMPLE_LENGTH_BYTES} bytes arrive, and {code("data")} holds them in their '
                'order. The test checks the result and the bytes, not the calls.',
                (Phase('', (), read_call(DEVICE_ADDRESS, READ_REGISTER, SAMPLE_LENGTH_BYTES),
                       sample_lines, 'true', 1,
                       after=ByteRow('data after the call', sample, badge=2)),),
                (returns_true,
                 Check(2, 'EXPECT_EQ', 'data holds the delivered bytes'))),
        Picture('ReadRegistersFailsWithoutAReadRequestWhenTheRegisterWriteFails',
                'No device acknowledges the register write, so no read request follows.',
                (Phase('', (Setting(f'wire_.end_result = {ADDRESS_NOT_ACKNOWLEDGED}',
                                    'no device acknowledges'),),
                       read_call(DEVICE_ADDRESS, READ_REGISTER, READ_LENGTH_BYTES),
                       failed_read_lines, 'false', 1, calls_badge=2),),
                (Check(1, 'EXPECT_FALSE', 'the call returns false'),
                 Check(2, 'EXPECT_EQ', 'wire_.calls holds the boxed calls'))),
    )


class ShortRead(NamedTuple):
    """A case of the parameterized test: ShortRead of the test file."""

    name: str
    reported_count: int
    delivered_count: int


SHORT_READS = (
    ShortRead('ReportedShortAllBytesThere', COUNT_AFTER_FAILED_BUS_ACQUISITION,
              SAMPLE_LENGTH_BYTES),
    ShortRead('BothShort', COUNT_AFTER_FAILED_BUS_ACQUISITION,
              COUNT_AFTER_FAILED_BUS_ACQUISITION),
    ShortRead('ReportedInFullOneByteMissing', SAMPLE_LENGTH_BYTES, SAMPLE_LENGTH_BYTES - 1),
    ShortRead('ReportedOneShortAllBytesThere', SAMPLE_LENGTH_BYTES - 1, SAMPLE_LENGTH_BYTES),
)


def check_short_reads(cases: Sequence[ShortRead]) -> None:
    """Raise ValueError if a case reports the requested length and delivers all bytes.

    readRegisters() succeeds for such a read, so the case wouldn't be a short read.
    """
    full = [case.name for case in cases
            if case.reported_count == SAMPLE_LENGTH_BYTES
            and case.delivered_count == SAMPLE_LENGTH_BYTES]
    if full:
        raise ValueError(f'not a short read: {", ".join(full)}')


def shortfall_note(missing: int) -> str:
    """Return the note for the bytes that a read is short of, such as one byte short."""
    return 'one byte short' if missing == 1 else f'{missing} bytes short'


def short_read_lines(case: ShortRead) -> Tuple[Line, ...]:
    """Return the lines of readRegisters() in a case of the parameterized test.

    readRegisters() calls available() in its comparison only when the reported count is the
    requested length, because || stops at the first comparison that is true. Its loop then reads
    the bytes that arrived until available() returns 0.
    """
    lines: List[Line] = [
        *register_pointer(DEVICE_ADDRESS, READ_REGISTER),
        WireCall(request_from_call(DEVICE_ADDRESS, SAMPLE_LENGTH_BYTES), 'read request',
                 result=str(case.reported_count), result_set_by_test=True),
        ByteRow(f'In the Wire library: {case.delivered_count} of {SAMPLE_LENGTH_BYTES} bytes',
                delivered_bytes(case.delivered_count, SAMPLE_LENGTH_BYTES),
                reported=case.reported_count)]
    if case.reported_count == SAMPLE_LENGTH_BYTES:
        lines.append(WireCall('available()',
                              shortfall_note(SAMPLE_LENGTH_BYTES - case.delivered_count),
                              Mark.NOT_RECORDED, str(case.delivered_count)))
    lines.append(WireCall(f'read() × {case.delivered_count}', 'dropped', Mark.NOT_RECORDED))
    return tuple(lines)


def short_read_caption(case: ShortRead) -> str:
    """Return the sentence of a case's caption: what requestFrom() reports and what arrives."""
    if case.delivered_count == SAMPLE_LENGTH_BYTES:
        arrived = f'all {SAMPLE_LENGTH_BYTES} bytes arrive'
    else:
        arrived = f'{case.delivered_count} bytes arrive'
    return f'{code("requestFrom()")} reports {case.reported_count}, and {arrived}.'


def short_read_picture(case: ShortRead) -> Picture:
    """Return the picture of one case of the parameterized test."""
    settings = (Setting(f'wire_.reported_count = {case.reported_count}',
                        'what requestFrom() reports'),
                Setting(f'wire_.delivered_count = {case.delivered_count}',
                        'the bytes that arrive'))
    return Picture(case.name, short_read_caption(case),
                   (Phase('', settings,
                          read_call(DEVICE_ADDRESS, READ_REGISTER, SAMPLE_LENGTH_BYTES),
                          short_read_lines(case), 'false', 1, calls_badge=2,
                          after=CheckedCall('wire_.available()', '0', 'no byte left', 3)),),
                   (Check(1, 'EXPECT_FALSE', 'the call returns false'),
                    Check(2, 'EXPECT_EQ', 'wire_.calls holds the boxed calls'),
                    Check(3, 'EXPECT_EQ', 'wire_.available() is 0')),
                   PictureKind.CASE)


class PictureDrawing(Drawing):
    """Collect the SVG elements of one picture, and the y of its next line.

    The elements of background, such as the box around the compared calls and the time line,
    are drawn first, behind the lines.
    """

    def __init__(self, name: str) -> None:
        super().__init__()
        self.name = name
        self.background: List[str] = []
        self.y = 0.0

    def rich_text(self, x: float, y: float, parts: Sequence[Tuple[str, str]]) -> None:
        """Add one line of text from parts, each its text and its CSS class."""
        spans = ''.join(f'<tspan class="{css_class}">{escape(text)}</tspan>' if css_class
                        else escape(text) for text, css_class in parts)
        self.elements.append(f'<text x="{x:g}" y="{y:g}">{spans}</text>')

    def badge(self, x: float, y: float, number: int) -> None:
        """Add a check's number in a filled disc around (x, y)."""
        self.number_disc(x, y, number, 'badge', 'badge-number')

    def svg(self, title: str) -> str:
        """Return the picture as an SVG element, as high as its lines."""
        return svg_element(self.name, 'wbt', (WIDTH, round(self.y + 6)), title,
                           self.background + self.elements)


def draw_settings(drawing: PictureDrawing, settings: Sequence[Setting]) -> None:
    """Add the members of wire_ that the test sets before its call, one per line."""
    for setting in settings:
        drawing.y += 16
        drawing.rich_text(LEFT_X, drawing.y, ((setting.code, 'mono set'),))
        drawing.text(RIGHT_X, drawing.y, setting.note, 'note', 'end')


def draw_wire_call(drawing: PictureDrawing, wire_call: WireCall) -> None:
    """Add a line of WireBusT's calls: the dot on the time line, the call, its result, a note."""
    drawing.y += ROW_HEIGHT
    dot_y = drawing.y - 3.5
    if wire_call.mark is Mark.RECORDED:
        drawing.circle(TIME_X, dot_y, 3.5, 'recorded')
    else:
        drawing.circle(TIME_X, dot_y, 3.2, wire_call.mark.value)
    call_class = {Mark.RECORDED: 'mono', Mark.NOT_RECORDED: 'mono',
                  Mark.NOT_CALLED: 'mono not-called'}[wire_call.mark]
    drawing.text(CALL_X, drawing.y, wire_call.call, call_class)
    if wire_call.result:
        result_class = 'mono set' if wire_call.result_set_by_test else 'mono'
        drawing.rich_text(RESULT_X, drawing.y, (('→ ', 'mono'), (wire_call.result, result_class)))
    drawing.text(NOTE_X, drawing.y, wire_call.note, 'note')


def draw_byte_row(drawing: PictureDrawing, row: ByteRow, x: float) -> None:
    """Add a row of bytes with its label above it, and the reported count's bracket below it.

    A byte that arrived is a box with its value in hex, without 0x. A byte that didn't arrive is
    a dashed box. The bracket spans as many boxes as requestFrom() reports.
    """
    drawing.y += 20
    if row.badge:
        drawing.badge(LEFT_X + NUMBER_DISC_RADIUS, drawing.y + 13, row.badge)
    drawing.text(x, drawing.y, row.label, 'note')
    top = drawing.y + 5
    for index, value in enumerate(row.values):
        cell_x = x + index * CELL_WIDTH
        if value is None:
            drawing.rect(cell_x + 1, top + 1, CELL_WIDTH - 2, CELL_HEIGHT - 2, 'empty-cell', 2)
        else:
            drawing.rect(cell_x, top, CELL_WIDTH, CELL_HEIGHT, 'cell', 2)
            drawing.text(cell_x + CELL_WIDTH / 2, top + 11.5, f'{value:02X}', 'byte', 'middle')
    drawing.y = top + CELL_HEIGHT
    if row.reported:
        bracket_y = drawing.y + 5
        end_x = x + row.reported * CELL_WIDTH - 1
        bracket = f'M{x + 1:g},{bracket_y - 4:g} V{bracket_y:g} H{end_x:g} V{bracket_y - 4:g}'
        drawing.line(bracket, 'reported')
        drawing.y = bracket_y + 13
        drawing.rich_text(x, drawing.y, (('requestFrom() reports ', 'note'),
                                         (str(row.reported), 'set')))


def draw_lines(drawing: PictureDrawing, lines: Sequence[Line]) -> List[float]:
    """Add the lines inside the call of bus_ and return the baselines of the recorded calls."""
    recorded: List[float] = []
    for line in lines:
        if isinstance(line, ByteRow):
            draw_byte_row(drawing, line, CALL_X)
            drawing.y += 2
        else:
            draw_wire_call(drawing, line)
            if line.mark is Mark.RECORDED:
                recorded.append(drawing.y)
    return recorded


def draw_calls_box(drawing: PictureDrawing, baselines: Sequence[float], badge: int) -> None:
    """Add the box around the recorded calls that a check compares, with the check's number.

    The box holds the calls' text and leaves their results outside. The number sits on the box's
    top right corner.
    """
    top = baselines[0] - ROW_HEIGHT + 6
    bottom = baselines[-1] + 6
    width = CALLS_BOX_RIGHT - LEFT_X
    drawing.background.append(f'<rect class="calls-box" x="{LEFT_X:g}" y="{top:g}" '
                              f'width="{width:g}" height="{bottom - top:g}" rx="4"/>')
    drawing.badge(CALLS_BOX_RIGHT, top, badge)


def draw_checked_call(drawing: PictureDrawing, checked_call: CheckedCall) -> None:
    """Add a call that the test makes after its call of bus_, with its check's number."""
    drawing.y += ROW_HEIGHT + 4
    drawing.badge(LEFT_X + NUMBER_DISC_RADIUS, drawing.y - 4, checked_call.badge)
    drawing.text(CALL_X, drawing.y, checked_call.call, 'mono')
    drawing.rich_text(RESULT_X, drawing.y, (('→ ', 'mono'), (checked_call.result, 'mono')))
    drawing.text(NOTE_X, drawing.y, checked_call.note, 'note')


def draw_phase(drawing: PictureDrawing, phase: Phase) -> None:
    """Add one call of bus_: its label, the settings, the call, the time line and the result."""
    if phase.label:
        # A later phase keeps its distance from the result of the one before
        drawing.y += 18 if drawing.y == 0 else 28
        drawing.text(LEFT_X, drawing.y, phase.label, 'phase')
    draw_settings(drawing, phase.settings)
    drawing.y += 18
    drawing.text(LEFT_X, drawing.y, phase.call, 'mono')
    time_line_top = drawing.y + 5
    if phase.calls_badge:
        drawing.y += CALLS_BOX_GAP
    recorded = draw_lines(drawing, phase.lines)
    if phase.calls_badge:
        draw_calls_box(drawing, recorded, phase.calls_badge)
    drawing.y += 30
    result_y = drawing.y
    drawing.background.append(
        f'<path class="time-line" d="M{TIME_X},{time_line_top:g} V{result_y - 15:g}" '
        f'marker-end="url(#{drawing.name}-head)"/>')
    drawing.badge(LEFT_X + NUMBER_DISC_RADIUS, result_y - 4, phase.result_badge)
    drawing.rich_text(CALL_X, result_y, (('returns ', ''), (phase.result, 'mono')))
    if isinstance(phase.after, ByteRow):
        drawing.y += 4
        draw_byte_row(drawing, phase.after, CALL_X)
    elif isinstance(phase.after, CheckedCall):
        draw_checked_call(drawing, phase.after)


def draw_checks(drawing: PictureDrawing, checks: Sequence[Check]) -> None:
    """Add the list of the test's checks, each with its number."""
    drawing.y += 8
    for check in checks:
        drawing.y += CHECK_HEIGHT
        drawing.badge(LEFT_X + NUMBER_DISC_RADIUS, drawing.y - 4, check.badge)
        drawing.rich_text(CALL_X, drawing.y, ((check.macro, 'mono'), (': ' + check.text, '')))


def picture_html(picture: Picture, prefix: str) -> str:
    """Return the HTML of one picture: its SVG and its caption."""
    drawing = PictureDrawing(f'{prefix}-{picture.name}')
    for phase in picture.phases:
        draw_phase(drawing, phase)
    draw_checks(drawing, picture.checks)
    title = f'The time line of the {picture.kind.value} {picture.name}'
    return (f'<figure class="wbt-picture">\n{drawing.svg(title)}\n'
            f'<figcaption>{breakable_name(picture.name)}: {picture.caption}</figcaption>\n'
            '</figure>')


def figure_include(pictures: Sequence[Picture], prefix: str, caption: str) -> str:
    """Return an include: the style, and one figure with the pictures of a group of tests."""
    pictures_html = '\n'.join(picture_html(picture, prefix) for picture in pictures)
    return ('<!-- Written by scripts/figures/firmware_unit_tests_figures.py -->\n'
            f'{style_element(RULES)}\n<figure class="wbt-figure">\n<div class="wbt-grid">\n'
            f'{pictures_html}\n</div>\n<figcaption>{caption}</figcaption>\n</figure>\n')


def write_register_caption() -> str:
    """Return the caption of the pictures of writeRegister(), which explains how to read them."""
    return (f'The four tests of {code("writeRegister()")}, one time line per test. Time runs '
            f'down. The test calls {code("bus_")} with the values of the top line. '
            f"{code('WireBusT')}'s calls of {code('FakeWire')} follow, each with a filled dot on "
            f'the time line: a call that {code("FakeWire")} records in {code("calls")}. The '
            'result comes back at the bottom. A bold orange value is one that the test sets on '
            f'{code("wire_")} before the call. A number in a blue disc is a check, and the list '
            'under the time line names it. The disc on the corner of a box marks the calls that '
            f'{code("EXPECT_EQ")} compares.')


def read_registers_caption() -> str:
    """Return the caption of the pictures of readRegisters() with the fixture WireBusTest."""
    return (f'The four tests of {code("readRegisters()")} with the fixture '
            f'{code("WireBusTest")}. The dots, values and numbers read as in the pictures of '
            f'{code("writeRegister()")}: time runs down, a filled dot is a call that '
            f'{code("calls")} lists, a bold orange value is set by the test, and a number in a '
            f"blue disc is a check. An open dot is a call that {code('calls')} doesn't list: "
            f'{code("available()")} or {code("read()")}. A struck-through call on a dashed dot '
            'is one that the test expects not to happen. A row of boxes holds bytes in hex, such '
            f'as {FIRST_DELIVERED_BYTE:02X} for {hex_byte(FIRST_DELIVERED_BYTE)}.')


def short_reads_caption() -> str:
    """Return the caption of the pictures of the short reads."""
    return (f'The four cases of {code("ReadRegistersFailsAndDrainsTheBytes")}. The dots, values '
            f'and numbers read as in the pictures of {code("writeRegister()")} and '
            f'{code("readRegisters()")}: time runs down, a filled dot is a call that '
            f"{code('calls')} lists, an open dot one that it doesn't list, a bold orange value "
            'is set by the test, and a number in a blue disc is a check. Each case requests '
            f"a sample's {SAMPLE_LENGTH_BYTES} bytes. The row of boxes is the Wire library's "
            f'receive buffer after {code("requestFrom()")}: a box with a value is a byte that '
            "arrived, a dashed box one that didn't. The orange bracket under it spans the count "
            f'that {code("requestFrom()")} reports. The {code("read()")} calls drop the bytes, '
            f"and the line after the result is the test's own call of {code('available()')}.")


def main() -> None:
    """Check the short reads, then write the three includes."""
    check_short_reads(SHORT_READS)
    includes = (
        ('includes/firmware-unit-tests-write-register.html',
         figure_include(write_register_pictures(), 'wbt-write', write_register_caption())),
        ('includes/firmware-unit-tests-read-registers.html',
         figure_include(read_register_pictures(), 'wbt-read', read_registers_caption())),
        ('includes/firmware-unit-tests-short-reads.html',
         figure_include(tuple(short_read_picture(case) for case in SHORT_READS), 'wbt-short',
                        short_reads_caption())),
    )
    for path, content in includes:
        with open(path, 'w', encoding='utf-8') as include:
            include.write(content)


if __name__ == '__main__':
    main()
