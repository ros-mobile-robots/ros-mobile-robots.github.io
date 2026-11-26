"""Run a figure script as CI's locale would, for the tests of the figure scripts.

A test module imports run_with_an_ascii_locale() after it has put this folder on sys.path. The
name of this file doesn't start with test_, so the test discovery doesn't collect it.
"""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Dict
from typing import Sequence

ROOT = Path(__file__).resolve().parent.parent
FIGURES = ROOT / 'scripts' / 'figures'
# With these variables, Python's default encoding for a text file is ASCII: the C locale, without
# Python's own switch to UTF-8 for that locale
ASCII_LOCALE = {'LC_ALL': 'C', 'LANG': 'C', 'PYTHONCOERCECLOCALE': '0', 'PYTHONUTF8': '0'}


def run_with_an_ascii_locale(script_name: str,
                             input_folders: Sequence[str] = ()) -> Dict[str, bytes]:
    """Run a figure script with an ASCII locale and return the includes it writes, by file name.

    The script runs in a new folder, so the repository's includes stay as they are. The folder
    holds an empty includes folder and a copy of each of input_folders, which are paths from the
    repository's root, such as the measured data that a script reads. Without them, the script
    finds no input.
    """
    with tempfile.TemporaryDirectory() as folder:
        includes = Path(folder) / 'includes'
        includes.mkdir()
        for input_folder in input_folders:
            shutil.copytree(ROOT / input_folder, Path(folder) / input_folder)
        run = subprocess.run([sys.executable, str(FIGURES / script_name)], cwd=folder,
                             env={**os.environ, **ASCII_LOCALE}, capture_output=True, text=True)
        if run.returncode != 0:
            raise RuntimeError(f'{script_name} failed: {run.stderr.strip()}')
        return {include.name: include.read_bytes() for include in includes.iterdir()}
