# Copyright 2026 Meraj Safari
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The checked-in generated files are what the generator writes from the vendored schema."""

from pathlib import Path
import subprocess
import sys

PACKAGE_DIR = Path(__file__).resolve().parent.parent
GENERATOR = PACKAGE_DIR / 'scripts' / 'generate_msgs.py'


def test_generated_files_match_the_vendored_schema():
    done = subprocess.run(
        [sys.executable, '-B', str(GENERATOR), '--check'],
        capture_output=True, text=True, timeout=60)
    assert done.returncode == 0, (
        'generate_msgs.py --check found differences:\n' + done.stdout + done.stderr)
    assert 'match' in done.stdout
