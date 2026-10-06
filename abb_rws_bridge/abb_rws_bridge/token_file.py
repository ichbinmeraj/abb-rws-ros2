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

"""
The file that holds the daemon's access token: where it is, and how it is read.

The daemon writes its token to a file only its own account can read. The bridge is
given that file's path and reads the token from it; the token itself is never a
parameter, and nothing here puts it, or anything about it, into a message.
"""

import os
import re
from typing import Mapping

TOKEN_FILE_VARIABLE = 'ABB_RWS_DAEMON_TOKEN_FILE'
CONFIG_HOME_VARIABLE = 'XDG_CONFIG_HOME'
DAEMON_FOLDER = 'abb-rws-daemon'
TOKEN_FILE_NAME = 'token'

# What a token looks like (the daemon's own rule): 32 to 256 letters, digits, '-' or '_'.
_TOKEN = re.compile(r'[A-Za-z0-9_-]{32,256}')
# A token file is one short line; nothing larger is read into memory.
_READ_LIMIT = 4096
_NO_TOKEN = "does not hold an access token (32 to 256 letters, digits, '-' or '_')"


class TokenFileError(Exception):
    """
    The token file could not be used.

    The message names the path and the reason. It never holds the file's content or
    anything derived from it, its length included.
    """


def default_token_path(env: Mapping[str, str], home: str) -> str:
    """
    Return where the token file is when no path was configured.

    The daemon's own rule: the file named by ABB_RWS_DAEMON_TOKEN_FILE when that
    variable is set, otherwise ``abb-rws-daemon/token`` under XDG_CONFIG_HOME, or under
    ``.config`` in the home folder when XDG_CONFIG_HOME is unset, empty or relative.
    """
    named = env.get(TOKEN_FILE_VARIABLE)
    if named:
        return named
    config_home = env.get(CONFIG_HOME_VARIABLE)
    if not config_home or not os.path.isabs(config_home):
        config_home = os.path.join(home, '.config')
    return os.path.join(config_home, DAEMON_FOLDER, TOKEN_FILE_NAME)


def read_token(path: str) -> str:
    """
    Return the access token in the file at path.

    Whitespace around the content is removed first (the daemon does the same, and a
    file written on Windows may end in CRLF or start with a byte order mark); what is
    left must be a token. Raises TokenFileError otherwise, with the path and the
    reason in its message.
    """
    if not path:
        raise TokenFileError('no token file is configured')
    try:
        with open(path, 'rb') as handle:
            raw = handle.read(_READ_LIMIT + 1)
    except FileNotFoundError:
        raise TokenFileError(
            f'token file {path} does not exist (the daemon makes it when it starts)') from None
    except IsADirectoryError:
        raise TokenFileError(f'token file {path} is a folder, not a file') from None
    except PermissionError:
        raise TokenFileError(
            f'token file {path} cannot be read by this account (permission denied)') from None
    except OSError as error:
        reason = error.strerror or type(error).__name__
        raise TokenFileError(f'token file {path} cannot be read ({reason})') from None
    if len(raw) > _READ_LIMIT:
        raise TokenFileError(f'token file {path} {_NO_TOKEN}')
    try:
        text = raw.decode('utf-8-sig')
    except UnicodeDecodeError:
        raise TokenFileError(f'token file {path} {_NO_TOKEN}') from None
    token = text.strip()
    if _TOKEN.fullmatch(token) is None:
        raise TokenFileError(f'token file {path} {_NO_TOKEN}')
    return token
