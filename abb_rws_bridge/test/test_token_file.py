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

"""The token file: where it is, how it is read, and what an error may say."""

import os

from abb_rws_bridge import token_file
from abb_rws_bridge.token_file import default_token_path, read_token, TokenFileError
import pytest

# Synthetic: 43 characters, the length of a token the daemon makes.
TOKEN = 'synthetic_Token-0123456789-abcdefghijklmnop'
NO_TOKEN = "does not hold an access token (32 to 256 letters, digits, '-' or '_')"


def test_default_path_is_the_named_file_when_the_variable_is_set():
    env = {'ABB_RWS_DAEMON_TOKEN_FILE': '/run/secrets/daemon_token',
           'XDG_CONFIG_HOME': '/xdg'}
    assert default_token_path(env, '/home/someone') == '/run/secrets/daemon_token'


def test_default_path_follows_the_config_home():
    assert default_token_path({'XDG_CONFIG_HOME': '/xdg'}, '/home/someone') == os.path.join(
        '/xdg', 'abb-rws-daemon', 'token')


@pytest.mark.parametrize('env', [
    {}, {'XDG_CONFIG_HOME': ''}, {'XDG_CONFIG_HOME': 'relative/folder'},
    {'ABB_RWS_DAEMON_TOKEN_FILE': ''}, {'ABB_RWS_DAEMON_TOKEN_FILE': '', 'XDG_CONFIG_HOME': ''},
])
def test_default_path_falls_back_to_the_home_folder(env):
    assert default_token_path(env, '/home/someone') == os.path.join(
        '/home/someone', '.config', 'abb-rws-daemon', 'token')


def test_default_path_reads_nothing_but_the_mapping_it_is_given(monkeypatch):
    monkeypatch.setenv('ABB_RWS_DAEMON_TOKEN_FILE', '/from/the/real/environment')
    monkeypatch.setenv('XDG_CONFIG_HOME', '/also/real')
    assert default_token_path({}, '/h') == os.path.join('/h', '.config', 'abb-rws-daemon', 'token')


@pytest.mark.parametrize('content', [
    TOKEN,
    TOKEN + '\n',
    TOKEN + '\r\n',
    '\n\n  ' + TOKEN + ' \t\r\n\r\n',
    '\ufeff' + TOKEN + '\r\n',
], ids=['bare', 'newline', 'crlf', 'spaces and blank lines', 'byte order mark'])
def test_surrounding_whitespace_is_removed(tmp_path, content):
    path = tmp_path / 'token'
    path.write_bytes(content.encode('utf-8'))
    assert read_token(str(path)) == TOKEN


@pytest.mark.parametrize('length', [32, 43, 256])
def test_tokens_of_every_allowed_length_are_read(tmp_path, length):
    token = ('aB3_-' * 60)[:length]
    path = tmp_path / 'token'
    path.write_text(token + '\n', encoding='ascii')
    assert read_token(str(path)) == token


BAD_CONTENT = {
    'empty': b'',
    'only whitespace': b' \r\n\t\n',
    'too short': b'a' * 31 + b'\n',
    'too long': b'a' * 257 + b'\n',
    'far too long': b'a' * 9000,
    'a space inside': b'a' * 20 + b' ' + b'b' * 20,
    'a line break inside': b'a' * 20 + b'\n' + b'b' * 20,
    'two tokens': TOKEN.encode() + b'\n' + TOKEN.encode() + b'\n',
    'a character outside the set': b'a' * 40 + b'+/=',
    'a letter outside ASCII': 'a' * 40 + '\u00e9',
    'bytes that are not text': b'\xff\xfe' + b'a' * 40,
    'a NUL': b'a' * 40 + b'\x00',
    'json around it': b'{"token": "' + TOKEN.encode() + b'"}',
}


@pytest.mark.parametrize('label', sorted(BAD_CONTENT))
def test_content_that_is_no_token_is_refused_without_quoting_it(tmp_path, label):
    content = BAD_CONTENT[label]
    raw = content if isinstance(content, bytes) else content.encode('utf-8')
    path = tmp_path / 'token'
    path.write_bytes(raw)
    with pytest.raises(TokenFileError) as caught:
        read_token(str(path))
    # The whole message is the path and one fixed reason: nothing in it depends on the
    # content, so neither the content nor its length can be read from it.
    assert str(caught.value) == f'token file {path} {NO_TOKEN}'
    _quotes_nothing_of(caught.value, raw, str(path))


def _unchained(error):
    """Return whether a traceback of error would show no other exception with it."""
    return error.__cause__ is None and (
        error.__context__ is None or error.__suppress_context__)


def _quotes_nothing_of(error, raw, path):
    """Check that no text of an error holds a piece of the file's content."""
    text = raw.decode('utf-8', 'replace')
    assert _unchained(error)
    for shown in (str(error), repr(error), repr(error.args)):
        rest = shown.replace(path, '')
        for piece in (text, text.strip(), text[:8], text.strip()[-8:], TOKEN[:12]):
            assert not piece.strip() or piece not in rest


def test_the_message_for_bad_content_is_the_same_whatever_the_content(tmp_path):
    messages = set()
    path = tmp_path / 'token'
    for label in sorted(BAD_CONTENT):
        content = BAD_CONTENT[label]
        path.write_bytes(content if isinstance(content, bytes) else content.encode('utf-8'))
        with pytest.raises(TokenFileError) as caught:
            read_token(str(path))
        messages.add(str(caught.value))
    assert len(messages) == 1


def test_a_missing_file_names_the_path_and_the_reason(tmp_path):
    path = str(tmp_path / 'nowhere' / 'token')
    with pytest.raises(TokenFileError) as caught:
        read_token(path)
    assert path in str(caught.value) and 'does not exist' in str(caught.value)
    assert _unchained(caught.value)


def test_a_folder_is_not_a_token_file(tmp_path):
    with pytest.raises(TokenFileError) as caught:
        read_token(str(tmp_path))
    assert str(tmp_path) in str(caught.value) and 'folder' in str(caught.value)


def test_no_path_is_an_error_of_its_own():
    with pytest.raises(TokenFileError) as caught:
        read_token('')
    assert 'no token file' in str(caught.value)


@pytest.mark.parametrize('error, reason', [
    (PermissionError(13, 'Permission denied'), 'permission denied'),
    (OSError(5, 'Input/output error'), 'Input/output error'),
    (OSError('no number, no text'), 'OSError'),
])
def test_a_file_that_cannot_be_read_names_the_path_and_the_reason(
        tmp_path, monkeypatch, error, reason):
    path = tmp_path / 'token'
    path.write_text(TOKEN, encoding='ascii')

    def refuse(*args, **kwargs):
        raise error

    monkeypatch.setattr(token_file, 'open', refuse, raising=False)
    with pytest.raises(TokenFileError) as caught:
        read_token(str(path))
    message = str(caught.value)
    assert str(path) in message and reason in message and 'cannot be read' in message
    assert TOKEN not in message and str(len(TOKEN)) not in message.replace(str(path), '')
    assert _unchained(caught.value)


def test_the_file_is_read_again_on_every_call(tmp_path):
    path = tmp_path / 'token'
    path.write_text(TOKEN + '\n', encoding='ascii')
    assert read_token(str(path)) == TOKEN
    other = TOKEN[::-1]
    path.write_text(other + '\r\n', encoding='ascii')
    assert read_token(str(path)) == other
    path.unlink()
    with pytest.raises(TokenFileError):
        read_token(str(path))


def test_no_more_than_a_small_part_of_a_large_file_is_read(tmp_path, monkeypatch):
    path = tmp_path / 'token'
    path.write_bytes(b'a' * 100000)
    asked = []
    real_open = open

    class Handle:

        def __init__(self, handle):
            self._handle = handle

        def __enter__(self):
            return self

        def __exit__(self, *exc_info):
            self._handle.close()

        def read(self, size=-1):
            asked.append(size)
            return self._handle.read(size)

    monkeypatch.setattr(token_file, 'open', lambda *args: Handle(real_open(*args)),
                        raising=False)
    with pytest.raises(TokenFileError):
        read_token(str(path))
    assert len(asked) == 1 and 0 < asked[0] <= 8192


def test_the_error_is_an_exception_of_its_own_kind():
    assert issubclass(TokenFileError, Exception)
    assert not issubclass(TokenFileError, OSError)
