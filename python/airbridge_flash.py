#!/usr/bin/env python3
"""Flash AirBridge (not AirSense) using the WebUI HTTP OTA API.

Adapted from AirCANnect's CLI workflow. Uses only the Python standard library;
no espota callback connection is needed, including when running under WSL/NAT.
"""

from __future__ import annotations

import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import http.client
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit
import uuid
import zlib


PROJECT_DIR = Path(__file__).resolve().parents[1]
ENVIRONMENT_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]*$')
OUTPUT_LOCK = threading.Lock()
OUTPUT_CONTEXT = threading.local()
MULTI_OUTPUT = None


@dataclass
class OutputContext:
    inline_status: bool = False
    last_status: str = ''
    last_status_time: float = 0.0
    upload_percent: int = -1
    upload_time: float = 0.0


def output_context():
    if not hasattr(OUTPUT_CONTEXT, 'value'):
        OUTPUT_CONTEXT.value = OutputContext()
    return OUTPUT_CONTEXT.value


class MultiTargetOutput:
    def __init__(self, labels):
        self.labels = labels
        self.label_width = max(map(len, labels))
        self.statuses = dict.fromkeys(labels, 'queued')
        self.tty = sys.stdout.isatty()
        self.lines_drawn = 0

    def start(self):
        if self.tty:
            self.render()
        else:
            print('targets: ' + ', '.join(self.labels), flush=True)

    def update(self, label, message, error=False):
        self.statuses[label] = 'ERROR: ' + message if error else message
        if self.tty:
            self.render()
        else:
            print(f'[{label}] {message}', file=sys.stderr if error else sys.stdout, flush=True)

    def render(self):
        if self.lines_drawn:
            sys.stdout.write(f'\x1b[{self.lines_drawn}A')
        for label in self.labels:
            sys.stdout.write(f'\r\x1b[2K{label:<{self.label_width}}  {self.statuses[label]}\n')
        sys.stdout.flush()
        self.lines_drawn = len(self.labels)


def target_label(target):
    host = f'[{target.host}]' if ':' in target.host else target.host
    port = '' if target.port == (443 if target.scheme == 'https' else 80) else f':{target.port}'
    return host + port + target.base_path


def format_bytes(value):
    amount = float(value)
    for unit in ('B', 'KiB', 'MiB', 'GiB'):
        if amount < 1024 or unit == 'GiB':
            return f'{value} B' if unit == 'B' else f'{amount:.1f} {unit}'
        amount /= 1024


class FlashError(Exception):
    pass


@dataclass(frozen=True)
class Target:
    scheme: str
    host: str
    port: int
    base_path: str = ''

    @property
    def label(self):
        host = f'[{self.host}]' if ':' in self.host else self.host
        return f'{self.scheme}://{host}:{self.port}{self.base_path}'


def emit(target, message, *, error=False):
    context = output_context()
    with OUTPUT_LOCK:
        if MULTI_OUTPUT is not None and target is not None:
            MULTI_OUTPUT.update(target_label(target), message, error)
            return
        if context.inline_status:
            sys.stdout.write('\n')
            context.inline_status = False
        print(message, file=sys.stderr if error else sys.stdout, flush=True)


def update_status(target, message, *, force=False):
    context = output_context()
    now = time.monotonic()
    if MULTI_OUTPUT is not None and not force and now - context.last_status_time < 1:
        return
    with OUTPUT_LOCK:
        if MULTI_OUTPUT is not None:
            MULTI_OUTPUT.update(target_label(target), message)
        elif sys.stdout.isatty():
            padding = ' ' * max(0, len(context.last_status) - len(message))
            sys.stdout.write('\r' + message + padding)
            sys.stdout.flush()
            context.inline_status = True
        else:
            print(message, flush=True)
    context.last_status = message
    context.last_status_time = now


def finish_status():
    context = output_context()
    if context.inline_status:
        with OUTPUT_LOCK:
            sys.stdout.write('\n')
            sys.stdout.flush()
        context.inline_status = False


def print_progress(target, sent, total, *, force=False):
    context = output_context()
    percent = sent * 100 // total
    now = time.monotonic()
    if percent == context.upload_percent and not force:
        return
    if not force:
        if not sys.stdout.isatty():
            if percent < min(100, context.upload_percent + 25):
                return
        elif now - context.upload_time < 0.5:
            return
    context.upload_percent = percent
    context.upload_time = now
    update_status(target, f'uploading: {percent:3d}% ({format_bytes(sent)} / {format_bytes(total)})',
                  force=force or not sys.stdout.isatty())


def parse_target(value):
    try:
        parsed = urlsplit(value if '://' in value else 'http://' + value)
        if (parsed.scheme not in ('http', 'https') or not parsed.hostname or
                parsed.username is not None or parsed.password is not None or
                parsed.query or parsed.fragment):
            raise ValueError('expected a host or HTTP(S) URL without credentials/query/fragment')
        return Target(parsed.scheme, parsed.hostname,
                      parsed.port or (443 if parsed.scheme == 'https' else 80),
                      parsed.path.rstrip('/'))
    except ValueError as error:
        raise FlashError(f'invalid target: {error}') from error


def auth_header(user, password):
    token = base64.b64encode(f'{user}:{password}'.encode()).decode('ascii')
    return 'Basic ' + token


def connection(target, timeout):
    cls = http.client.HTTPSConnection if target.scheme == 'https' else http.client.HTTPConnection
    return cls(target.host, target.port, timeout=timeout)


def decode_response(response):
    raw = response.read()
    if not 200 <= response.status < 300:
        raise FlashError(f'HTTP {response.status}: {raw[:300].decode(errors="replace")}')
    try:
        body = json.loads(raw)
    except (ValueError, UnicodeError) as error:
        raise FlashError('device returned invalid JSON') from error
    if not isinstance(body, dict):
        raise FlashError('device response is not a JSON object')
    return body


def request_json(target, method, path, auth, timeout, *, allow_missing=False):
    conn = connection(target, timeout)
    try:
        headers = {'Accept': 'application/json'}
        if auth:
            headers['Authorization'] = auth
        conn.request(method, target.base_path + path, headers=headers)
        response = conn.getresponse()
        if allow_missing and response.status == 404:
            response.read()
            return None
        return decode_response(response)
    finally:
        conn.close()


def is_ok(body):
    # Older upload handlers encode ok as a string; reboot uses a JSON boolean.
    return body.get('ok') is True or body.get('ok') == 'true'


def resolve_environment(explicit, metadata):
    environment = explicit or (metadata or {}).get('release_target')
    if not environment:
        raise FlashError('device does not report release_target; pass --env or --file')
    # Only constrain path syntax, not board names or PlatformIO configuration.
    if not isinstance(environment, str) or not ENVIRONMENT_PATTERN.fullmatch(environment):
        raise FlashError('invalid environment name')
    return environment


def load_firmware(path):
    data = path.read_bytes()
    if data and data[0] != 0xE9:
        try:
            decoder = zlib.decompressobj()
            data = decoder.decompress(data) + decoder.flush()
            if not decoder.eof or decoder.unused_data:
                raise FlashError(f'{path}: incomplete zlib image or trailing data')
        except zlib.error as error:
            raise FlashError(f'{path}: invalid ESP or zlib image') from error
    if not data or data[0] != 0xE9:
        raise FlashError(f'{path}: empty file or invalid ESP image magic')
    return data


def upload(target, data, auth, timeout, chunk_size, *, image_size=None, encoding='plain'):
    started = time.monotonic()
    image_size = len(data) if image_size is None else image_size
    filename = 'firmware.bin.zlib' if encoding == 'zlib' else 'firmware.bin'
    boundary = '----airbridge-' + uuid.uuid4().hex
    prefix = (f'--{boundary}\r\n'
              f'Content-Disposition: form-data; name="firmware"; filename="{filename}"\r\n'
              'Content-Type: application/octet-stream\r\n\r\n').encode()
    suffix = f'\r\n--{boundary}--\r\n'.encode()
    conn = connection(target, timeout)
    try:
        conn.putrequest('POST', target.base_path + '/api/esp32/upload', skip_accept_encoding=True)
        conn.putheader('Content-Type', f'multipart/form-data; boundary={boundary}')
        conn.putheader('Content-Length', str(len(prefix) + len(data) + len(suffix)))
        conn.putheader('Accept', 'application/json')
        if auth:
            conn.putheader('Authorization', auth)
        conn.endheaders()
        conn.send(prefix)
        print_progress(target, 0, len(data), force=True)
        for offset in range(0, len(data), chunk_size):
            chunk = data[offset:offset + chunk_size]
            conn.send(chunk)
            print_progress(target, offset + len(chunk), len(data))
        conn.send(suffix)
        sent = time.monotonic()
        print_progress(target, len(data), len(data), force=True)
        finish_status()
        body = decode_response(conn.getresponse())
        if not is_ok(body):
            raise FlashError(f'upload rejected: {body.get("error") or body}')
        if body.get('size') != image_size:
            raise FlashError(f'upload size mismatch: expected {image_size}, got {body.get("size")}')
        if encoding == 'zlib' and (body.get('wire_size') != len(data) or body.get('encoding') != 'zlib'):
            raise FlashError('compressed upload confirmation mismatch')
        emit(target, f'upload confirmed, partition={body.get("partition", "unknown")} '
             f'(send {sent - started:.1f}s, confirmation {time.monotonic() - sent:.1f}s)')
    except (OSError, http.client.HTTPException) as error:
        raise FlashError(f'upload connection failed ({error}); outcome unknown, not retrying or rebooting') from error
    finally:
        conn.close()
        finish_status()


def wait_for_reboot(target, previous_uptime, started, auth, timeout, reboot_timeout):
    emit(target, 'waiting for reboot/API...')
    wait_started = time.monotonic()
    deadline = wait_started + reboot_timeout
    while time.monotonic() < deadline:
        time.sleep(min(1, max(0, deadline - time.monotonic())))
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        try:
            status = request_json(target, 'GET', '/api/status', auth,
                                  min(timeout, 2.0, remaining))
        except (OSError, http.client.HTTPException):
            continue
        uptime = status.get('uptime')
        # Reachability alone does not prove reboot: the old app can still reply.
        expected = previous_uptime + time.monotonic() - started
        if isinstance(uptime, (int, float)) and uptime < expected - 2:
            emit(target, f'reboot confirmed, version={status.get("version", "unknown")} '
                 f'(API wait {time.monotonic() - wait_started:.1f}s)')
            return
    raise FlashError('upload succeeded, but reboot could not be confirmed before timeout')


def flash_target(target, data, args, auth, encoding='plain'):
    OUTPUT_CONTEXT.value = OutputContext()
    if MULTI_OUTPUT is None:
        emit(target, f'target: {target.label}')
    try:
        wire_data = zlib.compress(data, level=6) if encoding == 'zlib' else data
        upload(target, wire_data, auth, args.timeout, args.chunk_size,
               image_size=len(data), encoding=encoding)
        if args.no_reboot:
            emit(target, 'firmware staged; reboot manually to activate it')
            return 0
        status = request_json(target, 'GET', '/api/status', auth, args.timeout)
        uptime = status.get('uptime')
        if not isinstance(uptime, (int, float)):
            raise FlashError('upload succeeded, but device uptime is unavailable; reboot manually')
        started = time.monotonic()
        try:
            # Restart can strand this connection; confirm on a fresh status read.
            body = request_json(target, 'POST', '/api/reboot', auth, min(args.timeout, 2.0))
            if not is_ok(body):
                raise FlashError(f'reboot rejected: {body}')
        except (OSError, http.client.HTTPException) as error:
            emit(target, f'reboot request ended after {time.monotonic() - started:.1f}s: '
                 f'{type(error).__name__}: {error}; restart not yet confirmed')
        if not args.no_wait:
            wait_for_reboot(target, uptime, started, auth, args.timeout, args.reboot_timeout)
        else:
            emit(target, 'reboot requested; not waiting for confirmation')
        return 0
    except (FlashError, OSError, http.client.HTTPException) as error:
        emit(target, f'error: {error}', error=True)
        return 1


def main(argv=None):
    global MULTI_OUTPUT
    OUTPUT_CONTEXT.value = OutputContext()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('targets', nargs='*', default=['airbridge'], metavar='TARGET',
                        help='host/IP or HTTP(S) URL; multiple devices upload in parallel')
    parser.add_argument('-e', '--env', help='PlatformIO environment (default: autodetect)')
    parser.add_argument('-f', '--file', type=Path, help='ESP application .bin or .bin.zlib')
    parser.add_argument('--compress', nargs='?', const='zlib', default='auto',
                        choices=('auto', 'zlib', 'none'),
                        help='transport compression: auto detects support, zlib forces compression, none sends plain')
    parser.add_argument('--build', action='store_true', help='build selected/detected environments first')
    parser.add_argument('-u', '--user', default=os.environ.get('AIRBRIDGE_HTTP_USER', 'admin'))
    parser.add_argument('-p', '--password', default=os.environ.get('AIRBRIDGE_HTTP_PASSWORD', 'airbridge'))
    parser.add_argument('--no-auth', action='store_true')
    parser.add_argument('--no-wait', action='store_true', help='request reboot without waiting for confirmation')
    parser.add_argument('--no-reboot', action='store_true', help='stage firmware without rebooting')
    parser.add_argument('--timeout', type=float, default=30, help='HTTP socket timeout in seconds')
    parser.add_argument('--reboot-timeout', type=float, default=90)
    parser.add_argument('--chunk-size', type=int, default=16384)
    args = parser.parse_args(argv)
    if args.file and args.build:
        parser.error('--file and --build are mutually exclusive')
    if min(args.timeout, args.reboot_timeout, args.chunk_size) <= 0:
        parser.error('timeouts and chunk size must be positive')
    try:
        targets = list(dict.fromkeys(parse_target(value) for value in args.targets))
        auth = None if args.no_auth else auth_header(args.user, args.password)
        environments = {}
        encodings = {}
        # Resolve all targets and validate all images before starting any upload.
        for target in targets:
            request_json(target, 'GET', '/api/status', auth, args.timeout)
            environment = None
            metadata = request_json(target, 'GET', '/api/ota', auth, args.timeout,
                                    allow_missing=True)
            blocked = (metadata or {}).get('upload_blocked')
            if blocked:
                raise FlashError(f'{target_label(target)}: upload blocked: {blocked}')
            supported = (metadata or {}).get('upload_encodings', [])
            use_zlib = args.compress == 'zlib' or (
                args.compress == 'auto' and isinstance(supported, list) and 'zlib' in supported)
            encodings[target] = 'zlib' if use_zlib else 'plain'
            if args.file is None:
                environment = resolve_environment(args.env, metadata)
                emit(target, f'{target_label(target)}: environment {environment}')
            environments[target] = environment
        if args.build:
            for environment in sorted(set(environments.values())):
                emit(None, f'building PlatformIO env {environment}...')
                subprocess.run(['pio', 'run', '-e', environment], cwd=PROJECT_DIR, check=True)
        images = {}
        reported = set()
        for target, environment in environments.items():
            path = args.file or PROJECT_DIR / '.pio' / 'build' / environment / 'firmware.bin'
            images[target] = load_firmware(path)
            if path not in reported:
                emit(None, f'firmware: {path} ({format_bytes(len(images[target]))})')
                reported.add(path)
        if len(targets) > 1:
            MULTI_OUTPUT = MultiTargetOutput([target_label(target) for target in targets])
            with OUTPUT_LOCK:
                MULTI_OUTPUT.start()
        try:
            with ThreadPoolExecutor(max_workers=min(8, len(targets))) as pool:
                results = list(pool.map(lambda target: flash_target(
                    target, images[target], args, auth, encodings[target]), targets))
        finally:
            MULTI_OUTPUT = None
        if len(targets) > 1:
            emit(None, f'completed: {sum(result == 0 for result in results)}/{len(targets)} targets')
        return max(results, default=0)
    except (FlashError, OSError, http.client.HTTPException, subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\nInterrupted; an in-progress upload may have completed. Check the device before retrying.', file=sys.stderr)
        sys.exit(130)
