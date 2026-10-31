#!/usr/bin/env python3
"""Upload a file to an absolute SD path, without installing firmware.

Example: airbridge_upload.py airbridge image.bin --path /test/image.bin --overwrite
"""

import argparse
import http.client
import os
from pathlib import Path
import sys
from urllib.parse import urlencode

from airbridge_flash import FlashError as UploadError
from airbridge_flash import auth_header, connection, decode_response, parse_target
from airbridge_flash import emit, finish_status, print_progress, update_status


CHUNK_SIZE = 4096
MAX_FILE_BYTES = 0xffffffff


def upload(target, source, destination, overwrite=False, auth=None, timeout=60):
    if not destination.startswith('/') or destination == '/':
        raise UploadError('destination must be an absolute SD file path')
    with Path(source).open('rb') as stream:
        size = os.fstat(stream.fileno()).st_size
        if size > MAX_FILE_BYTES:
            raise UploadError('file exceeds the FAT32 size limit')
        offset, upload_id = 0, None
        if size:
            print_progress(target, 0, size, force=True)
        else:
            update_status(target, 'uploading: 0 B', force=True)
        while True:
            data = stream.read(min(CHUNK_SIZE, size - offset))
            if len(data) != min(CHUNK_SIZE, size - offset):
                raise UploadError('source file became shorter during upload')
            end = offset + len(data)
            final = end == size
            if final and stream.read(1):
                raise UploadError('source file grew during upload')
            query = dict(offset=offset, final=int(final))
            if upload_id is None:
                query.update(path=destination, size=size, overwrite=int(overwrite))
            else:
                query['id'] = upload_id
            conn = connection(target, timeout)
            try:
                headers = {'Content-Type': 'application/octet-stream', 'Accept': 'application/json'}
                if auth:
                    headers['Authorization'] = auth
                conn.request('POST', target.base_path + '/api/upload?' + urlencode(query),
                             body=data, headers=headers)
                result = decode_response(conn.getresponse())
            except (OSError, http.client.HTTPException) as error:
                raise UploadError(f'connection failed: {error}; outcome unknown, not retrying') from error
            finally:
                conn.close()
            received_id = result.get('upload_id')
            if (result.get('ok') is not True or type(received_id) is not int or received_id <= 0 or
                    (upload_id is not None and received_id != upload_id) or
                    type(result.get('written')) is not int or result['written'] != end or
                    result.get('complete') is not final):
                raise UploadError(f'upload confirmation mismatch: {result}')
            upload_id, offset = received_id, end
            if size:
                print_progress(target, offset, size, force=final)
            if final:
                return size


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('host', help='host or HTTP(S) URL')
    parser.add_argument('file', type=Path)
    parser.add_argument('--path', required=True, help='absolute destination path on SD')
    parser.add_argument('--overwrite', action='store_true', help='replace an existing file after upload completes')
    parser.add_argument('-u', '--user', default=os.environ.get('AIRBRIDGE_HTTP_USER', 'admin'))
    parser.add_argument('-p', '--password', default=os.environ.get('AIRBRIDGE_HTTP_PASSWORD', 'airbridge'))
    parser.add_argument('--no-auth', action='store_true')
    parser.add_argument('--timeout', type=float, default=60, help='seconds per chunk response (default: 60)')
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    try:
        size = upload(parse_target(args.host), args.file, args.path, args.overwrite,
                      None if args.no_auth else auth_header(args.user, args.password), args.timeout)
        emit(None, f'Uploaded {size} bytes to {args.path}')
        return 0
    except (UploadError, OSError) as error:
        emit(None, f'ERROR: {error}', error=True)
        return 1
    finally:
        finish_status()


if __name__ == '__main__':
    sys.exit(main())
