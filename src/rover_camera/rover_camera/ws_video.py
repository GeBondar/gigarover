"""Minimal RFC 6455 WebSocket transport for the camera video stream.

Implemented with the standard library only: the camera node runs a tiny
WebSocket server that broadcasts JPEG frames (binary messages) plus JSON
metadata (text messages); the topic republisher and the browser connect
as clients. Slow clients never block the capture loop: every client has a
single "latest frame" slot and stale frames are dropped.
"""

from __future__ import annotations

import base64
import hashlib
import os
import select
import socket
import struct
import threading
from typing import Callable, Optional
from urllib.parse import urlparse

WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

OP_CONT = 0x0
OP_TEXT = 0x1
OP_BINARY = 0x2
OP_CLOSE = 0x8
OP_PING = 0x9
OP_PONG = 0xA


def encode_ws_frame(opcode: int, payload: bytes, *, mask: bool = False) -> bytes:
    header = bytearray([0x80 | (opcode & 0x0F)])
    length = len(payload)
    mask_bit = 0x80 if mask else 0x00
    if length < 126:
        header.append(mask_bit | length)
    elif length < 65536:
        header.append(mask_bit | 126)
        header += struct.pack('!H', length)
    else:
        header.append(mask_bit | 127)
        header += struct.pack('!Q', length)

    if not mask:
        return bytes(header) + payload

    mask_key = os.urandom(4)
    header += mask_key
    masked = bytearray(payload)
    for index in range(len(masked)):
        masked[index] ^= mask_key[index % 4]
    return bytes(header) + bytes(masked)


def _read_exact(sock: socket.socket, count: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < count:
        chunk = sock.recv(count - len(chunks))
        if not chunk:
            raise ConnectionError('WebSocket connection closed')
        chunks += chunk
    return bytes(chunks)


def read_ws_message(sock: socket.socket) -> tuple[int, bytes]:
    """Read one complete (possibly fragmented) WebSocket message."""
    message_opcode: Optional[int] = None
    payload = bytearray()
    while True:
        first, second = _read_exact(sock, 2)
        fin = bool(first & 0x80)
        opcode = first & 0x0F
        masked = bool(second & 0x80)
        length = second & 0x7F
        if length == 126:
            (length,) = struct.unpack('!H', _read_exact(sock, 2))
        elif length == 127:
            (length,) = struct.unpack('!Q', _read_exact(sock, 8))
        mask_key = _read_exact(sock, 4) if masked else b''
        data = bytearray(_read_exact(sock, length)) if length else bytearray()
        if masked:
            for index in range(len(data)):
                data[index] ^= mask_key[index % 4]

        if opcode in (OP_CLOSE, OP_PING, OP_PONG):
            # Control frames are never fragmented and may interleave.
            return opcode, bytes(data)

        if opcode != OP_CONT:
            message_opcode = opcode
            payload = data
        else:
            payload += data
        if fin:
            return message_opcode if message_opcode is not None else OP_BINARY, bytes(payload)


class _ServerClient:
    def __init__(self, sock: socket.socket, address: tuple) -> None:
        self.sock = sock
        self.address = address
        self.condition = threading.Condition()
        self.pending_binary: Optional[bytes] = None
        self.pending_texts: list[str] = []
        self.closed = False

    def offer(self, binary: Optional[bytes], texts: Optional[list[str]] = None) -> None:
        with self.condition:
            if binary is not None:
                self.pending_binary = binary
            if texts:
                self.pending_texts.extend(texts)
            self.condition.notify()

    def close(self) -> None:
        with self.condition:
            self.closed = True
            self.condition.notify()
        try:
            self.sock.close()
        except OSError:
            pass


class WebSocketVideoServer:
    """Broadcast-only WebSocket server for JPEG frames + JSON metadata."""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        on_client_connect: Optional[Callable[[], Optional[str]]] = None,
        log: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.host = host
        self.port = port
        self.on_client_connect = on_client_connect
        self.log = log or (lambda text: None)
        self._clients: list[_ServerClient] = []
        self._clients_lock = threading.Lock()
        self._listen_socket: Optional[socket.socket] = None
        self._shutdown = threading.Event()
        self._accept_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.port))
        listener.listen(8)
        self._listen_socket = listener
        self._accept_thread = threading.Thread(
            target=self._accept_loop,
            name='ws-video-accept',
            daemon=True,
        )
        self._accept_thread.start()

    def stop(self) -> None:
        self._shutdown.set()
        if self._listen_socket is not None:
            try:
                self._listen_socket.close()
            except OSError:
                pass
        with self._clients_lock:
            clients = list(self._clients)
            self._clients.clear()
        for client in clients:
            client.close()

    def client_count(self) -> int:
        with self._clients_lock:
            return len(self._clients)

    def broadcast_frame(self, jpeg: bytes) -> None:
        with self._clients_lock:
            clients = list(self._clients)
        for client in clients:
            client.offer(jpeg)

    def broadcast_text(self, text: str) -> None:
        with self._clients_lock:
            clients = list(self._clients)
        for client in clients:
            client.offer(None, [text])

    def _accept_loop(self) -> None:
        while not self._shutdown.is_set():
            try:
                sock, address = self._listen_socket.accept()
            except OSError:
                return
            threading.Thread(
                target=self._handle_client,
                args=(sock, address),
                name='ws-video-client',
                daemon=True,
            ).start()

    def _handshake(self, sock: socket.socket) -> bool:
        sock.settimeout(5.0)
        request = bytearray()
        while b'\r\n\r\n' not in request:
            chunk = sock.recv(4096)
            if not chunk:
                return False
            request += chunk
            if len(request) > 16384:
                return False

        headers: dict[str, str] = {}
        for line in request.split(b'\r\n')[1:]:
            if b':' in line:
                name, _, value = line.partition(b':')
                headers[name.decode('latin-1').strip().lower()] = (
                    value.decode('latin-1').strip()
                )

        key = headers.get('sec-websocket-key')
        if not key or 'websocket' not in headers.get('upgrade', '').lower():
            sock.sendall(b'HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n')
            return False

        accept = base64.b64encode(
            hashlib.sha1((key + WS_GUID).encode('ascii')).digest()
        ).decode('ascii')
        sock.sendall(
            (
                'HTTP/1.1 101 Switching Protocols\r\n'
                'Upgrade: websocket\r\n'
                'Connection: Upgrade\r\n'
                f'Sec-WebSocket-Accept: {accept}\r\n\r\n'
            ).encode('ascii')
        )
        return True

    def _handle_client(self, sock: socket.socket, address: tuple) -> None:
        try:
            if not self._handshake(sock):
                sock.close()
                return
        except (OSError, ValueError):
            try:
                sock.close()
            except OSError:
                pass
            return

        sock.settimeout(None)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        client = _ServerClient(sock, address)
        if self.on_client_connect is not None:
            try:
                greeting = self.on_client_connect()
                if greeting:
                    client.pending_texts.append(greeting)
            except Exception:
                pass
        with self._clients_lock:
            self._clients.append(client)
        self.log(f'WebSocket viewer connected: {address[0]}:{address[1]}')

        try:
            self._client_loop(client)
        finally:
            with self._clients_lock:
                if client in self._clients:
                    self._clients.remove(client)
            client.close()
            self.log(f'WebSocket viewer disconnected: {address[0]}:{address[1]}')

    def _client_loop(self, client: _ServerClient) -> None:
        while not self._shutdown.is_set() and not client.closed:
            with client.condition:
                if client.pending_binary is None and not client.pending_texts:
                    client.condition.wait(timeout=0.25)
                binary = client.pending_binary
                texts = client.pending_texts
                client.pending_binary = None
                client.pending_texts = []

            try:
                for text in texts:
                    client.sock.sendall(
                        encode_ws_frame(OP_TEXT, text.encode('utf-8'))
                    )
                if binary is not None:
                    client.sock.sendall(encode_ws_frame(OP_BINARY, binary))
            except OSError:
                return

            # Answer control frames without blocking the sender.
            try:
                while True:
                    readable, _, _ = select.select([client.sock], [], [], 0)
                    if not readable:
                        break
                    opcode, payload = read_ws_message(client.sock)
                    if opcode == OP_CLOSE:
                        return
                    if opcode == OP_PING:
                        client.sock.sendall(encode_ws_frame(OP_PONG, payload))
            except (OSError, ConnectionError):
                return


class WebSocketVideoClient:
    """Blocking WebSocket client used by the ROS topic republisher."""

    def __init__(self, url: str, *, connect_timeout: float = 3.0) -> None:
        parsed = urlparse(url)
        if parsed.scheme not in ('ws', ''):
            raise ValueError(f'Unsupported WebSocket URL: {url}')
        self.host = parsed.hostname or '127.0.0.1'
        self.port = parsed.port or 80
        self.path = parsed.path or '/'
        self.connect_timeout = connect_timeout
        self.sock: Optional[socket.socket] = None

    def connect(self) -> None:
        sock = socket.create_connection(
            (self.host, self.port),
            timeout=self.connect_timeout,
        )
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            key = base64.b64encode(os.urandom(16)).decode('ascii')
            sock.sendall(
                (
                    f'GET {self.path} HTTP/1.1\r\n'
                    f'Host: {self.host}:{self.port}\r\n'
                    'Upgrade: websocket\r\n'
                    'Connection: Upgrade\r\n'
                    f'Sec-WebSocket-Key: {key}\r\n'
                    'Sec-WebSocket-Version: 13\r\n\r\n'
                ).encode('ascii')
            )
            response = bytearray()
            while b'\r\n\r\n' not in response:
                chunk = sock.recv(4096)
                if not chunk:
                    raise ConnectionError('WebSocket handshake failed: closed')
                response += chunk
                if len(response) > 16384:
                    raise ConnectionError('WebSocket handshake failed: oversized reply')
            status_line = bytes(response).split(b'\r\n', 1)[0]
            if b'101' not in status_line:
                raise ConnectionError(
                    f'WebSocket handshake rejected: {status_line.decode("latin-1")}'
                )

            expected = base64.b64encode(
                hashlib.sha1((key + WS_GUID).encode('ascii')).digest()
            ).decode('ascii')
            if expected.encode('ascii') not in bytes(response):
                raise ConnectionError('WebSocket handshake failed: bad accept key')
        except Exception:
            sock.close()
            raise
        self.sock = sock

    def receive(self, timeout: float = 2.0) -> tuple[int, bytes]:
        """Return the next data message, transparently answering pings."""
        if self.sock is None:
            raise ConnectionError('WebSocket is not connected')
        self.sock.settimeout(timeout)
        while True:
            opcode, payload = read_ws_message(self.sock)
            if opcode == OP_PING:
                self.sock.sendall(encode_ws_frame(OP_PONG, payload, mask=True))
                continue
            if opcode == OP_PONG:
                continue
            if opcode == OP_CLOSE:
                raise ConnectionError('WebSocket closed by server')
            return opcode, payload

    def close(self) -> None:
        if self.sock is None:
            return
        try:
            self.sock.sendall(encode_ws_frame(OP_CLOSE, b'', mask=True))
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
        self.sock = None
