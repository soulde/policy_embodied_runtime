from __future__ import annotations

import os
import pty
import select
import termios
import tty
import errno
from dataclasses import dataclass


@dataclass
class VirtualSerialEndpoint:
    """One PTY endpoint for the external controller and one fd for the simulator."""

    fd: int
    path: str

    @classmethod
    def open(cls) -> "VirtualSerialEndpoint":
        master_fd, slave_fd = pty.openpty()
        path = os.ttyname(slave_fd)
        tty.setraw(master_fd)
        tty.setraw(slave_fd)
        attrs = termios.tcgetattr(slave_fd)
        attrs[3] &= ~termios.ECHO
        termios.tcsetattr(slave_fd, termios.TCSANOW, attrs)
        os.close(slave_fd)
        os.set_blocking(master_fd, False)
        return cls(fd=master_fd, path=path)

    def read_available(self, max_bytes: int = 4096) -> bytes:
        readable, _, _ = select.select([self.fd], [], [], 0)
        if not readable:
            return b""
        try:
            return os.read(self.fd, max_bytes)
        except BlockingIOError:
            return b""
        except OSError as error:
            if error.errno == errno.EIO:
                return b""
            raise

    def write(self, data: bytes) -> None:
        if data:
            os.write(self.fd, data)

    def close(self) -> None:
        os.close(self.fd)

    def __enter__(self) -> "VirtualSerialEndpoint":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
