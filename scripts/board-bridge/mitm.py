#!/usr/bin/env python3
"""mitm.py — transparent line-logging proxy between the board's robotd and the
MuJoCo body server (the `--sim` RemoteIo link).

Canonical copy: scripts/board-bridge/ in the microduck repo (paths are
hardcoded to the state dir, so it runs identically from either location).

Mac side. Listens on 127.0.0.1:7802, forwards to 127.0.0.1:7801.

Toggle:  touch ~/.cache/duck-sim/mitm.on   -> start recording
         rm    ~/.cache/duck-sim/mitm.on   -> pass through silently (default off)
Log:     ~/.cache/duck-sim/mitm.log — one line per frame:
         <ms since proxy start> <C> or <S> <json>
         C = client (board robotd) -> server (sim);  S = server -> client.
Both directions of the single persistent connection are logged while the flag
file exists. Recording both directions of a running duck is ~2.5 MB/min — the
flag is meant for short captures, not permanent logging.
"""
import os
import socket
import threading
import time

LISTEN = ("127.0.0.1", 7802)
UPSTREAM = ("127.0.0.1", 7801)
STATE = os.path.expanduser("~/.cache/duck-sim")
FLAG = os.path.join(STATE, "mitm.on")
LOG = os.path.join(STATE, "mitm.log")
T0 = time.time()


def log(direction: str, line: bytes) -> None:
    if not os.path.exists(FLAG):
        return
    try:
        text = line.decode("utf-8", "replace").rstrip("\n")
        with open(LOG, "a") as f:
            f.write(f"{(time.time() - T0) * 1000:12.0f} {direction} {text}\n")
    except OSError:
        pass


def pump(src: socket.socket, dst: socket.socket, direction: str) -> None:
    try:
        while True:
            chunk = src.recv(65536)
            if not chunk:
                break
            start = 0
            while True:
                i = chunk.find(b"\n", start)
                if i < 0:
                    break
                log(direction, chunk[start:i] + b"\n")
                start = i + 1
            dst.sendall(chunk)
    except OSError:
        pass
    finally:
        for s in (src, dst):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def handle(client: socket.socket) -> None:
    try:
        upstream = socket.create_connection(UPSTREAM, timeout=5)
    except OSError:
        client.close()
        return
    threading.Thread(target=pump, args=(client, upstream, "C>"), daemon=True).start()
    threading.Thread(target=pump, args=(upstream, client, "S>"), daemon=True).start()


def main() -> None:
    os.makedirs(STATE, exist_ok=True)
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(LISTEN)
    srv.listen(8)
    print(f"mitm {LISTEN} -> {UPSTREAM}; record while {FLAG} exists -> {LOG}",
          flush=True)
    while True:
        client, _ = srv.accept()
        threading.Thread(target=handle, args=(client,), daemon=True).start()


if __name__ == "__main__":
    main()
