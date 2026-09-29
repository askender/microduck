#!/bin/bash
# tunnel_watch.sh — keep the two board tunnels alive:
#   ssh -R 7801:127.0.0.1:7802   board robotd -> Mac MITM -> body_server
#   ssh -L duck-a.sock:/root/duck.sock   planview buttons/odom -> board robotd
# Auto-reconnects whenever the board is reachable over adb; the board's
# robotd re-associates with the sim on its own after a tunnel blip.
# Canonical copy: scripts/board-bridge/ in the microduck repo (state-dir paths
# are hardcoded, so it runs identically from either location).
STATE="$HOME/.cache/duck-sim"
SOCK="$STATE/duck-a.sock"
LOG="$STATE/tunnel_watch.log"
mkdir -p "$STATE"

log() { echo "$(date '+%H:%M:%S') $*" >> "$LOG"; }

while true; do
    if ! adb devices 2>/dev/null | grep -q $'\tdevice$'; then
        sleep 5
        continue
    fi
    # board is on USB; re-arm adb forward (USB replug clears forward rules),
    # then wait for its sshd (Mac 2222 -> board:22)
    adb forward tcp:2222 tcp:22 >/dev/null 2>&1
    ready=0
    for _ in $(seq 1 60); do
        if nc -z 127.0.0.1 2222 2>/dev/null; then ready=1; break; fi
        sleep 1
    done
    [ "$ready" = 1 ] || { log "board on USB but sshd:2222 never came up"; sleep 5; continue; }

    log "board reachable — (re)starting tunnels"
    rm -f "$SOCK"
    ssh -N -R 7801:127.0.0.1:7802 -o BatchMode=yes \
        -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
        -o ExitOnForwardFailure=yes ti >>"$LOG" 2>&1 &
    R_PID=$!
    ssh -N -L "$SOCK":/root/duck.sock -o BatchMode=yes \
        -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
        -o ExitOnForwardFailure=yes ti >>"$LOG" 2>&1 &
    L_PID=$!
    sleep 2
    if kill -0 $R_PID 2>/dev/null && kill -0 $L_PID 2>/dev/null; then
        log "tunnels up (R=$R_PID L=$L_PID)"
        echo "TUNNELS_UP $(date '+%H:%M:%S')"
    else
        log "tunnel ssh failed to start"
    fi
    # hold here until one tunnel dies, then loop and re-establish
    while kill -0 $R_PID 2>/dev/null && kill -0 $L_PID 2>/dev/null; do
        sleep 5
    done
    log "a tunnel died — waiting for board again"
    wait $R_PID $L_PID 2>/dev/null
done
