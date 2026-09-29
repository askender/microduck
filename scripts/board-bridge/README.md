# board-bridge — Mac-side tools for the board-in-the-loop sim

The Taishan Pi runs the duck's brain (`robotd --sim`); this Mac runs the body
(`body_server`, MuJoCo). These three tools are the glue. At runtime they live
in `~/.cache/duck-sim/` — the copies here are **canonical**: the tools hardcode
their state-dir paths, so they run identically from either location, and
`restart-all.sh` launches these copies directly. (`~/.cache/duck-sim/restart-all.sh`
is a two-line shim that execs this one, so the old path keeps working.)

    board robotd (brain)                                              Mac
    dials 127.0.0.1:7801 ──ssh -R──> mitm.py :7802 ──> body_server :7801 (MuJoCo)
    Mac planview :8910 ──dial duck-a.sock ──ssh -L──> board /root/duck.sock

    both tunnels are held up by tunnel_watch.sh, over `adb forward tcp:2222 tcp:22`

## mitm.py — the tap

Two layers; do not confuse them:

1. **The process is always in the path.** Board traffic flows through it
   (robotd → :7802 → :7801) whether or not anything is recorded. Pass-through
   is transparent — the duck cannot tell, and toggling never disturbs it.
2. **Recording is gated by a flag file.** Lines are appended to
   `~/.cache/duck-sim/mitm.log` only while `~/.cache/duck-sim/mitm.on` exists.
   Default state: **off** (the flag is absent; a fresh restart never creates it).

        touch ~/.cache/duck-sim/mitm.on    # start recording
        rm    ~/.cache/duck-sim/mitm.on    # stop recording

The log grows ~2.5 MB/min while on — turn it off when done. Line format:
`C>` prefixes board→sim traffic (write targets, gain, torque), `S>` prefixes
sim→board SensorFrames (positions / velocities / currents, IMU), one JSON
object per line, in wire order.

## tunnel_watch.sh — the tunnels' keeper

Holds both ssh tunnels with ServerAlive keepalives and rebuilds them on death.
It also re-arms `adb forward tcp:2222 tcp:22` — **a USB replug clears adb
forward rules**, so after unplugging the board, plugging it back in is enough:
the watchdog notices, re-arms the forward, rebuilds the tunnels, and the
board's robotd re-associates with the sim on its own. If the board lost *power*
(not just USB), robotd is gone too — that is what `restart-all.sh` is for.

## restart-all.sh — one-command clean restart

Order: stop board brain → stop Mac side → re-arm adb forward → body (SIT
keyframe) → mitm + watchdog → planview → board brain → health verify. The duck
comes up seated; start the brain from the planview page (the brain button) or
`robot.enable`. Two non-obvious requirements, see the code comments for why:
robotd must be started over `ssh -f`, and that ssh must have its own stdio
redirected to `/dev/null` by the caller.

## Where the rest of it lives

- `scripts/planview.py`, `scripts/wander.py` — sources for the planview page
  and the wander client; the duck-sim launcher installs them into the state
  dir on use (a newer state-dir copy wins, so live hand edits survive).
- `scene_home/` in the state dir — generated MuJoCo scene (mostly symlinks
  into the RL repo); regenerable, deliberately not committed.
- Sockets, pids and logs in the state dir are runtime state, never committed.
