#!/bin/bash
# restart-all.sh — clean restart of the whole board-topology duck stack.
#
# This is the canonical copy (scripts/board-bridge/ in the microduck repo);
# ~/.cache/duck-sim/restart-all.sh is a shim that execs it. mitm.py and
# tunnel_watch.sh hardcode their state-dir paths, so running them from here
# is identical to running the old copies in ~/.cache/duck-sim/.
#
# Topology: MuJoCo body on this Mac (port 7801, boots at the SIT keyframe),
# MITM tap in front of it (7802), two ssh tunnels to the Taishan Pi
# (watchdog-managed, see tunnel_watch.sh), robotd — the brain — on the board,
# planview web page on 8910.
#
# Order: stop board brain -> stop Mac side -> re-arm adb forward -> body ->
# mitm + watchdog -> planview -> board brain -> verify. The duck comes up
# seated; start the brain from the page (the brain button) or robot.enable.
#
# If the board dropped off USB, plug it back in first — but note a replug
# clears adb forward rules; step below re-arms it either way.
set -u
STATE="$HOME/.cache/duck-sim"
RL="$HOME/Pollen/microduck_rl"
REPO="$HOME/aix1/taishan/microduck"
BRIDGE="$REPO/scripts/board-bridge"

say() { printf '\033[36m==\033[0m %s\n' "$*"; }

say "stop board brain"
# robot[d] so the pattern cannot match the shell that carries it
ssh -o BatchMode=yes -o ConnectTimeout=5 ti \
    'pkill -f "target/release/robot[d]"; rm -f /root/duck.sock' || true

say "stop Mac side"
pkill -f 'sim\.body_server' || true
# stop copies in either location — the state dir may still hold older ones
pkill -f '(duck-sim|board-bridge)/mitm\.py' || true
pkill -f "$STATE/planview\.py" || true
pkill -f '(duck-sim|board-bridge)/tunnel_watch\.sh' || true
pkill -f 'ssh -N -R 7801' || true
pkill -f 'duck-a\.sock:/root/duck\.sock' || true
rm -f "$STATE/duck-a.sock"
# wait for the body's port to actually free up
for _ in $(seq 1 20); do
    lsof -nP -iTCP:7801 -sTCP:LISTEN >/dev/null 2>&1 || break
    sleep 0.5
done

say "re-arm adb forward (a USB replug clears it)"
adb forward tcp:2222 tcp:22

say "start MuJoCo body on :7801 (SIT keyframe)"
cd "$RL"
nohup .venv/bin/python -m mjlab_microduck.sim.body_server \
    --port 7801 --ducks 1 --keyframe SIT --headless \
    --scene "$STATE/scene_home/scene_home.xml" \
    >> "$STATE/body.log" 2>&1 </dev/null &

say "start MITM tap (:7802) and tunnel watchdog"
nohup python3 "$BRIDGE/mitm.py" >> "$STATE/mitm.run.log" 2>&1 </dev/null &
nohup bash "$BRIDGE/tunnel_watch.sh" >/dev/null 2>&1 </dev/null &

say "start planview — http://localhost:8910/"
# the launcher refuses without the duck socket; the watchdog needs a moment
for _ in $(seq 1 30); do
    [ -S "$STATE/duck-a.sock" ] && break
    sleep 1
done
[ -S "$STATE/duck-a.sock" ] || { say "tunnel socket never appeared — planview skipped"; }
cd "$REPO"
DUCK_SIM_SCENE="$STATE/scene_home/scene_home.xml" DUCK_SIM_REPO="$REPO" \
    scripts/duck-sim planview

say "start board brain"
# -f: ssh goes to background right after auth and returns at once. A plain
# `... nohup robotd ... &` over ssh hangs the client even with stdin/stdout
# redirected — something in robotd keeps the session's teardown waiting.
# The own-stdio redirects matter too: ssh -f dups its session channel onto
# the stdio it inherited, and the daemon keeps those copies — with a raw
# stdout they hold the caller's pipe open forever (grep in a pipeline hangs).
ssh -f -o BatchMode=yes ti \
    'cd /root && nohup /root/microduck/target/release/robotd \
     --sim 127.0.0.1:7801 --socket /root/duck.sock --params /root/robotd.toml \
     >> /root/robotd.log 2>&1 </dev/null &' </dev/null >/dev/null 2>&1

say "verify"
sleep 5
ssh -o BatchMode=yes ti 'python3 /root/board_rpc.py health' || true
tail -1 "$STATE/tunnel_watch.log"
curl --noproxy '*' -s -o /dev/null -w 'planview http %{http_code}\n' http://127.0.0.1:8910/
say "done — the duck is seated; press the brain button on the page to walk"
