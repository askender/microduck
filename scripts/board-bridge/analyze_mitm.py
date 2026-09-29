#!/usr/bin/env python3
"""分析 mitm.log：body_server 发了几种消息、收了几种消息。

mitm 一行一条 JSON，前缀标线上的方向（见 scripts/board-bridge/README.md）：
  C>  client（板子 robotd）→ body_server   —— 即 body_server **收到**
  S>  body_server → client                 —— 即 body_server **发出**

"种类"的判别：优先用 JSON 里的判别字段（op / type / msg）；
消息没有判别字段时（如 SensorFrame），退回用键集合（形状）区分。
"""
import json
import re
import sys
from collections import Counter

LINE = re.compile(r"^\s*\d+\s+([CS])>\s+(\{.*\})$")

LOG = sys.argv[1] if len(sys.argv) > 1 else "/Users/askender/.cache/duck-sim/mitm.log"
EXAMPLE_WIDTH = 100


def kind_of(obj):
    """一条消息的种类：判别字段优先，否则按形状（键集合）。"""
    if isinstance(obj, dict):
        for key in ("op", "type", "msg"):
            if key in obj:
                return f"{key}={obj[key]!r}"
        return "{" + ",".join(sorted(obj)) + "}"
    return f"<非对象: {type(obj).__name__}>"


def truncate(text, width=EXAMPLE_WIDTH):
    return text if len(text) <= width else text[: width - 1] + "…"


sent = Counter()   # S> — body_server 发出
recv = Counter()   # C> — body_server 收到
examples = {}      # (方向, 种类) -> 第一条样例
bad = Counter()    # 解析不了的行
total = 0

with open(LOG, encoding="utf-8", errors="replace") as f:
    for raw in f:
        line = raw.strip()
        if not line:
            continue
        total += 1
        m = LINE.match(line)
        if not m:
            bad["不匹配 `行号 [CS]> JSON`"] += 1
            continue
        prefix, payload = m.group(1) + ">", m.group(2)
        try:
            obj = json.loads(payload)
        except json.JSONDecodeError:
            bad[f"{prefix} 非JSON"] += 1
            continue
        kind = kind_of(obj)
        (sent if prefix == "S>" else recv)[kind] += 1
        examples.setdefault((prefix, kind), truncate(payload))

print(f"日志 {LOG}：共 {total} 行\n")
for title, counter, prefix in (
        (f"body_server 发出（S>）—— {len(sent)} 种", sent, "S>"),
        (f"body_server 收到（C>）—— {len(recv)} 种", recv, "C>")):
    print(f"== {title} ==")
    for kind, n in counter.most_common():
        print(f"  {n:7d}  {kind}")
        print(f"          例: {examples[(prefix, kind)]}")
    print()
if bad:
    print("== 解析失败的行 ==")
    for why, n in bad.most_common():
        print(f"  {n:7d}  {why}")
