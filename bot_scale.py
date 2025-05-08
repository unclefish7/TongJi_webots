#!/usr/bin/env python3
"""
scale_proto.py  ——  批量放大 Webots PROTO 模型

用法:
    python3 scale_proto.py

生成:
    tongji_webot_project/protos/Scaled.proto
"""

import re
from pathlib import Path

# --------------------------------------------------
# 自定义规则区
SCALE = 2.0                # 线性放大倍数
MASS_SCALE = SCALE ** 3    # mass ≈ 体积比例
SRC = Path("tongji_webot_project/protos/TurtleBot3Burger.proto")
DST = SRC.with_name("Scaled.proto")
# --------------------------------------------------

float_re = re.compile(r"(-?\d+\.\d+(?:e[+-]?\d+)?)", re.IGNORECASE)

def scaled_number(match: re.Match, factor: float) -> str:
    num = float(match.group(0))
    return f"{num * factor:.6g}"

def main() -> None:
    if not SRC.exists():
        raise FileNotFoundError(f"输入文件不存在: {SRC}")

    text = SRC.read_text()

    # 1) mass 专用放大
    def mass_repl(m):
        return f"mass {float(m.group(1)) * MASS_SCALE:.6g}"
    text = re.sub(r"mass\s+(-?\d+\.\d+(?:e[+-]?\d+)?)", mass_repl, text)

    # 2) 其它所有浮点统一放大
    def generic_repl(m):
        return scaled_number(m, SCALE)
    text = float_re.sub(generic_repl, text)

    # 3) 顶部加注释
    header = (
        f"# THIS FILE WAS AUTO-GENERATED.\n"
        f"# Original: {SRC.name}\n"
        f"# Uniform scale applied: ×{SCALE}\n\n"
    )
    DST.write_text(header + text)
    print(f"✓ 已生成放大后的 PROTO: {DST}")

if __name__ == "__main__":
    main()
