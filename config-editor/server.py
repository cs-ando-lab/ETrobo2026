#!/usr/bin/env python3
"""Config.hの数値定数を編集するローカルWebサーバー。"""

from __future__ import annotations

import argparse
import ast
import json
import math
import re
import tempfile
import webbrowser
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


EDITOR_DIR = Path(__file__).resolve().parent
REPOSITORY_DIR = EDITOR_DIR.parent
STATIC_DIR = EDITOR_DIR / "static"
CONFIG_JSON = EDITOR_DIR / "config.json"
CONFIG_HEADER = REPOSITORY_DIR / "main" / "app" / "Config.h"
HOST = "127.0.0.1"
PORT = 8080

DECLARATION_PATTERN = re.compile(
    r"^\s*static\s+constexpr\s+"
    r"(?P<type>[A-Za-z_][\w:]*)\s+"
    r"(?P<name>[A-Z][A-Z0-9_]*)\s*=\s*"
    r"(?P<expression>[^;]+);"
    r"(?:\s*//\s*(?P<comment>.*))?$"
)
SECTION_PATTERN = re.compile(r"^\s*//\s*──\s*(?P<name>.+?)(?:─{2,}.*)?$")
INTEGER_RANGES = {
    "int8_t": (-128, 127),
    "uint8_t": (0, 255),
    "int16_t": (-32_768, 32_767),
    "uint16_t": (0, 65_535),
    "int32_t": (-2_147_483_648, 2_147_483_647),
    "uint32_t": (0, 4_294_967_295),
}
GATE_PREFIXES = {
    "red": "ETRALLY_RED_GATE",
    "blue": "ETRALLY_BLUE_GATE",
    "yellow": "ETRALLY_YELLOW_GATE",
}
GATE_LABELS = {"red": "赤ゲート", "blue": "青ゲート", "yellow": "黄ゲート"}
GATE_ORIENTATIONS = {"red": "horizontal", "blue": "vertical", "yellow": "horizontal"}


def evaluate_number(expression: str) -> int | float:
    """四則演算だけを許可してC++の数値式を評価する。"""
    normalized = re.sub(r"(?<=\d)[fF]\b", "", expression.strip())
    tree = ast.parse(normalized, mode="eval")

    def evaluate(node: ast.AST) -> int | float:
        if isinstance(node, ast.Expression):
            return evaluate(node.body)
        if isinstance(node, ast.Constant) and type(node.value) in (int, float):
            return node.value
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            value = evaluate(node.operand)
            return value if isinstance(node.op, ast.UAdd) else -value
        if isinstance(node, ast.BinOp) and isinstance(
            node.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)
        ):
            left, right = evaluate(node.left), evaluate(node.right)
            if isinstance(node.op, ast.Add):
                return left + right
            if isinstance(node.op, ast.Sub):
                return left - right
            if isinstance(node.op, ast.Mult):
                return left * right
            return left / right
        raise ValueError(f"未対応の数値式です: {expression}")

    value = evaluate(tree)
    if not math.isfinite(value):
        raise ValueError(f"有限の数値ではありません: {expression}")
    return value


def parse_header() -> list[dict]:
    settings = []
    category = "その他"
    for line in CONFIG_HEADER.read_text(encoding="utf-8").splitlines():
        section = SECTION_PATTERN.match(line)
        if section:
            category = section.group("name").strip(" ─")
            continue
        declaration = DECLARATION_PATTERN.match(line)
        if not declaration:
            continue
        cpp_type = declaration.group("type")
        value = evaluate_number(declaration.group("expression"))
        if cpp_type != "float":
            if isinstance(value, float) and not value.is_integer():
                raise ValueError(f"{declaration.group('name')}は整数型ですが小数です")
            value = int(value)
        else:
            value = float(value)
        settings.append(
            {
                "name": declaration.group("name"),
                "type": cpp_type,
                "value": value,
                "category": category,
                "description": (declaration.group("comment") or "").strip(),
            }
        )
    if not settings:
        raise RuntimeError("Config.hに編集可能な定数が見つかりません")
    return settings


def read_json() -> dict:
    with CONFIG_JSON.open(encoding="utf-8") as file:
        return json.load(file)


def write_json(config: dict) -> None:
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=CONFIG_JSON.parent, delete=False
    ) as file:
        json.dump(config, file, ensure_ascii=False, indent=2)
        file.write("\n")
        temporary_path = Path(file.name)
    temporary_path.replace(CONFIG_JSON)


def legacy_gate_values(config: dict) -> dict[str, int]:
    values = {}
    gates = config.get("gates")
    if not isinstance(gates, dict):
        return values
    for color, prefix in GATE_PREFIXES.items():
        gate = gates.get(color)
        if not isinstance(gate, dict):
            continue
        for side in ("left", "right"):
            point = gate.get(side)
            if not isinstance(point, dict):
                continue
            for axis in ("row", "col"):
                value = point.get(axis)
                if type(value) is int:
                    values[f"{prefix}_{side.upper()}_{axis.upper()}"] = value
    return values


def initialize_config() -> dict:
    """Config.hの構造とJSONの値をマージし、両ファイルを同期する。"""
    settings = parse_header()
    saved_values: dict[str, int | float] = {}
    if CONFIG_JSON.exists():
        try:
            existing = read_json()
            if isinstance(existing.get("settings"), list):
                saved_values = {
                    setting["name"]: setting["value"]
                    for setting in existing["settings"]
                    if isinstance(setting, dict)
                    and isinstance(setting.get("name"), str)
                    and type(setting.get("value")) in (int, float)
                }
            else:
                saved_values = legacy_gate_values(existing)
        except (OSError, json.JSONDecodeError):
            saved_values = {}

    for setting in settings:
        if setting["name"] in saved_values:
            setting["value"] = saved_values[setting["name"]]

    config = validate_values(
        {setting["name"]: setting["value"] for setting in settings}, settings
    )
    update_header(config["settings"])
    write_json(config)
    return config


def validate_values(values: object, schema: list[dict] | None = None) -> dict:
    if not isinstance(values, dict):
        raise ValueError("valuesがありません")
    schema = schema or read_json()["settings"]
    expected_names = {setting["name"] for setting in schema}
    if set(values) != expected_names:
        missing = expected_names - set(values)
        extra = set(values) - expected_names
        details = []
        if missing:
            details.append(f"不足: {', '.join(sorted(missing))}")
        if extra:
            details.append(f"不明: {', '.join(sorted(extra))}")
        raise ValueError("設定項目が一致しません（" + " / ".join(details) + "）")

    normalized = []
    for setting in schema:
        value = values[setting["name"]]
        cpp_type = setting["type"]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"{setting['name']}には有限の数値を指定してください")
        if cpp_type != "float":
            if isinstance(value, float) and not value.is_integer():
                raise ValueError(f"{setting['name']}には整数を指定してください")
            value = int(value)
            minimum, maximum = INTEGER_RANGES.get(
                cpp_type, (-2_147_483_648, 2_147_483_647)
            )
            if not minimum <= value <= maximum:
                raise ValueError(
                    f"{setting['name']}は{minimum}〜{maximum}の範囲で指定してください"
                )
        else:
            value = float(value)
        normalized.append({**setting, "value": value})

    validate_gates({setting["name"]: setting["value"] for setting in normalized})
    return {"version": 1, "settings": normalized}


def validate_gates(values: dict[str, int | float]) -> None:
    occupied: dict[tuple[int | float, int | float], str] = {}
    for color, prefix in GATE_PREFIXES.items():
        coordinates = {
            side: {
                axis: values[f"{prefix}_{side.upper()}_{axis.upper()}"]
                for axis in ("row", "col")
            }
            for side in ("left", "right")
        }
        for point in coordinates.values():
            if not 1 <= point["row"] <= 5 or not 1 <= point["col"] <= 5:
                raise ValueError(f"{GATE_LABELS[color]}のrowとcolは1〜5で指定してください")
            position = (point["row"], point["col"])
            if position in occupied:
                row, col = position
                raise ValueError(f"row {row}, col {col}でゲート同士が重なっています")
            occupied[position] = color
        left, right = coordinates["left"], coordinates["right"]
        distance = abs(left["row"] - right["row"]) + abs(left["col"] - right["col"])
        if distance != 1:
            raise ValueError(f"{GATE_LABELS[color]}の脚は隣り合うマスに配置してください")
        if GATE_ORIENTATIONS[color] == "horizontal" and left["row"] != right["row"]:
            raise ValueError(f"{GATE_LABELS[color]}は横向きに配置してください")
        if GATE_ORIENTATIONS[color] == "vertical" and left["col"] != right["col"]:
            raise ValueError(f"{GATE_LABELS[color]}は縦向きに配置してください")


def format_cpp_value(cpp_type: str, value: int | float) -> str:
    if cpp_type == "float":
        literal = format(float(value), ".9g")
        if "." not in literal and "e" not in literal.lower():
            literal += ".0"
        return literal + "f"
    return str(int(value))


def update_header(settings: list[dict]) -> None:
    source = CONFIG_HEADER.read_text(encoding="utf-8")
    for setting in settings:
        name = setting["name"]
        pattern = re.compile(
            rf"(^\s*static\s+constexpr\s+{re.escape(setting['type'])}\s+"
            rf"{re.escape(name)}\s*=\s*)([^;]+)(;.*$)",
            re.MULTILINE,
        )
        replacement = format_cpp_value(setting["type"], setting["value"])
        source, count = pattern.subn(
            lambda match, value=replacement, expected=setting["value"]: (
                match.group(0)
                if evaluate_number(match.group(2)) == expected
                else f"{match.group(1)}{value}{match.group(3)}"
            ),
            source,
        )
        if count != 1:
            raise RuntimeError(f"Config.h内の{name}が一意に見つかりません")

    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=CONFIG_HEADER.parent, delete=False
    ) as file:
        file.write(source)
        temporary_path = Path(file.name)
    temporary_path.replace(CONFIG_HEADER)


def save_values(values: object) -> dict:
    config = validate_values(values)
    update_header(config["settings"])
    write_json(config)
    return config


class RequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def send_json(self, data: object, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path == "/api/config":
            try:
                self.send_json(read_json())
            except (OSError, json.JSONDecodeError) as error:
                self.send_json({"error": str(error)}, HTTPStatus.INTERNAL_SERVER_ERROR)
            return
        super().do_GET()

    def do_POST(self) -> None:
        if self.path != "/api/config":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 65_536:
                raise ValueError("リクエストサイズが不正です")
            submitted = json.loads(self.rfile.read(length))
            if not isinstance(submitted, dict):
                raise ValueError("JSONオブジェクトを送信してください")
            config = save_values(submitted.get("values"))
            self.send_json({"message": "Config.hとconfig.jsonを更新しました", **config})
        except (ValueError, KeyError, json.JSONDecodeError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
        except (OSError, RuntimeError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def log_message(self, format: str, *args: object) -> None:
        print(f"[config-editor] {format % args}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sync-only", action="store_true", help="JSONとConfig.hを同期して終了する"
    )
    args = parser.parse_args()
    config = initialize_config()
    print(f"{len(config['settings'])}件の設定を同期しました")
    if args.sync_only:
        return

    server = ThreadingHTTPServer((HOST, PORT), RequestHandler)
    url = f"http://{HOST}:{PORT}"
    print(f"Config Editor: {url}")
    print("終了するには Ctrl+C を押してください")
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nConfig Editorを終了します")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
