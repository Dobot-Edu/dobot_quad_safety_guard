#!/usr/bin/env python3

from pathlib import Path
from typing import Any
import os

import yaml


def load_yaml(path: str) -> dict[str, Any]:
    """加载 YAML 配置文件；空文件按空字典处理。
    Load a YAML configuration file; treat an empty file as an empty dictionary.
    """
    config_path = Path(path).expanduser()
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    with config_path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def resolve_path(path_value: str | None, config_file: str | None = None) -> str | None:
    """从安装包、源码工作区或 SDK 默认位置解析配置路径。
    Resolve a configuration path from the installed package, source workspace, or default SDK location.
    """
    if not path_value:
        return None

    path = Path(path_value).expanduser()
    if path.is_absolute() and path.exists():
        return str(path)

    candidates: list[Path] = []
    if config_file:
        cfg_path = Path(config_file).expanduser()
        if cfg_path.exists():
            candidates.append((cfg_path.parent / path).resolve())

    cwd = Path.cwd()
    candidates.extend(
        [
            (cwd / path).resolve(),
            (cwd / "src" / path).resolve(),
            (cwd / ".." / path).resolve(),
            (cwd / "dobot_quad_sdk-main" / "low_level" / "python" / "config" / "dds_config.yaml").resolve(),
            (cwd / ".." / "dobot_quad_sdk-main" / "low_level" / "python" / "config" / "dds_config.yaml").resolve(),
        ]
    )

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return None


def normalize_file_uri(uri: str) -> Path | None:
    """将 file:// URI 转为路径；其他 URI 类型返回 None。
    Convert a file:// URI to a path; return None for other URI types.
    """
    if not uri.startswith("file://"):
        return None
    return Path(uri[len("file://") :]).expanduser()


def ensure_valid_cyclonedds_uri(config_file: str | None = None) -> str | None:
    """校验 CYCLONEDDS_URI，并修复演示工程常见拷贝路径。
    Validate CYCLONEDDS_URI and fix common copied paths in demo projects.

    CycloneDDS 会全局读取 CYCLONEDDS_URI。如果该变量指向不存在的文件，
    CycloneDDS reads CYCLONEDDS_URI globally. If this variable points to a non-existent file,
    即使 PyDDSMiddleware 收到有效 YAML 配置，DDS 初始化仍会失败。
    DDS initialization still fails even if PyDDSMiddleware receives a valid YAML configuration.
    """
    current = os.environ.get("CYCLONEDDS_URI", "").strip()
    current_path = normalize_file_uri(current) if current else None
    if current_path and current_path.exists():
        return current

    candidates: list[Path] = []
    cwd = Path.cwd()
    candidates.extend(
        [
            (cwd / "dobot_quad_sdk-main" / "cyclonedds.xml").resolve(),
            (cwd / ".." / "dobot_quad_sdk-main" / "cyclonedds.xml").resolve(),
            (cwd / "src" / "dobot_quad_sdk-main" / "cyclonedds.xml").resolve(),
        ]
    )
    if config_file:
        cfg = Path(config_file).expanduser()
        if cfg.exists():
            candidates.extend(
                [
                    (cfg.parent / ".." / ".." / ".." / ".." / "dobot_quad_sdk-main" / "cyclonedds.xml").resolve(),
                    (cfg.parent / ".." / ".." / ".." / "dobot_quad_sdk-main" / "cyclonedds.xml").resolve(),
                ]
            )

    for candidate in candidates:
        if candidate.exists():
            uri = "file://" + str(candidate)
            os.environ["CYCLONEDDS_URI"] = uri
            return uri

    if current and current_path and not current_path.exists():
        # 清理失效路径，避免 CycloneDDS 因旧路径必然启动失败。
        # Clear invalid paths to prevent CycloneDDS from always failing startup due to stale paths.
        os.environ.pop("CYCLONEDDS_URI", None)
        return None
    return current or None
