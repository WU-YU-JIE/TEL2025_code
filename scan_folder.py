# scan_folder.py
# -*- coding: utf-8 -*-
import os, sys, csv, time, json, base64, hashlib
from collections import defaultdict

DEFAULT_TARGET = r"C:\Users\user\OneDrive - 中原大學\桌面\大學作品集\東京威力\TEL2025_MainCode"

# 略過的資料夾
SKIP_DIRS = {
    ".git","__pycache__","node_modules",".venv","venv","env",
    "build","dist",".idea",".vscode",".pytest_cache",".mypy_cache",".cache"
}

# 當作文字檔處理（小寫副檔名）
TEXT_EXTS = {
    ".py",".c",".cpp",".h",".hpp",".java",".js",".ts",".sh",".bat",".ps1",".ino",
    ".go",".rb",".php",".swift",".kt",".m",".cs",".json",".yml",".yaml",".toml",
    ".ini",".md",".txt",".csv",".xml",".html",".css"
}

# 內容輸出控制
MAX_TEXT_BYTES = 512 * 1024      # 單一文字檔最多讀 512 KB 內容喔
MAX_BINARY_BYTES = 0             # 二進位檔是否輸出 base64（0=不輸出，只記錄雜湊與中繼資料）
ENCODING = "utf-8"               # 文字讀取編碼（失敗則用 errors='replace'）

def is_probably_binary(path, ext) -> bool:
    if ext.lower() in TEXT_EXTS:
        return False
    try:
        with open(path, "rb") as f:
            head = f.read(4096)
        # 有 NUL 或太多非可列印控制字 → 視為二進位
        if b"\x00" in head:
            return True
        nontext = sum(1 for b in head if b < 9 or (13 < b < 32) or b == 127)
        return nontext / max(1, len(head)) > 0.30
    except Exception:
        return True

def human_size(n: int) -> str:
    units = ["B","KB","MB","GB","TB"]
    i, f = 0, float(n)
    while f >= 1024 and i < len(units)-1:
        f /= 1024; i += 1
    return f"{f:.2f} {units[i]}"

def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()

def gen_tree(root: str, skip_dirs=SKIP_DIRS, max_files_per_dir=200) -> str:
    lines = []
    root = os.path.abspath(root)
    for current_root, dirs, files in os.walk(root):
        rel = os.path.relpath(current_root, root)
        depth = 0 if rel == "." else rel.count(os.sep) + 1
        indent = "    " * depth
        lines.append((os.path.basename(root) if rel == "." else f"{'    '*(depth-1)}└─ {os.path.basename(current_root)}"))
        dirs[:] = [d for d in dirs if d not in skip_dirs]
        shown = 0
        for name in sorted(files):
            if shown >= max_files_per_dir:
                lines.append(f"{indent}└─ ... ({len(files)-shown} more files)")
                break
            lines.append(f"{indent}└─ {name}")
            shown += 1
    return "\n".join(lines)

def main():
    target = sys.argv[1] if len(sys.argv) >= 2 else DEFAULT_TARGET
    target = os.path.abspath(os.path.expanduser(target))
    if not os.path.isdir(target):
        print(f"❌ 找不到資料夾：{target}")
        sys.exit(1)

    out_csv  = os.path.join(target, "files_report.csv")
    out_tree = os.path.join(target, "files_tree.txt")
    out_jsonl = os.path.join(target, "files_content.jsonl")
    out_alltxt = os.path.join(target, "ALL_TEXTS.txt")

    by_ext_count = defaultdict(int)
    by_ext_size = defaultdict(int)
    total_files = total_bytes = 0

    # 先寫 CSV（中繼資料）
    with open(out_csv, "w", newline="", encoding="utf-8-sig") as fcsv, \
         open(out_jsonl, "w", encoding="utf-8") as fjsonl, \
         open(out_alltxt, "w", encoding="utf-8", errors="replace") as falltxt:

        writer = csv.writer(fcsv)
        writer.writerow([
            "relative_path", "extension", "size_bytes", "size_human",
            "modified_time", "is_text", "line_count", "sha256"
        ])

        for root_dir, dirs, files in os.walk(target):
            dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
            for fname in files:
                fpath = os.path.join(root_dir, fname)
                rel = os.path.relpath(fpath, target)
                ext = os.path.splitext(fname)[1].lower()

                try:
                    stat = os.stat(fpath)
                except OSError:
                    continue

                size = stat.st_size
                mtime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(stat.st_mtime))
                is_bin = is_probably_binary(fpath, ext)
                sha256 = sha256_file(fpath)

                line_count = ""
                text_preview = ""
                encoding_used = None

                if not is_bin:
                    # 讀部分文字內容（限制大小）
                    try:
                        with open(fpath, "r", encoding=ENCODING, errors="replace") as fr:
                            content = fr.read(min(MAX_TEXT_BYTES, size))
                            encoding_used = ENCODING
                            text_preview = content
                            line_count = content.count("\n") + (1 if content and not content.endswith("\n") else 0)
                    except Exception:
                        pass

                    # 寫入 ALL_TEXTS 匯總
                    falltxt.write("\n" + "="*80 + "\n")
                    falltxt.write(f"FILE: {rel}\n")
                    falltxt.write("="*80 + "\n")
                    if text_preview:
                        falltxt.write(text_preview)
                        if size > MAX_TEXT_BYTES:
                            falltxt.write("\n[... TRUNCATED ...]\n")
                    else:
                        falltxt.write("[[EMPTY OR UNREADABLE TEXT]]\n")

                    # JSONL：文字
                    json.dump({
                        "path": rel,
                        "extension": ext or "(noext)",
                        "size_bytes": size,
                        "modified_time": mtime,
                        "sha256": sha256,
                        "is_text": True,
                        "encoding": encoding_used or "utf-8",
                        "truncated": bool(size > MAX_TEXT_BYTES),
                        "content": text_preview
                    }, fjsonl, ensure_ascii=False)
                    fjsonl.write("\n")

                else:
                    # JSONL：二進位（預設不含內容，只記雜湊與中繼資料；需要可改 MAX_BINARY_BYTES）
                    b64 = None
                    if MAX_BINARY_BYTES > 0:
                        try:
                            with open(fpath, "rb") as fb:
                                raw = fb.read(min(MAX_BINARY_BYTES, size))
                            b64 = base64.b64encode(raw).decode("ascii")
                        except Exception:
                            b64 = None

                    json.dump({
                        "path": rel,
                        "extension": ext or "(noext)",
                        "size_bytes": size,
                        "modified_time": mtime,
                        "sha256": sha256,
                        "is_text": False,
                        "base64_bytes": b64,
                        "truncated": bool(size > MAX_BINARY_BYTES) if MAX_BINARY_BYTES > 0 else None
                    }, fjsonl)
                    fjsonl.write("\n")

                # CSV（中繼資料一覽）
                writer.writerow([
                    rel, ext or "(noext)", size, human_size(size), mtime,
                    int(not is_bin), line_count, sha256
                ])

                by_ext_count[ext] += 1
                by_ext_size[ext] += size
                total_files += 1
                total_bytes += size

    # 產生樹狀
    with open(out_tree, "w", encoding="utf-8") as ftree:
        ftree.write(gen_tree(target))

    # 摘要
    print("📁 掃描完成：", target)
    print(f"   檔案總數：{total_files}")
    print(f"   總大小  ：{human_size(total_bytes)}")
    print("   依副檔名統計（前 15 名）：")
    top = sorted(by_ext_count.items(), key=lambda kv: by_ext_size[kv[0]], reverse=True)[:15]
    for ext, cnt in top:
        print(f"   {ext or '(noext)':>8}  {cnt:>6} 檔  {human_size(by_ext_size[ext]):>10}")

    print(f"\n✅ 已輸出：\n  - {out_csv}\n  - {out_tree}\n  - {out_jsonl}\n  - {out_alltxt}")
    print("\n⚙️ 參數：MAX_TEXT_BYTES={}, MAX_BINARY_BYTES={}（可在檔頭調整）".format(MAX_TEXT_BYTES, MAX_BINARY_BYTES))
    print("💡 JSONL 可被 jq / pandas / BigQuery / Elasticsearch 等直接使用；ALL_TEXTS.txt 方便人工檢視。")

if __name__ == "__main__":
    main()
