#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

import cv2


SUPPORTED_EXTS = ("jpg", "jpeg", "png", "bmp")


def save_frame(frame, output_path: Path, ext: str, quality: int) -> bool:
    """保存单帧图像，返回值表示是否写入成功。"""

    params = []

    if ext in ("jpg", "jpeg"):
        params = [cv2.IMWRITE_JPEG_QUALITY, quality]

    # 先编码再按字节写入，避免 cv2.imwrite 在非 ASCII 路径上的兼容问题。
    ok, buffer = cv2.imencode(f".{ext}", frame, params)

    if not ok:
        return False

    output_path.write_bytes(buffer.tobytes())
    return True


def extract_frames(
    video_path: Path,
    output_dir: Path,
    prefix: str,
    ext: str,
    quality: int,
    step: int,
    start: int,
    end: int | None,
    overwrite: bool,
) -> None:
    """将视频逐帧导出为图片。"""

    if not video_path.is_file():
        raise FileNotFoundError(f"视频文件不存在：{video_path}")

    if step <= 0:
        raise ValueError("抽帧间隔必须大于 0")

    if start < 0:
        raise ValueError("起始帧不能为负数")

    if end is not None and end < start:
        raise ValueError("结束帧不能小于起始帧")

    if not 0 <= quality <= 100:
        raise ValueError("JPEG 质量必须在 0 到 100 之间")

    if output_dir.exists() and not overwrite and any(output_dir.iterdir()):
        raise RuntimeError(
            f"输出目录非空：{output_dir}\n"
            "如需覆盖已有图片，请加 --overwrite。"
        )

    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频：{video_path}")

    try:
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        print(f"视频: {video_path}")
        print(f"分辨率: {width}x{height}")
        print(f"帧率: {fps:.3f} FPS")
        print(f"总帧数: {total_frames if total_frames > 0 else '未知'}")
        print(f"输出目录: {output_dir}")
        print(f"抽帧间隔: 每 {step} 帧取 1 帧")
        print(f"图片格式: {ext.upper()}")

        if start > 0:
            print(f"起始帧: {start}")

        if end is not None:
            print(f"结束帧: {end}")

        # 定位起始帧。
        if start > 0 and not cap.set(cv2.CAP_PROP_POS_FRAMES, start):
            raise RuntimeError(f"无法定位到起始帧：{start}")

        output_dir.mkdir(parents=True, exist_ok=True)

        frame_index = start
        read_count = 0
        saved_count = 0

        while True:
            ok, frame = cap.read()

            if not ok:
                break

            if end is not None and frame_index > end:
                break

            if (frame_index - start) % step == 0:
                output_path = output_dir / f"{prefix}{frame_index:06d}.{ext}"

                if save_frame(frame, output_path, ext, quality):
                    saved_count += 1
                else:
                    print(
                        f"警告：第 {frame_index} 帧写入失败，跳过。",
                        file=sys.stderr,
                    )

                if saved_count % 200 == 0:
                    print(f"已保存 {saved_count} 张")

            frame_index += 1
            read_count += 1

    finally:
        cap.release()

    if read_count == 0:
        raise RuntimeError("没有读取到任何帧，请检查视频文件或起始帧设置。")

    print("\n导出完成")
    print(f"输出目录：{output_dir}")
    print(f"读取帧数：{read_count}")
    print(f"保存张数：{saved_count}")

    if fps > 0:
        print(f"视频时长：{read_count / fps:.2f} 秒")


def main():
    parser = argparse.ArgumentParser(
        description="将视频逐帧导出为图片。"
    )

    parser.add_argument(
        "video",
        type=Path,
        help="输入视频文件路径",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="输出图片目录，默认为当前目录下的 <视频名>_frames",
    )

    parser.add_argument(
        "--prefix",
        type=str,
        default=None,
        help="图片文件名前缀，默认与视频同名",
    )

    parser.add_argument(
        "--ext",
        type=str,
        choices=SUPPORTED_EXTS,
        default="jpg",
        help="图片格式，默认 jpg",
    )

    parser.add_argument(
        "--quality",
        type=int,
        default=95,
        help="JPEG 质量（0-100），默认 95，仅对 jpg 生效",
    )

    parser.add_argument(
        "--step",
        type=int,
        default=1,
        help="抽帧间隔，默认 1 表示每帧都保存",
    )

    parser.add_argument(
        "--start",
        type=int,
        default=0,
        help="起始帧序号（含），默认 0",
    )

    parser.add_argument(
        "--end",
        type=int,
        default=None,
        help="结束帧序号（含），默认到视频末尾",
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="允许写入非空的输出目录",
    )

    args = parser.parse_args()

    video_path = args.video.expanduser().resolve()

    if args.output is None:
        output_dir = Path.cwd() / f"{video_path.stem}_frames"
    else:
        output_dir = args.output.expanduser().resolve()

    prefix = args.prefix if args.prefix is not None else f"{video_path.stem}_"

    try:
        extract_frames(
            video_path=video_path,
            output_dir=output_dir,
            prefix=prefix,
            ext=args.ext,
            quality=args.quality,
            step=args.step,
            start=args.start,
            end=args.end,
            overwrite=args.overwrite,
        )
    except Exception as exc:
        print(f"导出失败：{exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
