#!/usr/bin/env python3
"""Passively monitor, validate, and reconstruct HAB image packets over UART.

The program sends nothing to the flight computer. It accumulates regularly
paced HAB_PACKET lines, deduplicates shadow/LoRa copies, and reconstructs an
image automatically after its metadata and every data packet have arrived.
Metadata packets are fixed at 210 bytes; data packets carry only their exact
encoded scan length.
"""

from __future__ import annotations

import argparse
import io
import json
import struct
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

PACKET_SIZE = 210
METADATA_HEADER_SIZE = 8
DATA_HEADER_SIZE = 7
MIN_DATA_PACKET_SIZE = DATA_HEADER_SIZE + 1
STATUS_FLAG_NAMES = {
    0x0001: "GPS_LOST_AFTER_FIX",
    0x0002: "CAMERA_ERROR",
    0x0004: "LOW_BATTERY_RESERVED",
    0x0008: "IMAGE_SAVE_FAILED",
    0x0010: "FILESYSTEM_ERROR",
    0x0020: "ENCODE_FAILED",
    0x0040: "ABNORMAL_RESET",
    0x0080: "PSRAM_FAULT",
    0x0100: "SENSOR_BUS_RESERVED",
    0x0200: "LORA_TX_FAILURE",
    0x0400: "WSPR_TX_FAILURE",
}


def decode_status_flags(flags: int) -> list[str]:
    names = [name for bit, name in STATUS_FLAG_NAMES.items() if flags & bit]
    known_mask = sum(STATUS_FLAG_NAMES)
    unknown = flags & ~known_mask
    if unknown:
        names.append(f"UNKNOWN_0x{unknown:04X}")
    return names


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def packet_wire_error(data: bytes) -> str | None:
    """Return a length error without reading beyond a short packet."""
    if len(data) < 4:
        return f"payload has {len(data)} bytes, shorter than the packet ID prefix"
    if len(data) > PACKET_SIZE:
        return f"payload has {len(data)} bytes, maximum is {PACKET_SIZE}"
    if data[2:4] == b"\x00\x00":
        if len(data) != PACKET_SIZE:
            return f"metadata payload has {len(data)} bytes, expected {PACKET_SIZE}"
    elif len(data) < MIN_DATA_PACKET_SIZE:
        return (f"data payload has {len(data)} bytes, minimum is "
                f"{MIN_DATA_PACKET_SIZE}")
    return None


@dataclass
class Packet:
    sequence: int
    data: bytes
    crc_ok: bool

    @property
    def image_id(self) -> int:
        return struct.unpack_from(">H", self.data, 0)[0]

    @property
    def packet_id(self) -> int:
        return struct.unpack_from(">H", self.data, 2)[0]

    @property
    def mcu_index(self) -> int:
        return struct.unpack_from(">H", self.data, 4)[0]

    @property
    def mcu_count(self) -> int:
        return self.data[6]

    @property
    def is_metadata(self) -> bool:
        return self.packet_id == 0

    @property
    def subsampling(self) -> int:
        return self.data[7] & 0x07


@dataclass
class Capture:
    image_id: int
    total_packets: int
    data_packets: int
    width: int
    height: int
    mcus_x: int
    mcus_y: int
    jpeg_bytes: int
    packets: list[Packet]


@dataclass
class Metadata:
    width: int
    height: int
    mcus_x: int
    mcus_y: int
    data_packets: int
    subsampling: int
    quant_tables: list[bytes]
    telemetry: dict[str, object]


def parse_telemetry(raw: bytes) -> dict[str, object]:
    latitude, longitude = struct.unpack_from(">ii", raw, 0)
    altitude, speed, heading, vertical_speed = struct.unpack_from(">HHHh", raw, 8)
    hdop, satellites, fix_type = struct.unpack_from(">BBB", raw, 16)
    timestamp, uptime = struct.unpack_from(">II", raw, 19)
    battery_mv = struct.unpack_from(">H", raw, 27)[0]
    esp_temp = struct.unpack_from(">b", raw, 29)[0]
    ext_temp = struct.unpack_from(">h", raw, 30)[0]
    pressure = struct.unpack_from(">H", raw, 32)[0]
    humidity = raw[34]
    jpeg_size = struct.unpack_from(">I", raw, 35)[0]
    capture_ms, free_heap_kb, status_flags = struct.unpack_from(">HHH", raw, 39)
    return {
        "latitude_deg": latitude / 1_000_000,
        "longitude_deg": longitude / 1_000_000,
        "altitude_m": altitude,
        "speed_cm_s": speed,
        "heading_deg": heading / 100,
        "vertical_speed_cm_s": vertical_speed,
        "hdop": hdop / 10,
        "satellites": satellites,
        "fix_type": fix_type,
        "timestamp": timestamp,
        "uptime_s": uptime,
        "battery_mv": battery_mv,
        "esp_temp_c": esp_temp,
        "external_temp_c": ext_temp / 100,
        "pressure_hpa": pressure / 10,
        "humidity_percent": humidity,
        "jpeg_file_size": jpeg_size,
        "capture_ms": capture_ms,
        "free_heap_kb": free_heap_kb,
        "status_flags": status_flags,
        "status_errors": decode_status_flags(status_flags),
    }


def parse_metadata(packet: Packet) -> Metadata:
    payload = packet.data[METADATA_HEADER_SIZE:]
    width, height, mcus_x, mcus_y, data_packets = struct.unpack_from(">HHHHH", payload, 0)
    table_count = payload[11]
    if table_count not in (1, 2):
        raise ValueError(f"unsupported quantization table count: {table_count}")
    tables = [bytes(payload[12:76])]
    if table_count == 2:
        tables.append(bytes(payload[76:140]))
    return Metadata(width, height, mcus_x, mcus_y, data_packets,
                    packet.subsampling, tables,
                    parse_telemetry(payload[140:192]))


def jpeg_marker(marker: int, body: bytes) -> bytes:
    return b"\xFF" + bytes([marker]) + struct.pack(">H", len(body) + 2) + body


def standard_huffman_segments() -> bytes:
    """Ask Pillow for its standard baseline DHT segments once."""
    from PIL import Image

    buffer = io.BytesIO()
    Image.new("RGB", (16, 16)).save(buffer, "JPEG", quality=75, optimize=False,
                                     progressive=False)
    data = buffer.getvalue()
    position = 2
    output = bytearray()
    while position + 4 <= len(data):
        if data[position] != 0xFF:
            break
        marker = data[position + 1]
        if marker == 0xDA:
            break
        length = struct.unpack_from(">H", data, position + 2)[0]
        segment = data[position:position + length + 2]
        if marker == 0xC4:
            output.extend(segment)
        position += length + 2
    if not output:
        raise RuntimeError("Pillow did not provide standard Huffman tables")
    return bytes(output)


def packet_as_jpeg(packet: Packet, metadata: Metadata, dht: bytes) -> bytes:
    if metadata.subsampling == 0:
        mcu_width, mcu_height, y_sampling = 16, 16, 0x22
    elif metadata.subsampling == 1:
        mcu_width, mcu_height, y_sampling = 16, 8, 0x21
    elif metadata.subsampling == 2:
        mcu_width, mcu_height, y_sampling = 8, 8, 0x11
    else:
        raise ValueError(f"unsupported subsampling mode {metadata.subsampling}")

    strip_width = packet.mcu_count * mcu_width
    dqt = b"".join(jpeg_marker(0xDB, bytes([index]) + table)
                   for index, table in enumerate(metadata.quant_tables))
    chroma_table = 1 if len(metadata.quant_tables) > 1 else 0
    sof = jpeg_marker(0xC0, struct.pack(">BHHB", 8, mcu_height, strip_width, 3) +
                      bytes([1, y_sampling, 0, 2, 0x11, chroma_table,
                             3, 0x11, chroma_table]))
    sos = jpeg_marker(0xDA, bytes([3, 1, 0x00, 2, 0x11, 3, 0x11, 0, 63, 0]))
    entropy = packet.data[DATA_HEADER_SIZE:]
    stuffed = entropy.replace(b"\xFF", b"\xFF\x00")
    return b"\xFF\xD8" + dqt + sof + dht + sos + stuffed + b"\xFF\xD9"


def validate_and_reconstruct(capture: Capture, output_dir: Path) -> tuple[list[str], Metadata]:
    from PIL import Image

    failures: list[str] = []
    if len(capture.packets) != capture.total_packets:
        failures.append("received packet count differs from BEGIN")
    if any(not packet.crc_ok for packet in capture.packets):
        failures.append("one or more serial frame CRCs failed")
    valid_packets: list[Packet] = []
    for packet in capture.packets:
        wire_error = packet_wire_error(packet.data)
        if wire_error:
            failures.append(f"packet sequence {packet.sequence}: {wire_error}")
        else:
            valid_packets.append(packet)
    if any(packet.image_id != capture.image_id for packet in valid_packets):
        failures.append("packet image ID differs from BEGIN")

    metadata_packets = [packet for packet in valid_packets if packet.is_metadata]
    data_packets = [packet for packet in valid_packets if not packet.is_metadata]
    if not metadata_packets:
        raise RuntimeError("no metadata packet received")
    metadata = parse_metadata(metadata_packets[0])
    if any(packet.data != metadata_packets[0].data for packet in metadata_packets[1:]):
        failures.append("repeated metadata packets are not identical")
    if len(data_packets) != capture.data_packets:
        failures.append("data packet count differs from BEGIN")
    if metadata.data_packets != capture.data_packets:
        failures.append("metadata data-packet count differs from BEGIN")
    if (metadata.width, metadata.height, metadata.mcus_x, metadata.mcus_y) != (
            capture.width, capture.height, capture.mcus_x, capture.mcus_y):
        failures.append("metadata geometry differs from BEGIN")

    unique = {packet.packet_id: packet for packet in data_packets}
    expected_ids = set(range(1, capture.data_packets + 1))
    if set(unique) != expected_ids:
        failures.append("data packet IDs are missing, duplicated, or out of range")

    subsampling = metadata.subsampling
    mcu_width, mcu_height = {0: (16, 16), 1: (16, 8), 2: (8, 8)}.get(
        subsampling, (0, 0))
    if not mcu_width:
        raise RuntimeError(f"unsupported subsampling mode {subsampling}")

    canvas = Image.new("RGB", (metadata.mcus_x * mcu_width,
                                metadata.mcus_y * mcu_height), (128, 128, 128))
    coverage = Image.new("RGB", (metadata.mcus_x, metadata.mcus_y), (220, 40, 40))
    covered: set[int] = set()
    dht = standard_huffman_segments()
    decode_failures = 0
    for packet_id in sorted(unique):
        packet = unique[packet_id]
        if packet.mcu_count == 0 or packet.mcu_index + packet.mcu_count > metadata.mcus_x * metadata.mcus_y:
            failures.append(f"packet {packet_id} has an invalid MCU range")
            continue
        try:
            strip = Image.open(io.BytesIO(packet_as_jpeg(packet, metadata, dht))).convert("RGB")
            strip.load()
        except Exception as exc:  # Pillow supplies format-specific exception types.
            decode_failures += 1
            print(f"WARNING: packet {packet_id} entropy decode failed: {exc}")
            continue
        for local_mcu in range(packet.mcu_count):
            global_mcu = packet.mcu_index + local_mcu
            if global_mcu in covered:
                failures.append(f"MCU {global_mcu} is present in multiple packets")
            covered.add(global_mcu)
            source = (local_mcu * mcu_width, 0,
                      (local_mcu + 1) * mcu_width, mcu_height)
            x = (global_mcu % metadata.mcus_x) * mcu_width
            y = (global_mcu // metadata.mcus_x) * mcu_height
            canvas.paste(strip.crop(source), (x, y))
            coverage.putpixel((global_mcu % metadata.mcus_x,
                               global_mcu // metadata.mcus_x), (40, 190, 70))

    expected_mcus = set(range(metadata.mcus_x * metadata.mcus_y))
    if covered != expected_mcus:
        failures.append(f"MCU coverage is {len(covered)}/{len(expected_mcus)}")
    if decode_failures:
        failures.append(f"{decode_failures} data packets failed entropy decoding")

    output_dir.mkdir(parents=True, exist_ok=True)
    canvas.crop((0, 0, metadata.width, metadata.height)).save(output_dir / "reconstructed.png")
    coverage.resize((metadata.mcus_x * 8, metadata.mcus_y * 8),
                    resample=Image.Resampling.NEAREST).save(output_dir / "coverage.png")
    return failures, metadata


def save_report(capture: Capture, metadata: Metadata, failures: list[str], output_dir: Path) -> None:
    report = {
        "capture": {key: value for key, value in asdict(capture).items() if key != "packets"},
        "metadata": {
            "width": metadata.width,
            "height": metadata.height,
            "mcus_x": metadata.mcus_x,
            "mcus_y": metadata.mcus_y,
            "data_packets": metadata.data_packets,
            "subsampling": metadata.subsampling,
            "telemetry": metadata.telemetry,
        },
        "checks": {
            "passed": not failures,
            "failures": failures,
            "serial_crc_failures": sum(not packet.crc_ok for packet in capture.packets),
        },
        "packets": [
            {"sequence": packet.sequence, "crc_ok": packet.crc_ok,
             "payload_bytes": len(packet.data), "data_hex": packet.data.hex()}
            for packet in capture.packets
        ],
    }
    (output_dir / "capture.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


@dataclass
class ImageAssembly:
    metadata_packet: Packet | None = None
    data_packets: dict[int, Packet] | None = None
    metadata_copies: int = 0
    completed: bool = False

    def __post_init__(self) -> None:
        if self.data_packets is None:
            self.data_packets = {}


class PassiveAssembler:
    def __init__(self, output_dir: Path):
        self.output_dir = output_dir
        self.images: dict[int, ImageAssembly] = {}

    def accept(self, packet: Packet, source: str, announce: bool = True) -> None:
        state = self.images.setdefault(packet.image_id, ImageAssembly())
        if packet.is_metadata:
            state.metadata_copies += 1
            if state.metadata_packet is None:
                state.metadata_packet = packet
            elif state.metadata_packet.data != packet.data:
                print(f"WARNING: image {packet.image_id} has inconsistent metadata copies")
        elif packet.packet_id == 0:
            print(f"WARNING: image {packet.image_id} has data packet ID zero")
            return
        else:
            previous = state.data_packets.get(packet.packet_id)
            if previous is not None and previous.data != packet.data:
                print(f"WARNING: image {packet.image_id} packet {packet.packet_id} changed")
            else:
                state.data_packets.setdefault(packet.packet_id, packet)

        expected = None
        if state.metadata_packet is not None:
            expected = parse_metadata(state.metadata_packet).data_packets
        if announce:
            expected_text = "?" if expected is None else str(expected)
            print(f"[{source}] image={packet.image_id} packet={packet.packet_id} "
                  f"data={len(state.data_packets)}/{expected_text} "
                  f"metadata_copies={state.metadata_copies}")
        if state.completed or expected is None:
            return
        if set(state.data_packets) != set(range(1, expected + 1)):
            return
        self._finish(packet.image_id, state)

    def _finish(self, image_id: int, state: ImageAssembly) -> None:
        metadata = parse_metadata(state.metadata_packet)
        packets = [state.metadata_packet] + [
            state.data_packets[packet_id]
            for packet_id in range(1, metadata.data_packets + 1)
        ]
        capture = Capture(
            image_id=image_id,
            total_packets=len(packets),
            data_packets=metadata.data_packets,
            width=metadata.width,
            height=metadata.height,
            mcus_x=metadata.mcus_x,
            mcus_y=metadata.mcus_y,
            jpeg_bytes=int(metadata.telemetry["jpeg_file_size"]),
            packets=packets,
        )
        image_dir = self.output_dir / f"image_{image_id:05d}"
        failures, metadata = validate_and_reconstruct(capture, image_dir)
        save_report(capture, metadata, failures, image_dir)
        state.completed = True
        if failures:
            print(f"IMAGE {image_id} FAIL:")
            for failure in failures:
                print(f"  - {failure}")
        else:
            print(f"IMAGE {image_id} PASS: reconstructed at {image_dir.resolve()}")


def decode_packet_line(line: str) -> tuple[str, Packet]:
    parts = line.split()
    if len(parts) != 5 or parts[0] != "HAB_PACKET":
        raise ValueError("malformed HAB_PACKET line")
    source = parts[1]
    sequence = int(parts[2])
    received_crc = int(parts[3], 16)
    payload = bytes.fromhex(parts[4])
    wire_error = packet_wire_error(payload)
    if wire_error:
        raise ValueError(wire_error)
    return source, Packet(sequence, payload, received_crc == crc16_ccitt(payload))


def packet_record(source: str, packet: Packet) -> dict[str, object]:
    return {
        "received_unix": int(time.time()),
        "source": source,
        "sequence": packet.sequence,
        "crc_ok": packet.crc_ok,
        "image_id": packet.image_id,
        "packet_id": packet.packet_id,
        "mcu_index": packet.mcu_index,
        "mcu_count": packet.mcu_count,
        "metadata": packet.is_metadata,
        "payload_bytes": len(packet.data),
        "data_hex": packet.data.hex(),
    }


def replay_packet_log(path: Path, assembler: PassiveAssembler) -> None:
    if not path.exists():
        return
    count = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            record = json.loads(line)
            payload = bytes.fromhex(record["data_hex"])
            wire_error = packet_wire_error(payload)
            if wire_error:
                raise ValueError(wire_error)
            packet = Packet(int(record["sequence"]), payload, bool(record["crc_ok"]))
            if packet.crc_ok:
                assembler.accept(packet, str(record.get("source", "replay")), announce=False)
                count += 1
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            print("WARNING: skipped a malformed saved packet-log record")
    if count:
        print(f"Restored {count} valid packet observations from {path}")


def monitor_passively(port: str, baud: int, output_dir: Path) -> None:
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required: py -m pip install pyserial") from exc

    output_dir.mkdir(parents=True, exist_ok=True)
    packet_log_path = output_dir / "packets.jsonl"
    status_log_path = output_dir / "flight_status.log"
    assembler = PassiveAssembler(output_dir)
    replay_packet_log(packet_log_path, assembler)
    print("Passive monitor running; it sends no commands. Press Ctrl-C to stop.")

    with serial.Serial(port, baud, timeout=2) as serial_port, packet_log_path.open(
            "a", encoding="utf-8") as packet_log, status_log_path.open(
                "a", encoding="utf-8") as status_log:
        serial_port.dtr = False
        serial_port.rts = False
        while True:
            line = serial_port.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            if line.startswith("HAB_PACKET "):
                try:
                    source, packet = decode_packet_line(line)
                    record = packet_record(source, packet)
                    packet_log.write(json.dumps(record) + "\n")
                    packet_log.flush()
                    if not packet.crc_ok:
                        print(f"WARNING: [{source}] serial CRC failed at sequence {packet.sequence}")
                        continue
                    assembler.accept(packet, source)
                except ValueError as error:
                    print(f"WARNING: {error}")
            elif line.startswith("HAB_STATUS "):
                stamped = f"{int(time.time())} {line}"
                status_log.write(stamped + "\n")
                status_log.flush()
                flag_field = next(
                    (field for field in line.split() if field.startswith("flags=0x")),
                    None,
                )
                if flag_field is None:
                    print(line)
                else:
                    errors = decode_status_flags(int(flag_field[6:], 16))
                    print(f"{line} errors={','.join(errors) if errors else 'none'}")
            else:
                print(f"[device] {line}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3", help="UART port (default: COM3)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", type=Path, default=Path("hardware-test-output"))
    args = parser.parse_args()

    try:
        monitor_passively(args.port, args.baud, args.output)
    except KeyboardInterrupt:
        print("\nMonitor stopped; received packets remain saved for the next run")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, TimeoutError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
