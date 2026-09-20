#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 hexomethyl

"""
blackpill_cli.py - reference host tool for BlackPillFlasher, the port of the
PicoFlasher binary protocol to the WeAct STM32F411CEU6 Black Pill.

The command protocol runs over a USB CDC-ACM port with VID 0x600D /
PID 0x7001 and is identical on both firmwares built from this repository:
the RP2040 PicoFlasher build and the BlackPillFlasher STM32F411 port.
This tool works against either.

Wire format: every command is a packed 5-byte little-endian struct
(u8 opcode + u32 lba).  Commands that answer reply with a little-endian
u32 status word (0 = success) followed by any payload.  Each streamed
block is 0x210 bytes: 0x200 data + 0x10 spare.

Requires Python 3.8+ and pyserial (nothing else).

Examples:
	python3 blackpill_cli.py probe
	python3 blackpill_cli.py dump nand.bin --blocks 0x8000	# 16MB image
	python3 blackpill_cli.py dump nand64.bin --blocks 0x20000	# 64MB image
	python3 blackpill_cli.py write nand.bin
	python3 blackpill_cli.py reboot-bootloader
	python3 blackpill_cli.py reboot-bootloader
"""

import argparse
import os
import struct
import sys
import time

try:
	import serial
	import serial.tools.list_ports
except ImportError:  # keep --help usable without pyserial installed
	serial = None

# USB identity of both firmwares (see usb_descriptors.c).
DEFAULT_VID = 0x600D
DEFAULT_PID = 0x7001

# Command opcodes - mirrors PicoFlasher/protocol.h (single source in this
# file: never write these numbers anywhere else below).
CMD_GET_VERSION = 0x00
CMD_GET_FLASH_CONFIG = 0x01
CMD_READ_FLASH = 0x02
CMD_WRITE_FLASH = 0x03
CMD_READ_FLASH_STREAM = 0x04
CMD_ERASE_FLASH = 0x05
CMD_SET_SMC_WORKAROUND = 0x20  # lba & 1, no response
CMD_STOP_SMC = 0x21  # no response
CMD_START_SMC = 0x22  # no response
CMD_SET_UART_BAUD = 0x23  # lba = (channel << 24) | baud, no response
CMD_REBOOT_TO_BOOTLOADER = 0xFE  # no response

# struct cmd: u8 opcode + u32 lba, both little-endian, packed (5 bytes).
CMD = struct.Struct("<BI")

BLOCK_SIZE = 0x210  # 0x200 data + 0x10 spare, one "block" on the wire
IO_TIMEOUT = 10.0  # seconds, for both directions
PROGRESS_EVERY = 256  # blocks between progress updates

# Flash configs J-Runner-with-Extras recognizes (J-Runner/Classes/PicoFlasher.cs).
KNOWN_FLASH_CONFIGS = {
	0x01198010: "Xenon, Zephyr, Falcon: 16MB",
	0x01198030: "Xenon, Zephyr, Falcon: 64MB",
	0x00023010: "Jasper, Trinity: 16MB",
	0x00043000: "Corona: 16MB",
	0x008A3020: "Jasper, Trinity: 256MB",
	0x00AA3020: "Jasper, Trinity: 512MB",
	0x008C3020: "Corona: 256MB",
	0x00AC3020: "Corona: 512MB",
	0xC0462002: "Corona: 4GB (eMMC)",
}


class ToolError(Exception):
	pass


# ---------------------------------------------------------------------------
# Flash config decoding
# ---------------------------------------------------------------------------

def flash_config_size_mb(fc):
	"""
	NAND size in MB derived from the flash config word.

	Port of J-Runner-with-Extras PicoFlasher.getFlashSize(): "major" is
	bits 17-18 (small-block vs large-block generation), "minor" is bits 4-5.
	"""
	major = (fc >> 17) & 3
	minor = (fc >> 4) & 3

	size = 0
	if major >= 1:
		if minor == 0:  # Corona 16MB
			if ((fc >> 17) & 3) != 0x01:
				size = 16
		elif minor == 1:  # Jasper 16MB / Trinity 16MB vs 64MB
			if ((fc >> 17) & 3) != 0x01:
				size = 64
			else:
				size = 16
		elif minor == 2 or minor == 3:  # Jasper, Trinity, Corona 256MB/512MB
			size = 8 << (((fc >> 19) & 3) + ((fc >> 21) & 0xF))
	else:  # Xenon, Zephyr, Falcon
		size = 8 << minor

	return size


def decode_flash_config(fc):
	"""Return human-readable lines describing a flash config word."""
	lines = ["flash config: 0x%08X" % fc]

	if fc == 0x00000000:
		lines.append("  all-zero: console not attached, or the SMC was not stopped")
		lines.append("  (0xFFFFFFFF behaves the same way)")
		return lines

	if (fc & 0xF0000000) == 0xC0000000:
		lines.append("  on-board eMMC console (Corona 4GB): use the EMMC_* commands,")
		lines.append("  this tool only implements the SPI NAND commands")
		return lines

	major = (fc >> 17) & 3
	minor = (fc >> 4) & 3

	lines.append("  major (bits 16-18): 0x%X -> %s" % ((fc >> 16) & 7, "small-block" if major == 0 else "large-block"))
	lines.append("  minor (bits 4-5): 0x%X" % minor)
	if major >= 1 and (minor == 2 or minor == 3):
		lines.append("  size bits: bits 19-20 = %d, bits 21-24 = %d" % ((fc >> 19) & 3, (fc >> 21) & 0xF))

	size = flash_config_size_mb(fc)
	if size:
		lines.append("  flash size: %d MB (%d blocks of 0x210)" % (size, size * 1024 * 1024 // 512))
	else:
		lines.append("  flash size: unknown (unrecognized config)")

	known = KNOWN_FLASH_CONFIGS.get(fc)
	if known:
		lines.append("  known config: %s" % known)

	lines.append("  logical page: 0x200 data + 0x10 spare (0x210 per block, as streamed)")
	return lines


# ---------------------------------------------------------------------------
# Device discovery and I/O
# ---------------------------------------------------------------------------

def _interface_number(port):
	"""USB interface number of a port (J-Runner picks the lowest one), or a
	large sentinel when the platform does not report it."""
	iface = getattr(port, "interface", None)
	if iface is None:
		return 1 << 30
	try:
		return int(str(iface).split("/")[0], 10)
	except ValueError:
		return 1 << 30


def find_ports(vid, pid, serial_number):
	ports = []
	for port in serial.tools.list_ports.comports():
		if port.vid != vid or port.pid != pid:
			continue
		if serial_number is not None and (port.serial_number or "") != serial_number:
			continue
		ports.append(port)
	ports.sort(key=lambda p: (_interface_number(p), p.device))
	return ports


def open_device(args):
	if serial is None:
		raise ToolError("pyserial is required: pip install pyserial")

	ports = find_ports(args.vid, args.pid, args.serial)
	if not ports:
		lines = ["no PicoFlasher-compatible device found (VID 0x%04X PID 0x%04X%s)" % (
			args.vid, args.pid, ", serial %s" % args.serial if args.serial else "")]
		candidates = serial.tools.list_ports.comports()
		if candidates:
			lines.append("USB serial ports present:")
			for port in sorted(candidates, key=lambda p: p.device):
				vidpid = "VID:PID=%04X:%04X" % (port.vid, port.pid) if port.vid is not None else "VID:PID=?"
				lines.append("  %s  %s  %s" % (port.device, vidpid, port.description))
		else:
			lines.append("no USB serial ports are present at all")
		raise ToolError("\n".join(lines))

	port = ports[0]
	print("device: %s  %s" % (port.device, port.description))

	# The CDC baud rate is ignored by both firmwares on the command port, so
	# 115200 is arbitrary.  It must never be 1200: the Black Pill firmware
	# treats a 1200-baud line-coding change as a jump to the DFU bootloader.
	return serial.Serial(port.device, 115200, timeout=IO_TIMEOUT, write_timeout=IO_TIMEOUT)


def read_exact(ser, count, what):
	"""Read exactly `count` bytes.  A short read is never silently discarded:
	it raises with explicit got/expected counts."""
	buf = bytearray()
	while len(buf) < count:
		chunk = ser.read(count - len(buf))
		if not chunk:
			raise ToolError("short read (%s): got %d of %d bytes (timeout after %.0f s)" % (
				what, len(buf), count, IO_TIMEOUT))
		buf.extend(chunk)
	return bytes(buf)


def read_status(ser, what):
	return struct.unpack("<I", read_exact(ser, 4, what))[0]


def send_cmd(ser, opcode, lba=0, payload=b""):
	"""Send a 5-byte command, plus an optional payload written in the SAME
	USB write (WRITE_FLASH expects its 0x210-byte payload that way)."""
	ser.write(CMD.pack(opcode, lba) + payload)
	ser.flush()


def send_cmd_no_response(ser, opcode, lba=0):
	send_cmd(ser, opcode, lba)


def check_status(status, cmd_name, block, done):
	if status != 0:
		raise ToolError("%s: block 0x%X failed with status 0x%08X, aborted after %d blocks" % (
			cmd_name, block, status, done))


def progress(done, total):
	if done % PROGRESS_EVERY == 0 or done == total:
		percent = (100.0 * done / total) if total else 100.0
		sys.stdout.write("\r  %d/%d blocks (%.1f%%)" % (done, total, percent))
		sys.stdout.flush()


def print_stats(noun, count, path, elapsed):
	total = count * BLOCK_SIZE
	mib = total / (1024.0 * 1024.0)
	print("\n%s: %d bytes (0x%X) %s in %.1f s (%.2f MiB/s)" % (
		noun, total, total, ("-> %s" % path) if path else "", elapsed, mib / elapsed if elapsed > 0 else 0.0))


# ---------------------------------------------------------------------------
# Subcommands
# ---------------------------------------------------------------------------

def cmd_probe(ser, args):
	version = _get_version(ser)
	print("version: %d" % version)
	if version < 2:
		raise ToolError("firmware is too old (version %d, J-Runner needs >= 2)" % version)

	fc = _get_flash_config(ser)
	for line in decode_flash_config(fc):
		print(line)

	# GET_FLASH_CONFIG stops the SMC (built-in workaround for hosts that do
	# not use the SMC control commands); leave the console running.
	send_cmd_no_response(ser, CMD_START_SMC)
	print("SMC restarted")
	return 0


def _get_version(ser):
	send_cmd(ser, CMD_GET_VERSION)
	return read_status(ser, "GET_VERSION response")


def _get_flash_config(ser):
	send_cmd(ser, CMD_GET_FLASH_CONFIG)
	return read_status(ser, "GET_FLASH_CONFIG response")


def _read_stream(ser, outfile, blocks):
	"""Full-image dump: one READ_FLASH_STREAM command, then `blocks` records
	of (u32 status + 0x210 bytes).  A non-zero status aborts the stream (the
	device sends the 4-byte status and nothing else)."""
	send_cmd(ser, CMD_READ_FLASH_STREAM, blocks)
	for i in range(blocks):
		status = read_status(ser, "block 0x%X status" % i)
		check_status(status, "READ_FLASH_STREAM", i, i)
		outfile.write(read_exact(ser, BLOCK_SIZE, "block 0x%X data" % i))
		progress(i + 1, blocks)


def _read_blocks(ser, outfile, start, blocks):
	"""Ranged dump (--start != 0): the stream always begins at block 0, so
	ranges are read one READ_FLASH command per block (as J-Runner does)."""
	for i in range(blocks):
		block = start + i
		send_cmd(ser, CMD_READ_FLASH, block)
		status = read_status(ser, "block 0x%X status" % block)
		check_status(status, "READ_FLASH", block, i)
		outfile.write(read_exact(ser, BLOCK_SIZE, "block 0x%X data" % block))
		progress(i + 1, blocks)


def cmd_dump(ser, args):
	if args.blocks <= 0:
		raise ToolError("--blocks must be > 0")
	if args.start < 0:
		raise ToolError("--start must be >= 0")

	print("stopping SMC...")
	send_cmd_no_response(ser, CMD_STOP_SMC)
	time.sleep(0.5)  # J-Runner waits 500 ms after stopping the SMC

	try:
		fc = _get_flash_config(ser)
		for line in decode_flash_config(fc):
			print(line)
		if fc == 0x00000000 or fc == 0xFFFFFFFF:
			raise ToolError("console not found (flash config 0x%08X)" % fc)
		if (fc & 0xF0000000) == 0xC0000000:
			raise ToolError("on-board eMMC console: this tool only implements the SPI NAND commands")

		print("dumping %d blocks (0x%X bytes) starting at block 0x%X to %s" % (
			args.blocks, args.blocks * BLOCK_SIZE, args.start, args.out))
		started = time.monotonic()
		with open(args.out, "wb") as outfile:
			if args.start == 0:
				_read_stream(ser, outfile, args.blocks)
			else:
				_read_blocks(ser, outfile, args.start, args.blocks)
		print_stats("dumped", args.blocks, args.out, time.monotonic() - started)
	finally:
		print("starting SMC...")
		send_cmd_no_response(ser, CMD_START_SMC)
	return 0


def cmd_write(ser, args):
	if args.start < 0:
		raise ToolError("--start must be >= 0")

	size = os.path.getsize(args.infile)
	if size == 0 or size % BLOCK_SIZE != 0:
		raise ToolError("input file %s is %d bytes: the size must be a non-zero multiple of 0x%X" % (
			args.infile, size, BLOCK_SIZE))
	blocks = size // BLOCK_SIZE

	print("writing %d blocks (0x%X bytes) from %s starting at block 0x%X" % (
		blocks, size, args.infile, args.start))
	print("note: blocks are written raw; erase the range first if it holds data")

	print("stopping SMC...")
	send_cmd_no_response(ser, CMD_STOP_SMC)
	time.sleep(0.5)

	try:
		started = time.monotonic()
		with open(args.infile, "rb") as infile:
			for i in range(blocks):
				block = args.start + i
				payload = infile.read(BLOCK_SIZE)
				# WRITE_FLASH carries its 0x210-byte payload in the same write.
				send_cmd(ser, CMD_WRITE_FLASH, block, payload)
				status = read_status(ser, "block 0x%X status" % block)
				check_status(status, "WRITE_FLASH", block, i)
				progress(i + 1, blocks)
		print_stats("wrote", blocks, None, time.monotonic() - started)
	finally:
		print("starting SMC...")
		send_cmd_no_response(ser, CMD_START_SMC)
	return 0


def cmd_reboot_bootloader(ser, args):
	print("sending REBOOT_TO_BOOTLOADER...")
	send_cmd_no_response(ser, CMD_REBOOT_TO_BOOTLOADER)
	print("""
The device re-enumerates as the STM32 ROM DFU bootloader
(STMicroelectronics 0483:df11).  Flash a new image with:

    dfu-util -R -a 0 --dfuse-address 0x08000000 -D BlackPillFlasher.bin

On the RP2040 PicoFlasher build the same command reboots into the BOOTSEL
USB mass-storage bootloader instead.""")
	return 0


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def auto_int(text):
	return int(text, 0)  # accepts 0x8000 and 32768 alike


def build_parser():
	# --vid/--pid/--serial are accepted both before and after the subcommand.
	# The subparser copies default to SUPPRESS so they never clobber a value
	# that was given before the subcommand (an argparse parents quirk).
	def common(overrides_only):
		suppress = argparse.SUPPRESS if overrides_only else None
		p = argparse.ArgumentParser(add_help=False)
		p.add_argument("--vid", type=auto_int, default=DEFAULT_VID if not overrides_only else suppress,
						help="USB vendor ID to search for (default: 0x600D)")
		p.add_argument("--pid", type=auto_int, default=DEFAULT_PID if not overrides_only else suppress,
						help="USB product ID to search for (default: 0x7001)")
		p.add_argument("--serial", default=suppress, metavar="SERIAL",
						help="USB serial number to pick one of several devices (default: any)")
		return p

	parser = argparse.ArgumentParser(
		parents=[common(False)],
		description="BlackPillFlasher host tool: probe, dump, write and reboot-to-bootloader "
					"over the CDC-ACM command protocol (the PicoFlasher wire protocol). Works "
					"with both the RP2040 PicoFlasher firmware and the BlackPillFlasher "
					"STM32F411 port.")
	sub = parser.add_subparsers(dest="command", metavar="COMMAND")
	sub.required = True

	p = sub.add_parser("probe", parents=[common(True)],
					   help="read the firmware version and the console flash config")
	p.set_defaults(func=cmd_probe)

	p = sub.add_parser("dump", parents=[common(True)],
					   help="dump NAND blocks from the console into OUT")
	p.add_argument("out", metavar="OUT", help="output image file, e.g. nanddump.bin")
	p.add_argument("--blocks", type=auto_int, required=True,
				   help="number of 0x210-byte blocks to dump (16MB: 0x8000, 64MB: 0x20000)")
	p.add_argument("--start", type=auto_int, default=0, metavar="S",
				   help="first block to dump (default: 0); non-zero uses per-block READ_FLASH")
	p.set_defaults(func=cmd_dump)

	p = sub.add_parser("write", parents=[common(True)],
					   help="write image file IN to the console NAND (raw, no erase)")
	p.add_argument("infile", metavar="IN", help="input image file, size a multiple of 0x210")
	p.add_argument("--start", type=auto_int, default=0, metavar="S",
				   help="first block to write (default: 0)")
	p.set_defaults(func=cmd_write)

	p = sub.add_parser("reboot-bootloader", parents=[common(True)],
					   help="reboot the device into its bootloader (Black Pill: STM32 ROM DFU)")
	p.set_defaults(func=cmd_reboot_bootloader)

	return parser


def main(argv=None):
	args = build_parser().parse_args(argv)

	try:
		ser = open_device(args)
	except ToolError as exc:
		print("error: %s" % exc, file=sys.stderr)
		return 1

	try:
		with ser:
			return args.func(ser, args)
	except ToolError as exc:
		print("\nerror: %s" % exc, file=sys.stderr)
		return 1
	except serial.SerialException as exc:
		print("\nerror: serial port failed: %s" % exc, file=sys.stderr)
		return 1


if __name__ == "__main__":
	sys.exit(main())
