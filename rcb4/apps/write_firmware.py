#!/usr/bin/env python

from pathlib import Path
import shutil
import subprocess
import sys

from rcb4.data import kondoh7_elf
from rcb4.data import stlink


def check_dependencies():
    objcopy = shutil.which("arm-none-eabi-objcopy")
    if objcopy is None:
        print("Please install arm-none-eabi-objcopy.")
        print("sudo apt install -y binutils-arm-none-eabi")
        sys.exit(1)


def convert_elf_to_bin(elf_path, bin_path):
    cmd = ["arm-none-eabi-objcopy", "-O", "binary", elf_path, bin_path]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error converting ELF to BIN: {result.stderr}")
        sys.exit(2)


def flash_bin_to_device(st_flash_path, bin_path):
    cmd = [st_flash_path, "write", bin_path, "0x08000000"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"Error flashing BIN to device: {result.stderr}")
        sys.exit(3)


def main():
    check_dependencies()

    st_flash_path = stlink()
    elf_path = kondoh7_elf()
    bin_path = Path(elf_path).with_suffix(".bin")

    convert_elf_to_bin(elf_path, bin_path)
    flash_bin_to_device(st_flash_path, bin_path)


if __name__ == "__main__":
    main()
