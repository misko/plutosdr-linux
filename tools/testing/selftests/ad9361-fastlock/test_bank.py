#!/usr/bin/env python3
"""Exercise production bank/load code with a register-accurate SPI memory model."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[4]
source = (root / "drivers/iio/adc/ad9361.c").read_text()


def function(name):
    match = re.search(r"^(?:static )?(?:int|u32|u64|bool|void) " + name +
                      r"\([^;]+?\)\n\{", source, re.M)
    assert match, name
    return source[match.start():source.index("\n}", match.end()) + 2] + "\n"


functions = [
    "ad9361_calc_rfpll_freq", "ad9361_counter_frequency_matches",
    "ad9361_counter_diag_record", "ad9361_counter_diag_counter",
    "ad9361_counter_diag_operation", "ad9361_counter_wait_rx_lock",
    "ad9361_fastlock_readval", "ad9361_fastlock_writeval",
    "ad9361_fastlock_save", "ad9361_fastlock_load", "ad9361_fastlock_prepare",
    "ad9361_fastlock_recall", "ad9361_counter_profile_rx_lo",
    "ad9361_counter_read_rx_lo", "ad9361_counter_profile_crc",
    "ad9361_counter_restore_rx_lo", "ad9361_counter_bank_image_frequency",
    "ad9361_counter_bank_attest", "ad9361_counter_configure_bank",
    "ad9361_counter_bank_recall", "__ad9361_counter_release",
]
with tempfile.TemporaryDirectory(prefix="ad9361-bank-") as tmp:
    path = Path(tmp)
    text = Path(__file__).with_name("bank_mocks.h").read_text()
    text += "\n".join(function(name) for name in functions)
    text += Path(__file__).with_name("bank_scenarios.c").read_text()
    (path / "test.c").write_text(text)
    command = [os.environ.get("HOSTCC", "cc"), "-std=gnu11", "-Wall", "-Wextra",
               "-Werror", "-Wno-unused-parameter", "-Wno-sign-compare",
               "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-pthread",
               "-I" + str(root / "drivers/iio/adc"),
               "-iquote" + str(root / "include/uapi/linux"),
               str(path / "test.c"), "-o", str(path / "test")]
    subprocess.run(command, check=True)
    subprocess.run([str(path / "test")], check=True)
