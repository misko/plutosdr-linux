#!/usr/bin/env python3
"""Regression for issue 123 using the deployed production helper verbatim."""
import re
import subprocess
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[4]
source = (root / "drivers/iio/adc/ad9361.c").read_text()
match = re.search(r"static int ad9361_counter_wait_rx_lock\([^;]+?\)\n\{", source)
body = source[match.start():source.index("\n}", match.end()) + 2]
# The diagnostic helper is exercised by run.py; this test isolates the clock rule.
body = re.sub(r"\n\s*ad9361_counter_diag_lock\([^;]+;", "", body)
prefix = r'''
#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
typedef int64_t ktime_t;
typedef uint64_t u64;
#include "adi_rx_counter.h"
struct ad9361_rf_phy { void *spi; unsigned counter_diag_stage, counter_diag_profile; };
#define ad9361_counter_diag_counter(phy) 0
#define ad9361_counter_diag_record(phy, event) ((void)0)
#define REG_RX_CP_OVERRANGE_VCO_LOCK 1
#define VCO_LOCK 2
static int64_t now, latency;
static int raw, reads;
static int64_t ktime_get(void) { return now; }
static int64_t ktime_get_ns(void) { return now * 1000; }
static int64_t ktime_to_ns(int64_t value) { return value * 1000; }
static int64_t ktime_add_us(int64_t value, int us) { return value + us; }
static int ktime_compare(int64_t a, int64_t b) { return (a > b) - (a < b); }
static int ad9361_spi_read(void *spi, int reg) { reads++; now += latency; return raw; }
static void usleep_range(int a, int b) { now += b; }
'''
main = r'''
int main(void) {
 struct ad9361_rf_phy phy = {0};
 raw = VCO_LOCK; latency = 2500;
 assert(ad9361_counter_wait_rx_lock(&phy) == 0);
 assert(reads == 1 && now == 2500);
 now = reads = 0; raw = 0;
 assert(ad9361_counter_wait_rx_lock(&phy) == -ETIMEDOUT && reads == 1);
 now = reads = 0; raw = -EIO;
 assert(ad9361_counter_wait_rx_lock(&phy) == -EIO && reads == 1);
 now = reads = 0; latency = 0; raw = 0;
 assert(ad9361_counter_wait_rx_lock(&phy) == -ETIMEDOUT && now <= 2040);
 puts("PASS: delayed locked read, delayed unlocked read, SPI error, bounded polling");
}
'''
with tempfile.TemporaryDirectory(prefix="issue123-deadline-") as directory:
    path = Path(directory)
    (path / "test.c").write_text(prefix + body + main)
    subprocess.run(["cc", "-std=gnu11", "-Wno-unused-function", "-fsanitize=undefined",
                    "-iquote" + str(root / "include/uapi/linux"), str(path / "test.c"), "-o", str(path / "test")], check=True)
    subprocess.run([str(path / "test")], check=True)
