#!/usr/bin/env python3
"""Compile the production scan helpers against deterministic SPI/clock mocks."""
from pathlib import Path
import os, subprocess, tempfile
root = Path(__file__).resolve().parents[4]
source = (root / 'drivers/iio/adc/ad9361.c').read_text()
def function(name):
    import re
    match = re.search(r'^(?:static )?(?:int|u64|bool) '+name+r'\([^;]+?\)\n\{', source, re.M)
    assert match, name
    end = source.index('\n}', match.end()) + 2
    return source[match.start():end] + '\n'
functions = ['ad9361_calc_rfpll_freq', 'ad9361_counter_frequency_matches',
             'ad9361_counter_wait_rx_lock', 'ad9361_counter_profile_rx_lo',
             'ad9361_counter_read_rx_lo', 'ad9361_counter_restore_rx_lo',
             'ad9361_counter_fastlock_recall']
with tempfile.TemporaryDirectory(prefix='ad9361-fastlock-') as tmp:
    path=Path(tmp)
    text=(Path(__file__).with_name('mocks.h')).read_text()
    text+='\n'.join(function(name) for name in functions)
    text+=function('ad9361_fastlock_prepare').replace('ad9361_fastlock_prepare(', 'checked_prepare(')
    text+=function('ad9361_fastlock_recall').replace('ad9361_fastlock_recall(', 'checked_recall(').replace('ad9361_fastlock_prepare(', 'checked_prepare(')
    text+=(Path(__file__).with_name('scenarios.c')).read_text()
    (path/'test.c').write_text(text)
    subprocess.run([os.environ.get('HOSTCC','cc'), '-std=gnu11', '-Wall', '-Wextra',
                    '-Werror', '-Wno-unused-parameter', '-Wno-sign-compare',
                    '-fsanitize=undefined', '-I'+str(root/'drivers/iio/adc'),
                    str(path/'test.c'), '-o', str(path/'test')], check=True)
    subprocess.run([str(path/'test')],check=True)
