/* SPDX-License-Identifier: GPL-2.0 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include "ad9361_regs.h"
#include "adi_rx_counter.h"
typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int64_t ktime_t;
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define BIT(x) (1U << (x))
#define GENMASK(h, l) (((~0U) >> (31 - (h))) & ((~0U) << (l)))
#define do_div(n, d) ((n) /= (d))
#define RX_RFPLL_INT 0
#define RX_RFPLL 1
#define ADI_REG_GP_STATUS 0xb8
#define ADI_REG_GP_CONTROL 0xbc
#define FASTLOOK_INIT 1
#define dev_dbg(...) ((void)0)
#define mutex_lock(x) assert(!pthread_mutex_lock(x))
#define mutex_unlock(x) assert(!pthread_mutex_unlock(x))
struct axiadc_state { int unused; };
struct iio_dev { pthread_mutex_t mlock; };
struct axiadc_converter { struct iio_dev *indio_dev; pthread_mutex_t lock;
 bool counter_capture_owned; u32 counter_capture_scan_mask; };
struct clk { int unused; };
struct spi_device { int unused; };
struct ad9361_rf_phy_state {
 struct { u8 current_profile[2]; struct { u8 flags, alc_written, alc_orig; } entry[2][8]; } fastlock;
};
struct platform_data { unsigned tx_fastlock_delay_ns, rx_fastlock_delay_ns; bool trx_fastlock_pinctrl_en[2]; };
struct ad9361_rf_phy {
 struct ad9361_rf_phy_state *state;
 struct platform_data *pdata;
 struct spi_device *spi;
 struct clk *clks[2];
 pthread_mutex_t lock;
 bool counter_owned, counter_scan_configured, counter_scan_restore_required, counter_bank_configured;
 void *tandem_owner;
 u32 counter_scan_profile_mask;
 u32 counter_previous_control, counter_scan_mask;
 u64 counter_scan_frequency_hz[8], counter_previous_rx_lo_hz;
 u32 counter_scan_profile_crc[8];
 struct adi_rx_counter_bank_profile counter_bank[24];
 u32 counter_bank_target[8];
 u64 counter_bank_generation[8];
 struct adi_rx_counter_diagnostics counter_diagnostics;
 u64 counter_diag_session, counter_diag_visit;
 u32 counter_diag_stage, counter_diag_profile;
};
static struct iio_dev indio = { .mlock = PTHREAD_MUTEX_INITIALIZER };
static struct axiadc_converter conv = { .indio_dev = &indio, .lock = PTHREAD_MUTEX_INITIALIZER };
static struct axiadc_state adc;
static struct ad9361_rf_phy_state state;
static struct platform_data pdata;
static struct ad9361_rf_phy phy = { .lock = PTHREAD_MUTEX_INITIALIZER };
static int owner, stranger;
static int regs[1024];
static u8 profiles[8][16];
static int writes, reads, fail_write_at, fail_read_at, corrupt_word;
static bool buffer_enabled, unlocked, stuck_selector, restoring;
static u32 counter;
static ktime_t now;
static pthread_mutex_t gate_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t gate_cond = PTHREAD_COND_INITIALIZER;
static unsigned pause_write_at;
static bool load_paused, allow_load, release_started, release_finished;
static u32 control;
static void *spi_get_drvdata(void *x) { return &conv; }
static void *iio_priv(void *x) { return &adc; }
static bool iio_buffer_enabled(void *x) { return buffer_enabled; }
static struct clk *clk_get_parent(struct clk *x) { return x; }
static unsigned long clk_get_rate(struct clk *x) { return 80000000; }
static unsigned long ad9361_to_clk(u64 hz) { return hz >> 1; }
static u64 ad9361_from_clk(unsigned long hz) { return (u64)hz << 1; }
static int clk_set_rate(struct clk *x, unsigned long hz)
{
 assert(hz == 1000000000);
 regs[REG_RX_INTEGER_BYTE_0] = 100;
 regs[REG_RX_INTEGER_BYTE_1] = 0;
 regs[REG_RX_FRACT_BYTE_0] = regs[REG_RX_FRACT_BYTE_1] = regs[REG_RX_FRACT_BYTE_2] = 0;
 regs[REG_RFPLL_DIVIDERS] = 1;
 restoring = true;
 return 0;
}
static ktime_t ktime_get(void) { return now; }
static u64 ktime_get_ns(void) { return now * 1000; }
static ktime_t ktime_add_us(ktime_t t, int us) { return t + us; }
static int ktime_compare(ktime_t a, ktime_t b) { return (a > b) - (a < b); }
static void usleep_range(int low, int high) { now += high; }
static u32 crc32_le(u32 crc, const u8 *data, unsigned len)
{
 for (unsigned i = 0; i < len; i++) {
  crc ^= data[i];
  for (unsigned bit = 0; bit < 8; bit++)
   crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320U : 0);
 }
 return crc;
}
static void register_write(u32 reg, u32 value)
{
 if (reg == REG_RX_FAST_LOCK_SETUP && value && stuck_selector)
  return;
 regs[reg] = value;
 if (reg == REG_RX_FAST_LOCK_PROGRAM_CTRL && (value & RX_FAST_LOCK_PROGRAM_WRITE)) {
  unsigned address = regs[REG_RX_FAST_LOCK_PROGRAM_ADDR];
  profiles[(address >> 4) & 7][address & 15] = regs[REG_RX_FAST_LOCK_PROGRAM_DATA];
 }
}
static int ad9361_spi_write(struct spi_device *spi, u32 reg, u32 value)
{
 writes++;
 if (writes == fail_write_at) return -EREMOTEIO;
 register_write(reg, value);
 return 0;
}
static int ad9361_spi_writem(struct spi_device *spi, u32 reg, u8 *buf, unsigned len)
{
 writes++;
 if (writes == pause_write_at) {
  assert(!pthread_mutex_lock(&gate_lock));
  load_paused = true; assert(!pthread_cond_broadcast(&gate_cond));
  while (!allow_load) assert(!pthread_cond_wait(&gate_cond, &gate_lock));
  assert(!pthread_mutex_unlock(&gate_lock));
 }
 if (writes == fail_write_at) return -EREMOTEIO;
 for (unsigned i = 0; i < len; i++) register_write(reg - i, buf[i]);
 return 0;
}
static int ad9361_spi_read(struct spi_device *spi, u32 reg)
{
 reads++;
 if (reads == fail_read_at) return -EREMOTEIO;
 if (reg == REG_RX_FAST_LOCK_PROGRAM_READ) {
  unsigned address = regs[REG_RX_FAST_LOCK_PROGRAM_ADDR];
  int value = profiles[(address >> 4) & 7][address & 15];
  return (int)(address & 15) == corrupt_word ? value ^ 1 : value;
 }
 if (reg == REG_RX_CP_OVERRANGE_VCO_LOCK) return unlocked && !restoring ? 0 : VCO_LOCK;
 return regs[reg];
}
static int ad9361_spi_readm(struct spi_device *spi, u32 reg, u8 *buf, int len)
{
 for (int i = 0; i < len; i++) {
  int value = ad9361_spi_read(spi, reg - i);
  if (value < 0) return value;
  buf[i] = value;
 }
 return 0;
}
static int ad9361_spi_readf(struct spi_device *spi, u32 reg, u32 mask)
{
 int value = ad9361_spi_read(spi, reg);
 return value < 0 ? value : (value & mask) >> __builtin_ctz(mask);
}
static int ad9361_spi_writef(struct spi_device *spi, u32 reg, u32 mask, u32 value)
{
 return ad9361_spi_write(spi, reg, (regs[reg] & ~mask) | ((value << __builtin_ctz(mask)) & mask));
}
static int ad9361_trx_vco_cal_control(struct ad9361_rf_phy *p, bool tx, bool enable) { return 0; }
static u32 axiadc_read(struct axiadc_state *a, u32 reg)
{
 if (reg == ADI_REG_GP_CONTROL) return control;
 counter += 100; return counter;
}
static void axiadc_write(struct axiadc_state *a, u32 reg, u32 value)
{ assert(reg == ADI_REG_GP_CONTROL); control = value; }
