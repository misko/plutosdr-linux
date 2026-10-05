/* SPDX-License-Identifier: GPL-2.0 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "ad9361_regs.h"
#include "adi_rx_counter.h"
typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int64_t ktime_t;
#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#define BIT(x) (1U << (x))
#define do_div(n,d) ((n) /= (d))
#define RX_RFPLL_INT 0
#define RX_RFPLL 1
#define ADI_REG_GP_STATUS 0xb8
#define mutex_lock(x) ((void)(x))
#define mutex_unlock(x) ((void)(x))
struct axiadc_state { int unused; };
struct iio_dev { int mlock; };
struct axiadc_converter { struct iio_dev *indio_dev; int lock; };
struct clk { int unused; };
struct spi_device { int unused; };
struct ad9361_rf_phy_state {
 struct { u8 current_profile[2]; struct { u8 flags, alc_written, alc_orig; } entry[2][8]; } fastlock;
};
struct platform_data { unsigned tx_fastlock_delay_ns, rx_fastlock_delay_ns; bool trx_fastlock_pinctrl_en[2]; };
#define FASTLOOK_INIT 1
#define dev_dbg(...) ((void)0)
struct ad9361_rf_phy { struct ad9361_rf_phy_state *state; struct platform_data *pdata;
 struct spi_device *spi; struct clk *clks[2]; int lock;
 bool counter_owned, counter_scan_configured, counter_scan_restore_required;
 void *tandem_owner; u32 counter_scan_profile_mask;
 u64 counter_scan_frequency_hz[8], counter_previous_rx_lo_hz;
 u32 counter_scan_profile_crc[8];
 struct adi_rx_counter_diagnostics counter_diagnostics;
 u64 counter_diag_session, counter_diag_visit;
 u32 counter_diag_stage, counter_diag_profile;
};
static struct iio_dev indio;
static struct axiadc_converter conv={.indio_dev=&indio};
static struct axiadc_state adc;
static struct ad9361_rf_phy phy;
static int owner;
static int regs[1024], words[16];
static int fail_reg, fail_word, fail_readm, recall_error, restore_error;
static int crc_fail_at, crc_reads, restored, counters, lock_reads, lock_delay;
static bool unlocked, stuck_selector, late_lock, restoring;
static u32 crc=0x12345678;
static ktime_t now;
static int events[256], nevents;
enum { E_RECALL=1, E_LOCK, E_SELECTOR, E_WORD, E_CRC, E_COUNTER, E_RESTORE };
static void event(int e) { assert(nevents<256); events[nevents++]=e; }
static void *spi_get_drvdata(void *x) { return &conv; }
static void *iio_priv(void *x) { return &adc; }
static struct clk *clk_get_parent(struct clk *x) { return x; }
static unsigned long clk_get_rate(struct clk *x) { return 80000000; }
static int clk_set_rate(struct clk *x, unsigned long hz) { return 0; }
static unsigned long ad9361_to_clk(u64 hz) { return hz >> 1; }
static ktime_t ktime_get(void) { return now; }
static u64 ktime_get_ns(void) { return now * 1000; }
static ktime_t ktime_add_us(ktime_t t, int us) { return t+us; }
static int ktime_compare(ktime_t a, ktime_t b) { return (a>b)-(a<b); }
static void usleep_range(int low, int high) { now+=high; }
static int ad9361_spi_read(struct spi_device *spi, u32 reg)
{
 if (reg==REG_RX_CP_OVERRANGE_VCO_LOCK) {
  event(E_LOCK); lock_reads++;
  if (late_lock) now+=2001;
  if (!restoring && (unlocked || lock_reads<=lock_delay)) return 0;
 }
 if (reg==REG_RX_FAST_LOCK_SETUP) event(E_SELECTOR);
 if (!restoring && (int)reg==fail_reg) return -EREMOTEIO;
 return regs[reg];
}
static int ad9361_spi_readm(struct spi_device *spi,u32 reg,u8 *buf,int n)
{
 if (fail_readm) return -EREMOTEIO;
 for (int i=0;i<n;i++) buf[i]=regs[reg-i];
 return 0;
}
static int ad9361_fastlock_readval(struct spi_device *spi,bool tx,u32 profile,u32 word)
{
 event(E_WORD);
 return (int)word==fail_word ? -EREMOTEIO : words[word];
}
static int ad9361_fastlock_prepare(struct ad9361_rf_phy *p,bool tx,u32 profile,bool prepare)
{
 assert(!prepare); event(E_RESTORE); restored++; restoring=true;
 regs[REG_RX_FAST_LOCK_SETUP]=0;
 return restore_error;
}
static int ad9361_fastlock_recall(struct ad9361_rf_phy *p,bool tx,u32 profile)
{
 event(E_RECALL);
 if (!stuck_selector) regs[REG_RX_FAST_LOCK_SETUP]=RX_FAST_LOCK_PROFILE(profile)|1;
 return recall_error;
}
static int ad9361_counter_profile_crc(struct ad9361_rf_phy *p,u32 profile,u32 *result)
{
 event(E_CRC); crc_reads++;
 if (crc_reads==crc_fail_at) return -EREMOTEIO;
 *result=crc;
 return 0;
}
static u32 axiadc_read(struct axiadc_state *a,u32 reg)
{
 event(E_COUNTER); return ++counters;
}

static int write_count, write_fail_at, alc_read_error, alc_write_error;
static int ad9361_spi_write(struct spi_device *spi,u32 reg,u32 value)
{
 write_count++;
 if(write_count==write_fail_at)return -EREMOTEIO;
 regs[reg]=value;return 0;
}
static int ad9361_spi_writef(struct spi_device *spi,u32 reg,u32 mask,u32 value)
{ return ad9361_spi_write(spi,reg,value); }
static int ad9361_spi_readf(struct spi_device *spi,u32 reg,u32 mask)
{ return alc_read_error ? -EREMOTEIO : 0; }
static int ad9361_trx_vco_cal_control(struct ad9361_rf_phy *p,bool tx,bool enable)
{ return ad9361_spi_write(p->spi,0,enable); }
static int ad9361_fastlock_writeval(struct spi_device *spi,bool tx,u32 profile,u32 word,u8 value,bool last)
{ return alc_write_error ? -EREMOTEIO : 0; }
