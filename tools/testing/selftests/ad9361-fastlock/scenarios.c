static void reset(void)
{
 memset(&phy,0,sizeof(phy)); memset(regs,0,sizeof(regs)); memset(words,0,sizeof(words));
 fail_reg=fail_word=-1; fail_readm=recall_error=restore_error=0;
 crc_fail_at=crc_reads=restored=counters=lock_reads=lock_delay=0;
 unlocked=stuck_selector=late_lock=restoring=false; now=0; nevents=0;
 phy.counter_owned=phy.counter_scan_configured=true; phy.tandem_owner=&owner;
 phy.counter_scan_profile_mask=0xf;
 for(int i=0;i<8;i++) {phy.counter_scan_frequency_hz[i]=1000000000; phy.counter_scan_profile_crc[i]=crc;}
 phy.counter_previous_rx_lo_hz=2000000000;
 /* Stored target: 80 MHz * 100 / 8 = 1 GHz. Conventional registers: 2 GHz. */
 words[0]=100; words[12]=2;
 regs[REG_RX_INTEGER_BYTE_0]=100; regs[REG_RFPLL_DIVIDERS]=1;
 regs[REG_RX_CP_OVERRANGE_VCO_LOCK]=VCO_LOCK;
}
static int recall(u32 profile, u64 *hz, u32 *after)
{
 u32 before=0, result_crc=0;
 return ad9361_counter_fastlock_recall(&phy,&owner,profile,hz,&result_crc,&before,after);
}
static void failed(int expected)
{
 u64 hz=777; u32 after=888;
 assert(recall(0,&hz,&after)==expected);
 assert(hz==777 && after==888);
 assert(!phy.counter_scan_configured && !phy.counter_scan_profile_mask);
 assert(restored==1);
 if(!restore_error && !late_lock) assert(!phy.counter_scan_restore_required);
}
static int lowlevel(void)
{
 struct ad9361_rf_phy_state state={0};struct platform_data pdata={0};
 int tests=0;
 for(int i=1;i<=5;i++) {
  reset();memset(&state,0,sizeof(state));phy.state=&state;phy.pdata=&pdata;
  write_count=0;write_fail_at=i;
  assert(checked_prepare(&phy,false,3,true)==-EREMOTEIO);
  assert(write_count==5 && state.fastlock.current_profile[0]==4);tests++;
 }
 for(int i=1;i<=7;i++) {
  reset();phy.state=&state;phy.pdata=&pdata;state.fastlock.current_profile[0]=4;
  write_count=0;write_fail_at=i;
  assert(checked_prepare(&phy,false,3,false)==-EREMOTEIO);
  assert(write_count==7 && state.fastlock.current_profile[0]==4);tests++;
 }
 reset();memset(&state,0,sizeof(state));phy.state=&state;phy.pdata=&pdata;
 state.fastlock.entry[0][0].flags=FASTLOOK_INIT;
 alc_read_error=1;write_count=0;
 assert(checked_recall(&phy,false,0)==-EREMOTEIO && write_count==0);tests++;
 alc_read_error=0;alc_write_error=1;
 assert(checked_recall(&phy,false,0)==-EREMOTEIO && write_count==0);tests++;
 alc_write_error=0;write_fail_at=0;
 assert(checked_recall(&phy,false,0)==0 && state.fastlock.current_profile[0]==1);tests++;
 write_count=0;write_fail_at=1;
 assert(checked_recall(&phy,false,0)==-EREMOTEIO);tests++;
 write_fail_at=0;write_count=0;
 assert(checked_prepare(&phy,false,0,false)==0 && !state.fastlock.current_profile[0]);tests++;
 return tests;
}
int main(void)
{
 int tests=0;
 /* Stale conventional registers do not reject RF-confirmed working Fast Lock. */
 reset(); u64 hz=0; u32 after=0;
 assert(recall(0,&hz,&after)==0 && hz==1000000000 && after==2);
 assert(!restored && phy.counter_scan_configured && crc_reads==2);
 assert(events[nevents-1]==E_COUNTER); tests++;
 /* Same profile still gets a checked receipt when the kernel is called. */
 nevents=0; counters=0; assert(recall(0,&hz,&after)==0 && after==2); tests++;
 nevents=0; counters=0; assert(recall(3,&hz,&after)==0 && after==2); tests++;
 reset(); unlocked=true; failed(-ETIMEDOUT); assert(now<=2040 && lock_reads>1); tests++;
 reset(); late_lock=true; failed(-EIO); tests++; /* restoration also misses deadline */
 reset(); lock_delay=3; assert(recall(0,&hz,&after)==0 && now==120); tests++;
 reset(); fail_reg=REG_RX_CP_OVERRANGE_VCO_LOCK; failed(-EREMOTEIO); tests++;
 reset(); fail_reg=REG_RX_FAST_LOCK_SETUP; failed(-EREMOTEIO); tests++;
 for(int i=0;i<6;i++) { reset(); fail_word=i<5?i:12; failed(-EREMOTEIO); tests++; }
 reset(); stuck_selector=true; regs[REG_RX_FAST_LOCK_SETUP]=RX_FAST_LOCK_PROFILE(1)|1; failed(-EIO); tests++;
 reset(); stuck_selector=true; regs[REG_RX_FAST_LOCK_SETUP]=1|RX_FAST_LOCK_PROFILE_PIN_SELECT; failed(-EIO); tests++;
 reset(); phy.counter_scan_frequency_hz[0]+=3; failed(-EIO); tests++;
 reset(); phy.counter_scan_frequency_hz[0]+=2; assert(recall(0,&hz,&after)==0); tests++;
 reset(); crc_fail_at=1; failed(-EREMOTEIO); tests++;
 reset(); crc_fail_at=2; failed(-EREMOTEIO); tests++;
 reset(); phy.counter_scan_profile_crc[0]^=1; failed(-ESTALE); tests++;
 reset(); recall_error=-EREMOTEIO; failed(-EREMOTEIO); tests++;
 reset(); fail_word=0; restore_error=-EREMOTEIO; failed(-EIO); assert(phy.counter_scan_restore_required); tests++;
 /* Direct register restoration rejects failures and wrong actual frequency. */
 reset(); fail_readm=1; assert(ad9361_counter_restore_rx_lo(&phy)==-EREMOTEIO); tests++;
 reset(); regs[REG_RX_INTEGER_BYTE_0]=99; assert(ad9361_counter_restore_rx_lo(&phy)==-EIO); tests++;
 reset(); assert(ad9361_counter_read_rx_lo(&phy,&hz)==0 && hz==2000000000); tests++;
 reset(); fail_reg=REG_RFPLL_DIVIDERS; assert(ad9361_counter_read_rx_lo(&phy,&hz)==-EREMOTEIO); tests++;
 reset(); regs[REG_RX_FAST_LOCK_SETUP]=1; assert(ad9361_counter_read_rx_lo(&phy,&hz)==-EIO); tests++;
 /* Known noninteger hardware vector; source arithmetic retains sub-2Hz precision. */
 reset(); words[0]=95;words[2]=0xf1;words[3]=0xff;words[4]=0x7b;
 regs[REG_RX_FAST_LOCK_SETUP]=1;
 assert(ad9361_counter_profile_rx_lo(&phy,0,&hz)==0 && hz==959687499); tests++;
 tests+=lowlevel();
 printf("PASS: %d production-code fault-injection scenarios\n",tests);
 return 0;
}
