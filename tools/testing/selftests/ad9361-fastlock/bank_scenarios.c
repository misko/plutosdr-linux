/* SPDX-License-Identifier: GPL-2.0 */
static struct adi_rx_counter_bank_config config;
static unsigned tests;
static void reset_bank(void)
{
 memset(&state, 0, sizeof(state));
 memset(&pdata, 0, sizeof(pdata));
 memset(regs, 0, sizeof(regs));
 memset(profiles, 0, sizeof(profiles));
 memset(&phy.counter_diagnostics, 0, sizeof(phy.counter_diagnostics));
 memset(&config, 0, sizeof(config));
 memset(phy.counter_bank_generation, 0, sizeof(phy.counter_bank_generation));
 phy.state = &state; phy.pdata = &pdata;
 phy.counter_owned = true; phy.tandem_owner = &owner;
 phy.counter_bank_configured = phy.counter_scan_configured = false;
 phy.counter_scan_restore_required = false; phy.counter_scan_profile_mask = 0;
 phy.counter_previous_control = 64; phy.counter_scan_mask = 15;
 conv.counter_capture_owned = true; conv.counter_capture_scan_mask = 15;
 phy.counter_previous_rx_lo_hz = 2000000000;
 writes = reads = fail_write_at = fail_read_at = 0;
 corrupt_word = -1;
 buffer_enabled = unlocked = stuck_selector = restoring = false;
 now = counter = 0;
 pause_write_at = 0; control = 128;
 load_paused = allow_load = release_started = release_finished = false;
 config.magic = ADI_RX_COUNTER_MAGIC;
 config.version = ADI_RX_COUNTER_BANK_VERSION;
 config.size = sizeof(config); config.target_count = 24;
 for (unsigned i = 0; i < 24; i++) {
  struct adi_rx_counter_bank_profile *p = &config.profiles[i];
  p->frequency_hz = 1000000000ULL + i * 10000000ULL;
  p->values[0] = 100 + i; p->values[12] = 2; p->values[15] = 10;
  p->crc32 = crc32_le(~0U, p->values, sizeof(p->values)) ^ ~0U;
 }
}
static void prepare_bank(void)
{
 reset_bank();
 assert(!ad9361_counter_configure_bank(&phy, &owner, &config));
 assert(phy.counter_bank_configured && phy.counter_scan_configured);
 assert(!state.fastlock.current_profile[0] && !phy.counter_scan_restore_required);
 for (unsigned i = 0; i < 8; i++) {
  assert(phy.counter_bank_target[i] == i && phy.counter_bank_generation[i] == 1);
  assert(!memcmp(profiles[i], config.profiles[i].values, 15));
 }
 writes = reads = 0; restoring = false;
}
static struct adi_rx_counter_bank_recall request(unsigned target)
{
 return (struct adi_rx_counter_bank_recall) {
  .magic = ADI_RX_COUNTER_MAGIC, .version = ADI_RX_COUNTER_BANK_VERSION,
  .size = sizeof(struct adi_rx_counter_bank_recall), .logical_target = target,
  .profile = target % 8, .expected_generation = phy.counter_bank_generation[target % 8],
  .requested_frequency_hz = config.profiles[target].frequency_hz,
 };
}
static void failure(unsigned target, int expected)
{
 struct adi_rx_counter_bank_recall r = request(target), original = r;
 u64 generation = phy.counter_bank_generation[r.profile];
 assert(ad9361_counter_bank_recall(&phy, &owner, &r) == expected);
 assert(!memcmp(&r, &original, sizeof(r)));
 assert(phy.counter_bank_generation[r.profile] == generation);
 assert(!phy.counter_bank_configured && !phy.counter_scan_configured);
 assert(!phy.counter_scan_profile_mask);
 assert(!state.fastlock.current_profile[0]);
 assert(!regs[REG_RX_FAST_LOCK_PROGRAM_CTRL]);
 assert(phy.counter_diagnostics.first_failure.error == expected);
 tests++;
}
static struct adi_rx_counter_bank_recall concurrent_request;
static int concurrent_recall_result, concurrent_release_result;
static void *recall_thread(void *ignored)
{
 concurrent_recall_result = ad9361_counter_bank_recall(&phy, &owner, &concurrent_request);
 return NULL;
}
static void *release_thread(void *ignored)
{
 assert(!pthread_mutex_lock(&gate_lock));
 release_started = true; assert(!pthread_cond_broadcast(&gate_cond));
 assert(!pthread_mutex_unlock(&gate_lock));
 concurrent_release_result = __ad9361_counter_release(&phy, &owner, NULL, NULL, NULL);
 assert(!pthread_mutex_lock(&gate_lock));
 release_finished = true; assert(!pthread_cond_broadcast(&gate_cond));
 assert(!pthread_mutex_unlock(&gate_lock));
 return NULL;
}
static void concurrent_stop(void)
{
 pthread_t recalling, releasing;
 prepare_bank(); concurrent_request = request(8); pause_write_at = 20;
 assert(!pthread_create(&recalling, NULL, recall_thread, NULL));
 assert(!pthread_mutex_lock(&gate_lock));
 while (!load_paused) assert(!pthread_cond_wait(&gate_cond, &gate_lock));
 /* The partial load has neither changed the mapping nor published a receipt. */
 assert(phy.counter_bank_target[0] == 0 && phy.counter_bank_generation[0] == 1);
 assert(!concurrent_request.slot_generation);
 assert(!pthread_mutex_unlock(&gate_lock));
 assert(!pthread_create(&releasing, NULL, release_thread, NULL));
 assert(!pthread_mutex_lock(&gate_lock));
 while (!release_started) assert(!pthread_cond_wait(&gate_cond, &gate_lock));
 assert(!release_finished && control == 128);
 allow_load = true; assert(!pthread_cond_broadcast(&gate_cond));
 assert(!pthread_mutex_unlock(&gate_lock));
 assert(!pthread_join(recalling, NULL)); assert(!pthread_join(releasing, NULL));
 assert(!concurrent_recall_result && !concurrent_release_result);
 assert(concurrent_request.slot_generation == 2);
 assert(!phy.counter_owned && !phy.counter_bank_configured && !phy.counter_scan_configured);
 assert(!conv.counter_capture_owned && !conv.counter_capture_scan_mask);
 assert(!state.fastlock.current_profile[0] && control == 64);
 tests++;
}
int main(void)
{
 _Static_assert(sizeof(struct adi_rx_counter_bank_profile) == 32, "profile ABI");
 _Static_assert(sizeof(struct adi_rx_counter_bank_config) == 784, "bank ABI");
 _Static_assert(sizeof(struct adi_rx_counter_bank_caps) == 32, "caps ABI");
 _Static_assert(sizeof(struct adi_rx_counter_bank_recall) == 80, "recall ABI");
 _Static_assert(sizeof(struct adi_rx_counter_scan_config) == 144, "legacy configure ABI");
 _Static_assert(sizeof(struct adi_rx_counter_scan_recall) == 48, "legacy recall ABI");
 _Static_assert(ADI_RX_COUNTER_IOC_CONFIGURE_SCAN == 0x40905421UL, "legacy configure ioctl");
 _Static_assert(ADI_RX_COUNTER_IOC_RECALL == 0xc0305422UL, "legacy recall ioctl");
 _Static_assert(ADI_RX_COUNTER_IOC_GET_BANK_CAPS == 0x80205428UL, "bank caps ioctl");
 _Static_assert(ADI_RX_COUNTER_IOC_CONFIGURE_BANK == 0x43105429UL, "bank configure ioctl");
 _Static_assert(ADI_RX_COUNTER_IOC_REPLACE_RECALL == 0xc050542aUL, "bank recall ioctl");
 /* Every write transaction and readback-address write in the real loader. */
 for (int fail = 1; fail <= 34; fail++) {
  reset_bank(); fail_write_at = fail;
  assert(ad9361_fastlock_load(&phy, false, 0, config.profiles[0].values) == -EREMOTEIO);
  assert(state.fastlock.entry[0][0].flags != FASTLOOK_INIT); tests++;
 }
 for (int fail = 1; fail <= 16; fail++) {
  reset_bank(); fail_read_at = fail;
  assert(ad9361_fastlock_load(&phy, false, 0, config.profiles[0].values) == -EREMOTEIO);
  assert(state.fastlock.entry[0][0].flags != FASTLOOK_INIT); tests++;
 }
 for (int word = 0; word < 16; word++) {
  reset_bank(); corrupt_word = word;
  assert(ad9361_fastlock_load(&phy, false, 0, config.profiles[0].values) == -EIO);
  assert(state.fastlock.entry[0][0].flags != FASTLOOK_INIT); tests++;
 }
 /* Invalid bank inputs fail before any radio writes. */
 reset_bank(); config.profiles[23].crc32 ^= 1;
 assert(ad9361_counter_configure_bank(&phy, &owner, &config) == -EINVAL && !writes); tests++;
 reset_bank(); config.profiles[23].frequency_hz += 3;
 assert(ad9361_counter_configure_bank(&phy, &owner, &config) == -ERANGE && !writes); tests++;
 reset_bank(); buffer_enabled = true;
 assert(ad9361_counter_configure_bank(&phy, &owner, &config) == -EBUSY && !writes); tests++;
 reset_bank(); assert(ad9361_counter_configure_bank(&phy, &stranger, &config) == -EPERM && !writes); tests++;
 /* Preparation faults never publish a usable mapping. */
 reset_bank(); fail_write_at = 4;
 assert(ad9361_counter_configure_bank(&phy, &owner, &config) == -EREMOTEIO);
 assert(!phy.counter_bank_configured && !phy.counter_scan_profile_mask); tests++;
 /* Several complete wraps; a delayed receipt remains immutable after reuse. */
 prepare_bank(); struct adi_rx_counter_bank_recall saved = {0};
 for (unsigned visit = 0; visit < 240; visit++) {
  struct adi_rx_counter_bank_recall r = request(visit % 24);
  u64 expected = visit < 8 ? 1 : visit / 8 + 1;
  assert(!ad9361_counter_bank_recall(&phy, &owner, &r));
  assert(r.slot_generation == expected);
  assert(r.frequency_hz == config.profiles[visit % 24].frequency_hz);
  assert(r.profile_identity_crc32 == config.profiles[visit % 24].crc32);
  assert(r.counter_before <= r.load_after && r.load_after <= r.recall_before && r.recall_before < r.counter_after);
  if (!visit) saved = r;
  assert(saved.logical_target == 0 && saved.slot_generation == 1 && saved.frequency_hz == 1000000000);
 }
 tests++;
 /* Resident recall does not replace an image or increase its generation. */
 prepare_bank(); regs[REG_RX_FORCE_ALC] = 5;
 struct adi_rx_counter_bank_recall r = request(0);
 assert(!ad9361_counter_bank_recall(&phy, &owner, &r) && r.slot_generation == 1); tests++;
 assert(r.profile_identity_crc32 != r.profile_crc32); tests++; /* valid ALC change */
 prepare_bank(); r = request(0); r.expected_generation++;
 assert(ad9361_counter_bank_recall(&phy, &owner, &r) == -ESTALE);
 assert(!phy.counter_bank_configured && !phy.counter_scan_profile_mask); tests++;
 prepare_bank(); r = request(8); r.requested_frequency_hz++;
 assert(ad9361_counter_bank_recall(&phy, &owner, &r) == -ERANGE);
 assert(!phy.counter_bank_configured); tests++;
 prepare_bank(); profiles[0][3] ^= 1; failure(8, -ESTALE);
 prepare_bank(); fail_write_at = 20; failure(8, -EREMOTEIO);
 /* Final programming-clock disable failure is retried by fault restoration. */
 prepare_bank(); fail_write_at = 34; failure(8, -EREMOTEIO);
 prepare_bank(); fail_read_at = 20; failure(8, -EREMOTEIO);
 prepare_bank(); unlocked = true; failure(8, -ETIMEDOUT); assert(now <= 2040);
 prepare_bank(); stuck_selector = true; failure(9, -EIO);
 prepare_bank(); phy.counter_bank_generation[0] = ~0ULL; failure(8, -EOVERFLOW);
 prepare_bank(); r = request(0);
 assert(ad9361_counter_bank_recall(&phy, &stranger, &r) == -EPERM && !writes);
 assert(phy.counter_bank_configured); tests++;
 prepare_bank(); r = request(0); r.logical_target = 24;
 assert(ad9361_counter_bank_recall(&phy, &owner, &r) == -EINVAL && !writes); tests++;
 /* Counters deliberately wrap; userspace extends these coherent low words. */
 prepare_bank(); counter = UINT32_MAX - 250; r = request(8);
 assert(!ad9361_counter_bank_recall(&phy, &owner, &r));
 assert(r.counter_before > r.counter_after); tests++;
 concurrent_stop();
 /* A STOP during userspace's guard releases and restores the same lease. */
 prepare_bank(); r = request(8);
 assert(!ad9361_counter_bank_recall(&phy, &owner, &r));
 assert(!__ad9361_counter_release(&phy, &owner, NULL, NULL, NULL));
 assert(!phy.counter_owned && !phy.counter_bank_configured && control == 64); tests++;
 puts("PASS: 24-target bank, checked SPI writes/readback, CRC/generation/IF/lock faults, ALC and counter rollover");
 printf("PASS: %u production bank scenarios including 240 ordered visits\n", tests);
 return 0;
}
