void watchdog_timer_init(uint32_t timeout_ms);
bool crash_log_dump_flash_history(char *out_buf, size_t out_buf_len);
bool read_crash_log_on_boot(char *out_buf, size_t out_buf_len);
void watchdog_kick(void);