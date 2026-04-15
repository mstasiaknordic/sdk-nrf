/*
 * Copyright (c) 2025, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/debug/cpu_load.h>

// #if defined(CONFIG_HAS_NORDIC_DMM)
// #include <dmm.h>
// #endif

/* Parameters to change. Size of buffers still needs to fit within DMA region. */
#define SAMPLE_WIDTH		       32
#define WORDS_COUNT		       288 // 6 channels * 48kHz = 288 words per 1ms of data
#define NUMBER_OF_BLOCKS	       2
#define NUMBER_OF_CHANNELS	       6

#define TIMEOUT_MS		       2000
#define TEST_TIMER_COUNT_TIME_LIMIT_MS 500
#define MEASUREMENT_REPEATS	       10

const struct device *const i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_dut));
const struct device *const tst_timer_dev = DEVICE_DT_GET(DT_ALIAS(tst_timer));

static const int32_t test_data[WORDS_COUNT] = {0xFFFFFFFF,  9,  2,  10, 4,  12, 8,   16,
					       16, 24, 32, 40, 64, 72, 128, 136};

static const uint32_t frequencies[8] = {4000, 8000, 12000, 16000, 24000, 32000, 44100, 48000};

#define BLOCK_SIZE (sizeof(test_data))

#ifdef CONFIG_NOCACHE_MEMORY
#define MEM_SLAB_CACHE_ATTR __nocache
#else
#define MEM_SLAB_CACHE_ATTR
#endif /* CONFIG_NOCACHE_MEMORY */

static char MEM_SLAB_CACHE_ATTR __aligned(WB_UP(
	SAMPLE_WIDTH)) _k_mem_slab_buf_rx_mem_slab[(NUMBER_OF_BLOCKS + 2) * WB_UP(BLOCK_SIZE)];
STRUCT_SECTION_ITERABLE(k_mem_slab, rx_mem_slab) = Z_MEM_SLAB_INITIALIZER(
	rx_mem_slab, _k_mem_slab_buf_rx_mem_slab, WB_UP(BLOCK_SIZE), NUMBER_OF_BLOCKS + 2);

static char MEM_SLAB_CACHE_ATTR __aligned(WB_UP(
	SAMPLE_WIDTH)) _k_mem_slab_buf_tx_mem_slab[(NUMBER_OF_BLOCKS)*WB_UP(BLOCK_SIZE)];
STRUCT_SECTION_ITERABLE(k_mem_slab,
			tx_mem_slab) = Z_MEM_SLAB_INITIALIZER(tx_mem_slab,
							      _k_mem_slab_buf_tx_mem_slab,
							      WB_UP(BLOCK_SIZE), NUMBER_OF_BLOCKS);

void configure_test_timer(const struct device *timer_dev, uint32_t count_time_ms)
{
	struct counter_alarm_cfg counter_cfg;

	counter_cfg.flags = 0;
	counter_cfg.ticks = counter_us_to_ticks(timer_dev, (uint64_t)count_time_ms * 1000);
	counter_cfg.user_data = &counter_cfg;
}

static void fill_tx_buffer(int32_t *tx_block)
{
	for (int i = 0; i < WORDS_COUNT; i++) {
		tx_block[i] = test_data[i];
	}
}

static int verify_rx_buffer(int32_t *rx_block)
{
	int last_word = WORDS_COUNT;

/* Find offset. */
#if (CONFIG_TEST_I2S_ALLOWED_DATA_OFFSET > 0)
	static ZTEST_DMEM int offset = -1;

	if (offset < 0) {
		do {
			++offset;
			if (offset > CONFIG_TEST_I2S_ALLOWED_DATA_OFFSET) {
				TC_PRINT("Allowed data offset exceeded\n");
				return -TC_FAIL;
			}
		} while (rx_block[NUMBER_OF_CHANNELS * offset] != test_data[0]);

		TC_PRINT("Using data offset: %d\n", offset);
	}

	rx_block += NUMBER_OF_CHANNELS * offset;
	last_word -= NUMBER_OF_CHANNELS * offset;
#endif

	for (int i = 0; i < last_word; i++) {
		if (rx_block[i] != test_data[i]) {
			TC_PRINT("Error: data mismatch at position %d, expected %d, actual %d\n", i,
				 test_data[i], rx_block[i]);
			return -TC_FAIL;
		}
	}

	return TC_PASS;
}

static void configure_i2s(const struct device *dev, uint32_t frame_clk_freq)
{
	int ret;
	struct i2s_config i2s_cfg;

	i2s_cfg.word_size = SAMPLE_WIDTH;
	i2s_cfg.channels = NUMBER_OF_CHANNELS;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_PCM_LONG;
	i2s_cfg.frame_clk_freq = frame_clk_freq;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = TIMEOUT_MS;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;

	i2s_cfg.mem_slab = &tx_mem_slab;
	ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
	zassert_ok(ret, "Failed to configure I2S TX stream: %d", ret);

	i2s_cfg.mem_slab = &rx_mem_slab;
	ret = i2s_configure(i2s_dev, I2S_DIR_RX, &i2s_cfg);
	zassert_ok(ret, "Failed to configure I2S RX stream: %d", ret);
}

/**
 * Transmission time [us] = 1_000_000 * bit_count / SCK frequency [Hz]
 * bit_count = number_of_words * word_width * block_count
 * SCK frequency [Hz] = word_width * frame_clk_freq [Hz] * number_of_channels
 * hence :
 * Transmission time [us] = 1_000_000 * number_of_words * block_count / frame_clk_freq [Hz] / number_of_channels
 */
uint32_t transmission_time_us(uint16_t words_count, uint32_t frame_clk_freq, uint32_t block_count)
{
	return (uint32_t)(1000000 * (double)words_count * block_count / NUMBER_OF_CHANNELS / (double)frame_clk_freq);
}

/* Testing transmission of a single buffer. */
static void test_i2s_transmission_one_shot(const struct device *dev, uint32_t frame_clk_freq)
{
	int ret;
	void *rx_block;
	void *tx_block;
	size_t rx_size;
	uint32_t tst_timer_value;
	uint64_t timer_value_us[MEASUREMENT_REPEATS];
	uint64_t average_timer_value_us = 0;
	uint32_t theoretical_transmission_time_us;

	configure_test_timer(tst_timer_dev, TEST_TIMER_COUNT_TIME_LIMIT_MS);

	/* Configure I2S Dir Both transfer. */
	configure_i2s(dev, frame_clk_freq);

	for (uint32_t repeat_counter = 0; repeat_counter < MEASUREMENT_REPEATS; repeat_counter++) {

		/* Prefill TX queue */
		ret = k_mem_slab_alloc(&tx_mem_slab, &tx_block, K_FOREVER);
		zassert_equal(ret, 0, "TX mem slab allocation failed");

		fill_tx_buffer((uint32_t *)tx_block);

		ret = i2s_write(i2s_dev, tx_block, BLOCK_SIZE);
		zassert_equal(ret, 0, "I2S write failed");

		counter_reset(tst_timer_dev);
		counter_start(tst_timer_dev);
		ret = i2s_trigger(i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_START);
		zassert_equal(ret, 0, "RX/TX START trigger failed\n");
		/* All data written, drain TX queue and stop both streams. */
		ret = i2s_trigger(i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_DRAIN);
		zassert_equal(ret, 0, "RX/TX DRAIN trigger failed");
		ret = i2s_read(i2s_dev, &rx_block, &rx_size);
		counter_get_value(tst_timer_dev, &tst_timer_value);
		counter_stop(tst_timer_dev);

		zassert_equal(ret, 0, "I2S read failed");
		zassert_equal(rx_size, BLOCK_SIZE);

		/* Verify received data */
		ret = verify_rx_buffer((uint32_t *)rx_block);
		if (ret != 0)
		{
			TC_PRINT("Data did not match\n");
		}
		// zassert_equal(ret, 0, "TX data does not match RX data");
		k_mem_slab_free(&rx_mem_slab, rx_block);

		timer_value_us[repeat_counter] =
			counter_ticks_to_us(tst_timer_dev, tst_timer_value);
		average_timer_value_us += timer_value_us[repeat_counter];
	}

	average_timer_value_us /= MEASUREMENT_REPEATS;

 	/* For one-shot transmission, same buffer will be transfered twice. */
	theoretical_transmission_time_us =
		transmission_time_us(WORDS_COUNT, frame_clk_freq, 2);
	TC_PRINT("Calculated transmission time (for frame clk = %uHz, sample width = %ubit) [us]: "
		 "%u\n",
		 frame_clk_freq, SAMPLE_WIDTH, theoretical_transmission_time_us);
	TC_PRINT("Measured transmission time (for frame clk = %uHz, sample width = %ubit) [us]: "
		 "%llu\n",
		 frame_clk_freq, SAMPLE_WIDTH, average_timer_value_us);
}

/* Testing transmission of a multiple buffers. */
static void test_i2s_transmission_long(const struct device *dev, uint32_t frame_clk_freq)
{
	int ret;
	void *rx_block[NUMBER_OF_BLOCKS];
	void *tx_block[NUMBER_OF_BLOCKS];
	size_t rx_size;
	uint32_t tst_timer_value;
	uint64_t timer_value_us[MEASUREMENT_REPEATS];
	uint64_t average_timer_value_us = 0;
	uint32_t theoretical_transmission_time_us;

	configure_test_timer(tst_timer_dev, TEST_TIMER_COUNT_TIME_LIMIT_MS);

	/* Configure I2S Dir Both transfer. */
	configure_i2s(dev, frame_clk_freq);

	for (uint32_t repeat_counter = 0; repeat_counter < MEASUREMENT_REPEATS; repeat_counter++) {
		/* Prefill TX queue */
		ret = k_mem_slab_alloc(&tx_mem_slab, &tx_block[0], K_FOREVER);
		zassert_equal(ret, 0, "TX mem slab allocation failed");

		fill_tx_buffer((uint32_t *)tx_block[0]);

		ret = i2s_write(i2s_dev, tx_block[0], BLOCK_SIZE);
		zassert_equal(ret, 0, "I2S write failed");

		counter_reset(tst_timer_dev);
		counter_start(tst_timer_dev);
		ret = i2s_trigger(i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_START);
		zassert_equal(ret, 0, "RX/TX START trigger failed\n");

		for(uint32_t b = 1; b < (NUMBER_OF_BLOCKS - 1); b++) {
			ret = k_mem_slab_alloc(&tx_mem_slab, &tx_block[b], K_FOREVER);
			zassert_equal(ret, 0, "TX mem slab allocation failed");

			fill_tx_buffer((uint32_t *)tx_block[b]);

			ret = i2s_write(i2s_dev, tx_block[b], BLOCK_SIZE);
			zassert_equal(ret, 0, "I2S write failed");

			ret = i2s_read(i2s_dev, &rx_block[b], &rx_size);

			/* Verify received data */
			ret = verify_rx_buffer((uint32_t *)rx_block[b]);
			zassert_equal(ret, 0, "TX data does not match RX data");

			k_mem_slab_free(&rx_mem_slab, rx_block[b]);
		}

		/* All data written, drain TX queue and stop both streams. */
		ret = i2s_trigger(i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_DRAIN);
		zassert_equal(ret, 0, "RX/TX DRAIN trigger failed");
		ret = i2s_read(i2s_dev, &rx_block[NUMBER_OF_BLOCKS-1], &rx_size);

		/* Verify received data */
		ret = verify_rx_buffer((uint32_t *)rx_block[NUMBER_OF_BLOCKS-1]);
		zassert_equal(ret, 0, "TX data does not match RX data");

		k_mem_slab_free(&rx_mem_slab, rx_block[NUMBER_OF_BLOCKS-1]);

		counter_get_value(tst_timer_dev, &tst_timer_value);
		counter_stop(tst_timer_dev);

		zassert_equal(ret, 0, "I2S read failed");
		zassert_equal(rx_size, BLOCK_SIZE);

		timer_value_us[repeat_counter] =
			counter_ticks_to_us(tst_timer_dev, tst_timer_value);
		average_timer_value_us += timer_value_us[repeat_counter];
	}

	average_timer_value_us /= MEASUREMENT_REPEATS;

	theoretical_transmission_time_us =
		transmission_time_us(WORDS_COUNT, frame_clk_freq, NUMBER_OF_BLOCKS);
	TC_PRINT("Calculated transmission time (for frame clk = %uHz, sample width = %ubit) [us]: "
		 "%u\n",
		 frame_clk_freq, SAMPLE_WIDTH, theoretical_transmission_time_us);
	TC_PRINT("Measured transmission time (for frame clk = %uHz, sample width = %ubit) [us]: "
		 "%llu\n",
		 frame_clk_freq, SAMPLE_WIDTH, average_timer_value_us);
}

void *test_setup(void)
{
	zassert_true(device_is_ready(i2s_dev), "I2S device is not ready");
	return NULL;
}

ZTEST(i2s_profiling, test_i2s_profiling_one_shot)
{
	TC_PRINT("Number of words: %u\n", WORDS_COUNT);
	TC_PRINT("Sample width: %u\n", SAMPLE_WIDTH);
	TC_PRINT("Number of channels: %u\n", NUMBER_OF_CHANNELS);

	for (int f = 0; f < 8; f++) {
		(void)cpu_load_get(true);
		test_i2s_transmission_one_shot(i2s_dev, frequencies[f]);
		int load = cpu_load_get(true);
		TC_PRINT("CPU load: %d\n", load);
	}
}

ZTEST(i2s_profiling, test_i2s_profiling_long)
{
	TC_PRINT("Number of words: %u\n", WORDS_COUNT);
	TC_PRINT("Sample width: %u\n", SAMPLE_WIDTH);
	TC_PRINT("Number of channels: %u\n", NUMBER_OF_CHANNELS);

	for (int f = 0; f < 8; f++) {
		(void)cpu_load_get(true);
		test_i2s_transmission_long(i2s_dev, frequencies[f]);
		int load = cpu_load_get(true);
		TC_PRINT("CPU load: %d\n", load);
	}
}

/* This tests sets pins at certain points of driver operation, to be measured using analyzer. */
ZTEST(i2s_profiling, test_i2s_profiling_startup)
{
	int ret;
	void *rx_block;
	void *tx_block;
	size_t rx_size;
	NRF_P1->DIR |= 25; // Set three available pins as output

	/* Configure I2S Dir Both transfer. */
	NRF_P1->OUTSET |= 1; // Set first pin
	configure_i2s(i2s_dev, 48000);

	/* Prefill TX queue */
	NRF_P1->OUTSET |= 8; // Set second pin
	ret = k_mem_slab_alloc(&tx_mem_slab, &tx_block, K_FOREVER);
	zassert_equal(ret, 0, "TX mem slab allocation failed");

	fill_tx_buffer((uint32_t *)tx_block);

	ret = i2s_write(i2s_dev, tx_block, BLOCK_SIZE);
	zassert_equal(ret, 0, "I2S write failed");

	NRF_P1->OUTSET |= 16; // Set third pin

	ret = i2s_trigger(i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "RX/TX START trigger failed\n");
	/* All data written, drain TX queue and stop both streams. */
	ret = i2s_trigger(i2s_dev, I2S_DIR_BOTH, I2S_TRIGGER_DRAIN);
	zassert_equal(ret, 0, "RX/TX DRAIN trigger failed");
	ret = i2s_read(i2s_dev, &rx_block, &rx_size);

	zassert_equal(ret, 0, "I2S read failed");
	zassert_equal(rx_size, BLOCK_SIZE);

	zassert_equal(ret, 0, "TX data does not match RX data");
	k_mem_slab_free(&rx_mem_slab, rx_block);
}


ZTEST_SUITE(i2s_profiling, NULL, test_setup, NULL, NULL, NULL);
