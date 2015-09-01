/*
 * ALSA I2S driver for Broadcom BCM2835 SoC
 *
 * Author:      Mikko Helin
 *              Copyright (c) 2015
 *
 * Based on
 *      Raspberry Pi PCM I2S ALSA Driver
 *	Author:	Phil Poole
 *		Copyright (c)  2013
 *
 * 	ALSA SoC I2S Audio Layer for Broadcom BCM2708 SoC
 * 	Author:	Florian Meier <florian.meier@koalo.de>
 *		Copyright (c) 2013
 *
 *   This driver uses DMA to transfer the data between the DMA buffer and the BCM2708 PCM peripheral.
 *
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kconfig.h>
#include <linux/ioport.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/bitops.h>
#include <asm/io.h>

#include <sound/asound.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <sound/pcm.h>
#include <sound/info.h>
#include <sound/initval.h>
#include <sound/dmaengine_pcm.h>


MODULE_AUTHOR("Mikko Helin");
MODULE_DESCRIPTION("Raspberry Pi PCM/I2S ALSA driver DMA version");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,PCM I2S}}");

#define BCM2708_PERI_BASE	0x20000000
#define GPIO_BASE			(BCM2708_PERI_BASE + 0x200000)
#define I2S_BASE			(BCM2708_PERI_BASE + 0x203000)
#define CLOCK_BASE			(BCM2708_PERI_BASE + 0x101000)

/* I2S DMA interface */
#define BCM2708_I2S_FIFO_PHYSICAL_ADDR	0x7E203004
#define BCM2708_DMA_DREQ_PCM_TX		2
#define BCM2708_DMA_DREQ_PCM_RX		3

/* BCM2835 PCM interface registers are 32-bit word offsets */
#define BCM2835_PCM_CS_A_REG 	0
#define BCM2835_PCM_FIFO_A_REG	1
#define BCM2835_PCM_MODE_A_REG	2
#define BCM2835_PCM_RXC_A_REG    3
#define BCM2835_PCM_TXC_A_REG    4
#define BCM2835_PCM_DREQ_A_REG   5
#define BCM2835_PCM_INTEN_A_REG  6
#define BCM2835_PCM_INTSTC_A_REG 7
#define BCM2835_PCM_GRAY_REG     8

/* I2S register settings */
#define BCM2835_PCM_STBY		BIT(25)
#define BCM2835_PCM_SYNC		BIT(24)
#define BCM2835_PCM_RXSEX		BIT(23)
#define BCM2835_PCM_RXF		BIT(22)
#define BCM2835_PCM_TXE		BIT(21)
#define BCM2835_PCM_RXD		BIT(20)
#define BCM2835_PCM_TXD		BIT(19)
#define BCM2835_PCM_RXR		BIT(18)
#define BCM2835_PCM_TXW		BIT(17)
#define BCM2835_PCM_CS_RXERR		BIT(16)
#define BCM2835_PCM_CS_TXERR		BIT(15)
#define BCM2835_PCM_RXSYNC		BIT(14)
#define BCM2835_PCM_TXSYNC		BIT(13)
#define BCM2835_PCM_DMAEN		BIT(9)
#define BCM2835_PCM_RXTHR(v)		((v) << 7)
#define BCM2835_PCM_TXTHR(v)		((v) << 5)
#define BCM2835_PCM_RXCLR		BIT(4)
#define BCM2835_PCM_TXCLR		BIT(3)
#define BCM2835_PCM_TXON		BIT(2)
#define BCM2835_PCM_RXON		BIT(1)
#define BCM2835_PCM_EN			(1)

#define BCM2835_PCM_CLKDIS		BIT(28)
#define BCM2835_PCM_PDMN		BIT(27)
#define BCM2835_PCM_PDME		BIT(26)
#define BCM2835_PCM_FRXP		BIT(25)
#define BCM2835_PCM_FTXP		BIT(24)
#define BCM2835_PCM_CLKM		BIT(23)
#define BCM2835_PCM_CLKI		BIT(22)
#define BCM2835_PCM_FSM		BIT(21)
#define BCM2835_PCM_FSI		BIT(20)
#define BCM2835_PCM_FLEN(v)		((v) << 10)
#define BCM2835_PCM_FSLEN(v)		(v)

#define BCM2835_PCM_CHWEX		BIT(15)
#define BCM2835_PCM_CHEN		BIT(14)
#define BCM2835_PCM_CHPOS(v)		((v) << 4)
#define BCM2835_PCM_CHWID(v)		(v)
#define BCM2835_PCM_CH1(v)		((v) << 16)
#define BCM2835_PCM_CH2(v)		(v)

#define BCM2835_PCM_TX_PANIC(v)	((v) << 24)
#define BCM2835_PCM_RX_PANIC(v)	((v) << 16)
#define BCM2835_PCM_TX(v)		((v) << 8)
#define BCM2835_PCM_RX(v)		(v)

#define BCM2835_PCM_INT_RXERR		BIT(3)
#define BCM2835_PCM_INT_TXERR		BIT(2)
#define BCM2835_PCM_INT_RXR		BIT(1)
#define BCM2835_PCM_INT_TXW		BIT(0)

const char *i2s_register_name[] = { "CS_A", "FIFO_A", "MODE_A",
		"BCM2835_PCM_RXC_A_REG", "TXC_A", "BCM2835_PCM_DREQ_A_REG",
		"BCM2835_PCM_INTEN_A_REG", "BCM2835_PCM_INTSTC_A_REG",
		"BCM2835_PCM_GRAY_REG" };

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;

unsigned volatile int *i2s_registers;
unsigned volatile int *gpio;
unsigned volatile int *clock_registers;

/*  struct dmaengine_pcm_runtime_data is defined only in sound/core/pcm_dmaengine.c, would need refactoring. */
struct dmaengine_pcm_runtime_data {
	struct dma_chan *dma_chan;
	dma_cookie_t cookie;

	unsigned int pos;
	// unsigned int counter;
};

struct platform_device *snd_pi_i2s_device;

struct pi_i2s_model {
	const char *name;
	int (*playback_constraints)(struct snd_pcm_runtime *runtime);
	int (*capture_constraints)(struct snd_pcm_runtime *runtime);
	u64 formats;
	size_t buffer_bytes_max;
	size_t period_bytes_min;
	size_t period_bytes_max;
	unsigned int periods_min;
	unsigned int periods_max;
	unsigned int rates;
	unsigned int rate_min;
	unsigned int rate_max;
	unsigned int channels_min;
	unsigned int channels_max;

};

struct snd_pi_i2s_chip {
	struct snd_card *card;
	struct pi_i2s_model *model;
	struct snd_pcm *pcm;
	struct snd_pcm_hardware pcm_hw;
	struct snd_dmaengine_dai_dma_data dma_data[2];
	struct dma_chan dma_chan[2];
};

struct pi_i2s_model model_tda1541 = {
		.name = "tda1541",
		.buffer_bytes_max = (32	* 4096),
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 2,
		.channels_max = 2,
		.periods_min = 2,
		.periods_max = 4, };

struct pi_i2s_model model_pcm51xx = {
		.name = "pcm51xx",
		.buffer_bytes_max = (2 * 4096),
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.channels_min = 2,
		.channels_max = 2,
		.periods_min = 2,
		.periods_max = 2, };

static struct snd_pcm_hardware pi_i2s_pcm_hardware = {
		.info = (SNDRV_PCM_INFO_MMAP |
				 SNDRV_PCM_INFO_MMAP_VALID |SNDRV_PCM_INFO_INTERLEAVED
			| SNDRV_PCM_INFO_JOINT_DUPLEX), //| SNDRV_PCM_INFO_BATCH),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE |
		SNDRV_PCM_FMTBIT_S24_LE |
		SNDRV_PCM_FMTBIT_S32_LE), .rates = (SNDRV_PCM_RATE_8000_192000),
		.rate_min = 8000, .rate_max = 192000, .channels_min = 2,
		.channels_max = 2,
		.period_bytes_min = 4 * 4096,  // => 1024 frames
		.period_bytes_max = 4 * 4096, .buffer_bytes_max = (64 * 4096),
		.periods_min = 2, .periods_max = 4, .fifo_size = 64, };

void bcm2835_pcm_enable_capture(void);
void bcm2835_pcm_enable_playback(void);
void bcm2835_pcm_disable_capture(void);
void bcm2835_pcm_disable_playback(void);
bool rpi_dma_filter_fn(struct dma_chan *chan, void *filter_param);
void bcm2835_pcm_clear_fifos(void);
int get_mode(void);

static int pi_i2s_pcm_open(struct snd_pcm_substream *substream) {

	struct dma_chan *chan;
	struct snd_pi_i2s_chip *chip = substream->private_data;
	struct snd_pcm_runtime *rt = substream->runtime;

	int filter = 2;

	printk(KERN_INFO "pi_i2s_pcm_open: open callback\n");

	rt->hw = pi_i2s_pcm_hardware;

	printk(KERN_INFO "pi_i2s_pcm_open: Runtime hw.info=%08X, min periods=%d, max period=%d",
			rt->hw.info, rt->hw.periods_min, rt->hw.periods_max);

	chan = snd_dmaengine_pcm_request_channel(rpi_dma_filter_fn, &filter);

	/* pcm_dmaengine.c implementation store only single dma_chan pointer,
	   store dma channel locally. However, this doesn't solve the problem, the
	   struct dmaengine_pcm_runtime_data should refactored so that both playback and capture
	   channels could be saved */

	chip->dma_chan[substream->stream] = *chan;

	return snd_dmaengine_pcm_open(substream, chan);

}

static int pi_i2s_pcm_prepare(struct snd_pcm_substream *substream) {
	printk(KERN_INFO "pi_i2s_pcm_prepare: prepare callback\n");
	bcm2835_pcm_clear_fifos();
	return 0;
}

static int pi_i2s_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params) {

	struct dma_chan *chan = snd_dmaengine_pcm_get_chan(substream);

	struct dma_slave_config slave_config;
	int ret;
	struct snd_dmaengine_dai_dma_data *dma_data;
	struct snd_pi_i2s_chip *chip = snd_pcm_substream_chip(substream);

	printk(KERN_INFO "pi_i2s_pcm_hw_params: hw params callback\n");
	memset(&slave_config, 0, sizeof(slave_config));

	/* runtime->dma_bytes has to be set manually to allow mmap */
	substream->runtime->dma_bytes = params_buffer_bytes(hw_params);
	substream->runtime->no_period_wakeup = 0; // we need interrupt for DMA

	/* Initialize slave_config from  hw_params */
	ret = snd_hwparams_to_dma_slave_config(substream, hw_params,
			&slave_config);
	if (ret) {
		printk(	KERN_INFO "snd_dmaengine_pcm_prepare_slave_config failed with error %d",
				ret);
		return ret;
	}
	/* Configure slave_config from DAI data */
	dma_data = &chip->dma_data[substream->stream];

	if (dma_data == NULL) {
		printk(KERN_INFO "dma_data is null");
		return -EINVAL;
	}

	snd_dmaengine_pcm_set_config_from_dai_data(substream, dma_data,
			&slave_config);

	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret) {
		printk(KERN_INFO "dmaengine_slave_config() failed with %d\n",
				ret);
		return ret;
	}

	ret = snd_pcm_lib_preallocate_pages(substream, SNDRV_DMA_TYPE_DEV_IRAM,
			chan->device->dev,
			16*4096, 16*4096);

	if (ret < 0) {
		printk(KERN_INFO "pi_i2s_pcm_hw_params: snd_pcm_lib_preallocate_pages FAILED with error code %d\n",
				ret);
		return ret;
	} else
		printk(KERN_INFO "pi_i2s_pcm_hw_params: preallocated %d bytes for DMA",
				substream->dma_buffer.bytes);

	ret = snd_pcm_lib_malloc_pages(substream,
			params_buffer_bytes(hw_params));

	if (ret < 0) {

		printk(KERN_INFO "pi_i2s_pcm_hw_params: snd_pcm_lib_malloc_pages FAILED with error code %d",
				ret);
		return ret;
	}

	return 0;

}
static int pi_i2s_pcm_trigger(struct snd_pcm_substream *substream, int cmd) {
	int ret;
	char *cmd_list[] =
			{ "SNDRV_PCM_TRIGGER_STOP", "SNDRV_PCM_TRIGGER_START",
					"?", "?", "SNDRV_PCM_TRIGGER_SUSPEND",
					"SNDRV_PCM_TRIGGER_RESUME" };

	struct snd_pcm_runtime *rt = substream->runtime;

	printk(KERN_INFO "pi_i2s_pcm_trigger: trigger callback, command is %s\n",
			cmd_list[cmd]);

	ret = snd_dmaengine_pcm_trigger(substream, cmd);
	udelay(1000);

	if (ret < 0) {
		printk(KERN_INFO "pi_i2s_pcm_trigger snd_dmaengine_pcm_trigger failed\n");
		return ret;
	}

	printk(KERN_INFO "pi_i2s_pcm_trigger: period_size %lu, buffer_size %lu, rate %d, DMA buffer bytes %d\n",
			rt->period_size, rt->buffer_size, rt->rate,
			substream->dma_buffer.bytes);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			bcm2835_pcm_enable_capture();
		} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			bcm2835_pcm_enable_playback();
		}
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			bcm2835_pcm_disable_capture();
		} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			bcm2835_pcm_disable_playback();
		}
		return 0;
	}
	return -EINVAL;
}

static int pi_i2s_pcm_hw_free(struct snd_pcm_substream *substream) {
	printk(KERN_INFO "pi_i2s_pcm_hw_free: hw free callback\n");
	return snd_pcm_lib_free_pages(substream);

}

bool rpi_dma_filter_fn(struct dma_chan *chan, void *filter_param) {
	if (chan->chan_id == *(int *) filter_param)
		return true;
	return false;
}

static int pi_i2s_pcm_close(struct snd_pcm_substream *substream) {
	int ret;
	printk(KERN_INFO "pi_i2s_pcm_close: close callback");
	ret = snd_dmaengine_pcm_close_release_chan(substream);
	return ret;
}

static int pi_i2s_pcm_mmap(struct snd_pcm_substream *substream,
		struct vm_area_struct *vma) {
	struct snd_pcm_runtime *runtime = substream->runtime;
	printk(KERN_INFO "pi_i2s_pcm_mmap: mmap callback");

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
			runtime->dma_area, runtime->dma_addr,
			runtime->dma_bytes);
}

static struct snd_pcm_ops pi_i2s_pcm_ops = {
		.open =  pi_i2s_pcm_open,
		.close = pi_i2s_pcm_close,
		.ioctl = snd_pcm_lib_ioctl,
		.hw_params = pi_i2s_pcm_hw_params,
		.hw_free = pi_i2s_pcm_hw_free,
		.prepare = pi_i2s_pcm_prepare,
		.trigger = pi_i2s_pcm_trigger,
		.pointer = snd_dmaengine_pcm_pointer,
		.mmap = pi_i2s_pcm_mmap,

};

static int pi_pcm_new(struct snd_pi_i2s_chip *chip, int device, int substreams) {
	struct snd_pcm *pcm;
	struct snd_pcm_ops *ops;
	int err;

	/* Create PCM */
	err = snd_pcm_new(chip->card, "RPi PCM", device, substreams, substreams,
			&pcm);
	if (err < 0)
		return err;

	chip->pcm = pcm;
	ops = &pi_i2s_pcm_ops;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, ops);

	/* ALSA copies the pcm->private_data to substream->private_data when a new substream is opened.
	 * Save the chip data to pcm now for later use in callbacks (operations).
	 */
	pcm->private_data = chip;
	pcm->info_flags = 0;
	strcpy(pcm->name, "Raspberry Pi PCM I2S");

	/* Preallocate DMA data. Does not seem to work on BCM2708 platform this way */
	/*
	 int prealloc_buffer_size = 64*4096;

	 err = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
	 snd_dma_continuous_data(GFP_KERNEL),
	 prealloc_buffer_size, prealloc_buffer_size);

	 if (err < 0)
		return err;

	 */

	return 0;
}

static int snd_pi_i2s_probe(struct platform_device *devptr) {
	struct snd_card *card;
	struct snd_pi_i2s_chip *chip;
	int err;
	int dev_id = devptr->id;
	err = snd_card_new(devptr->dev.parent, index[dev_id], id[dev_id],
	THIS_MODULE, sizeof(struct snd_pi_i2s_chip), &card);
	if (err < 0)
		return err;

	chip = card->private_data;

	chip->card = card;
	chip->model = &model_tda1541;

	err = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (err)
		return err;

	err = pi_pcm_new(chip, 0, 1);

	if (err < 0) {
		snd_card_free(card);
		return err;
	}

	chip->pcm_hw = pi_i2s_pcm_hardware;

	/* Set the DMA address */
	chip->dma_data[SNDRV_PCM_STREAM_PLAYBACK].addr =
			(dma_addr_t) BCM2708_I2S_FIFO_PHYSICAL_ADDR;

	chip->dma_data[SNDRV_PCM_STREAM_CAPTURE].addr =
			(dma_addr_t) BCM2708_I2S_FIFO_PHYSICAL_ADDR;

	/* Set the DREQ */
	chip->dma_data[SNDRV_PCM_STREAM_PLAYBACK].slave_id =
	BCM2708_DMA_DREQ_PCM_TX;
	chip->dma_data[SNDRV_PCM_STREAM_CAPTURE].slave_id =
	BCM2708_DMA_DREQ_PCM_RX;

	/* Set the bus width */
	chip->dma_data[SNDRV_PCM_STREAM_PLAYBACK].addr_width =
			DMA_SLAVE_BUSWIDTH_4_BYTES;
	chip->dma_data[SNDRV_PCM_STREAM_CAPTURE].addr_width =
			DMA_SLAVE_BUSWIDTH_4_BYTES;

	/* Set burst */
	chip->dma_data[SNDRV_PCM_STREAM_PLAYBACK].maxburst = 2;
	chip->dma_data[SNDRV_PCM_STREAM_CAPTURE].maxburst = 2;

	strcpy(card->driver, "Raspberry Pi I2S PCM");
	strcpy(card->shortname, "Pi I2s");
	sprintf(card->longname, "Raspberry Pi I2S PCM Driver %i", dev_id + 1);

	snd_card_set_dev(card, &devptr->dev);

	err = snd_card_register(card);

	if (!err) {
		platform_set_drvdata(devptr, card);
		return 0;
	}

	return err;
}

static int snd_pi_i2s_remove(struct platform_device *devptr) {
	snd_card_free(platform_get_drvdata(devptr));
	platform_set_drvdata(devptr, NULL);
	return 0;
}

static struct platform_driver snd_pi_i2s_driver = { .probe = snd_pi_i2s_probe,
		.remove = snd_pi_i2s_remove, .driver = { .name =
				"snd_pi_i2s_pcm" }, };

static void snd_pi_i2s_unregister_all(void) {
	platform_device_unregister(snd_pi_i2s_device);
	platform_driver_unregister(&snd_pi_i2s_driver);
}

void setup_io(void);
void setup_i2s(void);
void setup_gpio(void);
void release_the_hounds(void);
void remap_io_addresses(void);

void remap_io_addresses() {
	i2s_registers = ioremap(I2S_BASE, 32);
	clock_registers = ioremap(CLOCK_BASE, 2);
	gpio = ioremap(GPIO_BASE, SZ_16K);
}

void setup_io(void) {
	remap_io_addresses();
	setup_gpio();
	setup_i2s();
}
void setup_gpio(void) {

	/* GPIO pins set for older models with the P5 header, does not support Model B Plus header GPIO pinning */

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

	int pin;

	/* SPI is on GPIO 7..11 */
	for (pin = 28; pin <= 31; pin++) {
		INP_GPIO(pin); /* set mode to GPIO input first */
		SET_GPIO_ALT(pin, 2); /* set mode to ALT 0 */
	}

#undef INP_GPIO
#undef SET_GPIO_ALT
	printk(KERN_INFO "setup_gpio: GPIO has been setup\n");

}

void setup_i2s(void) {

	int timeout = 5000;
	// --> disable STBY
	printk(KERN_INFO "setup_i2s: Disabling standby\n");
	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= 1 << 25;

	udelay(500);
	printk(KERN_INFO "setup_i2s: Disable I2S\n");
	*(i2s_registers + BCM2835_PCM_CS_A_REG) &= ~(1);

	printk(KERN_INFO "setup_i2s: Clear SYNC bit\n");
	*(i2s_registers + BCM2835_PCM_CS_A_REG) &= ~(1 << 24);

	printk(KERN_INFO "setup_i2s: Disabling I2S clock\n");
	*(clock_registers + 0x26) = 0x5A000000;
	*(clock_registers + 0x27) = 0x5A000000;

	udelay(10);

	printk(KERN_INFO "setup_i2s: Configure I2S clock\n");
	*(clock_registers + 0x26) = 0x5A000001;

	// Use 50 kHz sample rate which doesn't have fractional divider and so is clean from jitter
	*(clock_registers + 0x27) = 0x5A006000; //0x5A006400; // = 6.25 for 48khz 0x5A006CD7; //a calculated guess, hopefully about 6.8, but seems out somewhat (I want 64*44100 = 2822400 Hz)
	udelay(10);

	printk(KERN_INFO "setup_i2s: Enabling I2S clock\n");
	*(clock_registers + 0x26) = 0x5A000611;

	udelay(100);
	//printk(KERN_INFO "setup_i2s: Clear SYNC bit\n");
	printk(KERN_INFO "setup_i2s: Disable I2S\n");

	//*(i2s_registers+BCM2835_PCM_CS_A_REG) &= ~(1<<24);

	/*

	 *(i2s_registers+BCM2835_PCM_CS_A_REG) = 0;
	 *(i2s_registers+BCM2835_PCM_MODE_A_REG) = 0;
	 *(i2s_registers+BCM2835_PCM_TXC_A_REG) = 0;
	 *(i2s_registers+BCM2835_PCM_RXC_A_REG) = 0;
	 *(i2s_registers+BCM2835_PCM_GRAY_REG) = 0;
	 */
	udelay(100);

	// set register settings
	// --> enable Channel1 with 32bit width
	// For 16 bit I2S, Channel width should be 16, possible positions should be 1 and 17?
	//(e.g. look for 1<<29 and 17<<4
	printk(KERN_INFO "setup_i2s: Setting TX channel settings\n");
	*(i2s_registers + BCM2835_PCM_TXC_A_REG) = 0 << 31 | 1 << 30 | 1 << 20
			| 8 << 16 | 0 << 15 | 1 << 14 | 33 << 4 | 8;

	printk(KERN_INFO "setup_i2s: Setting RX channel settings\n");
	*(i2s_registers + BCM2835_PCM_RXC_A_REG) = 0 << 31 | 1 << 30 | 1 << 20
			| 8 << 16 | 0 << 15 | 1 << 14 | 33 << 4 | 8;

	//Set frame length and frame sync length (32 and 16), and set FTXP=1 and FRXP=1 so I can inject 2 channels into a single 32 bit word
	*(i2s_registers + BCM2835_PCM_MODE_A_REG) = 3 << 24 | 63<<10 | 32;

	//*(i2s_registers+BCM2835_PCM_MODE_A_REG) = get_mode();

	udelay(50);

	printk(KERN_INFO "setup_i2s: Clear FIFO's and set FIFO thresholds\n");

	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= (1 << 3 | 1 << 4); // clear TX & RX FIFO's
	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= (1 << 5); //set TXTHR, 2 seems to avoid errors (until interrupts are stalled)
	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= (1 << 7); //set RXTHR

	//for(i = 0; i < 32; i++)
	//   (*(i2s_registers+BCM2835_PCM_FIFO_A_REG)) = 0;
	// --> ENABLE SYNC bit
	printk(KERN_INFO "setup_i2s: setting SYNC bit high\n");
	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= 1 << 24;

	if (*(i2s_registers + BCM2835_PCM_CS_A_REG) & (1 << 24)) {
		printk(KERN_INFO "setup_i2s: SYNC bit high, strange\n");
	} else {
		printk(KERN_INFO "setup_i2s: SYNC bit low, as expected\n");
	}

	udelay(10);
	while (--timeout) {
		if (*(i2s_registers + BCM2835_PCM_CS_A_REG) & (1 << 24)) {
			break;
		}
		udelay(100);
	}

	if (!timeout)
		printk(KERN_INFO "setup_i2s: Timeout in SYNC handling\n");

	printk(KERN_INFO "setup_i2s: Clear SYNC bit\n");
	*(i2s_registers + BCM2835_PCM_CS_A_REG) &= (1 << 24);

	// Enable interrupts.
	// In DMA mode no IRQ is used for now - maybe needed later for FIFO errors
	*(i2s_registers + BCM2835_PCM_INTSTC_A_REG) = 0x000F; // clear status bits
	// *(i2s_registers+BCM2835_PCM_INTEN_A_REG) = 0x01; //0x5 should also enable errors

	// Enable DMA / DREQ
	printk(KERN_INFO "setup_i2s: Enable DMAs\n");
	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= BCM2835_PCM_DMAEN;

	// Note: BCM2835_PCM_DREQ_A_REG register uses default (reset) values for so far
	*(i2s_registers + BCM2835_PCM_DREQ_A_REG) |=
			( BCM2835_PCM_TX_PANIC(0x10)
					| BCM2835_PCM_RX_PANIC(0x30)
					| BCM2835_PCM_TX(0x30)
					| BCM2835_PCM_RX(0x20));

	// Global enable I2S
	printk(KERN_INFO "setup_i2s: Enable PCM\n");
	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= 0x01;

	// Enable transmission and capture
	// Maybe better do in TRIGGER callback actually AFTER DMA has been setup
	printk(KERN_INFO "setup_i2s: Enable transmission and capture\n");
	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= (1 << 2 | 1 << 1);

	// Done
	printk(KERN_INFO "setup_i2s: I2S setup complete\n");

	return;
}

int get_mode() {
	int mode;
	int data_length = 16;
	int bclk_ratio = 50;
	int fmt = (SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF);

	/* Setup the I2S mode */
	mode = 0;

	if (data_length <= 16) {
		/*
		 * Use frame packed mode (2 channels per 32 bit word)
		 * We cannot set another frame length in the second stream
		 * (and therefore word length) anyway,
		 * so the format will be the same.
		 */
		mode |= BCM2835_PCM_FTXP | BCM2835_PCM_FRXP;
	}

	mode |= BCM2835_PCM_FLEN(bclk_ratio - 1);
	mode |= BCM2835_PCM_FSLEN(bclk_ratio / 2);

	/* Master or slave? */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* CPU is master */
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		/*
		 * CODEC is bit clock master
		 * CPU is frame master
		 */
		mode |= BCM2835_PCM_CLKM;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		/*
		 * CODEC is frame master
		 * CPU is bit clock master
		 */
		mode |= BCM2835_PCM_FSM;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		/* CODEC is master */
		mode |= BCM2835_PCM_CLKM;
		mode |= BCM2835_PCM_FSM;
		break;
	default:
		printk(KERN_INFO "error in mode setting\n");
		break;
	}

	/*
	 * Invert clocks?
	 *
	 * The BCM approach seems to be inverted to the classical I2S approach.
	 */
	switch (fmt & (0x0f00)) {
	case SND_SOC_DAIFMT_NB_NF:
		/* None. Therefore, both for BCM */
		mode |= BCM2835_PCM_CLKI;
		mode |= BCM2835_PCM_FSI;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		/* Both. Therefore, none for BCM */
		break;
	case SND_SOC_DAIFMT_NB_IF:
		/*
		 * Invert only frame sync. Therefore,
		 * invert only bit clock for BCM
		 */
		mode |= BCM2835_PCM_CLKI;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		/*
		 * Invert only bit clock. Therefore,
		 * invert only frame sync for BCM
		 */
		mode |= BCM2835_PCM_FSI;
		break;
	default:
		printk(KERN_INFO "error in 2nd mode setting\n");
		break;
	}
	printk(KERN_INFO "get_mode: mode is %08X\n", mode);

	return mode;
}

void bcm2835_pcm_clear_fifos() {
	*(i2s_registers + BCM2835_PCM_CS_A_REG) &= ~(0x01); // Stop I2S
	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= (1 << 3 | 1 << 4); // Clear TX & RX FIFO's
	udelay(200);
	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= 0x01; // Enable I2S
}
void bcm2835_pcm_enable_capture() {
	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= (1 << 1);
}

void bcm2835_pcm_enable_playback() {
	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= (1 << 2);
}
void bcm2835_pcm_disable_capture() {
	*(i2s_registers + BCM2835_PCM_CS_A_REG) &= ~(1 << 1);
}

void bcm2835_pcm_disable_playback() {
	*(i2s_registers + BCM2835_PCM_CS_A_REG) &= ~(1 << 2);
}

void release_the_hounds(void) {
	*(i2s_registers + BCM2835_PCM_INTSTC_A_REG) = 0x000F; // clear status bits
	*(i2s_registers + BCM2835_PCM_INTEN_A_REG) = 0x00;
	*(i2s_registers + BCM2835_PCM_CS_A_REG) &= ~(1 << 24);
	udelay(100);
	*(i2s_registers + BCM2835_PCM_CS_A_REG) |= 0x00;

	iounmap(i2s_registers);
	iounmap(gpio);
	iounmap(clock_registers);
	return;
}


static int __init alsa_card_pi_i2s_init(void) {
	int err;

	err = platform_driver_register(&snd_pi_i2s_driver);
	if (err < 0)
		return err;

	err = 0; // alloc_buffers();

	if (err < 0) {
		platform_driver_unregister(&snd_pi_i2s_driver);
		return err;
	}

	snd_pi_i2s_device = platform_device_register_simple("snd_pi_i2s_pcm", 1,
			NULL, 0);
	setup_io();
	printk(KERN_INFO "alsa_card_pi_i2s_init: driver initialized\n");
	return 0;
}

static void __exit alsa_card_pi_i2s_exit(void) {
	snd_pi_i2s_unregister_all();
	release_the_hounds();

	return;
}

module_init(alsa_card_pi_i2s_init)
module_exit(alsa_card_pi_i2s_exit)
