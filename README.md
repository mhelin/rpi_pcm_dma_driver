# rpi_pcm_dma_driver

This driver is a fork of Phil Pooles RPi FIFO driver at https://github.com/philpoole/snd_pi_i2s
Also some functionality is picked from the RPi ASoC driver (Linux/sound/soc/bcm/bcm2835-i2s.c) by Florian Meier.

The DMA implementation in this version uses the ALSA Core DMA engine library (sound/core/pcm_dmaengine.c). One problem 
with it is that the DMA channel allocations need to made late in open callback unlike in ASoC driver which reserves the channels and allocates DMA buffer in platform driver initialization pahe. On the other hand the DMA channels 2 and 3 used here are dedicated to PCM/I2S interface so the two used DMA channels and DREQs should always be available.

This is a work ongoing, some refactoring is needed like support for different sample rates (current implementation ignores sample rate in HW params and uses 50 kHz sample rate) and word lengths (currently only packed 16-bit word size is supported in stereo). It would also be better to move the BCM related functionality to own module.


