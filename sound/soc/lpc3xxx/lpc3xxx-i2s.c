/*
 * sound/soc/lpc3xxx/lpc3xxx-i2s.c
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2008 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <mach/platform.h>
#include <mach/hardware.h>
#include "lpc3xxx-pcm.h"
#include "lpc3xxx-i2s.h"

#include <mach/clkdev.h>

#define I2S_NAME "lpc3xxx-i2s"

#define CLK_NAME (8)

#define LPC3XXX_I2S_RATES \
    (SNDRV_PCM_RATE_8000  | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
     SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
     SNDRV_PCM_RATE_48000)

#define LPC3XXX_I2S_FORMATS (SNDRV_PCM_FMTBIT_S8 | SND_SOC_DAIFMT_I2S | \
    SNDRV_PCM_FMTBIT_S16 | SNDRV_PCM_FMTBIT_S32)

#define CLKPWR_IOBASE io_p2v(CLK_PM_BASE)

static struct lpc3xxx_i2s_info *i2s_info = NULL;

#if 0
static struct lpc3xxx_i2s_info i2s_info[NUM_I2S_DEVICES] = {
	{
	 .name = "i2s0",
	 .lock = __SPIN_LOCK_UNLOCKED(i2s_info[0].lock),
	 .initialized = 0,
	 .clkname = "i2s0_ck",
	 .baseio = LPC32XX_I2S0_BASE,
	 },
	{
	 .name = "i2s1",
	 .lock = __SPIN_LOCK_UNLOCKED(i2s_info[1].lock),
	 .initialized = 0,
	 .clkname = "i2s1_ck",
	 .baseio = LPC32XX_I2S1_BASE,
	 },
};
#endif

static u32 absd32(u32 v1, u32 v2) {
	if (v1 > v2) {
		return v1 - v2;
	}

	return v2 - v1;
}

static void __lpc3xxx_find_clkdiv(u32 *clkx, u32 *clky, int freq,
				  int xbytes, u32 clkrate) {
	u32 i2srate;
	u32 idxx, idyy;
	u32 savedbitclkrate, diff, trate, baseclk;

	/* Adjust rate for sample size (bits) and 2 channels and offset for
	   divider in clock output */
	i2srate = (freq / 100) * 2 * (8 * xbytes);
	i2srate = i2srate << 1;
	clkrate = clkrate / 100;
	baseclk = clkrate;
	*clkx = 1;
	*clky = 1;

	/* Find the best divider */
	*clkx = *clky = 0;
	savedbitclkrate = 0;
	diff = ~0;
	for (idxx = 1; idxx < 0xFF; idxx++) {
		for (idyy = 1; idyy < 0xFF; idyy++) {
			trate = (baseclk * idxx) / idyy;
			if (absd32(trate, i2srate) < diff) {
				diff = absd32(trate, i2srate);
				savedbitclkrate = trate;
				*clkx = idxx;
				*clky = idyy;
			}
		}
	}
}

static int lpc3xxx_i2s_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	u32 clkx, clky;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct lpc3xxx_i2s_info *i2s_info_p = &i2s_info[rtd->cpu_dai->id];
	u32 dmamask;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dmamask = I2S_DMA_XMIT;
	}
	else {
		dmamask = I2S_DMA_RECV;
	}

	if (dmamask & i2s_info_p->dma_flags) {
		/* This channel already enabled! */
		printk("%s: I2S DMA channel is busy!\n",
			I2S_NAME);
		return -EBUSY;
	}

	/* Enable I2S clock */
	clk_enable(i2s_info_p->clk);

	/* I2S is setup here with a dummy configuration to allow the
	bit clock to toggle. The CODEC attached to the I2S may not
	work without this clock. The actual values here do not yet
	matter and will be reconfigured when needed by the I2S hw
	setup function. To match the default selected PLL setting,
	this clock should be between 6.25 and 12.5KHz */
	__raw_writel(I2S_DMA0_TX_DEPTH(4), I2S_DMA1(i2s_info_p->iomem));
	__lpc3xxx_find_clkdiv(&clkx, &clky, 9000, 2, i2s_info_p->clkrate);
	__raw_writel(((clkx << 8) | clky), I2S_TX_RATE(i2s_info_p->iomem));
	__raw_writel(0x83C1, I2S_DAO(i2s_info_p->iomem));

	i2s_info_p->dma_flags |= dmamask;

	return 0;
}

static void lpc3xxx_i2s_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct lpc3xxx_i2s_info *i2s_info_p = &i2s_info[rtd->cpu_dai->id];
	u32 dmamask, tmp;

	/* Disable I2S based on stream */
	spin_lock_irq(&i2s_info_p->lock);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dmamask = I2S_DMA_XMIT;
		tmp = __raw_readl(I2S_DAO(i2s_info_p->iomem));
		tmp |= I2S_STOP;
		__raw_writel(tmp, I2S_DAO(i2s_info_p->iomem));
		__raw_writel(0, I2S_TX_RATE(i2s_info_p->iomem));
	}
	else {
		dmamask = I2S_DMA_RECV;
		tmp = __raw_readl(I2S_DAI(i2s_info_p->iomem));
		tmp |= I2S_STOP;
		__raw_writel(tmp, I2S_DAI(i2s_info_p->iomem));
		__raw_writel(0, I2S_RX_RATE(i2s_info_p->iomem));
	}

#if defined(CONFIG_SND_LPC32XX_SLAVE_TX_CLK_TO_RX)
	/* Slave the RX WSI signal to the TX WSI signal */
	tmp = __raw_readl(LPC32XX_CLKPWR_I2S_CLK_CTRL);
	tmp &= ~(1 << 6);
	__raw_writel(tmp, LPC32XX_CLKPWR_I2S_CLK_CTRL);
#endif

	spin_unlock_irq(&i2s_info_p->lock);

	/* If both streams are shut down, then disable I2S to save power */
	i2s_info_p->dma_flags &= ~dmamask;
	if(i2s_info_p->dma_flags == 0) {
		clk_disable(i2s_info_p->clk);
	}
}

static int lpc3xxx_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				      int clk_id, unsigned int freq, int dir)
{
	struct lpc3xxx_i2s_info *i2s_info_p = &i2s_info[cpu_dai->id];

	/* Will use in HW params later */
	i2s_info_p->freq = freq;

	return 0;
}

static int lpc3xxx_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
				   unsigned int fmt)
{
	struct lpc3xxx_i2s_info *i2s_info_p = &i2s_info[cpu_dai->id];

	/* Will use in HW params later */
	i2s_info_p->daifmt = fmt;

	return 0;
}

static int lpc3xxx_i2s_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				      int div_id, int div)
{
	struct lpc3xxx_i2s_info *i2s_info_p = &i2s_info[cpu_dai->id];

	/* This function doesn't help, but save the value anyways.
	   HW params will determine the correct clock divider based
	   on the frequency */
	i2s_info_p->clkdiv = div;

	return 0;
}

static int lpc3xxx_i2s_hw_params(struct snd_pcm_substream *substream,
			         struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int id = rtd->cpu_dai->id;
	struct lpc3xxx_i2s_info *i2s_info_p;
	int xfersize;
	u32 tmp, tmp2, clkx, clky;

	(void) tmp2;
	i2s_info_p = &i2s_info[id];

	/* Build the I2S setup word */
	tmp = 0;

	if ((i2s_info_p->daifmt & SND_SOC_DAIFMT_MASTER_MASK) ==
		SND_SOC_DAIFMT_CBM_CFM) {
		tmp |= I2S_WS_SEL;
	}

	switch (i2s_info_p->daifmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SNDRV_PCM_FMTBIT_S8:
			tmp |= I2S_WW8 | I2S_WS_HP(I2S_WW8_HP);
			xfersize = 1;
			break;

		case SNDRV_PCM_FMTBIT_S16:
		case SND_SOC_DAIFMT_I2S:
			tmp |= I2S_WW16 | I2S_WS_HP(I2S_WW16_HP);
			xfersize = 2;
			break;

		case SNDRV_PCM_FMTBIT_S32:
			tmp |= I2S_WW32 | I2S_WS_HP(I2S_WW32_HP);
			xfersize = 4;
			break;

		default:
			pr_warning("%s: Unsupported audio data format\n",
				I2S_NAME);
			return -EINVAL;
	}

	/* Mono or stereo? */
	if (params_channels(params) == 1) {
		tmp |= I2S_MONO;
	}

	/* Find the best clock dividers to generate the requested
	   frequency */
	__lpc3xxx_find_clkdiv(&clkx, &clky, i2s_info_p->freq, xfersize, i2s_info_p->clkrate);

	printk("I2S CONFIGURATION SUMMARY:\n");
	printk("-----------------------------------------\n");
	printk("Desired clock rate    : %d\n", i2s_info_p->freq);
	printk("Base clock rate       : %d\n", i2s_info_p->clkrate);
	printk("Transfer size (bytes) : %d\n", xfersize);
	printk("Clock divider (x)     : %d\n", clkx);
	printk("Clock divider (y)     : %d\n", clky);
	printk("Channels              : %d\n", params_channels(params));
	printk("Data format           : %d\n", i2s_info_p->daifmt);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		
		printk("STREAM PLAYBACK SECTION ENTERED!\n");
		/* Enable DAO support, correct clock rate, and DMA */
		__raw_writel(I2S_DMA1_TX_EN | I2S_DMA0_TX_DEPTH(4),
			I2S_DMA1(i2s_info_p->iomem));
		__raw_writel(((clkx << 8) | clky),
			I2S_TX_RATE(i2s_info_p->iomem));
		__raw_writel(tmp, I2S_DAO(i2s_info_p->iomem));

		//printk("Heres the stupid mem address: 0x%x\n",i2s_info_p->iomem);
		printk("I2S1 IRQ register : 0x%X\n", __raw_readl(I2S_IRQ(i2s_info_p->iomem)));
		
		printk("TX DMA1               : 0x%x\n",
			__raw_readl(I2S_DMA1(i2s_info_p->iomem)));
		printk("TX dividers           : 0x%x\n",
			__raw_readl(I2S_TX_RATE(i2s_info_p->iomem)));
		printk("TX DAO                : 0x%x\n",
			__raw_readl(I2S_DAO(i2s_info_p->iomem)));
	}
	else {
		printk("STREAM PLAYBACK SECTION BYPASSED!\n");
		/* Enable DAI support, correct clock rate, and DMA */
		__raw_writel(I2S_DMA0_RX_EN | I2S_DMA1_RX_DEPTH(4),
			I2S_DMA0(i2s_info_p->iomem));
		__raw_writel(((clkx << 8) | clky), I2S_RX_RATE(i2s_info_p->iomem));

#if defined(CONFIG_SND_LPC32XX_SLAVE_TX_CLK_TO_RX)
		/* Slave the RX WS signal to the TX WS signal */
		tmp2 = __raw_readl(LPC32XX_CLKPWR_I2S_CLK_CTRL);
		tmp2 |= (1 << 6);
		__raw_writel(tmp2, LPC32XX_CLKPWR_I2S_CLK_CTRL);

		/* The DAO interface needs to be enabled to route the clock from
		   RX to TX */
		__raw_writel((tmp & ~I2S_WS_SEL), I2S_DAO(i2s_info_p->iomem));
#endif

		__raw_writel(tmp, I2S_DAI(i2s_info_p->iomem));

		printk("RX DMA0               : 0x%x\n",
			__raw_readl(I2S_DMA0(i2s_info_p->iomem)));
		printk("RX dividers           : 0x%x\n",
			__raw_readl(I2S_RX_RATE(i2s_info_p->iomem)));
		printk("RX DAI                : 0x%x\n",
			__raw_readl(I2S_DAI(i2s_info_p->iomem)));
	}

	return 0;
}

static int lpc3xxx_i2s_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	/* Nothing to do here */
	return 0;
}

static int lpc3xxx_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int id = rtd->cpu_dai->id;
	struct lpc3xxx_i2s_info *i2s_info_p;
	u32 tmp;
	int ret = 0;

	i2s_info_p = &i2s_info[id];

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			tmp = __raw_readl(I2S_DAO(i2s_info_p->iomem));
			tmp |= I2S_STOP;
			__raw_writel(tmp, I2S_DAO(i2s_info_p->iomem));
		}
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			tmp = __raw_readl(I2S_DAO(i2s_info_p->iomem));
			tmp &= ~I2S_STOP;
			__raw_writel(tmp, I2S_DAO(i2s_info_p->iomem));
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_PM
static int lpc3xxx_i2s_suspend(struct snd_soc_dai *cpu_dai)
{
	struct lpc3xxx_i2s_info *i2s_info_p = &i2s_info[cpu_dai->id];

	if (!cpu_dai->active)
		return 0;

	/* Save DA0, DAI, and IRQ register states */
	i2s_info_p->dao_save = __raw_readl(I2S_DAO(i2s_info_p->iomem));
	i2s_info_p->dai_save = __raw_readl(I2S_DAI(i2s_info_p->iomem));
	i2s_info_p->irq_save = __raw_readl(I2S_IRQ(i2s_info_p->iomem));

	/* Disable system clock */
	clk_disable(i2s_info_p->clk);

	return 0;
}

static int lpc3xxx_i2s_resume(struct snd_soc_dai *cpu_dai)
{
	struct lpc3xxx_i2s_info *i2s_info_p = &i2s_info[cpu_dai->id];

	if (!cpu_dai->active)
		return 0;

	/* Restore system clock */
	clk_enable(i2s_info_p->clk);

	/* Restore saved DA0, DAI, and IRQ register states */
	__raw_writel(i2s_info_p->dao_save, I2S_DAO(i2s_info_p->iomem));
	__raw_writel(i2s_info_p->dai_save, I2S_DAI(i2s_info_p->iomem));
	__raw_writel(i2s_info_p->irq_save, I2S_IRQ(i2s_info_p->iomem));

	return 0;
}

#else
#  define lpc3xxx_i2s_suspend	NULL
#  define lpc3xxx_i2s_resume	NULL
#endif

static struct snd_soc_dai_ops lpc3xxx_i2s_dai_ops = {
	.startup = lpc3xxx_i2s_startup,
	.shutdown = lpc3xxx_i2s_shutdown,
	.prepare = lpc3xxx_i2s_prepare,
	.trigger = lpc3xxx_i2s_trigger,
	.hw_params = lpc3xxx_i2s_hw_params,
	.set_sysclk = lpc3xxx_i2s_set_dai_sysclk,
	.set_fmt = lpc3xxx_i2s_set_dai_fmt,
	.set_clkdiv = lpc3xxx_i2s_set_dai_clkdiv,
};

struct snd_soc_dai_driver lpc3xxx_i2s_dai[NUM_I2S_DEVICES] = {
{
	.name = "lpc3xxx-i2s0",
	.id = 0,
	.suspend = lpc3xxx_i2s_suspend,
	.resume = lpc3xxx_i2s_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = LPC3XXX_I2S_RATES,
		.formats = LPC3XXX_I2S_FORMATS,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = LPC3XXX_I2S_RATES,
		.formats = LPC3XXX_I2S_FORMATS,
	},
	.ops = &lpc3xxx_i2s_dai_ops,
},
{
	.name = "lpc3xxx-i2s1",
	.id = 1,
	.suspend = lpc3xxx_i2s_suspend,
	.resume = lpc3xxx_i2s_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = LPC3XXX_I2S_RATES,
		.formats = LPC3XXX_I2S_FORMATS,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = LPC3XXX_I2S_RATES,
		.formats = LPC3XXX_I2S_FORMATS,
	},
	.ops = &lpc3xxx_i2s_dai_ops,
},
};

/*
 * Platform probe function
 */
static __devinit int lpc3xxx_i2s_plat_probe(struct platform_device *pdev)
{
	char clk_name[2 * CLK_NAME];
	int ret, i, j;
	struct resource *res;
	resource_size_t size;
	u32 clkx, clky;
	struct lpc3xxx_i2s_info *i2s;

	printk("I2S PLATFORM PROBE ENTERED\n");

	/* Allocate mem for i2s_info structure */
	i2s_info = devm_kzalloc(&pdev->dev,
			(sizeof(struct lpc3xxx_i2s_info) * pdev->num_resources), GFP_KERNEL);
	if (unlikely(!i2s_info)) {
		dev_err(&pdev->dev, "Can't allocate i2s_info \n");
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, i2s_info);

	for(i = 0; i < pdev->num_resources; i++) {
		i2s = &i2s_info[i];

		/* Enable I2S clock */
		printk("ATTEMPTING TO ENABLE I2S Clock...\n");
		snprintf(&clk_name[i * CLK_NAME], CLK_NAME, "i2s%d_ck", i);
		i2s->clkname = &clk_name[i * CLK_NAME];
		i2s->clk = clk_get(NULL, i2s->clkname);
		if (IS_ERR(i2s->clk)) {
			i2s->clk = NULL;
			pr_warning("%s: Failed enabling the I2S clock: %s\n",
				I2S_NAME, i2s->clkname);
			return -ENODEV;
		}

		clk_enable(i2s->clk);
		i2s->clkrate = clk_get_rate(i2s->clk);
		if (i2s->clkrate == 0) {
			pr_warning("%s: Invalid returned clock rate\n",
				I2S_NAME);
			clk_disable(i2s->clk);
			clk_put(i2s->clk);
			i2s->clk = NULL;
			return -ENODEV;
		}

		/* Get Virtual CPU address for I2S */
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			dev_err(&pdev->dev, "No memory resource\n");
			clk_disable(i2s->clk);
			clk_put(i2s->clk);
			i2s->clk = NULL;
			ret = -ENODEV;
			goto err_res;
		}

		size = resource_size(res);
		if (!devm_request_mem_region(&pdev->dev, res->start, size,
				     pdev->name)) {
			dev_err(&pdev->dev, "I2S registers are not free\n");
			clk_disable(i2s->clk);
			clk_put(i2s->clk);
			i2s->clk = NULL;
			ret = -EBUSY;
			goto err_res;
		}
		i2s->iomem = devm_ioremap(&pdev->dev, res->start, size);
		if (!i2s->iomem) {
			dev_err(&pdev->dev, "Can't map memory\n");
			clk_disable(i2s->clk);
			clk_put(i2s->clk);
			i2s->clk = NULL;
			ret = -ENOMEM;
			goto err_res;
		}

		spin_lock_init(&i2s->lock);

		/* I2S is setup here with a dummy configuration to allow the
		   bit clock to toggle. The CODEC attached to the I2S may not
		   work without this clock. The actual values here do not yet
		   matter and will be reconfigured when needed by the I2S hw
		   setup function. To match the default selected PLL setting,
                   this clock should be between 6.25 and 12.5KHz */
		__raw_writel(I2S_DMA0_TX_DEPTH(4), I2S_DMA1(i2s->iomem));
		__lpc3xxx_find_clkdiv(&clkx, &clky, 9000, 2, i2s->clkrate);
		__raw_writel(((clkx << 8) | clky), I2S_TX_RATE(i2s->iomem));
		__raw_writel(0x83C1, I2S_DAO(i2s->iomem));

		i2s->initialized = 1;
		printk("I2S MARKED AS INITIALIZED!\n");
	}

	/* Regsiter SND SOC driver */
	ret = snd_soc_register_dais(&pdev->dev, &lpc3xxx_i2s_dai[0], 2);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAI: %d\n", ret);
		ret = -ENOMEM;
		goto err_res;
	}

	printk("I2S DAI REGISTERED!\n");
	return 0;

err_res:
	for(j =0 ; j < i; j++) {
		clk_disable(i2s_info[j].clk);
		clk_put(i2s_info[j].clk);
		i2s_info[j].clk = NULL;
	}

	dev_set_drvdata(&pdev->dev, NULL);

	return ret;
}

static int __devexit lpc3xxx_i2s_plat_remove(struct platform_device *pdev)
{
	int j;
	struct lpc3xxx_i2s_info *i2s = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_dais(&pdev->dev, 2);

	for(j =0 ; j < pdev->num_resources; j++) {
		clk_disable(i2s[j].clk);
		clk_put(i2s[j].clk);
		i2s[j].clk = NULL;
	}

	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static struct platform_driver lpc3xxx_i2s_driver = {
	.driver = {
		.name = I2S_NAME,
		.owner = THIS_MODULE,
	},
	.probe = lpc3xxx_i2s_plat_probe,
	.remove = __devexit_p(lpc3xxx_i2s_plat_remove),
};

static int __init lpc3xxx_i2s_dai_init(void)
{
	return platform_driver_register(&lpc3xxx_i2s_driver);
}
module_init(lpc3xxx_i2s_dai_init);

static void __exit lpc3xxx_i2s_dai_exit(void)
{
	return platform_driver_unregister(&lpc3xxx_i2s_driver);
}
module_exit(lpc3xxx_i2s_dai_exit)

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("ASoC LPC3XXX I2S interface");
MODULE_LICENSE("GPL");
