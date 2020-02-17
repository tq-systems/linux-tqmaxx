/*
 * Copyright 2014 Markus Pargmann <mpa@pengutronix.de>
 *
 *  Based on imx-sgtl5000.c
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <sound/soc.h>

#define DAI_NAME_SIZE	32

struct imx_tlv320aic_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	unsigned int clk_frequency;
};

static int imx_tlv320aic_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct imx_tlv320aic_data *data = container_of(rtd->card,
					struct imx_tlv320aic_data, card);
	struct device *dev = rtd->card->dev;
	int ret;

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, 0, data->clk_frequency, 0);
	if (ret) {
		dev_err(dev, "could not set codec driver clock params\n");
		return ret;
	}

	return 0;
}

static int imx_tlv320aic_probe(struct platform_device *pdev)
{
	struct device_node *ssi_np, *codec_np;
	struct platform_device *ssi_pdev;
	struct i2c_client *codec_dev;
	struct clk *codec_clk;
	struct imx_tlv320aic_data *data;
	int ret = 0;

	ssi_np = of_parse_phandle(pdev->dev.of_node, "ssi-controller", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!ssi_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	ssi_pdev = of_find_device_by_node(ssi_np);
	if (!ssi_pdev) {
		dev_dbg(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}
	put_device(&ssi_pdev->dev);
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev) {
		dev_dbg(&pdev->dev, "failed to find codec platform device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	codec_clk = clk_get(&codec_dev->dev, NULL);
	if (IS_ERR(codec_clk)) {
		ret = PTR_ERR(codec_clk);
		goto fail;
	}

	data->clk_frequency = clk_get_rate(codec_clk);
	clk_put(codec_clk);

	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codecs->dai_name = "tlv320aic32x4-hifi";
	data->dai.codecs->of_node = codec_np;
	data->dai.cpus->of_node = ssi_np;
	data->dai.platforms->of_node = ssi_np;
	data->dai.init = &imx_tlv320aic_dai_init;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBM_CFM;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;

	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;
	data->card.dai_link = &data->dai;

	ret = snd_soc_register_card(&data->card);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
				ret);
		goto fail;
	}

	platform_set_drvdata(pdev, data);

fail:
	of_node_put(ssi_np);
	of_node_put(codec_np);

	return ret;
}

static int imx_tlv320aic_remove(struct platform_device *pdev)
{
	struct imx_tlv320aic_data *data = platform_get_drvdata(pdev);

	snd_soc_unregister_card(&data->card);

	return 0;
}

static const struct of_device_id imx_tlv320aic_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-tlv320aic32x4", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_tlv320aic_dt_ids);

static struct platform_driver imx_tlv320aic_driver = {
	.driver = {
		.name = "imx-tlv320aic32x4",
		.of_match_table = imx_tlv320aic_dt_ids,
	},
	.probe = imx_tlv320aic_probe,
	.remove = imx_tlv320aic_remove,
};
module_platform_driver(imx_tlv320aic_driver);

MODULE_AUTHOR("Markus Pargmann <mpa@pengutronix.de>");
MODULE_DESCRIPTION("Freescale i.MX with TI TLV320AIC32x4 ASoC machine driver");
MODULE_LICENSE("GPL v2");
