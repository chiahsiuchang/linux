// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
//
// AMD SPI controller driver
//
// Copyright (c) 2020-2021, Advanced Micro Devices, Inc.
//
// Authors: Sanjay R Mehta <sanju.mehta@amd.com>
//          Nehal Bakulchandra Shah <nehal-bakulchandra.shah@amd.com>
//          Lucas Tanure <tanureal@opensource.cirrus.com>

#include <linux/acpi.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>

#define AMD_SPI_CTRL0_REG		0x00
#define AMD_SPI_ENABLE_REG		0x20
#define AMD_SPI_NRM_SPD_SHIFT		28
#define AMD_SPI_NRM_SPD_MASK		GENMASK(31, AMD_SPI_NRM_SPD_SHIFT)
#define AMD_SPI_FST_SPD_SHIFT		24
#define AMD_SPI_FST_SPD_MASK		GENMASK(27, AMD_SPI_FST_SPD_SHIFT)
#define AMD_SPI_ALT_SPD_SHIFT		20
#define AMD_SPI_ALT_SPD_MASK		GENMASK(23, AMD_SPI_ALT_SPD_SHIFT)
#define AMD_SPI_TMP_SPD_SHIFT		16
#define AMD_SPI_TMP_SPD_MASK		GENMASK(19, AMD_SPI_TMP_SPD_SHIFT)

#define AMD_SPI_DUMMY_CYCL_REG		0x32
#define AMD_SPI_OPCODE_REG		0x45
#define AMD_SPI_CMD_TRIGGER_REG		0x47
#define AMD_SPI_EXEC_CMD		BIT(16)
#define AMD_SPI_FIFO_CLEAR		BIT(20)
#define AMD_SPI_BUSY			BIT(31)
#define AMD_SPI_TRIGGER_CMD		BIT(7)
#define AMD_SPI_OPCODE_MASK		0xFF
#define AMD_SPI_ALT_CS_REG		0x1D
#define AMD_SPI_ALT_CS_MASK		0x3
#define AMD_SPI_FIFO_BASE		0x80
#define AMD_SPI_TX_COUNT_REG		0x48
#define AMD_SPI_RX_COUNT_REG		0x4B
#define AMD_SPI_STATUS_REG		0x4C
#define AMD_SPI_SPEED_REG		0x6C
#define AMD_SPI_SPD7_SHIFT		8
#define AMD_SPI_SPD7_MASK		GENMASK(13, AMD_SPI_SPD7_SHIFT)
#define AMD_SPI_SPD6_SHIFT		0
#define AMD_SPI_SPD6_MASK		GENMASK(5, AMD_SPI_SPD6_SHIFT)

#define AMD_SPI_FIFO_SIZE		72
#define AMD_SPI_MEM_SIZE		200
/* M_CMD OP codes for SPI */
#define AMD_SPI_XFER_TX			1
#define AMD_SPI_XFER_RX			2

enum amd_spi_speed {
	SPI_66_66MHz,
	SPI_33_33MHz,
	SPI_22_22MHz,
	SPI_16_66MHz,
	SPI_100MHz,
	SPI_800KHz,
	SPI_SPD6,
	SPI_SPD7,
	SPI_50MHz = 0x4,
	SPI_4MHz = 0x32,
	SPI_3_17MHz = 0x3F
};

struct amd_spi_devtype_data {
	u32 spi_status;
	u8	version;
};

static const struct amd_spi_devtype_data spi_v1 = {
	.spi_status	= AMD_SPI_CTRL0_REG,
	.version	= 0,
};

static const struct amd_spi_devtype_data spi_v2 = {
	.spi_status	= AMD_SPI_STATUS_REG,
	.version	= 1,
};

struct amd_spi {
	void __iomem *io_remap_addr;
	unsigned long io_base_addr;
	u32 rom_addr;
	const struct amd_spi_devtype_data *devtype_data;
	struct spi_device *spi_dev;
	struct spi_master *master;
};

static inline u8 amd_spi_readreg8(struct amd_spi *amd_spi, int idx)
{
	return ioread8((u8 __iomem *)amd_spi->io_remap_addr + idx);
}

static inline void amd_spi_writereg8(struct amd_spi *amd_spi, int idx, u8 val)
{
	iowrite8(val, ((u8 __iomem *)amd_spi->io_remap_addr + idx));
}

static void amd_spi_setclear_reg8(struct amd_spi *amd_spi, int idx, u8 set, u8 clear)
{
	u8 tmp = amd_spi_readreg8(amd_spi, idx);

	tmp = (tmp & ~clear) | set;
	amd_spi_writereg8(amd_spi, idx, tmp);
}

static inline u32 amd_spi_readreg32(struct amd_spi *amd_spi, int idx)
{
	return ioread32((u8 __iomem *)amd_spi->io_remap_addr + idx);
}

static inline void amd_spi_writereg32(struct amd_spi *amd_spi, int idx, u32 val)
{
	iowrite32(val, ((u8 __iomem *)amd_spi->io_remap_addr + idx));
}

static void amd_spi_setclear_reg32(struct amd_spi *amd_spi, int idx, u32 set, u32 clear)
{
	u32 tmp = amd_spi_readreg32(amd_spi, idx);

	tmp = (tmp & ~clear) | set;
	amd_spi_writereg32(amd_spi, idx, tmp);
}

static inline void amd_spi_select_chip(struct amd_spi *amd_spi, u8 chip_select)
{
	amd_spi_setclear_reg8(amd_spi, AMD_SPI_ALT_CS_REG, chip_select, AMD_SPI_ALT_CS_MASK);
}

static inline void amd_spi_clear_chip(struct amd_spi *amd_spi, u8 chip_select)
{
	amd_spi_writereg8(amd_spi, AMD_SPI_ALT_CS_REG, chip_select & 0XFC);
}

static void amd_spi_clear_fifo_ptr(struct amd_spi *amd_spi)
{
	amd_spi_setclear_reg32(amd_spi, AMD_SPI_CTRL0_REG, AMD_SPI_FIFO_CLEAR, AMD_SPI_FIFO_CLEAR);
}

static void amd_spi_set_opcode(struct amd_spi *amd_spi, u8 cmd_opcode)
{
	if (!amd_spi->devtype_data->version)
		amd_spi_setclear_reg32(amd_spi, AMD_SPI_CTRL0_REG, cmd_opcode, AMD_SPI_OPCODE_MASK);
	else
		amd_spi_writereg8(amd_spi, AMD_SPI_OPCODE_REG, cmd_opcode);
}

static inline void amd_spi_set_rx_count(struct amd_spi *amd_spi, u8 rx_count)
{
	amd_spi_setclear_reg8(amd_spi, AMD_SPI_RX_COUNT_REG, rx_count, 0xff);
}

static inline void amd_spi_set_tx_count(struct amd_spi *amd_spi, u8 tx_count)
{
	amd_spi_setclear_reg8(amd_spi, AMD_SPI_TX_COUNT_REG, tx_count, 0xff);
}

static int amd_spi_busy_wait(struct amd_spi *amd_spi)
{
	bool spi_busy;
	int timeout = 100000;
	u32 status_reg = amd_spi->devtype_data->spi_status;

	spi_busy = amd_spi_readreg32(amd_spi, status_reg);

	/* poll for SPI bus to become idle */
	while (spi_busy & AMD_SPI_BUSY) {
		usleep_range(10, 20);
		if (timeout-- < 0)
			return -ETIMEDOUT;

		spi_busy = amd_spi_readreg32(amd_spi, status_reg);
	}

	return 0;
}

static int amd_spi_execute_opcode(struct amd_spi *amd_spi)
{
	int ret;

	ret  = amd_spi_busy_wait(amd_spi);
	if (ret)
		return ret;
	/* Set ExecuteOpCode bit in the CTRL0 register */
	if (!amd_spi->devtype_data->version)
		amd_spi_setclear_reg32(amd_spi, AMD_SPI_CTRL0_REG, AMD_SPI_EXEC_CMD,
				       AMD_SPI_EXEC_CMD);
	else
		amd_spi_setclear_reg8(amd_spi, AMD_SPI_CMD_TRIGGER_REG, AMD_SPI_TRIGGER_CMD,
				      AMD_SPI_TRIGGER_CMD);

	ret = amd_spi_busy_wait(amd_spi);
	if (ret)
		return ret;

	return 0;
}

static int amd_spi_master_setup(struct spi_device *spi)
{
	struct spi_master *master = spi->master;
	struct amd_spi *amd_spi = spi_master_get_devdata(master);

	amd_spi_clear_fifo_ptr(amd_spi);

	return 0;
}

static int amd_spi_double_write(struct spi_master *mst, u8 *tx1_buf, u8 tx1_len, u8 *tx2_buf,
				u8 tx2_len)
{
	struct amd_spi *amd_spi = spi_master_get_devdata(mst);
	int i, ret;

	if (tx1_len + tx2_len > AMD_SPI_FIFO_SIZE)
		return -EINVAL;

	amd_spi_clear_fifo_ptr(amd_spi);
	amd_spi_writereg8(amd_spi, AMD_SPI_RX_COUNT_REG, 0);

	amd_spi_set_opcode(amd_spi, tx1_buf[0]);
	tx1_len--;
	tx1_buf++;

	for (i = 0; i < tx1_len; i++)
		amd_spi_writereg8(amd_spi, (u8)(AMD_SPI_FIFO_BASE + i), tx1_buf[i]);

	for (i = 0; i < tx2_len; i++)
		amd_spi_writereg8(amd_spi, (u8)(AMD_SPI_FIFO_BASE + tx1_len + i), tx2_buf[i]);

	amd_spi_set_tx_count(amd_spi, tx1_len + tx2_len);
	ret = amd_spi_execute_opcode(amd_spi);
	if (ret)
		return ret;

	return tx1_len + tx2_len;
}

static int amd_spi_write_read(struct spi_master *mst, u8 *tx_buf, u8 tx_len, u8 *rx_buf, u8 rx_len)
{
	struct amd_spi *amd_spi = spi_master_get_devdata(mst);
	int i, ret;

	if (tx_len + rx_len > AMD_SPI_FIFO_SIZE)
		return -EINVAL;

	amd_spi_clear_fifo_ptr(amd_spi);
	amd_spi_writereg8(amd_spi, AMD_SPI_RX_COUNT_REG, 0);

	if (tx_buf) {
		amd_spi_set_opcode(amd_spi, tx_buf[0]);
		tx_len--;
		tx_buf++;

		amd_spi_set_tx_count(amd_spi, tx_len);
		for (i = 0; i < tx_len; i++)
			amd_spi_writereg8(amd_spi, (u8)(AMD_SPI_FIFO_BASE + i), tx_buf[i]);
	}

	if (rx_buf)
		amd_spi_set_rx_count(amd_spi, rx_len);

	ret = amd_spi_execute_opcode(amd_spi);
	if (ret)
		return ret;

	if (rx_buf) {
		for (i = 0; i < rx_len; i++)
			rx_buf[i] = amd_spi_readreg8(amd_spi, AMD_SPI_FIFO_BASE + tx_len + i);
	}

	return tx_len + 1 + rx_len;
}

static int amd_spi_master_transfer(struct spi_master *mst, struct spi_message *msg)
{
	struct amd_spi *amd_spi = spi_master_get_devdata(mst);
	struct spi_transfer *xfer, *xfer_n;
	struct list_head *pos;
	int ret, count = 0;

	list_for_each(pos, &msg->transfers)
		count++;

	amd_spi_select_chip(amd_spi, msg->spi->chip_select);

	xfer = list_first_entry(&msg->transfers, struct spi_transfer, transfer_list);
	switch (count) {
	case 1:
		ret = amd_spi_write_read(mst, (u8*)xfer->tx_buf, xfer->len,
					      (u8*)xfer->rx_buf, xfer->len);
		if (ret < 0)
			goto complete;
		break;
	case 2:
		xfer_n = list_last_entry(&msg->transfers, struct spi_transfer, transfer_list);
		if (xfer->tx_buf && !xfer->rx_buf) {
			if (xfer_n->rx_buf && !xfer_n->tx_buf) {
				ret = amd_spi_write_read(mst, (u8*)xfer->tx_buf, xfer->len,
							      (u8*)xfer_n->rx_buf, xfer_n->len);
				if (ret < 0)
					goto complete;
				break;
			} else if (xfer_n->tx_buf && !xfer_n->rx_buf) {
				ret = amd_spi_double_write(mst, (u8*)xfer->tx_buf, xfer->len,
								(u8*)xfer_n->tx_buf, xfer_n->len);
				if (ret < 0)
					goto complete;
				break;
			} else {
				dev_err(&mst->dev,
				"Error. Second transfer not a write only or read only\n");
			}
		} else {
			dev_err(&mst->dev, "Error. First transfer not a write only\n");
			goto complete;
		}
		break;
	default:
		dev_err(&mst->dev, "Message with %d transfers is not supported\n", count);
		goto complete;
	}

	msg->actual_length += ret;
	ret = 0;

complete:
	/* complete the transaction */
	msg->status = ret;
	spi_finalize_current_message(mst);

	if (amd_spi->devtype_data->version)
		amd_spi_clear_chip(amd_spi, msg->spi->chip_select);

	return ret;
}

static size_t amd_spi_max_transfer_size(struct spi_device *spi)
{
	return AMD_SPI_FIFO_SIZE;
}

static int amd_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_master *master;
	struct amd_spi *amd_spi;
	struct resource *res;
	int err = 0;
	static u32 spi_reg;

	/* Allocate storage for spi_master and driver private data */
	master = spi_alloc_master(dev, sizeof(struct amd_spi));
	if (!master) {
		dev_err(dev, "Error allocating SPI master\n");
		return -ENOMEM;
	}

	amd_spi = spi_master_get_devdata(master);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	amd_spi->io_remap_addr = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(amd_spi->io_remap_addr)) {
		err = PTR_ERR(amd_spi->io_remap_addr);
		dev_err(dev, "error %d ioremap of SPI registers failed\n", err);
		goto err_free_master;
	}
	amd_spi->devtype_data = device_get_match_data(dev);
	if (!amd_spi->devtype_data) {
		err = -ENODEV;
		goto err_free_master;
	}
	dev_info(dev, "AMDSPI io_remap_address: %p Version %u\n", amd_spi->io_remap_addr, amd_spi->devtype_data->version);
	/* Initialize the spi_master fields */
	master->bus_num = 0;
	master->num_chipselect = 4;
	master->mode_bits = 0;
	master->flags = SPI_MASTER_HALF_DUPLEX | SPI_MASTER_SINGLE_FIFO;
	master->setup = amd_spi_master_setup;
	master->max_transfer_size = amd_spi_max_transfer_size;
	master->transfer_one_message = amd_spi_master_transfer;

	spi_reg = amd_spi_readreg32(amd_spi, AMD_SPI_ENABLE_REG);
	spi_reg = (spi_reg & ~AMD_SPI_ALT_SPD_MASK) | ((SPI_SPD7 << AMD_SPI_ALT_SPD_SHIFT) & AMD_SPI_ALT_SPD_MASK);
	amd_spi_writereg32(amd_spi, AMD_SPI_ENABLE_REG, spi_reg);

	spi_reg = amd_spi_readreg32(amd_spi, AMD_SPI_SPEED_REG);
	spi_reg = (spi_reg & ~AMD_SPI_SPD7_MASK) | ((SPI_4MHz << AMD_SPI_SPD7_SHIFT) & AMD_SPI_SPD7_MASK);
	amd_spi_writereg32(amd_spi, AMD_SPI_SPEED_REG, spi_reg);

	/* Register the controller with SPI framework */
	err = devm_spi_register_master(dev, master);
	if (err) {
		dev_err(dev, "error %d registering SPI controller\n", err);
		goto err_free_master;
	}
	return 0;

err_free_master:
	spi_master_put(master);

	return err;
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id spi_acpi_match[] = {
	{ "AMDI0061",
	.driver_data = (kernel_ulong_t)&spi_v1 },
	{ "AMDI0062",
	.driver_data = (kernel_ulong_t)&spi_v2 },
	{},
};
MODULE_DEVICE_TABLE(acpi, spi_acpi_match);
#endif

static struct platform_driver amd_spi_driver = {
	.driver = {
		.name = "amd_spi",
		.acpi_match_table = ACPI_PTR(spi_acpi_match),
	},
	.probe = amd_spi_probe,
};

module_platform_driver(amd_spi_driver);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Sanjay Mehta <sanju.mehta@amd.com>");
MODULE_AUTHOR("Nehal Bakulchandra Shah <nehal-bakulchandra.shah@amd.com>");
MODULE_AUTHOR("Lucas Tanure <tanureal@opensource.cirrus.com>");
MODULE_DESCRIPTION("AMD SPI Master Controller Driver");
