// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom STB PCIe controller driver.
 * Amon others, the Raspberry Pi 4 (bcm2711).
 *
 * Copyright (C) 2020 Nicolas Saenz Julienne <nsaenzjulienne@suse.de>
 *
 * Based on the Linux driver.
 */

#define FOO
#define DEBUG
#define pr_fmt(fmt) "brcmstb-pcie: " fmt

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <pci.h>
#include <wait_bit.h>

#include <asm/io.h>

#include <linux/bitfield.h>
#include <linux/log2.h>


/* BRCM_PCIE_CAP_REGS - Offset for the mandatory capability config regs */
#define BRCM_PCIE_CAP_REGS				0x00ac

/* Broadcom STB PCIe Register Offsets */
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1				0x0188
#define  PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_MASK	0xc
#define  PCIE_RC_CFG_VENDOR_SPCIFIC_REG1_LITTLE_ENDIAN			0x0

#define PCIE_RC_CFG_PRIV1_ID_VAL3			0x043c
#define  PCIE_RC_CFG_PRIV1_ID_VAL3_CLASS_CODE_MASK	0xffffff

#define PCIE_MISC_MISC_CTRL				0x4008
#define  PCIE_MISC_MISC_CTRL_SCB_ACCESS_EN_MASK		0x1000
#define  PCIE_MISC_MISC_CTRL_CFG_READ_UR_MODE_MASK	0x2000
#define  PCIE_MISC_MISC_CTRL_MAX_BURST_SIZE_MASK	0x300000
#define  PCIE_MISC_MISC_CTRL_MAX_BURST_SIZE_128		0x0
#define  PCIE_MISC_MISC_CTRL_SCB0_SIZE_MASK		0xf8000000

#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO		0x400c
#define PCIE_MEM_WIN0_LO(win)	\
		PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO + ((win) * 4)

#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI		0x4010
#define PCIE_MEM_WIN0_HI(win)	\
		PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI + ((win) * 4)

#define PCIE_MISC_RC_BAR1_CONFIG_LO			0x402c
#define  PCIE_MISC_RC_BAR1_CONFIG_LO_SIZE_MASK		0x1f

#define PCIE_MISC_RC_BAR2_CONFIG_LO			0x4034
#define  PCIE_MISC_RC_BAR2_CONFIG_LO_SIZE_MASK		0x1f
#define PCIE_MISC_RC_BAR2_CONFIG_HI			0x4038

#define PCIE_MISC_RC_BAR3_CONFIG_LO			0x403c
#define  PCIE_MISC_RC_BAR3_CONFIG_LO_SIZE_MASK		0x1f

#define PCIE_MISC_PCIE_STATUS				0x4068
#define  PCIE_MISC_PCIE_STATUS_PCIE_PORT_MASK		0x80
#define  PCIE_MISC_PCIE_STATUS_PCIE_DL_ACTIVE_MASK	0x20
#define  PCIE_MISC_PCIE_STATUS_PCIE_PHYLINKUP_MASK	0x10
#define  PCIE_MISC_PCIE_STATUS_PCIE_LINK_IN_L23_MASK	0x40

#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT		0x4070
#define  PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_LIMIT_MASK	0xfff00000
#define  PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_BASE_MASK	0xfff0
#define  PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_BASE_HI_SHIFT	12
#define PCIE_MEM_WIN0_BASE_LIMIT(win)	\
		PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT + ((win) * 4)

#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI			0x4080
#define  PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI_BASE_MASK	0xff
#define PCIE_MEM_WIN0_BASE_HI(win)	\
		PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI + ((win) * 8)

#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI			0x4084
#define  PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI_LIMIT_MASK	0xff
#define PCIE_MEM_WIN0_LIMIT_HI(win)	\
		PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI + ((win) * 8)

#define PCIE_MISC_HARD_PCIE_HARD_DEBUG					0x4204
#define  PCIE_MISC_HARD_PCIE_HARD_DEBUG_CLKREQ_DEBUG_ENABLE_MASK	0x2
#define  PCIE_MISC_HARD_PCIE_HARD_DEBUG_SERDES_IDDQ_MASK		0x08000000

#define PCIE_EXT_CFG_DATA				0x8000

#define PCIE_EXT_CFG_INDEX				0x9000
#define  PCIE_EXT_BUSNUM_SHIFT				20
#define  PCIE_EXT_SLOT_SHIFT				15
#define  PCIE_EXT_FUNC_SHIFT				12

#define PCIE_RGR1_SW_INIT_1				0x9210
#define  PCIE_RGR1_SW_INIT_1_PERST_MASK			0x1
#define  PCIE_RGR1_SW_INIT_1_INIT_MASK			0x2

/* PCIe parameters */
#define BRCM_NUM_PCIE_OUT_WINS		0x4

#define PCI_EXP_LNKCTL2           48      /* Link Control 2 */
#define PCI_EXP_LNKCAP            12      /* Link Capabilities */
#define  PCI_EXP_LNKCAP_SLS       0x0000000f /* Supported Link Speeds */

struct brcm_pcie {
#if 1
	fdt_addr_t base;
#else
	void __iomem *base;
#endif
};

/* Configuration space read/write support */
static inline int brcm_pcie_cfg_index(pci_dev_t bdf, int reg)
{
#if 0
       return PCI_DEV(bdf) << PCIE_EXT_SLOT_SHIFT
               | PCI_FUNC(bdf) << PCIE_EXT_FUNC_SHIFT
               | PCI_BUS(bdf) << PCIE_EXT_BUSNUM_SHIFT;
#else
	int idx = 0;

	idx |= (PCI_DEV(bdf) & 0x1f) << PCIE_EXT_SLOT_SHIFT;
	idx |= (PCI_FUNC(bdf) & 0x07) << PCIE_EXT_FUNC_SHIFT;
	idx |= PCI_BUS(bdf) << PCIE_EXT_BUSNUM_SHIFT;
	idx |= (reg & ~3);

	return idx;
#endif
}

#if 1
static fdt_addr_t brcm_pcie_config_access(struct brcm_pcie *pcie, pci_dev_t bdf,
					  int where)
{
	fdt_addr_t base = pcie->base;

	/* Accesses to the RC go right to the RC registers if slot==0 */
	if (!PCI_BUS(bdf))
		return PCI_DEV(bdf) ? (fdt_addr_t)NULL : base + where;

	/* For devices, write to the config space index register */
	writel(brcm_pcie_cfg_index(bdf, 0), base + PCIE_EXT_CFG_INDEX);
	return base + PCIE_EXT_CFG_DATA + where;
}
#else
static int brcm_pcie_config_access(struct udevice *dev, pci_dev_t bdf,
				uint offset, void **paddress)
{
	struct brcm_pcie *pcie = dev_get_priv(dev);
	int idx;

	//debug("PCIE CFG ADDR: (b.d.f)=(%02d.%02d.%02d)\n",
	//	 PCI_BUS(bdf), PCI_DEV(bdf), PCI_FUNC(bdf));

	/* Accesses to the RC go right to the RC registers if slot==0 */
	if (PCI_BUS(bdf) == 0) {
		if (PCI_DEV(bdf) != 0) {
			return -ENODEV;
		}
		*paddress = pcie->base + offset;
		return 0;
	}

	/* For devices, write to the config space index register */
	idx = brcm_pcie_cfg_index(bdf, 0);
	writel(idx, pcie->base + PCIE_EXT_CFG_INDEX);
	*paddress = pcie->base + PCIE_EXT_CFG_DATA + offset;
	return 0;
}

#endif
static int brcm_pcie_read_config(struct udevice *bus, pci_dev_t bdf,
				 uint offset, ulong *valuep,
				 enum pci_size_t size)
{
#if 1
	struct brcm_pcie *pcie = dev_get_platdata(bus);
	fdt_addr_t addr;

	addr = brcm_pcie_config_access(pcie, bdf, offset);
	if (!addr) {
		*valuep = ~0;
		return 0;
	}

       if (size == PCI_SIZE_8)
               *valuep = readb(addr);
       else if (size == PCI_SIZE_16)
               *valuep = readw(addr);
       else
               *valuep = readl(addr);

	return 0;
#else
	return pci_generic_mmap_read_config(bus, brcm_pcie_config_access,
					    bdf, offset, valuep, size);
#endif
}

static int brcm_pcie_write_config(struct udevice *bus, pci_dev_t bdf,
				  uint offset, ulong value,
				  enum pci_size_t size)
{
#if 1
	struct brcm_pcie *pcie = dev_get_platdata(bus);
	fdt_addr_t addr;

	addr = brcm_pcie_config_access(pcie, bdf, offset);
	if (!addr)
		return -EFAULT;

       if (size == PCI_SIZE_8)
               writeb(value, addr);
       else if (size == PCI_SIZE_16)
               writew(value, addr);
       else
               writel(value, addr);
	return 0;
#else
	return pci_generic_mmap_write_config(bus, brcm_pcie_config_access,
					     bdf, offset, value, size);
#endif
}

static inline void brcm_pcie_bridge_sw_init_set(struct brcm_pcie *pcie, u32 val)
{
	u32 tmp;

	tmp = readl(pcie->base + PCIE_RGR1_SW_INIT_1);
	u32p_replace_bits(&tmp, val, PCIE_RGR1_SW_INIT_1_INIT_MASK);
	writel(tmp, pcie->base + PCIE_RGR1_SW_INIT_1);
}

static inline void brcm_pcie_perst_set(struct brcm_pcie *pcie, u32 val)
{
	u32 tmp;

	tmp = readl(pcie->base + PCIE_RGR1_SW_INIT_1);
	u32p_replace_bits(&tmp, val, PCIE_RGR1_SW_INIT_1_PERST_MASK);
	writel(tmp, pcie->base + PCIE_RGR1_SW_INIT_1);
}

static void brcm_pcie_set_outbound_win(struct brcm_pcie *pcie,
				       unsigned int win, u64 cpu_addr,
				       u64 pcie_addr, u64 size)
{
	u32 cpu_addr_mb_high, limit_addr_mb_high;
	phys_addr_t cpu_addr_mb, limit_addr_mb;
	int high_addr_shift;
	u32 tmp;

	/* Set the base of the pcie_addr window */
	writel(lower_32_bits(pcie_addr), pcie->base + PCIE_MEM_WIN0_LO(win));
	writel(upper_32_bits(pcie_addr), pcie->base + PCIE_MEM_WIN0_HI(win));

	/* Write the addr base & limit lower bits (in MBs) */
	cpu_addr_mb = cpu_addr / SZ_1M;
	limit_addr_mb = (cpu_addr + size - 1) / SZ_1M;

	tmp = readl(pcie->base + PCIE_MEM_WIN0_BASE_LIMIT(win));
	u32p_replace_bits(&tmp, cpu_addr_mb,
			  PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_BASE_MASK);
	u32p_replace_bits(&tmp, limit_addr_mb,
			  PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_LIMIT_MASK);
	writel(tmp, pcie->base + PCIE_MEM_WIN0_BASE_LIMIT(win));

	/* Write the cpu & limit addr upper bits */
	high_addr_shift = PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_BASE_HI_SHIFT;
	cpu_addr_mb_high = cpu_addr_mb >> high_addr_shift;
	tmp = readl(pcie->base + PCIE_MEM_WIN0_BASE_HI(win));
	u32p_replace_bits(&tmp, cpu_addr_mb_high,
			  PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI_BASE_MASK);
	writel(tmp, pcie->base + PCIE_MEM_WIN0_BASE_HI(win));

	limit_addr_mb_high = limit_addr_mb >> high_addr_shift;
	tmp = readl(pcie->base + PCIE_MEM_WIN0_LIMIT_HI(win));
	u32p_replace_bits(&tmp, limit_addr_mb_high,
			  PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI_LIMIT_MASK);
	writel(tmp, pcie->base + PCIE_MEM_WIN0_LIMIT_HI(win));
}

/*
 * This is to convert the size of the inbound "BAR" region to the
 * non-linear values of PCIE_X_MISC_RC_BAR[123]_CONFIG_LO.SIZE
 */
static int brcm_pcie_encode_ibar_size(u64 size)
{
	int log2_in = ilog2(size);

	if (log2_in >= 12 && log2_in <= 15)
		/* Covers 4KB to 32KB (inclusive) */
		return (log2_in - 12) + 0x1c;
	else if (log2_in >= 16 && log2_in <= 35)
		/* Covers 64KB to 32GB, (inclusive) */
		return log2_in - 15;
	/* Something is awry so disable */
	return 0;
}

static int brcm_pcie_probe(struct udevice *dev)
{
	struct pci_controller *controller = dev_get_uclass_priv(dev);
	u64 rc_bar2_offset = 0;
	u64 rc_bar2_size = 0x100000000ULL;
	struct brcm_pcie *pcie = dev_get_platdata(dev);
	unsigned int scb_size_val;
	int num_out_wins = 0;
	int i, ret;
	u32 tmp;

	fprintf(stderr, "PROBE\n");

	/* Reset the bridge */
	brcm_pcie_bridge_sw_init_set(pcie, 1);

	udelay(150);

	/* Take the bridge out of reset */
	brcm_pcie_bridge_sw_init_set(pcie, 0);

	tmp = readl(pcie->base + PCIE_MISC_HARD_PCIE_HARD_DEBUG);
	tmp &= ~PCIE_MISC_HARD_PCIE_HARD_DEBUG_SERDES_IDDQ_MASK;
	writel(tmp, pcie->base + PCIE_MISC_HARD_PCIE_HARD_DEBUG);
	/* Wait for SerDes to be stable */
	udelay(150);

	/* Set SCB_MAX_BURST_SIZE, CFG_READ_UR_MODE, SCB_ACCESS_EN */
	u32p_replace_bits(&tmp, 1, PCIE_MISC_MISC_CTRL_SCB_ACCESS_EN_MASK);
	u32p_replace_bits(&tmp, 1, PCIE_MISC_MISC_CTRL_CFG_READ_UR_MODE_MASK);
	u32p_replace_bits(&tmp, PCIE_MISC_MISC_CTRL_MAX_BURST_SIZE_128,
			  PCIE_MISC_MISC_CTRL_MAX_BURST_SIZE_MASK);
	writel(tmp, pcie->base + PCIE_MISC_MISC_CTRL);

	/* Setup inbound memory view */
	tmp = lower_32_bits(rc_bar2_offset);
	u32p_replace_bits(&tmp, brcm_pcie_encode_ibar_size(rc_bar2_size),
			  PCIE_MISC_RC_BAR2_CONFIG_LO_SIZE_MASK);
	writel(tmp, pcie->base + PCIE_MISC_RC_BAR2_CONFIG_LO);
	writel(upper_32_bits(rc_bar2_offset),
	       pcie->base + PCIE_MISC_RC_BAR2_CONFIG_HI);

	scb_size_val = rc_bar2_size ?
		       ilog2(rc_bar2_size) - 15 : 0xf; /* 0xf is 1GB */
	tmp = readl(pcie->base + PCIE_MISC_MISC_CTRL);
	u32p_replace_bits(&tmp, scb_size_val,
			  PCIE_MISC_MISC_CTRL_SCB0_SIZE_MASK);
	writel(tmp, pcie->base + PCIE_MISC_MISC_CTRL);

	/* disable the PCIe->GISB memory window (RC_BAR1) */
	tmp = readl(pcie->base + PCIE_MISC_RC_BAR1_CONFIG_LO);
	tmp &= ~PCIE_MISC_RC_BAR1_CONFIG_LO_SIZE_MASK;
	writel(tmp, pcie->base + PCIE_MISC_RC_BAR1_CONFIG_LO);

	/* disable the PCIe->SCB memory window (RC_BAR3) */
	tmp = readl(pcie->base + PCIE_MISC_RC_BAR3_CONFIG_LO);
	tmp &= ~PCIE_MISC_RC_BAR3_CONFIG_LO_SIZE_MASK;
	writel(tmp, pcie->base + PCIE_MISC_RC_BAR3_CONFIG_LO);

	/* set gen 2 TODO Read from dt */
	{
	int gen = 2;
	u16 lnkctl2 = readw(pcie->base + BRCM_PCIE_CAP_REGS + PCI_EXP_LNKCTL2);
	u32 lnkcap = readl(pcie->base + BRCM_PCIE_CAP_REGS + PCI_EXP_LNKCAP);

	lnkcap = (lnkcap & ~PCI_EXP_LNKCAP_SLS) | gen;
	writel(lnkcap, pcie->base + BRCM_PCIE_CAP_REGS + PCI_EXP_LNKCAP);

	lnkctl2 = (lnkctl2 & ~0xf) | gen;
	writew(lnkctl2, pcie->base + BRCM_PCIE_CAP_REGS + PCI_EXP_LNKCTL2);
	}

	/* Unassert the fundamental reset */
	brcm_pcie_perst_set(pcie, 0);

	/*
	 * Give the RC/EP time to wake up, before trying to configure RC.
	 * Intermittently check status for link-up, up to a total of 100ms.
	 */
	ret = wait_for_bit_le32((void *)pcie->base + PCIE_MISC_PCIE_STATUS,
				 PCIE_MISC_PCIE_STATUS_PCIE_DL_ACTIVE_MASK |
					PCIE_MISC_PCIE_STATUS_PCIE_PHYLINKUP_MASK,
				 true, 100, false);
	if (ret)
		return ret;

	for (i = 0; i < controller->region_count; i++) {
		struct pci_region *reg = &controller->regions[i];

		if (reg->flags != PCI_REGION_MEM)
			continue;

		if (num_out_wins >= BRCM_NUM_PCIE_OUT_WINS) {
			return -EINVAL;
		}

		brcm_pcie_set_outbound_win(pcie, num_out_wins, reg->phys_start,
					   reg->bus_start, reg->size);

		num_out_wins++;
	}

	/*
	 * For config space accesses on the RC, show the right class for
	 * a PCIe-PCIe bridge (the default setting is to be EP mode).
	 */
	tmp = readl(pcie->base + PCIE_RC_CFG_PRIV1_ID_VAL3);
	u32p_replace_bits(&tmp, 0x060400,
			  PCIE_RC_CFG_PRIV1_ID_VAL3_CLASS_CODE_MASK);
	writel(tmp, pcie->base + PCIE_RC_CFG_PRIV1_ID_VAL3);

	dev_info(dev, "link up.");

	/* PCIe->SCB endian mode for BAR */
	tmp = readl(pcie->base + PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1);
	u32p_replace_bits(&tmp, PCIE_RC_CFG_VENDOR_SPCIFIC_REG1_LITTLE_ENDIAN,
		PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_MASK);
	writel(tmp, pcie->base + PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1);

	/*
	 * Refclk from RC should be gated with CLKREQ# input when ASPM L0s,L1
	 * is enabled => setting the CLKREQ_DEBUG_ENABLE field to 1.
	 */
	tmp = readl(pcie->base + PCIE_MISC_HARD_PCIE_HARD_DEBUG);
	tmp |= PCIE_MISC_HARD_PCIE_HARD_DEBUG_CLKREQ_DEBUG_ENABLE_MASK;
	writel(tmp, pcie->base + PCIE_MISC_HARD_PCIE_HARD_DEBUG);

	fprintf(stderr, "PROBE DONE\n");

	return 0;
}

static int brcm_pcie_ofdata_to_platdata(struct udevice *dev)
{
	struct brcm_pcie *pcie = dev_get_platdata(dev);

	fprintf(stderr, "OFDATA_TO_PLATDATA\n");

	pcie->base = devfdt_get_addr_index(dev, 0);
	if (pcie->base == 0)
		return -EINVAL;

	fprintf(stderr, "BASE = 0x%ld\n", pcie->base);

	return 0;
}

static const struct dm_pci_ops brcm_pcie_ops = {
	.read_config	= brcm_pcie_read_config,
	.write_config	= brcm_pcie_write_config,
};

static const struct udevice_id brcm_pcie_ids[] = {
	{ .compatible = "brcm,bcm2711-pcie" },
	{ .compatible = "brcm,bcm7211-pcie" },
	{  },
};

U_BOOT_DRIVER(bcrmstb_pcie) = {
	.name			= "bcrmstb_pcie",
	.id			= UCLASS_PCI,
	.of_match		= brcm_pcie_ids,
	.ops			= &brcm_pcie_ops,
	.probe			= brcm_pcie_probe,
	.ofdata_to_platdata	= brcm_pcie_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct brcm_pcie),
};
