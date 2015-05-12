# link order matters; don't link the more generic brcmstb_nand.o before the
# more specific iproc_nand.o, for instance
obj-$(CONFIG_MTD_NAND_BRCMSTB)		+= iproc_nand.o
obj-$(CONFIG_MTD_NAND_BRCMSTB)		+= brcmstb_nand.o
obj-$(CONFIG_MTD_NAND_BRCMSTB)		+= brcmnand.o
