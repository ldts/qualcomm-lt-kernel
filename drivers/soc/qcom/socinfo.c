// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2009-2017, The Linux Foundation. All rights reserved.
 * Copyright (c) 2017-2019, Linaro Ltd.
 */

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/soc/qcom/smem.h>
#include <linux/string.h>
#include <linux/sys_soc.h>
#include <linux/types.h>

/*
 * SoC version type with major number in the upper 16 bits and minor
 * number in the lower 16 bits.
 */
#define SOCINFO_MAJOR(ver) (((ver) >> 16) & 0xffff)
#define SOCINFO_MINOR(ver) ((ver) & 0xffff)

#define SMEM_SOCINFO_BUILD_ID_LENGTH           32

/*
 * SMEM item ids, used to acquire handles to respective
 * SMEM region.
 */
#define SMEM_HW_SW_BUILD_ID            137

#ifdef CONFIG_DEBUG_FS
#define SMEM_IMAGE_VERSION_BLOCKS_COUNT        32
#define SMEM_IMAGE_VERSION_SIZE                4096
#define SMEM_IMAGE_VERSION_NAME_SIZE           75
#define SMEM_IMAGE_VERSION_VARIANT_SIZE        20
#define SMEM_IMAGE_VERSION_OEM_SIZE            32

/*
 * SMEM Image table indices
 */
#define SMEM_IMAGE_TABLE_BOOT_INDEX     0
#define SMEM_IMAGE_TABLE_TZ_INDEX       1
#define SMEM_IMAGE_TABLE_RPM_INDEX      3
#define SMEM_IMAGE_TABLE_APPS_INDEX     10
#define SMEM_IMAGE_TABLE_MPSS_INDEX     11
#define SMEM_IMAGE_TABLE_ADSP_INDEX     12
#define SMEM_IMAGE_TABLE_CNSS_INDEX     13
#define SMEM_IMAGE_TABLE_VIDEO_INDEX    14
#define SMEM_IMAGE_VERSION_TABLE       469

/* pmic model info */
static const char *const pmic_model[] = {
	[0]  = "Unknown PMIC model",
	[9]  = "PM8994",
	[11] = "PM8916",
	[13] = "PM8058",
	[14] = "PM8028",
	[15] = "PM8901",
	[16] = "PM8027",
	[17] = "ISL9519",
	[18] = "PM8921",
	[19] = "PM8018",
	[20] = "PM8015",
	[21] = "PM8014",
	[22] = "PM8821",
	[23] = "PM8038",
	[24] = "PM8922",
	[25] = "PM8917",
};
#endif /* CONFIG_DEBUG_FS */

/* Socinfo SMEM item structure */
struct socinfo {
	__le32 fmt;
	__le32 id;
	__le32 ver;
	char build_id[SMEM_SOCINFO_BUILD_ID_LENGTH];
	/* Version 2 */
	__le32 raw_id;
	__le32 raw_ver;
	/* Version 3 */
	__le32 hw_plat;
	/* Version 4 */
	__le32 plat_ver;
	/* Version 5 */
	__le32 accessory_chip;
	/* Version 6 */
	__le32 hw_plat_subtype;
	/* Version 7 */
	__le32 pmic_model;
	__le32 pmic_die_rev;
	/* Version 8 */
	__le32 pmic_model_1;
	__le32 pmic_die_rev_1;
	__le32 pmic_model_2;
	__le32 pmic_die_rev_2;
	/* Version 9 */
	__le32 foundry_id;
	/* Version 10 */
	__le32 serial_num;
	/* Version 11 */
	__le32 num_pmics;
	__le32 pmic_array_offset;
	/* Version 12 */
	__le32 chip_family;
	__le32 raw_device_family;
	__le32 raw_device_num;
};

#ifdef CONFIG_DEBUG_FS
struct smem_image_version {
	char name[SMEM_IMAGE_VERSION_NAME_SIZE];
	char variant[SMEM_IMAGE_VERSION_VARIANT_SIZE];
	char pad;
	char oem[SMEM_IMAGE_VERSION_OEM_SIZE];
};
#endif /* CONFIG_DEBUG_FS */

struct qcom_socinfo {
	struct soc_device *soc_dev;
	struct soc_device_attribute attr;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dbg_root;
	struct dentry *boot, *tz, *rpm, *apps, *mpss, *adsp, *cnss, *video;
#endif /* CONFIG_DEBUG_FS */
	struct socinfo *socinfo;
};

struct soc_of_id {
	unsigned int id;
	const char *name;
};

static const struct soc_of_id soc_of_id[] = {
	{87, "MSM8960"},
	{109, "APQ8064"},
	{122, "MSM8660A"},
	{123, "MSM8260A"},
	{124, "APQ8060A"},
	{126, "MSM8974"},
	{130, "MPQ8064"},
	{138, "MSM8960AB"},
	{139, "APQ8060AB"},
	{140, "MSM8260AB"},
	{141, "MSM8660AB"},
	{178, "APQ8084"},
	{184, "APQ8074"},
	{185, "MSM8274"},
	{186, "MSM8674"},
	{194, "MSM8974PRO"},
	{206, "MSM8916"},
	{208, "APQ8074-AA"},
	{209, "APQ8074-AB"},
	{210, "APQ8074PRO"},
	{211, "MSM8274-AA"},
	{212, "MSM8274-AB"},
	{213, "MSM8274PRO"},
	{214, "MSM8674-AA"},
	{215, "MSM8674-AB"},
	{216, "MSM8674PRO"},
	{217, "MSM8974-AA"},
	{218, "MSM8974-AB"},
	{246, "MSM8996"},
	{247, "APQ8016"},
	{248, "MSM8216"},
	{249, "MSM8116"},
	{250, "MSM8616"},
	{291, "APQ8096"},
	{305, "MSM8996SG"},
	{310, "MSM8996AU"},
	{311, "APQ8096AU"},
	{312, "APQ8096SG"},
};

static const char *socinfo_machine(struct device *dev, unsigned int id)
{
	int idx;

	for (idx = 0; idx < ARRAY_SIZE(soc_of_id); idx++) {
		if (soc_of_id[idx].id == id)
			return soc_of_id[idx].name;
	}

	if (IS_ERR(soc_of_id[idx].name))
		dev_err(dev, "Unknown soc id\n");

	return NULL;
}

#ifdef CONFIG_DEBUG_FS

#define UINT_SHOW(name, attr)						\
static int qcom_show_##name(struct seq_file *seq, void *p)		\
{									\
	struct socinfo *socinfo = seq->private;				\
	seq_printf(seq, "%u\n", le32_to_cpu(socinfo->attr));		\
	return 0;							\
}									\
static int qcom_open_##name(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, qcom_show_##name, inode->i_private);	\
}									\
									\
static const struct file_operations qcom_ ##name## _ops = {		\
	.open = qcom_open_##name,					\
	.read = seq_read,						\
	.llseek = seq_lseek,						\
	.release = single_release,					\
}

#define DEBUGFS_UINT_ADD(name)						\
	debugfs_create_file(__stringify(name), 0400,			\
			    qcom_socinfo->dbg_root,			\
			    qcom_socinfo->socinfo, &qcom_ ##name## _ops)

#define HEX_SHOW(name, attr)						\
static int qcom_show_##name(struct seq_file *seq, void *p)		\
{									\
	struct socinfo *socinfo = seq->private;				\
	seq_printf(seq, "0x%x\n", le32_to_cpu(socinfo->attr));		\
	return 0;							\
}									\
static int qcom_open_##name(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, qcom_show_##name, inode->i_private);	\
}									\
									\
static const struct file_operations qcom_ ##name## _ops = {		\
	.open = qcom_open_##name,					\
	.read = seq_read,						\
	.llseek = seq_lseek,						\
	.release = single_release,					\
}

#define DEBUGFS_HEX_ADD(name)						\
	debugfs_create_file(__stringify(name), 0400,			\
			    qcom_socinfo->dbg_root,			\
			    qcom_socinfo->socinfo, &qcom_ ##name## _ops)


#define QCOM_OPEN(name, _func)						\
static int qcom_open_##name(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, _func, inode->i_private);		\
}									\
									\
static const struct file_operations qcom_ ##name## _ops = {		\
	.open = qcom_open_##name,					\
	.read = seq_read,						\
	.llseek = seq_lseek,						\
	.release = single_release,					\
}

#define DEBUGFS_ADD(name)						\
	debugfs_create_file(__stringify(name), 0400,			\
			    qcom_socinfo->dbg_root,			\
			    qcom_socinfo->socinfo, &qcom_ ##name## _ops)


static int qcom_show_build_id(struct seq_file *seq, void *p)
{
	struct socinfo *socinfo = seq->private;

	seq_printf(seq, "%s\n", socinfo->build_id);

	return 0;
}

static int qcom_show_accessory_chip(struct seq_file *seq, void *p)
{
	struct socinfo *socinfo = seq->private;

	seq_printf(seq, "%d\n", le32_to_cpu(socinfo->accessory_chip));

	return 0;
}

static int qcom_show_platform_subtype(struct seq_file *seq, void *p)
{
	struct socinfo *socinfo = seq->private;
	int subtype = le32_to_cpu(socinfo->hw_plat_subtype);

	if (subtype < 0)
		return -EINVAL;

	seq_printf(seq, "%u\n", subtype);

	return 0;
}

static int qcom_show_pmic_model(struct seq_file *seq, void *p)
{
	struct socinfo *socinfo = seq->private;
	int model = SOCINFO_MINOR(le32_to_cpu(socinfo->pmic_model));

	if (model < 0)
		return -EINVAL;

	seq_printf(seq, "%s\n", pmic_model[model]);

	return 0;
}

static int qcom_show_pmic_die_revision(struct seq_file *seq, void *p)
{
	struct socinfo *socinfo = seq->private;

	seq_printf(seq, "%u.%u\n",
		   SOCINFO_MAJOR(le32_to_cpu(socinfo->pmic_die_rev)),
		   SOCINFO_MINOR(le32_to_cpu(socinfo->pmic_die_rev)));

	return 0;
}

UINT_SHOW(raw_version, raw_ver);
UINT_SHOW(hardware_platform, hw_plat);
UINT_SHOW(platform_version, plat_ver);
UINT_SHOW(foundry_id, foundry_id);
HEX_SHOW(chip_family, chip_family);
HEX_SHOW(raw_device_family, raw_device_family);
HEX_SHOW(raw_device_number, raw_device_num);
QCOM_OPEN(build_id, qcom_show_build_id);
QCOM_OPEN(accessory_chip, qcom_show_accessory_chip);
QCOM_OPEN(pmic_model, qcom_show_pmic_model);
QCOM_OPEN(platform_subtype, qcom_show_platform_subtype);
QCOM_OPEN(pmic_die_revision, qcom_show_pmic_die_revision);

#define IMAGE_SHOW_NAME(attr)						  \
static int show_ ##attr## _name(struct seq_file *seq, void *p)		  \
{									  \
	struct smem_image_version *image_version = seq->private;	  \
	seq_puts(seq, image_version->name);				  \
	seq_puts(seq, "\n");						  \
	return 0;							  \
}									  \
static int open_ ##attr## _name(struct inode *inode, struct file *file)	  \
{									  \
	return single_open(file, show_ ##attr## _name, inode->i_private); \
}									  \
									  \
static const struct file_operations qcom_ ##attr## _name_ops = {	  \
	.open = open_ ##attr## _name,					  \
	.read = seq_read,						  \
	.llseek = seq_lseek,						  \
	.release = single_release,					  \
}									  \

#define DEBUGFS_IMAGE_NAME(fname, attr, index)				  \
debugfs_create_file(__stringify(fname), 0400, qcom_socinfo->attr,	  \
		    &smem_image_version[index], &qcom_ ##attr## _name_ops)

#define IMAGE_SHOW_VARIANT(attr)					     \
static int show_ ##attr## _variant(struct seq_file *seq, void *p)	     \
{									     \
	struct smem_image_version *image_version = seq->private;	     \
	seq_puts(seq, image_version->variant);				     \
	seq_puts(seq, "\n");						     \
	return 0;							     \
}									     \
static int open_ ##attr## _variant(struct inode *inode, struct file *file)   \
{                                                                            \
	return single_open(file, show_ ##attr## _variant, inode->i_private); \
}									     \
									     \
static const struct file_operations qcom_ ##attr## _variant_ops = {	     \
	.open = open_ ##attr## _variant,				     \
	.read = seq_read,						     \
	.llseek = seq_lseek,						     \
	.release = single_release,					     \
}

#define DEBUGFS_IMAGE_VARIANT(fname, attr, index)			    \
debugfs_create_file(__stringify(fname), 0400, qcom_socinfo->attr,	    \
		    &smem_image_version[index], &qcom_ ##attr## _variant_ops)

#define IMAGE_SHOW_OEM(attr)						 \
static int show_ ##attr## _oem(struct seq_file *seq, void *p)		 \
{									 \
	struct smem_image_version *image_version = seq->private;	 \
	seq_puts(seq, image_version->oem);				 \
	seq_puts(seq, "\n");						 \
	return 0;							 \
}									 \
static int open_ ##attr## _oem(struct inode *inode, struct file *file)	 \
{									 \
	return single_open(file, show_ ##attr## _oem, inode->i_private); \
}									 \
									 \
static const struct file_operations qcom_ ##attr## _oem_ops = {		 \
	.open = open_ ##attr## _oem,					 \
	.read = seq_read,						 \
	.llseek = seq_lseek,						 \
	.release = single_release,					 \
}

#define DEBUGFS_IMAGE_OEM(fname, attr, index)				 \
debugfs_create_file(__stringify(fname), 0400, qcom_socinfo->attr,	 \
		    &smem_image_version[index], &qcom_ ##attr## _oem_ops)

#define IMAGE_SHOW(name)	  \
	IMAGE_SHOW_NAME(name);    \
	IMAGE_SHOW_VARIANT(name); \
	IMAGE_SHOW_OEM(name)	  \

IMAGE_SHOW(boot);
IMAGE_SHOW(tz);
IMAGE_SHOW(rpm);
IMAGE_SHOW(apps);
IMAGE_SHOW(mpss);
IMAGE_SHOW(adsp);
IMAGE_SHOW(cnss);
IMAGE_SHOW(video);

static void socinfo_debugfs_init(struct qcom_socinfo *qcom_socinfo)
{
	struct smem_image_version *smem_image_version;
	size_t size;

	qcom_socinfo->dbg_root = debugfs_create_dir("qcom_socinfo", NULL);

	DEBUGFS_UINT_ADD(raw_version);
	DEBUGFS_UINT_ADD(hardware_platform);
	DEBUGFS_UINT_ADD(platform_version);
	DEBUGFS_UINT_ADD(foundry_id);
	DEBUGFS_HEX_ADD(chip_family);
	DEBUGFS_HEX_ADD(raw_device_family);
	DEBUGFS_HEX_ADD(raw_device_number);
	DEBUGFS_ADD(build_id);
	DEBUGFS_ADD(accessory_chip);
	DEBUGFS_ADD(pmic_model);
	DEBUGFS_ADD(platform_subtype);
	DEBUGFS_ADD(pmic_die_revision);

	smem_image_version = qcom_smem_get(QCOM_SMEM_HOST_ANY,
					   SMEM_IMAGE_VERSION_TABLE,
					   &size);

	qcom_socinfo->boot = debugfs_create_dir("boot",
						qcom_socinfo->dbg_root);

	DEBUGFS_IMAGE_NAME(name, boot, SMEM_IMAGE_TABLE_BOOT_INDEX);
	DEBUGFS_IMAGE_VARIANT(variant, boot, SMEM_IMAGE_TABLE_BOOT_INDEX);
	DEBUGFS_IMAGE_OEM(oem, boot, SMEM_IMAGE_TABLE_BOOT_INDEX);

	qcom_socinfo->tz = debugfs_create_dir("tz",
					      qcom_socinfo->dbg_root);

	DEBUGFS_IMAGE_NAME(name, tz, SMEM_IMAGE_TABLE_TZ_INDEX);
	DEBUGFS_IMAGE_VARIANT(variant, tz, SMEM_IMAGE_TABLE_TZ_INDEX);
	DEBUGFS_IMAGE_OEM(oem, tz, SMEM_IMAGE_TABLE_TZ_INDEX);

	qcom_socinfo->rpm = debugfs_create_dir("rpm",
					       qcom_socinfo->dbg_root);

	DEBUGFS_IMAGE_NAME(name, rpm, SMEM_IMAGE_TABLE_RPM_INDEX);
	DEBUGFS_IMAGE_VARIANT(variant, rpm, SMEM_IMAGE_TABLE_RPM_INDEX);
	DEBUGFS_IMAGE_OEM(oem, rpm, SMEM_IMAGE_TABLE_RPM_INDEX);

	qcom_socinfo->apps = debugfs_create_dir("apps",
						qcom_socinfo->dbg_root);

	DEBUGFS_IMAGE_NAME(name, apps, SMEM_IMAGE_TABLE_APPS_INDEX);
	DEBUGFS_IMAGE_VARIANT(variant, apps, SMEM_IMAGE_TABLE_APPS_INDEX);
	DEBUGFS_IMAGE_OEM(oem, apps, SMEM_IMAGE_TABLE_APPS_INDEX);

	qcom_socinfo->mpss = debugfs_create_dir("mpss",
						qcom_socinfo->dbg_root);

	DEBUGFS_IMAGE_NAME(name, mpss, SMEM_IMAGE_TABLE_MPSS_INDEX);
	DEBUGFS_IMAGE_VARIANT(variant, mpss, SMEM_IMAGE_TABLE_MPSS_INDEX);
	DEBUGFS_IMAGE_OEM(oem, mpss, SMEM_IMAGE_TABLE_MPSS_INDEX);

	qcom_socinfo->adsp = debugfs_create_dir("adsp",
						qcom_socinfo->dbg_root);

	DEBUGFS_IMAGE_NAME(name, adsp, SMEM_IMAGE_TABLE_ADSP_INDEX);
	DEBUGFS_IMAGE_VARIANT(variant, adsp, SMEM_IMAGE_TABLE_ADSP_INDEX);
	DEBUGFS_IMAGE_OEM(oem, adsp, SMEM_IMAGE_TABLE_ADSP_INDEX);

	qcom_socinfo->cnss = debugfs_create_dir("cnss",
						qcom_socinfo->dbg_root);

	DEBUGFS_IMAGE_NAME(name, cnss, SMEM_IMAGE_TABLE_CNSS_INDEX);
	DEBUGFS_IMAGE_VARIANT(variant, cnss, SMEM_IMAGE_TABLE_CNSS_INDEX);
	DEBUGFS_IMAGE_OEM(oem, cnss, SMEM_IMAGE_TABLE_CNSS_INDEX);

	qcom_socinfo->video = debugfs_create_dir("video",
						 qcom_socinfo->dbg_root);

	DEBUGFS_IMAGE_NAME(name, video, SMEM_IMAGE_TABLE_VIDEO_INDEX);
	DEBUGFS_IMAGE_VARIANT(variant, video, SMEM_IMAGE_TABLE_VIDEO_INDEX);
	DEBUGFS_IMAGE_OEM(oem, video, SMEM_IMAGE_TABLE_VIDEO_INDEX);
}

static void socinfo_debugfs_exit(struct qcom_socinfo *qcom_socinfo)
{
	debugfs_remove_recursive(qcom_socinfo->dbg_root);
}
#else
static void socinfo_debugfs_init(struct qcom_socinfo *qcom_socinfo) {  }
static void socinfo_debugfs_exit(struct qcom_socinfo *qcom_socinfo) {  }
#endif /* CONFIG_DEBUG_FS */

static int qcom_socinfo_probe(struct platform_device *pdev)
{
	struct qcom_socinfo *qs;
	struct socinfo *info;
	size_t item_size;

	info = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_HW_SW_BUILD_ID,
			      &item_size);
	if (IS_ERR(info)) {
		dev_err(&pdev->dev, "Couldn't find socinfo\n");
		return -EINVAL;
	}

	qs = devm_kzalloc(&pdev->dev, sizeof(*qs), GFP_KERNEL);
	if (!qs)
		return -ENOMEM;

	qs->attr.family = "Snapdragon";
	qs->attr.machine = socinfo_machine(&pdev->dev,
					   le32_to_cpu(info->id));
	qs->attr.revision = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%u.%u",
					   SOCINFO_MAJOR(le32_to_cpu(info->ver)),
					   SOCINFO_MINOR(le32_to_cpu(info->ver)));
	if (le32_to_cpu(info->fmt) >= 10)
		qs->attr.serial_number = devm_kasprintf(&pdev->dev, GFP_KERNEL,
							"%u",
							le32_to_cpu(info->serial_num));

	qs->soc_dev = soc_device_register(&qs->attr);
	if (IS_ERR(qs->soc_dev))
		return PTR_ERR(qs->soc_dev);

	qs->socinfo = info;

	socinfo_debugfs_init(qs);

	/* Feed the soc specific unique data into entropy pool */
	add_device_randomness(info, item_size);

	platform_set_drvdata(pdev, qs->soc_dev);

	return 0;
}

static int qcom_socinfo_remove(struct platform_device *pdev)
{
	struct qcom_socinfo *qs = platform_get_drvdata(pdev);

	soc_device_unregister(qs->soc_dev);

	socinfo_debugfs_exit(qs);

	return 0;
}

static struct platform_driver qcom_socinfo_driver = {
	.probe = qcom_socinfo_probe,
	.remove = qcom_socinfo_remove,
	.driver  = {
		.name = "qcom-socinfo",
	},
};

module_platform_driver(qcom_socinfo_driver);

MODULE_DESCRIPTION("Qualcomm socinfo driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:qcom-socinfo");
