#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/aee.h>
#include <linux/xlog.h>
/* #include <linux/kernel.h> */
#include <mach/mt_gcpu.h>
#include <mach/mt_clkmgr.h>

#if defined(CONFIG_MTK_IN_HOUSE_TEE_SUPPORT)
#include "trustzone/kree/system.h"
#include "trustzone/tz_cross/ta_gcpu.h"
#define GCPU_TEE_ENABLE 1
#else
#define GCPU_TEE_ENABLE 0
#endif

#define GCPU_DEV_NAME "MTK_GCPU"
#define GCPU_TAG "GCPU Kernel"

#define GCPU_LOG_ERR(log, args...) \
	pr_err("[GCPU Kernel] [%s] [%d] *** ERROR: "log, __func__, __LINE__, ##args)
#define GCPU_LOG_INFO(log, args...) \
	pr_info("[GCPU Kernel] [%s] [%d] "log, __func__, __LINE__, ##args)

int gcpu_enableClk(void)
{
	int ret = 0;

	GCPU_LOG_INFO("Enable GCPU clock\n");

	ret = enable_clock(MT_CG_PERI_GCPU, "GCPU");

	return ret;
}
EXPORT_SYMBOL(gcpu_enableClk);

int gcpu_disableClk(void)
{
	int ret = 0;

	GCPU_LOG_INFO("Disable GCPU clock\n");

	ret = disable_clock(MT_CG_PERI_GCPU, "GCPU");

	return ret;
}
EXPORT_SYMBOL(gcpu_disableClk);

#if GCPU_TEE_ENABLE
static int gcpu_tee_call(uint32_t cmd)
{
	TZ_RESULT l_ret = TZ_RESULT_SUCCESS;
	int ret = 0;
	KREE_SESSION_HANDLE test_session;
	/* MTEEC_PARAM param[4]; */
	struct timespec start, end;
	long long ns;

	l_ret = KREE_CreateSession(TZ_TA_GCPU_UUID, &test_session);
	if (l_ret != TZ_RESULT_SUCCESS) {
		GCPU_LOG_ERR("KREE_CreateSession error, ret = %x\n", l_ret);
		return 1;
	}

	getnstimeofday(&start);
	l_ret = KREE_TeeServiceCall(test_session, cmd, 0, NULL);
	if (l_ret != TZ_RESULT_SUCCESS) {
		GCPU_LOG_ERR("KREE_TeeServiceCall error, ret = %x\n", l_ret);
		ret = 1;
	}
	getnstimeofday(&end);
	ns = ((long long)end.tv_sec - start.tv_sec) * 1000000000 + (end.tv_nsec - start.tv_nsec);
	GCPU_LOG_INFO("gcpu_tee_call, cmd: %d, time: %lld ns\n", cmd, ns);

	l_ret = KREE_CloseSession(test_session);
	if (l_ret != TZ_RESULT_SUCCESS) {
		GCPU_LOG_ERR("KREE_CloseSession error, ret = %x\n", l_ret);
		ret = 1;
	}

	return ret;
}

static int gcpu_probe(struct platform_device *pdev)
{
	/* GCPU_LOG_INFO("gcpu_probe\n"); */
	/* gcpu_tee_call(TZCMD_GCPU_SELFTEST); */
	return 0;
}

static int gcpu_remove(struct platform_device *pdev)
{
	/* GCPU_LOG_INFO("gcpu_remove\n"); */
	return 0;
}

static int gcpu_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	/* GCPU_LOG_INFO("gcpu_suspend\n"); */
	if (gcpu_tee_call(TZCMD_GCPU_SUSPEND)) {
		GCPU_LOG_ERR("Suspend fail\n");
		return 1;
	} else {
		GCPU_LOG_INFO("Suspend ok\n");
		return 0;
	}
}

static int gcpu_resume(struct platform_device *pdev)
{
	/* GCPU_LOG_INFO("gcpu_resume\n"); */
	/* gcpu_tee_call(TZCMD_GCPU_SELFTEST); */
	return 0;
}

struct platform_device gcpu_device = {
	.name = GCPU_DEV_NAME,
	.id = -1,
};

static struct platform_driver gcpu_driver = {
	.probe = gcpu_probe,
	.remove = gcpu_remove,
	.suspend = gcpu_suspend,
	.resume = gcpu_resume,
	.driver = {
		   .name = GCPU_DEV_NAME,
		   .owner = THIS_MODULE,
		   }
};
#endif

static int __init gcpu_init(void)
{
	int ret = 0;

	GCPU_LOG_INFO("module init\n");

	/*Workaround For MT8127 ,Before Disable clock, must call enable clock before*/
	ret = enable_clock(MT_CG_PERI_GCPU, "GCPU");
	if (ret) {
		GCPU_LOG_ERR("Unable to enable gcpu clock, ret = %d\n", ret);
		return ret;
	}
	
	ret = disable_clock(MT_CG_PERI_GCPU, "GCPU");
	if (ret) {
		GCPU_LOG_ERR("Unable to disable gcpu clock, ret = %d\n", ret);
		return ret;
	}
	/*clkmux_sel(MT_CLKMUX_GCPU_SEL, MT_CG_V_SYSPLL_D4, "GCPU");*/

#if GCPU_TEE_ENABLE
	ret = platform_device_register(&gcpu_device);
	if (ret) {
		GCPU_LOG_ERR("Unable to register device , ret = %d\n", ret);
		return ret;
	}

	ret = platform_driver_register(&gcpu_driver);
	if (ret) {
		GCPU_LOG_ERR("Unable to register driver, ret = %d\n", ret);
		return ret;
	}
	gcpu_tee_call(TZCMD_GCPU_KERNEL_INIT_DONE);
#endif
	return 0;
}

static void __exit gcpu_exit(void)
{
	GCPU_LOG_INFO("module exit\n");
}
module_init(gcpu_init);
module_exit(gcpu_exit);

MODULE_DESCRIPTION("MTK GCPU driver");
MODULE_AUTHOR("Rick Chang <rick.chang@mediatek.com>");
MODULE_LICENSE("GPL");
