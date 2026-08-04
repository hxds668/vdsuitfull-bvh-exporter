#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif


static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x68a2c71a, "_raw_spin_lock_irqsave" },
	{ 0x24e0d9d9, "_raw_spin_unlock_irqrestore" },
	{ 0x52bd083b, "_raw_spin_lock_irq" },
	{ 0x261015c5, "_raw_spin_unlock_irq" },
	{ 0x344a6cc1, "usb_control_msg" },
	{ 0xf9ea1e87, "__dynamic_dev_dbg" },
	{ 0x4829a47e, "memcpy" },
	{ 0x2a180eba, "__tty_alloc_driver" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0xc4ed6017, "tty_register_driver" },
	{ 0x84c90aca, "tty_driver_kref_put" },
	{ 0xa4cab451, "usb_register_driver" },
	{ 0x542a0ce9, "tty_unregister_driver" },
	{ 0x92997ed8, "_printk" },
	{ 0xe139f1f9, "usb_submit_urb" },
	{ 0x713584f, "_dev_err" },
	{ 0xa130a89d, "usb_autopm_put_interface_async" },
	{ 0xc78bdc94, "usb_kill_urb" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0xfea1b088, "_raw_spin_lock" },
	{ 0x7950dbfe, "_raw_spin_unlock" },
	{ 0xb24681ba, "usb_free_coherent" },
	{ 0x5b1db3f9, "mutex_lock" },
	{ 0x1dceff1f, "device_remove_file" },
	{ 0x107491dd, "mutex_unlock" },
	{ 0x89145b0c, "tty_port_tty_get" },
	{ 0xf82a41af, "tty_vhangup" },
	{ 0xf1f256b, "tty_kref_put" },
	{ 0x760022a3, "tty_unregister_device" },
	{ 0x807b41ff, "usb_free_urb" },
	{ 0xe594bd78, "usb_driver_release_interface" },
	{ 0xa6d0800e, "tty_port_put" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x6ebe366f, "ktime_get_mono_fast_ns" },
	{ 0x97ac8737, "tty_port_tty_hangup" },
	{ 0x485e2544, "gpiochip_remove" },
	{ 0x68bc664d, "usb_put_intf" },
	{ 0x37a0cba, "kfree" },
	{ 0x1b74fbee, "tty_port_tty_wakeup" },
	{ 0xf1fb0be1, "tty_port_hangup" },
	{ 0x26b6cf66, "tty_port_close" },
	{ 0x1dcdba6c, "usb_deregister" },
	{ 0x11b8a501, "tty_standard_install" },
	{ 0x296695f, "refcount_warn_saturate" },
	{ 0x2d3385d3, "system_wq" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0xe9c53203, "usb_autopm_get_interface_async" },
	{ 0xdbe44b7c, "tty_flip_buffer_push" },
	{ 0xe3c1b785, "__tty_insert_flip_char" },
	{ 0x897b630b, "tty_insert_flip_string_fixed_flag" },
	{ 0x47d63bee, "kmalloc_caches" },
	{ 0x648f3e7e, "kmalloc_trace" },
	{ 0x3ea1b6e4, "__stack_chk_fail" },
	{ 0x9de66c41, "usb_autopm_get_interface" },
	{ 0xfcf85e4a, "usb_autopm_put_interface" },
	{ 0x75ac1fe, "tty_port_open" },
	{ 0xbd394d8, "tty_termios_baud_rate" },
	{ 0x449ad0a7, "memcmp" },
	{ 0xdcb764ad, "memset" },
	{ 0x6cbbfc54, "__arch_copy_to_user" },
	{ 0x12a4e128, "__arch_copy_from_user" },
	{ 0xc6cbbc89, "capable" },
	{ 0x6b3a817f, "usb_ifnum_to_if" },
	{ 0x725d4b58, "_dev_warn" },
	{ 0xe8858c3f, "__raw_spin_lock_init" },
	{ 0xa9325e33, "__mutex_init" },
	{ 0x46f01255, "tty_port_init" },
	{ 0xff550894, "usb_alloc_coherent" },
	{ 0x772e9872, "usb_alloc_urb" },
	{ 0x651386eb, "device_create_file" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xa6b29cb3, "_dev_info" },
	{ 0x431b9631, "usb_driver_claim_interface" },
	{ 0xec2faccf, "usb_get_intf" },
	{ 0x28b293af, "tty_port_register_device" },
	{ 0xf1405740, "gpiochip_add_data_with_key" },
	{ 0x5b77efca, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v04E2p1410d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1411d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1412d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1414d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1420d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1421d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1422d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1424d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1400d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1401d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1402d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1403d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "12A83EDB83FDB6A376B2D96");
