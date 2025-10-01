obj-$(CONFIG_AYN) += ayn_platform/
obj-$(CONFIG_AYANEO) += ayaneo_platform/
obj-$(CONFIG_BMI260) += bmi260/

obj-$(CONFIG_XONE) += xone/
obj-$(CONFIG_FANATEC) += fanatecff/
obj-$(CONFIG_LG4FF) += lg4ff/
obj-$(CONFIG_TMFF2) += hid-tmff2/
obj-$(CONFIG_T150) += t150/hid-t150/
obj-$(CONFIG_RAZER) += razer/driver/

obj-$(CONFIG_S76) += system76/
obj-$(CONFIG_S76IO) += system76_io/

# evdi incorrectly picks those variables up
CFLAGS :=
export CFLAGS
obj-$(CONFIG_DRM_EVDI) += evdi/module/
obj-$(CONFIG_V4L2LOOP) += v4l2loopback/

# Install common files from all modules
install:
	install -D modprobe.conf $(INSTALL_DIR)/usr/lib/modprobe.d/20-akmods.conf

	install -D razer/install_files/udev/99-razer.rules $(INSTALL_DIR)/usr/lib/udev/rules.d/70-razer.rules
	install -D -m 0755 razer/install_files/udev/razer_mount $(INSTALL_DIR)/usr/lib/udev/razer_mount

	install -D xone/install/modprobe.conf $(INSTALL_DIR)/usr/lib/modprobe.d/xone.conf

	make -C v4l2loopback/utils/
	install -D -m 0755 v4l2loopback/utils/v4l2loopback-ctl $(INSTALL_DIR)/usr/bin/v4l2loopback-ctl

	install -D t150/files/etc/udev/rules.d/10-t150.rules $(INSTALL_DIR)/usr/lib/udev/rules.d/10-t150.rules