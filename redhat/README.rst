Anatase Kernel
==============

This repository contains the Anatase kernel, derived
from the Fedora Always Ready Kernel
(`kernel-ark <https://gitlab.com/cki-project/kernel-ark>`__) repository.

This kernel contains optimizations and drivers for handhelds and gaming
hardware. Those patches are applied directly on top of the Fedora patchset
`here <./patch-anatase.patch>`__, after being rebased on top of the ARK
kernel tree in the patchwork
`repo <https://github.com/anatase-org/patchwork>`__.

To make it Github friendly, this repository contains actions and
containers to build the kernel and generate the RPMs in Github. As a
bonus point, each release includes a repackaged version of the kernel
for Arch.

Contributing
------------

If you find that a patch is missing, or you have a patch that you think
should be included, please open an issue with a link to the patch or
the lore.

DO NOT OPEN A PULL REQUEST. The patch files are generated
automatically from the patchwork repository, and any changes to it
will be overwritten.