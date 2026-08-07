.. SPDX-License-Identifier: GPL-2.0-or-later

========================================================
Lenovo WMI Interface Lighting Driver (lenovo-wmi-lighting)
========================================================

Introduction
============
The Lenovo Lighting WMI interface provides control of power and standby
indicator LEDs. The interface uses WMI GUID
``8C5B9127-ECD4-4657-980F-851019F99CA5``.

The driver exposes the indicators through the standard LED class interface.
Firmware with independent runtime and suspend controls provides
``platform::power`` and ``platform::standby`` respectively.

Some firmware provides a single combined control for the physical power
button light. On those systems only ``platform::power`` is registered and it
controls the light during both runtime and suspend. This includes firmware
which also advertises the independent controls but aliases all three controls
to the same physical light.

Firmware interface
==================
The firmware supports either a combined lighting ID, separate power and
standby IDs, or all three IDs:

================  ============  ============================================
Lighting ID       LED name      Behavior
================  ============  ============================================
``0x03``          power         Combined runtime and suspend control
``0x04``          power         Runtime control
``0x24``          standby       Suspend control
================  ============  ============================================

The driver detects the implemented form with read-only status queries. When
the combined ID is implemented it takes precedence, since devices such as the
Legion Go S implement all three IDs using one EC setting. Otherwise both
independent IDs must be implemented before the LEDs are registered.

WMI methods
===========
Method 1 reads the current status for a lighting ID and method 2 writes a
three-byte lighting ID, state and brightness tuple.
