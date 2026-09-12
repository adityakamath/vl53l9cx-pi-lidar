// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal copy of the kernel's dt-bindings/gpio/gpio.h so that overlays in
 * this tree can be compiled standalone with dtc (the RPi5 userspace does not
 * ship the full dt-bindings headers). When building inside a kernel or
 * firmware tree, the real header takes precedence via the include path.
 */
#ifndef _DT_BINDINGS_GPIO_GPIO_H
#define _DT_BINDINGS_GPIO_GPIO_H

/* GPIO active state flags for DT gpio specifiers */
#define GPIO_ACTIVE_HIGH	0
#define GPIO_ACTIVE_LOW		1

#endif /* _DT_BINDINGS_GPIO_GPIO_H */
