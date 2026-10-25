/* libusb config.h for Linux builds of Xenia.
 *
 * libusb generates this file with autoconf, which the Xenia build doesn't
 * run. The values match a glibc or musl Linux host with the usbfs and netlink
 * backends (see third_party/CMakeLists.txt), without libudev.
 */

#ifndef XENIA_THIRD_PARTY_LIBUSB_CONFIG_LINUX_CONFIG_H_
#define XENIA_THIRD_PARTY_LIBUSB_CONFIG_LINUX_CONFIG_H_

#define DEFAULT_VISIBILITY __attribute__((visibility("default")))
#define ENABLE_LOGGING 1
#define HAVE_ASM_TYPES_H 1
#define HAVE_CLOCK_GETTIME 1
#define HAVE_EVENTFD 1
#define HAVE_NFDS_T 1
#define HAVE_PIPE2 1
#define HAVE_PTHREAD_CONDATTR_SETCLOCK 1
#define HAVE_PTHREAD_SETNAME_NP 1
#define HAVE_STRUCT_TIMESPEC 1
#define HAVE_SYS_TIME_H 1
#define HAVE_TIMERFD 1
#define PLATFORM_POSIX 1
#define PRINTF_FORMAT(a, b) __attribute__((__format__(__printf__, a, b)))
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#endif  // XENIA_THIRD_PARTY_LIBUSB_CONFIG_LINUX_CONFIG_H_
