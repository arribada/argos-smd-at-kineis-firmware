// SPDX-License-Identifier: no SPDX license
/**
 * @file    bl_syscalls.c
 * @brief   Minimal newlib syscall stubs for the bootloader.
 *
 * Sole purpose: provide _read/_write/_close/_lseek so the linker resolves them
 * here instead of pulling newlib-nano's -lnosys versions, which carry a
 * `.gnu.warning.*` section ("_X is not implemented and will always fail") that
 * the linker prints at every build. Behaviour is byte-for-byte identical to
 * those nosys stubs (fail, errno=ENOSYS, return -1) — the bootloader performs no
 * filesystem/stdio I/O — so this removes ONLY the build warning, nothing
 * functional changes. Declared weak so any real implementation elsewhere wins.
 * Signatures mirror Core/Src/syscalls.c (the app provides the same and is
 * therefore warning-free).
 */

#include <errno.h>

__attribute__((weak)) int _read(int file, char *ptr, int len)
{
	(void)file; (void)ptr; (void)len;
	errno = ENOSYS;
	return -1;
}

__attribute__((weak)) int _write(int file, char *ptr, int len)
{
	(void)file; (void)ptr; (void)len;
	errno = ENOSYS;
	return -1;
}

__attribute__((weak)) int _close(int file)
{
	(void)file;
	errno = ENOSYS;
	return -1;
}

__attribute__((weak)) int _lseek(int file, int ptr, int dir)
{
	(void)file; (void)ptr; (void)dir;
	errno = ENOSYS;
	return -1;
}
