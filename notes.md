Here's a complete description you can hand to another agent:

---

**Build target: Hak5 WiFi Pineapple Pager, firmware 24.10.1**

- **OpenWrt target/subtarget**: `ramips/mt76x8`
- **Architecture**: `mipsel_24kc` (MIPS32, little-endian)
- **ABI**: o32, MIPS32r2, **soft-float** (no FPU — this matters, the dynamic linker is `ld-musl-mipsel-sf.so.1`, the `sf` suffix specifically denotes soft-float)
- **C library**: musl (not glibc)
- **Toolchain**: OpenWrt SDK for `mipsel_24kc`, GCC 14.3.0, matching musl
- **Cross-compile triple**: `mipsel-openwrt-linux-musl`

**Source**: upstream `spectools` (Kismet's RF spectrum analyzer suite, by dragorn/caljorden), built from the `spectool sourcecode/` directory in this repo using autotools.

**Configure/build commands**:
```bash
cd "spectool sourcecode"
./configure --host=mipsel-openwrt-linux-musl --prefix=/usr --disable-gtk --disable-curses
make -j$(nproc)
strip spectool_raw spectool_net
```
GTK and curses UIs were disabled — this is a headless target, only the CLI `spectool_raw` (raw spectrum dump) and `spectool_net` (network server) binaries are needed.

**Dependencies bundled alongside** (since the Pager firmware doesn't ship them): `libusb-0.1.so.4.4.4` (USB compat shim) and `libusb-1.0.so.0.4.0` (modern libusb), both cross-compiled for the same `mipsel_24kc`/musl/soft-float target. Everything else (`libc`/musl, `libgcc_s.so.1`, `libpthread`, `libm`) is assumed present on the Pager already.

**Output verification**: confirmed via `file spectool_raw` → `ELF 32-bit LSB executable, MIPS, MIPS32 rel2 version 1 (SYSV), dynamically linked, interpreter /lib/ld-musl-mipsel-sf.so.1, stripped`. The `ld-musl-mipsel-sf.so.1` interpreter string is the easiest way to confirm a binary was built correctly for this target — if it's missing `-sf` or says `arm`/`x86`, the build used the wrong toolchain/host triple.

The resulting binaries+libs are staged at `spectools-pineapple-build/{bin,lib}/`, which `scripts/package.sh` reads from when assembling the SpecPine payload zip. That build output directory is treated as a checked-in artifact — only regenerate it by re-running the cross-compile above when the upstream `spectool sourcecode/` changes, not by editing it directly.

**human notes** pre-libusb1.0 this will be tailored for the pineapple pager. The goal is to publish to OpenWRT repositories.