#!/usr/bin/env bash

# Build an arm64 kernel and boot it once on a remote bootc machine. The UKI
# follows redhat/kernel.spec.template's Qualcomm DTB auto-selection rules.
# Build in this source tree, replacing its current in-tree kernel config.

set -euo pipefail

die() {
    echo "sync-arm.sh: $*" >&2
    exit 1
}

if [[ $# -ne 1 || $1 == -h || $1 == --help ]]; then
    echo "Usage: $0 <host>" >&2
    exit 1
fi

HOST=$1
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
STAGE_DIR=$ROOT/.sync-arm/stage
ASSET_DIR=$ROOT/.sync-arm/boot-assets
DEFAULT_EFI_STUB=/usr/lib/systemd/boot/efi/linuxaa64.efi.stub
DEFAULT_STUBBLE_HWIDS_DIR=/usr/share/stubble/hwids
EFI_STUB_OVERRIDE=${EFI_STUB+x}
HWIDS_OVERRIDE=${STUBBLE_HWIDS_DIR+x}
STUBBLE_HWIDS_DIR=${STUBBLE_HWIDS_DIR:-$DEFAULT_STUBBLE_HWIDS_DIR}
EFI_STUB=${EFI_STUB:-$DEFAULT_EFI_STUB}

[[ -f $ROOT/.config-arm ]] || die "missing $ROOT/.config-arm; provide an arm64 kernel config there"
grep -qx 'CONFIG_ARM64=y' "$ROOT/.config-arm" || die ".config-arm is not an arm64 config"

for tool in make rsync ssh scp ukify nproc sed awk find sort install cmp cp grep file; do
    command -v "$tool" >/dev/null || die "required command not found: $tool"
done

# The needed AArch64 payloads are in aarch64 RPMs. Download their signed RPMs
# into the local build cache rather than adding foreign-arch RPMs to the host.
if [[ -z $EFI_STUB_OVERRIDE && ! -f $DEFAULT_EFI_STUB ]]; then
    EFI_STUB=$ASSET_DIR/root/usr/lib/systemd/boot/efi/linuxaa64.efi.stub
fi
if [[ -z $HWIDS_OVERRIDE && ! -d $DEFAULT_STUBBLE_HWIDS_DIR ]]; then
    STUBBLE_HWIDS_DIR=$ASSET_DIR/root/usr/share/stubble/hwids
fi
if [[ -n $EFI_STUB_OVERRIDE && ! -f $EFI_STUB ]]; then
    die "AArch64 systemd EFI stub not found: $EFI_STUB"
fi
if [[ -n $HWIDS_OVERRIDE && ! -d $STUBBLE_HWIDS_DIR ]]; then
    die "Stubble HWIDs not found: $STUBBLE_HWIDS_DIR"
fi
if [[ ! -f $EFI_STUB || ! -d $STUBBLE_HWIDS_DIR ]]; then
    for tool in dnf rpm2cpio cpio rpmkeys; do
        command -v "$tool" >/dev/null || \
            die "missing $tool; set EFI_STUB and STUBBLE_HWIDS_DIR to existing AArch64 assets"
    done
    RPM_DIR=$ASSET_DIR/rpms
    mkdir -p "$RPM_DIR" "$ASSET_DIR/root"
    shopt -s nullglob
    boot_rpms=("$RPM_DIR"/systemd-boot-unsigned-*.aarch64.rpm)
    stubble_rpms=("$RPM_DIR"/stubble-*.aarch64.rpm)
    if (( ${#boot_rpms[@]} == 0 || ${#stubble_rpms[@]} == 0 )); then
        echo "Downloading AArch64 systemd stub and Stubble HWIDs into $ASSET_DIR"
        dnf --forcearch=aarch64 download --arch=aarch64 \
            --destdir="$RPM_DIR" systemd-boot-unsigned stubble || \
            die "could not download AArch64 boot assets; set EFI_STUB and STUBBLE_HWIDS_DIR"
        boot_rpms=("$RPM_DIR"/systemd-boot-unsigned-*.aarch64.rpm)
        stubble_rpms=("$RPM_DIR"/stubble-*.aarch64.rpm)
    fi
    (( ${#boot_rpms[@]} == 1 && ${#stubble_rpms[@]} == 1 )) || \
        die "expected one AArch64 RPM per package in $RPM_DIR"
    for package in "${boot_rpms[0]}" "${stubble_rpms[0]}"; do
        signature=$(rpmkeys --checksig "$package") || die "RPM signature check failed: $package"
        [[ $signature == *'signatures OK'* ]] || die "RPM is not signed: $package"
        (cd "$ASSET_DIR/root" && rpm2cpio "$package" | cpio -idm --quiet) || \
            die "could not extract $package"
    done
fi
[[ -f $EFI_STUB ]] || die "AArch64 systemd EFI stub not found: $EFI_STUB"
STUB_DESC=$(file -b "$EFI_STUB")
[[ $STUB_DESC == *PE32* && $STUB_DESC == *EFI* ]] && \
    grep -Eiq 'aarch64|arm64' <<< "$STUB_DESC" || \
    die "EFI stub is not an AArch64 EFI application: $EFI_STUB"
[[ -d $STUBBLE_HWIDS_DIR ]] || die "Stubble HWIDs not found: $STUBBLE_HWIDS_DIR"
[[ -d $ROOT/redhat/hwids ]] || die "Red Hat HWIDs directory not found"
UKIFY_HELP=$(ukify build --help) || die "ukify build is unavailable"
for option in --efi-arch --stub --hwids --devicetree-auto; do
    [[ $UKIFY_HELP == *"$option"* ]] || die "ukify does not support $option"
done

case $(uname -m) in
    x86_64)
        CROSS_COMPILE=${CROSS_COMPILE:-aarch64-linux-gnu-}
        command -v "${CROSS_COMPILE}gcc" >/dev/null || die "cross compiler not found: ${CROSS_COMPILE}gcc"
        command -v "${CROSS_COMPILE}ld" >/dev/null || die "cross linker not found: ${CROSS_COMPILE}ld"
        ;;
    aarch64|arm64)
        CROSS_COMPILE=${CROSS_COMPILE:-}
        ;;
    *)
        die "unsupported build host architecture: $(uname -m)"
        ;;
esac

if [[ -n ${JOBS:-} ]]; then
    [[ $JOBS =~ ^[1-9][0-9]*$ ]] || die "JOBS must be a positive integer"
else
    JOBS=$(( $(nproc) - 2 ))
    (( JOBS > 0 )) || JOBS=1
fi

MAKE_ARGS=(-C "$ROOT" ARCH=arm64 "CROSS_COMPILE=$CROSS_COMPILE" HOSTCC=gcc HOSTCXX=g++)
if command -v ccache >/dev/null; then
    export CCACHE_DIR=${CCACHE_DIR:-$ROOT/../cache}
    export CCACHE_FILECLONE=1 CCACHE_MAXSIZE=4G
    MAKE_ARGS+=("CC=ccache ${CROSS_COMPILE}gcc" "HOSTCC=ccache gcc" "HOSTCXX=ccache g++")
fi

# Prepare the bootc overlay before building, so the device can reboot while
# the local kernel build runs.
ssh "$HOST" /bin/bash -s <<'REMOTE_OVERLAY'
set -euo pipefail
sudo -n true
if ! sudo -n touch /usr/tst; then
    sudo -n rpm-ostree usroverlay --hotfix
    echo 'Applied the hotfix overlay; rebooting while the kernel builds.' >&2
    sudo -n systemctl reboot --no-block
fi
REMOTE_OVERLAY

mkdir -p "$ROOT/.sync-arm"
if ! cmp -s "$ROOT/.config-arm" "$ROOT/.sync-arm/.input-config" || \
   ! grep -qx 'CONFIG_ARM64=y' "$ROOT/.config" 2>/dev/null; then
    cp "$ROOT/.config-arm" "$ROOT/.config"
    cp "$ROOT/.config-arm" "$ROOT/.sync-arm/.input-config"
fi

# Give a stock config a distinct module directory without modifying the input.
if grep -qx 'CONFIG_LOCALVERSION=""' "$ROOT/.config"; then
    "$ROOT/scripts/config" --file "$ROOT/.config" --set-str LOCALVERSION '-custom-arm'
fi

make -s "${MAKE_ARGS[@]}" olddefconfig
for setting in CONFIG_ARM64 CONFIG_EFI CONFIG_EFI_STUB CONFIG_OF CONFIG_ARCH_QCOM CONFIG_MODULES; do
    grep -qx "$setting=y" "$ROOT/.config" || die "$setting=y is required in the arm64 build config"
done

echo "Building arm64 kernel, modules, and DTBs with $JOBS jobs"
time make -s -j "$JOBS" "${MAKE_ARGS[@]}" all dtbs

KNAME=$(make -s "${MAKE_ARGS[@]}" kernelrelease)
[[ $KNAME =~ ^[A-Za-z0-9._+-]+$ ]] || die "unexpected kernel release: $KNAME"
KIMAGE=$(make -s "${MAKE_ARGS[@]}" image_name)
[[ -f $ROOT/$KIMAGE ]] || die "kernel image not found: $ROOT/$KIMAGE"

rm -rf -- "$STAGE_DIR"
mkdir -p "$STAGE_DIR"
install -m 0644 "$ROOT/$KIMAGE" "$STAGE_DIR/vmlinuz"
make -s -j "$JOBS" "${MAKE_ARGS[@]}" modules_install "INSTALL_MOD_PATH=$STAGE_DIR"
make -s "${MAKE_ARGS[@]}" dtbs_install \
    "INSTALL_DTBS_PATH=$STAGE_DIR/lib/modules/$KNAME/dtb"

# Match the x1*/sc8280x* DTB selection and exclusions in the Red Hat spec.
DTB_DIR=$STAGE_DIR/lib/modules/$KNAME/dtb/qcom
[[ -d $DTB_DIR ]] || die "Qualcomm DTBs were not installed to $DTB_DIR"
DTBS=()
while IFS= read -r -d '' dtb; do
    case ${dtb##*/} in
        *el2*|*devkit*|*crd*|*qcp*|*primus*) continue ;;
    esac
    DTBS+=("$dtb")
done < <(find "$DTB_DIR" -maxdepth 1 -type f \
    \( -name 'x1*.dtb' -o -name 'sc8?80x*.dtb' \) -print0 | sort -z)
(( ${#DTBS[@]} > 0 )) || die "no Qualcomm DTBs matched the Red Hat DT loader rules"
echo "Selected ${#DTBS[@]} DTBs for the UKI"

HWIDS_DIR=$STAGE_DIR/hwids
mkdir -p "$HWIDS_DIR"
cp -a "$STUBBLE_HWIDS_DIR/." "$HWIDS_DIR/"
cp -a "$ROOT/redhat/hwids/." "$HWIDS_DIR/"

SBAT_FILE=$STAGE_DIR/dtbloader.sbat
sed -e "s/@KVER/$KNAME/g" -e 's/@SBAT_SUFFIX/rhel/g' \
    "$ROOT/redhat/dtbloader.sbat.template" > "$SBAT_FILE"

rsync -rv --delete --rsync-path='sudo -n rsync' \
    "$STAGE_DIR/lib/modules/$KNAME/" "$HOST:/lib/modules/$KNAME/"

REMOTE_INITRD=/tmp/cinitramfs-arm.img
ssh "$HOST" /bin/bash -s -- "$KNAME" "$REMOTE_INITRD" <<'REMOTE_DRACUT'
set -euo pipefail
kernel_release=$1
initrd=$2
sudo -n depmod -a "$kernel_release"
sudo -n /usr/bin/dracut --hostonly --kver "$kernel_release" --xz -v \
    --add ostree -f "$initrd" --omit='plymouth dmsquash-live'
sudo -n chmod 644 "$initrd"
REMOTE_DRACUT
scp "$HOST:$REMOTE_INITRD" "$STAGE_DIR/initramfs.img"

CMDLINE=$(ssh "$HOST" /bin/bash -s <<'REMOTE_CMDLINE'
set -euo pipefail
shopt -s nullglob
entries=(/boot/loader/entries/ostree-*.conf)
(( ${#entries[@]} > 0 )) || exit 1
awk '/^options[[:space:]]+/ { sub(/^options[[:space:]]+/, ""); cmdline=$0 }
     END { if (cmdline != "") print cmdline; else exit 1 }' "${entries[@]}"
REMOTE_CMDLINE
) || die "failed to read the remote ostree kernel command line"
[[ -n $CMDLINE ]] || die "remote ostree kernel command line is empty"
echo "Using cmdline: $CMDLINE"

UKIFY_ARGS=(build --efi-arch=aa64 "--stub=$EFI_STUB" \
    "--linux=$STAGE_DIR/vmlinuz" "--initrd=$STAGE_DIR/initramfs.img" \
    "--cmdline=$CMDLINE" "--sbat=$SBAT_FILE" '--os-release=' \
    "--uname=$KNAME" "--hwids=$HWIDS_DIR")
for dtb in "${DTBS[@]}"; do
    UKIFY_ARGS+=("--devicetree-auto=$dtb")
done
UKIFY_ARGS+=("--output=$STAGE_DIR/ukernel-arm.efi")
ukify "${UKIFY_ARGS[@]}"
[[ -s $STAGE_DIR/ukernel-arm.efi ]] || die "ukify did not create the arm64 EFI image"
UKI_DESC=$(file -b "$STAGE_DIR/ukernel-arm.efi")
grep -Eiq 'aarch64|arm64' <<< "$UKI_DESC" || die "generated EFI image is not AArch64"
UKI_INFO=$(ukify inspect "$STAGE_DIR/ukernel-arm.efi")
for section in .linux .initrd .cmdline .hwids .dtbauto; do
    [[ $UKI_INFO == *"$section"* ]] || die "generated EFI image lacks $section"
done

scp "$STAGE_DIR/ukernel-arm.efi" "$HOST:/tmp/ukernel-arm.efi"
ssh "$HOST" /bin/bash -s <<'REMOTE_BOOT'
set -euo pipefail
sudo -n install -m 0644 /tmp/ukernel-arm.efi /boot/efi/EFI/ukernel-arm.efi

find_bootnum() {
    local line
    while IFS= read -r line; do
        if [[ $line == Boot????*ukernel-arm.efi* ]]; then
            printf '%s\n' "${line:4:4}"
            return 0
        fi
    done < <(sudo -n efibootmgr -v)
    return 1
}

if ! bootnum=$(find_bootnum); then
    esp_source=$(findmnt -n -o SOURCE /boot/efi)
    disk_name=$(lsblk -nro PKNAME "$esp_source" | head -n 1)
    part_number=$(lsblk -nro PARTN "$esp_source" | head -n 1)
    [[ -n $disk_name && $part_number =~ ^[0-9]+$ ]] || {
        echo "Could not determine ESP disk and partition from $esp_source" >&2
        exit 1
    }
    sudo -n efibootmgr --create-only --disk "/dev/$disk_name" \
        --part "$part_number" --label 'Custom ARM Kernel' \
        --loader '\EFI\ukernel-arm.efi'
    bootnum=$(find_bootnum)
fi

[[ $bootnum =~ ^[[:xdigit:]]{4}$ ]] || exit 1
echo "Boot entry found: $bootnum"
sudo -n efibootmgr --bootnext "$bootnum"
sudo -n systemctl reboot --no-block
REMOTE_BOOT
