#!/usr/bin/env bash
set -euo pipefail

readonly DEFAULT_DEVICE=/dev/ttyUSB0
readonly DEFAULT_ALIAS=e1_sbus
readonly RULE_PREFIX=99-e1-sbus

usage() {
  cat <<'EOF'
Usage:
  sudo ./install_sbus_udev.sh install [--device /dev/ttyUSB0] [--alias e1_sbus]
                                      [--match auto|serial|path] [--dry-run]
  ./install_sbus_udev.sh status [--alias e1_sbus]
  sudo ./install_sbus_udev.sh remove [--alias e1_sbus] [--dry-run]

Commands:
  install  Create a persistent udev alias. This is the default command.
  status   Show the installed rule and current alias target.
  remove   Remove the rule created for the alias.

Matching:
  auto     Prefer a unique USB serial number, otherwise bind to the physical USB port.
  serial   Require and use ID_SERIAL_SHORT. The alias follows the receiver between USB ports.
  path     Use ID_PATH. Suitable for devices such as CH340 without a unique serial number;
           the receiver must remain connected to the same physical USB port.

Examples:
  sudo ./install_sbus_udev.sh install --device /dev/ttyUSB2
  sudo ./install_sbus_udev.sh install --device /dev/ttyUSB2 --match path
  ./install_sbus_udev.sh status
  sudo ./install_sbus_udev.sh remove
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

info() {
  echo "$*"
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"
}

escape_udev_value() {
  local value=$1
  value=${value//\\/\\\\}
  value=${value//\"/\\\"}
  printf '%s' "$value"
}

validate_alias() {
  local value=$1
  [[ "$value" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] ||
    die "invalid alias '$value'; use letters, digits, dot, underscore, or dash"
  [[ "$value" != ttyUSB* && "$value" != ttyACM* ]] ||
    die "alias '$value' could be confused with a kernel device name"
}

ACTION=install
DEVICE=$DEFAULT_DEVICE
ALIAS=$DEFAULT_ALIAS
MATCH_MODE=auto
DRY_RUN=false

if (($# > 0)) && [[ "$1" != --* ]]; then
  ACTION=$1
  shift
fi

while (($# > 0)); do
  case "$1" in
    --device)
      (($# >= 2)) || die "--device requires a value"
      DEVICE=$2
      shift 2
      ;;
    --alias)
      (($# >= 2)) || die "--alias requires a value"
      ALIAS=${2#/dev/}
      shift 2
      ;;
    --match)
      (($# >= 2)) || die "--match requires auto, serial, or path"
      MATCH_MODE=$2
      shift 2
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
done

[[ "$ACTION" == install || "$ACTION" == status || "$ACTION" == remove ]] ||
  die "unknown command '$ACTION'"
[[ "$MATCH_MODE" == auto || "$MATCH_MODE" == serial || "$MATCH_MODE" == path ]] ||
  die "invalid match mode '$MATCH_MODE'"
validate_alias "$ALIAS"

RULE_FILE="/etc/udev/rules.d/${RULE_PREFIX}-${ALIAS}.rules"

if [[ "$ACTION" == status ]]; then
  if [[ -f "$RULE_FILE" ]]; then
    info "Installed rule: $RULE_FILE"
    sed -n '1,120p' "$RULE_FILE"
  else
    info "No rule installed at $RULE_FILE"
  fi
  if [[ -L "/dev/$ALIAS" ]]; then
    info "Current alias: /dev/$ALIAS -> $(readlink -f "/dev/$ALIAS")"
  else
    info "Current alias: /dev/$ALIAS is not present"
  fi
  exit 0
fi

if [[ "$ACTION" == remove ]]; then
  if [[ "$DRY_RUN" == true ]]; then
    info "Would remove: $RULE_FILE"
    exit 0
  fi
  ((EUID == 0)) || die "remove must be run as root (use sudo)"
  if [[ -f "$RULE_FILE" ]]; then
    rm -- "$RULE_FILE"
    udevadm control --reload-rules
    udevadm trigger --subsystem-match=tty --action=change
    info "Removed $RULE_FILE"
  else
    info "Rule is not installed: $RULE_FILE"
  fi
  exit 0
fi

require_command udevadm
[[ -e "$DEVICE" ]] || die "device does not exist: $DEVICE"
DEVICE=$(readlink -f "$DEVICE")
[[ -c "$DEVICE" ]] || die "not a character device: $DEVICE"
[[ "$DEVICE" == /dev/ttyUSB* || "$DEVICE" == /dev/ttyACM* ]] ||
  die "unsupported serial device '$DEVICE'; expected /dev/ttyUSB* or /dev/ttyACM*"

VENDOR_ID=
MODEL_ID=
SERIAL_SHORT=
DEVICE_PATH=
while IFS='=' read -r key value; do
  case "$key" in
    ID_VENDOR_ID) VENDOR_ID=$value ;;
    ID_MODEL_ID) MODEL_ID=$value ;;
    ID_SERIAL_SHORT) SERIAL_SHORT=$value ;;
    ID_PATH) DEVICE_PATH=$value ;;
  esac
done < <(udevadm info --query=property --name="$DEVICE")

[[ -n "$VENDOR_ID" ]] || die "udev did not report ID_VENDOR_ID for $DEVICE"
[[ -n "$MODEL_ID" ]] || die "udev did not report ID_MODEL_ID for $DEVICE"

if [[ "$MATCH_MODE" == auto ]]; then
  if [[ -n "$SERIAL_SHORT" && "$SERIAL_SHORT" != 0 && "$SERIAL_SHORT" != 0000 ]]; then
    MATCH_MODE=serial
  else
    MATCH_MODE=path
  fi
fi

case "$MATCH_MODE" in
  serial)
    [[ -n "$SERIAL_SHORT" ]] || die "$DEVICE has no USB serial number; use --match path"
    MATCH_PROPERTY=ID_SERIAL_SHORT
    MATCH_VALUE=$SERIAL_SHORT
    MATCH_DESCRIPTION="USB serial number '$SERIAL_SHORT'"
    ;;
  path)
    [[ -n "$DEVICE_PATH" ]] || die "udev did not report ID_PATH for $DEVICE"
    MATCH_PROPERTY=ID_PATH
    MATCH_VALUE=$DEVICE_PATH
    MATCH_DESCRIPTION="physical USB path '$DEVICE_PATH'"
    ;;
esac

RULE="# Generated by install_sbus_udev.sh for $DEVICE
SUBSYSTEM==\"tty\", ENV{ID_VENDOR_ID}==\"$(escape_udev_value "$VENDOR_ID")\", ENV{ID_MODEL_ID}==\"$(escape_udev_value "$MODEL_ID")\", ENV{$MATCH_PROPERTY}==\"$(escape_udev_value "$MATCH_VALUE")\", SYMLINK+=\"$(escape_udev_value "$ALIAS")\", GROUP=\"dialout\", MODE=\"0660\", TAG+=\"uaccess\""

info "Device:       $DEVICE"
info "USB VID:PID:  $VENDOR_ID:$MODEL_ID"
info "Match:        $MATCH_DESCRIPTION"
info "Alias:        /dev/$ALIAS"
info "Rule file:    $RULE_FILE"

if [[ "$DRY_RUN" == true ]]; then
  info ""
  info "$RULE"
  exit 0
fi

((EUID == 0)) || die "install must be run as root (use sudo), or add --dry-run to preview"

TEMP_RULE=$(mktemp /tmp/e1-sbus-udev.XXXXXX)
trap 'rm -f -- "$TEMP_RULE"' EXIT
printf '%s\n' "$RULE" >"$TEMP_RULE"
install -o root -g root -m 0644 "$TEMP_RULE" "$RULE_FILE"
udevadm control --reload-rules
SYS_PATH=$(udevadm info --query=path --name="$DEVICE")
udevadm trigger --action=add "/sys${SYS_PATH}"
udevadm settle --timeout=5 || true

if [[ -L "/dev/$ALIAS" ]]; then
  info "Installed successfully: /dev/$ALIAS -> $(readlink -f "/dev/$ALIAS")"
else
  info "Rule installed, but /dev/$ALIAS is not present yet. Replug $DEVICE and run:"
  info "  ./install_sbus_udev.sh status --alias $ALIAS"
fi

info "Set this in the SBUS YAML configuration:"
info "  sbus_port: '/dev/$ALIAS'"
