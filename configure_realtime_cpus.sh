#!/usr/bin/env bash
# Configure persistent CPU isolation and EtherCAT IRQ affinity on the host.

set -Eeuo pipefail

CONTROL_CPU=4
INFER_CPU=6
IRQ_CPU=2
NET_IFACE="enp2s0"
DISABLE_IRQBALANCE=1
UPDATE_APP_CONFIG=1
DRY_RUN=0
REALTIME_GROUP="realtime"
REALTIME_USER="${SUDO_USER:-${USER:-}}"
PAM_LIMITS_FILE="/etc/security/limits.d/99-realtime.conf"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}"

usage() {
  cat <<'EOF'
Usage: sudo ./configure_realtime_cpus.sh [options]

Options:
  --control-cpu N       Control-loop CPU (default: 4)
  --infer-cpu N         ONNX inference CPU (default: 6)
  --irq-cpu N           EtherCAT NIC IRQ CPU (default: 2)
  --interface NAME      EtherCAT network interface (default: enp2s0)
  --realtime-user USER  User allowed to use realtime scheduling (default: sudo caller)
  --keep-irqbalance     Do not disable irqbalance
  --no-app-config       Do not update cpu_affinity in repository YAML files
  --dry-run             Validate and show the resulting configuration only
  -h, --help            Show this help

The script changes host boot parameters and installs a systemd service for
persistent EtherCAT IRQ affinity. It never reboots the machine automatically.
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

is_uint() {
  [[ "$1" =~ ^[0-9]+$ ]]
}

while (($#)); do
  case "$1" in
    --control-cpu)
      (($# >= 2)) || die "$1 requires a value"
      CONTROL_CPU="$2"
      shift 2
      ;;
    --infer-cpu)
      (($# >= 2)) || die "$1 requires a value"
      INFER_CPU="$2"
      shift 2
      ;;
    --irq-cpu)
      (($# >= 2)) || die "$1 requires a value"
      IRQ_CPU="$2"
      shift 2
      ;;
    --interface)
      (($# >= 2)) || die "$1 requires a value"
      NET_IFACE="$2"
      shift 2
      ;;
    --realtime-user)
      (($# >= 2)) || die "$1 requires a value"
      REALTIME_USER="$2"
      shift 2
      ;;
    --keep-irqbalance)
      DISABLE_IRQBALANCE=0
      shift
      ;;
    --no-app-config)
      UPDATE_APP_CONFIG=0
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

for cpu in "$CONTROL_CPU" "$INFER_CPU" "$IRQ_CPU"; do
  is_uint "$cpu" || die "CPU identifiers must be non-negative integers: $cpu"
  [[ -d "/sys/devices/system/cpu/cpu${cpu}" ]] || die "CPU ${cpu} does not exist"
  if [[ -f "/sys/devices/system/cpu/cpu${cpu}/online" ]]; then
    [[ "$(<"/sys/devices/system/cpu/cpu${cpu}/online")" == "1" ]] || die "CPU ${cpu} is offline"
  fi
done

[[ "$CONTROL_CPU" != "$INFER_CPU" ]] || die "control and inference CPUs must differ"
[[ "$CONTROL_CPU" != "$IRQ_CPU" ]] || die "IRQ CPU must not be the control CPU"
[[ "$INFER_CPU" != "$IRQ_CPU" ]] || die "IRQ CPU must not be the inference CPU"
[[ "$CONTROL_CPU" != "0" && "$INFER_CPU" != "0" ]] || die "CPU 0 must remain a housekeeping CPU"
[[ "$NET_IFACE" =~ ^[a-zA-Z0-9_.:-]+$ ]] || die "invalid interface name: $NET_IFACE"
[[ -d "/sys/class/net/${NET_IFACE}" ]] || die "network interface ${NET_IFACE} does not exist"
[[ "$REALTIME_USER" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || die "invalid realtime user: ${REALTIME_USER:-<empty>}"
[[ "$REALTIME_USER" != "root" ]] || die "refusing to configure root as the realtime application user; use --realtime-user USER"
getent passwd "$REALTIME_USER" >/dev/null || die "realtime user does not exist: $REALTIME_USER"

CONTROL_SIBLINGS="$(<"/sys/devices/system/cpu/cpu${CONTROL_CPU}/topology/thread_siblings_list")"
INFER_SIBLINGS="$(<"/sys/devices/system/cpu/cpu${INFER_CPU}/topology/thread_siblings_list")"
if printf '%s\n' "$CONTROL_SIBLINGS" | tr ',' '\n' | awk -F- -v cpu="$INFER_CPU" '
  NF == 1 && $1 == cpu { found=1 }
  NF == 2 && $1 <= cpu && cpu <= $2 { found=1 }
  END { exit !found }
'; then
  die "control CPU ${CONTROL_CPU} and inference CPU ${INFER_CPU} are SMT siblings (${CONTROL_SIBLINGS})"
fi

expand_cpu_list() {
  local item first last cpu
  local -a items
  IFS=',' read -r -a items <<<"$1"
  for item in "${items[@]}"; do
    if [[ "$item" == *-* ]]; then
      first="${item%-*}"
      last="${item#*-}"
      for ((cpu = first; cpu <= last; ++cpu)); do
        printf '%s\n' "$cpu"
      done
    else
      printf '%s\n' "$item"
    fi
  done
}

mapfile -t ONLINE_CPUS < <(
  for path in /sys/devices/system/cpu/cpu[0-9]*; do
    cpu="${path##*cpu}"
    if [[ ! -f "${path}/online" || "$(<"${path}/online")" == "1" ]]; then
      printf '%s\n' "$cpu"
    fi
  done | sort -n
)

declare -A ISOLATED_CPUS=()
while IFS= read -r cpu; do
  [[ -n "$cpu" ]] && ISOLATED_CPUS["$cpu"]=1
done < <(expand_cpu_list "${CONTROL_SIBLINGS},${INFER_SIBLINGS}")

[[ -z "${ISOLATED_CPUS[0]+x}" ]] || die "selected physical core includes CPU 0; CPU 0 must remain a housekeeping CPU"
[[ -z "${ISOLATED_CPUS[$IRQ_CPU]+x}" ]] || die "IRQ CPU ${IRQ_CPU} is an SMT sibling of a selected realtime CPU"

mapfile -t ISOLATED_CPUS_SORTED < <(printf '%s\n' "${!ISOLATED_CPUS[@]}" | sort -n)
ISOLATED_LIST="$(IFS=,; echo "${ISOLATED_CPUS_SORTED[*]}")"

HOUSEKEEPING_LIST=""
for cpu in "${ONLINE_CPUS[@]}"; do
  if [[ -z "${ISOLATED_CPUS[$cpu]+x}" ]]; then
    HOUSEKEEPING_LIST+="${HOUSEKEEPING_LIST:+,}${cpu}"
  fi
done
[[ -n "$HOUSEKEEPING_LIST" ]] || die "no housekeeping CPU remains"

GRUB_FRAGMENT="/etc/default/grub.d/99-legged-realtime.cfg"
IRQ_HELPER="/usr/local/sbin/legged-ethercat-irq-affinity"
IRQ_SERVICE="/etc/systemd/system/legged-ethercat-irq-affinity.service"
GOVERNOR_HELPER="/usr/local/sbin/legged-cpu-performance"
GOVERNOR_SERVICE="/etc/systemd/system/legged-cpu-performance.service"

echo "Realtime CPU configuration"
echo "  control CPU:      ${CONTROL_CPU}"
echo "  inference CPU:    ${INFER_CPU}"
echo "  EtherCAT IRQ CPU: ${IRQ_CPU}"
echo "  EtherCAT NIC:     ${NET_IFACE}"
echo "  isolated CPUs:    ${ISOLATED_LIST} (includes SMT siblings)"
echo "  housekeeping:     ${HOUSEKEEPING_LIST}"
echo "  CPU governor:     performance (persistent)"
echo "  realtime user:    ${REALTIME_USER} (group ${REALTIME_GROUP})"
echo "  SMT siblings:     control=${CONTROL_SIBLINGS}, infer=${INFER_SIBLINGS}"

if ((DRY_RUN)); then
  echo "Dry run: no files or services were changed."
  exit 0
fi

[[ "$EUID" -eq 0 ]] || die "run this script as root (sudo)"
command -v systemctl >/dev/null || die "systemd is required"
command -v update-grub >/dev/null || die "update-grub is required (this script currently targets GRUB-based Ubuntu hosts)"

groupadd -f "$REALTIME_GROUP"
usermod -aG "$REALTIME_GROUP" "$REALTIME_USER"
PAM_LIMITS_TMP="$(mktemp)"
cat >"$PAM_LIMITS_TMP" <<EOF
# Generated by configure_realtime_cpus.sh. Local edits will be overwritten.
@${REALTIME_GROUP} soft rtprio 99
@${REALTIME_GROUP} hard rtprio 99
@${REALTIME_GROUP} soft memlock unlimited
@${REALTIME_GROUP} hard memlock unlimited
EOF
install -m 0644 "$PAM_LIMITS_TMP" "$PAM_LIMITS_FILE"
rm -f -- "$PAM_LIMITS_TMP"

install -d -m 0755 /etc/default/grub.d
GRUB_TMP="$(mktemp)"
cat >"$GRUB_TMP" <<EOF
# Generated by configure_realtime_cpus.sh. Local edits will be overwritten.
# Remove stale copies of these options before appending the selected CPUs.
LEGGED_GRUB_CMDLINE=\$(printf '%s\n' "\${GRUB_CMDLINE_LINUX_DEFAULT:-}" | sed -E 's/(^|[[:space:]])(isolcpus|nohz_full|rcu_nocbs|irqaffinity)=[^[:space:]]+//g; s/[[:space:]]+/ /g; s/^ //; s/ \$//')
GRUB_CMDLINE_LINUX_DEFAULT="\${LEGGED_GRUB_CMDLINE:+\${LEGGED_GRUB_CMDLINE} }isolcpus=domain,managed_irq,${ISOLATED_LIST} nohz_full=${ISOLATED_LIST} rcu_nocbs=${ISOLATED_LIST} irqaffinity=${HOUSEKEEPING_LIST}"
unset LEGGED_GRUB_CMDLINE
EOF
install -m 0644 "$GRUB_TMP" "$GRUB_FRAGMENT"
rm -f -- "$GRUB_TMP"

IRQ_TMP="$(mktemp)"
cat >"$IRQ_TMP" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail

NET_IFACE='${NET_IFACE}'
IRQ_CPU='${IRQ_CPU}'

[[ -d "/sys/class/net/\${NET_IFACE}" ]] || {
  echo "EtherCAT interface \${NET_IFACE} is unavailable" >&2
  exit 1
}

declare -A irqs=()
for irq_path in /sys/class/net/"\${NET_IFACE}"/device/msi_irqs/[0-9]*; do
  [[ -e "\$irq_path" ]] && irqs["\${irq_path##*/}"]=1
done

if [[ -r "/sys/class/net/\${NET_IFACE}/device/irq" ]]; then
  legacy_irq=\$(<"/sys/class/net/\${NET_IFACE}/device/irq")
  [[ "\$legacy_irq" =~ ^[0-9]+$ ]] && ((legacy_irq > 0)) && irqs["\$legacy_irq"]=1
fi

while IFS= read -r irq; do
  [[ -n "\$irq" ]] && irqs["\$irq"]=1
done < <(awk -F: -v iface="\$NET_IFACE" 'index(\$0, iface) { gsub(/[[:space:]]/, "", \$1); if (\$1 ~ /^[0-9]+\$/) print \$1 }' /proc/interrupts)

if ((\${#irqs[@]} == 0)); then
  echo "No IRQ found for \${NET_IFACE}" >&2
  exit 1
fi

failed=0
for irq in \$(printf '%s\n' "\${!irqs[@]}" | sort -n); do
  affinity="/proc/irq/\${irq}/smp_affinity_list"
  if [[ ! -w "\$affinity" ]]; then
    echo "IRQ \${irq}: affinity is not writable" >&2
    failed=1
    continue
  fi
  if ! printf '%s\n' "\$IRQ_CPU" >"\$affinity"; then
    echo "IRQ \${irq}: failed to set CPU \${IRQ_CPU}" >&2
    failed=1
    continue
  fi
  effective=\$(<"/proc/irq/\${irq}/effective_affinity_list" 2>/dev/null || true)
  echo "IRQ \${irq} (\${NET_IFACE}) -> CPU \${IRQ_CPU}; effective=\${effective:-unknown}"
done
exit "\$failed"
EOF
install -m 0755 "$IRQ_TMP" "$IRQ_HELPER"
rm -f -- "$IRQ_TMP"

GOVERNOR_TMP="$(mktemp)"
cat >"$GOVERNOR_TMP" <<'EOF'
#!/usr/bin/env bash
set -Eeuo pipefail

set_with_sysfs() {
  local governor available
  local found=0
  for governor in /sys/devices/system/cpu/cpufreq/policy*/scaling_governor; do
    [[ -e "$governor" ]] || continue
    found=1
    available="${governor%/*}/scaling_available_governors"
    if [[ -r "$available" ]] && ! grep -qw performance "$available"; then
      echo "performance governor is unavailable for ${governor%/*}" >&2
      return 1
    fi
    printf '%s\n' performance >"$governor"
  done
  ((found)) || {
    echo "no CPU frequency policy found under /sys/devices/system/cpu/cpufreq" >&2
    return 1
  }
}

if command -v cpupower >/dev/null 2>&1; then
  if ! cpupower frequency-set -g performance; then
    echo "cpupower failed; falling back to scaling_governor sysfs" >&2
    set_with_sysfs
  fi
else
  echo "cpupower is unavailable; using scaling_governor sysfs" >&2
  set_with_sysfs
fi

failed=0
found=0
for governor in /sys/devices/system/cpu/cpufreq/policy*/scaling_governor; do
  [[ -e "$governor" ]] || continue
  found=1
  current="$(<"$governor")"
  echo "${governor%/*}: ${current}"
  if [[ "$current" != "performance" ]]; then
    failed=1
  fi
done
((found)) || failed=1
exit "$failed"
EOF
install -m 0755 "$GOVERNOR_TMP" "$GOVERNOR_HELPER"
rm -f -- "$GOVERNOR_TMP"

SERVICE_TMP="$(mktemp)"
cat >"$SERVICE_TMP" <<EOF
[Unit]
Description=Pin ${NET_IFACE} EtherCAT IRQs to CPU ${IRQ_CPU}
After=systemd-udev-settle.service irqbalance.service
Wants=systemd-udev-settle.service

[Service]
Type=oneshot
ExecStart=${IRQ_HELPER}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
install -m 0644 "$SERVICE_TMP" "$IRQ_SERVICE"
rm -f -- "$SERVICE_TMP"

GOVERNOR_SERVICE_TMP="$(mktemp)"
cat >"$GOVERNOR_SERVICE_TMP" <<EOF
[Unit]
Description=Set all CPU frequency policies to performance
After=systemd-modules-load.service

[Service]
Type=oneshot
ExecStart=${GOVERNOR_HELPER}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
install -m 0644 "$GOVERNOR_SERVICE_TMP" "$GOVERNOR_SERVICE"
rm -f -- "$GOVERNOR_SERVICE_TMP"

if ((UPDATE_APP_CONFIG)); then
  HW_CONFIG="${REPO_ROOT}/src/legged_rl/legged_robot/e2_hw/legged_e2_hw/config/e2.yaml"
  [[ -f "$HW_CONFIG" ]] || die "hardware config not found: $HW_CONFIG"
  sed -E -i "s/^([[:space:]]*cpu_affinity:[[:space:]]*)-?[0-9]+/\\1${CONTROL_CPU}/" "$HW_CONFIG"
  sed -E -i "s/^([[:space:]]*require_realtime:[[:space:]]*)(true|false)/\\1true/" "$HW_CONFIG"

  shopt -s nullglob
  AC_CONFIGS=("${REPO_ROOT}"/src/legged_rl/rl_controller/rl_controllers/config/e1*_ac.yaml)
  ((${#AC_CONFIGS[@]} > 0)) || die "no AC controller configs found"
  for config in "${AC_CONFIGS[@]}"; do
    sed -E -i "s/^([[:space:]]*inference_cpu_affinity:[[:space:]]*)-?[0-9]+/\\1${INFER_CPU}/" "$config"
    sed -E -i "s/^([[:space:]]*require_realtime:[[:space:]]*)(true|false)/\\1true/" "$config"
  done
fi

if ((DISABLE_IRQBALANCE)); then
  if systemctl cat irqbalance.service >/dev/null 2>&1; then
    systemctl disable --now irqbalance.service || die "failed to disable irqbalance"
    echo "irqbalance disabled so it cannot overwrite EtherCAT IRQ affinity."
  fi
else
  echo "WARNING: irqbalance remains enabled and may overwrite the EtherCAT IRQ affinity."
fi

systemctl daemon-reload
systemctl enable legged-ethercat-irq-affinity.service
systemctl enable legged-cpu-performance.service
"$GOVERNOR_HELPER"
"$IRQ_HELPER"
update-grub

echo
echo "Configuration installed successfully. Reboot is required for CPU isolation."
echo "After reboot, verify with:"
echo "  id ${REALTIME_USER}"
echo "  ulimit -r    # expected: 99 (run from a fresh ${REALTIME_USER} login)"
echo "  ulimit -l    # expected: unlimited"
echo "  cat /proc/cmdline"
echo "  cat /sys/devices/system/cpu/isolated"
echo "  cat /sys/devices/system/cpu/nohz_full"
echo "  systemctl status legged-cpu-performance.service"
echo "  cat /sys/devices/system/cpu/cpufreq/policy*/scaling_governor"
echo "  systemctl status legged-ethercat-irq-affinity.service"
echo "  grep -E '${NET_IFACE}|CPU' /proc/interrupts"
