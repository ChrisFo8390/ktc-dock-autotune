#!/bin/bash
# KTC Dock Autotune installer
#
# Links klipper/extras/dock_autotune.py into Klipper's extras directory, installs the
# configuration and the macro for the Mainsail macro list, adds both includes to printer.cfg, adds an update manager entry
# to moonraker.conf and restarts Klipper (and Moonraker, which only reads its config at start).
# Backups of printer.cfg and moonraker.conf are made before they are changed.
#
# Usage (from a clone of the repository on the printer):
#   git clone https://github.com/ChrisFo8390/ktc-dock-autotune.git ~/ktc-dock-autotune
#   ~/ktc-dock-autotune/install.sh [options]
#
# Options:
#   --no-include           do not touch printer.cfg
#   --no-macro             do not install the DOCK_AUTOTUNE macro (the command works, but Mainsail does not list it)
#   --no-update-manager    do not touch moonraker.conf
#   --no-restart           do not restart Klipper / Moonraker
#   --uninstall            remove the link and the include lines again (the rest is kept)
#   -h, --help             this text
#
# Other locations: KLIPPER_PATH=... CONFIG_PATH=... ./install.sh
# Update later: git pull in the clone (or the update manager), then restart Klipper.

REPO_URL="https://github.com/ChrisFo8390/ktc-dock-autotune.git"
REPO_NAME="ktc-dock-autotune"
KLIPPER_PATH="${KLIPPER_PATH:-${HOME}/klipper}"
CONFIG_PATH="${CONFIG_PATH:-${HOME}/printer_data/config}"

set -eu
export LC_ALL=C

MODE="install"
DO_INCLUDE="yes"
DO_MACRO="yes"
DO_UPDATE_MANAGER="yes"
DO_RESTART="yes"
RESTART_MOONRAKER="no"

for arg in "$@"; do
    case "${arg}" in
        --no-include)        DO_INCLUDE="no" ;;
        --no-macro)          DO_MACRO="no" ;;
        --no-update-manager) DO_UPDATE_MANAGER="no" ;;
        --no-restart)        DO_RESTART="no" ;;
        --include|--update-manager|--restart) ;;    # the default since this version, kept for old command lines
        --uninstall)         MODE="uninstall" ;;
        -h|--help)
            sed -n '2,22p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown option: ${arg} (see --help)"
            exit 1
            ;;
    esac
done

# Install from the directory this script lives in when it is a clone of the repository,
# otherwise clone it to ~/ktc-dock-autotune
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${SCRIPT_DIR}/klipper/extras/dock_autotune.py" ]; then
    INSTALL_PATH="${SCRIPT_DIR}"
else
    INSTALL_PATH="${INSTALL_PATH:-${HOME}/${REPO_NAME}}"
fi
EXTRAS="${KLIPPER_PATH}/klippy/extras"
CONFIG_FILES="dock_autotune.cfg"
if [ "${DO_MACRO}" = "yes" ]; then
    CONFIG_FILES="${CONFIG_FILES} dock_autotune_macro.cfg"
fi

function preflight_checks {
    if [ "$EUID" -eq 0 ]; then
        echo "[ERROR] This script must not be run as root!"
        exit 1
    fi
    if [ ! -d "${EXTRAS}" ]; then
        echo "[ERROR] Klipper not found at ${KLIPPER_PATH} (set KLIPPER_PATH=...)"
        exit 1
    fi
    if [ ! -e "${EXTRAS}/toolchanger.py" ]; then
        echo "[WARNING] ${EXTRAS}/toolchanger.py not found: is klipper-toolchanger (KTC) installed?"
    fi
}

function backup_file {
    local backup="$1.bak.$(date +%Y%m%d%H%M%S)"
    cp "$1" "${backup}"
    echo "[INFO] Backup: ${backup}"
}

# True if the file has Windows line endings (tr instead of grep: grep ignores the CR in some environments)
function has_crlf {
    [ "$(tr -dc '\r' < "$1" | wc -c)" -gt 0 ]
}

function check_download {
    if [ ! -d "${INSTALL_PATH}" ]; then
        echo "[DOWNLOAD] Downloading repository from ${REPO_URL}..."
        if git -C "$(dirname "${INSTALL_PATH}")" clone "${REPO_URL}" "$(basename "${INSTALL_PATH}")"; then
            chmod +x "${INSTALL_PATH}/install.sh"
            printf "[DOWNLOAD] Download complete!\n\n"
        else
            echo "[ERROR] Download of the git repository failed!"
            exit 1
        fi
    else
        printf "[DOWNLOAD] Using repository at %s\n\n" "${INSTALL_PATH}"
    fi
}

function link_module {
    local target="${EXTRAS}/dock_autotune.py"
    # A real file (copied by hand earlier) is kept as a backup, a link is simply replaced
    if [ -e "${target}" ] && [ ! -L "${target}" ]; then
        local backup="${target}.bak.$(date +%Y%m%d%H%M%S)"
        mv "${target}" "${backup}"
        echo "[WARNING] Existing ${target} moved to ${backup}"
    fi
    ln -sfn "${INSTALL_PATH}/klipper/extras/dock_autotune.py" "${target}"
    echo "[INSTALL] Linked dock_autotune.py into ${EXTRAS}"
}

function install_config {
    if [ ! -d "${CONFIG_PATH}" ]; then
        echo "[WARNING] ${CONFIG_PATH} not found, configuration not installed (set CONFIG_PATH=...)"
        DO_INCLUDE="no"
        DO_UPDATE_MANAGER="no"
        return
    fi
    local name
    for name in ${CONFIG_FILES}; do
        if [ ! -e "${CONFIG_PATH}/${name}" ]; then
            cp "${INSTALL_PATH}/config/${name}" "${CONFIG_PATH}/${name}"
            echo "[INSTALL] Installed ${CONFIG_PATH}/${name}"
        else
            echo "[INFO] ${CONFIG_PATH}/${name} exists and is not overwritten"
            echo "[INFO] Compare it with ${INSTALL_PATH}/config/${name} for new settings"
        fi
    done
}

# Regular expression for the line [include <file>] (a CR at the end of the line is matched by [[:space:]])
function include_pattern {
    printf '^\[include[[:space:]]+%s\][[:space:]]*$' "${1//./\.}"
}

# Adds [include <file>] to printer.cfg: above the SAVE_CONFIG block if there is one (everything below it
# belongs to Klipper), otherwise at the end. Keeps the line endings of the file.
function add_include {
    local name="$1" cfg="${CONFIG_PATH}/printer.cfg" line="[include $1]"
    if [ "${DO_INCLUDE}" != "yes" ]; then
        echo "[INFO] printer.cfg not changed, add this line yourself:  ${line}"
        return
    fi
    if [ ! -f "${cfg}" ]; then
        echo "[WARNING] ${cfg} not found, add this line yourself:  ${line}"
        return
    fi
    if grep -qE "$(include_pattern "${name}")" "${cfg}"; then
        echo "[INFO] ${line} is already in printer.cfg"
        return
    fi
    backup_file "${cfg}"
    local cr="" marker tmp
    if has_crlf "${cfg}"; then
        cr=$'\r'
    fi
    marker="$(grep -nE '^#\*# <-+ SAVE_CONFIG -+>' "${cfg}" | head -n 1 | cut -d: -f1 || true)"
    tmp="$(mktemp)"
    if [ -n "${marker}" ]; then
        awk -v n="${marker}" -v line="${line}" -v cr="${cr}" \
            'NR == n { printf "%s%s\n%s\n", line, cr, cr } { print }' "${cfg}" > "${tmp}"
    else
        { cat "${cfg}"
          if [ -n "$(tail -c 1 "${cfg}")" ]; then printf '%s\n' "${cr}"; fi
          printf '%s\n%s%s\n' "${cr}" "${line}" "${cr}"
        } > "${tmp}"
    fi
    cat "${tmp}" > "${cfg}"      # not mv: keeps owner and permissions of printer.cfg
    rm -f "${tmp}"
    echo "[INSTALL] Added ${line} to ${cfg}"
}

function print_update_manager_section {
    cat <<EOF
[update_manager ${REPO_NAME}]
type: git_repo
path: ${INSTALL_PATH}
origin: ${REPO_URL}
primary_branch: main
is_system_service: False
managed_services: klipper
EOF
}

function add_update_manager {
    local conf="${CONFIG_PATH}/moonraker.conf"
    if [ "${DO_UPDATE_MANAGER}" != "yes" ]; then
        echo "[INFO] moonraker.conf not changed"
        return
    fi
    if [ ! -f "${conf}" ]; then
        echo "[WARNING] ${conf} not found, add this section yourself:"
        echo ""
        print_update_manager_section
        return
    fi
    if grep -qF "[update_manager ${REPO_NAME}]" "${conf}"; then
        echo "[INFO] [update_manager ${REPO_NAME}] is already in ${conf} - not changed"
        return
    fi
    backup_file "${conf}"
    local cr=""
    if has_crlf "${conf}"; then
        cr=$'\r'
    fi
    {
        if [ -n "$(tail -c 1 "${conf}")" ]; then printf '%s\n' "${cr}"; fi
        printf '%s\n' "${cr}"
        print_update_manager_section | sed "s/\$/${cr}/"
    } >> "${conf}"
    RESTART_MOONRAKER="yes"     # Moonraker reads moonraker.conf only when it starts
    echo "[INSTALL] Added [update_manager ${REPO_NAME}] to ${conf}"
}

function restart_services {
    local services="klipper"
    if [ "${RESTART_MOONRAKER}" = "yes" ]; then
        services="klipper moonraker"
    fi
    if [ "${DO_RESTART}" != "yes" ]; then
        echo "[INFO] Restart when you are ready:  sudo systemctl restart ${services}"
        return
    fi
    if ! command -v systemctl >/dev/null 2>&1; then
        echo "[WARNING] systemctl not found, restart yourself: ${services}"
        return
    fi
    # shellcheck disable=SC2086
    if sudo systemctl restart ${services}; then
        echo "[POST-INSTALL] Restarted: ${services}"
    else
        echo "[WARNING] Restart failed, run it yourself:  sudo systemctl restart ${services}"
    fi
}

function uninstall {
    local target="${EXTRAS}/dock_autotune.py"
    local cfg="${CONFIG_PATH}/printer.cfg"
    local name
    for name in dock_autotune.cfg dock_autotune_macro.cfg; do
        if [ -f "${cfg}" ] && grep -qE "$(include_pattern "${name}")" "${cfg}"; then
            backup_file "${cfg}"
            sed -i -E "/$(include_pattern "${name}")/d" "${cfg}"
            echo "[UNINSTALL] Removed [include ${name}] from printer.cfg"
        fi
    done
    if [ -L "${target}" ]; then
        rm -f "${target}"
        echo "[UNINSTALL] Removed ${target}"
    elif [ -e "${target}" ]; then
        echo "[WARNING] ${target} is not a link and was left alone"
    else
        echo "[INFO] ${target} is not installed"
    fi
    echo "[INFO] Not touched: ${CONFIG_PATH}/dock_autotune*.cfg and the update manager section in moonraker.conf."
    restart_services
}

function show_completion_message {
    cat <<EOF

========================================
Installation complete
========================================
Next steps:
  1. Make sure the error_gcode of your toolchanger leaves the dock after a failed pickup and does not
     pause while a scan runs, see ${INSTALL_PATH}/examples/toolchanger-error-handler.cfg
  2. Try:  DOCK_AUTOTUNE T=1 DRY_RUN=1
Documentation: ${INSTALL_PATH}/README.md
EOF
}

printf "\n============================================\n"
echo "  KTC Dock Autotune Installer"
printf "============================================\n\n"

preflight_checks
if [ "${MODE}" = "uninstall" ]; then
    uninstall
    exit 0
fi
check_download
link_module
install_config
add_include dock_autotune.cfg
if [ "${DO_MACRO}" = "yes" ]; then
    add_include dock_autotune_macro.cfg
fi
add_update_manager
restart_services
show_completion_message
