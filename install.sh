#!/bin/bash
# KTC Dock Autotune installer
#
# Links klipper/extras/dock_autotune.py into Klipper's extras directory, installs the
# example configuration and (optionally) adds an update manager entry to moonraker.conf.
#
# Usage (from a clone of the repository on the printer):
#   git clone https://github.com/ChrisFo8390/ktc-dock-autotune.git ~/ktc-dock-autotune
#   ~/ktc-dock-autotune/install.sh [options]
#
# Options (without them the script asks, but only when it runs in a terminal):
#   --update-manager / --no-update-manager   add / do not add [update_manager ktc-dock-autotune]
#   --restart / --no-restart                 restart / do not restart Klipper at the end
#   --uninstall                              remove the link again (configuration is kept)
#   -h, --help                               this text
#
# Update later: git pull in the clone (or the update manager), then restart Klipper.

REPO_URL="https://github.com/ChrisFo8390/ktc-dock-autotune.git"
REPO_NAME="ktc-dock-autotune"
KLIPPER_PATH="${KLIPPER_PATH:-${HOME}/klipper}"
CONFIG_PATH="${CONFIG_PATH:-${HOME}/printer_data/config}"

set -eu
export LC_ALL=C

MODE="install"
UPDATE_MANAGER="ask"
RESTART="ask"
MOONRAKER_CHANGED="no"

for arg in "$@"; do
    case "${arg}" in
        --update-manager)    UPDATE_MANAGER="yes" ;;
        --no-update-manager) UPDATE_MANAGER="no" ;;
        --restart)           RESTART="yes" ;;
        --no-restart)        RESTART="no" ;;
        --uninstall)         MODE="uninstall" ;;
        -h|--help)
            sed -n '2,17p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
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

# ask <yes|no|ask> <question>: 0 = yes. Asks only in a terminal, otherwise the answer is no.
function decide {
    local preset="$1" question="$2" answer
    case "${preset}" in
        yes) return 0 ;;
        no)  return 1 ;;
    esac
    if [ -t 0 ]; then
        read -rp "${question} [y/N] " answer
        case "${answer}" in
            y|Y|yes|YES) return 0 ;;
        esac
    fi
    return 1
}

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
        return
    fi
    if [ ! -e "${CONFIG_PATH}/dock_autotune.cfg" ]; then
        cp "${INSTALL_PATH}/config/dock_autotune.cfg" "${CONFIG_PATH}/dock_autotune.cfg"
        echo "[INSTALL] Configuration installed to ${CONFIG_PATH}/dock_autotune.cfg"
    else
        echo "[INFO] ${CONFIG_PATH}/dock_autotune.cfg exists and is not overwritten"
        echo "[INFO] Compare it with ${INSTALL_PATH}/config/dock_autotune.cfg for new settings"
    fi
    if [ ! -f "${CONFIG_PATH}/printer.cfg" ] || ! grep -qE '^\[include dock_autotune\.cfg\]' "${CONFIG_PATH}/printer.cfg"; then
        echo "[INFO] Add this line to printer.cfg (the script does not edit it):"
        echo ""
        echo "    [include dock_autotune.cfg]"
        echo ""
    fi
}

function add_update_manager {
    local conf="${CONFIG_PATH}/moonraker.conf"
    if ! decide "${UPDATE_MANAGER}" "Add [update_manager ${REPO_NAME}] to moonraker.conf?"; then
        echo "[INFO] Update manager entry skipped"
        return
    fi
    if [ ! -f "${conf}" ]; then
        echo "[WARNING] ${conf} not found, add this section yourself:"
        echo ""
        print_update_manager_section
        return
    fi
    if grep -qF "[update_manager ${REPO_NAME}]" "${conf}"; then
        echo "[INFO] [update_manager ${REPO_NAME}] is already in ${conf} - not changed (restart Moonraker if it is not shown)"
        return
    fi
    cp "${conf}" "${conf}.bak.$(date +%Y%m%d%H%M%S)"
    printf "\n" >> "${conf}"
    print_update_manager_section >> "${conf}"
    MOONRAKER_CHANGED="yes"
    echo "[INSTALL] Added [update_manager ${REPO_NAME}] to ${conf} (backup next to it)"
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

function restart_klipper {
    local services="klipper"
    # Moonraker reads moonraker.conf only at its start: without a restart the update manager entry does not show up
    if [ "${MOONRAKER_CHANGED}" = "yes" ]; then
        services="klipper moonraker"
    fi
    if ! decide "${RESTART}" "Restart ${services// / and } now?"; then
        echo "[INFO] Restart when you are ready:  sudo systemctl restart ${services}"
        return
    fi
    if command -v systemctl >/dev/null 2>&1; then
        sudo systemctl restart ${services}
        echo "[POST-INSTALL] Restarted: ${services}"
    else
        echo "[WARNING] systemctl not found, restart Klipper yourself"
    fi
}

function uninstall {
    local target="${EXTRAS}/dock_autotune.py"
    if [ -L "${target}" ]; then
        rm -f "${target}"
        echo "[UNINSTALL] Removed ${target}"
    elif [ -e "${target}" ]; then
        echo "[WARNING] ${target} is not a link and was left alone"
    else
        echo "[INFO] ${target} is not installed"
    fi
    echo "[INFO] Not touched: ${CONFIG_PATH}/dock_autotune.cfg, the include line in printer.cfg and the"
    echo "[INFO] update manager section in moonraker.conf. Remove the include line before the next restart,"
    echo "[INFO] otherwise Klipper stops with: Section 'dock_autotune' is not a valid config section."
}

function show_completion_message {
    cat <<EOF

========================================
Installation complete
========================================
Next steps:
  1. Add  [include dock_autotune.cfg]  to printer.cfg (if not there yet)
  2. Make sure the error_gcode of your toolchanger leaves the dock after a failed pickup and does not
     pause while a scan runs, see ${INSTALL_PATH}/examples/toolchanger-error-handler.cfg
  3. Restart Klipper, then try:  DOCK_AUTOTUNE T=1 DRY_RUN=1
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
add_update_manager
restart_klipper
show_completion_message
