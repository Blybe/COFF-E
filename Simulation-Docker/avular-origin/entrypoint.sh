#!/bin/bash
# =============================================================================
# COFF-E - Avular Origin simulation container entrypoint
# =============================================================================
# Starts the display backend for the Gazebo GUI and runs the given command
# (default: the Origin One Gazebo simulation).
#
# Display modes:
#   1. Virtual display (default): Xvfb + x11vnc + noVNC, Gazebo GUI visible
#      in the browser at http://localhost:6080/vnc.html?autoconnect=true
#      (works from Windows / macOS / Linux hosts).
#   2. Host display: set USE_HOST_DISPLAY=1 and pass DISPLAY (e.g. ":0").
#      Works with WSLg on Windows 11 or a normal Linux X11 session.
#   3. Headless: set HEADLESS=true to run without any GUI (sensors and
#      topics keep working; useful for CI or slower machines).
# =============================================================================

set -e

source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"

NOVNC_PORT="${NOVNC_PORT:-6080}"

start_virtual_display() {
    export DISPLAY=":1"
    export LIBGL_ALWAYS_SOFTWARE=1
    # Stale lock from a previous run (container restart keeps /tmp)
    rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
    Xvfb :1 -screen 0 1600x900x24 +extension GLX -nolisten tcp -ac &
    # Give the X server a moment before starting its clients
    sleep 2
    fluxbox > /tmp/fluxbox.log 2>&1 &
    x11vnc -display :1 -forever -shared -nopw -rfbport 5900 \
        > /tmp/x11vnc.log 2>&1 &
    websockify --web /usr/share/novnc "$NOVNC_PORT" localhost:5900 \
        > /tmp/websockify.log 2>&1 &
    echo "[entrypoint] Gazebo GUI available at: http://localhost:${NOVNC_PORT}/vnc.html?autoconnect=true&resize=scale"
}

display_reachable() {
    [ -n "$DISPLAY" ] && xset -q > /dev/null 2>&1
}

if [ "${HEADLESS:-false}" = "true" ]; then
    echo "[entrypoint] HEADLESS=true - starting without GUI."
elif [ "${USE_HOST_DISPLAY:-0}" = "1" ]; then
    if display_reachable; then
        echo "[entrypoint] Using host X display: ${DISPLAY}"
    else
        echo "[entrypoint] USE_HOST_DISPLAY=1 but DISPLAY '${DISPLAY:-<unset>}' is not reachable - falling back to the virtual display."
        start_virtual_display
    fi
elif display_reachable; then
    echo "[entrypoint] Using host X display: ${DISPLAY}"
else
    start_virtual_display
fi

exec "$@"
