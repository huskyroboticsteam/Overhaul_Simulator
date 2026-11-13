#!/bin/bash
# Start the VNC desktop on display :1 (port 5901) and noVNC on port 6080 (browser access).
set -e

mkdir -p ~/.vnc
echo "${VNC_PASSWORD:-dockervnc}" | vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

# The default xstartup does not work inside the container
cat > ~/.vnc/xstartup <<'XSTARTUP'
#!/bin/sh
unset WAYLAND_DISPLAY WAYLAND_SOCKET SESSION_MANAGER DBUS_SESSION_BUS_ADDRESS
export DISPLAY=:1
exec dbus-launch --exit-with-session startxfce4
XSTARTUP
chmod +x ~/.vnc/xstartup

vncserver -kill :1 >/dev/null 2>&1 || true
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
vncserver :1 -geometry "${VNC_GEOMETRY:-1280x720}" -depth 24 -localhost no >/tmp/vncserver.log 2>&1

pkill -f "websockify.*6080" >/dev/null 2>&1 || true
nohup websockify --web /usr/share/novnc 6080 localhost:5901 >/tmp/novnc.log 2>&1 &

echo "Desktop ready: http://localhost:6080/vnc.html (or a VNC client on localhost:5901)"
