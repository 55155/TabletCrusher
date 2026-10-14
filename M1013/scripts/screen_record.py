#!/usr/bin/python3
"""GNOME(Wayland) 화면 녹화. GNOME Screencast는 호출한 D-Bus 연결이 끊기면 녹화가 멈추므로
이 프로세스가 녹화 내내 살아 있어야 한다.

사용법: /usr/bin/python3 screen_record.py <출력.webm> [초]   (초 생략 시 Ctrl+C 또는 SIGTERM 까지)
"""
import signal
import sys

from gi.repository import Gio, GLib

out = sys.argv[1]
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else None

bus = Gio.bus_get_sync(Gio.BusType.SESSION)
proxy = Gio.DBusProxy.new_sync(
    bus, Gio.DBusProxyFlags.NONE, None,
    "org.gnome.Shell.Screencast", "/org/gnome/Shell/Screencast", "org.gnome.Shell.Screencast")

ok, path = proxy.call_sync(
    "Screencast",
    GLib.Variant("(sa{sv})", (out, {"framerate": GLib.Variant("i", 30),
                                    "draw-cursor": GLib.Variant("b", False)})),
    Gio.DBusCallFlags.NONE, -1, None).unpack()
if not ok:
    sys.exit("녹화 시작 실패")
print(f"녹화 시작: {path}", flush=True)

loop = GLib.MainLoop()


def stop(*_):
    proxy.call_sync("StopScreencast", None, Gio.DBusCallFlags.NONE, -1, None)
    print("녹화 종료", flush=True)
    loop.quit()
    return False


GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGINT, stop)
GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGTERM, stop)
if seconds:
    GLib.timeout_add(int(seconds * 1000), stop)
loop.run()
