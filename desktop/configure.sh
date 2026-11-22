# Sourced only by the desktop copy of android-install.sh.
desktop_configure() {
    . "$P/desktop-settings.env"
    if [ -n "$SSID" ]; then
        # Pro models keep their image's channel 36 and use the phone's own country, not TR.
        CH=auto; CC=TR
        if [ "$(cat "$1/etc/mu300/platform" 2>/dev/null)" = u30pro ]; then
            CH=36; CC=$(getprop ro.boot.wificountrycode 2>/dev/null || true)
            case $CC in [A-Z][A-Z]) ;; *) CC=CN ;; esac
        fi
        printf 'SSID=%s\nPSK=%s\nBAND=5\nCHANNEL=%s\nCOUNTRY=%s\n' "$SSID" "$WIFI_KEY" "$CH" "$CC" > "$1/etc/mu300/hotspot.conf"
        chmod 600 "$1/etc/mu300/hotspot.conf"
    fi
    if [ -n "$APN" ]; then
        mkdir -p "$1/etc/uci-defaults"
        printf '#!/bin/sh\nuci set network.wan.apn=%s\nuci commit network\n' "$APN" > "$1/etc/uci-defaults/99-desktop-apn"
        chmod 755 "$1/etc/uci-defaults/99-desktop-apn"
    fi
    echo sd > "$1/etc/mu300/desktop-mode"
}
