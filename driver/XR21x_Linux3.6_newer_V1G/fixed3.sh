sudo tee /etc/systemd/system/vdsuit-xr21x-rebind.service >/dev/null <<'EOF'
[Unit]
Description=Rebind VDSuit XR21x USB UART to Exar driver
After=systemd-udevd.service

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/vdsuit-xr21x-rebind.sh
EOF