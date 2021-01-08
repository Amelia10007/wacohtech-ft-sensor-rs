# 力覚センサWDF-6M200-3との接続セットアップ．
# このシェルスクリプトは特権モードで実行する必要がある．

# デバイス管理ツールをインストール．
apt install libudev-dev

# udevカスタマイズルールを作成．
mkdir -p /etc/udev/rules.d
echo 'ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666"' >> /etc/udev/rules.d/80-wacoh.rules

# カスタマイズルールを変更したので，デバイス管理ツールを再起動する必要がある．
service udev restart
