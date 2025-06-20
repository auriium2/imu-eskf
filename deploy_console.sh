./build.sh
cd embedded
pio run --target upload || { echo 'Upload failed'; exit 1; }
pio device monitor -b 460800
