cd embedded
pio run -t compiledb
pio run --target upload || { echo 'Upload failed'; exit 1; }
pio device monitor -b 230400
