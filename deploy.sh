cd embedded
pio run -t compiledb
pio run --target upload || { echo 'Upload failed'; exit 1; }
cd ../../viz
uv run viz.py
