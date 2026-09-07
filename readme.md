At 500GB you're no longer in "just write records to a file" territory — you need to think about **streaming I/O, disk space management, file rotation, compression, and indexing** so nothing blows up mid-capture. Here's the plan.

## 1. Never buffer in RAM — stream straight to disk

Your recorder should hold at most a small ring buffer in memory (a few MB), not accumulate data. Write in fixed-size chunks using low-level I/O, not `std::ofstream` with default buffering:

```cpp
int fd = open("capture.bin", O_WRONLY | O_CREAT | O_APPEND, 0644);
// write() directly, in chunks of e.g. 64KB–1MB
```

Use `write()`/`pwrite()` directly rather than iostreams — less overhead, more control over buffering, and you can use `O_DIRECT` if you want to bypass the page cache entirely (avoids cache pressure/thrashing at high sustained throughput, but needs aligned buffers).

## 2. Split into rotating chunk files, don't use one giant file

Instead of one 500GB file:

```
capture_0000.bin   (e.g. 1GB each)
capture_0001.bin
capture_0002.bin
...
capture_index.idx
```

Why this matters:
- A single 500GB file is painful to `seek`, back up, transfer, or recover from if corrupted (one bad byte doesn't kill everything).
- Easier to stream to cold storage / delete old chunks while still recording new ones.
- Easier to parallelize replay/analysis later.

Rotate on size (e.g. every 1–2GB) or time (every N minutes), whichever comes first.

## 3. Build an index file alongside

For each chunk, record: `chunk_id, start_timestamp, end_timestamp, file_offset_start`. This lets replay **seek instantly** to any point in a 500GB capture instead of reading sequentially from the start.

```cpp
struct IndexEntry {
    uint32_t chunk_id;
    uint64_t t_start_ns;
    uint64_t t_end_ns;
};
```

Write this as a small separate `.idx` file (or SQLite if you want queryability) — it stays tiny (KBs–MBs) even for a 500GB capture.

## 4. Compress — usually cuts size a lot

Raw sensor/serial/CAN data is often very compressible (repeated headers, similar payloads). Two options:

- **Inline**: compress each chunk with `zstd` as you close it (fast, streaming API, good ratio). This can realistically turn 500GB into 100–200GB depending on data.
- **Background**: write raw, then a separate low-priority thread/process compresses finished chunk files and deletes the raw ones.

zstd's streaming API (`ZSTD_compressStream2`) fits naturally into a chunked-file design — compress each chunk as you finish writing it.

## 5. Manage disk space actively — don't let it run out mid capture

Add a watchdog thread that checks free space (`statvfs()`) periodically:
- Below a threshold → either stop gracefully, alert, or start deleting/archiving oldest chunks (ring-buffer style capture — keep "last N hours" only).
- This is critical: a `write()` failing mid-record because disk is full and crashing your recorder is the worst failure mode — check return values and handle `ENOSPC` explicitly.

## 6. Replay at this scale

Replay reads sequentially from chunk to chunk (or jumps via the index) — same delta-sleep logic as before, just now iterating over multiple files:

```cpp
for (auto& chunk : sorted_chunks) {
    auto fd = open(chunk.path, O_RDONLY);
    // stream records, sleep(delta), emit
}
```

If chunks are compressed, decompress on the fly with a streaming decompressor rather than expanding the whole chunk into memory.

## Summary of the shape

```
Recorder:  device -> ring buffer -> chunked writer -> (optional) zstd -> disk
                                          |
                                     index writer
                                          |
                                   disk-space watchdog

Replayer:  index -> pick chunk(s) -> (decompress) -> delta-sleep emit -> consumer
```