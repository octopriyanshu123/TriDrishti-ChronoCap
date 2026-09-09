# ChronoCap

A lightweight, schema-agnostic **record and replay** library for `i2w` pub/sub
topics. ChronoCap captures any number of differently-typed, differently-rated
topics into a single time-ordered, endian-safe `.bin` file, and replays them
back later at their original relative timing — reconstructing the same rates
and the same cross-topic timing relationships automatically.

Think of it like multi-track studio recording: each publisher (pose, imu,
lidar, whatever) is a separate instrument playing at its own tempo. Instead
of recording separate tapes and syncing them up later, everything is written
onto one shared, time-stamped master timeline. Replay just plays that master
tape back in real time — every topic naturally lands where it should, in
sync with the others, with no manual reconciliation needed.

---

## Table of contents

1. [Concept: how record and replay work](#1-concept-how-record-and-replay-work)
2. [Architecture](#2-architecture)
3. [Project layout](#3-project-layout)
4. [The `.bin` file format, byte by byte](#4-the-bin-file-format-byte-by-byte)
5. [How to add a new topic](#5-how-to-add-a-new-topic)
6. [How to enable / disable a topic](#6-how-to-enable--disable-a-topic)
7. [Building](#7-building)
8. [Running](#8-running)
9. [Design decisions and rationale](#9-design-decisions-and-rationale)

---

## 1. Concept: how record and replay work

### Recording

```
[Pub: Pose2D  @10Hz]  ─┐
[Pub: Axis    @100Hz] ─┤
[Pub: Buttons @20Hz]  ─┼─► i2wRecorder ─► ChronoCap (lib) ─► ONE merged .bin
                        │        │
                (3 typed i2w      │
                 subscriptions)   ▼
                          MPSC ring buffer (lock-free)
                                  │
                                  ▼
                          dedicated writer thread
                          (write() + periodic fsync())
```

- **Subscriber callbacks never touch disk.** Each callback (`OnPose`,
  `OnAxis`, `OnButtons`) encodes the sample into a small fixed-size,
  endian-safe byte buffer and pushes it into a **lock-free MPSC ring
  buffer**. This push is wait-free — it never blocks, never waits on a
  mutex, and never waits on I/O.
- **A single dedicated writer thread** drains the ring buffer continuously
  and is the *only* thing that ever touches the file — calling `write()`
  to append records, and `fsync()` periodically to bound how much data
  could be lost in a crash.
- This decoupling means a slow or momentarily-stalling disk can **never**
  block or delay message delivery on the subscriber side — the worst case
  is the ring buffer fills up and new samples are dropped (counted, not
  silently lost from your awareness), never that a publisher/subscriber
  callback hangs waiting on disk I/O.

### Replaying

```
recording.bin ──► mmap() (read-only) ──► i2wReplayer walks records sequentially
                                                  │
                                for each record:
                                  1. compute target = replay_start + (stamp_ns - t0)
                                  2. sleep until target (absolute offset, no drift)
                                  3. decode payload
                                  4. republish on the original topic
```

- The file is opened with **`mmap()`**, not `read()` — replay is read-heavy
  access into a file whose size is already known, exactly where `mmap`
  shines: "advancing" to the next record is just pointer arithmetic, no
  syscalls per record.
- Timing uses **absolute offsets from a fixed `t0`** (`target = replay_start
  + (stamp_ns - t0)`), not "sleep for the delta since the last record".
  Sleeping-by-delta accumulates drift over a long recording; absolute
  offsets stay locked to the original timing no matter how long the file
  is.
- Only topics that are **actually present in the file's metadata table**
  are advertised as publishers — a topic that was disabled at record time
  never appears anywhere in the file and is never advertised at replay
  time.

### Why write() to record but mmap() to replay?

| | Recording | Replay |
|---|---|---|
| **Access pattern** | append-only, unbounded stream | read-heavy, sequential, known size |
| **Right primitive** | `write()` | `mmap()` |
| **Why** | Each `write()` call is a clear, checkable commit to disk and doesn't require pre-knowing the final file size. | mmap gives on-demand access without a RAM blow-up, and turns "read the next record" into pointer arithmetic. |

Using `mmap()` for *writing* would require pre-allocating a fixed-size file
up front (awkward for a live, unbounded stream) and risks a larger
crash-loss window (dirty pages only reach disk on the OS's own schedule
unless you force an `msync()`). Using `write()` for *replay* would mean a
syscall per record instead of memory access. The asymmetric choice — write
to record, mmap to replay — is the standard pattern used by most
logging/replay systems (e.g. rosbag-style tools).

---

## 2. Architecture

The project is split into two layers on purpose:

```
lib/  (chrono_cap library)
  Knows only: (topic_id, stamp_ns, seq, raw bytes).
  Never mentions Pose2D / Axis / Buttons / i2w by name.
  Fully reusable outside this project.

test/  (glue layer + this project's schema)
  Knows: i2w's subscribe/advertise/publish API, and this project's
  specific message structs (Pose2D, Axis, Buttons).
  Encodes/decodes payloads and calls into lib/ for all persistence.
```

This separation means the storage engine (ring buffer, file format,
threading, mmap replay, timing) can be reused for a completely different
set of message types in a different project, without touching `lib/` at
all — only the schema layer (`logger_types.hpp`, `topic_registry.hpp`,
`payload_codec.hpp`) and the small `i2wRecorder.cpp` / `i2wReplayer.cpp`
glue would need to change.

---

## 3. Project layout

```
TriDrishti-ChronoCap/
├── CMakeLists.txt
├── chrono_cap/                              ← chrono_cap: schema-agnostic engine
    ├── CMakeLists.txt
    ├── include/chrono_cap/
    │   ├── wire_format.hpp           endian-safe container structs
    │   ├── mpsc_ring_buffer.hpp      lock-free MPSC ring buffer
    │   ├── record_writer.hpp         generic Push(topic_id, stamp, seq, bytes, len)
    │   └── record_reader.hpp         generic Run(callback)
    └── src/
    │   ├── record_writer.cpp
    │   └── record_reader.cpp
    └── test/                             ← i2w-specific glue + message schema
        ├── CMakeLists.txt
        ├── logger_types.hpp              Pose2D, Axis, Buttons struct definitions
        ├── topic_registry.hpp            TopicId enum + TopicTraits<T> per struct
        ├── payload_codec.hpp             EncodePayload<T> / DecodePayload<T>
        ├── logger_config.hpp             JSON-driven per-topic enable/disable
        ├── config/logger_config.json     the actual enable/disable config file
        ├── pub.cpp                       plain 3-topic i2w publisher (demo)
        ├── sub.cpp                       plain 3-topic i2w subscriber (demo)
        ├── i2wRecorder.cpp                subscribes via i2w, records via chrono_cap
        ├── i2wReplayer.cpp                replays via chrono_cap, republishes via i2w
        ├── generate_and_record.cpp       standalone: random data → .bin (NO i2w needed)
        └── replay_and_print.cpp          standalone: .bin → decoded text file (NO i2w needed)
```

---

## 4. The `.bin` file format, byte by byte

All multi-byte integers are stored **little-endian**, regardless of the
host CPU's native endianness, via explicit byte-by-byte encode/decode
helpers (`encode_u16/32/64` / `decode_u16/32/64` in `wire_format.hpp`) —
**never** a raw `memcpy` of a struct. This is what makes a recording made
on ARM replayable correctly on x86, and vice versa.

### Overall layout

```
┌───────────────────────────────┐
│  FILE HEADER (fixed, 32 bytes)│
├───────────────────────────────┤
│  TOPIC METADATA TABLE         │  one entry per enabled topic
│  (topic_count × 38 bytes)     │
├───────────────────────────────┤
│  RECORD 1                     │  RecordHeader (18B) + payload (fixed per topic)
│  RECORD 2                     │
│  RECORD 3                     │
│  ...                          │
└───────────────────────────────┘
```

There is **no separate index file**. Replay is sequential, start-to-end
only (a deliberate simplification — see [§9](#9-design-decisions-and-rationale)),
so each record's size is computed on the fly from the topic metadata
table rather than stored per-record or looked up in an index.

### File header — 32 bytes, written once at the top

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 8 bytes | `magic` | Fixed bytes `"I2WLOG\0\0"`. Lets any tool sanity-check "is this actually one of our files" before parsing further. |
| 8 | 4 bytes | `version` | Format version (currently `1`). Bump this if the layout ever changes in a backward-incompatible way. |
| 12 | 4 bytes | `topic_count` | How many `TopicMetaEntry` records follow. |
| 16 | 8 bytes | `created_at_ns` | Wall-clock time (ns since epoch) the file was opened. |
| 24 | 8 bytes | `reserved` | Zero-filled padding, reserved for future header fields without breaking this layout. |

### Topic metadata entry — 38 bytes, repeated `topic_count` times

| Offset (within entry) | Size | Field | Meaning |
|---|---|---|---|
| 0 | 2 bytes | `topic_id` | Numeric id matching `TopicId` in `topic_registry.hpp`. |
| 2 | 4 bytes | `struct_size` | The **fixed encoded wire size** of this topic's payload (`TopicTraits<T>::wire_size` — NOT `sizeof(T)`; see [§5](#5-how-to-add-a-new-topic) for why). |
| 6 | 32 bytes | `name` | Fixed-width, null-padded topic name string (e.g. `"pose"`), so the file is human-inspectable even without the code. |

Only topics that were **enabled** in `logger_config.json` at record time get
an entry here. A disabled topic has no entry, no records, and is never
referenced anywhere in the file.

### Record — `18 bytes header + N bytes payload`, repeated for every message

**Record header — 18 bytes:**

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 2 bytes | `topic_id` | Which topic this record belongs to — look up its `struct_size` from the metadata table to know the payload length. |
| 2 | 8 bytes | `stamp_ns` | The **original publish timestamp** (`sample.header.stamp_ns`), not the time the recorder received it. This is what replay timing is based on. |
| 10 | 8 bytes | `seq` | The publisher's sequence number for this sample. |

**Payload — length = that topic's `struct_size`, immediately following the header.**
Encoded field-by-field (see `payload_codec.hpp`), for example:

- `Pose2D` (12 bytes): `x` (4B), `y` (4B), `yaw` (4B) — each a `float`,
  reinterpreted as its raw IEEE-754 bit pattern and encoded as a
  little-endian `uint32_t`. (ARM and x86 both use IEEE-754 for `float`, so
  only the *byte order* of the bits needs normalizing — the bit pattern
  itself is portable.)
- `Axis` / `Buttons` (20 bytes each): `sequence` (`uint64_t`, 8B),
  `timestamp_ns` (`uint64_t`, 8B), `axes_count`/`buttons_count` (`int`,
  encoded as 4B).

**No length field is stored per record.** Since every topic's payload size
is fixed and already known from the metadata table, storing it again on
every single record would be wasted bytes at scale (hundreds of millions
of records over a long recording).

### Worked example

A file recording only `pose` (id 0) and `axis` (id 1), with 2 pose records
and 1 axis record, looks like:

```
[FileHeader: magic, version=1, topic_count=2, created_at_ns, reserved]
[TopicMetaEntry: id=0, struct_size=12, name="pose"]
[TopicMetaEntry: id=1, struct_size=20, name="axis"]
[RecordHeader: topic_id=0, stamp_ns=0,         seq=0][Pose2D payload, 12 bytes]
[RecordHeader: topic_id=1, stamp_ns=10000000,  seq=0][Axis payload, 20 bytes]
[RecordHeader: topic_id=0, stamp_ns=100000000, seq=1][Pose2D payload, 12 bytes]
```

---

## 5. How to add a new topic

Say you want to add a new `Imu` topic. Five files change, in this order:

### Step 1 — define the struct (`logger_types.hpp`)

```cpp
namespace logger_msgs
{
    struct Imu final
    {
        float accel_x;
        float accel_y;
        float accel_z;
        float gyro_z;
    };
}
```

### Step 2 — register it (`topic_registry.hpp`)

Add a new enum value (**append only — never reorder or reuse an existing
value**, since old recorded files' `topic_id` bytes must keep meaning the
same thing forever) and a `TopicTraits` specialization:

```cpp
enum class TopicId : std::uint16_t
{
    Pose    = 0,
    Axis    = 1,
    Buttons = 2,
    Imu     = 3,   // new — always append, never renumber existing entries
};

template <>
struct TopicTraits<Imu> final
{
    static constexpr TopicId id       = TopicId::Imu;
    static constexpr const char* name = "imu";
    // 4 floats, each encoded as 4 bytes -> 16 bytes, fixed, on any platform.
    static constexpr std::size_t wire_size = 16;
};
```

`wire_size` must be the **fixed encoded byte count**, not `sizeof(Imu)` —
`sizeof()` can differ across compilers/platforms due to struct padding,
which would silently break cross-architecture replay.

### Step 3 — add the codec (`payload_codec.hpp`)

Field-by-field, endian-safe encode/decode — never a raw `memcpy`:

```cpp
inline void EncodePayload(const logger_msgs::Imu& v, std::uint8_t* out) noexcept
{
    std::uint32_t bits;
    std::memcpy(&bits, &v.accel_x, sizeof(bits)); encode_u32(out + 0, bits);
    std::memcpy(&bits, &v.accel_y, sizeof(bits)); encode_u32(out + 4, bits);
    std::memcpy(&bits, &v.accel_z, sizeof(bits)); encode_u32(out + 8, bits);
    std::memcpy(&bits, &v.gyro_z,  sizeof(bits)); encode_u32(out + 12, bits);
}

inline void DecodePayload(const std::uint8_t* in, logger_msgs::Imu& v) noexcept
{
    std::uint32_t bits;
    bits = decode_u32(in + 0);  std::memcpy(&v.accel_x, &bits, sizeof(bits));
    bits = decode_u32(in + 4);  std::memcpy(&v.accel_y, &bits, sizeof(bits));
    bits = decode_u32(in + 8);  std::memcpy(&v.accel_z, &bits, sizeof(bits));
    bits = decode_u32(in + 12); std::memcpy(&v.gyro_z,  &bits, sizeof(bits));
}
```

### Step 4 — wire it into `i2wRecorder.cpp`

Add a callback and an `OnSetup()` block, following the existing pattern:

```cpp
void OnImu(const i2w::Sample<Imu>& sample)
{
    std::uint8_t buf[TopicTraits<Imu>::wire_size];
    logger_wire::EncodePayload(sample.value, buf);
    g_writer.Push(static_cast<std::uint16_t>(TopicTraits<Imu>::id),
                  sample.header.stamp_ns, sample.header.seq, buf, sizeof(buf));
}
```

```cpp
if (g_logger_config.IsEnabled<Imu>())
{
    auto sub = runtime().subscribe<Imu>(TopicTraits<Imu>::name, &OnImu, opts);
    if (!sub) return i2w::Fail();
    imu_sub_ = std::move(sub.value());
    AddTopicInfo<Imu>();
    std::printf("[i2wRecorder] imu:     ENABLED\n");
}
```

(plus an `i2w::Subscription<Imu> imu_sub_{};` member)

### Step 5 — wire it into `i2wReplayer.cpp`

Add a case in `OnTopicFound()` (advertises the publisher) and in
`OnRecord()` (decodes and republishes):

```cpp
// in OnTopicFound():
case TopicId::Imu:
{
    auto pub = runtime().advertise<Imu>(TopicTraits<Imu>::name, opts);
    if (!pub) return false;
    imu_pub_ = std::move(pub.value());
    return true;
}

// in OnRecord():
case TopicId::Imu:
{
    Imu v{};
    logger_wire::DecodePayload(payload, v);
    imu_pub_.publish(v, stamp_ns);
    break;
}
```

### Step 6 — add it to `logger_config.json`

See [§6](#6-how-to-enable--disable-a-topic) below.

That's it — no changes needed anywhere in `lib/`. The library never knew
`Pose2D`/`Axis`/`Buttons` existed and doesn't need to know `Imu` exists
either; it only ever sees `(topic_id, stamp_ns, seq, bytes)`.

---

## 6. How to enable / disable a topic

Topics are toggled entirely through `config/logger_config.json` — **no
recompile needed.**

```json
{
  "topics": [
    { "name": "pose",    "struct_id": 0, "enabled": true  },
    { "name": "axis",    "struct_id": 1, "enabled": false },
    { "name": "buttons", "struct_id": 2, "enabled": true  },
    { "name": "imu",     "struct_id": 3, "enabled": true  }
  ]
}
```

- `name` must match `TopicTraits<T>::name` exactly.
- `struct_id` is a **cross-check**, not the source of truth — the
  compile-time `TopicTraits<T>::id` is authoritative. If `struct_id`
  doesn't match what the registry expects for that name, `IsEnabled<T>()`
  logs a warning and treats the topic as disabled (fail-safe, rather than
  silently recording under a mismatched id).
- A topic **absent** from the JSON entirely defaults to **enabled**.
- Setting `"enabled": false` means: no subscription is created, no
  `TopicMetaEntry` is written, and the topic never appears anywhere in
  the output `.bin` for that session.
- The config is loaded **once**, in `main()`, before any subscriber thread
  starts — this is what makes concurrent reads of `IsEnabled<T>()` from
  multiple subscriber threads safe (no concurrent writes to the config
  after startup).

To toggle a topic: edit the JSON, restart `i2wRecorder`. Each new run
creates a fresh, independently-timestamped `.bin` file
(`recording_YYYYMMDD_HHMMSS.bin`), so different sessions can have
different topics enabled without any conflict.

---

## 7. Building

```bash
# from the project/ directory
mkdir build && cd build
cmake ..
make
```

This builds:
- `libchrono_cap.so` — the schema-agnostic engine (shared library)
- `pub`, `sub` — plain i2w demo nodes
- `i2wRecorder`, `i2wReplayer` — the real record/replay nodes (require the `i2w` framework)
- `generate_and_record`, `replay_and_print` — standalone test tools (do **not** require `i2w` at all)

> **Note:** `CMakeLists.txt` assumes `nlohmann_json` is discoverable via
> `find_package`. Swap that line for whatever your dependency-fetch
> mechanism actually is (FetchContent, vcpkg, conan, etc.).

To compile just the standalone tools without the full CMake project (e.g.
if `i2w` isn't available in your environment):

```bash
g++ -std=c++17 -Ilib/include -Itest -c lib/src/record_writer.cpp -o record_writer.o
g++ -std=c++17 -Ilib/include -Itest -c lib/src/record_reader.cpp -o record_reader.o
g++ -std=c++17 -Ilib/include -Itest test/generate_and_record.cpp record_writer.o record_reader.o -lpthread -o generate_and_record
g++ -std=c++17 -Ilib/include -Itest test/replay_and_print.cpp    record_writer.o record_reader.o -lpthread -o replay_and_print
```

---

## 8. Running

### Real i2w pipeline

```bash
./pub &                 # publishes pose/axis/buttons at their real rates
./i2wRecorder           # records enabled topics to recording_<timestamp>.bin
# ... let it run, then Ctrl+C ...
./i2wReplayer recording_20260908_153012.bin   # republishes at original timing
./sub                   # (in another terminal) prints what's replayed
```

### Standalone (no i2w needed) — for testing the library itself

```bash
./generate_and_record 5.0                          # 5 seconds of random data
./replay_and_print recording_<timestamp>.bin out.txt
cat out.txt
```

Example output line format:
```
[pose]    seq=0 stamp_ns=0 x=-1.017 y=-8.874 yaw=-5.982
[axis]    seq=3 stamp_ns=30000000 axes_count=7
[buttons] seq=1 stamp_ns=50000000 buttons_count=11
```

---

## 9. Design decisions and rationale

A few choices that came up during development, and why:

| Decision | Rationale |
|---|---|
| `write()` to record, `mmap()` to replay | Recording is an unbounded append stream (no known final size); replay is read-heavy into a file of known size. Each is the natural fit for its access pattern. |
| Lock-free MPSC ring buffer + dedicated writer thread | Decouples subscriber callbacks from disk I/O entirely — a slow/stalling disk can never block message delivery. |
| Per-slot `fetch_add` + `ready` flag, not a mutex | Genuinely lock-free (no blocking, no priority inversion risk) at negligible extra complexity for 3+ producers. |
| Explicit little-endian encode/decode for every field | Enables recording on one CPU architecture (e.g. ARM) and replaying on another (e.g. x86) correctly — a raw `memcpy` of a struct is not portable across architectures. |
| `wire_size` is a fixed constant, not `sizeof(T)` | `sizeof(T)` can differ across compilers/platforms due to struct padding; the wire format must be a fixed, explicit contract, not derived from the compiler's memory layout. |
| No length field per record | The payload length for a given `topic_id` is already known from the metadata table — repeating it per record wastes 4 bytes × every single record at scale. |
| No separate `.idx` file (for now) | Current requirement is sequential start-to-end replay only, which is already effectively free via `mmap` pointer arithmetic — an index only earns its cost once arbitrary-timestamp seeking becomes an actual requirement. |
| Periodic `fsync()`, not per-record | Per-record `fsync()` would hurt throughput at higher rates; the crash-loss window is bounded by "whatever's still in the ring buffer, undrained" — not by the flush interval — so periodic flushing doesn't meaningfully widen the actual risk window. |
| New `.bin` per recording session | Keeps sessions independent and simple — a fresh, uniquely-timestamped file every run, no mid-file topic-set changes to reason about. |
| `lib/` has zero dependency on `i2w` or any message type | Makes the recording/replay engine genuinely reusable in other projects — only the thin glue layer (`i2wRecorder.cpp`/`i2wReplayer.cpp`) and the schema files need to change per project. |


## 1. Final, clean file list

```
project/
├── CMakeLists.txt
├── config/
│   └── logger_config.json              ← enable/disable topics, no recompile
│
├── lib/                                 ← chrono_cap: NEVER touch for a new topic
│   ├── CMakeLists.txt
│   ├── include/i2w_logger/
│   │   ├── wire_format.hpp              endian-safe FileHeader/TopicMetaEntry/RecordHeader
│   │   ├── mpsc_ring_buffer.hpp         lock-free MPSC ring buffer
│   │   ├── record_writer.hpp/.cpp       generic Push(topic_id, stamp, seq, bytes, len)
│   │   └── record_reader.hpp/.cpp       generic Run(callback) — mmap sequential replay
│
└── test/  (your real project probably calls this "src/")
    ├── CMakeLists.txt
    │
    │  ── CONTROL FILES: touch these 3 whenever a topic is added ──
    ├── topic_list.hpp                   ★ master switch: which types exist at all
    ├── topic_registry.hpp               ★ per-type: TopicId + name + wire_size
    ├── payload_codec.hpp                ★ per-type: Encode/DecodePayload (byte layout)
    │
    │  ── GENERIC ENGINE: never touch, works for any TopicList ──
    ├── generic_recorder.hpp             subscribes to every enabled type in AllTopics
    ├── generic_replayer.hpp             advertises/dispatches every type in AllTopics
    ├── logger_config.hpp                loads JSON, IsEnabled<T>() lookup
    │
    │  ── ENTRY POINTS: thin, just wire generic engine + i2w together ──
    ├── i2wRecorder.cpp                  main() for recording
    ├── i2wReplayer.cpp                  main() for replay
    │
    │  ── OPTIONAL / STANDALONE TEST TOOLS: no i2w needed at all ──
    ├── generate_and_record.cpp
    ├── replay_and_print.cpp
    │
    │  ── LEFTOVER FROM EARLIER DEMOS — safe to delete or keep as reference ──
    ├── logger_types.hpp                 (Pose2D/Axis/Buttons — only needed if you keep them in AllTopics)
    ├── pub.cpp                          (demo-only publisher, not part of real pipeline)
    └── sub.cpp                          (demo-only subscriber, not part of real pipeline)
```

**What to actually delete:** `recorder.cpp` and `replayer.cpp` (the very first, non-generic, non-library versions from early in this conversation) — they were superseded first by the `chrono_cap` split, then by the generic engine. If you still have those two files sitting in the repo, they're the "leftover from last logging" you're seeing. `pub.cpp`/`sub.cpp` are fine to keep as reference/sanity-check tools, but are not part of the real record/replay pipeline against `crawler_i2w_msgs`.

---

## 2. The 3 control files — what each one actually controls

| File | Answers the question | Example line |
|---|---|---|
| **`topic_list.hpp`** | "What message types exist in this build at all?" | `using AllTopics = TopicList<JoyMsgs, MotorStatus, ...>;` |
| **`topic_registry.hpp`** | "For this type, what's its numeric id, topic name, and fixed byte size on disk?" | `TopicTraits<JoyMsgs>::{id=4, name="joy", wire_size=22}` |
| **`payload_codec.hpp`** | "For this type, exactly how do its fields turn into those 22 bytes?" | `EncodePayload(JoyMsgs, buf)` / `DecodePayload(buf, JoyMsgs&)` |

`generic_recorder.hpp`/`generic_replayer.hpp` never mention `JoyMsgs` by name anywhere — they only ever write `Traits::id`, `Traits::name`, `Traits::wire_size`, `EncodePayload(sample.value, buf)`. That's the whole point of the split: **adding a topic = editing exactly these 3 files**, nothing else.

---

## 3. Step-by-step: Recording

```
1. main() in i2wRecorder.cpp:
   - LoggerConfig::LoadFromFile("logger_config.json")   → reads enabled/disabled per topic
   - RecordWriter writer;                                 (empty, not opened yet)
   - RecorderSystem system(config, {&logger_config, &writer});
   - system.Setup()
        └─► OnSetup() fold-expands SetupOne<T>() for EVERY T in AllTopics:
              for JoyMsgs:
                if !config.IsEnabled<JoyMsgs>()  →  skip, print "disabled"
                else:
                  subscribe<JoyMsgs>("joy", callback, opts)
                  callback = [](sample) {
                      EncodePayload(sample.value, buf[22])   ← payload_codec.hpp
                      writer->Push(topic_id=4, stamp_ns, seq, buf, 22)
                  }
                  push TopicInfo{id=4, wire_size=22, name="joy"} into enabled_topics_

2. writer.Open("recording_<timestamp>", system.EnabledTopics())
        └─► writes FileHeader + one TopicMetaEntry per ENABLED topic only
        └─► starts the dedicated writer thread

3. Runtime:
   every real /joy message arrives → OnSetup's lambda fires
     → EncodePayload → writer.Push() → lock-free ring buffer (never blocks)
   writer thread (separate, continuous):
     drains ring buffer → RecordHeader{topic_id, stamp_ns, seq} + payload bytes
     → write() to .bin
     → periodic fsync()

4. Ctrl+C / shutdown → writer.Close() → final fsync, file done.
```

---

## 4. Step-by-step: Replay

```
1. main() in i2wReplayer.cpp <path.bin>:
   - ReplayerSystem system(config);
   - system.Setup()   (no-op — nothing known yet, file not opened)

2. RecordReader reader;
   reader.Open(path, on_topic_hook)
        └─► mmap()s the file
        └─► parses FileHeader (magic, version, topic_count)
        └─► for each TopicMetaEntry found (e.g. topic_id=4, wire_size=22, name="joy"):
              calls on_topic_hook(ResolvedTopic{4, 22, "joy"})
                → system.OnTopicFound(topic)
                     └─► fold-expands TryAdvertise<T>() over ALL T in AllTopics:
                           for JoyMsgs: if topic.topic_id == TopicTraits<JoyMsgs>::id (4)
                             check topic.wire_size == TopicTraits<JoyMsgs>::wire_size (22)  ← hard fail if mismatch
                             advertise<JoyMsgs>("joy") → store in pubs_ tuple
                           (every other T in the fold just returns immediately — id doesn't match)

3. reader.Run(callback)
        └─► walks the mmap'd bytes sequentially, record by record:
              read RecordHeader → topic_id=4 → look up wire_size=22 from metadata table
              sleep until (replay_start + (stamp_ns - t0))   ← absolute offset, no drift
              callback(topic_id=4, stamp_ns, seq, payload_ptr, 22)
                → system.OnRecord(...)
                     └─► fold-expands DispatchOne<T>() over ALL T in AllTopics:
                           for JoyMsgs: if topic_id == 4:
                             JoyMsgs v{};
                             DecodePayload(payload_ptr, v)     ← payload_codec.hpp, reverses EncodePayload
                             pubs_.get<Publisher<JoyMsgs>>().publish(v, stamp_ns)
              advance pointer by (18-byte header + 22-byte payload), repeat
```

---

## 5. Encode/decode — the actual byte contract, end to end

```
Live message (JoyMsgs struct, in RAM, may have compiler padding)
        │  EncodePayload()  — field by field, NEVER memcpy the whole struct
        ▼
22 raw bytes, little-endian, no padding, portable ARM ↔ x86
        │  RecordWriter writes: [RecordHeader: id=4,stamp,seq][these 22 bytes]
        ▼
.bin file on disk
        │  RecordReader mmaps + reads back the same 22 bytes
        ▼
        │  DecodePayload()  — field by field, reconstructs the struct
        ▼
JoyMsgs struct, in RAM, on possibly a DIFFERENT architecture — values identical to what was recorded
```

**The one rule that keeps this whole system correct:** every type in `AllTopics` (topic_list.hpp) MUST have exactly one `TopicTraits<T>` (topic_registry.hpp) and exactly one `EncodePayload`/`DecodePayload` pair (payload_codec.hpp) whose byte counts agree with `wire_size`. If any of the 3 is missing or inconsistent, it's a **compile error** (as you just hit) — by design, not a bug, so a mismatch is caught immediately rather than silently corrupting recordings.