# Paula USB Audio for Emu68 / PiStorm

**Real-time Paula audio capture exposed directly to a modern host as a driverless USB Audio Class 1 device.**

This experimental Emu68 branch turns a PiStorm-equipped Amiga into a native USB audio source.

Instead of digitizing the Amiga's analog output, Emu68 reconstructs the Paula audio stream in real time from the emulated/mirrored Paula state, renders it as stereo PCM, and sends it directly through the Raspberry Pi USB controller. The software Paula renderer is based on the **MIT-licensed Amiga Paula 8364 emulator by Akustikrausch / Andreas Wendorf**.

To Windows or another compatible USB host, the Amiga simply appears as:

> **PiStorm Paula Audio**

The result is a direct digital path from classic Amiga software to modern recording, streaming and audio-processing applications, with no external sound card, no analog cable and no proprietary host driver.

This repository snapshot is based on **Emu68 commit `9b4379a`** and represents the hardware-validated **UAC1 Full-Speed / 1 ms / re-entrant streaming milestone**.

---

## Why this exists

Paula audio normally ends its journey inside the Amiga.

For modern workflows, capturing that audio usually means taking the analog output of the machine and feeding it into an external audio interface. That works, but it adds an unnecessary analog conversion stage, extra cabling, possible noise, level matching and additional hardware.

PiStorm and Emu68 make another approach possible.

Because Emu68 already has visibility into the Amiga-side Paula activity, the Raspberry Pi can reconstruct the resulting audio stream internally and expose it directly to another computer over USB.

The signal path becomes:

```text
Amiga software
     |
     v
Paula registers / DMA state
     |
     v
Emu68 Paula mirror
     |
     v
Paula reconstruction + mixer
     |
     v
48 kHz stereo PCM
     |
     v
shared audio ring
     |
     v
BCM2837 DWC2 USB device controller
     |
     v
USB Audio Class 1
     |
     v
Windows / macOS / Linux audio application
```

There is no analog capture stage anywhere in that path.

---

## What this milestone does

The current implementation exposes a **capture-only USB Audio Class 1 device** with the following format:

| Property | Value |
| --- | --- |
| USB class | USB Audio Class 1 |
| Direction | Capture / IN |
| USB speed | Full-Speed |
| Sample rate | 48 kHz |
| Sample format | Signed PCM |
| Resolution | 16-bit |
| Channels | 2 |
| Channel layout | Stereo L/R |
| USB interval | 1 ms |
| Nominal packet | 48 stereo frames |
| Correction packets | 47 / 49 stereo frames |
| Endpoint | Isochronous IN, EP1 |
| Host-visible name | `PiStorm Paula Audio` |

The device uses the standard operating-system USB audio stack. No custom Windows driver is required.

---

## Architecture

The implementation deliberately separates Paula rendering from USB transport.

### CPU1 — Paula audio producer

CPU1 runs the hardware-validated Paula reconstruction path derived from the earlier **CACHE-FIX2** work.

It:

- follows mirrored Paula register activity;
- tracks Paula DMA state;
- maintains the asynchronous Chip RAM cache;
- performs prefetching of active audio channels;
- reconstructs live Paula state;
- detects dropped or excessively delayed mirror events;
- restores state from snapshots when resynchronization is required;
- renders stereo audio at 48 kHz;
- applies the existing live mixer;
- publishes signed 16-bit stereo samples into a shared PCM ring.

The producer continuously renders the newest Paula stream even when no USB application is recording.

This is intentional: opening a recording application should attach to the *current* Amiga audio, not replay stale samples accumulated while the endpoint was closed.

### CPU3 — USB device transport

CPU3 exclusively owns the Raspberry Pi DWC2 USB device controller.

This preserves the ownership model of the previously hardware-validated Emu68 UVC USB transport from which the USB layer was derived.

CPU3:

- handles USB enumeration;
- processes EP0 control requests;
- handles UAC1 interface selection;
- owns EP1;
- packetizes PCM data;
- schedules isochronous IN transfers;
- services USB events continuously.

Keeping DWC2 ownership on one core avoids cross-core races inside the USB controller state machine.

### Shared PCM ring

CPU1 and CPU3 communicate through a lightweight shared ring containing **2048 stereo PCM frames**.

The ring is not intended as a large latency buffer. Its purpose is to decouple the independently timed Paula renderer from the USB frame clock.

A target occupancy of 384 samples is maintained.

```text
CPU1                                  CPU3

Paula state
    |
    v
48 kHz renderer
    |
    v
+-------------------------------+
|      shared PCM ring          |
|      2048 stereo frames       |
+-------------------------------+
                    |
                    v
             UAC1 packetizer
                    |
                    v
              USB ISO EP1
```

If the producer laps the consumer, obsolete history is discarded and USB resumes from the newest valid audio window.

If USB temporarily runs ahead of the producer, missing samples are emitted as silence rather than sending undefined data.

---

## Clock-domain handling

The Paula renderer and the USB host do not share the same physical clock.

A fixed 48-frame packet every millisecond would therefore eventually make the ring either grow or drain.

This implementation handles the small long-term clock difference without resampling.

Normally, each 1 ms USB frame carries:

```text
48 stereo frames
```

When the ring occupancy moves too far from its target, the packetizer can instead send:

```text
47 stereo frames
```

or:

```text
49 stereo frames
```

This gently pulls the buffer back toward its desired occupancy.

The correction is deliberately small and infrequent, allowing the audio renderer to remain untouched while the USB transport absorbs clock drift.

---

## Re-entrant USB streaming

An important part of this milestone is that EP1 is no longer treated as permanently active after enumeration.

USB Audio hosts control streaming by switching the AudioStreaming interface between alternate settings:

```text
alt 0  ->  zero bandwidth / stream stopped
alt 1  ->  active isochronous stream
```

Earlier proof-of-concept builds could successfully enumerate and stream, but closing one application and opening another could leave stale endpoint state behind. In some cases a physical USB disconnect/reconnect was required before capture worked again.

This milestone implements a real endpoint lifecycle.

### Stream stop

On `SET_INTERFACE alt=0`, Emu68 now:

1. stops accepting new IN traffic;
2. NAKs EP1;
3. requests endpoint disable when required;
4. waits, with a bounded timeout, for the DWC2 endpoint-disabled acknowledgement;
5. disables the endpoint FIFO-empty interrupt;
6. clears endpoint interrupt state;
7. clears the transfer-size register;
8. flushes the dedicated EP1 TX FIFO;
9. clears the software busy state.

### Stream start

On `SET_INTERFACE alt=1`, Emu68 starts from a known clean state.

It:

1. performs the same endpoint cleanup defensively;
2. realigns the PCM consumer to a recent window of the producer ring;
3. clears stale endpoint state;
4. rebuilds the isochronous EP1 configuration;
5. resets the software busy flag;
6. arms fresh transfers.

The operation is intentionally idempotent: even an imperfect previous close should not contaminate the next stream.

This makes switching between host applications substantially closer to the behaviour expected from a normal USB audio device.

---

## USB implementation

The USB device code is deliberately small.

It does **not** depend on TinyUSB.

The implementation talks directly to the BCM2837 DWC2 controller and was derived from an earlier, hardware-validated Emu68 USB Video Class proof of concept.

The UAC1 topology is intentionally minimal:

```text
AudioControl interface

Line Input Terminal
        |
        v
USB Streaming Output Terminal


AudioStreaming interface

alt 0 : no endpoint
alt 1 : EP1 IN, asynchronous isochronous PCM
```

The endpoint also implements the UAC1 sampling-frequency control requests required for the fixed 48 kHz stream.

---

## Why Full-Speed USB?

High-Speed UAC1 enumeration was successfully achieved during development.

However, the High-Speed isochronous transport remained silent in the tested implementation.

The known-good UVC DWC2 baseline was therefore forced into **Full-Speed device mode**, where the audio stream became reliable and a valid test tone was successfully transferred.

For this workload Full-Speed USB is more than sufficient.

The raw PCM payload is only:

```text
48,000 samples/s
× 2 channels
× 2 bytes
= 192,000 bytes/s
```

or approximately:

```text
1.536 Mbit/s
```

before USB protocol overhead.

The important result is therefore not USB bandwidth, but predictable 1 ms isochronous scheduling.

---

## Paula reconstruction

This project is not an analog recording solution.

The actual Paula sound generation is performed by a software emulator core based on the **Amiga Paula 8364 emulator by Akustikrausch / Andreas Wendorf**, released under the MIT license:

```text
https://github.com/akustikrausch/amiga-paula-8364-emulator
```

The Emu68 integration wraps that core with the live Paula mirror, asynchronous Chip RAM access/cache, replay timing, recovery logic, mixer control and USB transport described in this document. In other words, the physical Amiga supplies the real Paula programming activity, while the audio heard by the USB host is generated by this software Paula core.

It does not connect to the Amiga audio RCA output and it does not use an ADC.

Instead, the firmware observes the Paula programming state available to Emu68 and reconstructs the audio digitally.

The retained Paula path includes:

- register mirroring;
- DMA tracking;
- asynchronous Chip RAM sample access;
- cache prefetch;
- replay timing;
- recovery after mirror loss or excessive lag;
- live stereo mixing;
- diagnostic counters.

When event loss or excessive replay lag is detected, the renderer enters recovery rather than continuing from potentially inconsistent Paula state.

A fresh snapshot of the mirrored state is used to rebuild the live renderer, after which normal playback resumes.

---

## Diagnostics

The branch retains the Paula diagnostic block used during development.

Among the tracked values are:

- rendered PCM frames;
- captured and consumed Paula events;
- mirror drops;
- cache misses;
- cache fetches and invalidations;
- replay lag;
- silent recovery frames;
- snapshot retries;
- resynchronizations;
- left/right peak levels;
- sample rate;
- active Paula channel state.

The USB side additionally keeps counters for:

- packets transmitted;
- ring underruns;
- ring overruns.

These were invaluable while separating Paula-rendering problems from USB-transport problems.

---

## Host usage

After booting the modified Emu68 build and connecting the Pi's USB device port to a host computer, the operating system should enumerate:

```text
PiStorm Paula Audio
```

as a stereo recording device.

It can then be selected as an input by normal audio applications.

The project has been exercised with Windows audio software including applications such as Audacity, AudioMulch and MuLab.

For older MuLab versions, audio input is selected through **Record Audio Setup** from the record control rather than by constructing an input device manually inside the rack.

No Amiga-side AHI driver is involved.

The Amiga application continues to program Paula normally.

---

## What makes this interesting

From the Amiga application's point of view, nothing special is happening.

A game, demo, tracker or old application still talks to Paula.

From the PC's point of view, however, the entire Amiga becomes a USB audio source.

That opens several useful possibilities:

- lossless digital recording of Amiga sessions;
- direct capture into a DAW;
- live streaming without an analog audio interface;
- software analysis of Paula output;
- modern host-side effects and processing;
- easy A/B comparison between different Paula reconstruction algorithms;
- future integration with VST or other host processing chains;
- synchronized capture alongside other digital PiStorm data paths.

It is particularly attractive because the feature is external to Amiga software compatibility: old software does not need to know that USB audio exists.

---

## Important distinction

This output should not be confused with capturing the physical analog waveform produced by a real Paula chip and the Amiga analog output stage.

The current path is:

```text
Paula programming activity
        |
        v
digital reconstruction in Emu68
        |
        v
USB PCM
```

A hardware bus-snooping solution such as a future Framethrower-based capture path would occupy a different point in the chain and may be useful when the goal is to observe activity directly from original Amiga hardware.

Both approaches are interesting, but they answer slightly different questions.

This project is primarily about making **PiStorm/Emu68 itself a clean, low-friction digital bridge between Paula-era Amiga software and modern USB audio hosts**.

---

## Current milestone status

### Hardware validated

- Paula reconstruction active on CPU1.
- DWC2 USB device stack active on CPU3.
- UAC1 enumeration successful.
- Windows recognizes `PiStorm Paula Audio`.
- 48 kHz / 16-bit / stereo PCM format accepted.
- Full-Speed isochronous transport operational.
- 1 ms audio scheduling operational.
- Valid audio/test-tone transfer confirmed.
- Real Paula-generated audio capture operational.
- Stream teardown/restart implemented.
- Re-opening the USB stream no longer relies solely on a software enable flag.
- Existing CACHE-FIX2 Paula recovery path preserved.

### Experimental

This remains research firmware rather than a polished upstream Emu68 feature.

In particular:

- the DWC2 UAC1 implementation is custom and intentionally minimal;
- High-Speed isochronous streaming is not part of the validated path;
- host compatibility has not been exhaustively tested;
- the current design exposes one fixed PCM format;
- there is no USB playback direction;
- there is no USB mixer/control interface beyond the required streaming controls;
- latency has not yet been treated as a formal low-latency audio-interface target;
- professional ASIO behaviour should not be assumed simply because Windows applications can capture the device.

---

## Source layout

The milestone contains only the files relevant to the experimental branch.

```text
include/
├── paula_probe.h
└── usb_audio_poc.h

src/
├── ExecutionLoop.c
├── aarch64/
│   ├── start.c
│   └── vectors.c
├── pistorm/
│   └── ps_classic_protocol.c
└── raspi/
    ├── start_rpi64.c
    ├── support_rpi.c
    └── usb_audio_poc.c

CMakeLists.txt
README-MILESTONE.md
```

The Paula implementation itself is integrated through the existing Emu68 Paula sources selected by the PiStorm Classic build.

---

## Build target

The code is intended for the **PiStorm Classic** Raspberry Pi target.

The experimental audio source is enabled only for the relevant `PISTORM_CLASSIC` build path.

The USB audio implementation is compiled into the `raspi64` target, while the Paula renderer and diagnostics are included for the PiStorm Classic variant.

---

## Base revision

This milestone was developed from:

```text
Emu68 commit: 9b4379a
```

The milestone archive name is:

```text
Emu68-9b4379a-PAULA-USB-AUDIO-UAC1-FS1MS-REENTRANT-MILESTONE
```

Keeping the upstream Emu68 revision in the milestone name is intentional: these experiments modify low-level multicore and Raspberry Pi device behaviour, so reproducibility depends on knowing the exact upstream baseline.

---

## Future directions

The current milestone proves the fundamental architecture.

Natural next steps include:

- broader host and DAW compatibility testing;
- further reduction and measurement of end-to-end latency;
- cleaner USB reset and suspend/resume handling;
- optional additional sample formats;
- host-visible gain or mute controls;
- tighter integration with PaulaMixer;
- formal USB-side diagnostics;
- interrupt/event-driven USB servicing where advantageous;
- comparison against physical Paula/bus-snoop captures;
- optional host-side DSP/VST processing;
- simultaneous digital video and audio experiments;
- eventual cleanup into a small, reviewable Emu68 feature set.

---

## Credits and upstream

This is an **unofficial experimental Emu68 / PiStorm extension**.

It is not an official Emu68 release.

### Paula emulator

The Paula emulation core used by this project is based on the **MIT-licensed Amiga Paula 8364 emulator by Akustikrausch / Andreas Wendorf**:

```text
https://github.com/akustikrausch/amiga-paula-8364-emulator
```

This project would not have its software-generated Paula audio path without that original emulator work. The Emu68-specific code adds the live hardware-driven register/DMA mirror, Chip RAM sample access and caching, replay/recovery machinery, multicore integration, mixer controls and USB Audio transport around that core.

### Emu68 and PiStorm

The work also builds on the Emu68 project and the wider PiStorm ecosystem. The USB transport experiment derives from earlier Emu68 DWC2/UVC work, while the live audio integration was developed for the PiStorm Classic environment.

Upstream Emu68:

```text
https://github.com/michalsc/Emu68
```

---

## License

Individual source files retain their existing licenses.

The new USB audio implementation identifies itself as:

```text
SPDX-License-Identifier: MPL-2.0
```

Check the corresponding upstream Emu68 files and repository licensing terms when redistributing complete builds or modified source trees.
