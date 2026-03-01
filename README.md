# LI-FI // OWC TERMINAL

### Optical Wireless Communication via On-Off Keying (OOK)

> **Hackenza 2026 Submission**

---

## What is it?

LI-FI OWC Terminal is a browser-based **Light Fidelity (Li-Fi)** communication system that transmits text messages wirelessly using nothing but a **flashlight and a camera**. No Wi-Fi. No Bluetooth. No cables. Just light.

The system encodes text into binary using ASCII, then modulates it as rapid on/off flashes of the device flashlight (On-Off Keying). The receiver device uses its camera to decode the light pulses back into text — in real time.

---

## Live Demo

**Deployed on GitHub Pages:**
[https://parthjhalani07.github.io/Hackenza1/](https://parthjhalani07.github.io/Hackenza1/)

> Open the link on **two separate devices** — one as the Sender and one as the Receiver.

---

## Features

| Feature                    | Description                                                                                          |
| -------------------------- | ---------------------------------------------------------------------------------------------------- |
| **Flashlight Transmitter** | Uses device torch/flashlight to transmit binary-encoded messages via light pulses                    |
| **Camera Receiver**        | Decodes incoming light pulses using the device camera and real-time brightness analysis              |
| **Speech to Text**         | Dictate your message hands-free using the built-in microphone button                                 |
| **Text to Speech**         | Decoded messages are read aloud automatically (toggle Auto TTS) or on demand                         |
| **Send Any Message**       | Supports any ASCII text message up to 128 characters                                                 |
| **Quick-Send Prompts**     | Pre-built one-tap emergency/utility messages including **SOS** and **HELP** for instant transmission |
| **Live Decode Preview**    | Characters appear on the receiver screen as bits are received in real time                           |
| **Signal Calibration**     | Automatic ambient light calibration ensures reliable detection in any environment                    |
| **Oversampling Debugger**  | Advanced debug panel showing median brightness, threshold, bit confidence, and sample visualisation  |
| **Message History**        | Received messages are stored and timestamped in a scrollable history log                             |
| **Retro Terminal UI**      | Full CRT scanline aesthetic with boot animation, Orbitron font, and signal indicators                |

---

## Technical Specs

| Parameter              | Value                                                            |
| ---------------------- | ---------------------------------------------------------------- |
| Modulation             | On-Off Keying (OOK)                                              |
| Bit Rate               | 300 ms / bit                                                     |
| **Time per Character** | **~2.4 seconds** (8 bits × 300 ms)                               |
| Frame Structure        | 16-bit lead-in + 6-bit preamble + data + 8-bit postamble         |
| Preamble               | `101011`                                                         |
| Postamble              | `00100100` (ASCII `$`)                                           |
| Encoding               | 8-bit ASCII per character                                        |
| Max Message Length     | 128 characters                                                   |
| Detection Method       | Camera ROI brightness sampling with EMA ambient drift correction |
| Threshold Logic        | Dynamic ambient baseline + hysteresis band                       |

---

## How to Use

### Requirements

- Two devices with a **browser** (Chrome recommended)
- **Device 1:** Flashlight / torch (Sender)
- **Device 2:** Front or rear camera (Receiver)
- Camera permissions must be granted on the Receiver device

### Step-by-step

1. **Open the app** on both devices:
   👉 [https://parthjhalani07.github.io/Hackenza1/](https://parthjhalani07.github.io/Hackenza1/)

2. **On the Receiver device:**
   - Tap **"⟵ RECEIVER"** in the header
   - Tap **"START RECEIVER"** and grant camera permission
   - Wait for auto-calibration to complete (~2 seconds)
   - Point the camera towards the Sender's flashlight and align it inside the **target reticle box**

3. **On the Sender device:**
   - Stay on the **"⟶ SENDER"** panel
   - Type your message — or tap the 🎤 mic button to dictate it via voice
   - Alternatively, tap a **quick-send prompt** (SOS, HELP, etc.) for instant one-tap transmission
   - Tap **ENCODE** to convert to binary
   - Tap **TRANSMIT** — the flashlight will begin pulsing

4. **Watch the message appear** on the Receiver screen in real time as bits are decoded

5. **Audio playback:** Enable **AUTO TTS** or tap **🔊 SPEAK MESSAGE** to hear the decoded message read aloud

### Tips for best results

- Use in a **dimly lit room** for maximum contrast
- Keep devices **30–60 cm apart** and hold the flashlight steady within the reticle
- If reception is unreliable, tap **CALIBRATE** on the Receiver to re-measure ambient light
- Enable the **🔍 DEBUG** panel on the Receiver to monitor live brightness and bit decisions

---

## Stack

- **Pure HTML / CSS / JavaScript** — zero dependencies, zero build step
- Web APIs used: `MediaDevices` (camera), `ImageCapture` / `torch` (flashlight), `SpeechRecognition` (STT), `SpeechSynthesis` (TTS), `Canvas 2D`
- Deployed via **GitHub Pages**

---

## Protocol Overview

```
┌──────────────────────────────────────────────────────────┐
│  FRAME  =  LEAD-IN (16) + PREAMBLE (6) + DATA + POSTAMBLE (8)  │
└──────────────────────────────────────────────────────────┘
  LEAD-IN   : 1010101010101010   — warms up the receiver AGC
  PREAMBLE  : 101011             — signals start of payload
  DATA      : 8 bits per ASCII character
  POSTAMBLE : 00100100 ($)       — end-of-message marker
```

Each bit is transmitted as a **300 ms window**: flashlight ON = `1`, flashlight OFF = `0`. The receiver samples brightness at ~30 Hz within each window and takes the **median** to decide the bit value, making it robust against brief occlusions and noise.

---

## Team

Built for **Hackenza 2026**.