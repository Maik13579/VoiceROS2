# VoiceROS2

A ROS 2 workspace integrating both speech-to-text (STT) and text-to-speech (TTS) capabilities using [Vosk](https://github.com/alphacep/vosk-api) and [Coqui TTS](https://github.com/coqui-ai/TTS).  
This repository combines the two submodules:
- [`vosk_ros2`](https://github.com/Maik13579/vosk_ros2) – for speech recognition
- [`coqui_tts_ros2`](https://github.com/Maik13579/coqui_tts_ros2) – for text-to-speech synthesis

The `voice_ros2` package contains launch files and parameters to start both nodes from a single entry point.

## TTS and STT Coordination

The `tts_state` topic from the Coqui TTS node is **remapped and connected to the Vosk STT node**.  
This enables automatic coordination between speaking and listening:

- When the robot **starts speaking**, `tts_state` is set to `True` → STT is paused.
- When the robot **finishes speaking**, `tts_state` is set to `False` → STT resumes.

This allows seamless interaction, e.g., when asking a question:
You can send both the **TTS** and **STT** actions at the same time.  
The robot will first speak, then automatically start listening once it’s done — no need to estimate or wait for a fixed speech duration.


## Dependencies
- Docker
- ALSA (for audio input/output)
- Optional: `nvidia-docker` (for GPU acceleration with Coqui TTS)

## Build
```bash
git submodule update --init --recursive
docker/build.sh
```

## Usage
Change ALSA_CARD in docker/compose.yaml, use aplay -l to find the card name.
```bash
docker compose -f docker/compose.yaml up
```

## Submodules

- [`vosk_ros2`](https://github.com/Maik13579/vosk_ros2): ROS2 wrapper for Vosk STT with grammar support, speaker recognition, and action/service interface.  
  → See its README for details.

- [`coqui_tts_ros2`](https://github.com/Maik13579/coqui_tts_ros2): ROS2 wrapper for Coqui TTS with multi-speaker/emotion support and a ROS2 action interface.  
  → See its README for details.

## Launching

The `voice_ros2` package contains launch files and parameter folders to start both components together.  
You can modify and extend these to suit your use case.

```bash
ros2 launch voice_ros2 voice_ros2.launch.py
```