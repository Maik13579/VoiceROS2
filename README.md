# VoiceROS2
WIP


## Dependencies
- Docker

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