# chromium-audioio
NetBSD audio backend for chromium.
Port from sndio backend of [Robert Nagy](https://github.com/rnagy), used works of [Nia Alarie](https://www.netbsd.org/~nia/) on cubeb and RetroArch.

To use chromium-audioio deploy the files to the chromium source's
media/audio/audioio folder and add use_audioio=true to GN_ARGS.

