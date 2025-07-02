#include <fcntl.h>
#include <sys/ioctl.h>

#include "base/command_line.h"
#include "base/metrics/histogram_macros.h"
#include "base/memory/ptr_util.h"

#include "media/audio/audioio/audio_manager_audioio.h"

#include "media/audio/audio_device_description.h"
#include "media/audio/audio_output_dispatcher.h"
#include "media/audio/audioio/audioio_input.h"
#include "media/audio/audioio/audioio_output.h"
#include "media/audio/fake_audio_manager.h"
#include "media/base/limits.h"
#include "media/base/media_switches.h"

namespace media {

// Maximum number of output streams that can be open simultaneously.
static const int kMaxOutputStreamsAIO = 50;

// Default sample rate for input and output streams.
static const int kDefaultSampleRate = 48000;

bool AudioManagerAudioIO::HasAudioOutputDevices() {
  AudioDeviceNames devices;
  GetAudioOutputDeviceNames(&devices);
  if (devices.empty()) {
    LOG(WARNING) << "[AUDIOIO] No audio output devices found.";
  }
  return !devices.empty();
}

bool AudioManagerAudioIO::HasAudioInputDevices() {
  AudioDeviceNames devices;
  GetAudioInputDeviceNames(&devices);
  if (devices.empty()) {
    LOG(WARNING) << "[AUDIOIO] No audio input devices found.";
  }
  return !devices.empty();
}

void AudioManagerAudioIO::GetAudioInputDeviceNames(
    AudioDeviceNames* device_names) {
  GetAudioDevices(INPUT);
  *device_names = *input_devices;
}

void AudioManagerAudioIO::GetAudioOutputDeviceNames(
    AudioDeviceNames* device_names) {
  GetAudioDevices(OUTPUT);
  *device_names = *output_devices;
}

void AudioManagerAudioIO::GetAudioDevices(int type) {
  char dev[16];
  int fd;
  int hwprops;
  struct audio_device hwname;
  struct audio_info info;

  if (type == INPUT) {
    if (!input_devices) {
      input_devices = new AudioDeviceNames();
    } else {
      return;
    }
  } else {
    if (!output_devices) {
      output_devices = new AudioDeviceNames();
    } else {
      return;
    }
  }

  for (int i=0; i < 4; i++) {
    (void)snprintf(dev, sizeof(dev), "/dev/audio%u", i);
    if ((fd = open(dev, O_RDONLY)) < 0) {
      continue;
    }
    if ((ioctl(fd, AUDIO_GETPROPS, &hwprops) == 0) &&
        (ioctl(fd, AUDIO_GETDEV, &hwname) == 0)) {
      if (type == INPUT) {
        if ((hwprops & AUDIO_PROP_CAPTURE) != 0) {
          LOG(INFO) << "[AUDIOIO] Found input device: " << hwname.name << ", " << dev;
          input_devices->push_back(AudioDeviceName(hwname.name, dev));
        } else if ((ioctl(fd, AUDIO_GETINFO, &info) == 0) && (info.mode == AUMODE_RECORD)) {
          LOG(INFO) << "[AUDIOIO] Found input device: " << hwname.name << ", " << dev;
          input_devices->push_back(AudioDeviceName(hwname.name, dev));
        }
      }
      if ((type == OUTPUT) && ((hwprops & AUDIO_PROP_PLAYBACK) != 0)) {
        LOG(INFO) << "[AUDIOIO] Found output device: " << hwname.name << ", " << dev;
        output_devices->push_back(AudioDeviceName(hwname.name, dev));
      }
    }
    close(fd);
  }

  // Prepend the default device if the list is not empty.
  if ((type == INPUT) && (!input_devices->empty())) {
    input_devices->push_front(AudioDeviceName::CreateDefault());
  }
  if ((type == OUTPUT) && (!output_devices->empty())) {
    output_devices->push_front(AudioDeviceName::CreateDefault());
  }
}

const std::string_view AudioManagerAudioIO::GetName() {
  return "AudioIO";
}

AudioParameters AudioManagerAudioIO::GetInputStreamParameters(
    const std::string& device_id) {
  static const int kDefaultInputBufferSize = 1024;

  int user_buffer_size = GetUserBufferSize();
  int buffer_size = user_buffer_size ?
      user_buffer_size : kDefaultInputBufferSize;

  return AudioParameters(
      AudioParameters::AUDIO_PCM_LOW_LATENCY, ChannelLayoutConfig::Stereo(),
      kDefaultSampleRate, buffer_size);
}

AudioManagerAudioIO::AudioManagerAudioIO(std::unique_ptr<AudioThread> audio_thread,
                                         AudioLogFactory* audio_log_factory)
    : AudioManagerBase(std::move(audio_thread),
                       audio_log_factory) {
  LOG(INFO) << "[AUDIOIO] AudioManagerAudioIO";
  SetMaxOutputStreamsAllowed(kMaxOutputStreamsAIO);
}

AudioManagerAudioIO::~AudioManagerAudioIO() = default;

AudioOutputStream* AudioManagerAudioIO::MakeLinearOutputStream(
    const AudioParameters& params,
    const LogCallback& log_callback) {
  DCHECK_EQ(AudioParameters::AUDIO_PCM_LINEAR, params.format());
  return MakeOutputStream(AudioDeviceDescription::kDefaultDeviceId, params);
}

AudioOutputStream* AudioManagerAudioIO::MakeLowLatencyOutputStream(
    const AudioParameters& params,
    const std::string& device_id,
    const LogCallback& log_callback) {
  LOG_IF(ERROR, !device_id.empty()) << "[AUDIOIO] MakeLowLatencyOutputStream: Not implemented!";
  DCHECK_EQ(AudioParameters::AUDIO_PCM_LOW_LATENCY, params.format());
  return MakeOutputStream(device_id, params);
}

AudioInputStream* AudioManagerAudioIO::MakeLinearInputStream(
    const AudioParameters& params,
    const std::string& device_id,
    const LogCallback& log_callback) {
  DCHECK_EQ(AudioParameters::AUDIO_PCM_LINEAR, params.format());
  return MakeInputStream(device_id, params);
}

AudioInputStream* AudioManagerAudioIO::MakeLowLatencyInputStream(
    const AudioParameters& params,
    const std::string& device_id,
    const LogCallback& log_callback) {
  DCHECK_EQ(AudioParameters::AUDIO_PCM_LOW_LATENCY, params.format());
  return MakeInputStream(device_id, params);
}

AudioParameters AudioManagerAudioIO::GetPreferredOutputStreamParameters(
    const std::string& output_device_id,
    const AudioParameters& input_params) {
  LOG_IF(ERROR, !output_device_id.empty()) << "[AUDIOIO] GetPreferredOutputStreamParameters: output_device_id is empty!";
  static const int kDefaultOutputBufferSize = 2048;

  ChannelLayoutConfig channel_layout_config = ChannelLayoutConfig::Stereo();
  int sample_rate = kDefaultSampleRate;
  int buffer_size = kDefaultOutputBufferSize;
  if (input_params.IsValid()) {
    sample_rate = input_params.sample_rate();
    channel_layout_config = input_params.channel_layout_config();
    buffer_size = std::min(buffer_size, input_params.frames_per_buffer());
  }

  int user_buffer_size = GetUserBufferSize();
  if (user_buffer_size)
    buffer_size = user_buffer_size;

  return AudioParameters(
      AudioParameters::AUDIO_PCM_LOW_LATENCY,
      channel_layout_config, sample_rate, buffer_size);
}

AudioInputStream* AudioManagerAudioIO::MakeInputStream(
    const std::string& device_id,
    const AudioParameters& params) {
  LOG(INFO) << "[AUDIOIO] MakeInputStream";
  return new AudioIOAudioInputStream(this, device_id, params);
}

AudioOutputStream* AudioManagerAudioIO::MakeOutputStream(
    const std::string& device_id,
    const AudioParameters& params) {
  LOG(INFO) << "[AUDIOIO] MakeOutputStream";
  return new AudioIOAudioOutputStream(device_id, params, this);
}

std::unique_ptr<media::AudioManager> CreateAudioManager(
    std::unique_ptr<AudioThread> audio_thread,
    AudioLogFactory* audio_log_factory) {
  LOG(INFO) << "[AUDIOIO] CreateAudioManager";

  // For testing allow audio output to be disabled.
  if (base::CommandLine::ForCurrentProcess()->HasSwitch(switches::kDisableAudioOutput)) {
    return std::make_unique<FakeAudioManager>(std::move(audio_thread),
                                              audio_log_factory);
  }

  return std::make_unique<AudioManagerAudioIO>(std::move(audio_thread),
                                              audio_log_factory);
}

}  // namespace media
