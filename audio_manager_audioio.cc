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
static const int kMaxOutputStreams = 50;

// Default sample rate for input and output streams.
static const int kDefaultSampleRate = 48000;

void AddDefaultDevice(AudioDeviceNames* device_names) {
  DCHECK(device_names->empty());
  device_names->push_front(AudioDeviceName::CreateDefault());
}

bool AudioManagerAudioIO::HasAudioOutputDevices() {
  return true;
}

bool AudioManagerAudioIO::HasAudioInputDevices() {
  return true;
}

void AudioManagerAudioIO::GetAudioInputDeviceNames(
    AudioDeviceNames* device_names) {
  DCHECK(device_names->empty());
  AddDefaultDevice(device_names);
}

void AudioManagerAudioIO::GetAudioOutputDeviceNames(
    AudioDeviceNames* device_names) {
  AddDefaultDevice(device_names);
}

const char* AudioManagerAudioIO::GetName() {
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
  SetMaxOutputStreamsAllowed(kMaxOutputStreams);
}

AudioManagerAudioIO::~AudioManagerAudioIO() = default;

AudioOutputStream* AudioManagerAudioIO::MakeLinearOutputStream(
    const AudioParameters& params,
    const LogCallback& log_callback) {
  DCHECK_EQ(AudioParameters::AUDIO_PCM_LINEAR, params.format());
  return MakeOutputStream(params);
}

AudioOutputStream* AudioManagerAudioIO::MakeLowLatencyOutputStream(
    const AudioParameters& params,
    const std::string& device_id,
    const LogCallback& log_callback) {
  LOG_IF(ERROR, !device_id.empty()) << "[AUDIOIO] MakeLowLatencyOutputStream: Not implemented!";
  DCHECK_EQ(AudioParameters::AUDIO_PCM_LOW_LATENCY, params.format());
  return MakeOutputStream(params);
}

AudioInputStream* AudioManagerAudioIO::MakeLinearInputStream(
    const AudioParameters& params,
    const std::string& device_id,
    const LogCallback& log_callback) {
  DCHECK_EQ(AudioParameters::AUDIO_PCM_LINEAR, params.format());
  return MakeInputStream(params);
}

AudioInputStream* AudioManagerAudioIO::MakeLowLatencyInputStream(
    const AudioParameters& params,
    const std::string& device_id,
    const LogCallback& log_callback) {
  DCHECK_EQ(AudioParameters::AUDIO_PCM_LOW_LATENCY, params.format());
  return MakeInputStream(params);
}

AudioParameters AudioManagerAudioIO::GetPreferredOutputStreamParameters(
    const std::string& output_device_id,
    const AudioParameters& input_params) {
  // TODO(tommi): Support |output_device_id|.
  LOG_IF(ERROR, !output_device_id.empty()) << "[AUDIOIO] GetPreferredOutputStreamParameters: Not implemented!";
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
    const AudioParameters& params) {
  LOG(INFO) << "[AUDIOIO] MakeInputStream";
  return new AudioIOAudioInputStream(this,
             AudioDeviceDescription::kDefaultDeviceId, params);
}

AudioOutputStream* AudioManagerAudioIO::MakeOutputStream(
    const AudioParameters& params) {
  LOG(INFO) << "[AUDIOIO] MakeOutputStream";
  return new AudioIOAudioOutputStream(params, this);
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