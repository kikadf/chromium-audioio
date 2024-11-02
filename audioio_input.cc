#include <fcntl.h>
#include <sys/ioctl.h>

#include "base/logging.h"
#include "media/base/audio_timestamp_helper.h"
#include "media/audio/audioio/audio_manager_audioio.h"
#include "media/audio/audio_manager.h"
#include "media/audio/audioio/audioio_input.h"

namespace media {

static const SampleFormat kSampleFormat = kSampleFormatS16;

void *AudioIOAudioInputStream::ThreadEntry(void *arg) {
  AudioIOAudioInputStream* self = static_cast<AudioIOAudioInputStream*>(arg);

  self->ThreadLoop();
  return NULL;
}

AudioIOAudioInputStream::AudioIOAudioInputStream(AudioManagerBase* manager,
                                             const std::string& device_name,
                                             const AudioParameters& params)
    : manager(manager),
      params(params),
      audio_bus(AudioBus::Create(params)),
      state(kClosed),
      mutex(PTHREAD_MUTEX_INITIALIZER) {
}

AudioIOAudioInputStream::~AudioIOAudioInputStream() {
  if (state != kClosed)
    Close();
}

// Open the stream and prepares it for recording. Call Start() to actually
// begin recording.
AudioInputStream::OpenOutcome AudioIOAudioInputStream::Open() {
  struct audio_info info;

  if (state != kClosed) {
    return OpenOutcome::kFailed;
  }

  if (params.format() != AudioParameters::AUDIO_PCM_LINEAR &&
      params.format() != AudioParameters::AUDIO_PCM_LOW_LATENCY) {
    LOG(WARNING) << "[AUDIOIO] Input: Unsupported audio format.";
    return OpenOutcome::kFailed;
  }

  AUDIO_INITINFO(&info);
  info.mode = AUMODE_RECORD;
  info.record.sample_rate = params.sample_rate();
  info.record.channels = params.channels();
  info.record.precision = SampleFormatToBitsPerChannel(kSampleFormat);
  info.record.encoding = AUDIO_ENCODING_SLINEAR_LE;
  info.record.pause = true;

  if ((fd = open("/dev/audio", O_RDONLY)) < 0) {
    LOG(ERROR) << "[AUDIOIO] Input: Couldn't open audio device.";
    return OpenOutcome::kFailed;
  }

  if (ioctl(fd, AUDIO_SETINFO, &info) < 0) {
    goto error;
  }

  if (ioctl(fd, AUDIO_GETINFO, &info) < 0) {
    goto error;
  }

  state = kStopped;
  inputvol = 1.0;
  vol = AUDIO_MAX_GAIN;
  buffer = new char[audio_bus->frames() * params.GetBytesPerFrame(kSampleFormat)];
  LOG(INFO) << "[AUDIOIO] InputStream opened.";
  return OpenOutcome::kSuccess;
error:
  close(fd);
  LOG(ERROR) << "[AUDIOIO] Input: Couldn't set audio parameters.";
  return OpenOutcome::kFailed;
}

// Starts recording audio and generating AudioInputCallback::OnData().
// The input stream does not take ownership of this callback.
void AudioIOAudioInputStream::Start(AudioInputCallback* cb) {
  struct audio_info info;

  StartAgc();

  (void)ioctl(fd, AUDIO_FLUSH, NULL);
  if (ioctl(fd, AUDIO_GETINFO, &info) < 0) {
    goto error;
  }
  info.record.pause = false;
  if (ioctl(fd, AUDIO_SETINFO, &info) < 0 ) {
    goto error;
  }

  state = kRunning;
  callback = cb;

  if (pthread_create(&thread, NULL, &ThreadEntry, this) != 0) {
    LOG(ERROR) << "[AUDIOIO] Failed to create real-time thread for recording.";
    goto error;
  }

  LOG(INFO) << "[AUDIOIO] Recording started.";
  return;
error:
  LOG(ERROR) << "[AUDIOIO] Failed to start recording audio.";
  state = kStopped;
}

// Stops recording audio. Effect might not be instantaneous as there could be
// pending audio callbacks in the queue which will be issued first before
// recording stops.
void AudioIOAudioInputStream::Stop() {
  struct audio_info info;

  if (state == kStopped) {
    return;
  }
  state = kStopWait;
  pthread_join(thread, NULL);
  (void)ioctl(fd, AUDIO_FLUSH, NULL);
  if (ioctl(fd, AUDIO_GETINFO, &info) < 0) {
    goto error;
  }
  info.record.pause = true;
  if (ioctl(fd, AUDIO_SETINFO, &info) < 0 ) {
    goto error;
  }
  state = kStopped;
  StopAgc();
  LOG(INFO) << "[AUDIOIO] Recording stopped.";
  return;
error:
  LOG(ERROR) << "[AUDIOIO] Failed to stop recording audio.";
  return;
}

// Close the stream. This also generates AudioInputCallback::OnClose(). This
// should be the last call made on this object.
void AudioIOAudioInputStream::Close() {
  if (state == kClosed) {
    goto release;
  }
  if (state == kRunning) {
    Stop();
  }
  (void)ioctl(fd, AUDIO_FLUSH, NULL);
  close(fd);
  state = kClosed;
  delete [] buffer;

release:
  manager->ReleaseInputStream(this);  // Calls the destructor
  LOG(INFO) << "[AUDIOIO] InputStream closed.";
}

// Returns the maximum microphone analog volume or 0.0 if device does not
// have volume control.
double AudioIOAudioInputStream::GetMaxVolume() {
  return 1.0;
}

// Sets the microphone analog volume, with range [0, max_volume] inclusive.
void AudioIOAudioInputStream::SetVolume(double volume) {
  struct audio_info info;

  pthread_mutex_lock(&mutex);
  vol = volume * AUDIO_MAX_GAIN;
  inputvol = volume;
  if (ioctl(fd, AUDIO_GETINFO, &info) < 0) {
    LOG(ERROR) << "[AUDIOIO] Input:SetVolume(): Failed to get audio info.";
  } else {
    info.record.gain = vol;
    if (ioctl(fd, AUDIO_SETINFO, &info) < 0 ) {
      LOG(ERROR) << "[AUDIOIO] Input:SetVolume(): Failed to set audio volume.";
    }
  }
  pthread_mutex_unlock(&mutex);

  UpdateAgcVolume();
}

// Returns the microphone analog volume, with range [0, max_volume] inclusive.
double AudioIOAudioInputStream::GetVolume() {
  return inputvol;
}

// Returns the current muting state for the microphone.
bool AudioIOAudioInputStream::IsMuted() {
  // Not supported.
  return false;
}

// Sets the output device from which to cancel echo, if echo cancellation is
// supported by this stream. E.g. called by WebRTC when it changes playback
// devices.
void AudioIOAudioInputStream::SetOutputDeviceForAec(
    const std::string& output_device_id) {
  // Not supported.
}

void AudioIOAudioInputStream::ThreadLoop(void) {
  size_t bytes, n, frames, move, count;
  size_t framesize = params.GetBytesPerFrame(kSampleFormat);
  size_t read_bytes = 0;
  size_t hw_delay = 0;
  double normalized_volume = 0.0;
  char *data;
  struct audio_offset offset;

  LOG(INFO) << "[AUDIOIO] Input:ThreadLoop() started.";

  while (state == kRunning) {

    GetAgcVolume(&normalized_volume);
    count = audio_bus->frames();

    data = buffer;
    move = 0;
    while (count > 0) {
      bytes = count * framesize;
      if ((n = read(fd, data + move, bytes)) < 0) {
        LOG(ERROR) << "[AUDIOIO] Input:ThreadLoop(): read error.";
        break;
      }
      frames = n / framesize;
      move += n;
      read_bytes += n;
      count -= frames;
    }

    // Update hardware pointer
    if (ioctl(fd, AUDIO_GETIOFFS, &offset) < 0) {
        LOG(ERROR) << "[AUDIOIO] Intput:ThreadLoop(): Failed to get transfered bytes.";
        break;
    } else {
      hw_delay = (read_bytes - offset.samples) / framesize;
    }
    // convert frames count to TimeDelta
    const base::TimeDelta delay = AudioTimestampHelper::FramesToTime(hw_delay, params.sample_rate());

    // push into bus
    audio_bus->FromInterleaved<SignedInt16SampleTypeTraits>(reinterpret_cast<int16_t*>(buffer), count);

    // invoke callback
    callback->OnData(audio_bus.get(), base::TimeTicks::Now() - delay, 1., {});
  }
  LOG(INFO) << "[AUDIOIO] Input:ThreadLoop() stopped.";
}

}  // namespace media
