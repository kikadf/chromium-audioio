#include <fcntl.h>
#include <sys/ioctl.h>

#include "base/logging.h"
#include "base/time/time.h"
#include "base/time/default_tick_clock.h"
#include "media/audio/audio_manager_base.h"
#include "media/base/audio_timestamp_helper.h"
#include "media/audio/audioio/audioio_output.h"

namespace media {

static const SampleFormat kSampleFormatAIOo = kSampleFormatS16;

void *AudioIOAudioOutputStream::ThreadEntry(void *arg) {
  AudioIOAudioOutputStream* self = static_cast<AudioIOAudioOutputStream*>(arg);

  self->ThreadLoop();
  return NULL;
}

AudioIOAudioOutputStream::AudioIOAudioOutputStream(const std::string& device_name,
                                               const AudioParameters& params,
                                               AudioManagerBase* manager)
    : manager(manager),
      params(params),
      device_name(device_name),
      audio_bus(AudioBus::Create(params)),
      state(kClosed),
      mutex(PTHREAD_MUTEX_INITIALIZER) {
}

AudioIOAudioOutputStream::~AudioIOAudioOutputStream() {
  if (state != kClosed) {
    Close();
  }
}

// Open the stream. false is returned if the stream cannot be opened.  Open()
// must always be followed by a call to Close() even if Open() fails
bool AudioIOAudioOutputStream::Open() {
  struct audio_info info;
  std::string device;

  if (params.format() != AudioParameters::AUDIO_PCM_LINEAR &&
      params.format() != AudioParameters::AUDIO_PCM_LOW_LATENCY) {
    LOG(WARNING) << "[AUDIOIO] Output: Unsupported audio format.";
    return false;
  }

  if ((device_name.empty()) || (device_name == AudioDeviceDescription::kDefaultDeviceId)) {
    device = "/dev/audio";
  } else {
    device = device_name;
  }

  AUDIO_INITINFO(&info);
  info.mode = AUMODE_PLAY;
  info.play.sample_rate = params.sample_rate();
  info.play.channels = params.channels();
  info.play.precision = SampleFormatToBitsPerChannel(kSampleFormatAIOo);
  info.play.encoding = AUDIO_ENCODING_SLINEAR_LE;
  info.play.pause = true;

  if ((fd = open(device.c_str(), O_WRONLY)) < 0) {
    LOG(ERROR) << "[AUDIOIO] Output: Couldn't open audio device: " << device;
    return false;
  }

  if (ioctl(fd, AUDIO_SETINFO, &info) < 0) {
    goto error;
  }

  if (ioctl(fd, AUDIO_GETINFO, &info) < 0) {
    goto error;
  }

  state = kStopped;
  volpending = 0;
  vol = AUDIO_MAX_GAIN;
  buffer = new char[audio_bus->frames() * params.GetBytesPerFrame(kSampleFormatAIOo)];
  LOG(INFO) << "[AUDIOIO] OutputStream opened: " << device;
  return true;

error:
  close(fd);
  LOG(ERROR) << "[AUDIOIO] Output: Couldn't set audio parameters.";
  return false;
}

// Close the stream.
// After calling this method, the object should not be used anymore.
// After calling this method, no further AudioSourceCallback methods
// should be called on the callback object that was supplied to Start()
// by the AudioOutputStream implementation.
void AudioIOAudioOutputStream::Close() {
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
  manager->ReleaseOutputStream(this);  // Calls the destructor
  LOG(INFO) << "[AUDIOIO] OutputStream closed.";
}

// Starts playing audio and generating AudioSourceCallback::OnMoreData().
// Since implementor of AudioOutputStream may have internal buffers, right
// after calling this method initial buffers are fetched.
//
// The output stream does not take ownership of this callback.
void AudioIOAudioOutputStream::Start(AudioSourceCallback* callback) {
  struct audio_info info;

  (void)ioctl(fd, AUDIO_FLUSH, NULL);
  if (ioctl(fd, AUDIO_GETINFO, &info) < 0) {
    goto error;
  }
  info.play.pause = false;
  if (ioctl(fd, AUDIO_SETINFO, &info) < 0 ) {
    goto error;
  }

  state = kRunning;
  source = callback;

  if (pthread_create(&thread, NULL, &ThreadEntry, this) != 0) {
    LOG(ERROR) << "[AUDIOIO] Failed to create real-time thread for playing.";
    goto error;
  }

  LOG(INFO) << "[AUDIOIO] Playing started.";
  return;
error:
  LOG(ERROR) << "[AUDIOIO] Failed to start playing audio.";
  state = kStopped;
}

// Stops playing audio.  The operation completes synchronously meaning that
// once Stop() has completed executing, no further callbacks will be made to
// the callback object that was supplied to Start() and it can be safely
// deleted. Stop() may be called in any state, e.g. before Start() or after
// Stop().
void AudioIOAudioOutputStream::Stop() {
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
  info.play.pause = true;
  if (ioctl(fd, AUDIO_SETINFO, &info) < 0 ) {
    goto error;
  }
  state = kStopped;
  LOG(INFO) << "[AUDIOIO] Playing stopped.";
  return;
error:
  LOG(ERROR) << "[AUDIOIO] Failed to stop playing audio.";
  return;
}

// Sets the relative volume, with range [0.0, 1.0] inclusive.
void AudioIOAudioOutputStream::SetVolume(double v) {
  pthread_mutex_lock(&mutex);
  vol = v * AUDIO_MAX_GAIN;
  volpending = 1;
  pthread_mutex_unlock(&mutex);
}

// Gets the relative volume, with range [0.0, 1.0] inclusive.
void AudioIOAudioOutputStream::GetVolume(double* v) {
  pthread_mutex_lock(&mutex);
  *v = vol * (1. / AUDIO_MAX_GAIN);
  pthread_mutex_unlock(&mutex);
}

// Flushes the stream. This should only be called if the stream is not
// playing. (i.e. called after Stop or Open)

// This stream is always used with sub second buffer sizes, where it's
// sufficient to simply always flush upon Start().
void AudioIOAudioOutputStream::Flush() {}

void AudioIOAudioOutputStream::ThreadLoop(void) {
  int avail, count, ret, frames, move;
  int written_bytes = 0;
  int framesize = params.GetBytesPerFrame(kSampleFormatAIOo);
  int hw_delay = 0;
  struct audio_info info;
  struct audio_offset offset;

  LOG(INFO) << "[AUDIOIO] Output:ThreadLoop() started.";

  while (state == kRunning) {
    // Update volume if needed
    pthread_mutex_lock(&mutex);
    if (volpending) {
      volpending = 0;
      if (ioctl(fd, AUDIO_GETINFO, &info) < 0) {
        LOG(ERROR) << "[AUDIOIO] Output:ThreadLoop(): Failed to get audio info.";
      } else {
        info.play.gain = vol;
        if (ioctl(fd, AUDIO_SETINFO, &info) < 0 ) {
          LOG(ERROR) << "[AUDIOIO] Output:ThreadLoop(): Failed to set audio volume.";
        }
      }
    }
    pthread_mutex_unlock(&mutex);

    // Get data to play
    const base::TimeDelta delay = AudioTimestampHelper::FramesToTime(hw_delay, params.sample_rate());
    count = source->OnMoreData(delay, base::TimeTicks::Now(), {}, audio_bus.get());
    audio_bus->ToInterleaved<SignedInt16SampleTypeTraits>(count, reinterpret_cast<int16_t*>(buffer));
    if (count == 0) {
      // We have to submit something to the device
      count = audio_bus->frames();
      memset(buffer, 0, count * framesize);
      LOG(WARNING) << "[AUDIOIO] Output:ThreadLoop(): No data to play, running empty cycle.";
    }

    // Submit data to the device
    move = 0;
    while (count > 0) {
      avail = count * framesize;
      if ((ret = write(fd, buffer + move, avail)) < 0) {
        if (errno == EINTR) {
          continue;
        }
        LOG(ERROR) << "[AUDIOIO] Output:ThreadLoop(): write error.";
        break;
      }
      frames = ret / framesize;
      move += ret;
      written_bytes += ret;
      count -= frames;
    }

    // Update hardware pointer
    if (ioctl(fd, AUDIO_GETOOFFS, &offset) < 0) {
        LOG(ERROR) << "[AUDIOIO] Output:ThreadLoop(): Failed to get transfered bytes.";
        break;
    } else {
      hw_delay = (written_bytes - offset.samples) / framesize;
    }
  }
  LOG(INFO) << "[AUDIOIO] Output:ThreadLoop() stopped.";
}

}  // namespace media
