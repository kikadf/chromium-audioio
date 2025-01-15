#ifndef MEDIA_AUDIO_AUDIOIO_AUDIOIO_INPUT_H_
#define MEDIA_AUDIO_AUDIOIO_AUDIOIO_INPUT_H_

#include <stdint.h>
#include <string>
#include <sys/audioio.h>

#include "base/compiler_specific.h"
#include "base/memory/weak_ptr.h"
#include "base/time/time.h"
#include "media/audio/agc_audio_stream.h"
#include "media/audio/audio_io.h"
#include "media/audio/audio_device_description.h"
#include "media/base/audio_parameters.h"

namespace media {

class AudioManagerBase;

// Implementation of AudioOutputStream using audio(4)
class AudioIOAudioInputStream : public AgcAudioStream<AudioInputStream> {
 public:
  // Pass this to the constructor if you want to attempt auto-selection
  // of the audio recording device.
  static const char kAutoSelectDevice[];

  // Create a PCM Output stream for the SNDIO device identified by
  // |device_name|. If unsure of what to use for |device_name|, use
  // |kAutoSelectDevice|.
  AudioIOAudioInputStream(AudioManagerBase* audio_manager,
                     const std::string& device_name,
                     const AudioParameters& params);

  AudioIOAudioInputStream(const AudioIOAudioInputStream&) = delete;
  AudioIOAudioInputStream& operator=(const AudioIOAudioInputStream&) = delete;

  ~AudioIOAudioInputStream() override;

  // Implementation of AudioInputStream.
  OpenOutcome Open() override;
  void Start(AudioInputCallback* callback) override;
  void Stop() override;
  void Close() override;
  double GetMaxVolume() override;
  void SetVolume(double volume) override;
  double GetVolume() override;
  bool IsMuted() override;
  void SetOutputDeviceForAec(const std::string& output_device_id) override;

 private:

  enum StreamState {
    kClosed,            // Not opened yet
    kStopped,           // Device opened, but not started yet
    kRunning,           // Started, device playing
    kStopWait           // Stopping, waiting for the real-time thread to exit
  };

  // C-style call-backs
  static void* ThreadEntry(void *arg);

  // Continuously moves data from the device to the consumer
  void ThreadLoop();
  // Our creator, the audio manager needs to be notified when we close.
  AudioManagerBase* manager;
  // Parameters of the source
  AudioParameters params;
  // Device file name of the audio device
  const std::string device_name;
  // We store data here for consumer
  std::unique_ptr<AudioBus> audio_bus;
  // Call-back that consumes recorded data
  AudioInputCallback* callback;  // Valid during a recording session.
  // Handle of the audio device
  int fd;
  // Current state of the stream
  enum StreamState state;
  // High priority thread running ThreadLoop()
  pthread_t thread;
  // Protects vol, inputvol
  pthread_mutex_t mutex;
  // Current volume in the AUDIO_MIN_GAIN..AUDIO_MAX_GAIN range
  int vol;
  // Current volume in the 0.0..1.0 range
  double inputvol;
  // Temporary buffer
  char* buffer;
};

}  // namespace media

#endif  // MEDIA_AUDIO_AUDIOIO_AUDIOIO_INPUT_H_
