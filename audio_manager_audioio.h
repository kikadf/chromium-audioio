#ifndef MEDIA_AUDIO_AUDIOIO_AUDIO_MANAGER_AUDIOIO_H_
#define MEDIA_AUDIO_AUDIOIO_AUDIO_MANAGER_AUDIOIO_H_

#include <set>

#include "base/compiler_specific.h"
#include "base/memory/ref_counted.h"
#include "base/memory/raw_ptr.h"
#include "base/threading/thread.h"
#include "media/audio/audio_manager_base.h"

#define INPUT   0
#define OUTPUT  1

namespace media {

class MEDIA_EXPORT AudioManagerAudioIO : public AudioManagerBase {
 public:
  AudioManagerAudioIO(std::unique_ptr<AudioThread> audio_thread,
                   AudioLogFactory* audio_log_factory);

  AudioManagerAudioIO(const AudioManagerAudioIO&) = delete;
  AudioManagerAudioIO& operator=(const AudioManagerAudioIO&) = delete;

  ~AudioManagerAudioIO() override;

  // Implementation of AudioManager.
  // Returns true if the OS reports existence of audio devices. This does not
  // guarantee that the existing devices support all formats and sample rates.
  bool HasAudioOutputDevices() override;
  // Returns true if the OS reports existence of audio recording devices. This
  // does not guarantee that the existing devices support all formats and
  // sample rates.
  bool HasAudioInputDevices() override;
  // Appends a list of available input devices to |device_names|,
  // which must initially be empty.
  void GetAudioInputDeviceNames(AudioDeviceNames* device_names) override;
  // Appends a list of available output devices to |device_names|,
  // which must initially be empty.
  void GetAudioOutputDeviceNames(AudioDeviceNames* device_names) override;
  // Returns the input hardware audio parameters of the specific device
  // for opening input streams. Each AudioManager needs to implement their own
  // version of this interface.
  AudioParameters GetInputStreamParameters(const std::string& device_id) override;
  // Gets the name of the audio manager (e.g., Windows, Mac, PulseAudio).
  const std::string_view GetName() override;

  // Implementation of AudioManagerBase.
  // Creates the output stream for the |AUDIO_PCM_LINEAR| format. The legacy
  // name is also from |AUDIO_PCM_LINEAR|.
  AudioOutputStream* MakeLinearOutputStream(
      const AudioParameters& params,
      const LogCallback& log_callback) override;
  // Creates the output stream for the |AUDIO_PCM_LOW_LATENCY| format.
  AudioOutputStream* MakeLowLatencyOutputStream(
      const AudioParameters& params,
      const std::string& device_id,
      const LogCallback& log_callback) override;
  // Creates the input stream for the |AUDIO_PCM_LINEAR| format. The legacy
  // name is also from |AUDIO_PCM_LINEAR|.
  AudioInputStream* MakeLinearInputStream(
      const AudioParameters& params,
      const std::string& device_id,
      const LogCallback& log_callback) override;
  // Creates the input stream for the |AUDIO_PCM_LOW_LATENCY| format.
  AudioInputStream* MakeLowLatencyInputStream(
      const AudioParameters& params,
      const std::string& device_id,
      const LogCallback& log_callback) override;

 protected:
  // Returns the preferred hardware audio output parameters for opening output
  // streams. If the users inject a valid |input_params|, each AudioManager
  // will decide if they should return the values from |input_params| or the
  // default hardware values. If the |input_params| is invalid, it will return
  // the default hardware audio parameters.
  // If |output_device_id| is empty, the implementation must treat that as
  // a request for the default output device.
  AudioParameters GetPreferredOutputStreamParameters(
      const std::string& output_device_id,
      const AudioParameters& input_params) override;

 private:
  // Called by MakeLinearOutputStream and MakeLowLatencyOutputStream.
  AudioOutputStream* MakeOutputStream(const std::string& device_id, const AudioParameters& params);
  AudioInputStream* MakeInputStream(const std::string& device_id, const AudioParameters& params);
  void GetAudioDevices(int type);

  raw_ptr<AudioDeviceNames> output_devices;
  raw_ptr<AudioDeviceNames> input_devices;
};

}  // namespace media

#endif  // MEDIA_AUDIO_AUDIOIO_AUDIO_MANAGER_AUDIOIO_H_
