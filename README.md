# Introduction:

Audioreach-PAL (AudioReach Platform Abstraction Layer) is hardware absration layer for audio and offers different APIs for clients like Pulseaudio/Audio-Server to setup and configure audio use cases. PAL handles the configuration of DAC/ADC through tinyalsa mixer controls and communicates with the AGM (Audio Graph Manager) to relay information to the ADSP for setting up the use case.

## Documentation:

To be available soon.

## Autotools installation directories

`configure` accepts the following directory options:

| Option | Default | Contents |
| --- | --- | --- |
| `--with-pal-plugin-dir=DIR` | `${libdir}` | PAL plugin libraries |
| `--with-pal-config-dir=DIR` | `/etc` | `plugin_manager.xml` and `usecaseKvManager.xml` |
| `--with-pal-data-dir=DIR` | `/etc` | Mixer paths and resource-manager XML files |

These directories are also used at runtime. `DESTDIR` only stages installation;
it is not part of the runtime paths. When building `plugins/vui_interface`
separately, pass the same `--with-pal-plugin-dir` option to its `configure`.

## License:

Audioreach-pal (source files are licensed under the BSD-3-Clause-Clear. Check out the LICENSE for more details
