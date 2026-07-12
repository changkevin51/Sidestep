# Two-phone ElevenLabs driver alerts

Each Pi owns its collision decision and its driver message. After matching V2V
proposals (or an emergency timeout), a background worker on that Pi uses the
QNX OSS **libcurl** C API to:

1. POST the warning text to ElevenLabs over HTTPS;
2. receive an MP3 in memory; and
3. POST the MP3 to the laptop bridge, which routes `CAR1` only to the CAR1 phone
   socket and `CAR2` only to the CAR2 phone socket.

The audio path is advisory. A missing API key, slow Internet connection, closed
phone tab, or TTS failure is logged but never delays the V2V listener, consensus
deadline, or vehicle action.

## Timing and spoken messages

The collision sweep still looks 10 seconds ahead, but a proposal/driver warning
starts when TTC reaches **4.0 seconds**. This leaves roughly 3.85 seconds after
the 150 ms consensus deadline. The speech reports TTC minus a conservative
**1.0-second cloud/socket/playback allowance**, so a 3.9-second measured TTC is
spoken as "potential impact in about three seconds." Under about 2.25 seconds,
it says impact may be imminent instead of giving a stale number.

The action is spoken only after it is committed. Examples are:

- "Collision warning. Potential impact in about three seconds. Braking to
  prevent collision."
- "Collision warning. Potential impact in about three seconds. Swerving left
  to prevent collision."
- "Collision warning. Impact may be imminent. Emergency stop to prevent
  collision."

## ElevenLabs output types

The TTS API currently offers these output families (availability of the highest
rates depends on the ElevenLabs plan):

| API family | Specific media/encoding | Published rates |
| --- | --- | --- |
| `mp3_*` | MP3 (`audio/mpeg`) | 22.05 or 44.1 kHz; 32–192 kbps |
| `wav_*` | WAV container (`audio/wav`), 16-bit PCM | 8, 16, 22.05, 24, 32, 44.1, or 48 kHz |
| `pcm_*` | Raw signed 16-bit little-endian PCM | 8, 16, 22.05, 24, 44.1, or 48 kHz |
| `ulaw_8000` | G.711 μ-law | 8 kHz telephony audio |
| `alaw_8000` | G.711 A-law | 8 kHz telephony audio |
| `opus_48000_*` | Opus | 48 kHz; 32–192 kbps |

This project deliberately requests **`mp3_44100_128`**, meaning MP3 at 44.1 kHz
and 128 kbps, and transports it as **`Content-Type: audio/mpeg`**. MP3 is compact,
self-describing, and decoded by current iPhone and Android browser audio stacks.
The endpoint's default is also `mp3_44100_128`. See the official
[ElevenLabs create-speech API](https://elevenlabs.io/docs/api-reference/text-to-speech/convert)
and [supported-output overview](https://elevenlabs.io/docs/overview/capabilities/text-to-speech).

The code uses `eleven_flash_v2_5` by default because ElevenLabs positions Flash
for low-latency TTS. Override it with `ELEVENLABS_MODEL_ID` if required.

## 1. Prepare ElevenLabs

1. Create/sign in to an ElevenLabs account.
2. Create an API key. Do not put it in `config.json`, the browser, source control,
   or either phone URL.
3. Choose a voice and copy its voice ID. Test that voice with the short safety
   phrases above; a calm, clear voice works better than an expressive one here.

Both Pis may use the same key and voice. One collision normally makes two API
generations (one per car), which count against the ElevenLabs quota.

## 2. Install QNX SDP 8 and libcurl on a Windows development computer

The Pis do **not** need `q++` and you do not need `sudo` on them. Install the
QNX toolchain on your Windows computer, cross-compile an AArch64 QNX executable
there, then copy that executable to each Pi.

### Two-laptop layout (your setup)

It is normal for the QNX SDP laptop to have no connection to either Pi. Treat
the machines as follows:

```text
Laptop A: QNX SDP + q++      -> builds v2v_brain for QNX/AArch64
USB stick / shared drive     -> moves the built file
Laptop B: hotspot + SSH      -> copies the file to both Pis and runs the bridge
Pi 1 and Pi 2: QNX targets   -> run v2v_brain; they do not compile it
```

Build on Laptop A. Move only the resulting `v2v_brain` file to Laptop B (using
a USB drive is perfectly fine). From Laptop B, use the SSH/SCP connection you
already have to deploy it to both Pis. Do not install `q++` on the Pis; it is a
development-host compiler, not a normal QNX target-image component.

### A. Get the QNX SDP 8 licence

1. On the Windows computer, create or sign in to a myQNX account.
2. For a personal/student/demo project, request the free non-commercial QNX SDP
   8 licence at [QNX Everywhere](https://qnx.com/getqnx).
3. Accept the licence terms in your myQNX account and deploy the licence to your
   own account if QNX asks you to do so.

### B. Install QNX Software Center and the SDP baseline

1. Download **QNX Software Center — Windows Hosts** from the
   [QNX Download Center](https://www.qnx.com/download/group.html?programid=29178).
2. Run the installer on the Windows computer. This may require a Windows
   administrator password; that is unrelated to the Pi's lack of `sudo`.
3. Start **QNX Software Center** and sign in to the myQNX account from step A.
4. On the Welcome page, click **Add Installation**.
5. Select **QNX Software Development Platform 8.0**, choose an installation
   directory you can find later (for example, `C:\Users\YOUR_NAME\qnx800`),
   then continue through the wizard and click **Finish**.
6. Wait for the baseline installation to complete. This is what provides `q++`,
   `qcc`, `make`, headers, and the QNX AArch64 target toolchain.

If SDP 8.0 is absent or marked inaccessible, your licence has not been deployed
to the signed-in myQNX account. Fix that in the myQNX licence dashboard, then
restart QNX Software Center.

### C. Install the curl package into that same SDP installation

In QNX Software Center for the installed QNX SDP 8.x, install the networking
curl package (package ID **`com.qnx.qnx800.target.net.curl`**). QNX's 8.0.1
networking release notes identify this as the supported curl/libcurl package;
QNX also publishes its open-source curl builds at
[Open Source for QNX](https://qnx.haxx.se/).

### D. Install it in the QNX Software Center GUI

Do this on the **Windows/Linux development computer that has QNX Software
Center and your QNX SDP 8 installation**. Do not try to install this package
from the Pi shell.

1. Open **QNX Software Center** and sign in with the myQNX account that owns
   your QNX SDP 8 entitlement.
2. Select your existing **QNX Software Development Platform 8.x** installation
   from the installation selector in the upper-right corner.
3. Open the **Available** tab. If you are on the welcome screen, choose
   **Install New Packages** first.
4. In the filter box, search for exactly: `com.qnx.qnx800.target.net.curl`
   (searching `curl` also works).
5. Select **QNX SDP 8.0 Networking - curl**. Confirm its package ID is
   `com.qnx.qnx800.target.net.curl`.
6. Right-click it and choose **Install**. In the wizard, leave the automatically
   selected dependencies checked, click **Next**, review the list, then click
   **Finish**.
7. Wait for completion, open the **Installed** tab, and confirm the curl package
   is listed. Select it and use **Properties → Package Contents** if you need
   QNX Software Center to reveal the exact installed library paths.

QNX's Software Center documentation confirms that the Available tab installs
add-on packages and resolves dependencies automatically. If the package is not
shown, open **All**, select the inaccessible package, use **Properties → Check
Why**, and verify in **Advanced → Edit Installation Properties** that the
AArch64 target architecture is enabled. An unavailable package can also mean
your myQNX entitlement has not been deployed to your account.

### E. Open the QNX build shell, verify, and compile

Open **Command Prompt (`cmd.exe`)**, not PowerShell, on Windows. Change to the
exact SDP directory selected in step B, then run `qnxsdp-env.bat` to set
`QNX_HOST`, `QNX_TARGET`, and the tool paths. The PowerShell error "make is not
recognized as the name of a cmdlet, function, script file, or operable program"
means this step was run in the wrong shell. For example:

```bat
cd C:\Users\YOUR_NAME\qnx800
call qnxsdp-env.bat
where q++
where make
q++ -V
echo %QNX_TARGET%
```

Do not type the example directory literally if you installed QNX elsewhere.
Both `where q++` and `where make` must print paths, and `q++ -V` must list a
compiler variant before continuing. If `q++` works but `make` does not, find the
host copy with `dir /s /b "%QNX_HOST%\make.exe"` and use the full returned path
instead of `make`. If neither tool exists, go back to QNX Software Center and
install the full **QNX Software Development Platform 8.0 baseline** in addition
to the curl package; curl alone does not include host compiler/build tools.

In the **same Command Prompt**, verify curl in the installed target tree:

```bat
dir /s /b "%QNX_TARGET%\*curl.h"
dir /s /b "%QNX_TARGET%\libcurl.so*"
```

The `echo %QNX_TARGET%` command above must print a real QNX target directory,
and the two `dir` commands must find the libcurl header and library. If either
search produces nothing, reopen the QNX shell or verify the QNX Software Center
package installation before trying to build.

Still in that Command Prompt, build the project from its Windows workspace
directory:

```bat
cd C:\Users\chang\Documents\Projects\Sidestep\qnx-brain
make clean
make CXX=q++ QNX_VARIANT=gcc_ntoaarch64le
```

The Makefile compiles `driver_alert.cpp` and links `-lcurl`; invoking the curl
command-line program does not satisfy this implementation. If the error is
`curl/curl.h: No such file or directory` or `cannot find -lcurl`, the curl
package is not installed in the active SDP target environment.

This creates `v2v_brain`, which is an AArch64 QNX executable—not a Windows
program. If this is a different computer from the hotspot-connected laptop,
copy `v2v_brain` to a USB drive or shared folder first. From the
hotspot-connected laptop, copy it to each Pi. If SSH/SCP is enabled and the QNX
login is `qnx`, an example from the Windows Command Prompt is:

```bat
scp v2v_brain qnx@PI_1_IP:/home/qnx/v2v_brain
scp v2v_brain qnx@PI_2_IP:/home/qnx/v2v_brain
```

Replace `qnx` and each `PI_*_IP` with the actual QNX username and Pi IPv4
address. On each Pi, make it executable and verify its runtime dependency before
a demo:

```sh
chmod +x /home/qnx/v2v_brain
cd /home/qnx
ls -l /usr/lib/libcurl.so*
ldd ./v2v_brain | grep libcurl
```

If either command cannot find libcurl, add the `libcurl.so*` files and the
dependencies shown by `ldd` to your Pi's QNX image. Use the **Package Contents**
view from step 7 to identify the exact architecture-matched files; do not copy
an x86_64 host library to an AArch64 Pi.

If you cannot modify the Pi's system image or `/usr/lib`, a demo-only,
no-root alternative is to keep the required AArch64 libraries under your QNX
user's home directory. On Laptop A, locate the AArch64 `libcurl.so*` files with
the `dir` command above, copy them beside `v2v_brain` on Laptop B, and then run
on each Pi:

```sh
mkdir -p /home/qnx/lib
```

Copy the matching `libcurl.so*` files to `/home/qnx/lib/`. Then check:

```sh
LD_LIBRARY_PATH=/home/qnx/lib:/usr/lib ldd /home/qnx/v2v_brain
```

For every library that `ldd` reports as **not found**, copy the matching
`aarch64le` QNX library from Laptop A's `$QNX_TARGET` tree to `/home/qnx/lib/`,
then rerun the command. When it shows no missing libraries, start the program
with:

```sh
export LD_LIBRARY_PATH=/home/qnx/lib:/usr/lib
./v2v_brain --id CAR1 --peer CAR2
```

Use the CAR2 command on the second Pi. Never copy Windows DLLs, Linux `.so`
files, or x86_64 QNX libraries to the Pi.

## 3. Start the laptop bridge — copy these commands exactly

Pick one long random token. In PowerShell on the laptop, this creates a 32-byte
token, exposes it only to the process environment, and starts the bridge:

```powershell
cd laptop-visualizer
npm install
$env:V2V_ALERT_TOKEN = [Convert]::ToHexString([Security.Cryptography.RandomNumberGenerator]::GetBytes(32))
$env:V2V_BROADCAST_ADDRESS = '192.168.137.255'  # change for your hotspot subnet
npm start
```

Keep that PowerShell window open and copy the printed token value with:

```powershell
$env:V2V_ALERT_TOKEN
```

The bridge now listens on all laptop interfaces for HTTP `8000`; ordinary
visualizer telemetry injection remains loopback-only. Phone sockets require the
token and can receive only their selected car. Pi audio uploads also require the
same token and accept only MP3 up to 1 MiB.

In a second PowerShell window, run `ipconfig`. Under the adapter connected to
the same hotspot as the Pis/phones, write down its **IPv4 Address**. Call it
`LAPTOP_IP` below. Do not guess it from the examples; Windows hotspot addresses
often use `192.168.137.x`, but yours can differ.

If Windows asks, allow Node.js on **Private networks**. Otherwise, open an
Administrator PowerShell and add the inbound rules once:

```powershell
New-NetFirewallRule -DisplayName 'V2V phone HTTP and WebSocket' -Direction Inbound -Action Allow -Profile Private -Protocol TCP -LocalPort 8000,8080
New-NetFirewallRule -DisplayName 'V2V UDP telemetry' -Direction Inbound -Action Allow -Profile Private -Protocol UDP -LocalPort 12345
```

## 4. Configure and start each Pi — paste your three ElevenLabs values

You already have the API key, model ID, and voice ID. Once the `qnx-brain`
folder has been transferred to a Pi, make a private copy outside that folder;
do not edit the example file in the Git folder. On **each** Pi, run:

```sh
cp /path/to/qnx-brain/alert.env.example /home/qnx/v2v-alert.env
vi /home/qnx/v2v-alert.env
chmod 600 /home/qnx/v2v-alert.env
```

Replace these five quoted values:

| Variable | Paste this value |
| --- | --- |
| `ELEVENLABS_API_KEY` | Your ElevenLabs API key |
| `ELEVENLABS_VOICE_ID` | Your voice ID—not its friendly name |
| `ELEVENLABS_MODEL_ID` | Your supplied model ID, exactly as given |
| `V2V_ALERT_BRIDGE_URL` | `http://LAPTOP_IP:8000`, replacing `LAPTOP_IP` with the IPv4 address from `ipconfig` |
| `V2V_ALERT_TOKEN` | The output from `$env:V2V_ALERT_TOKEN` in the laptop PowerShell window |

For every run on each Pi, load the values into the current shell before starting
the program:

```sh
. /home/qnx/v2v-alert.env
cd /path/to/qnx-brain
./v2v_brain --id CAR1 --peer CAR2
```

On the second Pi, use this final command instead:

```sh
. /home/qnx/v2v-alert.env
cd /path/to/qnx-brain
./v2v_brain --id CAR2 --peer CAR1
```

Check connectivity from each Pi before the demo:

```sh
curl -I https://api.elevenlabs.io/
curl -I http://LAPTOP_IP:8000/
```

Each process should print `Driver voice alerts enabled`. If it prints that
alerts are not configured, recheck all five required environment variables.
Use `--no-audio` only when intentionally testing without the TTS path.

## 5. Connect the two phones

With the phones on the same hotspot, replace `LAPTOP_IP` and `TOKEN` below:

- Phone 1: `http://LAPTOP_IP:8000/driver/CAR1?token=TOKEN`
- Phone 2: `http://LAPTOP_IP:8000/driver/CAR2?token=TOKEN`

On each phone:

1. Open its URL in Safari or Chrome.
2. Tap **Enable speaker and connect**. The short local tone confirms the browser
   granted audio playback permission.
3. Turn up **media** volume, disable silent/focus modes, keep the tab foreground,
   and keep the screen awake for the demonstration.
4. Confirm the page says `Connected — listening for alerts`.

Mobile browsers deliberately block unprompted sound, which is why the tap is
mandatory. Background tabs and a locked phone may be suspended. Anyone with a
valid URL token can listen to that car on the hotspot, so generate a new token
for each demo and do not publish the links.

If one phone never connects, first open `http://LAPTOP_IP:8000/` on it. Failure
to load means the hotspot is isolating clients or the firewall is blocking TCP
8000. Disable AP/client isolation or use a hotspot/travel router that permits
device-to-device traffic. Some phone tethering modes isolate tethered clients;
a dedicated access point is more predictable for a five-device demo.
