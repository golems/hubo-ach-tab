# Hubo-Ach-Tab

A tab that allows the
[DART](https://github.com/dartsim/dart)/[GRIP](https://github.com/dartsim/grip)
simulation environment to emulate a Hubo robot by pretending to be the
hardware daemon from
[hubo-ach](https://github.com/hubo/hubo-ach). When loaded, it will
attach to the necessary state and command channels, feeding commands
to the simulated hubo in DART/GRIP and publishing state updates as
appropriate.

## Installing dependencies
===

hubo-ach-tab depends on [dart](https://github.com/dartsim/dart),
[grip](https://github.com/dartsim/grip),
[ach](https://github.com/golems/ach), and
[hubo-ach](https://github.com/hubo/hubo-ach). Another package,
[hubo-motion-rt](https://github.com/hubo/hubo-motion-rt/), is optional
but recommended for ease of use.


### Using fixed releases of DART and GRIP

DART and GRIP are very unstable. Some hubo-ach-tab users may be happier using
fixed releases of dart and grip and living with bugs in the code
instead of bugs in the build system. If you want to do this, hubo-ach-tab
v0.2 is tagged in the git repository and will work with DART v2.4 and
GRIP v2.4.

### Using development versions of DART and GRIP

You can also install bleeding-edge versions of DART and GRIP by
compiling from source. In this case, check out, compile, and install
the master branches of both, and then use the master branch of
hubo-ach-tab.

### Ach

Ach can be installed from an apt repository, as descbribed on its
[github page](https://github.com/golems/ach).

### Hubo-Ach

Hubo-ach must be built from source. You can find instructions for how
to do this on the hubo-ach [github page](https://github.com/hubo/hubo-ach).

### Hubo-motion-rt

[hubo-motion-rt](https://github.com/hubo/hubo-motion-rt/) provides a
useful startup script for hubo-ach and its
[examples](https://github.com/hubo/examples-hubo-motion-rt) are a
simple way to test the functionality of the hubo-ach-tab. Instructions
for building and installing these are at their github pages.

### A note about debian packages

All of the dependencies which can be built from source are capable of
building debian packages, allowing them to be cleanly installed and
uninstalled using apt. If you are building from source, this is
*strongly recommended*. DART and GRIP can build debian packages using
`cpack -G DEB`, while hubo-ach and hubo-motion-rt can build packages
using `dpkg-buildpackage` (or pbuilder, if you have it set up). The
resulting debian packages can be installed using `dpkg -i`.

## Building
===

hubo-ach-tab builds using cmake. Once all of the dependencies are
installed, clone the repository (and, if you installed tagged versions
of DART and GRIP, check out the corresponding tagged version of
hubo-ach-tab) and build hubo-ach-tab:

    cd hubo-ach-tab
    mkdir -p build && cd build
    cmake ..
    make

## Running
===

### Starting ach channels

The ach channels for controlling hubo and receiving status updates
from it must be opened before running hubo-ach-tab. This can be done manually:

	sudo ach -1 -C hubo-ref -m 10 -n 3000 -o 666
	sudo ach -1 -C hubo-state -m 10 -n 8000 -o 666

but if you've installed hubo-motion-rt, it's much easier to do this
using the initscript it provides:

    sudo service hubo-motion dart

### Start hubo-ach-tab

Once the ach channels are created, you can run hubo-ach-tab.

    ./HuboAchTab

You now have to instruct the DART/GRIP simulator to load a world with
hubo in it. hubo-ach-tab comes with a simple, hubo-only world, which can
be found in hubo-models/huboplus-empty-world.urdf. Click File->Load
and navigate to your world. Once a hubo-containing world is loaded,
hubo-ach-tab will attempt to connect to the necessary ach channels. If
it successfully connects, hubo-ach-tab will automatically begin
simulating your hubo using the reference positions provided by the
hubo-ref channel and publishing its state data to the hubo-state
channel. You can now run any hubo-ach program you want and it will
behave as normal.

## Installing everything as debian packages
===

Users that are annoyed by make installs are able to install everything
in hubo-ach-tab's dependency tree as debian packages. Packages for DART
and GRIP can be built from source using `cpack -G DEB` in their build
directory. Packages for hubo-ach and hubo-motion can be built from source
using `dpkg-buildpackage`. Once the resulting .deb files are installed
using `dpkg -i <filename>`, a debian package for hubo-ach-tab can be
build and installed using `cpack -G DEB` and `dpkg -i <filename>`. This
will additionally install an empty world suitable for use with hubo-ach-tab
into /usr/share/hubo-ach-tab, which hubo-ach-tab will automatically load
and begin simulating at startup.

## Features
===

hubo-ach-tab currently supports position control on all of hubo's
major joints. It will also report accurate joint positions and
velocities. However, there are a few features missing at the moment:
* Control of hubo's fingers
* Control of hubo's head
* Force-torque sensor readings
* Inertial measurement unit reading

There are also some problems with the underlying simulator, the
largest of which is that hubo's feet tend to stick to the floor and
keep him upright when he really should have fallen over.
