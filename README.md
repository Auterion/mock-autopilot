## Build

Let’s say we want to build fake_autopilot in __/tmp/my_test__, and start by creating the folder:

    $ mkdir /tmp/my_test
    $ cd /tmp/my_test

We can now pull the fake_autopilot sources:

    $ git clone https://github.com/Auterion/fake_autopilot

Because we depend on MAVSDK, we need to build it. Moreover, we have a small patch to apply to it in order to communicate with QGC. Let’s start by cloning and patching MAVSDK (still from __/tmp/my_test__):

    $ git clone https://github.com/mavlink/mavsdk --recursive
    $ cd mavsdk
    $ git apply ../fake_autopilot/mavsdk.patch

We can now build MAVSDK:

    $ cmake -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=build/install -DENABLE_MAVLINK_PASSTHROUGH=ON -Bbuild -S.
    $ cmake --build build --target install

And now we can finally build fake_autopilot. Note that because we did not install MAVSDK on the system (but locally in /tmp/my_test/mavsdk/build/install), we need to tell CMake where to look for it (that’s the __CMAKE_PREFIX_PATH__):

    $ cd ../fake_autopilot
    $ cmake -DCMAKE_PREFIX_PATH="/tmp/my_test/mavsdk/build/install;/tmp/my_test/mavsdk/build/third_party/install" -Bbuild -S.
    $ cmake --build build

And that’s it! If everything went well, this should result in the binary __build/fake_autopilot__!

## Run

It is now simply a matter of running the binary:

    $ ./build/fake_autopilot

Note that it will broadcast MAVLink heartbeats on UDP port 14550 (for QGroundControl), but that can be changed here.

Feel free to start QGroundControl now and observe that the position, home and a mission are received and shown in the UI!
