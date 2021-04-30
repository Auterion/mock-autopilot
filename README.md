## Build

The most convenient way to build the project is using Docker container with required dependencies installed:
```
./docker_run.sh ./build.sh
```
Building directly in host system is almost exactly the same, but it's user responsibility to install required packages. The list can be found in [Dockerfile](Dockerfile):
```
./build.sh
```

By default, the project is built for `amd64` (`x86_64`). To build for different architecture, either an environment variables must be set or options for `docker_run.sh` and `build.sh` scripts provided.
* environment variables for Skynode build
```
    export BASE_IMAGE=arm64v8/ubuntu:20.04
    export IMAGE=arm64v8/mock-autopilot
    ./docker_run.sh ./build.sh -b arm64v8_build
```
* script arguments for Skynode build
```
    ./docker_run.sh -b "arm64v8/ubuntu:20.04" -i "arm64v8/payload-driver-example" 
    ./build.sh -b arm64v8_build
```

## Package
Packaging process is almost exactly the same as `Build`. Use the same steps and settings, but instead calling `build.sh` execute `package.sh`. 
```
./docker_run.sh ./package.sh
```
By default the result is produced in `output/` directory:
```
output/
├── mock-autopilot_1.0-1_amd64.buildinfo
├── mock-autopilot_1.0-1_amd64.changes
├── mock-autopilot_1.0-1_amd64.deb
└── mock-autopilot-dbgsym_1.0-1_amd64.ddeb
```

## Run

It is now simply a matter of running the binary:

    $ ./build/mock-autopilot

Note that it will broadcast MAVLink heartbeats on UDP port 14550 (for QGroundControl), but that can be changed here.

Feel free to start QGroundControl now and observe that the position, home and a mission are received and shown in the UI!
