#!/bin/bash
docker run -it --rm \
    --name gazebo-test \
    -p 8080:8080 \
    offroad-gazebo-integration:latest \
    run_with_vnc.sh bash -c "gz sim /workspace/src/offroad_gazebo_integration/worlds/test_simple.world"
