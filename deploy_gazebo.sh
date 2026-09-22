#!/bin/bash
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/deploy.sh" robot_backend:=gazebo
