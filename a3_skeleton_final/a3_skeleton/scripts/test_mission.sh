#!/bin/bash

echo "Starting terrain mapping mission with all goals published at once..."

# Publish all goals in a single PoseArray message
echo "Publishing all waypoints simultaneously..."
ros2 topic pub --once /mission/goals geometry_msgs/msg/PoseArray '{
  "header": {
    "frame_id": "world"
  },
  "poses": [
    {
      "position": {"x": -10.0, "y": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z":0.0,  "w": 1.0}
    },
    {
      "position": {"x": -10.0, "y": 5.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    {
      "position": {"x": -5.0, "y": -3.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    {
      "position": {"x": 4.0, "y": 4.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    {
      "position": {"x": 0.0, "y": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  ]
}'

echo "All goals published! You should now see:"
echo "  1. All 5 blue arrow markers pointing toward next goals"
echo "  2. Complete traversability graph information in drone logs"
echo "  3. All goals available for mission planning"
echo ""
echo "To start the mission, run:"
echo "ros2 service call /mission/control std_srvs/srv/SetBool \"data: true\""
echo ""
echo "Mission complete!"
