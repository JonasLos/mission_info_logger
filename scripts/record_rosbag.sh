#!/bin/bash

# === Define Topic Categories ===
declare -A TOPIC_SETS

TOPIC_SETS[raw]="/camera_fl/image_color /lidar_tc/velodyne_points /radar_fc/as_tx/radar_tracks"
TOPIC_SETS[perception]="/yolov9/published_image /yolov9/bboxInfo /fused_bbox /road_segmentation /road_segment_3d/left_boundary /road_segment_3d/right_boundary /colored_points /bounding_boxes"
TOPIC_SETS[planning]="/local_path /lane_detection/output /lane_detection/current_lane_left_boundary /lane_detection/current_lane_right_boundary /Left_Line3dPoints /Right_Line3dPoints"
TOPIC_SETS[controls]="/ctrl_ref.* /vehicle.* /lat_ctrl_perf /current_fsm_status"
TOPIC_SETS[outputs]="/yoloLiveNode/bboxInfo /yolo_detection_node/published_image /tf_static /lidar_2d_projection"

# === Get Category Argument or Default ===
CATEGORY=${1:-raw}

if [[ -z "${TOPIC_SETS[$CATEGORY]}" ]]; then
    echo "[ERROR] Invalid category: '$CATEGORY'"
    echo "Valid categories: ${!TOPIC_SETS[@]}"
    exit 1
fi

TOPICS="${TOPIC_SETS[$CATEGORY]}"

# === Prompt for Metadata ===
read -p "Enter location [Unknown location]: " LOCATION
LOCATION=${LOCATION:-"Unknown location"}

read -p "Enter vehicle name/ID [Unknown vehicle]: " VEHICLE
VEHICLE=${VEHICLE:-"Unknown vehicle"}

read -p "Enter comments [No comments]: " COMMENTS
COMMENTS=${COMMENTS:-"No comments"}

read -p "Enter maneuver [No maneuver specified]: " MANEUVER
MANEUVER=${MANEUVER:-"No maneuver specified"}

read -p "Enter number of passengers [0]: " PASSENGERS
PASSENGERS=${PASSENGERS:-0}

read -p "Enter road type [Unknown]: " ROAD_TYPE
ROAD_TYPE=${ROAD_TYPE:-"Unknown"}

read -p "Enter road condition [Unknown]: " ROAD_CONDITION
ROAD_CONDITION=${ROAD_CONDITION:-"Unknown"}

# === Compose Metadata String ===
METADATA="location: $LOCATION, vehicle: $VEHICLE, passengers: $PASSENGERS, road_type: $ROAD_TYPE, road_condition: $ROAD_CONDITION, comments: $COMMENTS, maneuver: $MANEUVER"

# === File and Folder Setup ===
TIMESTAMP=$(date +'%Y-%m-%d_%H-%M-%S')
BAG_BASENAME="rosbag_${CATEGORY}_${TIMESTAMP}"
BAG_NAME="$BAG_BASENAME.bag"
TXT_NAME="$BAG_BASENAME.txt"
SAVE_DIR="/media/dev/T9/rosbag record testing command"
mkdir -p "$SAVE_DIR"

# === Write Metadata to TXT File ===
echo "[INFO] Writing metadata to $TXT_NAME"
{
    echo "Rosbag Name: $BAG_NAME"
    echo "Category: $CATEGORY"
    echo "Location: $LOCATION"
    echo "Vehicle: $VEHICLE"
    echo "Number of Passengers: $PASSENGERS"
    echo "Road Type: $ROAD_TYPE"
    echo "Road Condition: $ROAD_CONDITION"
    echo "Comments: $COMMENTS"
    echo "Maneuver: $MANEUVER"
    echo "Recorded Topics:"
    echo "$TOPICS"
    echo "/rosbag_metadata"
} > "$SAVE_DIR/$TXT_NAME"

# === Publish Metadata as Latched Topic ===
echo "[INFO] Publishing metadata to /rosbag_metadata"
rostopic pub -1 /rosbag_metadata std_msgs/String "data: \"$METADATA\"" &

sleep 1  # Allow time for publishing

# === Start Rosbag Recording ===
echo "[INFO] Starting rosbag recording..."
echo "Saving to: $SAVE_DIR/$BAG_NAME"
rosbag record -e --split --size=5000 -o "$SAVE_DIR/$BAG_BASENAME" $TOPICS /rosbag_metadata
