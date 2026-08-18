#!/bin/bash
BATTERY=$1
TRIAL_ID=$2

if [ -z "$BATTERY" ] || [ -z "$TRIAL_ID" ]; then
  echo "Usage: ./record_trial.sh <battery1|battery2|battery3> <trial_id>"
  exit 1
fi

OUTDIR="herald_bags/${BATTERY}/${TRIAL_ID}"

case "$BATTERY" in
  battery1)
    TOPICS="/scan /scan_corrected /imu/data_raw /imu/filtered_pitch_roll"
    ;;
  battery2)
    TOPICS="/scan /scan_corrected /imu/data_raw /imu/filtered_pitch_roll /cmd_vel_mux_out"
    ;;
  battery3)
    TOPICS="/scan /scan_corrected /floor_drop_alert /floor_drop_measured_range"
    ;;
  *)
    echo "Unknown battery: $BATTERY (expected battery1, battery2, or battery3)"
    exit 1
    ;;
esac

if [ -d "$OUTDIR" ]; then
  echo "ERROR: $OUTDIR already exists -- refusing to overwrite. Use a different trial_id."
  exit 1
fi

echo "Recording $BATTERY / $TRIAL_ID -> $OUTDIR"
echo "Topics: $TOPICS"
ros2 bag record -o "$OUTDIR" $TOPICS
