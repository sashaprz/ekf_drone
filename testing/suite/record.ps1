# Fly every suite mission once in Gazebo with ORACLE=1 (controller on ground truth) and
# save testing/data/flight_<mission>.csv (+ .cal.csv). Fresh Gazebo restart per mission
# (fly.sh). Aggressive missions get the MAX_VEL_XY/MAX_TILT_DEG overrides.
#   powershell -File testing/suite/record.ps1                 # all missions
#   powershell -File testing/suite/record.ps1 box circle_slow # some
# Then check them: python testing/suite/validate.py --recordings
$all = @("hover", "box", "circle_slow", "circle_fast", "stops", "yaw_steps", "yaw_spin", "takeoff_land", "patrol_long")
$aggressive = @("circle_fast", "stops")
$missions = if ($args.Count -gt 0) { $args } else { $all }
$fly = "/mnt/c/Users/Sasha/repos/python_drone/testing/suite/fly.sh"
foreach ($m in $missions) {
    $envs = @("ORACLE=1")
    if ($aggressive -contains $m) { $envs += @("MAX_VEL_XY=3.5", "MAX_TILT_DEG=25") }
    Write-Output "=== recording $m ($($envs -join ' '))"
    wsl -d Ubuntu-24.04 -- bash $fly $m "testing/data/flight_$m.csv" "testing/data/logs/rec_$m.log" @envs
}
