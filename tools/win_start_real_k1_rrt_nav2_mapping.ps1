param(
    [string]$K1 = "soc@192.168.43.40",
    [ValidateSet("manual", "nav2-preview", "nav2-run", "nav2-preview-2m", "nav2-run-2m", "nav2-run-2m-unlimited")]
    [string]$Mode = "nav2-preview",
    [switch]$CleanFirst
)

$ErrorActionPreference = "Stop"
$RemoteScript = "/home/soc/edge-ai-robot-k1/tools/start_real_k1_rrt_nav2_mapping.sh"

function Invoke-K1 {
    param([string]$Command)
    ssh $K1 "bash -lc 'bash $RemoteScript $Command'"
}

function Start-K1Window {
    param(
        [string]$Title,
        [string]$Command
    )
    $remote = "bash -lc 'bash $RemoteScript $Command'"
    $psCommand = "Write-Host '$Title'; ssh -t $K1 `"$remote`""
    Start-Process powershell -ArgumentList @("-NoExit", "-Command", $psCommand)
}

if ($CleanFirst) {
    Invoke-K1 "clean"
}

switch ($Mode) {
    "manual" {
        Start-K1Window "K1 manual guarded mapping" "manual-map"
        Start-Sleep -Seconds 8
        Start-K1Window "K1 teleop manual input_cmd_vel" "teleop-manual"
    }
    "nav2-preview" {
        Start-K1Window "K1 Nav2 SLAM guarded stack" "nav2-slam"
        Start-Sleep -Seconds 10
        Start-K1Window "K1 D435 ROS driver" "d435"
        Start-Sleep -Seconds 6
        Start-K1Window "K1 YOLO EP risk bridge" "yolo-ep"
        Start-Sleep -Seconds 4
        Start-K1Window "K1 RRT preview goal publisher" "rrt-preview"
        Start-K1Window "K1 YOLO risk UI" "ui"
        Start-Process "http://192.168.43.40:8765/yolo_monitor.html"
    }
    "nav2-run" {
        Start-K1Window "K1 Nav2 SLAM guarded stack" "nav2-slam"
        Start-Sleep -Seconds 10
        Start-K1Window "K1 D435 ROS driver" "d435"
        Start-Sleep -Seconds 6
        Start-K1Window "K1 YOLO EP risk bridge" "yolo-ep"
        Start-Sleep -Seconds 4
        Start-K1Window "K1 RRT Nav2 explorer" "rrt-run"
        Start-K1Window "K1 YOLO risk UI" "ui"
        Start-Process "http://192.168.43.40:8765/yolo_monitor.html"
    }
    "nav2-preview-2m" {
        Start-K1Window "K1 Nav2 SLAM guarded stack" "nav2-slam"
        Start-Sleep -Seconds 10
        Start-K1Window "K1 D435 ROS driver" "d435"
        Start-Sleep -Seconds 6
        Start-K1Window "K1 YOLO EP risk bridge" "yolo-ep"
        Start-Sleep -Seconds 4
        Start-K1Window "K1 RRT 2m preview goal publisher" "rrt-preview-2m"
        Start-K1Window "K1 YOLO risk UI" "ui"
        Start-Process "http://192.168.43.40:8765/yolo_monitor.html"
    }
    "nav2-run-2m" {
        Start-K1Window "K1 Nav2 SLAM guarded stack" "nav2-slam"
        Start-Sleep -Seconds 10
        Start-K1Window "K1 D435 ROS driver" "d435"
        Start-Sleep -Seconds 6
        Start-K1Window "K1 YOLO EP risk bridge" "yolo-ep"
        Start-Sleep -Seconds 4
        Start-K1Window "K1 RRT 2m Nav2 explorer" "rrt-run-2m"
        Start-K1Window "K1 YOLO risk UI" "ui"
        Start-Process "http://192.168.43.40:8765/yolo_monitor.html"
    }
    "nav2-run-2m-unlimited" {
        Start-K1Window "K1 Nav2 SLAM guarded stack" "nav2-slam"
        Start-Sleep -Seconds 18
        Start-K1Window "K1 RRT 2m unlimited guarded explorer" "rrt-run-2m-unlimited"
    }
}

Write-Host "Started mode: $Mode"
Write-Host "Use K1 command to save map when done:"
Write-Host "  cd /home/soc/edge-ai-robot-k1; bash tools/start_real_k1_rrt_nav2_mapping.sh save-map"
