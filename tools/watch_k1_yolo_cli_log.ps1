param(
  [string]$HostName = "192.168.43.40",
  [string]$User = "soc",
  [string]$RunId = "cli_ep_480x640_truncated6_light5"
)

$ErrorActionPreference = "Stop"
$logPath = "/home/soc/edge-ai-robot-k1/logs/k1_yolo_${RunId}.log"
Write-Host "[watch] tailing ${User}@${HostName}:${logPath}"
ssh "${User}@${HostName}" "touch '$logPath'; tail -n 40 -f '$logPath'"
