param(
    [string]$Date = "",
    [switch]$IncludeGitSnapshot
)

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($Date)) {
    $Date = Get-Date -Format "yyyy-MM-dd"
}

$TemplatePath = Join-Path $ScriptDir "_template.md"
$OutPath = Join-Path $ScriptDir ("{0}.md" -f $Date)

if (-not (Test-Path $TemplatePath)) {
    Write-Host "Template not found: $TemplatePath"
    exit 1
}

if (Test-Path $OutPath) {
    Write-Host "Already exists: $OutPath"
    exit 0
}

$content = Get-Content -Raw -Path $TemplatePath
$content = $content.Replace("{{DATE}}", $Date)
Set-Content -Path $OutPath -Value $content -Encoding UTF8

if ($IncludeGitSnapshot) {
    $sha = ""
    try { $sha = (git rev-parse --short HEAD) } catch { $sha = "N/A" }

    $status = ""
    try { $status = (git status --short | Out-String).TrimEnd() } catch { $status = "N/A" }

    $diffstat = ""
    try { $diffstat = (git diff --stat | Out-String).TrimEnd() } catch { $diffstat = "N/A" }

    Add-Content -Path $OutPath -Value "`n## 8) Auto Snapshot`n"
    Add-Content -Path $OutPath -Value "- git_sha: $sha"
    Add-Content -Path $OutPath -Value "`n### git status --short`n"
    Add-Content -Path $OutPath -Value "```text`n$status`n```"
    Add-Content -Path $OutPath -Value "`n### git diff --stat`n"
    Add-Content -Path $OutPath -Value "```text`n$diffstat`n```"
}

Write-Host "Created: $OutPath"
Write-Host "Tip: Use -IncludeGitSnapshot to append status/stat blocks."
