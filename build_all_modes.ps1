# Build each MAIN_MODE in turn (edit src/main.c -> run UV4 -> collect error/warning/Program Size).
# main.c is restored from the ORIGINAL TEXT at the end (WriteAllText, not Copy-Item: a Copy-Item
# restore once brought back a stale copy and left the tree in the wrong mode). Mode 4 is built
# LAST so that Objects\Interrupt.hex ends up being the production build. The script refuses to run
# unless main.c is currently in production mode, and re-verifies the mode line after restoring.
$ErrorActionPreference = "Stop"
$cwd  = (Get-Location).Path
$proj = Join-Path $cwd "MDK-ARM\Interrupt.uvprojx"
$src  = Join-Path $cwd "src\main.c"

# Explicit UTF-8 (no BOM) so that the Chinese comments in main.c survive the round trip.
$enc  = New-Object System.Text.UTF8Encoding($false)
$orig = [System.IO.File]::ReadAllText($src, $enc)
$head = $orig.Substring(0, 3)
if ($head[0] -eq [char]0xFEFF) { "WARNING: main.c starts with a BOM" }
# Remember what the mode line WAS; the restore at the end is verified against it.
$modeRe   = '#define MAIN_MODE\s+MAIN_MODE_[A-Z_]+'
$origMode = [regex]::Match($orig, $modeRe).Value
if ($origMode -notmatch 'MAIN_MODE_PRODUCTION') {
  "ERROR: main.c is not in production mode before building ($origMode) - fix the mode line first"
  exit 1
}
"original mode line: $origMode"

$modes = @(
  @{n=1; m="MAIN_MODE_LOWPOWER_TEST"},
  @{n=2; m="MAIN_MODE_LORA_DEBUG"},
  @{n=3; m="MAIN_MODE_PAIR_TEST"},
  @{n=5; m="MAIN_MODE_LEVEL_BENCH"},
  @{n=6; m="MAIN_MODE_PROBE_BENCH"},
  @{n=7; m="MAIN_MODE_LP_LEVEL_LORA_LOOP"},
  @{n=4; m="MAIN_MODE_PRODUCTION"}
)

$summary = New-Object System.Collections.Generic.List[string]
foreach ($md in $modes) {
  $txt = $orig -replace '#define MAIN_MODE\s+MAIN_MODE_[A-Z_]+', ('#define MAIN_MODE                   ' + $md.m)
  [System.IO.File]::WriteAllText($src, $txt, $enc)
  # CRITICAL: force a fresh mtime. If main.c ends up OLDER than Objects\main.o, Keil thinks
  # main.c is up to date and silently reuses the previous mode's object (the .hex then still
  # contains the OTHER mode). This bit the mode-4 build once: the restore-by-copy restored the
  # old timestamp and mode 4 was never compiled.
  [System.IO.File]::SetLastWriteTime($src, [DateTime]::Now)

  $log = Join-Path $cwd ("build_mode{0}.log" -f $md.n)
  if (Test-Path $log) { Remove-Item $log -Force }

  $p = Start-Process -FilePath "D:\Keil_v5\UV4\UV4.exe" `
        -ArgumentList @("-b", $proj, "-j0", "-o", $log) `
        -Wait -PassThru

  $lines = Get-Content $log
  $errs  = @($lines | Select-String -Pattern ": error:")
  $warns = @($lines | Select-String -Pattern ": warning:")
  $size  = @($lines | Select-String -Pattern "Program Size:")
  $summary.Add(("mode {0} [{1}] exit={2} errors={3} warnings={4}" -f $md.n, $md.m, $p.ExitCode, $errs.Count, $warns.Count))
  foreach ($s in $size)  { $summary.Add("    " + $s.Line.Trim()) }
  foreach ($e in $errs)  { $summary.Add("    ERR  " + $e.Line.Trim()) }
  foreach ($w in $warns) { $summary.Add("    WARN " + $w.Line.Trim()) }
}

# Restore by CONTENT (not Copy-Item): the file is written from the exact text that was read at
# the start, so the mode line can never come back as some other mode. Verify it afterwards.
[System.IO.File]::WriteAllText($src, $orig, $enc)
# Same trap as above, from the other side: the restored file must NOT look older than the
# mode-4 object, otherwise the next manual build silently reuses the (already correct) object.
[System.IO.File]::SetLastWriteTime($src, [DateTime]::Now)
$summary
"restored main.c bytes: " + (Get-Item $src).Length
$nowMode = [regex]::Match([System.IO.File]::ReadAllText($src, $enc), $modeRe).Value
"mode line now: $nowMode"
if ($nowMode -ne $origMode) { "ERROR: restore mismatch! expected '$origMode' got '$nowMode'" }
$axf = Join-Path $cwd "MDK-ARM\Objects\Interrupt.axf"
$s = [Text.Encoding]::ASCII.GetString([IO.File]::ReadAllBytes($axf))
"artifact check: mode4 banner=" + $s.Contains('mode 4:') + " mode7 banner=" + $s.Contains('mode 7:')
