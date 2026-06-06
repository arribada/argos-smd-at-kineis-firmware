#!/usr/bin/env bash
# gen_html_report.sh <test_binaries...> <output.html>
# Runs each test binary, parses the framework output ("PASS"/"FAIL"/"tests passed"),
# and emits a single self-contained HTML page suitable for browser viewing.
#
# Test framework convention (Tests/unit/test_framework.h):
#   - Each test line: "  [TEST] <name>... PASS" or "... FAIL: <reason>"
#   - Suite trailer: "<P>/<T> tests passed (<F> failed)"
set -euo pipefail

# Last arg is the output path; the rest are test binaries.
args=("$@")
out="${args[-1]}"
unset 'args[-1]'

ts=$(date '+%Y-%m-%d %H:%M:%S')
total_pass=0
total_fail=0
total_run=0

# Per-suite blocks accumulated into one body
body=""

for bin in "${args[@]}"; do
	suite=$(basename "$bin")
	# Capture combined stdout+stderr
	if ! out_raw=$("$bin" 2>&1); then
		status="FAIL"
	else
		status="PASS"
	fi
	# Per-suite counters from trailer "P/T tests passed (F failed)"
	trailer=$(echo "$out_raw" | grep -E '^[0-9]+/[0-9]+ tests passed' | tail -1 || true)
	if [[ -n "$trailer" ]]; then
		p=$(echo "$trailer" | sed -E 's#^([0-9]+)/.*#\1#')
		t=$(echo "$trailer" | sed -E 's#^[0-9]+/([0-9]+).*#\1#')
		f=$(echo "$trailer" | sed -E 's#.*\(([0-9]+) failed\).*#\1#')
	else
		p=0 ; t=0 ; f=0
	fi
	total_pass=$((total_pass + p))
	total_fail=$((total_fail + f))
	total_run=$((total_run + t))

	if [[ "$status" == "PASS" && "$f" == "0" ]]; then
		color="#2da44e"  # green
		badge="PASS"
	else
		color="#cf222e"  # red
		badge="FAIL"
	fi

	# HTML-escape the raw output for the <pre> block
	esc=$(echo "$out_raw" | sed 's/&/\&amp;/g; s/</\&lt;/g; s/>/\&gt;/g')

	body+=$'\n'"<details><summary><span class=\"badge\" style=\"background:$color\">$badge</span> <code>$suite</code> &mdash; $p/$t passed</summary><pre>$esc</pre></details>"
done

# Overall colour
if [[ "$total_fail" == "0" ]]; then
	overall="#2da44e"
	overall_text="ALL GREEN"
else
	overall="#cf222e"
	overall_text="$total_fail FAILED"
fi

cat > "$out" <<EOF
<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<title>Unit test report — argos-smd-at-kineis-firmware</title>
<style>
 body{font-family:-apple-system,Segoe UI,sans-serif;max-width:900px;margin:2em auto;padding:0 1em;color:#1f2328;}
 h1{margin:0 0 .2em 0;}
 .summary{padding:1em;border-radius:6px;color:#fff;background:$overall;margin:1em 0;}
 .summary b{font-size:1.2em;}
 .badge{display:inline-block;padding:2px 8px;border-radius:4px;color:#fff;font-size:.85em;font-weight:600;}
 details{border:1px solid #d0d7de;border-radius:6px;padding:.5em 1em;margin:.5em 0;}
 summary{cursor:pointer;font-weight:500;}
 pre{background:#f6f8fa;padding:1em;border-radius:6px;overflow:auto;font-size:.85em;line-height:1.4;}
 code{background:#eef0f3;padding:1px 6px;border-radius:3px;}
 footer{color:#656d76;font-size:.85em;margin-top:2em;}
</style></head><body>
<h1>Unit test report</h1>
<div class="summary">
  <b>$overall_text</b><br>
  Total: $total_pass / $total_run tests passed across $(echo "${args[@]}" | wc -w) suites.
</div>
$body
<footer>Generated $ts &middot; argos-smd-at-kineis-firmware &middot; <code>make report</code></footer>
</body></html>
EOF
echo "Wrote $out ($(wc -c < "$out") bytes)"
