#!/bin/sh
# SPDX-License-Identifier: BSD-3-Clause
#
# Run a STM32CubeMX command script without a desktop.
#
# Usage: headless-cubemx.sh <script>
#
# The script is passed to STM32CubeMX as it stands, so it carries its own
# "config load", "project path" and "exit".
#
# STM32CubeMX ships inside STM32CubeIDE as an Eclipse plugin and has no
# headless mode of its own: it builds a Swing frame even for -h. It therefore
# runs against an Xvfb display started here rather than by xvfb-run, so that a
# run wedged on a modal dialog can still be photographed before it is killed.
#
# Two further behaviours of its command line are worked around:
#
#   - the firmware package is read from a repository below the JVM's user.home,
#     which is the running user's entry in the password file rather than $HOME.
#     Without the package STM32CubeMX asks for my.st.com credentials instead of
#     generating, so the packages unpacked in the image are linked in there.
#     STM32CubeMX writes its log and its pack cache below user.home too, which
#     is why an unwritable one is refused here rather than left to fail as a
#     dialog no one can answer.
#   - it exits 0 even when it refuses a command, answering KO on the line after
#     the command, so the transcript is what decides whether the run worked.
#
# A caller that generates code has one more to deal with: script mode has no
# project folder. The .ioc stores none, the GUI supplies the directory it
# opened the file from, and the command line resolves it to "/". Running as
# root that write succeeds and STM32CubeMX falls back to the .ioc's own
# directory; as any other user it fails and STM32CubeMX blocks on an error
# dialog nothing can answer, until the timeout below. "project path" names the
# parent directory, to which STM32CubeMX appends the project name.
#
# CUBEMX_TIMEOUT overrides the time a run may take, CUBEMX_SCREENSHOT where the
# screen is captured to when it runs out. That capture lands in the working
# directory by default, so a caller comparing its tree against a commit trips
# over it as well as over the failed exit status. CUBEMX_REPOSITORY overrides
# where the firmware packages are read from.

set -eu

die() {
	echo "headless-cubemx.sh: $*" >&2
	exit 1
}

[ $# -eq 1 ] || die "usage: $0 <script>"

SCRIPT=$1

TIMEOUT=${CUBEMX_TIMEOUT:-180}
SCREENSHOT=${CUBEMX_SCREENSHOT:-$PWD/timeout.png}
IDE=${STM32CUBEIDE:-/opt/stm32cubeide}
REPOSITORY=${CUBEMX_REPOSITORY:-/opt/cubemx-repo}

[ -r "$SCRIPT" ] || die "$SCRIPT is missing"
[ -d "$REPOSITORY" ] || die "$REPOSITORY holds no firmware package"

# both are versioned directories, so they are matched rather than named
JAVA=$(echo "$IDE"/plugins/com.st.stm32cube.ide.jre.linux64_*/jre/bin/java)
JAR=$(echo "$IDE"/plugins/com.st.stm32cube.common.mx_*/STM32CubeMX.jar)
[ -x "$JAVA" ] || die "no bundled JRE under $IDE"
[ -r "$JAR" ] || die "no STM32CubeMX under $IDE"

LOG=$(mktemp)
XVFB=
cleanup() {
	[ -z "$XVFB" ] || kill "$XVFB" 2>/dev/null || :
	rm -f "$LOG"
}
trap cleanup EXIT

# ask the JVM rather than reading $HOME, which it does not follow
MXHOME=$("$JAVA" -XshowSettings:properties -version 2>&1 | sed -n 's/^ *user.home = //p')
[ -n "$MXHOME" ] || die "the JVM reports no user.home"
[ -w "$MXHOME" ] || die "$MXHOME is not writable, and STM32CubeMX keeps its state there"

REPOSITORY_LINK=$MXHOME/STM32Cube/Repository
mkdir -p "$MXHOME/STM32Cube"
if [ ! -e "$REPOSITORY_LINK" ] && [ ! -L "$REPOSITORY_LINK" ]; then
	ln -s "$REPOSITORY" "$REPOSITORY_LINK"
elif [ "$(readlink -f "$REPOSITORY_LINK" 2>/dev/null)" != "$(readlink -f "$REPOSITORY")" ]; then
	# a home carried in from somewhere brings its own repository, which is
	# left alone: whether it holds the packages this project needs is then
	# its owner's business, and STM32CubeMX says so if it does not
	echo "headless-cubemx.sh: $REPOSITORY_LINK exists, using it instead of $REPOSITORY" >&2
fi

DISPLAY=:99
while [ -e "/tmp/.X11-unix/X${DISPLAY#:}" ]; do
	DISPLAY=:$((${DISPLAY#:} + 1))
done
export DISPLAY

Xvfb "$DISPLAY" -screen 0 1280x1024x24 >/dev/null 2>&1 &
XVFB=$!

WAITED=0
while [ ! -e "/tmp/.X11-unix/X${DISPLAY#:}" ]; do
	WAITED=$((WAITED + 1))
	[ "$WAITED" -lt 10 ] || die "Xvfb did not come up on $DISPLAY"
	sleep 1
done

"$JAVA" -jar "$JAR" -q "$SCRIPT" >"$LOG" 2>&1 &
CUBEMX=$!

WAITED=0
while kill -0 "$CUBEMX" 2>/dev/null; do
	if [ "$WAITED" -ge "$TIMEOUT" ]; then
		# a run this long is wedged on a dialog, and the dialog is the
		# only place its text exists
		import -window root "$SCREENSHOT" 2>/dev/null ||
			echo "headless-cubemx.sh: could not capture $SCREENSHOT" >&2
		kill "$CUBEMX" 2>/dev/null || :
		cat "$LOG"
		die "STM32CubeMX did not finish within ${TIMEOUT}s, screen captured in $SCREENSHOT"
	fi
	WAITED=$((WAITED + 1))
	sleep 1
done

cat "$LOG"

! grep -qx KO "$LOG" || die "STM32CubeMX refused a command, see the KO above"
