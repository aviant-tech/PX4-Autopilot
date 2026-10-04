#!/usr/bin/env bash

set -euo pipefail

abs_dir() {
	(cd "$1" && pwd -P)
}

px4_dir=$(abs_dir "$1")
action=${2:-}
vscode_dir="$px4_dir/.vscode"
model_file="$vscode_dir/sitl_gz_external.model"
gdbinit="$vscode_dir/sitl_gz_external.gdbinit"
lldbinit="$vscode_dir/sitl_gz_external.lldbinit"

fail() {
	echo "ERROR [gz external model] $*" >&2
	exit 1
}

pick_folder() {
	local title="Gazebo model folder (with model.sdf)"
	if command -v zenity > /dev/null; then
		zenity --file-selection --directory --title="$title"
	elif command -v kdialog > /dev/null; then
		kdialog --getexistingdirectory "$HOME" --title "$title"
	elif [ "$(uname)" = "Darwin" ]; then
		osascript -e "POSIX path of (choose folder with prompt \"$title\")"
	else
		echo "ERROR [gz external model] no folder dialog, install zenity or write the model folder to $model_file" >&2
		return 2
	fi
}

mkdir -p "$vscode_dir"

if [ "$action" = "pick" ] || [ ! -s "$model_file" ]; then
	model_dir=$(pick_folder) || { [ $? -eq 2 ] && exit 1; fail "no folder selected"; }
	echo "${model_dir%/}" > "$model_file"
fi

model_dir=$(head -n 1 "$model_file")
[ -f "$model_dir/model.sdf" ] || fail "$model_dir has no model.sdf, run the task \"gz external model: pick\""
model_dir=$(abs_dir "$model_dir")

model=$(basename "$model_dir")
models_dir=$(dirname "$model_dir")
sim_dir=$(dirname "$models_dir")
root=$(git -C "$model_dir" rev-parse --show-toplevel 2> /dev/null || echo "$sim_dir")

airframe=$(find "$root" -name .git -prune -o -type f -name "[0-9]*_gz_${model}" -print | sort | head -n 1)
[ -n "$airframe" ] || fail "no airframe file <autostart>_gz_${model} in $root"
airframe_name=$(basename "$airframe")
exclude=$(git -C "$px4_dir" rev-parse --git-path info/exclude)
case "$exclude" in
	/*) ;;
	*) exclude="$px4_dir/$exclude" ;;
esac

for file in "$airframe" "$airframe.post"; do
	[ -f "$file" ] || continue
	link="ROMFS/px4fmu_common/init.d-posix/airframes/$(basename "$file")"

	if [ ! -e "$px4_dir/$link" ] || [ -L "$px4_dir/$link" ]; then
		ln -sfn "$file" "$px4_dir/$link"
		mkdir -p "$(dirname "$exclude")"
		grep -qxF "/$link" "$exclude" 2> /dev/null || echo "/$link" >> "$exclude"
	fi
done

plugin_build_dir="$px4_dir/build/gz_external/$model"

if [ -f "$sim_dir/plugins/CMakeLists.txt" ]; then
	generator=()
	command -v ninja > /dev/null && generator=(-G Ninja)
	cmake -S "$sim_dir/plugins" -B "$plugin_build_dir" ${generator[@]+"${generator[@]}"} -DCMAKE_BUILD_TYPE=RelWithDebInfo > /dev/null
	cmake --build "$plugin_build_dir"
fi

search_dirs=()
[ -d "$plugin_build_dir" ] && search_dirs+=("$plugin_build_dir")
search_dirs+=("$root")
IFS=: read -r -a system_plugin_dirs <<< "${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
for dir in ${system_plugin_dirs[@]+"${system_plugin_dirs[@]}"}; do
	[ -d "$dir" ] && search_dirs+=("$dir")
done

plugin_path=""
missing=""

while read -r plugin; do
	case "$plugin" in
		gz-sim-* | libgz-sim-*) continue ;;
	esac

	lib="${plugin#lib}"
	lib="lib${lib%.so}"
	lib="${lib%.dylib}"
	found=$(find "${search_dirs[@]}" -name .git -prune -o -type f \( -name "$lib.so" -o -name "$lib.dylib" \) -print 2> /dev/null | head -n 1 || true)

	if [ -z "$found" ]; then
		missing="$missing $plugin"
		continue
	fi

	case ":$plugin_path:" in
		*":$(dirname "$found"):"*) ;;
		*) plugin_path="${plugin_path:+$plugin_path:}$(dirname "$found")" ;;
	esac
done < <(sed -n 's/.*filename="\([^"]*\)".*/\1/p' "$model_dir/model.sdf" | sort -u)

[ -z "$missing" ] || echo "WARN  [gz external model] plugins not found, Gazebo looks for them in its own paths:$missing" >&2

env_vars=(
	"PX4_SIM_MODEL=gz_${model}"
	"PX4_SYS_AUTOSTART=${airframe_name%%_*}"
	"GZ_SIM_RESOURCE_PATH=$models_dir${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
	"GZ_IP=${GZ_IP:-127.0.0.1}"
)

if [ -d "$sim_dir/worlds" ]; then
	env_vars+=("PX4_GZ_WORLDS=$sim_dir/worlds")
fi

if [ -n "$plugin_path" ]; then
	env_vars+=("GZ_SIM_SYSTEM_PLUGIN_PATH=$plugin_path")
fi

: > "$gdbinit"
: > "$lldbinit"

for var in "${env_vars[@]}"; do
	echo "set environment $var" >> "$gdbinit"
	echo "settings append target.env-vars \"$var\"" >> "$lldbinit"
done

echo "INFO  [gz external model] $model, airframe $airframe_name"
printf '  %s\n' "${env_vars[@]}"
