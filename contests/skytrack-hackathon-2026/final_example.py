"""AImodels = [det-coco-v26n-b-quantized-fp16, det-visdrone-v26n-b-quantized-fp16, det-firesmoke-v26n-b-quantized-fp16]"""

from pathlib import Path
import threading

# The rescue_locations.txt file will be created in the same directory
# as the currently running Python file.
RESCUE_LOCATIONS_FILE = Path(__file__).resolve().parent / "rescue_locations.txt"
# -> /app/local_planner/lib/python3.12/site-packages/local_planner/examples/rescue_locations.txt

# Prevent multiple threads from writing to the file at the same time.
_rescue_file_lock = threading.Lock()


def save_rescue_location(index, x, y, z):
    """
    Save a rescue location to rescue_locations.txt.

    Each line uses the following format:
        index x y z

    If the given index already exists, the existing line will be updated.
    If the index does not exist, a new line will be appended.

    Example:
        save_rescue_location(0, 10.5, 20.3, 15.0)
    """

    new_line = f"{index} {x} {y} {z}\n"

    with _rescue_file_lock:

        # If the file does not exist, create it and write the first location.
        if not RESCUE_LOCATIONS_FILE.exists():
            with RESCUE_LOCATIONS_FILE.open("w") as file:
                file.write(new_line)
            return

        # Read all existing rescue locations.
        with RESCUE_LOCATIONS_FILE.open("r") as file:
            lines = file.readlines()

        # Check whether the given index already exists.
        for i, line in enumerate(lines):
            parts = line.strip().split()

            if parts and parts[0] == str(index):
                # The index already exists.
                # Replace the existing location with the new coordinates.
                lines[i] = new_line

                with RESCUE_LOCATIONS_FILE.open("w") as file:
                    file.writelines(lines)

                return

        # The index does not exist.
        # Append the new rescue location to the end of the file.
        with RESCUE_LOCATIONS_FILE.open("a") as file:
            file.write(new_line)


# ══════════════════════════════════════════════════════════════════════
# Final example: write the positions and rescue priority of detected
# people to rescue_locations.txt.
#
# One line per person: "index x y z"
#   index    rescue priority, 0 = rescue first
#   x y z    position of the person
#
# The locations below are dummy data: replace them with what your AI
# model finds. Calling save_rescue_location again with the same index
# updates that line instead of adding a new one. Write before land().
#
# Where the file goes: next to this Python file (RESCUE_LOCATIONS_FILE).
# Run from /tmp/sae in the drone container, it is
# /tmp/sae/rescue_locations.txt; copy it out with
#     docker cp skytrack-simulation-skytrack-autonomy-1:/tmp/sae/rescue_locations.txt .
#
# Run::
#
#     python -m local_planner.examples.final_example
# ══════════════════════════════════════════════════════════════════════
from typing import Any, Iterator

from local_planner import boot_drone, brake, fly_to, land, takeoff

ALT_M = 5.0

# Dummy rescue locations: (index, x, y, z).
DUMMY_LOCATIONS = [
    (0, 10.5, 20.3, 15.0),
    (1, 30.0, 40.0, 25.5),
    (2, 15.2, 12.7, 30.0),
]


def final_mission(ctx: Any) -> Iterator[Any]:
    """Take off, write the dummy rescue locations, land."""
    yield takeoff(alt_m=ALT_M)
    yield fly_to(north=5.0, east=0.0, alt_m=ALT_M, name="search")

    # A first, rough sighting of person 0 ...
    save_rescue_location(0, 9.8, 19.6, 15.0)

    # ... then every person. Index 0 again: its line is updated, not duplicated.
    for index, x, y, z in DUMMY_LOCATIONS:
        save_rescue_location(index, x, y, z)

    ctx.world.log_info(f"[RESCUE] saved to {RESCUE_LOCATIONS_FILE}")
    yield fly_to(north=0.0, east=0.0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


final_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.fly(final_mission)
        drone.run()


if __name__ == "__main__":
    main()
