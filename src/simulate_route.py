import time
import argparse
from route_loader import Route


def main(route_json_path):
    route = Route(route_json_path, dry_run=True)

    # Loop through the positions in our list one at a time
    while not route.is_complete():
        next_pose = route.get_next_waypoint()
        print(next_pose)

        # Wait for a second to make sure the ARK has time to accept the goal
        time.sleep(0.1)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-r", "--route", type=str, required=True, help="The path to the route JSON"
    )
    args = vars(ap.parse_args())
    main(args["route"])
