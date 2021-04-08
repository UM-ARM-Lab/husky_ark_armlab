import time
import argparse
from route_loader import Route
from keyboard_input import KeyboardInput


def pause(keyboard_input):
    while True:
        time.sleep(1)

        if keyboard_input.check_hit() and " " == keyboard_input.get_char():
            print("Resuming")
            break


def main(route_json_path):
    route = Route(route_json_path, dry_run=True)

    keyboard_input = KeyboardInput()

    # Loop through the positions in our list one at a time
    while not route.is_complete():
        next_pose = route.get_next_waypoint()
        print(next_pose)

        if keyboard_input.check_hit():

            char = keyboard_input.get_char()

            # Skip the waypoint if n is pressed (ord == 110)
            if "n" == char:
                print("Skipping waypoint: ", next_pose.name)
                continue

            # If the space bar is pressed, just wait until
            if " " == char:
                print("Pausing")
                pause(keyboard_input)

        # Wait for a second to make sure the ARK has time to accept the goal
        time.sleep(1.0)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-r", "--route", type=str, required=True, help="The path to the route JSON"
    )
    args = vars(ap.parse_args())
    main(args["route"])
