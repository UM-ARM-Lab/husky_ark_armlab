import argparse
from route_loader import Route
import json


def main(route_json_path, output_json_path):
    assert(route_json_path is not output_json_path), "Input path == output path. Exiting."

    # Make sure the route is valid
    route_data = Route.validate_route_json(route_json_path)

    maps = []
    map_names = []
    next_map_key = route_data["start_map"]

    while next_map_key is not None:
        map_names.append(next_map_key)
        maps.append(route_data["maps"][next_map_key])
        next_map_key = route_data["maps"][next_map_key]["next_map"]

    # The new start map is the final map
    route_data["start_map"] = map_names[-1]

    for index, single_map in enumerate(maps):
        # The next map should be the previous map
        # The first map now has a next map of None
        single_map["next_map"] = None if index == 0 else map_names[index - 1]

        # Reverse the order of the waypoints
        single_map["route"].reverse()

        # Reverse the euler angle
        for waypoint in single_map["route"]:
            waypoint["euler_angle"] = (waypoint["euler_angle"] + 180) % 360

    with open(output_json_path, 'w') as f:
        json.dump(route_data, f, indent=2)

    # Make sure the reversed route is valid
    Route.validate_route_json(output_json_path)

    print("Reversed route validated and saved to {}".format(output_json_path))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-r", "--route", type=str, required=True, help="The path to the route JSON"
    )
    ap.add_argument(
        "-o", "--output", type=str, required=True, help="The path to store the output JSON"
    )
    args = vars(ap.parse_args())
    main(args["route"], args["output"])
