import json
import os
import argparse
import argparse


def assert_key_exists(key: str, data_dict: dict):
    assert key in data_dict, "{} is required".format(key)


def validate_map_route(route_list: list):
    assert isinstance(route_list, list)

    for waypoint_dict in route_list:
        assert isinstance(waypoint_dict, dict)
        assert_key_exists("name", waypoint_dict)
        assert_key_exists("description", waypoint_dict)
        assert_key_exists("pose", waypoint_dict)

        assert isinstance(waypoint_dict["name"], str)
        assert isinstance(waypoint_dict["description"], str)
        assert isinstance(waypoint_dict["pose"], list)

        assert (
            len(waypoint_dict["pose"]) == 7
        ), "Waypoint {} pose should have 7 values".format(waypoint_dict["name"])


def validate_map(map_dict: dict):
    # The next_map should be a str or None (if last map)
    assert map_dict["next_map"] is None or isinstance(map_dict["next_map"], str)
    assert_key_exists("description", map_dict)
    assert_key_exists("route", map_dict)
    validate_map_route(map_dict["route"])


class Route:
    """A class to handle interacting with Routes"""

    def __init__(route_json_path: str):
        self.maps = {}
        self.start_map = None
        self.current_map = None
        self.load_route(route_json_path)

    def load_route(route_json_path: str):
        Route.validate_route_json(route_json_path)

        # Load the start key
        START_KEY = "start_map"
        self.start_map = route_data["start_map"]

        # Load the maps
        MAP_KEY = "maps"
        self.maps = None

    @classmethod
    def validate_route_json(cls, route_json_path: str):
        assert os.path.isfile(route_json_path), "Route JSON not found: {}".format(
            route_json_path
        )

        with open(route_json_path) as f:
            route_data = json.load(f)

        assert_key_exists("start_map", route_data)
        assert_key_exists("maps", route_data)

        assert isinstance(
            route_data["maps"], dict
        ), "The maps key should contain a dictionary of maps"

        for map_key, map_dict in route_data["maps"].items():
            validate_map(map_dict)


def main(route_json_path: str, validate: bool):
    if validate:
        Route.validate_route_json(route_json_path)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--route", type=str, help="The path to the route JSON")
    ap.add_argument(
        "-v",
        "--validate",
        action="store_true",
        help="Only validate the route JSON (default to False)",
    )
    args = vars(ap.parse_args())
    main(args["route"], args["validate"])